/**
 * @file WalterBlueCherry.cpp
 * @author Daan Pape <daan@dptechnics.com>
 * @author Arnoud Devoogdt <arnoud@dptechnics.com>
 * @date 21 September 2026
 * @version 1.5.1
 * @copyright DPTechnics bv <info@dptechnics.com>
 * @brief Walter Modem library
 *
 * @section LICENSE
 *
 * Copyright (C) 2026, DPTechnics bv
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright notice, this list of
 *      conditions and the following disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above copyright notice, this list of
 *      conditions and the following disclaimer in the documentation and/or other materials provided
 *      with the distribution.
 *
 *   3. Neither the name of DPTechnics bv nor the names of its contributors may be used to endorse
 *      or promote products derived from this software without specific prior written permission.
 *
 *   4. This software, with or without modification, must only be used with a Walter board from
 *      DPTechnics bv.
 *
 *   5. Any software provided in binary form under this license must not be reverse engineered,
 *      decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY DPTECHNICS BV “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, NONINFRINGEMENT, AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL DPTECHNICS BV OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @section DESCRIPTION
 *
 * This file contains the implementation of the BlueCherry cloud protocol in the Walter Modem
 * library. BlueCherry cloud uses CoAP + DTLS to communicate with the cloud: the CoAP messages are
 * composed and parsed here, while the modem provides the DTLS secured UDP socket underneath.
 *
 * The same transport carries Zero-Touch Provisioning (ZTP), which fetches a device ID and a
 * signed device certificate from the BlueCherry ZTP server so that Walter can provision itself
 * without manufacturing-time credentials. blueCherrySync triggers this automatically when Walter
 * is not provisioned yet and a device type id was passed to blueCherryInit.
 */

#include <WalterDefines.h>
#include <esp_log.h>

#if CONFIG_WALTER_MODEM_ENABLE_BLUECHERRY

#include <bootloader_random.h>
#include <esp_mac.h>
#include <esp_random.h>
#include <mbedtls/ctr_drbg.h>
#include <mbedtls/entropy.h>
#include <mbedtls/pem.h>
#include <mbedtls/pk.h>
#include <mbedtls/x509_crt.h>
#include <mbedtls/x509_csr.h>

#pragma region PRIVATE_CONSTANTS

/**
 * @brief The first header byte of every message BlueCherry transmits: CoAP version 1, message
 * type confirmable, token length zero.
 */
static constexpr uint8_t _BC_COAP_VER_TYPE_CON = 0x40;

/**
 * @brief The CoAP payload marker, separating the options from the payload.
 */
static constexpr uint8_t _BC_COAP_PAYLOAD_MARKER = 0xFF;

/**
 * @brief The CoAP method code for a GET request, used by the ZTP requests.
 */
static constexpr uint8_t _BC_COAP_CODE_GET = 0x01;

/**
 * @brief The CoAP option number of the Uri-Path option.
 */
static constexpr uint8_t _BC_COAP_OPT_URI_PATH = 11;

/**
 * @brief The maximum number of times a confirmable message is transmitted before giving up,
 * as defined by RFC 7252.
 */
static constexpr uint8_t _BC_COAP_MAX_RETRANSMIT = 4;

/**
 * @brief The initial number of seconds to wait for an acknowledgement, as defined by RFC 7252.
 */
static constexpr double _BC_COAP_ACK_TIMEOUT = 2.0;

/**
 * @brief The randomisation factor applied to the initial acknowledgement timeout, as defined by
 * RFC 7252.
 */
static constexpr double _BC_COAP_ACK_RANDOM_FACTOR = 1.5;

/**
 * @brief The BlueCherry root + intermediate CA certificate, used both to validate the ZTP
 * server and, once provisioned, the main BlueCherry cloud connection (slot 6).
 */
static constexpr const char* _blueCherryCaCert = "-----BEGIN CERTIFICATE-----\r\n\
MIIBlTCCATqgAwIBAgICEAAwCgYIKoZIzj0EAwMwGjELMAkGA1UEBhMCQkUxCzAJ\r\n\
BgNVBAMMAmNhMB4XDTI0MDMyNDEzMzM1NFoXDTQ0MDQwODEzMzM1NFowJDELMAkG\r\n\
A1UEBhMCQkUxFTATBgNVBAMMDGludGVybWVkaWF0ZTBZMBMGByqGSM49AgEGCCqG\r\n\
SM49AwEHA0IABJGFt28UrHlbPZEjzf4CbkvRaIjxDRGoeHIy5ynfbOHJ5xgBl4XX\r\n\
hp/r8zOBLqSbu6iXGwgjp+wZJe1GCDi6D1KjZjBkMB0GA1UdDgQWBBR/rtuEomoy\r\n\
49ovMAnj5Hpmk2gTGjAfBgNVHSMEGDAWgBR3Vw0Y1sUvMhkX7xySsX55tvsu8TAS\r\n\
BgNVHRMBAf8ECDAGAQH/AgEAMA4GA1UdDwEB/wQEAwIBhjAKBggqhkjOPQQDAwNJ\r\n\
ADBGAiEApN7DmuufC/aqyt6g2Y8qOWg6AXFUyTcub8/Y28XY3KgCIQCs2VUXCPwn\r\n\
k8jR22wsqNvZfbndpHthtnPqI5+yFXrY4A==\r\n\
-----END CERTIFICATE-----\r\n\
-----BEGIN CERTIFICATE-----\r\n\
MIIBmDCCAT+gAwIBAgIUDjfXeosg0fphnshZoXgQez0vO5UwCgYIKoZIzj0EAwMw\r\n\
GjELMAkGA1UEBhMCQkUxCzAJBgNVBAMMAmNhMB4XDTI0MDMyMzE3MzU1MloXDTQ0\r\n\
MDQwNzE3MzU1MlowGjELMAkGA1UEBhMCQkUxCzAJBgNVBAMMAmNhMFkwEwYHKoZI\r\n\
zj0CAQYIKoZIzj0DAQcDQgAEB00rHNthOOYyKj80cd/DHQRBGSbJmIRW7rZBNA6g\r\n\
fbEUrY9NbuhGS6zKo3K59zYc5R1U4oBM3bj6Q7LJfTu7JqNjMGEwHQYDVR0OBBYE\r\n\
FHdXDRjWxS8yGRfvHJKxfnm2+y7xMB8GA1UdIwQYMBaAFHdXDRjWxS8yGRfvHJKx\r\n\
fnm2+y7xMA8GA1UdEwEB/wQFMAMBAf8wDgYDVR0PAQH/BAQDAgGGMAoGCCqGSM49\r\n\
BAMDA0cAMEQCID7AcgACnXWzZDLYEainxVDxEJTUJFBhcItO77gcHPZUAiAu/ZMO\r\n\
VYg4UI2D74WfVxn+NyVd2/aXTvSBp8VgyV3odA==\r\n\
-----END CERTIFICATE-----\r\n";

#pragma endregion // PRIVATE_CONSTANTS

#pragma region PRIVATE_STATE

/**
 * @brief A parsed CoAP message header.
 */
typedef struct {
  /**
   * @brief The CoAP message type, one of WalterModemBlueCherryCoapSendType.
   */
  uint8_t type;

  /**
   * @brief The CoAP request/response code. BlueCherry cloud messages carry the number of
   * unacknowledged messages here instead.
   */
  uint8_t code;

  /**
   * @brief The CoAP message id.
   */
  uint16_t msgId;

  /**
   * @brief Whether the message carried a payload marker.
   */
  bool hasPayloadMarker;

  /**
   * @brief The offset of the first payload byte, equal to the message length when the message
   * carries no payload.
   */
  uint16_t payloadOffset;

  /**
   * @brief The number of payload bytes.
   */
  uint16_t payloadLen;
} _WalterModemBlueCherryCoapMsg;

/**
 * @brief A single BlueCherry ZTP device identification parameter, used to prove device identity
 * to the ZTP server when requesting a device ID.
 */
typedef struct {
  /**
   * @brief The type of device identifier.
   */
  WalterModemBlueCherryZtpDeviceIdParamType type;

  union {
    /**
     * @brief A MAC address used for authentication.
     */
    uint8_t mac[6];

    /**
     * @brief An IMEI number in ASCII format + 0-terminator.
     */
    char imei[16];

    /**
     * @brief A 64-bit OOB challenge.
     */
    uint64_t oobChallenge;
  } value;
} _WalterModemBlueCherryZtpParam;

/**
 * @brief The parsed header of the last accepted incoming CoAP message, which sits in
 * _blueCherry.messageIn.
 */
static _WalterModemBlueCherryCoapMsg _blueCherryCoapRx = {};

/**
 * @brief The device identification parameters to send to the ZTP server.
 */
static _WalterModemBlueCherryZtpParam
    _ztpParams[WALTER_MODEM_BLUECHERRY_ZTP_MAX_DEVICE_ID_PARAMS] = {};

/**
 * @brief The number of parameters currently in the _ztpParams array.
 */
static uint8_t _ztpParamCount = 0;

/**
 * @brief The BlueCherry device ID received from the ZTP server.
 */
static char _ztpDeviceId[WALTER_MODEM_BLUECHERRY_ZTP_ID_LEN + 1] = {};

/**
 * @brief The CSR subject buffer ("C=BE,CN=<deviceTypeId>.<deviceId>").
 */
static char _ztpSubject[32] = {};

/**
 * @brief The generated device private key in PEM format.
 */
static char _ztpPrivKeyPem[256] = {};

/**
 * @brief The ZTP-signed device certificate in PEM format.
 */
static char _ztpCertPem[576] = {};

/**
 * @brief The DER-encoded certificate signing request.
 */
static uint8_t _ztpCsr[576] = {};

/**
 * @brief The length of the DER-encoded certificate signing request.
 */
static size_t _ztpCsrLen = 0;

#pragma endregion // PRIVATE_STATE

#pragma region PRIVATE_COAP

/**
 * @brief Write the 4 byte fixed CoAP header of a confirmable message without a token.
 *
 * @return The offset just past the header.
 */
static uint16_t _blueCherryCoapWriteHeader(uint8_t* buf, uint8_t code, uint16_t msgId)
{
  buf[0] = _BC_COAP_VER_TYPE_CON;
  buf[1] = code;
  buf[2] = msgId >> 8;
  buf[3] = msgId & 0xFF;

  return 4;
}

/**
 * @brief Append a single Uri-Path option to a CoAP message.
 *
 * Only the compact single byte option header is emitted, which covers every BlueCherry path
 * since all segments are shorter than 13 bytes and Uri-Path is the only option in use.
 *
 * @param buf The message buffer.
 * @param offset The offset to write the option at.
 * @param cap The capacity of the message buffer.
 * @param prevOptNr The number of the previously written option, updated on success. Zero when no
 * option was written yet.
 * @param segment The path segment.
 *
 * @return The offset just past the option, or 0 on error.
 */
static uint16_t _blueCherryCoapWriteUriPath(uint8_t* buf, uint16_t offset, size_t cap,
                                            uint16_t* prevOptNr, const char* segment)
{
  size_t len = strlen(segment);
  uint16_t delta = _BC_COAP_OPT_URI_PATH - *prevOptNr;

  if(len >= 13 || delta >= 13) {
    ESP_LOGE("WalterModem", "BlueCherry CoAP Uri-Path segment '%s' needs extended encoding",
             segment);
    return 0;
  }

  if(offset + 1 + len > cap) {
    ESP_LOGE("WalterModem", "BlueCherry CoAP message does not fit the Uri-Path option");
    return 0;
  }

  buf[offset++] = (uint8_t) ((delta << 4) | len);
  memcpy(buf + offset, segment, len);

  *prevOptNr = _BC_COAP_OPT_URI_PATH;

  return offset + len;
}

/**
 * @brief Append the payload marker and the payload to a CoAP message.
 *
 * @param buf The message buffer.
 * @param offset The offset to write the marker at.
 * @param cap The capacity of the message buffer.
 * @param payload The payload, may be NULL when there is none.
 * @param len The number of payload bytes.
 *
 * @return The offset just past the payload, or 0 on error.
 */
static uint16_t _blueCherryCoapWritePayload(uint8_t* buf, uint16_t offset, size_t cap,
                                            const uint8_t* payload, size_t len)
{
  if(offset + 1 + len > cap) {
    ESP_LOGE("WalterModem", "BlueCherry CoAP message does not fit the payload");
    return 0;
  }

  buf[offset++] = _BC_COAP_PAYLOAD_MARKER;

  if(payload != NULL && len > 0) {
    memcpy(buf + offset, payload, len);
  }

  return offset + len;
}

/**
 * @brief Parse the header of an incoming CoAP message.
 *
 * Walks past the token and any options up to the payload marker, so that both the option-less
 * BlueCherry cloud messages and the ZTP server responses are located correctly.
 *
 * @param buf The received datagram.
 * @param len The number of bytes received.
 * @param msg The structure receiving the parsed header.
 *
 * @return True when the datagram is a well formed CoAP message, false when malformed.
 */
static bool _blueCherryCoapParse(const uint8_t* buf, uint16_t len,
                                 _WalterModemBlueCherryCoapMsg* msg)
{
  /* an acknowledgement may consist of nothing but the fixed 4 byte header */
  if(len < 4) {
    return false;
  }

  uint8_t header = buf[0];
  if(((header >> 6) & 0x03) != 1) {
    /* not a CoAP version 1 message */
    return false;
  }

  msg->type = (header >> 4) & 0x03;
  msg->code = buf[1];
  msg->msgId = ((uint16_t) buf[2] << 8) | buf[3];

  uint16_t offset = 4;
  uint8_t tokenLen = header & 0x0F;
  if(offset + tokenLen > len) {
    return false;
  }
  offset += tokenLen;

  msg->hasPayloadMarker = false;

  /* walk the options until the payload marker or the end of the message is reached */
  while(offset < len) {
    if(buf[offset] == _BC_COAP_PAYLOAD_MARKER) {
      msg->hasPayloadMarker = true;
      offset++;
      break;
    }

    uint8_t delta = buf[offset] >> 4;
    uint16_t optLen = buf[offset] & 0x0F;
    offset++;

    if(delta == 15 || optLen == 15) {
      /* reserved for the payload marker, which is handled above */
      return false;
    }

    /* the extended option delta precedes the extended option length */
    uint8_t deltaBytes = delta == 13 ? 1 : (delta == 14 ? 2 : 0);
    uint8_t lenBytes = optLen == 13 ? 1 : (optLen == 14 ? 2 : 0);

    if(offset + deltaBytes + lenBytes > len) {
      return false;
    }
    offset += deltaBytes;

    if(lenBytes == 1) {
      optLen = buf[offset] + 13;
    } else if(lenBytes == 2) {
      optLen = (((uint16_t) buf[offset] << 8) | buf[offset + 1]) + 269;
    }
    offset += lenBytes;

    if(offset + optLen > len) {
      return false;
    }
    offset += optLen;
  }

  msg->payloadOffset = offset;
  msg->payloadLen = len - offset;

  return true;
}

#pragma endregion // PRIVATE_COAP

#pragma region PRIVATE_CBOR

/**
 * @brief A minimalist CBOR encoder/decoder context, used only to talk to the BlueCherry ZTP
 * endpoint. Internal to this file, not part of the public WalterModem API.
 */
typedef struct {
  uint8_t* buffer;
  size_t capacity;
  size_t position;
} _WalterModemZtpCbor;

static int _ztpCborInit(_WalterModemZtpCbor* cbor, uint8_t* buffer, size_t capacity)
{
  if(buffer == NULL || capacity == 0) {
    return -1;
  }

  cbor->buffer = buffer;
  cbor->capacity = capacity;
  cbor->position = 0;

  return 0;
}

static size_t _ztpCborSize(const _WalterModemZtpCbor* cbor)
{
  return cbor->position;
}

static int _ztpCborWriteByte(_WalterModemZtpCbor* cbor, uint8_t byte)
{
  if(cbor->position < cbor->capacity) {
    cbor->buffer[cbor->position++] = byte;
    return 0;
  }
  return -1;
}

static int _ztpCborWriteBytes(_WalterModemZtpCbor* cbor, const uint8_t* data, size_t length)
{
  if(cbor->position + length <= cbor->capacity) {
    memcpy(&cbor->buffer[cbor->position], data, length);
    cbor->position += length;
    return 0;
  }
  return -1;
}

static int _ztpCborEncodeTypeAndValue(_WalterModemZtpCbor* cbor, uint8_t majorType, size_t value)
{
  if(value < 24) {
    return _ztpCborWriteByte(cbor, (majorType << 5) | value);
  } else if(value < 256) {
    if(_ztpCborWriteByte(cbor, (majorType << 5) | 0x18) < 0) {
      return -1;
    }
    return _ztpCborWriteByte(cbor, (uint8_t) value);
  } else if(value < 65536) {
    if(_ztpCborWriteByte(cbor, (majorType << 5) | 0x19) < 0) {
      return -1;
    }
    uint8_t bytes[] = { (uint8_t) (value >> 8), (uint8_t) value };
    return _ztpCborWriteBytes(cbor, bytes, 2);
  }
  return -1;
}

static int _ztpCborEncodeBytes(_WalterModemZtpCbor* cbor, const uint8_t* data, size_t length)
{
  if(_ztpCborEncodeTypeAndValue(cbor, 2, length) < 0) {
    return -1;
  }
  return _ztpCborWriteBytes(cbor, data, length);
}

static int _ztpCborEncodeString(_WalterModemZtpCbor* cbor, const char* str)
{
  size_t len = strlen(str);
  if(_ztpCborEncodeTypeAndValue(cbor, 3, len) < 0) {
    return -1;
  }
  return _ztpCborWriteBytes(cbor, (const uint8_t*) str, len);
}

static int _ztpCborEncodeUint64(_WalterModemZtpCbor* cbor, uint64_t value)
{
  if(_ztpCborEncodeTypeAndValue(cbor, 2, 8) < 0) {
    return -1;
  }

  uint8_t bytes[] = { (uint8_t) (value >> 56), (uint8_t) (value >> 48), (uint8_t) (value >> 40),
                      (uint8_t) (value >> 32), (uint8_t) (value >> 24), (uint8_t) (value >> 16),
                      (uint8_t) (value >> 8),  (uint8_t) value };

  return _ztpCborWriteBytes(cbor, bytes, 8);
}

static int _ztpCborEncodeInt(_WalterModemZtpCbor* cbor, int value)
{
  if(value >= 0) {
    return _ztpCborEncodeTypeAndValue(cbor, 0, (size_t) value);
  }
  return _ztpCborEncodeTypeAndValue(cbor, 1, (size_t) (-value - 1));
}

static int _ztpCborStartArray(_WalterModemZtpCbor* cbor, size_t size)
{
  return _ztpCborEncodeTypeAndValue(cbor, 4, size);
}

static int _ztpCborStartMap(_WalterModemZtpCbor* cbor, size_t size)
{
  return _ztpCborEncodeTypeAndValue(cbor, 5, size);
}

static int _ztpCborDecodeDeviceId(const uint8_t* cbor_data, size_t cbor_size, char* decoded_str,
                                  size_t decoded_size)
{
  if(cbor_size < 1 || !cbor_data) {
    return -1;
  }

  /* the device id is always encoded as a CBOR text string (major type 3) */
  uint8_t initial_byte = cbor_data[0];
  if((initial_byte >> 5) != 3) {
    return -2;
  }

  uint8_t additional_info = initial_byte & 0x1F;
  if(additional_info > 23) {
    return -3;
  }

  size_t length = additional_info;
  cbor_data++;
  cbor_size--;

  if(length > cbor_size) {
    return -4;
  }

  if(length >= decoded_size) {
    return -5;
  }

  memcpy(decoded_str, cbor_data, length);
  decoded_str[length] = '\0';

  return 0;
}

static int _ztpCborDecodeCertificate(const uint8_t* cbor_data, size_t cbor_size,
                                     unsigned char* decoded_data, size_t decoded_capacity,
                                     size_t* decoded_len)
{
  if(cbor_size < 1 || !cbor_data) {
    return -1;
  }

  /* the signed certificate is always encoded as a CBOR byte string (major type 2) */
  uint8_t initial_byte = cbor_data[0];
  if((initial_byte >> 5) != 2) {
    return -2;
  }

  size_t length = 0;
  size_t offset = 1;
  uint8_t additional_info = initial_byte & 0x1F;

  if(additional_info < 24) {
    length = additional_info;
  } else if(additional_info == 24) {
    length = cbor_data[offset++];
  } else if(additional_info == 25) {
    length = (cbor_data[offset] << 8) | cbor_data[offset + 1];
    offset += 2;
  } else if(additional_info == 26) {
    length = (cbor_data[offset] << 24) | (cbor_data[offset + 1] << 16) |
             (cbor_data[offset + 2] << 8) | cbor_data[offset + 3];
    offset += 4;
  } else {
    return -3;
  }

  if(offset + length > cbor_size) {
    return -4;
  }

  if(length > decoded_capacity) {
    return -5;
  }

  memcpy(decoded_data, cbor_data + offset, length);
  *decoded_len = length;

  return 0;
}

#pragma endregion // PRIVATE_CBOR

#pragma region PRIVATE_METHODS

bool WalterModem::_blueCherryProcessEvent(uint8_t* data, uint8_t len)
{
  switch(data[0]) {
  case WALTER_MODEM_BLUECHERRY_EVENT_TYPE_OTA_INITIALIZE:
    return _processOtaInitializeEvent(data + 1, len - 1);

  case WALTER_MODEM_BLUECHERRY_EVENT_TYPE_OTA_CHUNK:
    return _processOtaChunkEvent(data + 1, len - 1);

  case WALTER_MODEM_BLUECHERRY_EVENT_TYPE_OTA_FINISH:
    return _processOtaFinishEvent();

  case WALTER_MODEM_BLUECHERRY_EVENT_TYPE_MOTA_INITIALIZE:
    return _processMotaInitializeEvent(data + 1, len - 1);

  case WALTER_MODEM_BLUECHERRY_EVENT_TYPE_MOTA_CHUNK:
    return _processMotaChunkEvent(data + 1, len - 1);

  case WALTER_MODEM_BLUECHERRY_EVENT_TYPE_MOTA_FINISH:
    return _processMotaFinishEvent();

  default:
    ESP_LOGD("WalterModem", "Error: invalid BlueCherry event type 0x%x from cloud server", data[0]);
    return true;
  }

  return true;
}

bool WalterModem::_blueCherrySocketConfigure()
{
  if(_blueCherry.bcSocketId == 0) {
    return false;
  }

  bool success = true;

  success &= socketConfig(_blueCherry.bcSocketId);
  success &= socketConfigExtended(_blueCherry.bcSocketId);
  success &= socketConfigSecure(_blueCherry.bcSocketId, true, _blueCherry.tls_profile_id);

  return success;
}

bool WalterModem::_blueCherrySocketConnect(uint16_t port)
{
  if(_blueCherry.bcSocketId == 0) {
    if(WalterModemSocket* sock = _socketReserve(); sock != NULL) {
      _blueCherry.bcSocketId = sock->id;
    } else {
      return false;
    }
  }

  WalterModemSocket* sock = _socketGet(_blueCherry.bcSocketId);

  for(int attempt = 0; attempt < 5; ++attempt) {
    switch(sock->state) {
    case WALTER_MODEM_SOCKET_STATE_FREE:
    case WALTER_MODEM_SOCKET_STATE_RESERVED:
    case WALTER_MODEM_SOCKET_STATE_CLOSED:
      if(!_blueCherrySocketConfigure()) {
        break;
      }
      continue;

    case WALTER_MODEM_SOCKET_STATE_READY:
      if(!socketDial(_blueCherry.bcSocketId, WALTER_MODEM_SOCKET_PROTO_UDP, port,
                     WALTER_MODEM_BLUECHERRY_HOSTNAME)) {
        break;
      }
      continue;

    case WALTER_MODEM_SOCKET_STATE_OPENED:
    case WALTER_MODEM_SOCKET_STATE_PENDING_NO_DATA:
    case WALTER_MODEM_SOCKET_STATE_PENDING_WITH_DATA:
      return true;

    case WALTER_MODEM_SOCKET_STATE_SUSPENDED:
      break;

    default:
      break;
    }
  }

  _blueCherry.bcSocketId = 0;
  return false;
}

void WalterModem::_blueCherrySocketDisconnect()
{
  if(_blueCherry.bcSocketId == 0) {
    return;
  }

  socketClose(_blueCherry.bcSocketId);
  _blueCherry.bcSocketId = 0;
}

void WalterModem::_blueCherrySocketEventHandler(WMSocketEventType event, uint16_t dataReceived,
                                                uint8_t* dataBuffer)
{
  switch(event) {
  case WALTER_MODEM_SOCKET_EVENT_RING:
    /* Only accept a datagram when a transmission is waiting for one. The datagram is validated
     * by _blueCherryCoapTransmit, which owns the retransmission state machine. */
    if(_blueCherry.status != WALTER_MODEM_BLUECHERRY_STATUS_AWAITING_RESPONSE) {
      break;
    }

    if(dataReceived > sizeof(_blueCherry.messageIn)) {
      ESP_LOGE("WalterModem", "Dropping %u byte BlueCherry datagram, exceeds the %u byte buffer",
               (unsigned) dataReceived, (unsigned) sizeof(_blueCherry.messageIn));
      break;
    }

    memcpy(_blueCherry.messageIn, dataBuffer, dataReceived);
    _blueCherry.messageInLen = dataReceived;
    _blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_RESPONSE_READY;
    break;

  case WALTER_MODEM_SOCKET_EVENT_DISCONNECTED:
    _blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_NOT_CONNECTED;
    break;

  default:
    break;
  }
}

void WalterModem::_blueCherrySetCoapHeaders(uint8_t code, uint16_t msgId)
{
  uint16_t offset = _blueCherryCoapWriteHeader(_blueCherry.messageOut, code, msgId);
  _blueCherry.messageOut[offset] = _BC_COAP_PAYLOAD_MARKER;
}

uint16_t WalterModem::_blueCherryCoapNextMessageId()
{
  uint16_t msgId = _blueCherry.curMessageId + 1;

  if(msgId == 0) {
    /* on wrap around, skip msg id 0 which we use as a special/error value */
    msgId = 1;
  }

  return msgId;
}

bool WalterModem::_blueCherryCoapTransmit(uint16_t txLen, uint16_t txMsgId)
{
  double timeout =
      _BC_COAP_ACK_TIMEOUT * (1 + (rand() / (RAND_MAX + 1.0)) * (_BC_COAP_ACK_RANDOM_FACTOR - 1));

  for(uint8_t attempt = 1; attempt <= _BC_COAP_MAX_RETRANSMIT; ++attempt) {
    _blueCherry.lastTransmissionTime = time(NULL);
    _blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_AWAITING_RESPONSE;

    socketSend(_blueCherry.bcSocketId, _blueCherry.messageOut, txLen);

    while(true) {
      if(_blueCherry.status == WALTER_MODEM_BLUECHERRY_STATUS_RESPONSE_READY) {
        if(!_blueCherryCoapParse(_blueCherry.messageIn, _blueCherry.messageInLen,
                                 &_blueCherryCoapRx)) {
          ESP_LOGW("WalterModem", "Ignoring malformed CoAP datagram while awaiting an ACK");
        } else if(_blueCherryCoapRx.type != WALTER_MODEM_BLUECHERRY_COAP_SEND_TYPE_ACK) {
          ESP_LOGW("WalterModem", "Ignoring non-ACK CoAP datagram while awaiting an ACK");
        } else if(_blueCherryCoapRx.msgId != txMsgId) {
          ESP_LOGW("WalterModem", "Ignoring ACK for message id %u while awaiting %u",
                   (unsigned) _blueCherryCoapRx.msgId, (unsigned) txMsgId);
        } else {
          _blueCherry.curMessageId = txMsgId;
          return true;
        }

        /* the datagram was not the acknowledgement we are waiting for, keep waiting */
        _blueCherry.messageInLen = 0;
        _blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_AWAITING_RESPONSE;
      } else if(_blueCherry.status != WALTER_MODEM_BLUECHERRY_STATUS_AWAITING_RESPONSE) {
        return false;
      }

      vTaskDelay(pdMS_TO_TICKS(10));

      if(difftime(time(NULL), _blueCherry.lastTransmissionTime) >= timeout) {
        break;
      }
    }

    timeout *= 2;
  }

  _blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_TIMED_OUT;
  return false;
}

bool WalterModem::_blueCherryCoapSend()
{
  uint16_t txMsgId = _blueCherryCoapNextMessageId();
  uint8_t nrMissed = (uint8_t) (txMsgId - _blueCherry.lastAckedMessageId - 1);

  /* BlueCherry replaces the CoAP request/response code with the number of messages the cloud
   * has not acknowledged yet */
  _blueCherrySetCoapHeaders(nrMissed, txMsgId);

  if(!_blueCherryCoapTransmit(_blueCherry.messageOutLen, txMsgId)) {
    return false;
  }

  if(!_blueCherryCoapRx.hasPayloadMarker) {
    /* The message id was acknowledged but the datagram is unusable, so it is not counted as
     * acknowledged and the cloud is told about it on the next transmission. */
    ESP_LOGE("WalterModem", "Received BlueCherry CoAP message without payload marker");
    return false;
  }

  _blueCherry.lastAckedMessageId = _blueCherryCoapRx.msgId;
  /* BlueCherry cloud ack means our last error line can be cleared */
  _blueCherry.emitErrorEvent = false;

  switch(_blueCherryCoapRx.code) {
  case WALTER_MODEM_BLUECHERRY_COAP_RSP_VALID:
    _blueCherry.moreDataAvailable = false;
    break;

  case WALTER_MODEM_BLUECHERRY_COAP_RSP_CONTINUE:
    _blueCherry.moreDataAvailable = true;
    break;

  default:
    ESP_LOGW("WalterModem", "Unexpected BlueCherry CoAP response code 0x%x",
             (unsigned) _blueCherryCoapRx.code);
    break;
  }

  return true;
}

bool WalterModem::_blueCherryFinishInit()
{
  if(!blueCherryIsProvisioned() ||
     !tlsConfigProfile(_blueCherry.tls_profile_id, WALTER_MODEM_TLS_VALIDATION_URL_AND_CA,
                       WALTER_MODEM_TLS_VERSION_12, 6, 5, 0)) {
    _blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_NOT_PROVISIONED;
    return false;
  }

  _blueCherry.messageOutLen =
      WALTER_MODEM_BLUECHERRY_COAP_HEADER_SIZE; // Reserve space for CoAP headers
  _blueCherry.curMessageId = 0x0000;
  _blueCherry.lastAckedMessageId = 0x0000;
  _blueCherry.moreDataAvailable = false;

  _blueCherry.emitErrorEvent = false;
  _blueCherry.otaSize = 0;

  if(_blueCherrySocketConnect()) {
    _blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_IDLE;
    return true;
  }

  _blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_NOT_CONNECTED;
  return false;
}

#pragma endregion // PRIVATE_METHODS

#pragma region PRIVATE_METHODS_ZTP

/**
 * @brief The entropy function used by Mbed TLS to seed key generation.
 *
 * This function makes use of the true random number generator found in the ESP32-S3 hardware.
 */
static int _blueCherryZtpHardwareRandomEntropy(void* data, unsigned char* output, size_t len)
{
  esp_fill_random(output, len);
  return 0;
}

/**
 * @brief Seed the counter-mode deterministic random bit generator with the hardware RNG.
 */
static bool _blueCherryZtpSeedRandom(mbedtls_ctr_drbg_context* ctrDrbg,
                                     mbedtls_entropy_context* entropy)
{
  bootloader_random_enable();
  int ret =
      mbedtls_ctr_drbg_seed(ctrDrbg, _blueCherryZtpHardwareRandomEntropy, entropy, nullptr, 0);
  bootloader_random_disable();

  return ret == 0;
}

void WalterModem::_blueCherryZtpResetDeviceIdParams()
{
  _ztpParamCount = 0;
}

bool WalterModem::_blueCherryZtpAddDeviceIdParameter(WalterModemBlueCherryZtpDeviceIdParamType type,
                                                     const char* str)
{
  if(str == NULL || type != WALTER_MODEM_BLUECHERRY_ZTP_DEVICE_ID_TYPE_IMEI ||
     _ztpParamCount >= WALTER_MODEM_BLUECHERRY_ZTP_MAX_DEVICE_ID_PARAMS) {
    return false;
  }

  _WalterModemBlueCherryZtpParam* param = &_ztpParams[_ztpParamCount];
  param->type = type;
  _strncpy_s(param->value.imei, str, sizeof(param->value.imei));
  _ztpParamCount++;

  return true;
}

bool WalterModem::_blueCherryZtpAddDeviceIdParameter(WalterModemBlueCherryZtpDeviceIdParamType type,
                                                     const uint8_t* blob)
{
  if(blob == NULL || type != WALTER_MODEM_BLUECHERRY_ZTP_DEVICE_ID_TYPE_MAC ||
     _ztpParamCount >= WALTER_MODEM_BLUECHERRY_ZTP_MAX_DEVICE_ID_PARAMS) {
    return false;
  }

  _WalterModemBlueCherryZtpParam* param = &_ztpParams[_ztpParamCount];
  param->type = type;
  memcpy(param->value.mac, blob, sizeof(param->value.mac));
  _ztpParamCount++;

  return true;
}

bool WalterModem::_blueCherryZtpAddDeviceIdParameter(WalterModemBlueCherryZtpDeviceIdParamType type,
                                                     uint64_t number)
{
  if(type != WALTER_MODEM_BLUECHERRY_ZTP_DEVICE_ID_TYPE_OOB_CHALLENGE ||
     _ztpParamCount >= WALTER_MODEM_BLUECHERRY_ZTP_MAX_DEVICE_ID_PARAMS) {
    return false;
  }

  _WalterModemBlueCherryZtpParam* param = &_ztpParams[_ztpParamCount];
  param->type = type;
  param->value.oobChallenge = number;
  _ztpParamCount++;

  return true;
}

bool WalterModem::_blueCherryZtpTransact(const char* path_segment, const uint8_t* payload,
                                         size_t payload_len, uint8_t* rx_buf, size_t rx_buf_size,
                                         uint16_t* rx_len)
{
  uint16_t txMsgId = _blueCherryCoapNextMessageId();
  uint16_t prevOptNr = 0;

  uint16_t offset =
      _blueCherryCoapWriteHeader(_blueCherry.messageOut, _BC_COAP_CODE_GET, txMsgId);

  offset = _blueCherryCoapWriteUriPath(_blueCherry.messageOut, offset,
                                       sizeof(_blueCherry.messageOut), &prevOptNr,
                                       WALTER_MODEM_BLUECHERRY_ZTP_API_VERSION);
  if(offset == 0) {
    return false;
  }

  offset = _blueCherryCoapWriteUriPath(_blueCherry.messageOut, offset,
                                       sizeof(_blueCherry.messageOut), &prevOptNr, path_segment);
  if(offset == 0) {
    return false;
  }

  offset = _blueCherryCoapWritePayload(_blueCherry.messageOut, offset,
                                       sizeof(_blueCherry.messageOut), payload, payload_len);
  if(offset == 0) {
    return false;
  }

  if(!_blueCherryCoapTransmit(offset, txMsgId)) {
    ESP_LOGE("WalterModem", "No BlueCherry ZTP response on '%s/%s'",
             WALTER_MODEM_BLUECHERRY_ZTP_API_VERSION, path_segment);
    return false;
  }

  if(!_blueCherryCoapRx.hasPayloadMarker || _blueCherryCoapRx.payloadLen == 0) {
    ESP_LOGE("WalterModem", "BlueCherry ZTP response on '%s/%s' carries no payload",
             WALTER_MODEM_BLUECHERRY_ZTP_API_VERSION, path_segment);
    return false;
  }

  if(_blueCherryCoapRx.payloadLen > rx_buf_size) {
    ESP_LOGE("WalterModem", "BlueCherry ZTP response of %u bytes exceeds the %u byte buffer",
             (unsigned) _blueCherryCoapRx.payloadLen, (unsigned) rx_buf_size);
    return false;
  }

  memcpy(rx_buf, _blueCherry.messageIn + _blueCherryCoapRx.payloadOffset,
         _blueCherryCoapRx.payloadLen);
  *rx_len = _blueCherryCoapRx.payloadLen;

  return true;
}

bool WalterModem::_blueCherryZtpRequestDeviceId()
{
  uint8_t cborBuf[256];
  uint8_t coapData[16];
  uint16_t coapDataLen = 0;
  _WalterModemZtpCbor cbor;

  if(_ztpCborInit(&cbor, cborBuf, sizeof(cborBuf)) < 0 || _ztpCborStartArray(&cbor, 2) < 0 ||
     _ztpCborEncodeString(&cbor, _blueCherry.ztpDeviceTypeId) < 0 ||
     _ztpCborStartMap(&cbor, _ztpParamCount) < 0) {
    ESP_LOGE("WalterModem", "Failed to encode BlueCherry ZTP device id request");
    return false;
  }

  for(uint8_t i = 0; i < _ztpParamCount; i++) {
    _WalterModemBlueCherryZtpParam* param = &_ztpParams[i];

    if(_ztpCborEncodeInt(&cbor, (int) param->type) < 0) {
      ESP_LOGE("WalterModem", "Failed to encode BlueCherry ZTP device id parameter type");
      return false;
    }

    switch(param->type) {
    case WALTER_MODEM_BLUECHERRY_ZTP_DEVICE_ID_TYPE_IMEI: {
      uint64_t imei = strtoull(param->value.imei, NULL, 10);
      if(_ztpCborEncodeUint64(&cbor, imei) < 0) {
        ESP_LOGE("WalterModem", "Failed to encode BlueCherry ZTP IMEI");
        return false;
      }
    } break;

    case WALTER_MODEM_BLUECHERRY_ZTP_DEVICE_ID_TYPE_MAC:
      if(_ztpCborEncodeBytes(&cbor, param->value.mac, sizeof(param->value.mac)) < 0) {
        ESP_LOGE("WalterModem", "Failed to encode BlueCherry ZTP MAC address");
        return false;
      }
      break;

    case WALTER_MODEM_BLUECHERRY_ZTP_DEVICE_ID_TYPE_OOB_CHALLENGE:
      if(_ztpCborEncodeUint64(&cbor, param->value.oobChallenge) < 0) {
        ESP_LOGE("WalterModem", "Failed to encode BlueCherry ZTP OOB challenge");
        return false;
      }
      break;

    default:
      break;
    }
  }

  if(!_blueCherryZtpTransact(WALTER_MODEM_BLUECHERRY_ZTP_DEVID_PATH, cborBuf, _ztpCborSize(&cbor),
                             coapData, sizeof(coapData), &coapDataLen)) {
    return false;
  }

  if(_ztpCborDecodeDeviceId(coapData, coapDataLen, _ztpDeviceId, sizeof(_ztpDeviceId)) < 0) {
    ESP_LOGE("WalterModem", "Failed to decode BlueCherry ZTP device id");
    return false;
  }

  return true;
}

bool WalterModem::_blueCherryZtpGenerateKeyAndCsr()
{
  if(_blueCherry.ztpDeviceTypeId == NULL ||
     strlen(_blueCherry.ztpDeviceTypeId) != WALTER_MODEM_BLUECHERRY_ZTP_ID_LEN ||
     strlen(_ztpDeviceId) != WALTER_MODEM_BLUECHERRY_ZTP_ID_LEN) {
    return false;
  }

  mbedtls_pk_context key;
  mbedtls_entropy_context entropy;
  mbedtls_ctr_drbg_context ctrDrbg;
  mbedtls_x509write_csr csr;

  mbedtls_pk_init(&key);
  mbedtls_entropy_init(&entropy);
  mbedtls_ctr_drbg_init(&ctrDrbg);
  mbedtls_x509write_csr_init(&csr);

  bool ok = _blueCherryZtpSeedRandom(&ctrDrbg, &entropy) &&
            mbedtls_pk_setup(&key, mbedtls_pk_info_from_type(MBEDTLS_PK_ECKEY)) == 0 &&
            mbedtls_ecp_gen_key(MBEDTLS_ECP_DP_SECP256R1, mbedtls_pk_ec(key),
                                mbedtls_ctr_drbg_random, &ctrDrbg) == 0 &&
            mbedtls_pk_write_key_pem(&key, (unsigned char*) _ztpPrivKeyPem,
                                     sizeof(_ztpPrivKeyPem)) == 0;

  if(ok) {
    mbedtls_x509write_csr_set_md_alg(&csr, MBEDTLS_MD_SHA256);
    mbedtls_x509write_csr_set_key(&csr, &key);

    snprintf(_ztpSubject, sizeof(_ztpSubject), "C=BE,CN=%s.%s", _blueCherry.ztpDeviceTypeId,
             _ztpDeviceId);

    uint8_t csrBuf[sizeof(_ztpCsr)];
    int ret = -1;
    if(mbedtls_x509write_csr_set_subject_name(&csr, _ztpSubject) == 0) {
      ret = mbedtls_x509write_csr_der(&csr, csrBuf, sizeof(csrBuf), mbedtls_ctr_drbg_random,
                                      &ctrDrbg);
    }

    if(ret < 0) {
      ESP_LOGE("WalterModem", "Failed to write BlueCherry ZTP CSR: -0x%04X", -ret);
      ok = false;
    } else {
      /* mbedtls writes the DER at the end of the buffer */
      size_t offset = sizeof(csrBuf) - ret;
      _ztpCsrLen = ret;
      memcpy(_ztpCsr, csrBuf + offset, _ztpCsrLen);
    }
  }

  mbedtls_pk_free(&key);
  mbedtls_entropy_free(&entropy);
  mbedtls_ctr_drbg_free(&ctrDrbg);
  mbedtls_x509write_csr_free(&csr);

  if(!ok) {
    _ztpPrivKeyPem[0] = '\0';
    _ztpCertPem[0] = '\0';
  }

  return ok;
}

bool WalterModem::_blueCherryZtpRequestSignedCertificate()
{
  uint8_t buf[sizeof(_ztpCertPem)];
  uint8_t coapData[sizeof(_ztpCertPem)];
  uint16_t coapDataLen = 0;
  _WalterModemZtpCbor cbor;

  if(_ztpCborInit(&cbor, buf, sizeof(buf)) < 0 ||
     _ztpCborEncodeBytes(&cbor, _ztpCsr, _ztpCsrLen) < 0) {
    ESP_LOGE("WalterModem", "Failed to encode BlueCherry ZTP CSR");
    return false;
  }

  if(!_blueCherryZtpTransact(WALTER_MODEM_BLUECHERRY_ZTP_CSR_PATH, buf, _ztpCborSize(&cbor),
                             coapData, sizeof(coapData), &coapDataLen)) {
    return false;
  }

  size_t decodedSize = 0;
  if(_ztpCborDecodeCertificate(coapData, coapDataLen, buf, sizeof(buf), &decodedSize) < 0) {
    ESP_LOGE("WalterModem", "Failed to decode BlueCherry ZTP certificate");
    return false;
  }

  /* parsing the DER validates the certificate before it is written to the modem's NVRAM */
  mbedtls_x509_crt crt;
  mbedtls_x509_crt_init(&crt);

  int ret = mbedtls_x509_crt_parse_der(&crt, buf, decodedSize);
  if(ret < 0) {
    ESP_LOGE("WalterModem", "Failed to parse BlueCherry ZTP DER certificate: -0x%04X", -ret);
    mbedtls_x509_crt_free(&crt);
    return false;
  }

  size_t pemLen = 0;
  ret = mbedtls_pem_write_buffer("-----BEGIN CERTIFICATE-----\n", "-----END CERTIFICATE-----\n",
                                 crt.raw.p, crt.raw.len, (unsigned char*) _ztpCertPem,
                                 sizeof(_ztpCertPem) - 1, &pemLen);
  mbedtls_x509_crt_free(&crt);

  if(ret < 0) {
    ESP_LOGE("WalterModem", "Failed to write BlueCherry ZTP certificate PEM: -0x%04X", -ret);
    _ztpCertPem[0] = '\0';
    return false;
  }

  _ztpCertPem[pemLen] = '\0';

  return true;
}

bool WalterModem::_blueCherryZtpProvision()
{
  if(_blueCherry.ztpDeviceTypeId == NULL ||
     strlen(_blueCherry.ztpDeviceTypeId) != WALTER_MODEM_BLUECHERRY_ZTP_ID_LEN) {
    ESP_LOGE("WalterModem", "Cannot start BlueCherry ZTP: no valid device type id configured");
    return false;
  }

  ESP_LOGW("WalterModem", "Device is not provisioned for BlueCherry, starting ZTP...");

  /* validate the ZTP server against the BlueCherry CA only, we have no device cert/key yet. The
   * socket is closed first since the TLS profile it is configured with is rewritten here. */
  _blueCherrySocketDisconnect();

  if(!tlsWriteCredential(false, 6, _blueCherryCaCert) ||
     !tlsConfigProfile(_blueCherry.tls_profile_id, WALTER_MODEM_TLS_VALIDATION_CA,
                       WALTER_MODEM_TLS_VERSION_12, 6)) {
    ESP_LOGE("WalterModem", "Failed to configure TLS profile for BlueCherry ZTP");
    return false;
  }

  _blueCherry.curMessageId = 0x0000;
  _blueCherry.lastAckedMessageId = 0x0000;

  if(!_blueCherrySocketConnect(WALTER_MODEM_BLUECHERRY_ZTP_PORT)) {
    ESP_LOGE("WalterModem", "Could not connect to the BlueCherry ZTP server");
    _blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_NOT_PROVISIONED;
    return false;
  }

  _blueCherryZtpResetDeviceIdParams();

  uint8_t mac[6] = {};
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  if(!_blueCherryZtpAddDeviceIdParameter(WALTER_MODEM_BLUECHERRY_ZTP_DEVICE_ID_TYPE_MAC, mac)) {
    ESP_LOGE("WalterModem", "Could not add MAC address as BlueCherry ZTP device id parameter");
  }

  WalterModemRsp identityRsp = {};
  if(!getIdentity(&identityRsp) ||
     !_blueCherryZtpAddDeviceIdParameter(WALTER_MODEM_BLUECHERRY_ZTP_DEVICE_ID_TYPE_IMEI,
                                         identityRsp.data.identity.imei)) {
    ESP_LOGE("WalterModem", "Could not add IMEI as BlueCherry ZTP device id parameter");
  }

  bool ok = _blueCherryZtpRequestDeviceId() && _blueCherryZtpGenerateKeyAndCsr();

  if(ok) {
    /* give the ZTP server time to register the device id before requesting a certificate for it */
    vTaskDelay(pdMS_TO_TICKS(1000));

    ok = _blueCherryZtpRequestSignedCertificate() &&
         blueCherryProvision(_ztpCertPem, _ztpPrivKeyPem, _blueCherryCaCert);
  }

  /* the cloud connection uses mutual DTLS on a different port, so the ZTP socket is dropped and
   * _blueCherryFinishInit reconnects it */
  _blueCherrySocketDisconnect();

  memset(_ztpPrivKeyPem, 0, sizeof(_ztpPrivKeyPem));
  memset(_ztpCertPem, 0, sizeof(_ztpCertPem));
  memset(_ztpCsr, 0, sizeof(_ztpCsr));
  _ztpCsrLen = 0;
  _blueCherryZtpResetDeviceIdParams();

  /* whatever the transmissions left behind, we are provisioned or we are not */
  _blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_NOT_PROVISIONED;

  if(!ok) {
    ESP_LOGE("WalterModem", "BlueCherry ZTP provisioning failed");
  }

  return ok;
}

#pragma endregion // PRIVATE_METHODS_ZTP

#pragma region PUBLIC_METHODS

bool WalterModem::blueCherryProvision(const char* cert_pem, const char* priv_key_pem,
                                      const char* ca_cert, WalterModemRsp* rsp, walterModemCb cb,
                                      void* args)
{
  WalterModemState result = WALTER_MODEM_STATE_OK;

  if(cert_pem) {
    if(!tlsWriteCredential(false, 5, cert_pem)) {
      result = WALTER_MODEM_STATE_ERROR;
    }
  }

  if(priv_key_pem) {
    if(!tlsWriteCredential(true, 0, priv_key_pem)) {
      result = WALTER_MODEM_STATE_ERROR;
    }
  }

  if(ca_cert) {
    if(!tlsWriteCredential(false, 6, ca_cert)) {
      result = WALTER_MODEM_STATE_ERROR;
    }
  }

  _returnState(result);
}

bool WalterModem::blueCherryIsProvisioned()
{
  if(!_tlsIsCredentialPresent(false, 5)) {
    return false;
  }

  if(!_tlsIsCredentialPresent(false, 6)) {
    return false;
  }

  if(!_tlsIsCredentialPresent(true, 0)) {
    return false;
  }

  return true;
}

bool WalterModem::blueCherryInit(uint8_t tls_profile_id, uint8_t* ota_buffer, WalterModemRsp* rsp,
                                 uint16_t ack_timeout_s, const char* device_type_id)
{
  bool alreadyInitialized =
      _blueCherry.status != WALTER_MODEM_BLUECHERRY_STATUS_NOT_INITIALIZED &&
      _blueCherry.status != WALTER_MODEM_BLUECHERRY_STATUS_NOT_PROVISIONED;

  _blueCherry.tls_profile_id = tls_profile_id;
  _blueCherry.ota_buffer = ota_buffer;
  _blueCherry.ack_timeout_s = ack_timeout_s;
  _blueCherry.ztpDeviceTypeId = device_type_id;

  bool ok = true;
  if(alreadyInitialized) {
    ESP_LOGD("WalterModem", "BlueCherry already initialized");
  } else {
    ok = _blueCherryFinishInit();
  }

  if(rsp) {
    rsp->type = WALTER_MODEM_RSP_DATA_TYPE_BLUECHERRY;
    rsp->data.blueCherry.state = _blueCherry.status;
    rsp->data.blueCherry.messageCount = 0;
  }

  return ok;
}

bool WalterModem::blueCherryPublish(uint8_t topic, uint8_t len, uint8_t* data)
{
  if(_blueCherry.status != WALTER_MODEM_BLUECHERRY_STATUS_IDLE &&
     _blueCherry.status != WALTER_MODEM_BLUECHERRY_STATUS_NOT_CONNECTED) {
    return false;
  }

  if(_blueCherry.messageOutLen + len + 2 >= WALTER_MODEM_MAX_OUTGOING_MESSAGE_LEN) {
    return false;
  }

  _blueCherry.messageOut[_blueCherry.messageOutLen] = topic;
  _blueCherry.messageOut[_blueCherry.messageOutLen + 1] = len;
  memcpy(_blueCherry.messageOut + _blueCherry.messageOutLen + 2, data, len);

  _blueCherry.messageOutLen += len + 2;
  return true;
}

bool WalterModem::blueCherrySync(WalterModemRsp* rsp)
{
  walterModemCb cb = NULL;
  void* args = NULL;

  if(_blueCherry.status == WALTER_MODEM_BLUECHERRY_STATUS_NOT_PROVISIONED) {
    /* Zero-Touch Provisioning is only attempted when blueCherryInit was given a device type id;
     * otherwise Walter must still be provisioned manually via blueCherryProvision. */
    if(_blueCherry.ztpDeviceTypeId == NULL || !_blueCherryZtpProvision() ||
       !_blueCherryFinishInit()) {
      rsp->type = WALTER_MODEM_RSP_DATA_TYPE_BLUECHERRY;
      rsp->data.blueCherry.state = _blueCherry.status;
      rsp->data.blueCherry.messageCount = 0;
      _returnState(WALTER_MODEM_STATE_ERROR)
    }
  }

  if(_blueCherry.status != WALTER_MODEM_BLUECHERRY_STATUS_IDLE &&
     _blueCherry.status != WALTER_MODEM_BLUECHERRY_STATUS_NOT_CONNECTED) {
    _returnState(WALTER_MODEM_STATE_BUSY)
  }

  rsp->type = WALTER_MODEM_RSP_DATA_TYPE_BLUECHERRY;
  rsp->data.blueCherry.messageCount = 0;
  _blueCherry.messageInLen = 0;

  bool synced = false;

  if(_blueCherrySocketConnect()) {
    if(_blueCherryCoapSend()) {
      synced = true;
      // Reset Publish Buffer if successfull
      _blueCherry.messageOutLen = WALTER_MODEM_BLUECHERRY_COAP_HEADER_SIZE;
    }
  } else {
    _blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_NOT_CONNECTED;
  }

  /* only walk the payload of a datagram that was accepted during this cycle */
  uint16_t payloadOffset = synced ? _blueCherryCoapRx.payloadOffset : _blueCherry.messageInLen;
  while(payloadOffset + 2 <= _blueCherry.messageInLen) {
    uint8_t topic = _blueCherry.messageIn[payloadOffset++];
    uint8_t dataLen = _blueCherry.messageIn[payloadOffset++];

    if(payloadOffset + dataLen > _blueCherry.messageInLen) {
      ESP_LOGE("WalterModem", "Received malformed BlueCherry payload length");
      break;
    }

    /*
     * Topic 0 is reserved for BlueCherry events, which are also visible to walter as mqtt
     * messages on topic id 0
     */
    if(topic == 0) {
      _blueCherry.moreDataAvailable = true;

      if(_blueCherryProcessEvent(_blueCherry.messageIn + payloadOffset, dataLen)) {
        _blueCherry.emitErrorEvent = true;
        _blueCherry.otaSize = 0;
      }
    }

    if(rsp->data.blueCherry.messageCount <
       (int) (sizeof(rsp->data.blueCherry.messages) / sizeof(WalterModemBlueCherryMessage))) {
      WalterModemBlueCherryMessage* msg =
          rsp->data.blueCherry.messages + rsp->data.blueCherry.messageCount;
      msg->topic = topic;
      msg->dataSize = dataLen;
      msg->data = _blueCherry.messageIn + payloadOffset;

      rsp->data.blueCherry.messageCount++;
    }

    payloadOffset += dataLen;
  }

  // Mark the sync as finished whether we were successfull or not. This prevents an endless
  // loop and offloads any connectivity issues to the application.
  rsp->data.blueCherry.syncFinished = true;

  if(_blueCherry.emitErrorEvent) {
    uint8_t blueCherryErrorEventCode = WALTER_MODEM_BLUECHERRY_EVENT_TYPE_OTA_ERROR;
    blueCherryPublish(0, 1, &blueCherryErrorEventCode);
  }

  if(_blueCherry.status == WALTER_MODEM_BLUECHERRY_STATUS_TIMED_OUT) {
    _blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_IDLE;
    rsp->data.blueCherry.state = WALTER_MODEM_BLUECHERRY_STATUS_TIMED_OUT;
    _returnState(WALTER_MODEM_STATE_ERROR)
  }

  if(_blueCherry.status == WALTER_MODEM_BLUECHERRY_STATUS_NOT_CONNECTED) {
    _blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_IDLE;
    rsp->data.blueCherry.state = WALTER_MODEM_BLUECHERRY_STATUS_NOT_CONNECTED;
    _returnState(WALTER_MODEM_STATE_ERROR)
  }

  if(_blueCherry.moreDataAvailable) {
    rsp->data.blueCherry.syncFinished = false;
  } else {
    rsp->data.blueCherry.syncFinished = true;
  }

  _blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_IDLE;
  _returnState(WALTER_MODEM_STATE_OK);
}

bool WalterModem::blueCherryClose(WalterModemRsp* rsp, walterModemCb cb, void* args)
{
  _blueCherrySocketDisconnect();
  _blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_NOT_INITIALIZED;

  _returnState(WALTER_MODEM_STATE_OK);
}

size_t WalterModem::blueCherryGetOtaProgressPercentage()
{
  if(_blueCherry.otaSize == 0) {
    return 0; /* NO deviding by ZERO*/
  }
  return (_blueCherry.otaProgress * 100) / _blueCherry.otaSize;
}

size_t WalterModem::blueCherryGetOtaProgressBytes()
{
  return _blueCherry.otaProgress;
}
size_t WalterModem::blueCherryGetOtaSize()
{
  return _blueCherry.otaSize;
}

#pragma endregion // PUBLIC_METHODS
#endif
