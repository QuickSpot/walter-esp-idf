/**
 * @file WalterBlueCherryZTP.cpp
 * @author Daan Pape <daan@dptechnics.com>
 * @author Arnoud Devoogdt <arnoud@dptechnics.com>
 * @date 5 August 2026
 * @version 1.5.0
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
 * This file contains the implementation of BlueCherry Zero-Touch Provisioning (ZTP): fetching a
 * device ID and a signed device certificate from the BlueCherry cloud over CoAP + DTLS, so that
 * Walter can provision itself for BlueCherry cloud connectivity without manufacturing-time
 * credentials. blueCherrySync triggers this automatically when Walter is not provisioned yet.
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
                                     unsigned char* decoded_data, size_t* decoded_len)
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

  memcpy(decoded_data, cbor_data + offset, length);
  *decoded_len = length;

  return 0;
}

#pragma endregion // PRIVATE_CBOR

#pragma region PRIVATE_CONSTANTS

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

#pragma region PRIVATE_METHODS

/**
 * @brief Set when a CoAP ring is received while awaiting a ZTP server response.
 */
static bool _ztpCoapRingReceived = false;

/**
 * @brief The CoAP message id of the last ZTP ring, used to retrieve the response.
 */
static uint16_t _ztpCoapRingMsgId = 0;

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

void WalterModem::_blueCherryZtpCoapEventHandler(WMCoAPEventType event,
                                                 const WMCoAPEventData* data, void* args)
{
  if(event == WALTER_MODEM_COAP_EVENT_RING) {
    _ztpCoapRingMsgId = data->msg_id;
    _ztpCoapRingReceived = true;
  }
}

void WalterModem::_blueCherryZtpResetDeviceIdParams()
{
  _blueCherry.ztp.paramCount = 0;
}

bool WalterModem::_blueCherryZtpAddDeviceIdParameter(WalterModemBlueCherryZtpDeviceIdParamType type,
                                                     const char* str)
{
  if(str == NULL || type != WALTER_MODEM_BLUECHERRY_ZTP_DEVICE_ID_TYPE_IMEI ||
     _blueCherry.ztp.paramCount >= WALTER_MODEM_BLUECHERRY_ZTP_MAX_DEVICE_ID_PARAMS) {
    return false;
  }

  WalterModemBlueCherryZtpDeviceIdParam* param = &_blueCherry.ztp.params[_blueCherry.ztp.paramCount];
  param->type = type;
  _strncpy_s(param->value.imei, str, sizeof(param->value.imei));
  _blueCherry.ztp.paramCount++;

  return true;
}

bool WalterModem::_blueCherryZtpAddDeviceIdParameter(WalterModemBlueCherryZtpDeviceIdParamType type,
                                                     const uint8_t* blob)
{
  if(blob == NULL || type != WALTER_MODEM_BLUECHERRY_ZTP_DEVICE_ID_TYPE_MAC ||
     _blueCherry.ztp.paramCount >= WALTER_MODEM_BLUECHERRY_ZTP_MAX_DEVICE_ID_PARAMS) {
    return false;
  }

  WalterModemBlueCherryZtpDeviceIdParam* param = &_blueCherry.ztp.params[_blueCherry.ztp.paramCount];
  param->type = type;
  memcpy(param->value.mac, blob, sizeof(param->value.mac));
  _blueCherry.ztp.paramCount++;

  return true;
}

bool WalterModem::_blueCherryZtpAddDeviceIdParameter(WalterModemBlueCherryZtpDeviceIdParamType type,
                                                     uint64_t number)
{
  if(type != WALTER_MODEM_BLUECHERRY_ZTP_DEVICE_ID_TYPE_OOB_CHALLENGE ||
     _blueCherry.ztp.paramCount >= WALTER_MODEM_BLUECHERRY_ZTP_MAX_DEVICE_ID_PARAMS) {
    return false;
  }

  WalterModemBlueCherryZtpDeviceIdParam* param = &_blueCherry.ztp.params[_blueCherry.ztp.paramCount];
  param->type = type;
  param->value.oobChallenge = number;
  _blueCherry.ztp.paramCount++;

  return true;
}

bool WalterModem::_blueCherryZtpRequestDeviceId()
{
  uint8_t cborBuf[256];
  uint8_t coapData[16];
  WalterModemRsp rsp = {};
  _WalterModemZtpCbor cbor;

  if(_ztpCborInit(&cbor, cborBuf, sizeof(cborBuf)) < 0 || _ztpCborStartArray(&cbor, 2) < 0 ||
     _ztpCborEncodeString(&cbor, _blueCherry.ztp.deviceTypeId) < 0 ||
     _ztpCborStartMap(&cbor, _blueCherry.ztp.paramCount) < 0) {
    ESP_LOGE("WalterModem", "Failed to encode BlueCherry ZTP device id request");
    return false;
  }

  for(uint8_t i = 0; i < _blueCherry.ztp.paramCount; i++) {
    WalterModemBlueCherryZtpDeviceIdParam* param = &_blueCherry.ztp.params[i];

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

  walterModemCoAPEventHandler prevHandler = _eventHandlers[WALTER_MODEM_EVENT_TYPE_COAP].coapHandler;
  void* prevArgs = _eventHandlers[WALTER_MODEM_EVENT_TYPE_COAP].args;
  setCoAPEventHandler(_blueCherryZtpCoapEventHandler, NULL);

  bool sent =
      coapCreateContext(WALTER_MODEM_BLUECHERRY_ZTP_COAP_PROFILE, WALTER_MODEM_BLUECHERRY_HOSTNAME,
                        WALTER_MODEM_BLUECHERRY_ZTP_PORT, _blueCherry.tls_profile_id) &&
      coapSetOptions(WALTER_MODEM_BLUECHERRY_ZTP_COAP_PROFILE, WALTER_MODEM_COAP_OPT_SET,
                    WALTER_MODEM_COAP_OPT_CODE_URI_PATH, WALTER_MODEM_BLUECHERRY_ZTP_API_VERSION) &&
      coapSetOptions(WALTER_MODEM_BLUECHERRY_ZTP_COAP_PROFILE, WALTER_MODEM_COAP_OPT_EXTEND,
                    WALTER_MODEM_COAP_OPT_CODE_URI_PATH, WALTER_MODEM_BLUECHERRY_ZTP_DEVID_PATH) &&
      coapSendData(WALTER_MODEM_BLUECHERRY_ZTP_COAP_PROFILE, WALTER_MODEM_COAP_SEND_TYPE_CON,
                  WALTER_MODEM_COAP_SEND_METHOD_GET, _ztpCborSize(&cbor), cborBuf);

  bool received = false;
  if(sent) {
    _ztpCoapRingReceived = false;
    int attempts = WALTER_MODEM_BLUECHERRY_ZTP_TIMEOUT_S;
    while(attempts && !_ztpCoapRingReceived) {
      vTaskDelay(pdMS_TO_TICKS(1000));
      attempts--;
    }

    received = _ztpCoapRingReceived &&
               coapReceive(WALTER_MODEM_BLUECHERRY_ZTP_COAP_PROFILE, _ztpCoapRingMsgId, coapData,
                          sizeof(coapData), &rsp);
  }

  _eventHandlers[WALTER_MODEM_EVENT_TYPE_COAP].coapHandler = prevHandler;
  _eventHandlers[WALTER_MODEM_EVENT_TYPE_COAP].args = prevArgs;

  if(!received) {
    ESP_LOGE("WalterModem", "Failed to receive BlueCherry ZTP device id response");
    return false;
  }

  if(_ztpCborDecodeDeviceId(coapData, rsp.data.coapResponse.length, _blueCherry.ztp.deviceId,
                            sizeof(_blueCherry.ztp.deviceId)) < 0) {
    ESP_LOGE("WalterModem", "Failed to decode BlueCherry ZTP device id");
    return false;
  }

  return true;
}

bool WalterModem::_blueCherryZtpGenerateKeyAndCsr()
{
  if(_blueCherry.ztp.deviceTypeId == NULL ||
     strlen(_blueCherry.ztp.deviceTypeId) != WALTER_MODEM_BLUECHERRY_ZTP_ID_LEN ||
     strlen(_blueCherry.ztp.deviceId) != WALTER_MODEM_BLUECHERRY_ZTP_ID_LEN) {
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

  bool ok =
      _blueCherryZtpSeedRandom(&ctrDrbg, &entropy) &&
      mbedtls_pk_setup(&key, mbedtls_pk_info_from_type(MBEDTLS_PK_ECKEY)) == 0 &&
      mbedtls_ecp_gen_key(MBEDTLS_ECP_DP_SECP256R1, mbedtls_pk_ec(key), mbedtls_ctr_drbg_random,
                         &ctrDrbg) == 0 &&
      mbedtls_pk_write_key_pem(&key, (unsigned char*) _blueCherry.ztp.privKeyPem,
                              sizeof(_blueCherry.ztp.privKeyPem)) == 0;

  if(ok) {
    mbedtls_x509write_csr_set_md_alg(&csr, MBEDTLS_MD_SHA256);
    mbedtls_x509write_csr_set_key(&csr, &key);

    snprintf(_blueCherry.ztp.subject, sizeof(_blueCherry.ztp.subject), "C=BE,CN=%s.%s",
             _blueCherry.ztp.deviceTypeId, _blueCherry.ztp.deviceId);

    uint8_t csrBuf[sizeof(_blueCherry.ztp.csr)];
    int ret = -1;
    if(mbedtls_x509write_csr_set_subject_name(&csr, _blueCherry.ztp.subject) == 0) {
      ret = mbedtls_x509write_csr_der(&csr, csrBuf, sizeof(csrBuf), mbedtls_ctr_drbg_random,
                                      &ctrDrbg);
    }

    if(ret < 0) {
      ESP_LOGE("WalterModem", "Failed to write BlueCherry ZTP CSR: -0x%04X", -ret);
      ok = false;
    } else {
      size_t offset = sizeof(csrBuf) - ret;
      _blueCherry.ztp.csrLen = ret;
      memcpy(_blueCherry.ztp.csr, csrBuf + offset, _blueCherry.ztp.csrLen);
    }
  }

  mbedtls_pk_free(&key);
  mbedtls_entropy_free(&entropy);
  mbedtls_ctr_drbg_free(&ctrDrbg);
  mbedtls_x509write_csr_free(&csr);

  if(!ok) {
    _blueCherry.ztp.privKeyPem[0] = '\0';
    _blueCherry.ztp.certPem[0] = '\0';
  }

  return ok;
}

bool WalterModem::_blueCherryZtpRequestSignedCertificate()
{
  uint8_t buf[sizeof(_blueCherry.ztp.certPem)];
  uint8_t coapData[sizeof(_blueCherry.ztp.certPem)];
  WalterModemRsp rsp = {};
  _WalterModemZtpCbor cbor;

  _ztpCborInit(&cbor, buf, sizeof(buf));

  if(_ztpCborEncodeBytes(&cbor, _blueCherry.ztp.csr, _blueCherry.ztp.csrLen) < 0) {
    ESP_LOGE("WalterModem", "Failed to encode BlueCherry ZTP CSR");
    return false;
  }

  walterModemCoAPEventHandler prevHandler = _eventHandlers[WALTER_MODEM_EVENT_TYPE_COAP].coapHandler;
  void* prevArgs = _eventHandlers[WALTER_MODEM_EVENT_TYPE_COAP].args;
  setCoAPEventHandler(_blueCherryZtpCoapEventHandler, NULL);

  bool sent =
      coapSetOptions(WALTER_MODEM_BLUECHERRY_ZTP_COAP_PROFILE, WALTER_MODEM_COAP_OPT_SET,
                    WALTER_MODEM_COAP_OPT_CODE_URI_PATH, WALTER_MODEM_BLUECHERRY_ZTP_API_VERSION) &&
      coapSetOptions(WALTER_MODEM_BLUECHERRY_ZTP_COAP_PROFILE, WALTER_MODEM_COAP_OPT_EXTEND,
                    WALTER_MODEM_COAP_OPT_CODE_URI_PATH, WALTER_MODEM_BLUECHERRY_ZTP_CSR_PATH) &&
      coapSendData(WALTER_MODEM_BLUECHERRY_ZTP_COAP_PROFILE, WALTER_MODEM_COAP_SEND_TYPE_CON,
                  WALTER_MODEM_COAP_SEND_METHOD_GET, _ztpCborSize(&cbor), buf);

  bool received = false;
  if(sent) {
    _ztpCoapRingReceived = false;
    int attempts = WALTER_MODEM_BLUECHERRY_ZTP_TIMEOUT_S;
    while(attempts && !_ztpCoapRingReceived) {
      vTaskDelay(pdMS_TO_TICKS(1000));
      attempts--;
    }

    received = _ztpCoapRingReceived &&
               coapReceive(WALTER_MODEM_BLUECHERRY_ZTP_COAP_PROFILE, _ztpCoapRingMsgId, coapData,
                          sizeof(coapData), &rsp);
  }

  _eventHandlers[WALTER_MODEM_EVENT_TYPE_COAP].coapHandler = prevHandler;
  _eventHandlers[WALTER_MODEM_EVENT_TYPE_COAP].args = prevArgs;

  if(!received) {
    ESP_LOGE("WalterModem", "Failed to receive BlueCherry ZTP signed certificate response");
    return false;
  }

  size_t decodedSize;
  if(_ztpCborDecodeCertificate(coapData, rsp.data.coapResponse.length, buf, &decodedSize) < 0) {
    ESP_LOGE("WalterModem", "Failed to decode BlueCherry ZTP certificate");
    return false;
  }

  mbedtls_x509_crt crt;
  mbedtls_x509_crt_init(&crt);

  int ret = mbedtls_x509_crt_parse_der(&crt, buf, decodedSize);
  if(ret < 0) {
    ESP_LOGE("WalterModem", "Failed to parse BlueCherry ZTP DER certificate: -0x%04X", -ret);
    mbedtls_x509_crt_free(&crt);
    return false;
  }

  size_t pemLen;
  ret = mbedtls_pem_write_buffer("-----BEGIN CERTIFICATE-----\n", "-----END CERTIFICATE-----\n",
                                 crt.raw.p, crt.raw.len, buf, sizeof(buf), &pemLen);
  mbedtls_x509_crt_free(&crt);

  if(ret < 0) {
    ESP_LOGE("WalterModem", "Failed to write BlueCherry ZTP certificate PEM: -0x%04X", -ret);
    return false;
  }

  memcpy(_blueCherry.ztp.certPem, buf, pemLen);
  _blueCherry.ztp.certPem[pemLen] = '\0';

  return true;
}

bool WalterModem::_blueCherryZtpProvision()
{
  if(_blueCherry.ztp.deviceTypeId == NULL ||
     strlen(_blueCherry.ztp.deviceTypeId) != WALTER_MODEM_BLUECHERRY_ZTP_ID_LEN) {
    ESP_LOGE("WalterModem", "Cannot start BlueCherry ZTP: no valid device type id configured");
    return false;
  }

  ESP_LOGW("WalterModem", "Device is not provisioned for BlueCherry, starting ZTP...");

  /* validate the ZTP server against the BlueCherry CA only, we have no device cert/key yet */
  if(!tlsWriteCredential(false, 6, _blueCherryCaCert) ||
     !tlsConfigProfile(_blueCherry.tls_profile_id, WALTER_MODEM_TLS_VALIDATION_CA,
                       WALTER_MODEM_TLS_VERSION_12, 6)) {
    ESP_LOGE("WalterModem", "Failed to configure TLS profile for BlueCherry ZTP");
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

  bool ok = _blueCherryZtpRequestDeviceId() && _blueCherryZtpGenerateKeyAndCsr() &&
            _blueCherryZtpRequestSignedCertificate() &&
            blueCherryProvision(_blueCherry.ztp.certPem, _blueCherry.ztp.privKeyPem,
                               _blueCherryCaCert);

  coapClose(WALTER_MODEM_BLUECHERRY_ZTP_COAP_PROFILE);

  if(!ok) {
    ESP_LOGE("WalterModem", "BlueCherry ZTP provisioning failed");
  }

  return ok;
}

#pragma endregion // PRIVATE_METHODS

#endif
