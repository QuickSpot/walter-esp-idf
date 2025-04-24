/**
 * @file bluecherry_test.cpp
 * @author Jonas Maes <jonas@dptechnics.com>
 * @date 27 Nov 2023
 * @copyright DPTechnics bv
 * @brief Walter Modem library examples
 *
 * @section LICENSE
 *
 * Copyright (C) 2023, DPTechnics bv
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 * 
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 * 
 *   3. Neither the name of DPTechnics bv nor the names of its contributors may
 *      be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 * 
 *   4. This software, with or without modification, must only be used with a
 *      Walter board from DPTechnics bv.
 * 
 *   5. Any software provided in binary form under this license must not be
 *      reverse engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY DPTECHNICS BV “AS IS” AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL DPTECHNICS BV OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @section DESCRIPTION
 * 
 * This program sends and receives mqtt data using the DPTechnics BlueCherry cloud platform.
 * It also supports OTA updates which are scheduled through the BlueCherry web interface.
 */

#include <esp_mac.h>
#include <esp_log.h>
#include <driver/uart.h>
#include <WalterModem.h>
#include <components/Bluecherry_ZTP/BlueCherryZTP.h>
#include <components/Bluecherry_ZTP/BlueCherryZTP_CBOR.h>

WalterModem modem;

/**
 * @brief Cellular APN for SIM card. Leave empty to autodetect APN.
 */
#define CELLULAR_APN ""

// Define BlueCherry cloud device ID
#define BC_DEVICE_TYPE "walter01"

// Define modem TLS profile used for BlueCherry cloud platform
#define BC_TLS_PROFILE 1

uint8_t dataBuf[256] = { 0 };
uint8_t otaBuffer[SPI_FLASH_BLOCK_SIZE] = {0};
uint8_t counter = 0;


/* The keys below are not valid and hence this example will not
 * work without your own keys, but it serves as an illustration
 * on how to load the DTLS keys for secure COAP communication
 * with the BlueCherry lite cloud server.
 */

/**
 * @brief CA root certificate for DTLS (chain with intermediate: bandwidth!)
 */
const char *bc_ca_cert = "-----BEGIN CERTIFICATE-----\r\n\
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

/**
 * @brief This function tries to connect the modem to the cellular network.
 * @return true if the connection attempt is successful, else false.
 */
bool lteConnected()
{
  WalterModemNetworkRegState regState = modem.getNetworkRegState();
  return (regState == WALTER_MODEM_NETWORK_REG_REGISTERED_HOME ||
          regState == WALTER_MODEM_NETWORK_REG_REGISTERED_ROAMING);
}
bool waitForNetwork()
{
  /* Wait for the network to become available */
  int timeout = 0;
  while (!lteConnected())
  {
    vTaskDelay(pdMS_TO_TICKS(100));
    timeout += 100;
    if (timeout > 300000)
      return false;
  }
  ESP_LOGI("bluecherry_test", "Connected to the network");
  return true;
}

bool lteConnect()
{
  WalterModemRsp rsp = {};

  if (modem.setOpState(WALTER_MODEM_OPSTATE_NO_RF)) {
    ESP_LOGI("bluecherry_test", "Successfully set operational state to NO RF");
  } else {
    ESP_LOGI("bluecherry_test", "Could not set operational state to NO RF");
    return false;
  }

  /* Give the modem time to detect the SIM */
  vTaskDelay(pdMS_TO_TICKS(2000));

  /* Create PDP context */
  if (modem.definePDPContext(1,CELLULAR_APN)) {
    ESP_LOGI("coap_test", "Created PDP context");
  } else {
    ESP_LOGI("coap_test", "Could not create PDP context");
    return false;
  }

  /* Set the operational state to full */
  if (modem.setOpState(WALTER_MODEM_OPSTATE_FULL)) {
    ESP_LOGI("bluecherry_test", "Successfully set operational state to FULL");
  } else {
    ESP_LOGI("bluecherry_test", "Could not set operational state to FULL");
    return false;
  }

  /* Set the network operator selection to automatic */
  if (modem.setNetworkSelectionMode(WALTER_MODEM_NETWORK_SEL_MODE_AUTOMATIC)) {
    ESP_LOGI("bluecherry_test", "Network selection mode to was set to automatic");
  } else {
    ESP_LOGI("bluecherry_test", "Could not set the network selection mode to automatic");
    return false;
  }

  return waitForNetwork();
}
// This function will poll the BlueCherry cloud platform to check if there is an
// incoming MQTT message or new firmware version available. If a new firmware
// version is available, the device automatically downloads and reboots with the
// new firmware.
void syncBlueCherry()
{
  WalterModemRsp rsp = {};

  do
  {
    if (!modem.blueCherrySync(&rsp))
    {
      ESP_LOGE(
          "bluecherry_test",
          "Error during BlueCherry cloud platform synchronisation: %d",
          rsp.data.blueCherry.state);
      modem.softReset();
      lteConnect();
      return;
    }

    for (uint8_t msgIdx = 0; msgIdx < rsp.data.blueCherry.messageCount; msgIdx++)
    {
      if (rsp.data.blueCherry.messages[msgIdx].topic == 0)
      {
        ESP_LOGI("bluecherry_test", "Downloading new firmware version");
        break;
      }
      else
      {
        ESP_LOGI("bluecherry_test", "Incoming message %d/%d:", msgIdx + 1, rsp.data.blueCherry.messageCount);
        ESP_LOGI("bluecherry_test", "Topic: %02x\r\n", rsp.data.blueCherry.messages[msgIdx].topic);
        ESP_LOGI("bluecherry_test", "Data size: %d\r\n", rsp.data.blueCherry.messages[msgIdx].dataSize);

        for (uint8_t byteIdx = 0; byteIdx < rsp.data.blueCherry.messages[msgIdx].dataSize; byteIdx++) {
          ESP_LOGI("bluecherry_test", "%c", rsp.data.blueCherry.messages[msgIdx].data[byteIdx]);
        }

        ESP_LOGI("bluecherry_test", "\r\n");
      }
    }
  } while (!rsp.data.blueCherry.syncFinished);

  ESP_LOGI("bluecherry_test", "Synchronized with BlueCherry cloud platform");
  return;
}

void init()
{
  ESP_LOGI("bluecherry_test", "Walter modem test v0.0.1");

  /* Get the MAC address for board validation */
  esp_read_mac(dataBuf, ESP_MAC_WIFI_STA);
  ESP_LOGI("bluecherry_test", "Walter's MAC is: %02X:%02X:%02X:%02X:%02X:%02X",
           dataBuf[0],
           dataBuf[1],
           dataBuf[2],
           dataBuf[3],
           dataBuf[4],
           dataBuf[5]);

  if (WalterModem::begin(UART_NUM_1)) { 
    ESP_LOGI("bluecherry_test", "Modem initialization OK");
  } else {
    ESP_LOGI("bluecherry_test", "Modem initialization ERROR");
    return;
  }

  if (modem.checkComm()) {
    ESP_LOGI("bluecherry_test", "Modem communication is ok");
  } else {
    ESP_LOGI("bluecherry_test", "Modem communication error");
    return;
  }

  lteConnect();

  WalterModemRsp rsp = {};
  unsigned short attempt = 0;
  while (!modem.blueCherryInit(BC_TLS_PROFILE, otaBuffer, &rsp))
  {
    if (rsp.data.blueCherry.state ==
            WALTER_MODEM_BLUECHERRY_STATUS_NOT_PROVISIONED &&
        attempt <= 2)
    {
      ESP_LOGI(
        "bluecherry_test",
        "Device is not provisioned for BlueCherry \n communication, starting Zero Touch Provisioning");

      if (attempt == 0)
      {
        // Device is not provisioned yet, initialize BlueCherry zero touch
        // provisioning
        if (!BlueCherryZTP::begin(BC_DEVICE_TYPE, BC_TLS_PROFILE, bc_ca_cert,
                                  &modem))
        {
          ESP_LOGE("bluecherry_test", "Failed to initialize ZTP");
          continue;
        }

        // Fetch MAC address
        uint8_t mac[8] = {0};
        esp_read_mac(mac, ESP_MAC_WIFI_STA);
        if (!BlueCherryZTP::addDeviceIdParameter(
                BLUECHERRY_ZTP_DEVICE_ID_TYPE_MAC, mac))
        {
          ESP_LOGE("bluecherry_test", "Could not add MAC address as ZTP device ID parameter");
        }

        // Fetch IMEI number
        if (!modem.getIdentity(&rsp))
        {
          ESP_LOGE("bluecherry_test", "Could not fetch IMEI number from modem");
        }

        if (!BlueCherryZTP::addDeviceIdParameter(
                BLUECHERRY_ZTP_DEVICE_ID_TYPE_IMEI, rsp.data.identity.imei))
        {
          ESP_LOGE("bluecherry_test", "Could not add IMEI as ZTP device ID parameter");
        }
      }
      attempt++;

      // Request the BlueCherry device ID
      if (!BlueCherryZTP::requestDeviceId())
      {
        ESP_LOGE("bluecherry_test", "Could not request device ID");
        continue;
      }

      // Generate the private key and CSR
      if (!BlueCherryZTP::generateKeyAndCsr())
      {
        ESP_LOGE("bluecherry_test", "Could not generate private key");
      }
      vTaskDelay(pdMS_TO_TICKS(1000));

      // Request the signed certificate
      if (!BlueCherryZTP::requestSignedCertificate())
      {
        ESP_LOGE("bluecherry_test", "Could not request signed certificate");
        continue;
      }

      // Store BlueCherry TLS certificates + private key in the modem
      if (!modem.blueCherryProvision(BlueCherryZTP::getCert(),
                                     BlueCherryZTP::getPrivKey(), bc_ca_cert))
      {
        ESP_LOGE("bluecherry_test", "Failed to upload the DTLS certificates");
        continue;
      }
    }
    else
    {
      ESP_LOGE("bluecherry_test", "Failed to initialize BlueCherry cloud platform, \n restarting Walter in 10 seconds");
      vTaskDelay(pdMS_TO_TICKS(10000));
      esp_restart();
    }
  }
  ESP_LOGI("bluecherry_test", "Successfully initialized BlueCherry cloud platform");
}

void loop()
{
  /* this loop is basically the Arduino loop function */

  for (;;)
  {
    WalterModemRsp rsp = {};
    if (modem.getCellInformation(WALTER_MODEM_SQNMONI_REPORTS_SERVING_CELL,
                                 &rsp))
    {
      char msg[18];
      snprintf(msg, sizeof(msg), "{\"RSRP\": %7.2f}", rsp.data.cellInformation.rsrp);
      modem.blueCherryPublish(0x84, sizeof(msg) - 1, (uint8_t *)msg);
    }

    // Poll BlueCherry platform if an incoming message or firmware update is available
    syncBlueCherry();
  }
}

extern "C" void app_main(void)
{
  init();
  loop();
}
