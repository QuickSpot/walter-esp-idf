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
#include "WalterModem.h"

WalterModem modem;

uint8_t dataBuf[256] = { 0 };
uint8_t otaBuffer[SPI_FLASH_BLOCK_SIZE];
uint8_t counter = 0;

#define TLS_PROFILE 1

/* The keys below are not valid and hence this example will not
 * work without your own keys, but it serves as an illustration
 * on how to load the DTLS keys for secure COAP communication
 * with the BlueCherry lite cloud server.
 */

/**
 * @brief CA root certificate for DTLS (chain with intermediate: bandwidth!)
 */
const char *caCert = "-----BEGIN CERTIFICATE-----\r\n\
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
 * @brief Walter client certificate for DTLS
 */
const char *walterClientCert = "-----BEGIN CERTIFICATE-----\r\n\
MIIBNTCB3AICEAEwCgYIKoZIzj0EAwMwJDELMAkGA1UEBhMCQkUxFTATBgNVBAMM\r\n\
DGludGVybWVkaWF0ZTAeFw0yNDAzMjUxMDU5MzRaFw00NDA0MDkxMDU5MzRaMCkx\r\n\
CzAJBgNVBAYTAkJFMRowGAYDVQQDDBFsaXRlMDAwMS4xMTExMTExMTBZMBMGByqG\r\n\
SM49AgEGCCqGSM49AwEHA0IABPnA7m6yDd0w6iNuKWJ5T3eMB38Upk1yfM+fUUth\r\n\
AY/qh/BM8JYqG0KFpbR0ymNe+KU0m2cUCPR1QIUVvp3sIYYwCgYIKoZIzj0EAwMD\r\n\
SAAwRQIgDkAa7P78ieIamFqj8el2zL0oL/VHBYcTQL9/ZzsJBSkCIQCRFMsbIHc/\r\n\
AiKVsr/pbTYtxbyz0UJKUlVoM2S7CjeAKg==\r\n\
-----END CERTIFICATE-----\r\n";

/**
 * @brief Walter client private key for DTLS
 */
const char *walterClientKey = "-----BEGIN EC PRIVATE KEY-----\r\n\
MHcCAQEEIHsCxTfyp5l7OA0RbKTKkfbTOeZ26WtpfduUvXD6Ly0YoAoGCCqGSM49\r\n\
AwEHoUQDQgAE+cDubrIN3TDqI24pYnlPd4wHfxSmTXJ8z59RS2EBj+qH8Ezwliob\r\n\
QoWltHTKY174pTSbZxQI9HVAhRW+newhhg==\r\n\
-----END EC PRIVATE KEY-----\r\n";


void waitForNetwork()
{
  /* Wait for the network to become available */
  WalterModemNetworkRegState regState = modem.getNetworkRegState();
  while(!(regState == WALTER_MODEM_NETOWRK_REG_REGISTERED_HOME ||
          regState == WALTER_MODEM_NETOWRK_REG_REGISTERED_ROAMING))
  {
    vTaskDelay(pdMS_TO_TICKS(100));
    regState = modem.getNetworkRegState();
  }
  ESP_LOGI("bluecherry_test", "Connected to the network");
}

extern "C" void app_main(void)
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

  if(WalterModem::begin(UART_NUM_1)) {
    ESP_LOGI("bluecherry_test", "Modem initialization OK");
  } else {
    ESP_LOGI("bluecherry_test", "Modem initialization ERROR");
    return;
  }

  if(modem.checkComm()) {
    ESP_LOGI("bluecherry_test", "Modem communication is ok");
  } else {
    ESP_LOGI("bluecherry_test", "Modem communication error");
    return;
  }

  WalterModemRsp rsp = {};
  if(modem.getOpState(&rsp)) {
    ESP_LOGI("bluecherry_test", "Modem operational state: %d", rsp.data.opState);
  } else {
    ESP_LOGI("bluecherry_test", "Could not retrieve modem operational state");
    return;
  }

  if(modem.getRadioBands(&rsp)) {
    ESP_LOGI("bluecherry_test", "Modem is configured for the following bands:");
    
    for(int i = 0; i < rsp.data.bandSelCfgSet.count; ++i) {
      WalterModemBandSelection *bSel = rsp.data.bandSelCfgSet.config + i;
      ESP_LOGI("bluecherry_test", "  - Operator '%s' on %s: 0x%05lx",
        bSel->netOperator.name,
        bSel->rat == WALTER_MODEM_RAT_NBIOT ? "NB-IoT" : "LTE-M",
        bSel->bands);
    }
  } else {
    ESP_LOGI("bluecherry_test", "Could not retrieve configured radio bands");
    return;
  }

  if(modem.setOpState(WALTER_MODEM_OPSTATE_NO_RF)) {
    ESP_LOGI("bluecherry_test", "Successfully set operational state to NO RF");
  } else {
    ESP_LOGI("bluecherry_test", "Could not set operational state to NO RF");
    return;
  }

  /* Give the modem time to detect the SIM */
  vTaskDelay(pdMS_TO_TICKS(2000));

  if(modem.unlockSIM()) {
    ESP_LOGI("bluecherry_test", "Successfully unlocked SIM card");
  } else {
    ESP_LOGI("bluecherry_test", "Could not unlock SIM card");
    return;
  }

  /* Create PDP context */
  if(modem.createPDPContext("", WALTER_MODEM_PDP_AUTH_PROTO_PAP, "sora", "sora"))
  {
    ESP_LOGI("bluecherry_test", "Created PDP context");
  } else {
    ESP_LOGI("bluecherry_test", "Could not create PDP context");
    return;
  }

  /* Authenticate the PDP context */
  if(modem.authenticatePDPContext()) {
    ESP_LOGI("bluecherry_test", "Authenticated the PDP context");
  } else {
    ESP_LOGI("bluecherry_test", "Could not authenticate the PDP context");
    return;
  }

  /* Set the operational state to full */
  if(modem.setOpState(WALTER_MODEM_OPSTATE_FULL)) {
    ESP_LOGI("bluecherry_test", "Successfully set operational state to FULL");
  } else {
    ESP_LOGI("bluecherry_test", "Could not set operational state to FULL");
    return;
  }

  /* Set the network operator selection to automatic */
  if(modem.setNetworkSelectionMode(WALTER_MODEM_NETWORK_SEL_MODE_AUTOMATIC)) {
    ESP_LOGI("bluecherry_test", "Network selection mode to was set to automatic");
  } else {
    ESP_LOGI("bluecherry_test", "Could not set the network selection mode to automatic");
    return;
  }

  waitForNetwork();

  /* Activate the PDP context */
  if(modem.setPDPContextActive(true)) {
    ESP_LOGI("bluecherry_test", "Activated the PDP context");
  } else {
    ESP_LOGI("bluecherry_test", "Could not activate the PDP context");
    return;
  }

  /* Attach the PDP context */
  if(modem.attachPDPContext(true)) {
    ESP_LOGI("bluecherry_test", "Attached to the PDP context");
  } else {
    ESP_LOGI("bluecherry_test", "Could not attach to the PDP context");
    return;
  }

  if(modem.getPDPAddress(&rsp)) {
    ESP_LOGI("bluecherry_test", "PDP context address list:");
    ESP_LOGI("bluecherry_test", "  - %s", rsp.data.pdpAddressList.pdpAddress);
    if(rsp.data.pdpAddressList.pdpAddress2[0] != '\0') {
      ESP_LOGI("bluecherry_test", "  - %s", rsp.data.pdpAddressList.pdpAddress2);
    }
  } else {
    ESP_LOGI("bluecherry_test", "Could not retrieve PDP context addresses");
    return;
  }

  /* Upload keys to modem NVRAM keystore */
  if(modem.tlsProvisionKeys(walterClientCert, walterClientKey, caCert)) {
    ESP_LOGI("bluecherry_test", "Successfully uploaded the TLS keys");
  } else {
    ESP_LOGI("bluecherry_test", "Failed to upload the TLS keys");
  }

  /* Configure TLS profile */
  if(modem.tlsConfigProfile(TLS_PROFILE, WALTER_MODEM_TLS_VALIDATION_URL_AND_CA, WALTER_MODEM_TLS_VERSION_12, 6, 5, 0)) {
    ESP_LOGI("bluecherry_test", "Successfully configured the TLS profile");
  } else {
    ESP_LOGI("bluecherry_test", "Failed to configure TLS profile\r\n");
  }

  modem.initBlueCherry(1, "coap.bluecherry.io", 5684, otaBuffer);
  ESP_LOGI("bluecherry_test", "BlueCherry cloud platform link initialized");

  /* this loop is basically the Arduino loop function */
  for(;;) {
    WalterModemRsp rsp = {};
    bool moreDataAvailable;
  
    vTaskDelay(pdMS_TO_TICKS(15000));
  
    dataBuf[6] = counter++;
    modem.blueCherryPublish(0x84, 7, dataBuf);
  
    do {
      if(!modem.blueCherrySynchronize()) {
        ESP_LOGI("bluecherry_test", "Error communicating with BlueCherry cloud platform!");
        ESP_LOGI("bluecherry_test", "Rebooting modem after BlueCherry sync failure (CoAP stack may be broken)");
        modem.reset();
        modem.setOpState(WALTER_MODEM_OPSTATE_FULL);
        waitForNetwork();
        ESP_LOGI("bluecherry_test", "Continuing");
        break;
      }
  
      ESP_LOGI("bluecherry_test", "Synchronized with the BlueCherry cloud platform, awaiting ring for ACK");
  
      while(!modem.blueCherryDidRing(&moreDataAvailable, &rsp)) {
        vTaskDelay(pdMS_TO_TICKS(100));
      }
  
      if(rsp.data.blueCherry.nak) {
        ESP_LOGI("bluecherry_test", "Rebooting modem after timeout waiting for ACK (workaround bug)");
        modem.reset();
        modem.setOpState(WALTER_MODEM_OPSTATE_FULL);
        waitForNetwork();
        ESP_LOGI("bluecherry_test", "Continuing");
        break;
      }
  
      ESP_LOGI("bluecherry_test", "Successfully sent message. Nr incoming msgs: %d",
        rsp.data.blueCherry.messageCount);
  
      for(uint8_t msgIdx = 0; msgIdx < rsp.data.blueCherry.messageCount; msgIdx++) {
        ESP_LOGI("bluecherry_test", "Incoming message %d/%d:", msgIdx + 1, rsp.data.blueCherry.messageCount);
        ESP_LOGI("bluecherry_test", "topic: %02x", rsp.data.blueCherry.messages[msgIdx].topic);
        ESP_LOGI("bluecherry_test", "data size: %d", rsp.data.blueCherry.messages[msgIdx].dataSize);
      }
  
      if(moreDataAvailable) {
        ESP_LOGI("bluecherry_test", "(got some incoming data but more is waiting to be fetched: doing another sync call)");
      }
    } while(moreDataAvailable);
  }
}
