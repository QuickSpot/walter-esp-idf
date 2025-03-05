/**
 * @file mqtt_test.cpp
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
#include <cstring>
#include "WalterModem.h"

#define TLS_PROFILE 1

WalterModem modem;

uint8_t incomingBuf[256] = { 0 };

char macString[32];

void waitForNetwork()
{
  /* Wait for the network to become available */
  WalterModemNetworkRegState regState = modem.getNetworkRegState();
  while(!(regState == WALTER_MODEM_NETWORK_REG_REGISTERED_HOME ||
          regState == WALTER_MODEM_NETWORK_REG_REGISTERED_ROAMING))
  {
    vTaskDelay(pdMS_TO_TICKS(100));
    regState = modem.getNetworkRegState();
  }
  ESP_LOGI("mqtt_test", "Connected to the network");
}

extern "C" void app_main(void)
{
  ESP_LOGI("mqtt_test", "Walter modem test v0.0.1");

  /* Get the MAC address for board validation */
  esp_read_mac(incomingBuf, ESP_MAC_WIFI_STA);
  sprintf(macString, "walter%02X:%02X:%02X:%02X:%02X:%02X",
    incomingBuf[0],
    incomingBuf[1],
    incomingBuf[2],
    incomingBuf[3],
    incomingBuf[4],
    incomingBuf[5]);

  if(WalterModem::begin(UART_NUM_1)) {
    ESP_LOGI("mqtt_test", "Modem initialization OK");
  } else {
    ESP_LOGI("mqtt_test", "Modem initialization ERROR");
    return;
  }

  if(modem.checkComm()) {
    ESP_LOGI("mqtt_test", "Modem communication is ok");
  } else {
    ESP_LOGI("mqtt_test", "Modem communication error");
    return;
  }

  WalterModemRsp rsp = {};
  if(modem.getOpState(&rsp)) {
    ESP_LOGI("mqtt_test", "Modem operational state: %d", rsp.data.opState);
  } else {
    ESP_LOGI("mqtt_test", "Could not retrieve modem operational state");
    return;
  }

  if(modem.getRadioBands(&rsp)) {
    ESP_LOGI("mqtt_test", "Modem is configured for the following bands:");
    
    for(int i = 0; i < rsp.data.bandSelCfgSet.count; ++i) {
      WalterModemBandSelection *bSel = rsp.data.bandSelCfgSet.config + i;
      ESP_LOGI("mqtt_test", "  - Operator '%s' on %s: 0x%05lx",
        bSel->netOperator.name,
        bSel->rat == WALTER_MODEM_RAT_NBIOT ? "NB-IoT" : "LTE-M",
        bSel->bands);
    }
  } else {
    ESP_LOGI("mqtt_test", "Could not retrieve configured radio bands");
    return;
  }

  if(modem.setOpState(WALTER_MODEM_OPSTATE_NO_RF)) {
    ESP_LOGI("mqtt_test", "Successfully set operational state to NO RF");
  } else {
    ESP_LOGI("mqtt_test", "Could not set operational state to NO RF");
    return;
  }

  /* Give the modem time to detect the SIM */
  vTaskDelay(pdMS_TO_TICKS(2000));

  if(modem.unlockSIM()) {
    ESP_LOGI("mqtt_test", "Successfully unlocked SIM card");
  } else {
    ESP_LOGI("mqtt_test", "Could not unlock SIM card");
    return;
  }

  /* Create PDP context */
  if(modem.createPDPContext("", WALTER_MODEM_PDP_AUTH_PROTO_PAP, "sora", "sora"))
  {
    ESP_LOGI("mqtt_test", "Created PDP context");
  } else {
    ESP_LOGI("mqtt_test", "Could not create PDP context");
    return;
  }

  /* Authenticate the PDP context */
  if(modem.authenticatePDPContext()) {
    ESP_LOGI("mqtt_test", "Authenticated the PDP context");
  } else {
    ESP_LOGI("mqtt_test", "Could not authenticate the PDP context");
    return;
  }

  /* Set the operational state to full */
  if(modem.setOpState(WALTER_MODEM_OPSTATE_FULL)) {
    ESP_LOGI("mqtt_test", "Successfully set operational state to FULL");
  } else {
    ESP_LOGI("mqtt_test", "Could not set operational state to FULL");
    return;
  }

  /* Set the network operator selection to automatic */
  if(modem.setNetworkSelectionMode(WALTER_MODEM_NETWORK_SEL_MODE_AUTOMATIC)) {
    ESP_LOGI("mqtt_test", "Network selection mode to was set to automatic");
  } else {
    ESP_LOGI("mqtt_test", "Could not set the network selection mode to automatic");
    return;
  }

  waitForNetwork();

  /* Activate the PDP context */
  if(modem.setPDPContextActive(true)) {
    ESP_LOGI("mqtt_test", "Activated the PDP context");
  } else {
    ESP_LOGI("mqtt_test", "Could not activate the PDP context");
    return;
  }

  /* Attach the PDP context */
  if(modem.attachPDPContext(true)) {
    ESP_LOGI("mqtt_test", "Attached to the PDP context");
  } else {
    ESP_LOGI("mqtt_test", "Could not attach to the PDP context");
    return;
  }

  if(modem.getPDPAddress(&rsp)) {
    ESP_LOGI("mqtt_test", "PDP context address list:");
    ESP_LOGI("mqtt_test", "  - %s", rsp.data.pdpAddressList.pdpAddress);
    if(rsp.data.pdpAddressList.pdpAddress2[0] != '\0') {
      ESP_LOGI("mqtt_test", "  - %s", rsp.data.pdpAddressList.pdpAddress2);
    }
  } else {
    ESP_LOGI("mqtt_test", "Could not retrieve PDP context addresses");
    return;
  }

  /* Configure TLS profile */
  if(modem.tlsConfigProfile(TLS_PROFILE, WALTER_MODEM_TLS_VALIDATION_NONE, WALTER_MODEM_TLS_VERSION_12, 1)) {
    ESP_LOGI("mqtt_test", "Successfully configured the TLS profile");
  } else {
    ESP_LOGI("mqtt_test", "Failed to configure TLS profile");
  }

  // other public mqtt broker with web client: mqtthq.com
  if (modem.mqttConfig("walter-mqtt-test-topic", "", ""))

    if (modem.mqttConnect("test.mosquitto.org", 1883,true))
    {
      ESP_LOGI("mqtt_test", "MQTT connection succeeded");

      if (modem.mqttSubscribe("waltertopic"))
      {
        ESP_LOGI("mqtt_test", "MQTT subscribed to topic 'waltertopic'");
      } else {
      ESP_LOGI("mqtt_test", "MQTT subscribe failed");
      }
    } else {
      ESP_LOGI("mqtt_test", "MQTT connection failed");
    }
  else{
    ESP_LOGI("mqtt_test", "MQTT configuration failed");
  }

  /* this loop is basically the Arduino loop function */
  for(;;) {
    vTaskDelay(pdMS_TO_TICKS(15000));

    WalterModemRsp rsp = {};
  
    static int seq = 0;
    static char outgoingMsg[64];
    seq++;
    if(seq % 3 == 0) {
      sprintf(outgoingMsg, "%s-%d", macString, seq);

      if(modem.mqttPublish("waltertopic", (uint8_t *) outgoingMsg, strlen(outgoingMsg),2,&rsp)) {
        ESP_LOGI("mqtt_test", "published '%s' on topic 'waltertopic'", outgoingMsg);
      } else {
        
        ESP_LOGI("mqtt_test", "MQTT publish failed");
      }
    }

    while(modem.mqttDidRing("waltertopic", incomingBuf, sizeof(incomingBuf), &rsp)) {
      ESP_LOGI("mqtt_test", "incoming: qos=%d msgid=%d len=%d:",
          rsp.data.mqttResponse.qos,
          rsp.data.mqttResponse.messageId,
          rsp.data.mqttResponse.length);
      for(int i = 0; i < rsp.data.mqttResponse.length; i++) {
        ESP_LOGI("mqtt_test", "'%c' 0x%02x", incomingBuf[i], incomingBuf[i]);
      }
    }
  }
}
