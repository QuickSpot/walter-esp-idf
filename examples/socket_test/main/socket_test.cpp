/**
 * @file socket_test.cpp
 * @author Daan Pape <daan@dptechnics.com>
 * @date 24 Apr 2025
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
 * This program uses the modem in Walter to make a connection to a network
 * and upload counter values to the Walter demo server.
 */

#include <esp_mac.h>
#include <esp_log.h>
#include <driver/uart.h>
#include "WalterModem.h"

void registrationEvent(WalterModemNetworkRegState state, void* args) {
  if (state == WALTER_MODEM_NETWORK_REG_REGISTERED_HOME || state == WALTER_MODEM_NETWORK_REG_REGISTERED_ROAMING)
  {
    ESP_LOGE("event", "connection event: %i",state);
  }
}

/**
 * @brief The address of the server to upload the data to. 
 */
#define SERV_ADDR "64.225.64.140"

/**
 * @brief Cellular APN for SIM card. Leave empty to autodetect APN.
 */
#define CELLULAR_APN ""

/**
 * @brief The UDP port on which the server is listening.
 */
#define SERV_PORT 1999

/**
 * @brief The modem instance.
 */
WalterModem modem;

/**
 * @brief rsp
 */
WalterModemRsp rsp;

/**
 * @brief The buffer to transmit to the UDP server. The first 6 bytes will be
 * the MAC address of the Walter this code is running on.
 */
uint8_t dataBuf[8] = { 0 };

/**
 * @brief The counter used in the ping packets.
 */
uint16_t counter = 0;

void waitForNetwork()
{
  /* Wait for the network to become available */
  WalterModemNetworkRegState regState = modem.getNetworkRegState();
  while (!(regState == WALTER_MODEM_NETWORK_REG_REGISTERED_HOME ||
           regState == WALTER_MODEM_NETWORK_REG_REGISTERED_ROAMING))
  {
    vTaskDelay(pdMS_TO_TICKS(100));
    regState = modem.getNetworkRegState();
  }
  ESP_LOGI("socket_test", "Connected to the network");
}

extern "C" void app_main(void)
{
  ESP_LOGI("socket_test", "Walter modem test v0.0.1");

  /* Get the MAC address for board validation */
  esp_read_mac(dataBuf, ESP_MAC_WIFI_STA);
  ESP_LOGI("socket_test", "Walter's MAC is: %02X:%02X:%02X:%02X:%02X:%02X",
    dataBuf[0],
    dataBuf[1],
    dataBuf[2],
    dataBuf[3],
    dataBuf[4],
    dataBuf[5]);

  if(WalterModem::begin(UART_NUM_1)) {
    ESP_LOGI("socket_test", "Modem initialization OK");
  } else {
    ESP_LOGI("socket_test", "Modem initialization ERROR");
    return;
  }
  
  modem.setRegistrationEventHandler(registrationEvent);

  if(modem.setOpState(WALTER_MODEM_OPSTATE_NO_RF)) {
    ESP_LOGI("socket_test", "Successfully set operational state to NO RF");
  } else {
    ESP_LOGI("socket_test", "Could not set operational state to NO RF");
    return;
  }

  /* Give the modem time to detect the SIM */
  vTaskDelay(pdMS_TO_TICKS(2000));

  /* Create PDP context */
  if(modem.definePDPContext(1,CELLULAR_APN)) {
    ESP_LOGI("socket_test", "Created PDP context");
  } else {
    ESP_LOGI("socket_test", "Could not create PDP context");
    return;
  }
  
  /* Set the operational state to full */
  if(modem.setOpState(WALTER_MODEM_OPSTATE_FULL)) {
    ESP_LOGI("socket_test", "Successfully set operational state to FULL");
  } else {
    ESP_LOGI("socket_test", "Could not set operational state to FULL");
    return;
  }
  /* Set the network operator selection to automatic */
  if(modem.setNetworkSelectionMode(WALTER_MODEM_NETWORK_SEL_MODE_AUTOMATIC)) {
    ESP_LOGI("socket_test", "Network selection mode to was set to automatic");
  } else {
    ESP_LOGI("socket_test", "Could not set the network selection mode to automatic");
    return;
  }
  /* Wait for the network to become available */
  waitForNetwork();

  /* Construct a socket */
  if(modem.createSocket(&rsp)) {
    ESP_LOGI("socket_test", "Created a new socket");
  } else {
    ESP_LOGI("socket_test", "Could not create a new socket");
  }

  /* Configure the socket */
  if(modem.configSocket()) {
    ESP_LOGI("socket_test", "Successfully configured the socket");
  } else {
    ESP_LOGI("socket_test", "Could not configure the socket");
  }

  /* Connect to the UDP test server */
  if(modem.connectSocket(SERV_ADDR, SERV_PORT, SERV_PORT)) {
    ESP_LOGI("socket_test", "Connected to UDP server %s:%d", SERV_ADDR, SERV_PORT);
  } else {
    ESP_LOGI("socket_test", "Could not connect UDP socket");
  }

  /* this loop is basically the Arduino loop function */
  for(;;) {
    dataBuf[6] = counter >> 8;
    dataBuf[7] = counter & 0xFF;
  
    if(modem.socketSend(dataBuf, 8)) {
      ESP_LOGI("socket_test", "Transmitted counter value %d", counter);
      counter += 1;
    } else {
      ESP_LOGI("socket_test", "Could not transmit data");
      vTaskDelay(pdMS_TO_TICKS(1000));
      esp_restart();
    }
  
    vTaskDelay(pdMS_TO_TICKS(10000));
  }
}


