/**
 * @file socket_test.cpp
 * @author Daan Pape <daan@dptechnics.com>
 * @date 24 Apr 2025
 * @copyright DPTechnics bv
 * @brief Walter Modem library examples
 *
 * @section LICENSE
 *
 * Copyright (C) 2025, DPTechnics bv
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
 * This example demonstrates how to configure the modem in Walter to send sample
 * data to the Walter Demo server on walterdemo.quickspot.io.
 */

#include <esp_mac.h>
#include <esp_log.h>
#include <driver/uart.h>
#include "WalterModem.h"
/**
 * @brief Cellular APN for SIM card. Leave empty to autodetect APN.
 */
CONFIG(CELLULAR_APN,const char*, "")



/**
 * @brief The address of the Walter Demo server.
 */
CONFIG(SERV_ADDR,const char* ,"walterdemo.quickspot.io")

/**
 * @brief The UDP port of the Walter Demo server.
 */
CONFIG_INT(SERV_PORT,1999)

/**
 * @brief The modem instance.
 */
WalterModem modem;

/**
 * @brief Response object containing command response information.
 */
WalterModemRsp rsp;

/**
 * @brief The buffer to transmit to the demo server. The first 6 bytes will be
 * the MAC address of the Walter this code is running on.
 */
uint8_t dataBuf[8] = { 0 };

/**
 * @brief Incrementing counter value sent as payload data.
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
  ESP_LOGI("socket_test", "Walter Socket Example v1.0.0");

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
    ESP_LOGI("socket_test", "Successfully initialized modem");
  } else {
    ESP_LOGE("socket_test", "Could not initialize modem");
    return;
  }

  /* Create PDP context */
  if(modem.definePDPContext(1, CELLULAR_APN)) {
    ESP_LOGI("socket_test", "Successfully defined PDP context");
  } else {
    ESP_LOGE("socket_test", "Could not define PDP context");
    return;
  }
  
  /* Set the operational state to full */
  if(modem.setOpState(WALTER_MODEM_OPSTATE_FULL)) {
    ESP_LOGI("socket_test", "Successfully set operational state to FULL");
  } else {
    ESP_LOGE("socket_test", "Could not set operational state to FULL");
    return;
  }
  /* Set the network operator selection to automatic */
  if(modem.setNetworkSelectionMode(WALTER_MODEM_NETWORK_SEL_MODE_AUTOMATIC)) {
    ESP_LOGI("socket_test", "Network selection mode set to automatic");
  } else {
    ESP_LOGE("socket_test", "Could not set the network selection mode to automatic");
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

  // /* Configure the socket */
  // if(modem.configSocket()) {
  //   ESP_LOGI("socket_test", "Successfully configured the socket");
  // } else {
  //   ESP_LOGI("socket_test", "Could not configure the socket");
  // }

  /* Connect to the demo server */
  if(modem.dialSocket(SERV_ADDR, SERV_PORT)) {
    ESP_LOGI("socket_test", "Connected to demo server %s:%d", SERV_ADDR, SERV_PORT);
  } else {
    ESP_LOGE("socket_test", "Could not connect demo socket");
    return;
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


