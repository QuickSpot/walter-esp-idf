/**
 * @file http.cpp
 * @author Arnoud Devoogdt <arnoud@dptechnics.com>
 * @date 11 September 2025
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
 * This file contains a sketch which uses the modem in Walter to make a
 * HTTP GET/POST request and show the result.
 */

#include <driver/temperature_sensor.h>
#include <WalterModem.h>
#include <driver/uart.h>
#include <esp_timer.h>
#include <esp_log.h>
#include <esp_mac.h>

#define HTTP_PORT 80
#define HTTP_HOST "quickspot.io"
#define HTTP_GET_ENDPOINT "/hello/get"
#define HTTP_POST_ENDPOINT "/hello/post"

/**
 * @brief HTTP profile
 */
#define MODEM_HTTP_PROFILE 1

/**
 * @brief The modem instance.
 */
WalterModem modem;

/**
 * @brief Response object containing command response information.
 */
WalterModemRsp rsp = {};

/**
 * @brief Buffer for incoming HTTP response
 */
uint8_t incomingBuf[1024] = { 0 };

/**
 * @brief ESP-IDF log prefix.
 */
static constexpr const char* TAG = "[EXAMPLE]";

/**
 * @brief This function checks if we are connected to the LTE network
 *
 * @return true when connected, false otherwise
 */
bool lteConnected()
{
  WalterModemNetworkRegState regState = modem.getNetworkRegState();
  return (regState == WALTER_MODEM_NETWORK_REG_REGISTERED_HOME ||
          regState == WALTER_MODEM_NETWORK_REG_REGISTERED_ROAMING);
}

/**
 * @brief This function waits for the modem to be connected to the LTE network.
 *
 * @param timeout_sec The amount of seconds to wait before returning a time-out.
 *
 * @return true if connected, false on time-out.
 */
bool waitForNetwork(int timeout_sec = 300)
{
  ESP_LOGI(TAG, "Connecting to the network...");
  int time = 0;
  while(!lteConnected()) {
    printf(".");
    vTaskDelay(pdMS_TO_TICKS(1000));
    time++;
    if(time > timeout_sec)
      return false;
  }
  printf("\r\n");
  ESP_LOGI(TAG, "Connected to the network");
  return true;
}

/**
 * @brief Disconnect from the LTE network.
 *
 * This function will disconnect the modem from the LTE network and block until
 * the network is actually disconnected. After the network is disconnected the
 * GNSS subsystem can be used.
 *
 * @return true on success, false on error.
 */
bool lteDisconnect()
{
  /* Set the operational state to minimum */
  if(modem.setOpState(WALTER_MODEM_OPSTATE_MINIMUM)) {
    ESP_LOGI(TAG, "Successfully set operational state to MINIMUM");
  } else {
    ESP_LOGE(TAG, "Could not set operational state to MINIMUM");
    return false;
  }

  /* Wait for the network to become available */
  WalterModemNetworkRegState regState = modem.getNetworkRegState();
  while(regState != WALTER_MODEM_NETWORK_REG_NOT_SEARCHING) {
    vTaskDelay(pdMS_TO_TICKS(100));
    regState = modem.getNetworkRegState();
  }

  ESP_LOGI(TAG, "Disconnected from the network");
  return true;
}

/**
 * @brief This function tries to connect the modem to the cellular network.
 *
 * @return true on success, false on error.
 */
bool lteConnect()
{
  /* Set the operational state to NO RF */
  if(modem.setOpState(WALTER_MODEM_OPSTATE_NO_RF)) {
    ESP_LOGI(TAG, "Successfully set operational state to NO RF");
  } else {
    ESP_LOGE(TAG, "Could not set operational state to NO RF");
    return false;
  }

  /* Create PDP context */
  if(modem.definePDPContext()) {
    ESP_LOGI(TAG, "Created PDP context");
  } else {
    ESP_LOGE(TAG, "Could not create PDP context");
    return false;
  }

  /* Set the operational state to full */
  if(modem.setOpState(WALTER_MODEM_OPSTATE_FULL)) {
    ESP_LOGI(TAG, "Successfully set operational state to FULL");
  } else {
    ESP_LOGE(TAG, "Could not set operational state to FULL");
    return false;
  }

  /* Set the network operator selection to automatic */
  if(modem.setNetworkSelectionMode(WALTER_MODEM_NETWORK_SEL_MODE_AUTOMATIC)) {
    ESP_LOGI(TAG, "Network selection mode was set to automatic");
  } else {
    ESP_LOGE(TAG, "Could not set the network selection mode to automatic");
    return false;
  }

  return waitForNetwork();
}

/**
 * @brief Common routine to wait for and print an HTTP response.
 */
static bool waitForHttpResponse(uint8_t profile, const char* contentType)
{
  ESP_LOGI(TAG, "Waiting for reply...");
  const uint16_t maxPolls = 30;
  for(uint16_t i = 0; i < maxPolls; i++) {
    printf(".");
    if(modem.httpDidRing(profile, incomingBuf, sizeof(incomingBuf), &rsp)) {
      printf("\r\n");
      ESP_LOGI(TAG, "HTTP status code (Modem): %d\r\n", rsp.data.httpResponse.httpStatus);
      ESP_LOGI(TAG, "Content type: %s\r\n", contentType);
      ESP_LOGI(TAG, "Payload:\r\n%s\r\n", incomingBuf);
      return true;
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
  printf("\r\n");
  ESP_LOGE(TAG, "HTTP response timeout");
  return false;
}

/**
 * @brief Perform an HTTP GET request.
 */
bool httpGet(const char* path)
{
  char ctBuf[32] = { 0 };

  ESP_LOGI(TAG, "Sending HTTP GET to %s%s\r\n", HTTP_HOST, path);
  if(!modem.httpQuery(MODEM_HTTP_PROFILE, path, WALTER_MODEM_HTTP_QUERY_CMD_GET, ctBuf,
                      sizeof(ctBuf))) {
    ESP_LOGE(TAG, "HTTP GET query failed");
    return false;
  }
  ESP_LOGI(TAG, "HTTP GET successfully sent");
  return waitForHttpResponse(MODEM_HTTP_PROFILE, ctBuf);
}

/**
 * @brief Perform an HTTP POST request with a body.
 */
bool httpPost(const char* path, const uint8_t* body, size_t bodyLen,
              const char* mimeType = "application/json")
{
  char ctBuf[32] = { 0 };

  ESP_LOGI(TAG, "Sending HTTP POST to %s%s (%u bytes, type %s)\r\n", HTTP_HOST, path,
           (unsigned) bodyLen, mimeType);
  if(!modem.httpSend(MODEM_HTTP_PROFILE, path, (uint8_t*) body, (uint16_t) bodyLen,
                     WALTER_MODEM_HTTP_SEND_CMD_POST, WALTER_MODEM_HTTP_POST_PARAM_JSON, ctBuf,
                     sizeof(ctBuf))) {
    ESP_LOGE(TAG, "HTTP POST failed");
    return false;
  }
  ESP_LOGI(TAG, "HTTP POST successfully sent");
  return waitForHttpResponse(MODEM_HTTP_PROFILE, ctBuf);
}

/**
 * @brief The main application start method.
 */
extern "C" void app_main()
{
  ESP_LOGI(TAG, "=== WalterModem HTTP example ===");

  /* Start the modem */
  if(WalterModem::begin(UART_NUM_1)) {
    ESP_LOGI(TAG, "Successfully initialized the modem");
  } else {
    ESP_LOGE(TAG, "Could not initialize the modem");
    return;
  }

  /* Connect the modem to the LTE network */
  if(!lteConnect()) {
    ESP_LOGE(TAG, "Could not connect to LTE");
    return;
  }

  /* Configure the HTTP profile */
  if(modem.httpConfigProfile(MODEM_HTTP_PROFILE, HTTP_HOST, HTTP_PORT)) {
    ESP_LOGI(TAG, "Successfully configured the HTTP profile");
  } else {
    ESP_LOGE(TAG, "Failed to configure HTTP profile");
  }

  while(true) {
    static int64_t lastRequest = 0;
    const int64_t requestInterval = 30000 * 1000;

    int64_t now = esp_timer_get_time();
    if((now - lastRequest) >= requestInterval) {
      lastRequest = now;

      // Example GET
      if(!httpGet(HTTP_GET_ENDPOINT)) {
        ESP_LOGE(TAG, "HTTP GET failed, restarting...");
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
      }

      vTaskDelay(pdMS_TO_TICKS(2000));

      // Example POST
      const char jsonBody[] = "{\"hello\":\"walter\"}";
      if(!httpPost(HTTP_POST_ENDPOINT, (const uint8_t*) jsonBody, strlen(jsonBody),
                   "application/json")) {
        ESP_LOGE(TAG, "HTTP POST failed, restarting...");
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
      }
    }
  }
}