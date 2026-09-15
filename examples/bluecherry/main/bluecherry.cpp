/**
 * @file bluecherry.cpp
 * @author Jonas Maes <jonas@dptechnics.com>
 * @author Arnoud Devoogdt <arnoud@dptechnics.com>
 * @date 21 September 2026
 * @version 1.5.1
 * @copyright DPTechnics bv <info@dptechnics.com>
 * @brief Walter Modem library examples
 *
 * @section LICENSE
 *
 * Copyright (C) 2026, DPTechnics bv
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
 * This sketch sends and receives mqtt data using the DPTechnics BlueCherry cloud
 * platform. It also supports OTA updates which are scheduled through the BlueCherry web interface.
 */

#include <WalterModem.h>
#include <driver/uart.h>
#include <esp_sleep.h>
#include <esp_log.h>

// The cellular Access Point Name
// Leave blank for autodetection
#define CELLULAR_APN "soracom.io"

/**
 * @brief The modem instance.
 */
WalterModem modem;

/**
 * @brief The binary configuration settings for PSM.
 * These can be calculated using e.g.
 * https://www.soracom.io/psm-calculation-tool/
 */
const char* psmActive = "00000001";
const char* psmTAU = "00000110";

/**
 * @brief The binary configuration settings for eDRX.
 */
const char* edrxValue = "1101";
const char* edrxPagingTimeWindow = "0000";

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
  /* Configure power saving mode */
  if(modem.configPSM(WALTER_MODEM_PSM_ENABLE, psmTAU, psmActive)) {
    ESP_LOGI(TAG, "Successfully configured PSM");
  } else {
    ESP_LOGE(TAG, "Error: Could not configure PSM");
  }

  /* Configure eDRX */
  if(modem.configEDRX(WALTER_MODEM_EDRX_ENABLE_WITH_RESULT, edrxValue, edrxPagingTimeWindow)) {
    ESP_LOGI(TAG, "Successfully configured eDRX");
  } else {
    ESP_LOGE(TAG, "Error: Could not configure eDRX");
  }

  /* Set the operational state to NO RF */
  if(modem.setOpState(WALTER_MODEM_OPSTATE_NO_RF)) {
    ESP_LOGI(TAG, "Successfully set operational state to NO RF");
  } else {
    ESP_LOGE(TAG, "Could not set operational state to NO RF");
    return false;
  }

  /* Create PDP context */
  if(modem.definePDPContext(1, CELLULAR_APN)) {
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

/** * @brief The network registration event handler.
 *
 * This function will be called when network registration state changes or when
 * eDRX parameters are received from the network.
 *
 * @note Make sure to keep this handler as lightweight as possible to avoid blocking
 * the event processing task.
 *
 * @param[out] event The network registration state event.
 * @param[out] data The registration event data including state and PSM info.
 * @param[out] args User arguments.
 *
 * @return void
 */
static void myNetworkEventHandler(WMNetworkEventType event, const WMNetworkEventData* data,
                                  void* args)
{
  if(event == WALTER_MODEM_NETWORK_EVENT_REG_STATE_CHANGE) {
    switch(data->cereg.state) {
    case WALTER_MODEM_NETWORK_REG_REGISTERED_HOME:
      ESP_LOGI(TAG, "Network event: Network registration state changed: Registered (home)");
      if(data->cereg.hasPsmInfo) {
        ESP_LOGI(TAG, "PSM info received: Active Time: %s, TAU: %s", data->cereg.activeTime,
                 data->cereg.periodicTau);
      }
      break;

    case WALTER_MODEM_NETWORK_REG_REGISTERED_ROAMING:
      ESP_LOGI(TAG, "Network event: Network registration state changed: Registered (roaming)");
      if(data->cereg.hasPsmInfo) {
        ESP_LOGI(TAG, "PSM info received: Active Time: %s, TAU: %s", data->cereg.activeTime,
                 data->cereg.periodicTau);
      }
      break;

    case WALTER_MODEM_NETWORK_REG_NOT_SEARCHING:
      ESP_LOGI(TAG, "Network event: Network registration state changed: Not searching");
      break;

    case WALTER_MODEM_NETWORK_REG_SEARCHING:
      ESP_LOGI(TAG, "Network event: Network registration state changed: Searching");
      break;

    case WALTER_MODEM_NETWORK_REG_DENIED:
      ESP_LOGI(TAG, "Network event: Network registration state changed: Denied");
      break;

    case WALTER_MODEM_NETWORK_REG_UNKNOWN:
      ESP_LOGI(TAG, "Network event: Network registration state changed: Unknown");
      break;

    default:
      break;
    }
  } else if(event == WALTER_MODEM_NETWORK_EVENT_EDRX_RECEIVED) {
    ESP_LOGI(TAG, "Network event: eDRX received (ACT: %d) Requested: %s, NW-Provided: %s, PTW: %s",
             data->edrx.actType, data->edrx.requestedEdrx, data->edrx.nwProvidedEdrx,
             data->edrx.pagingTimeWindow);
  }
}

extern "C" void app_main(void)
{
  WalterModemRsp rsp = {};
  ESP_LOGI(TAG, "\r\n\r\n=== Walter BlueCherry example (IDF v1.5.1) ===\r\n\r\n");

  /* Start the modem */
  if(modem.begin(UART_NUM_1)) {
    ESP_LOGI(TAG, "Successfully initialized the modem");
  } else {
    ESP_LOGE(TAG, "Could not initialize the modem");
    return;
  }

  /* Register network event handler */
  modem.setNetworkEventHandler(myNetworkEventHandler, NULL);

  /* Connect to cellular network */
  if(!lteConnected() && !lteConnect()) {
    ESP_LOGE(TAG, "Unable to connect to cellular network, restarting Walter "
                  "in 10 seconds");
    vTaskDelay(pdMS_TO_TICKS(10000));
    esp_restart();
  }

  /* Enable temperature monitoring */
  if(modem.configTemperatureMonitor(WALTER_MODEM_TEMP_MONITOR_MODE_ON)) {
    ESP_LOGI(TAG, "Successfully enabled temperature monitoring");
  } else {
    ESP_LOGE(TAG, "Could not enable temperature monitoring");
  }

  /* Get temperature reading */
  int8_t temperature = 0;
  if(modem.getTemperature(&rsp)) {
    if(rsp.type == WALTER_MODEM_RSP_DATA_TYPE_TEMPERATURE) {
      temperature = rsp.data.temperature.temperature;
      ESP_LOGI(TAG, "Current temperature: %d°C (status: %d)", temperature,
               rsp.data.temperature.status);
    }
  } else {
    ESP_LOGE(TAG, "Could not get temperature reading");
  }

  /* Disable temperature monitoring */
  if(modem.configTemperatureMonitor(WALTER_MODEM_TEMP_MONITOR_MODE_OFF)) {
    ESP_LOGI(TAG, "Successfully disabled temperature monitoring");
  } else {
    ESP_LOGE(TAG, "Could not disable temperature monitoring");
  }

  /* Enable voltage monitoring */
  if(modem.configVoltageMonitor(WALTER_MODEM_VOLTAGE_MONITOR_MODE_ACTIVE)) {
    ESP_LOGI(TAG, "Successfully enabled voltage monitoring");
  } else {
    ESP_LOGE(TAG, "Could not enable voltage monitoring");
  }

  /* Get voltage reading */
  uint16_t voltage = 0;
  if(modem.getVoltage(&rsp)) {
    if(rsp.type == WALTER_MODEM_RSP_DATA_TYPE_VOLTAGE) {
      voltage = rsp.data.voltage.voltage;
      ESP_LOGI(TAG, "Current voltage: %dmV (status: %d)", voltage, rsp.data.voltage.status);
    }
  } else {
    ESP_LOGE(TAG, "Could not get voltage reading");
  }

  /* Disable voltage monitoring */
  if(modem.configVoltageMonitor(WALTER_MODEM_VOLTAGE_MONITOR_MODE_DISABLED)) {
    ESP_LOGI(TAG, "Successfully disabled voltage monitoring");
  } else {
    ESP_LOGE(TAG, "Could not disable voltage monitoring");
  }

  char msg[128];
  snprintf(msg, sizeof(msg),
           "{\"message\":\"Hello from Walter Modem!\",\"temperature\":%d,\"voltage\":%d}",
           temperature, voltage);
  ESP_LOGI(TAG, "%s", msg);

  ESP_LOGI(TAG, "I'm tired, I'm going to deep sleep now for 5 minutes...");
  modem.sleep(60 * 5);
}
