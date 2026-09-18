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

#include <WalterBlueCherry.h>
#include <driver/uart.h>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_sleep.h>

// The cellular Access Point Name
// Leave blank for autodetection
#define CELLULAR_APN "soracom.io"

// The BlueCherry device type this firmware belongs to, used for Zero-Touch Provisioning
#define BC_DEVICE_TYPE "walter01"

// The modem TLS profile BlueCherry may use
#define BC_TLS_PROFILE 1

// The size of the buffer holding messages waiting to be published
#define BC_PUBLISH_BUFFER_SIZE 4096

/**
 * @brief The modem instance.
 */
WalterModem modem;

/**
 * @brief The BlueCherry cloud client.
 */
WalterBlueCherry bc;

/**
 * @brief Buffer to stage incoming firmware in, sized for a modem update's whole erase block.
 */
uint8_t ota_buffer[SPI_FLASH_BLOCK_SIZE] = { 0 };

/**
 * @brief Flag used to signal when BlueCherry has nothing left to do.
 */
volatile bool bc_synchronized = false;

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

/**
 * @brief Handle a message the cloud sent down.
 *
 * Runs on the BlueCherry synchronisation task, so it must not block. The payload is a pointer into
 * the receive buffer, so copy anything that has to outlive the call.
 *
 * @param topic The single byte topic index the cloud maps to an MQTT topic.
 * @param len The number of bytes in data.
 * @param data The payload, valid only for the duration of the call.
 * @param args User arguments.
 *
 * @return void
 */
void myMessageHandler(uint8_t topic, uint16_t len, const uint8_t* data, void* args)
{
  ESP_LOGI(TAG, "Incoming message on topic 0x%02x (%u bytes)", topic, len);
  ESP_LOGI(TAG, "%.*s", len, (const char*) data);
}

/**
 * @brief Handle a firmware update event, taking both update decisions in the application.
 *
 * Runs on the BlueCherry synchronisation task, so it must not block.
 *
 * Two events carry a decision: AVAILABLE, where otaStart accepts the offer, and COMPLETE, where
 * the image is installed and only the restart is left. Returning false hands either back to the
 * library, which then downloads and restarts on its own. The other three are notifications.
 *
 * @param event The event that occurred.
 * @param info Details for the event, valid only for the duration of the call.
 * @param args User arguments.
 *
 * @return True when this handler took the event's decision.
 */
bool myOtaHandler(WalterModemBlueCherryOtaEvent event, const WalterModemBlueCherryOtaInfo* info,
                  void* args)
{
  switch(event) {
  case BLUECHERRY_OTA_EVENT_AVAILABLE:
    ESP_LOGI(TAG, "OTA: firmware v%d available (%lu bytes), accepting", info->version,
             (unsigned long) info->size);
    bc.otaStart();
    return true;

  case BLUECHERRY_OTA_EVENT_STARTED:
    ESP_LOGI(TAG, "OTA: downloading firmware v%d", info->version);
    break;

  case BLUECHERRY_OTA_EVENT_PROGRESS:
    ESP_LOGI(TAG, "OTA: %lu / %lu bytes", (unsigned long) info->bytes_received,
             (unsigned long) info->size);
    break;

  case BLUECHERRY_OTA_EVENT_COMPLETE:
    ESP_LOGI(TAG, "OTA: firmware v%d installed, restarting", info->version);
    esp_restart();
    return true;

  case BLUECHERRY_OTA_EVENT_FAILED:
    ESP_LOGE(TAG, "OTA: firmware v%d failed with error %u", info->version, info->error_code);
    break;
  }

  return false;
}

/**
 * @brief Report what the BlueCherry connection is doing.
 *
 * Runs on the BlueCherry synchronisation task, so it must not block. Publish and sync report the
 * transitions they make themselves, so it can also run on the calling task. IDLE is the only state
 * in which it is safe to sleep.
 *
 * @param state The state that was just entered.
 * @param args User arguments.
 *
 * @return void
 */
void myStateHandler(WalterModemBlueCherryState state, void* args)
{
  switch(state) {
  case BLUECHERRY_STATE_NOT_PROVISIONED:
    ESP_LOGI(TAG, "BlueCherry holds no credentials yet, provisioning...");
    break;

  case BLUECHERRY_STATE_AWAIT_CONNECTION:
    ESP_LOGI(TAG, "BlueCherry is connecting...");
    break;

  case BLUECHERRY_STATE_IDLE:
    ESP_LOGI(TAG, "Synchronized with the BlueCherry cloud platform");
    bc_synchronized = true;
    break;

  default:
    break;
  }
}

/**
 * @brief Give BlueCherry its buffers and handlers.
 *
 * Runs on every boot: it resumes a session that survived deep sleep and re-registers the handlers
 * and buffers, which do not survive one. Talks to the modem, never to the cloud, so it needs no
 * network and there is nothing to retry.
 *
 * @return True when BlueCherry is ready to be published to.
 */
static bool initializeBlueCherry(void)
{
  /* PSRAM, falling back to a library-allocated one in internal RAM. This descriptor is read during
   * init and not kept, so only the buffer it points at has to outlive the call. */
  WalterModemBlueCherryPublishBuffer publishBuffer = {};
  publishBuffer.buffer = (uint8_t*) heap_caps_malloc(BC_PUBLISH_BUFFER_SIZE, MALLOC_CAP_SPIRAM);
  publishBuffer.size = BC_PUBLISH_BUFFER_SIZE;

  if(publishBuffer.buffer == NULL) {
    ESP_LOGW(TAG, "No PSRAM available, letting BlueCherry allocate its own publish buffer");
    publishBuffer.size = 0;
  }

  if(!bc.init(BC_TLS_PROFILE, ota_buffer, BC_DEVICE_TYPE, myMessageHandler, NULL,
              publishBuffer.buffer != NULL ? &publishBuffer : NULL)) {
    ESP_LOGE(TAG, "Could not initialize BlueCherry");
    return false;
  }

  /* Both optional: without them the library takes the update decisions itself, and bc.getState()
   * answers what the state handler reports. */
  bc.setOtaHandler(myOtaHandler, NULL);
  bc.setStateHandler(myStateHandler, NULL);

  ESP_LOGI(TAG, "Successfully initialized BlueCherry");
  return true;
}

extern "C" void app_main(void)
{
  WalterModemRsp rsp = {};
  ESP_LOGI(TAG, "\r\n\r\n=== Walter BlueCherry example (IDF v1.5.1) ===\r\n\r\n");

  /* 1. Start the modem. */
  if(modem.begin(UART_NUM_1)) {
    ESP_LOGI(TAG, "Successfully initialized the modem");
  } else {
    ESP_LOGE(TAG, "Could not initialize the modem");
    return;
  }

  /* 2. Register the network event handler, before anything can change the registration state. */
  modem.setNetworkEventHandler(myNetworkEventHandler, NULL);

  /* 3. Hand BlueCherry its buffers and handlers. Nothing here needs the network. */
  if(!initializeBlueCherry()) {
    return;
  }

  /* 4. Read the sensors. Both monitors are modem-local, so they need no network either. */
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

  /* 5. Queue a message. Never touches the network; it goes out on the next synchronisation. */
  char msg[128];
  snprintf(msg, sizeof(msg),
           "{\"message\":\"Hello from Walter Modem!\",\"temperature\":%d,\"voltage\":%d}",
           temperature, voltage);
  ESP_LOGI(TAG, "Publishing to BlueCherry: %s", msg);
  bc.publish(0x84, strlen(msg), (const uint8_t*) msg);

  /* 6. Connect to the cellular network. Everything above this line is local to the board, so the
   * radio is only asked for once there is something to send. */
  if(!lteConnected() && !lteConnect()) {
    ESP_LOGE(TAG, "Unable to connect to cellular network, restarting Walter "
                  "in 10 seconds");
    vTaskDelay(pdMS_TO_TICKS(10000));
    esp_restart();
  }

  /* 7. Synchronise. A sleepy device drives this itself rather than with setAutoSync, so it decides
   * how many exchanges run before it sleeps. The flag is cleared here and not next to the publish:
   * an idle reported before this line answers an exchange the task ran on its own. */
  bc_synchronized = false;
  bc.sync();

  /* No time-out on purpose: an unsettled exchange is still retrying, or still pulling a firmware
   * update down, and sleeping through either costs the transfer its progress. */
  while(!bc_synchronized) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  /* The modem stays powered, so the session survives and the next boot resumes it. */
  ESP_LOGI(TAG, "I'm tired, I'm going to deep sleep now for 5 minutes...");
  modem.sleep(30);
}
