/**
 * @file tcp.cpp
 * @author Daan Pape <daan@dptechnics.com>
 * @author Arnoud Devoogdt <arnoud@dptechnics.com>
 * @date 16 January 2026
 * @version 1.5.0
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
 * This file contains a sketch which uses the modem in Walter to make a
 * connection to a network and upload data packets to the Walter demo server.
 */

#include <driver/temperature_sensor.h>
#include <WalterModem.h>
#include <driver/uart.h>
#include <esp_timer.h>
#include <esp_log.h>
#include <esp_mac.h>

#define TCP_PORT 1999
#define TCP_HOST "walterdemo.quickspot.io"

#define BASIC_INFO_PACKET_SIZE 24
#define COUNTER_PACKET_SIZE 8

/**
 * @brief The Socket profile to use (1..6)
 *
 * @note At least one socket should be available/reserved for BlueCherry.
 */
#define MODEM_SOCKET_ID 1

/**
 * @brief The modem instance.
 */
WalterModem modem;

/**
 * @brief Response object containing command response information.
 */
WalterModemRsp rsp = {};

/**
 * @brief The buffer to transmit to the TCP server. The first 6 bytes will be
 * the MAC address of the Walter this code is running on.
 */
uint8_t out_buf[8] = { 0 };

/**
 * @brief The buffer to receive from the TCP server.
 * @note Make sure this is sufficiently large enough for incoming data. (Up to 1500 bytes supported
 * by Sequans)
 */
uint8_t in_buf[1500] = { 0 };

/**
 * @brief The counter used in the ping packets.
 */
uint16_t counter = 0;

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
 * @brief The network registration event handler.
 *
 * You can use this handler to get notified of network registration state changes. For this example,
 * we use polling to get the network registration state. You can use this to implement your own
 * reconnection logic.
 *
 * @note Make sure to keep this handler as lightweight as possible to avoid blocking the event
 * processing task.
 *
 * @param[out] state The network registration state.
 * @param[out] args User arguments.
 *
 * @return void
 */
static void myNetworkEventHandler(WalterModemNetworkRegState state, void* args)
{
  switch(state) {
  case WALTER_MODEM_NETWORK_REG_REGISTERED_HOME:
    ESP_LOGI(TAG, "Network registration: Registered (home)");
    break;

  case WALTER_MODEM_NETWORK_REG_REGISTERED_ROAMING:
    ESP_LOGI(TAG, "Network registration: Registered (roaming)");
    break;

  case WALTER_MODEM_NETWORK_REG_NOT_SEARCHING:
    ESP_LOGI(TAG, "Network registration: Not searching");
    break;

  case WALTER_MODEM_NETWORK_REG_SEARCHING:
    ESP_LOGI(TAG, "Network registration: Searching");
    break;

  case WALTER_MODEM_NETWORK_REG_DENIED:
    ESP_LOGI(TAG, "Network registration: Denied");
    break;

  case WALTER_MODEM_NETWORK_REG_UNKNOWN:
    ESP_LOGI(TAG, "Network registration: Unknown");
    break;

  default:
    break;
  }
}

/**
 * @brief The Socket event handler.
 *
 * This function will be called on various socket events such as disconnection and incoming
 * messages. You can modify this handler to implement your own logic based on the events received.
 *
 * @note Make sure to keep this handler as lightweight as possible to avoid blocking the event
 * processing task.
 *
 * @param[out] event The type of socket event.
 * @param[out] data The data associated with the event.
 * @param[out] args User arguments.
 *
 * @return void
 */
static void mySocketEventHandler(WMSocketEventType event, const WMSocketEventData* data, void* args)
{
  switch(event) {
  case WALTER_MODEM_SOCKET_EVENT_DISCONNECTED:
    ESP_LOGI(TAG, "SOCKET: Disconnected (id %d)", data->conn_id);
    break;

  case WALTER_MODEM_SOCKET_EVENT_RING:
    ESP_LOGI(TAG, "SOCKET: Message received on socket %d (size: %u)", data->conn_id,
             data->data_len);

    /* Receive the HTTP message from the modem buffer */
    memset(in_buf, 0, sizeof(in_buf));
    if(modem.socketReceive(data->conn_id, in_buf, data->data_len)) {
      ESP_LOGI(TAG, "Received message on socket %d: %s", data->conn_id, in_buf);
    } else {
      ESP_LOGE(TAG, "Could not receive message for socket %d", data->conn_id);
    }
    break;

  default:
    break;
  }
}

float temperatureRead(void)
{
  float result = -9999.0f;

  temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 50);
  temperature_sensor_handle_t temp_sensor = NULL;
  temperature_sensor_install(&temp_sensor_config, &temp_sensor);
  temperature_sensor_enable(temp_sensor);
  temperature_sensor_get_celsius(temp_sensor, &result);
  temperature_sensor_disable(temp_sensor);
  temperature_sensor_uninstall(temp_sensor);

  return result;
}

/**
 * @brief Send a basic info packet to walterdemo
 */
bool tcpSendBasicInfoPacket()
{
  uint16_t packet_size = COUNTER_PACKET_SIZE;

  out_buf[6] = counter >> 8;
  out_buf[7] = counter & 0xFF;

  /* Only send the full packet if cellinfo is valid */
  if(rsp.data.cellInformation.cc != 0 || rsp.data.cellInformation.nc != 0 ||
     rsp.data.cellInformation.tac != 0 || rsp.data.cellInformation.cid != 0) {
    packet_size = BASIC_INFO_PACKET_SIZE;

    /* Read the temperature of Walter */
    float temp = temperatureRead();
    uint16_t rawTemp = (temp + 50) * 100;

    uint8_t rat = -1;
    if(modem.getRAT(&rsp)) {
      rat = (uint8_t) rsp.data.rat;
    }

    /* Construct the basic info packet */
    out_buf[8] = rawTemp >> 8;
    out_buf[9] = rawTemp & 0xFF;
    out_buf[10] = rsp.data.cellInformation.cc >> 8;
    out_buf[11] = rsp.data.cellInformation.cc & 0xFF;
    out_buf[12] = rsp.data.cellInformation.nc >> 8;
    out_buf[13] = rsp.data.cellInformation.nc & 0xFF;
    out_buf[14] = rsp.data.cellInformation.tac >> 8;
    out_buf[15] = rsp.data.cellInformation.tac & 0xFF;
    out_buf[16] = (rsp.data.cellInformation.cid >> 24) & 0xFF;
    out_buf[17] = (rsp.data.cellInformation.cid >> 16) & 0xFF;
    out_buf[18] = (rsp.data.cellInformation.cid >> 8) & 0xFF;
    out_buf[19] = rsp.data.cellInformation.cid & 0xFF;
    out_buf[20] = (uint8_t) (rsp.data.cellInformation.rsrp * -1);
    out_buf[21] = (uint8_t) (rsp.data.cellInformation.rsrq * -1);
    out_buf[22] = rat;
    out_buf[23] = 0xFF;
  }

  ESP_LOGI(TAG, "Sending packet...");

  if(!modem.socketSend(MODEM_SOCKET_ID, out_buf, packet_size)) {
    ESP_LOGE(TAG, "TCP send packet failed");
    return false;
  }

  vTaskDelay(pdMS_TO_TICKS(2000));

  /* Attempt to get the latest cell information (for next packet) */
  modem.getCellInformation(WALTER_MODEM_SQNMONI_REPORTS_SERVING_CELL, &rsp);

  ESP_LOGI(TAG, "TCP send basic packet succeeded");
  return true;
}

/**
 * @brief The main application start method.
 */
extern "C" void app_main()
{
  ESP_LOGI(TAG, "\r\n\r\n=== WalterModem TCP example (IDF v1.5.0) ===\r\n\r\n");

  /* Start the modem */
  if(modem.begin(UART_NUM_1)) {
    ESP_LOGI(TAG, "Successfully initialized the modem");
  } else {
    ESP_LOGE(TAG, "Could not initialize the modem");
    return;
  }

  /* Set the network registration event handler (optional) */
  modem.setRegistrationEventHandler(myNetworkEventHandler, NULL);

  /* Set the Socket event handler */
  modem.setSocketEventHandler(mySocketEventHandler, NULL);

  /* Retrieve and print the board MAC address */
  esp_read_mac(out_buf, ESP_MAC_WIFI_STA);
  ESP_LOGI(TAG, "Board MAC: %02X:%02X:%02X:%02X:%02X:%02X", out_buf[0], out_buf[1], out_buf[2],
           out_buf[3], out_buf[4], out_buf[5]);

  /* Configure a new socket */
  if(modem.socketConfig(MODEM_SOCKET_ID)) {
    ESP_LOGI(TAG, "Successfully configured a new socket");
  } else {
    ESP_LOGE(TAG, "Could not configure a new socket");
    return;
  }

  /* Disable TLS (the demo TCP server does not use it) */
  if(modem.socketConfigSecure(MODEM_SOCKET_ID, false)) {
    ESP_LOGI(TAG, "Successfully set socket to insecure mode");
  } else {
    ESP_LOGE(TAG, "Could not disable socket TLS");
    return;
  }

  while(true) {
    if(!lteConnected()) {
      if(!lteConnect()) {
        ESP_LOGE(TAG, "Failed to connect to network");
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
      }

      /* Connect (dial) the TCP test server */
      if(modem.socketDial(MODEM_SOCKET_ID, WALTER_MODEM_SOCKET_PROTO_TCP, TCP_PORT, TCP_HOST)) {
        ESP_LOGI(TAG, "Successfully connected Socket %u to TCP server %s:%d", MODEM_SOCKET_ID,
                 TCP_HOST, TCP_PORT);
      } else {
        ESP_LOGE(TAG, "Could not dial TCP server");
        return;
      }
    }

    if(!tcpSendBasicInfoPacket()) {
      ESP_LOGE(TAG, "TCP send failed, restarting...");
      vTaskDelay(pdMS_TO_TICKS(1000));
      esp_restart();
    }

    counter++;
    vTaskDelay(pdMS_TO_TICKS(15000));
    printf("\n");
  }
}