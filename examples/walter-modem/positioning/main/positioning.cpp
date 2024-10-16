/**
 * @file positioning.cpp
 * @author Jonas Maes <jonas@dptechnics.com>
 * @date 30 Nov 2023
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
 * This program connects to LTE to download GNSS assistance
 * data, gets a GNSS fix and uploads the position to the Walter demo server.
 */

#include <esp_mac.h>
#include <esp_log.h>
#include <inttypes.h>
#include <cstring>
#include <esp_system.h>
#include <driver/uart.h>
#include <driver/temp_sensor.h>
#include "WalterModem.h"

/**
 * @brief The address of the server to upload the data to. 
 */
#define SERV_ADDR "64.225.64.140"

/**
 * @brief The UDP port on which the server is listening.
 */
#define SERV_PORT 1999

/**
 * @brief The size in bytes of a minimal sensor + GNSS packet.
 */
#define PACKET_SIZE 18

/**
 * @brief All fixes with a confidence below this number are considered ok.
 */
#define MAX_GNSS_CONFIDENCE 100.0

/**
 * @brief The radio access technology to use - LTEM or NBIOT.
 */
#define RADIO_TECHNOLOGY WALTER_MODEM_RAT_LTEM

/**
 * @brief The modem instance.
 */
WalterModem modem;

/**
 * @brief Flag used to signal when a fix is received.
 */
volatile bool fixRcvd = false;

/**
 * @brief The last received GNSS fix.
 */
WalterModemGNSSFix posFix = {};

/**
 * @brief The buffer to transmit to the UDP server. The first 6 bytes will be
 * the MAC address of the Walter this code is running on.
 */
uint8_t dataBuf[PACKET_SIZE] = { 0 };

/**
 * @brief Configure the modem's network.
 * 
 * This function will set up the APN so that the modem can connect to
 * a network.
 * 
 * @param apn The APN to use for the PDP context.
 * @param user The APN username.
 * @param pass The APN password.
 * 
 * @return True on success, false on error.
 */
bool lteInit(const char *apn, const char *user = NULL, const char *pass = NULL)
{
  /* Create PDP context */
  if(user != NULL) {
    if(!modem.createPDPContext(
        apn,
        WALTER_MODEM_PDP_AUTH_PROTO_PAP,
        user,
        pass))
    {
      ESP_LOGI("positioning", "Could not create PDP context");
      return false;
    }
  } else {
    if(!modem.createPDPContext(apn)) {
      ESP_LOGI("positioning", "Could not create PDP context");
      return false;
    }
  }

  /* Authenticate the PDP context */
  if(!modem.authenticatePDPContext()) {
    ESP_LOGI("positioning", "Could not authenticate the PDP context");
    return false;
  }

  return true;
}

/**
 * @brief Connect to the LTE network.
 * 
 * This function will connect the modem to the LTE network. This function will
 * block until the modem is attached.
 * 
 * @return True on success, false on error.
 */
bool lteConnect()
{
  /* Set the operational state to full */
  if(!modem.setOpState(WALTER_MODEM_OPSTATE_FULL)) {
    ESP_LOGI("positioning", "Could not set operational state to FULL");
    return false;
  }

  /* Set the network operator selection to automatic */
  if(!modem.setNetworkSelectionMode(WALTER_MODEM_NETWORK_SEL_MODE_AUTOMATIC)) {
    ESP_LOGI("positioning", "Could not set the network selection mode to automatic");
    return false;
  }

  /* Wait for the network to become available */
  WalterModemNetworkRegState regState = modem.getNetworkRegState();
  while(!(regState == WALTER_MODEM_NETWORK_REG_REGISTERED_HOME ||
          regState == WALTER_MODEM_NETWORK_REG_REGISTERED_ROAMING))
  {
    vTaskDelay(pdMS_TO_TICKS(1000));
    WalterModemRsp rsp;
    modem.getRSSI(&rsp);
    ESP_LOGI("positioning", "rssi: %d", rsp.data.rssi);
    regState = modem.getNetworkRegState();
  }

  /* Stabilization time */
  ESP_LOGI("positioning", "Connected to the network");
  return true;
}

/**
 * @brief Disconnect from the LTE network.
 * 
 * This function will disconnect the modem from the LTE network and block until
 * the network is actually disconnected. After the network is disconnected the
 * GNSS subsystem can be used.
 * 
 * @return True on success, false on error.
 */
bool lteDisconnect()
{
  /* Set the operational state to minimum */
  if(!modem.setOpState(WALTER_MODEM_OPSTATE_MINIMUM)) {
    ESP_LOGI("positioning", "Could not set operational state to MINIMUM");
    return false;
  }

  /* Wait for the network to become available */
  WalterModemNetworkRegState regState = modem.getNetworkRegState();
  while(regState != WALTER_MODEM_NETWORK_REG_NOT_SEARCHING)
  {
    vTaskDelay(pdMS_TO_TICKS(100));
    regState = modem.getNetworkRegState();
  }

  ESP_LOGI("positioning", "Disconnected from the network");
  return true;
}

/**
 * @brief Check the assistance data in the modem response.
 * 
 * This function checks the availability of assistance data in the modem's 
 * response. This function also sets a flag if any of the assistance databases
 * should be updated.
 * 
 * @param rsp The modem response to check.
 * @param updateAlmanac Pointer to the flag to set when the almanac should be
 * updated. 
 * @param updateEphemeris Pointer to the flag to set when ephemeris should be
 * updated.
 * 
 * @return None.
 */
void checkAssistanceData(
  WalterModemRsp *rsp,
  bool *updateAlmanac = NULL,
  bool *updateEphemeris = NULL)
{
  if(updateAlmanac != NULL) {
    *updateAlmanac = false;
  }

  if(updateEphemeris != NULL) {
    *updateEphemeris = false;
  }

  ESP_LOGI("positioning", "Almanac data is ");
  if(rsp->data.gnssAssistance.almanac.available) {
    ESP_LOGI("positioning", "available and should be updated within %lds",
      rsp->data.gnssAssistance.almanac.timeToUpdate);
    if(updateAlmanac != NULL) {
      *updateAlmanac = rsp->data.gnssAssistance.almanac.timeToUpdate <= 0;
    }
  } else {
    ESP_LOGI("positioning", "not available.");
    if(updateAlmanac != NULL) {
      *updateAlmanac = true;
    }
  }

  ESP_LOGI("positioning", "Real-time ephemeris data is ");
  if(rsp->data.gnssAssistance.realtimeEphemeris.available) {
    ESP_LOGI("positioning", "available and should be updated within %lds",
      rsp->data.gnssAssistance.realtimeEphemeris.timeToUpdate);
    if(updateEphemeris != NULL) {
      *updateEphemeris = rsp->data.gnssAssistance.realtimeEphemeris.timeToUpdate <= 0;
    }
  } else {
    ESP_LOGI("positioning", "not available.");
    if(updateEphemeris != NULL) {
      *updateEphemeris = true;
    }
  }
}

/**
 * @brief This function will update GNSS assistance data when needed.
 * 
 * This funtion will check if the current real-time ephemeris data is good
 * enough to get a fast GNSS fix. If not the function will attach to the LTE 
 * network to download newer assistance data.
 * 
 * @return True on success, false on error.
 */
bool updateGNSSAssistance()
{
  bool lteConnected = false;
  WalterModemRsp rsp = {};

  lteDisconnect();

  /* Even with valid assistance data the system clock could be invalid */
  if(!modem.getClock(&rsp)) {
    ESP_LOGI("positioning", "Could not check the modem time");
    return false;
  }

  if(rsp.data.clock <= 0) {
    /* The system clock is invalid, connect to LTE network to sync time */
    if(!lteConnect()) {
      ESP_LOGI("positioning", "Could not connect to LTE network");
      return false;
    }

    lteConnected = true;

    /* 
     * Wait for the modem to synchronize time with the LTE network, try 5 times
     * with a delay of 500ms.
     */
    for(int i = 0; i < 5; ++i) {
      if(!modem.getClock(&rsp)) {
        ESP_LOGI("positioning", "Could not check the modem time");
        return false;
      }

      if(rsp.data.clock > 0) {
        ESP_LOGI("positioning", "Synchronized clock with network: %lld",
          rsp.data.clock);
        break;
      } else if(i == 4) {
        ESP_LOGI("positioning", "Could not sync time with network");
        return false;
      }

      vTaskDelay(pdMS_TO_TICKS(500));
    }
  }

  /* Check the availability of assistance data */
  if(!modem.getGNSSAssistanceStatus(&rsp) ||
     rsp.type != WALTER_MODEM_RSP_DATA_TYPE_GNSS_ASSISTANCE_DATA)
  {
    ESP_LOGI("positioning", "Could not request GNSS assistance status");
    return false;
  }

  bool updateAlmanac = false;
  bool updateEphemeris = false;
  checkAssistanceData(&rsp, &updateAlmanac, &updateEphemeris);

  if(!(updateAlmanac || updateEphemeris)) {
    if(lteConnected) {
      if(!lteDisconnect()) {
        ESP_LOGI("positioning", "Could not disconnect from the LTE network");
        return false;
      }
    }

    return true;
  }

  if(!lteConnected) {
    if(!lteConnect()) {
      ESP_LOGI("positioning", "Could not connect to LTE network");
      return false;
    }
  }

  if(updateAlmanac) {
    if(!modem.updateGNSSAssistance(WALTER_MODEM_GNSS_ASSISTANCE_TYPE_ALMANAC)) {
      ESP_LOGI("positioning", "Could not update almanac data");
      return false;
    }
  }

  if(updateEphemeris) {
    if(!modem.updateGNSSAssistance(
      WALTER_MODEM_GNSS_ASSISTANCE_TYPE_REALTIME_EPHEMERIS))
    {
      ESP_LOGI("positioning", "Could not update real-time ephemeris data");
      return false;
    }
  }

  if(!modem.getGNSSAssistanceStatus(&rsp) ||
    rsp.type != WALTER_MODEM_RSP_DATA_TYPE_GNSS_ASSISTANCE_DATA)
  {
    ESP_LOGI("positioning", "Could not request GNSS assistance status");
    return false;
  }

  checkAssistanceData(&rsp);
  
  if(!lteDisconnect()) {
    ESP_LOGI("positioning", "Could not disconnect from the LTE network");
    return false;
  }

  return true;
}

/**
 * @brief Connect to an UDP socket.
 * 
 * This function will set-up the modem and connect an UDP socket. The LTE 
 * connection must be active before this function can be called.
 * 
 * @param ip The IP address of the server to connect to.
 * @param port The port to connect to.
 * 
 * @return True on success, false on error.
 */
bool socketConnect(const char *ip, uint16_t port)
{
  WalterModemRsp rsp = {};

  /* Activate the PDP context */
  if(!modem.setPDPContextActive(true)) {
    ESP_LOGI("positioning", "Could not activate the PDP context");
    return false;
  }

  /* Attach the PDP context */
  if(!modem.attachPDPContext(true)) {
    ESP_LOGI("positioning", "Could not attach to the PDP context");
    return false;
  }

  /* Construct a socket */
  if(!modem.createSocket(&rsp)) {
    ESP_LOGI("positioning", "Could not create a new socket");
    return false;
  }

  /* Configure the socket */
  if(!modem.configSocket()) {
    ESP_LOGI("positioning", "Could not configure the socket");
    return false;
  }

  /* Connect to the UDP test server */
  if(modem.connectSocket(ip, port, port)) {
    ESP_LOGI("positioning", "Connected to UDP server %s:%d", ip, port);
  } else {
    ESP_LOGI("positioning", "Could not connect UDP socket");
    return false;
  }

  return true;
}

/**
 * @brief This function is called when a fix attempt finished.
 * 
 * This function is called by Walter's modem library as soon as a fix attempt
 * has finished. This function should be handled as an interrupt and should be
 * as short as possible as it is called within the modem data thread.
 * 
 * @param fix The fix data.
 * @param args Optional arguments, a NULL pointer in this case.
 * 
 * @return None.
 */
void fixHandler(const WalterModemGNSSFix *fix, void *args)
{
  memcpy(&posFix, fix, sizeof(WalterModemGNSSFix));
  fixRcvd = true;
}

float temperatureRead(void)
{
    float result = -9999;
    temp_sensor_config_t tsens = { .dac_offset = TSENS_DAC_L2, .clk_div = 6 };
    temp_sensor_set_config(tsens);
    temp_sensor_start();
    temp_sensor_read_celsius(&result); 
    temp_sensor_stop();

    return result;
}

extern "C" void app_main(void)
{
  ESP_LOGI("positioning", "Walter Positioning v0.0.1");

  /* Get the MAC address for board validation */
  esp_read_mac(dataBuf, ESP_MAC_WIFI_STA);
  ESP_LOGI("positioning", "Walter's MAC is: %02X:%02X:%02X:%02X:%02X:%02X",
    dataBuf[0],
    dataBuf[1],
    dataBuf[2],
    dataBuf[3],
    dataBuf[4],
    dataBuf[5]);

  /* Modem initialization */
  if(WalterModem::begin(UART_NUM_1)) {
    ESP_LOGI("positioning", "Modem initialization OK");
  } else {
    ESP_LOGI("positioning", "Modem initialization ERROR");
    return;
  }

  WalterModemRsp rsp = {};
  if(modem.getRAT(&rsp)) {
    if(rsp.data.rat != RADIO_TECHNOLOGY) {
      modem.setRAT(RADIO_TECHNOLOGY);
      ESP_LOGI("positioning", "Switched modem radio technology");
    }
  } else {
    ESP_LOGI("positioning", "Could not retrieve radio access technology");
  }

  if(lteInit("soracom.io", "sora", "sora")) {
    ESP_LOGI("positioning", "Initialized LTE parameters");
  } else {
    ESP_LOGI("positioning", "Could not initialize LTE network parameters");
    return;
  }

  if(!modem.setOpState(WALTER_MODEM_OPSTATE_MINIMUM)) {
    ESP_LOGI("positioning", "Could not set operational state to MINIMUM");
    return;
  }

  vTaskDelay(pdMS_TO_TICKS(500));

  if(!modem.configGNSS()) {
    ESP_LOGI("positioning", "Could not configure the GNSS subsystem");
    return;
  }

  modem.setGNSSfixHandler(fixHandler);

  /* this loop is basically the Arduino loop function */
  for(;;) {
    /* Check clock and assistance data, update if required */
    if(!updateGNSSAssistance()) {
      ESP_LOGI("positioning", "Could not update GNSS assistance data");
      return;
    }
  
    /* Try up to 5 times to get a good fix */
    for(int i = 0; i < 5; ++i) {
      fixRcvd = false;
      if(!modem.performGNSSAction()) {
        ESP_LOGI("positioning", "Could not request GNSS fix");
        return;
      }
      ESP_LOGI("positioning", "Started GNSS fix");
  
      int j = 0;
      while(!fixRcvd) {
          if (j >= 300) {
              ESP_LOGI("positioning", "Timed out while waiting for GNSS fix");
              return;
          }
          j++;
          vTaskDelay(pdMS_TO_TICKS(500));
      }
      
      if(posFix.estimatedConfidence <= MAX_GNSS_CONFIDENCE) {
        break;
      }
    }
  
    uint8_t abovedBTreshold = 0;
    for(int i = 0; i < posFix.satCount; ++i) {
      if(posFix.sats[i].signalStrength >= 30) {
        abovedBTreshold += 1;
      }
    }
  
    ESP_LOGI("positioning", "GNSS fix attempt finished:"
      "  Confidence: %.02f"
      "  Latitude: %.06f"
      "  Longitude: %.06f"
      "  Satcount: %d"
      "  Good sats: %d",
      posFix.estimatedConfidence,
      posFix.latitude,
      posFix.longitude,
      posFix.satCount,
      abovedBTreshold);
  
    /* Read the temperature of Walter */
    float temp = temperatureRead();
    ESP_LOGI("positioning", "The temperature of Walter is %.02f degrees Celsius", temp);
  
    float lat = posFix.latitude;
    float lon = posFix.longitude;
    
    if(posFix.estimatedConfidence > MAX_GNSS_CONFIDENCE) {
      posFix.satCount = 0xFF;
      lat = 0.0;
      lon = 0.0;
      ESP_LOGI("positioning", "Could not get a valid fix");
    }
  
    /* Construct the minimal sensor + GNSS */
    uint16_t rawTemp = (temp + 50) * 100;
    dataBuf[6] = 0x02;
    dataBuf[7] = rawTemp >> 8;
    dataBuf[8] = rawTemp & 0xFF;
    dataBuf[9] = posFix.satCount;
    memcpy(dataBuf + 10, &lat, 4);
    memcpy(dataBuf + 14, &lon, 4);
  
    /* Transmit the packet */
    if(!lteConnect()) {
      ESP_LOGI("positioning", "Could not connect to the LTE network");
      return;
    }
  
    if(!socketConnect(SERV_ADDR, SERV_PORT)) {
      ESP_LOGI("positioning", "Could not connect to UDP server socket");
      return;
    }
  
    if(!modem.socketSend(dataBuf, PACKET_SIZE)) {
      ESP_LOGI("positioning", "Could not transmit data");
      return;
    }
  
    vTaskDelay(pdMS_TO_TICKS(5000));
  
    if(!modem.closeSocket()) {
      ESP_LOGI("positioning", "Could not close the socket");
      return;
    }
  
    lteDisconnect();
  }
}
