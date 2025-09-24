/**
 * @file mqtts.cpp
 * @author Arnoud Devoogdt <arnoud@dptechnics.com>
 * @date 24 Sept 2025
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
 * This file contains a sketch which uses the modem in Walter to subscribe and
 * publish data to an MQTTS broker.
 */

#include <driver/temperature_sensor.h>
#include <WalterModem.h>
#include <driver/uart.h>
#include <esp_timer.h>
#include <esp_log.h>
#include <esp_mac.h>

#define MQTTS_PORT 8883
#define MQTTS_HOST "broker.emqx.io"
#define MQTTS_TOPIC "walter-tls-test-topic"
#define MQTTS_CLIENT_ID "walter-client"
#define MQTTS_USERNAME ""
#define MQTTS_PASSWORD ""

/**
 * @brief Root CA certificate in PEM format.
 *
 * @note Example: https://www.emqx.com/en/mqtt/public-mqtt5-broker
 *
 * Used to validate the server's TLS certificate.
 */
static const char ca_cert[] = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDrzCCApegAwIBAgIQCDvgVpBCRrGhdWrJWZHHSjANBgkqhkiG9w0BAQUFADBh
MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3
d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBD
QTAeFw0wNjExMTAwMDAwMDBaFw0zMTExMTAwMDAwMDBaMGExCzAJBgNVBAYTAlVT
MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j
b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IENBMIIBIjANBgkqhkiG
9w0BAQEFAAOCAQ8AMIIBCgKCAQEA4jvhEXLeqKTTo1eqUKKPC3eQyaKl7hLOllsB
CSDMAZOnTjC3U/dDxGkAV53ijSLdhwZAAIEJzs4bg7/fzTtxRuLWZscFs3YnFo97
nh6Vfe63SKMI2tavegw5BmV/Sl0fvBf4q77uKNd0f3p4mVmFaG5cIzJLv07A6Fpt
43C/dxC//AH2hdmoRBBYMql1GNXRor5H4idq9Joz+EkIYIvUX7Q6hL+hqkpMfT7P
T19sdl6gSzeRntwi5m3OFBqOasv+zbMUZBfHWymeMr/y7vrTC0LUq7dBMtoM1O/4
gdW7jVg/tRvoSSiicNoxBN33shbyTApOB6jtSj1etX+jkMOvJwIDAQABo2MwYTAO
BgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4EFgQUA95QNVbR
TLtm8KPiGxvDl7I90VUwHwYDVR0jBBgwFoAUA95QNVbRTLtm8KPiGxvDl7I90VUw
DQYJKoZIhvcNAQEFBQADggEBAMucN6pIExIK+t1EnE9SsPTfrgT1eXkIoyQY/Esr
hMAtudXH/vTBH1jLuG2cenTnmCmrEbXjcKChzUyImZOMkXDiqw8cvpOp/2PV5Adg
06O/nVsJ8dWO41P0jmP6P6fbtGbfYmbW0W5BjfIttep3Sp+dWOIrWcBAI+0tKIJF
PnlUkiaY4IBIqDfv8NZ5YBberOgOzW6sRBc4L0na4UU+Krk2U886UAb3LujEV0ls
YSEY1QSteDwsOoBrp+uvFRTp2InBuThs4pFsiv9kuXclVzDAGySj4dzp30d8tbQk
CAUw7C29C79Fv1C5qfPrmAESrciIxpg0X40KPMbp1ZWVbd4=
-----END CERTIFICATE-----
)EOF";

/**
 * The TLS profile to use for the application (1 is reserved for BlueCherry)
 */
#define MQTTS_TLS_PROFILE 2

/**
 * @brief The modem instance.
 */
WalterModem modem;

/**
 * @brief Response object containing command response information.
 */
WalterModemRsp rsp = {};

/**
 * @brief Buffer for incoming response
 */
uint8_t incomingBuf[256] = { 0 };

/**
 * @brief MQTTS client and message prefix based on mac address
 */
char macString[32];

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
 * @brief Writes TLS credentials to the modem's NVS and configures the TLS profile.
 *
 * This function stores the provided TLS certificates and private keys into the modem's
 * non-volatile storage (NVS), and then sets up a TLS profile for secure communication.
 * These configuration changes are persistent across reboots.
 *
 * @note
 * - Certificate indexes 0–10 are reserved for Sequans and BlueCherry internal usage.
 * - Private key index 1 is reserved for BlueCherry internal usage.
 * - Do not attempt to override or use these reserved indexes.
 *
 * @return
 * - true if the credentials were successfully written and the profile configured.
 * - false otherwise.
 */
bool setupTLSProfile(void)
{

  if(!modem.tlsWriteCredential(false, 12, ca_cert)) {
    ESP_LOGE(TAG, "CA cert upload failed");
    return false;
  }

  if(modem.tlsConfigProfile(MQTTS_TLS_PROFILE, WALTER_MODEM_TLS_VALIDATION_CA,
                            WALTER_MODEM_TLS_VERSION_12, 12)) {
    ESP_LOGI(TAG, "TLS profile configured");
  } else {
    ESP_LOGE(TAG, "TLS profile configuration failed");
    return false;
  }

  return true;
}

/**
 * @brief Common routine to publish a message to an MQTTS topic.
 */
static bool mqttPublishMessage(const char* topic, const char* message)
{
  ESP_LOGI(TAG, "Publishing to topic '%s': %s\r\n", topic, message);
  if(modem.mqttPublish(topic, (uint8_t*) message, strlen(message))) {
    ESP_LOGI(TAG, "MQTTS publish succeeded");
    return true;
  }
  ESP_LOGE(TAG, "MQTTS publish failed");
  return false;
}

/**
 * @brief Common routine to check for and print incoming MQTTS messages.
 */
static void mqttCheckIncoming(const char* topic)
{
  while(modem.mqttDidRing(topic, incomingBuf, sizeof(incomingBuf), &rsp)) {
    ESP_LOGI(TAG, "Incoming MQTTS message on '%s'\r\n", topic);
    ESP_LOGI(TAG, "  QoS: %d, Message ID: %d, Length: %d\r\n", rsp.data.mqttResponse.qos,
             rsp.data.mqttResponse.messageId, rsp.data.mqttResponse.length);
    ESP_LOGI(TAG, "  Payload:");
    for(int i = 0; i < rsp.data.mqttResponse.length; i++) {
      ESP_LOGI(TAG, "  '%c' 0x%02X\r\n", incomingBuf[i], incomingBuf[i]);
    }
  }
}

/**
 * @brief The main application start method.
 */
extern "C" void app_main()
{
  ESP_LOGI(TAG, "=== WalterModem MQTTS example ===");

  /* Build a unique client ID from the ESP MAC address */
  esp_read_mac(incomingBuf, ESP_MAC_WIFI_STA);
  sprintf(macString, "walter%02X:%02X:%02X:%02X:%02X:%02X", incomingBuf[0], incomingBuf[1],
          incomingBuf[2], incomingBuf[3], incomingBuf[4], incomingBuf[5]);

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

  /* Set up the TLS profile */
  if(setupTLSProfile()) {
    ESP_LOGI(TAG, "TLS Profile setup succeeded");
  } else {
    ESP_LOGE(TAG, "TLS Profile setup failed");
    return;
  }

  /* Configure the MQTTS client */
  if(modem.mqttConfig(MQTTS_CLIENT_ID, MQTTS_USERNAME, MQTTS_PASSWORD, MQTTS_TLS_PROFILE)) {
    ESP_LOGI(TAG, "Successfully configured the MQTTS client");
  } else {
    ESP_LOGE(TAG, "Failed to configure MQTTS client");
    return;
  }

  /* Connect to a public MQTTS broker */
  if(modem.mqttConnect(MQTTS_HOST, MQTTS_PORT)) {
    ESP_LOGI(TAG, "Successfully connected to MQTTS broker");
  } else {
    ESP_LOGE(TAG, "Failed to connect to MQTTS broker");
    return;
  }

  /* Subscribe to the test topic */
  if(modem.mqttSubscribe(MQTTS_TOPIC)) {
    ESP_LOGI(TAG, "Successfully subscribed to '%s'", MQTTS_TOPIC);
  } else {
    ESP_LOGE(TAG, "MQTTS subscribe failed");
  }

  while(true) {
    static int64_t lastPublish = 0;
    const int64_t publishInterval = 10000 * 1000;

    static int seq = 0;
    static char outgoingMsg[64];

    int64_t now = esp_timer_get_time();
    if((now - lastPublish) >= publishInterval) {
      lastPublish = now;
      seq++;

      if(seq % 3 == 0) {
        sprintf(outgoingMsg, "%s-%d", macString, seq);
        if(!mqttPublishMessage(MQTTS_TOPIC, outgoingMsg)) {
          ESP_LOGE(TAG, "MQTTS publish failed, restarting...");
          vTaskDelay(pdMS_TO_TICKS(1000));
          esp_restart();
        }
        printf("\r\n");
      }

      /* Check for incoming messages */
      mqttCheckIncoming(MQTTS_TOPIC);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}