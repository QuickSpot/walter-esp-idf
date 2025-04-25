/**
 * @file mqtt_example.cpp
 * @author Jonas Maes <jonas@dptechnics.com>
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
 * This program publishes data to an MQTT broker and listens to the same topic for incoming
 * messages.
 */

#include "WalterModem.h"

#include <cstring>
#include <driver/uart.h>
#include <esp_log.h>
#include <esp_mac.h>

/**
 * @brief Cellular APN for SIM card. Leave empty to autodetect APN.
 */
CONFIG(CELLULAR_APN, const char *, "")

/**
 * @brief Time delay in ms of data sent to the Walter demo server.
 */
CONFIG_UINT16(SEND_DELAY_MS, 10000)

/**
 * @brief The Modem TLS profile
 */
CONFIG_UINT8(MODEM_TLS_PROFILE, 1)

/**
 * @brief ESP-IDF log prefix.
 */
static constexpr const char *TAG = "mqtt_example";

/**
 * @brief The modem instance.
 */
WalterModem modem;

/**
 * @brief Incoming data buffer.
 */
uint8_t incomingBuf[256] = {0};

/**
 * @brief String which holds the MAC address of the Walter.
 */
char macString[32];

bool lteConnect()
{
    WalterModemRsp rsp = {};

    if (modem.setOpState(WALTER_MODEM_OPSTATE_NO_RF)) {
        ESP_LOGI(TAG, "Successfully set operational state to NO RF");
    } else {
        ESP_LOGI(TAG, "Could not set operational state to NO RF");
        return false;
    }

    /* Create PDP context */
    if (modem.definePDPContext(1, CELLULAR_APN)) {
        ESP_LOGI(TAG, "Created PDP context");
    } else {
        ESP_LOGI(TAG, "Could not create PDP context");
        return false;
    }

    /* Set the operational state to full */
    if (modem.setOpState(WALTER_MODEM_OPSTATE_FULL)) {
        ESP_LOGI(TAG, "Successfully set operational state to FULL");
    } else {
        ESP_LOGI(TAG, "Could not set operational state to FULL");
        return false;
    }

    /* Set the network operator selection to automatic */
    if (modem.setNetworkSelectionMode(WALTER_MODEM_NETWORK_SEL_MODE_AUTOMATIC)) {
        ESP_LOGI(TAG, "Network selection mode to was set to automatic");
    } else {
        ESP_LOGI(TAG, "Could not set the network selection mode to automatic");
        return false;
    }

    return waitForNetwork();
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Walter modem mqtt example v1.0.0");

    /* Get the MAC address for board validation */
    esp_read_mac(incomingBuf, ESP_MAC_WIFI_STA);
    sprintf(
        macString,
        "walter%02X:%02X:%02X:%02X:%02X:%02X",
        incomingBuf[0],
        incomingBuf[1],
        incomingBuf[2],
        incomingBuf[3],
        incomingBuf[4],
        incomingBuf[5]);

    /* Initialize the modem */
    if (WalterModem::begin(UART_NUM_1)) {
        ESP_LOGI(TAG, "Successfully initialized modem");
    } else {
        ESP_LOGE(TAG, "Could not initialize modem");
        return;
    }

    WalterModemRsp rsp = {};

    if (!lteConnect()) {
        ESP_LOGE(TAG, "Could Not Connect to LTE");
        return;
    }

    /* other public mqtt broker with web client: mqtthq.com */
    if (modem.mqttConfig("walter-mqtt-test-topic", "", "")) {
        ESP_LOGI(TAG, "MQTT configuration succeeded");
    } else {
        ESP_LOGE(TAG, "MQTT configuration failed");
        return;
    }

    if (modem.mqttConnect("test.mosquitto.org", 1883)) {
        ESP_LOGI(TAG, "MQTT connection succeeded");
    } else {
        ESP_LOGE(TAG, "MQTT connection failed");
        return;
    }

    if (modem.mqttSubscribe("walter-mqtt-test-topic")) {
        ESP_LOGI(TAG, "MQTT subscribed to topic 'walter-mqtt-test-topic'");
    } else {
        ESP_LOGE(TAG, "MQTT subscribe failed");
        return;
    }

    for (;;) {
        WalterModemRsp rsp = {};

        static int seq = 0;
        static char outgoingMsg[64];
        seq++;
        if (seq % 3 == 0) {
            sprintf(outgoingMsg, "%s-%d", macString, seq);

            if (modem.mqttPublish(
                    "waltertopic", (uint8_t *)outgoingMsg, strlen(outgoingMsg), 2, &rsp)) {
                ESP_LOGI(TAG, "published '%s' on topic 'waltertopic'", outgoingMsg);
            } else {
                ESP_LOGE(TAG, "MQTT publish failed");
            }
        }

        while (modem.mqttDidRing("waltertopic", incomingBuf, sizeof(incomingBuf), &rsp)) {
            ESP_LOGI(
                TAG,
                "incoming: qos=%d msgid=%d len=%d:",
                rsp.data.mqttResponse.qos,
                rsp.data.mqttResponse.messageId,
                rsp.data.mqttResponse.length);
            incomingBuf[rsp.data.mqttResponse.length] = '\0';
            ESP_LOGI(TAG, "%s", incomingBuf);
        }

        vTaskDelay(pdMS_TO_TICKS(SEND_DELAY_MS));
    }
}
