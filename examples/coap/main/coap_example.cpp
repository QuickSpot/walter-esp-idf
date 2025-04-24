/**
 * @file coap_test.cpp
 * @author Dries Vandenbussche <dries@dptechnics.com>
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
 * This file contains a sketch which communicates with the coap.me
 * COAP test server.
 */

#include <esp_mac.h>
#include <esp_log.h>

#include <WalterModem.h>

/**
 * @brief Cellular APN for SIM card. Leave empty to autodetect APN.
 */
CONFIG(CELLULAR_APN, const char *, "")

/**
 * @brief COAP profile used for COAP tests
 */
CONFIG_UINT8(MODEM_COAP_PROFILE,0)

/**
 * @brief Time delay in ms of data sent to the coap demo server.
 */
CONFIG_UINT16(SEND_DELAY_MS, 10000)

/**
 * @brief ESP-IDF log prefix.
 */
static constexpr const char *TAG = "coap_example";

/**
 * @brief The modem instance.
 */
WalterModem modem;

/**
 * @brief Response object containing command response information.
 */
WalterModemRsp rsp = {};

/**
 * @brief The buffer to transmit to the COAP server.
 */
uint8_t dataBuf[8] = {0};

/**
 * @brief Buffer for incoming COAP response
 */
uint8_t incomingBuf[256] = {0};

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
    ESP_LOGI("mqtt_test", "Connected to the network");
}

extern "C" void app_main(void)
{
    ESP_LOGI("mqtt_test", "Walter modem test v0.0.1");

    /* Get the MAC address for board validation */
    esp_read_mac(dataBuf, ESP_MAC_WIFI_STA);
    ESP_LOGI("socket_test", "Walter's MAC is: %02X:%02X:%02X:%02X:%02X:%02X",
        dataBuf[0],
        dataBuf[1],
        dataBuf[2],
        dataBuf[3],
        dataBuf[4],
        dataBuf[5]);

    if (WalterModem::begin(UART_NUM_1)) {
        ESP_LOGI(TAG, "Modem initialization OK");
    } else {
        ESP_LOGI(TAG, "Modem initialization ERROR");
        return;
    }

    WalterModemRsp rsp = {};

    /* Create PDP context */
    if(modem.definePDPContext(1, CELLULAR_APN)) {
        ESP_LOGI(TAG, "Created PDP context");
    } else {
        ESP_LOGI(TAG, "Could not create PDP context");
        return;
    }

    /* Set the operational state to full */
    if(modem.setOpState(WALTER_MODEM_OPSTATE_FULL)) {
        ESP_LOGI(TAG, "Successfully set operational state to FULL");
    } else {
        ESP_LOGI(TAG, "Could not set operational state to FULL");
        return;
    }

    /* Set the network operator selection to automatic */
    if(modem.setNetworkSelectionMode(WALTER_MODEM_NETWORK_SEL_MODE_AUTOMATIC)) {
        ESP_LOGI(TAG, "Network selection mode to was set to automatic");
    } else {
        ESP_LOGI(TAG, "Could not set the network selection mode to automatic");
        return;
    }

    waitForNetwork();
    
    for(;;) {
        vTaskDelay(pdMS_TO_TICKS(SEND_DELAY_MS));
        
        dataBuf[6] = counter >> 8;
        dataBuf[7] = counter & 0xFF;

        counter++;

        static short receiveAttemptsLeft = 0;

        if(!modem.coapCreateContext(MODEM_COAP_PROFILE, "coap.me", 5683)) {
            ESP_LOGE(TAG, "Could not create COAP context. Better luck next iteration?");
            continue;
        } else {
            ESP_LOGI(TAG, "Successfully created or refreshed COAP context");
        }

        if(!receiveAttemptsLeft) {
            if(modem.coapSetHeader(MODEM_COAP_PROFILE, counter)) {
                ESP_LOGI(TAG, "Set COAP header with message id %d", counter);
            } else {
                ESP_LOGE(TAG,"Could not set COAP header");
                vTaskDelay(pdMS_TO_TICKS(1000));
                esp_restart();
            }

            if(modem.coapSendData(MODEM_COAP_PROFILE, WALTER_MODEM_COAP_SEND_TYPE_CON,
            WALTER_MODEM_COAP_SEND_METHOD_GET, 8, dataBuf)) {
                ESP_LOGI(TAG, "Sent COAP datagram");
                receiveAttemptsLeft = 3;
            } else {
                ESP_LOGE(TAG, "Could not send COAP datagram");
                vTaskDelay(pdMS_TO_TICKS(1000));
                esp_restart();
            }
        } else {
            receiveAttemptsLeft--;
            ESP_LOGI(TAG, "Checking for incoming COAP message or response");

            while(modem.coapDidRing(MODEM_COAP_PROFILE, incomingBuf, sizeof(incomingBuf), &rsp)) {
            receiveAttemptsLeft = 0;

            ESP_LOGI(TAG, "COAP incoming:\r\n");
            ESP_LOGI(TAG, "profileId: %d (profile ID used by us: %d)\r\n",
                    rsp.data.coapResponse.profileId, MODEM_COAP_PROFILE);
            ESP_LOGI(TAG, "Message id: %d\r\n", rsp.data.coapResponse.messageId);
            ESP_LOGI(TAG, "Send type (CON, NON, ACK, RST): %d\r\n",
                    rsp.data.coapResponse.sendType);
            ESP_LOGI(TAG, "Method or response code: %d\r\n",
                    rsp.data.coapResponse.methodRsp);
            ESP_LOGI(TAG, "Data (%d bytes):\r\n", rsp.data.coapResponse.length);

            for(size_t i = 0; i < rsp.data.coapResponse.length; i++) {
                ESP_LOGI(TAG, "[%02x  %c] ", incomingBuf[i], incomingBuf[i]);
            }
            ESP_LOGI(TAG, "\r\n");
            }
        }
    }
}