/**
 * @file ModemCoapTest.ino
 * @author Dries Vandenbussche <dries@dptechnics.com>
 * @date 01 Jun 2023
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
#include <WalterModem.h>

/**
 * @brief COAP profile used for COAP tests
 */
#define COAP_PROFILE 1

/**
 * @brief The modem instance.
 */
WalterModem modem;
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

void app_main(void)
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
        ESP_LOGI("coap_test", "Modem initialization OK");
    } else {
        ESP_LOGI("coap_test", "Modem initialization ERROR");
        return;
    }

    if (modem.checkComm()) {
        ESP_LOGI("coap_test", "Modem communication is ok");
    } else {
        ESP_LOGI("coap_test", "Modem communication error");
        return;
    }

    WalterModemRsp rsp = {};
    if (modem.getOpState(&rsp)) {
        ESP_LOGI("coap_test", "Modem operational state: %d", rsp.data.opState);
    } else {
        ESP_LOGI("coap_test", "Could not retrieve modem operational state");
        return;
    }

    if(modem.setOpState(WALTER_MODEM_OPSTATE_NO_RF)) {
        ESP_LOGI("mqtt_test", "Successfully set operational state to NO RF");
    } else {
        ESP_LOGI("mqtt_test", "Could not set operational state to NO RF");
        return;
    }

    /* Give the modem time to detect the SIM */
    vTaskDelay(pdMS_TO_TICKS(2000));

    if(modem.unlockSIM()) {
        ESP_LOGI("mqtt_test", "Successfully unlocked SIM card");
    } else {
        ESP_LOGI("mqtt_test", "Could not unlock SIM card");
        return;
    }

    /* Create PDP context */
    if(modem.definePDPContext()) {
        ESP_LOGI("coap_test", "Created PDP context");
    } else {
        ESP_LOGI("coap_test", "Could not create PDP context");
        return;
    }

    if(modem.setPDPAuthParams(WALTER_MODEM_PDP_AUTH_PROTO_NONE,"sora","sora")) {
        ESP_LOGI("coap_test", "Authenticated the PDP context");
    } else {
        ESP_LOGI("coap_test", "Could not authenticate the PDP context");
        return;
    }


    if(modem.getPDPAddress(&rsp)) {
        ESP_LOGI("coap_test", "PDP context address list:");
        ESP_LOGI("coap_test", "  - %s", rsp.data.pdpAddressList.pdpAddress);
        if(rsp.data.pdpAddressList.pdpAddress2[0] != '\0') {
        ESP_LOGI("coap_test", "  - %s", rsp.data.pdpAddressList.pdpAddress2);
        }
    } else {
        ESP_LOGI("coap_test", "Could not retrieve PDP context addresses");
        return;
    }

    for(;;) {
        dataBuf[6] = counter >> 8;
        dataBuf[7] = counter & 0xFF;

        counter++;

        static short receiveAttemptsLeft = 0;

        if(!modem.coapCreateContext(COAP_PROFILE, "coap.me", 5683)) {
            ESP_LOGE("coap_test", "Could not create COAP context. Better luck next iteration?\r\n");
            return;
        } else {
            ESP_LOGI("coap_test", "Successfully created or refreshed COAP context\r\n");
        }

        if(!receiveAttemptsLeft) {
            if(modem.coapSetHeader(COAP_PROFILE, counter)) {
                ESP_LOGI("coap_test", "Set COAP header with message id %d\r\n", counter);
            } else {
                ESP_LOGE("coap_test","Could not set COAP header\r\n");
                vTaskDelay(pdMS_TO_TICKS(1000));
                esp_restart();
            }

            if(modem.coapSendData(COAP_PROFILE, WALTER_MODEM_COAP_SEND_TYPE_CON,
            WALTER_MODEM_COAP_SEND_METHOD_GET, 8, dataBuf)) {
                ESP_LOGI("coap_test", "Sent COAP datagram\r\n");
                receiveAttemptsLeft = 3;
            } else {
                ESP_LOGE("coap_test", "Could not send COAP datagram\r\n");
                vTaskDelay(pdMS_TO_TICKS(1000));
                esp_restart();
            }
        } else {
            receiveAttemptsLeft--;
            ESP_LOGI("coap_test", "Checking for incoming COAP message or response\r\n");

            while(modem.coapDidRing(COAP_PROFILE, incomingBuf, sizeof(incomingBuf), &rsp)) {
            receiveAttemptsLeft = 0;

            ESP_LOGI("coap_test", "COAP incoming:\r\n");
            ESP_LOGI("coap_test", "profileId: %d (profile ID used by us: %d)\r\n",
                    rsp.data.coapResponse.profileId, COAP_PROFILE);
            ESP_LOGI("coap_test", "Message id: %d\r\n", rsp.data.coapResponse.messageId);
            ESP_LOGI("coap_test", "Send type (CON, NON, ACK, RST): %d\r\n",
                    rsp.data.coapResponse.sendType);
            ESP_LOGI("coap_test", "Method or response code: %d\r\n",
                    rsp.data.coapResponse.methodRsp);
            ESP_LOGI("coap_test", "Data (%d bytes):\r\n", rsp.data.coapResponse.length);

            for(size_t i = 0; i < rsp.data.coapResponse.length; i++) {
                ESP_LOGI("coap_test", "[%02x  %c] ", incomingBuf[i], incomingBuf[i]);
            }
            ESP_LOGI("coap_test", "\r\n");
            }
        }
    }
}