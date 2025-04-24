#include <esp_mac.h>
#include <esp_log.h>
#include <driver/uart.h>
#include <cstring>
#include "WalterModem.h"
/**
 * @brief HTTP profile
 */
#define HTTP_PROFILE 1

/**
 * @brief The modem instance.
 */
WalterModem modem;

/**
 * @brief The buffer to transmit to the HTTP server. The first 6 bytes will be
 * the MAC address of the Walter this code is running on.
 */
uint8_t dataBuf[8] = {0};

/**
 * @brief Buffer for incoming HTTP response
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
    ESP_LOGI("http_test", "Connected to the network");
}



extern "C" void app_main(void)
{
    ESP_LOGI("http_test","WalterModem test example.");
    esp_read_mac(dataBuf, ESP_MAC_WIFI_STA);
    ESP_LOGI("socket_test", "Walter's MAC is: %02X:%02X:%02X:%02X:%02X:%02X",
             dataBuf[0],
             dataBuf[1],
             dataBuf[2],
             dataBuf[3],
             dataBuf[4],
             dataBuf[5]);
    
    if(!WalterModem::begin(UART_NUM_1)){
        ESP_LOGE("http_test","unable to start walterModem!");
        return;
    }

    ESP_LOGI("http_test", "waiting for network");

    WalterModemRsp rsp = {};
    if(modem.getOpState(&rsp)) {
        ESP_LOGI("http_test", "Modem operational state: %d", rsp.data.opState);
    } else {
        ESP_LOGE("http_test", "Could not retrieve modem operational state");
        return;
    }

    if(!modem.setOpState(WALTER_MODEM_OPSTATE_NO_RF)) {
        ESP_LOGE("http_test", "Could not set operational state to NO RF");
        return;
    }

    /* Give the modem time to detect the SIM */
    vTaskDelay(pdMS_TO_TICKS(2000));

    if(!modem.definePDPContext()) {
        ESP_LOGE("http_test", "Could not create PDP context");
        return;
    }

    if(!modem.setOpState(WALTER_MODEM_OPSTATE_FULL)) {
        ESP_LOGE("http_test", "Could not set operational state to FULL");
        return;
    }

    if(!modem.setNetworkSelectionMode(WALTER_MODEM_NETWORK_SEL_MODE_AUTOMATIC)) {
        ESP_LOGE("http_test", "Could not set the network selection mode to automatic");
        return;
    }

    waitForNetwork();

    if (!modem.httpConfigProfile(HTTP_PROFILE, "coap.bluecherry.io", 80)) {
        ESP_LOGE("http_test","Could not configure the http profile");
    }


    for(;;){
        dataBuf[6] = counter >> 8;
        dataBuf[7] = counter & 0xFF;

        WalterModemRsp rsp = {};

        /* HTTP test */
        static short httpReceiveAttemptsLeft = 0;
        static char ctbuf[32];

        if (!httpReceiveAttemptsLeft) {
            if(modem.httpSend(HTTP_PROFILE, "/", dataBuf, 8, WALTER_MODEM_HTTP_SEND_CMD_POST, WALTER_MODEM_HTTP_POST_PARAM_OCTET_STREAM, ctbuf, sizeof(ctbuf))) {
                ESP_LOGI("http_test","query performed\r\n");
                httpReceiveAttemptsLeft = 3;
            } else {
                ESP_LOGI("http_test","query failed\r\n");
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        } else {
            while (modem.httpDidRing(HTTP_PROFILE, incomingBuf, sizeof(incomingBuf), &rsp)){
                httpReceiveAttemptsLeft = 0;

                ESP_LOGI("http_test","status code: %d\r\n", rsp.data.httpResponse.httpStatus);
                ESP_LOGI("http_test","content type: %s\r\n", ctbuf);
                ESP_LOGI("http_test","[%s]\r\n", incomingBuf);
            }

            if (httpReceiveAttemptsLeft) {
                ESP_LOGW("htpp_test","response not yet received\r\n");
                httpReceiveAttemptsLeft--;
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}