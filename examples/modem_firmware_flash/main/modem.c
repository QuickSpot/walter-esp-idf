/* Modem Firmware Flash example

   Copyright (C) 2025, DPTechnics bv
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

     1. Redistributions of source code must retain the above copyright notice,
        this list of conditions and the following disclaimer.
   
     2. Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
   
     3. Neither the name of DPTechnics bv nor the names of its contributors may
        be used to endorse or promote products derived from this software
        without specific prior written permission.
   
     4. This software, with or without modification, must only be used with a
        Walter board from DPTechnics bv.
   
     5. Any software provided in binary form under this license must not be
        reverse engineered, decompiled, modified and/or disassembled.
   
   THIS SOFTWARE IS PROVIDED BY DPTECHNICS BV “AS IS” AND ANY EXPRESS OR IMPLIED
   WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
   MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL DPTECHNICS BV OR CONTRIBUTORS BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <stdio.h>
#include <stdlib.h>
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_modem_api.h"
#include "esp_netif.h"
#include "esp_netif_ppp.h"

#include "modem.h"
#include "modem_stp.h"

#define UART_NUM UART_NUM_1
#define TX_PIN 48
#define RX_PIN 14
#define CTS_PIN 47
#define RTS_PIN 21
#define RESET_PIN 45

#define BUF_SIZE 512
#define RSP_TIMEOUT_MS 30000

static const char *TAG = "modem";

static void modem_config_reset_gpio(void)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT_OD;
    io_conf.pin_bit_mask = (1ULL << RESET_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    gpio_hold_dis((gpio_num_t)RESET_PIN);
}

esp_err_t modem_init(esp_modem_dce_t **dce, esp_netif_t **esp_netif)
{
    esp_err_t ret;
    esp_modem_dce_config_t dce_config = ESP_MODEM_DCE_DEFAULT_CONFIG("");
    esp_modem_dte_config_t dte_config = ESP_MODEM_DTE_DEFAULT_CONFIG();
    esp_netif_config_t netif_ppp_config = ESP_NETIF_DEFAULT_PPP();

    *esp_netif = esp_netif_new(&netif_ppp_config);
    if (*esp_netif == NULL) {
        ESP_LOGE(TAG, "Failed to create esp netif instance");
        return ESP_FAIL;
    }

    dte_config.uart_config.tx_io_num = TX_PIN;
    dte_config.uart_config.rx_io_num = RX_PIN;
    dte_config.uart_config.cts_io_num = CTS_PIN;
    dte_config.uart_config.rts_io_num = RTS_PIN;
    dte_config.uart_config.flow_control = ESP_MODEM_FLOW_CONTROL_HW;

    *dce = esp_modem_new_dev(ESP_MODEM_DCE_SQNGM02S, &dte_config, &dce_config, *esp_netif);
    if (*dce == NULL) {
        ESP_LOGE(TAG, "Failed to create esp modem instance");
        return ESP_FAIL;
    }

    modem_config_reset_gpio();

    ret = modem_reset(dce);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset");
        return ret;
    }

    return ESP_OK;
}

esp_err_t modem_reset(esp_modem_dce_t **dce)
{
    if (*dce == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    gpio_set_level((gpio_port_t)RESET_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level((gpio_port_t)RESET_PIN, 1);

    esp_modem_at_raw(*dce, "", NULL, "+SYS", "ERROR", 15000);

    vTaskDelay(pdMS_TO_TICKS(1000));

    return esp_modem_at(*dce, "AT", NULL, RSP_TIMEOUT_MS);
}

esp_err_t modem_deinit(esp_modem_dce_t **dce)
{
    if (*dce == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_modem_destroy(*dce);
    return ESP_OK;
}

modem_boot_mode_t modem_get_boot_mode(esp_modem_dce_t **dce)
{
    esp_err_t ret;
    char data[BUF_SIZE];
    char *endptr;
    int mode;

    if (*dce == NULL) {
        return -1;
    }

    ret = esp_modem_at_raw(*dce, "AT+SMOD?\r", data, "OK", "ERROR", RSP_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read boot mode response");
        return -1;
    }

    mode = (int)strtol(data, &endptr, 10);
    if (endptr == data) {
        ESP_LOGE(TAG, "Failed to convert boot mode response to integer");
        return -1;
    }

    return (modem_boot_mode_t)mode;
}

esp_err_t modem_set_boot_mode(esp_modem_dce_t **dce, modem_boot_mode_t mode)
{
    esp_err_t ret;
    char cmd[BUF_SIZE];

    if (*dce == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    snprintf(cmd, BUF_SIZE, "AT+SMSWBOOT=%u,0", (mode == MODEM_BOOT_MODE_FFH_NO_MTOOLS) ? mode : (mode - 1));

    ret = esp_modem_at(*dce, cmd, NULL, RSP_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure boot mode");
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(2000));

    ret = modem_reset(dce);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset after setting boot mode");
        return ret;
    }

    return ESP_OK;
}

const char *modem_get_sw_version(esp_modem_dce_t **dce)
{
    esp_err_t ret;
    static char data[BUF_SIZE];

    if (*dce == NULL) {
        return NULL;
    }

    ret = esp_modem_at(*dce, "ATI1\r", data, RSP_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to fetch software version");
        return NULL;
    }

    return data;
}

static esp_err_t modem_stp_transfer(const char *filename)
{
    esp_err_t ret;
    FILE *file;
    uint16_t block_size;

    file = fopen(filename, "rb");
    if (!file) {
        ESP_LOGE(TAG, "Failed to open file (%s)", filename);
        return ESP_FAIL;
    }

    ret = stp_reset(UART_NUM);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset STP session");
        fclose(file);
        return ret;
    }

    block_size = stp_open_session(UART_NUM);
    if (block_size == 0) {
        ESP_LOGE(TAG, "Failed to open STP session");
        fclose(file);
        return ret;
    }

    uint8_t buffer[block_size];
    size_t bytes_read, total_bytes_read = 0;
    int last_progress = -1;
    uint16_t transaction_id = 2;

    fseek(file, 0, SEEK_END);
    size_t total_size = ftell(file);
    fseek(file, 0, SEEK_SET);

    while ((bytes_read = fread(buffer, 1, block_size, file)) > 0) {
        ret = stp_transfer_block(UART_NUM, transaction_id, buffer, bytes_read);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to transfer STP block");
            fclose(file);
            return ret;
        }
        transaction_id += 2;

        total_bytes_read += bytes_read;
        if (total_size > 0) {
            int progress = (total_bytes_read * 100) / total_size;
            if (progress != last_progress) {
                ESP_LOGI(TAG, "Transferring: %d%%", progress);
                last_progress = progress;
            }
        }
    }

    if (ferror(file)) {
        ESP_LOGE(TAG, "Failed to read file (%s)", filename);
        fclose(file);
        return ESP_FAIL;
    }

    fclose(file);

    ret = stp_reset(UART_NUM);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed final reset STP session");
        return ret;
    }

    return ESP_OK;
}

esp_err_t modem_transfer_mtools(esp_modem_dce_t **dce, const char *filename)
{
    esp_err_t ret;

    if (*dce == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ret = esp_modem_at(*dce, "AT+STP", NULL, RSP_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start STP mode");
        return ret;
    }

    ret = modem_stp_transfer(filename);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to transfer %s using STP", filename);
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(3000));

    ret = esp_modem_at(*dce, "AT", NULL, RSP_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to communicate after mtools.elf transfer");
        return ret;
    }

    return ESP_OK;
}

esp_err_t modem_transfer_dup(esp_modem_dce_t **dce, const char *filename)
{
    esp_err_t ret;

    if (*dce == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ret = esp_modem_at(*dce, "AT+SMSTPU=\"ON_THE_FLY\"", NULL, RSP_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start STP mode");
        return ret;
    }

    ret = modem_stp_transfer(filename);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to transfer %s using STP", filename);
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(3000));

    ret = esp_modem_at(*dce, "AT", NULL, RSP_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to communicate after .dup transfer");
        return ret;
    }

    ret = modem_set_boot_mode(dce, MODEM_BOOT_MODE_FFF);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to change boot mode after .dup transfer");
        return ret;
    }

    return ESP_OK;
}