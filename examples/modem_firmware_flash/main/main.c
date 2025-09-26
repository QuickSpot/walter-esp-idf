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

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "esp_err.h"
#include "esp_event.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "modem.h"

#define LATEST_RELEASE "LR8.2.1.0-61488"

#define FATFS_BASE_PATH "/ffat"
#define MTOOLS_PATH FATFS_BASE_PATH "/mtools.elf"
#define DUP_IMG_PATH FATFS_BASE_PATH "/GP02RBAQ-DM_LR8.2.1.0-61488.dup"

static const char *TAG = "main";

static esp_err_t fatfs_init()
{
    const esp_vfs_fat_mount_config_t conf = {
        .max_files = 2,
        .format_if_mount_failed = false,
        .allocation_unit_size = CONFIG_WL_SECTOR_SIZE,
        .use_one_fat = false,
    };

    return esp_vfs_fat_spiflash_mount_ro(FATFS_BASE_PATH, "ffat", &conf);
}

void app_main(void)
{
    esp_err_t ret;
    esp_modem_dce_t *dce = NULL;
    esp_netif_t *esp_netif = NULL;

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_LOGI(TAG, "Initializing FATFS partition");
    ret = fatfs_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount FATFS partition (%s)", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "Initializing modem");
    ret = modem_init(&dce, &esp_netif);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize modem");
        return;
    }

    for (;;) {
        modem_boot_mode_t current_boot_mode = modem_get_boot_mode(&dce);
        if (current_boot_mode == MODEM_BOOT_MODE_INVALID) {
            ESP_LOGE(TAG, "Failed to get current modem boot mode");
            return;
        }

        switch (current_boot_mode) {
        case MODEM_BOOT_MODE_FFH_NO_MTOOLS:
            ESP_LOGI(TAG, "Boot mode: FFH, bootloader waiting for mtools.elf");

            ESP_LOGI(TAG, "Starting transfer of mtools.elf");
            ret = modem_transfer_mtools(&dce, MTOOLS_PATH);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to transfer mtools.elf");
                return;
            }

            ESP_LOGI(TAG, "Transfer of mtools.elf complete");
            break;

        case MODEM_BOOT_MODE_FFH_MTOOLS_PRESENT:
            ESP_LOGI(TAG, "Boot mode: FFH, mtools.elf present");

            ESP_LOGI(TAG, "Starting transfer of .dup firmware image");
            ret = modem_transfer_dup(&dce, DUP_IMG_PATH);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to transfer .dup firmware image");
                return;
            }

            ESP_LOGI(TAG, "Transfer of .dup firmware image complete");
            break;

        case MODEM_BOOT_MODE_FFF:
            ESP_LOGI(TAG, "Boot mode: FFF");

            const char *version = modem_get_sw_version(&dce);

            if (strstr(version, LATEST_RELEASE) != NULL) {
                ESP_LOGI(TAG, "No upgrade needed, on latest software version: %s", version);
                return;
            } else {
                ESP_LOGI(TAG, "Not on the latest software version. Current: %s, Expected: %s", version, LATEST_RELEASE);

                ESP_LOGI(TAG, "Switching modem to RECOVERY boot mode to start upgrade procedure");
                ret = modem_set_boot_mode(&dce, MODEM_BOOT_MODE_RECOVERY);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to switch boot mode to RECOVERY");
                    return;
                }
            }
            break;

        case MODEM_BOOT_MODE_RECOVERY:
            ESP_LOGI(TAG, "Boot mode: RECOVERY");

            ESP_LOGI(TAG, "Starting transfer of .dup firmware image");
            ret = modem_transfer_dup(&dce, DUP_IMG_PATH);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to transfer .dup firmware image");
                return;
            }

            ESP_LOGI(TAG, "Transfer of .dup firmware image complete");
            break;

        default:
            ESP_LOGE(TAG, "Modem boot mode not supported");
            return;
        }

        ESP_LOGI(TAG, "Waiting for new boot mode poll");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}
