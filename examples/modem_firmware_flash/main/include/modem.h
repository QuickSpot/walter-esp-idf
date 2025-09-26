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

#pragma once

#include "esp_err.h"
#include "esp_modem_api.h"

/**
 * @brief Modem firmware boot modes
 */
typedef enum {
    /** Boot mode FFH, waiting for mtools.elf */
    MODEM_BOOT_MODE_FFH_NO_MTOOLS = 0,
    /** Boot mode FFH, mtools.elf present */
    MODEM_BOOT_MODE_FFH_MTOOLS_PRESENT = 1,
    /** Boot mode FFF */
    MODEM_BOOT_MODE_FFF = 2,
    /** Boot mode UPDATER */
    MODEM_BOOT_MODE_UPDATER = 3,
    /** Boot mode RECOVERY */
    MODEM_BOOT_MODE_RECOVERY = 4,
    /** Invalid boot mode */
    MODEM_BOOT_MODE_INVALID = -1
} modem_boot_mode_t;

/**
 * @brief Initialize the esp_modem DCE handle and esp_netif instance.
 *
 * @param dce Pointer to the esp_modem DCE handle
 * @param esp_netif Pointer to the esp_netif instance
 * 
 * @return ESP_OK on success, otherwise an esp error code
 */
esp_err_t modem_init(esp_modem_dce_t **dce, esp_netif_t **esp_netif);

/**
 * @brief Reset the modem using the hardware reset pin.
 *
 * @param dce Pointer to the esp_modem DCE handle
 * 
 * @return ESP_OK on success, otherwise an esp error code
 */
esp_err_t modem_reset(esp_modem_dce_t **dce);

/**
 * @brief Deinitialize the esp_modem DCE handle.
 * 
 * @param dce Pointer to the esp_modem DCE handle
 * 
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on invalid handle
 */
esp_err_t modem_deinit(esp_modem_dce_t **dce);

/**
 * @brief Get the boot mode of the modem.
 *
 * @param dce Pointer to the esp_modem DCE handle
 * 
 * @return The boot mode as a modem_boot_mode_t value
 */
modem_boot_mode_t modem_get_boot_mode(esp_modem_dce_t **dce);

/**
 * @brief Set the boot mode of the modem.
 * 
 * @param dce Pointer to the esp_modem DCE handle
 * @param mode The boot mode as a modem_boot_mode_t value
 * 
 * @return ESP_OK on success, otherwise an esp error code
 */
esp_err_t modem_set_boot_mode(esp_modem_dce_t **dce, modem_boot_mode_t mode);

/**
 * @brief Get the software version of the modem using ATI1.
 *
 * @param dce Pointer to the esp_modem DCE handle
 * 
 * @return The software version as a string
 */
const char *modem_get_sw_version(esp_modem_dce_t **dce);

/**
 * @brief Transfer the mtools.elf firmware to the modem using STP.
 * 
 * @param dce Pointer to the esp_modem DCE handle
 * @param filename Filename of the mtools.elf firmware
 * 
 * @return ESP_OK on success, otherwise an esp error code
 */
esp_err_t modem_transfer_mtools(esp_modem_dce_t **dce, const char *filename);

/**
 * @brief Transfer the modem .dup firmware image to the modem using STP.
 * 
 * @param dce Pointer to the esp_modem DCE handle
 * @param filename Filename of the modem .dup firmware image
 * 
 * @return ESP_OK on success, otherwise an esp error code
 */
esp_err_t modem_transfer_dup(esp_modem_dce_t **dce, const char *filename);