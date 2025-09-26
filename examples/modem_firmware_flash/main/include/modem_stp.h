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

#include <stdint.h>
#include "driver/uart.h"
#include "esp_err.h"

/**
 * @brief Signature value used in STP request.
 */
#define STP_REQUEST_SIGNATURE 0x66617374

/**
 * @brief Signature value used in STP response.
 */
#define STP_RESPONSE_SIGNATURE 0x74736166

/**
 * @brief STP request operations.
 */
typedef enum {
  /** Reset operation */
  STP_REQUEST_OPERATION_RESET = 0x00,
  /** Open session operation */
  STP_REQUEST_OPERATION_OPEN_SESSION = 0x01,
  /** Transfer block command operation */
  STP_REQUEST_OPERATION_TRANSFER_BLOCK_CMD = 0x02,
  /** Transfer block operation */
  STP_REQUEST_OPERATION_TRANSFER_BLOCK = 0x03
} stp_request_operation_t;

/**
 * @brief STP response operations.
 */
typedef enum {
  /** Reset operation */
  STP_RESPONSE_OPERATION_RESET = 0x80,
  /** Open session operation */
  STP_RESPONSE_OPERATION_OPEN_SESSION = 0x81,
  /** Transfer block command operation */
  STP_RESPONSE_OPERATION_TRANSFER_BLOCK_CMD = 0x82,
  /** Transfer block operation */
  STP_RESPONSE_OPERATION_TRANSFER_BLOCK = 0x83
} stp_response_operation_t;

/**
 * @brief STP message structure.
 */
struct stp_message {
  uint32_t signature;
  uint8_t operation;
  uint8_t session_id;
  uint16_t payload_length;
  uint32_t transaction_id;
  uint16_t header_crc16;
  uint16_t payload_crc16;
};

/**
 * @brief STP session open response structure.
 */
struct stp_response_session_open {
  uint8_t success;
  uint8_t version;
  uint16_t max_transfer_size;
};

/**
 * @brief STP transfer block command request structure.
 */
struct stp_request_transfer_block_cmd {
  uint16_t block_size;
};

/**
 * @brief STP transfer block response stucture.
 */
struct stp_response_transfer_block {
  uint16_t residue;
};

/**
 * @brief Send a Simple Transfer Protocol reset command to the modem.
 *
 * @param port_num UART port number connected to the
 *
 * @return ESP_OK on success, otherwise an esp error code
 */
esp_err_t stp_reset(uart_port_t port_num);

/**
 * @brief Send a Simple Transfer Protocol open session command to the modem.
 *
 * @param port_num Modem UART port number
 *
 * @return Maximum accepted transfer size of an STP payload (without header) by the modem
 */
uint16_t stp_open_session(uart_port_t port_num);

/**
 * @brief Transfer a Simple Transfer Protocol block to the modem.
 *
 * @param port_num Modem UART port number
 * @param transaction_id Transaction ID for the STP payload
 * @param buffer Buffer containing the data to be transferred
 * @param block_size Size of the data block in bytes to be transferred
 *
 * @return ESP_OK on success, otherwise an esp error code
 */
esp_err_t stp_transfer_block(uart_port_t port_num, uint16_t transaction_id, uint8_t* buffer,
                             size_t block_size);