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

#include <stddef.h>
#include <stdlib.h>
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"

#include "modem_stp.h"

#define UART_READ_TIMEOUT_MS 30000

static const char* TAG = "modem_stp";

static uint16_t calculate_crc16(const uint8_t* data, size_t length)
{
  uint16_t crc = 0;

  while(length-- > 0) {
    crc = (uint16_t) ((0xff & (crc >> 8)) | (crc << 8));
    crc ^= (uint16_t) *data++;
    crc ^= (uint16_t) ((crc & 0xff) >> 4);
    crc ^= (uint16_t) (crc << 12);
    crc ^= (uint16_t) ((uint8_t) (crc & 0xff) << 5);
  }

  return __builtin_bswap16(crc);
}

esp_err_t stp_reset(uart_port_t port_num)
{
  struct stp_message msg;
  int bytes_sent, bytes_read;

  msg.signature = __builtin_bswap32(STP_REQUEST_SIGNATURE);
  msg.operation = STP_REQUEST_OPERATION_RESET;
  msg.session_id = 0;
  msg.payload_length = 0;
  msg.transaction_id = 0;
  msg.header_crc16 = 0;
  msg.payload_crc16 = 0;
  msg.header_crc16 = calculate_crc16((uint8_t*) &msg, sizeof(msg));

  bytes_sent = uart_write_bytes(port_num, (const uint8_t*) &msg, sizeof(msg));
  if(bytes_sent != sizeof(msg)) {
    return ESP_FAIL;
  }

  ESP_LOGD(TAG, "STP reset req: tx=%d header: 0x%lx 0x%x %u %u %lu 0x%x 0x%x", bytes_sent,
           __builtin_bswap32(msg.signature), msg.operation, msg.session_id,
           __builtin_bswap16(msg.payload_length), __builtin_bswap32(msg.transaction_id),
           __builtin_bswap16(msg.header_crc16), __builtin_bswap16(msg.payload_crc16));

  bytes_read =
      uart_read_bytes(port_num, (uint8_t*) &msg, sizeof(msg), pdMS_TO_TICKS(UART_READ_TIMEOUT_MS));
  if(bytes_read != sizeof(msg)) {
    return ESP_FAIL;
  }

  ESP_LOGD(TAG, "STP reset rsp: rx=%d header: 0x%lx 0x%x %u %u %lu 0x%x 0x%x", bytes_sent,
           __builtin_bswap32(msg.signature), msg.operation, msg.session_id,
           __builtin_bswap16(msg.payload_length), __builtin_bswap32(msg.transaction_id),
           __builtin_bswap16(msg.header_crc16), __builtin_bswap16(msg.payload_crc16));

  return ESP_OK;
}

uint16_t stp_open_session(uart_port_t port_num)
{
  struct stp_message msg;
  struct stp_response_session_open rsp;
  int bytes_sent, bytes_read;

  msg.signature = __builtin_bswap32(STP_REQUEST_SIGNATURE);
  msg.operation = STP_REQUEST_OPERATION_OPEN_SESSION;
  msg.session_id = 1;
  msg.payload_length = 0;
  msg.transaction_id = __builtin_bswap32(1);
  msg.header_crc16 = 0;
  msg.payload_crc16 = 0;
  msg.header_crc16 = calculate_crc16((uint8_t*) &msg, sizeof(msg));

  bytes_sent = uart_write_bytes(port_num, (const uint8_t*) &msg, sizeof(msg));
  if(bytes_sent != sizeof(msg)) {
    return 0;
  }

  ESP_LOGD(TAG, "STP open session req: tx=%d header: 0x%lx 0x%x %u %u %lu 0x%x 0x%x", bytes_sent,
           __builtin_bswap32(msg.signature), msg.operation, msg.session_id,
           __builtin_bswap16(msg.payload_length), __builtin_bswap32(msg.transaction_id),
           __builtin_bswap16(msg.header_crc16), __builtin_bswap16(msg.payload_crc16));

  bytes_read =
      uart_read_bytes(port_num, (uint8_t*) &msg, sizeof(msg), pdMS_TO_TICKS(UART_READ_TIMEOUT_MS));
  if(bytes_read != sizeof(msg)) {
    return 0;
  }

  ESP_LOGD(TAG, "STP open session rsp: rx=%d header: 0x%lx 0x%x %u %u %lu 0x%x 0x%x", bytes_sent,
           __builtin_bswap32(msg.signature), msg.operation, msg.session_id,
           __builtin_bswap16(msg.payload_length), __builtin_bswap32(msg.transaction_id),
           __builtin_bswap16(msg.header_crc16), __builtin_bswap16(msg.payload_crc16));

  bytes_read =
      uart_read_bytes(port_num, (uint8_t*) &rsp, sizeof(rsp), pdMS_TO_TICKS(UART_READ_TIMEOUT_MS));
  if(bytes_read != sizeof(rsp)) {
    return 0;
  }

  ESP_LOGD(TAG, "STP open session rsp payload: rx=%d payload: %u %u %u", bytes_read, rsp.success,
           rsp.version, __builtin_bswap16(rsp.max_transfer_size));

  return __builtin_bswap16(rsp.max_transfer_size) - sizeof(msg);
}

esp_err_t stp_transfer_block(uart_port_t port_num, uint16_t transaction_id, uint8_t* buffer,
                             size_t block_size)
{
  struct stp_message msg;
  struct stp_request_transfer_block_cmd req;
  struct stp_response_transfer_block rsp;
  int bytes_sent, bytes_read;

  req.block_size = __builtin_bswap16(block_size);

  msg.signature = __builtin_bswap32(STP_REQUEST_SIGNATURE);
  msg.operation = STP_REQUEST_OPERATION_TRANSFER_BLOCK_CMD;
  msg.session_id = 1;
  msg.payload_length = __builtin_bswap16(sizeof(req));
  msg.transaction_id = __builtin_bswap32(transaction_id);
  msg.header_crc16 = 0;
  msg.payload_crc16 = calculate_crc16((uint8_t*) &req, sizeof(req));
  msg.header_crc16 = calculate_crc16((uint8_t*) &msg, sizeof(msg));

  bytes_sent = uart_write_bytes(port_num, (const uint8_t*) &msg, sizeof(msg));
  if(bytes_sent != sizeof(msg)) {
    return 0;
  }

  ESP_LOGD(TAG, "STP transfer block cmd req: tx=%d header: 0x%lx 0x%x %u %u %lu 0x%x 0x%x",
           bytes_sent, __builtin_bswap32(msg.signature), msg.operation, msg.session_id,
           __builtin_bswap16(msg.payload_length), __builtin_bswap32(msg.transaction_id),
           __builtin_bswap16(msg.header_crc16), __builtin_bswap16(msg.payload_crc16));

  bytes_sent = uart_write_bytes(port_num, (const uint8_t*) &req, sizeof(req));
  if(bytes_sent != sizeof(req)) {
    return 0;
  }

  ESP_LOGD(TAG, "STP transfer block cmd req payload: tx=%d payload: %u", bytes_sent,
           __builtin_bswap16(req.block_size));

  bytes_read =
      uart_read_bytes(port_num, (uint8_t*) &msg, sizeof(msg), pdMS_TO_TICKS(UART_READ_TIMEOUT_MS));
  if(bytes_read != sizeof(msg)) {
    return ESP_FAIL;
  }

  ESP_LOGD(TAG, "STP transfer block cmd rsp: rx=%d header: 0x%lx 0x%x %u %u %lu 0x%x 0x%x",
           bytes_read, __builtin_bswap32(msg.signature), msg.operation, msg.session_id,
           __builtin_bswap16(msg.payload_length), __builtin_bswap32(msg.transaction_id),
           __builtin_bswap16(msg.header_crc16), __builtin_bswap16(msg.payload_crc16));

  msg.signature = __builtin_bswap32(STP_REQUEST_SIGNATURE);
  msg.operation = STP_REQUEST_OPERATION_TRANSFER_BLOCK;
  msg.session_id = 1;
  msg.payload_length = __builtin_bswap16(block_size);
  msg.transaction_id = __builtin_bswap32(transaction_id + 1);
  msg.header_crc16 = 0;
  msg.payload_crc16 = calculate_crc16(buffer, block_size);
  msg.header_crc16 = calculate_crc16((uint8_t*) &msg, sizeof(msg));

  bytes_sent = uart_write_bytes(port_num, (const uint8_t*) &msg, sizeof(msg));
  if(bytes_sent != sizeof(msg)) {
    return 0;
  }

  ESP_LOGD(TAG, "STP transfer block req: tx=%d header: 0x%lx 0x%x %u %u %lu 0x%x 0x%x", bytes_sent,
           __builtin_bswap32(msg.signature), msg.operation, msg.session_id,
           __builtin_bswap16(msg.payload_length), __builtin_bswap32(msg.transaction_id),
           __builtin_bswap16(msg.header_crc16), __builtin_bswap16(msg.payload_crc16));

  bytes_sent = uart_write_bytes(port_num, (const uint8_t*) buffer, block_size);
  if(bytes_sent != block_size) {
    return 0;
  }

  ESP_LOGD(TAG, "STP transfer block req payload: tx=%d", bytes_sent);

  bytes_read =
      uart_read_bytes(port_num, (uint8_t*) &msg, sizeof(msg), pdMS_TO_TICKS(UART_READ_TIMEOUT_MS));
  if(bytes_read != sizeof(msg)) {
    return ESP_FAIL;
  }

  ESP_LOGD(TAG, "STP transfer block rsp: rx=%d header: 0x%lx 0x%x %u %u %lu 0x%x 0x%x", bytes_read,
           __builtin_bswap32(msg.signature), msg.operation, msg.session_id,
           __builtin_bswap16(msg.payload_length), __builtin_bswap32(msg.transaction_id),
           __builtin_bswap16(msg.header_crc16), __builtin_bswap16(msg.payload_crc16));

  bytes_read =
      uart_read_bytes(port_num, (uint8_t*) &rsp, sizeof(rsp), pdMS_TO_TICKS(UART_READ_TIMEOUT_MS));
  if(bytes_read != sizeof(rsp)) {
    return 0;
  }

  ESP_LOGD(TAG, "STP transfer block rsp payload: rx=%d payload: %u", bytes_read,
           __builtin_bswap16(rsp.residue));

  return ESP_OK;
}