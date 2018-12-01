/**
 * @file DPA support library (JSON support)
 * @version 1.4.0
 *
 * Copyright 2015-2018 IQRF Tech s.r.o.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef _DPA_JSON_H
#define _DPA_JSON_H

#if defined(__cplusplus)
extern "C" {
#endif

#include <dpa_library.h>

#define JSON_MSG_ID_BUFF_SIZE   10
#define JSON_REQUEST_BUFF_SIZE  32
#define JSON_OBJECT_BUFF_SIZE   384

typedef enum{
    JSON_PARSE_OK = 0,
    JSON_PARSE_ERROR
} PARSE_RESULT;

/**
 * Initialize support of JSON format in DPA library
 */
void jsonInit(void);

/**
 * Parse DPA request packet from MQTT JSON message
 * @param DpaPacket pointer to DPA request packet, filled with data from received MQTT JSON message
 * @param DataBuffer pointer to buffer, with received MQTT JSON message
 * @param DataBufferSize size of MQTT JSON message
 * @return operation result (JSON_PARSE_OK or JSON_PARSE_ERROR)
 */
PARSE_RESULT jsonParse(T_DPA_PACKET *DpaPacket, uint8_t *DataBuffer, uint16_t DataBufferSize);

/**
 * Create JSON message from received DPA packet
 * @param DataBuffer  - pointer to buffer with generated JSON message
 * @param DpaResponse - pointer to T_DPA_PACKET with DPA notification or response data
 * @param DpaOperationResult - result of dpaSendRequest(... ) function
 * @return size of created JSON message (0 = message was not created, due to its size is over the size of DataBuffer)
 */
uint16_t jsonCreate(uint8_t *DataBuffer, T_DPA_PACKET *DpaResponse, DPA_OPERATION_RESULT DpaOperationResult);

/**
 * Get the timeout parameter from JSON message ( used in dpaSendRequest(... ) function )
 * @return  timeout parameter from JSON message (0 = timeout parameter not defined)
 */
uint16_t jsonGetTimeout(void);

/**
 * Get the number of additional data bytes of DPA request in JSON message ( used in dpaSendRequest(... ) function )
 * @return  number of additional data bytes in DPA request message
 */
uint8_t jsonGetDataSize(void);

#if defined(__cplusplus)
}
#endif

#endif
