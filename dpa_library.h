/**
 * @file DPA support library
 * @version 0.93
 *
 * Copyright 2015-2016 MICRORISC s.r.o.
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

#ifndef _DPA_LIBRARY_H
#define _DPA_LIBRARY_H

#include <stdint.h>
#include <stddef.h>

#include "DPA.h"

#define __SPI_INTERFACE__         // select for comunication via SPI
//#define __UART_INTERFACE__      // select for comunication via UART
//#define __STORE_CODE_SUPPORT__  // uncomment for TR7xD modules code upload support

#define TR7xD                     // select for TR7xD module
//#define TR5xD                   // select for TR5xD module

/*
 * Library status
 */
/// DPA support library ready
#define  DPA_READY      0x00
/// DPA request is preparing
#define  DPA_PREPARE    0x01
/// DPA request processing
#define  DPA_BUSY       0x02

typedef struct {
	uint16_t NADR;
	uint8_t PNUM;
	uint8_t PCMD;
	uint16_t HWPID;
	uint8_t ResponseCode;
	uint8_t DpaValue;
	TDpaMessage DpaMessage;
} T_DPA_PACKET;

/// DPA response callback function type
typedef void (*T_DPA_ANSWER_HANDLER)(T_DPA_PACKET *DpaAnswer);

typedef struct {
	uint8_t status;
	uint8_t timeFlag;
	uint8_t timeCnt;
	uint8_t extraDataSize;
	T_DPA_ANSWER_HANDLER dpaAnswerHandler;
	T_DPA_PACKET *dpaRequestPacketPtr;
} T_DPA_CONTROL;

extern T_DPA_CONTROL dpaControl;

void DPA_Init(void);
void DPA_LibraryDriver(void);
void DPA_SendRequest(T_DPA_PACKET *dpaRequest, uint8_t dataSize);
uint16_t DPA_GetEstimatedTimeout(void);

#if defined(__UART_INTERFACE__)
void DPA_ReceiveUartByte(uint8_t rxByte);
#endif

/**
 * Return actual status of DPA support library
 * @return actual status of DPA support library
 */
#define DPA_GetStatus()   dpaControl.status

/**
 * Setting time flag for DPA_LibraryDriver function
 * Must be called periodicaly every 1 ms (TR5xD) or 150us (TR7xD)
 */
#define DPA_SetTimmingFlag()  dpaControl.timeFlag = 1;

/**
 * Set pointer to user DPA response handler
 * @param A1 Pointer to user DPA response handler
 */
#define DPA_SetAnswerHandler(A1) dpaControl.dpaAnswerHandler = A1

#endif
