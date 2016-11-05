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

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

#include "DPA.h"

#define __SPI_INTERFACE__         // select for comunication via SPI
//#define __UART_INTERFACE__      // select for comunication via UART
#define __STORE_CODE_SUPPORT__    // uncomment for TR7xD modules code upload support

#define TR7xD                     // select for TR7xD module
//#define TR5xD                   // select for TR5xD module

/*
 * Boolean
 */
#define FALSE 0
#define TRUE 1

/*
 * Library status
 */
/// DPA support library ready
#define  DPA_READY      0x00
/// DPA request is preparing
#define  DPA_PREPARE    0x01
/// DPA request processing
#define  DPA_BUSY       0x02

/// Address of the DPA Custom Handler
#define CUSTOM_HANDLER_ADDRESS     0x3A20
/// Address of the DPA Custom Handler end + 1
#define CUSTOM_HANDLER_ADDRESS_END 0x3D80

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
/// DPA timeout callback function type
typedef void (*T_DPA_TIMEOUT_HANDLER)(void);

typedef struct {
    uint8_t status;
    volatile uint8_t timeFlag;
    uint8_t timeCnt;
    uint8_t extraDataSize;
    volatile uint8_t timeoutPrescaller;
    volatile uint8_t timeoutModulator;
    volatile uint16_t timeoutTimer;
    uint16_t fileByteCounter;
    T_DPA_ANSWER_HANDLER dpaAnswerHandler;
    T_DPA_TIMEOUT_HANDLER dpaTimeoutHandler;
    T_DPA_PACKET *dpaRequestPacketPtr;
} T_DPA_CONTROL;

extern T_DPA_CONTROL dpaControl;

#if defined (__STORE_CODE_SUPPORT__) && defined(TR7xD)

#define DPA_CODE_FILE_NOT_DEFINED 0
#define DPA_CODE_FILE_HEX         1
#define DPA_CODE_FILE_IQRF        2

/// DPA_StoreCodeToEeeprom return code (operation ended with success)
#define    DPA_STORE_CODE_SUCCESS        111
/// DPA_StoreCodeToEeeprom return code (operation ended with error)
#define    DPA_STORE_CODE_ERROR        222

typedef struct {
    uint16_t trAddress; //!< DPA address of destination TR module
    uint16_t imageEeepromAdr; //!< absolute address in TR eeeprom to store code
    uint16_t imageSize; //!< size of code image stored in eeeprom
    uint16_t imageCRC; //!< initial CRC value (before save) / CRC of code image (after save)
    uint16_t fileSize; //!< size of code file on SD card
    uint8_t fileType; //!< file type (HEX / IQRF)
} T_DPA_CODE_FILE_INFO;

#endif

void DPA_Init(void);
void DPA_LibraryDriver(void);
void DPA_SendRequest(T_DPA_PACKET *dpaRequest, uint8_t dataSize);
uint16_t DPA_GetEstimatedTimeout(void);
void DPA_SetTimmingFlag(void);
uint8_t DPA_MakeConfigurationCRC(T_DPA_PACKET *dpaRequest);

#if defined(__UART_INTERFACE__)
void DPA_ReceiveUartByte(uint8_t rxByte);
#endif

#if defined (__STORE_CODE_SUPPORT__) && defined(TR7xD)
uint8_t DPA_StoreCodeToEeeprom(T_DPA_CODE_FILE_INFO *codeFileInfo);
#endif

/**
 * Return actual status of DPA support library
 * @return actual status of DPA support library
 */
#define DPA_GetStatus()   dpaControl.status

/**
 * Set pointer to user DPA response handler
 * @param A1 Pointer to user DPA response handler
 */
#define DPA_SetAnswerHandler(A1) dpaControl.dpaAnswerHandler = A1

/**
 * Set pointer to user DPA timeout handler
 * @param A1 Pointer to user DPA timeout handler
 */
#define DPA_SetTimeoutHandler(A1) dpaControl.dpaTimeoutHandler = A1

/**
 * Set pointer to user DPA timeout timer
 * @param A1 Pointer to user DPA timeout timer
 */
#define DPA_SetTimeoutTimer(A1)    dpaControl.timeoutTimer = A1

/**
 * Return value of file byte couter used in DPA_StoreCodeToEeeprom(...) function
 * @return value of file byte couter used in DPA_StoreCodeToEeeprom(...) function
 */
#define DPA_GetFileByteCounter() dpaControl.fileByteCounter

/**
 * Increments value of file byte couter used in DPA_StoreCodeToEeeprom(...) function
 */
#define DPA_IncFileByteCounter()    dpaControl.fileByteCounter++

#ifdef __cplusplus
}
#endif

#endif
