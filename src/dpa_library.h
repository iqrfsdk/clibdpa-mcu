/**
 * @file DPA support library
 * @version 1.01
 *
 * Copyright 2015-2017 IQRF Tech s.r.o.
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

#if defined(__cplusplus)
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define __SPI_INTERFACE__       // select for comunication via SPI
//#define __UART_INTERFACE__    // select for comunication via UART

#define __STORE_CODE_SUPPORT__  // uncomment for TR7xD modules code upload support

#define systemDisableInt() noInterrupts() // disable Interrupts (Arduino platform)
#define systemEnableInt()  interrupts()   // enable Interrupts (Arduino platform)

#include "DPA.h"

// Address of the DPA Custom Handler
#define  CUSTOM_HANDLER_ADDRESS    0x3A20

// Address of the DPA Custom Handler end + 1
#define CUSTOM_HANDLER_ADDRESS_END  0x3D80

// dpaSendRequest(...  ) function return codes
#define DPA_OPERATION_OK            0
#define DPA_OPERATION_IN_PROGRESS   1
#define DPA_OPERATION_TIMEOUT       2
#define DPA_CONFIRMATION_ERR        3
#define DPA_RESPONSE_ERR            4
#define DPA_TR_MODULE_NOT_READY     5

typedef struct {
	uint16_t NADR;
	uint8_t PNUM;
	uint8_t PCMD;
	uint16_t HWPID;
	uint8_t ResponseCode;
	uint8_t DpaValue;
	TDpaMessage DpaMessage;
} T_DPA_PACKET;

typedef void (*T_DPA_ANSWER_HANDLER)(T_DPA_PACKET *DpaAnswer); // DPA response callback function type
typedef void (*T_DPA_TIMEOUT_HANDLER)(void); // DPA timeout callback function type

typedef struct {
	volatile uint8_t Status;
	uint8_t SuspendFlag;
	uint8_t BroadcastRoutingFlag;
	uint8_t TRmoduleSelected;
	uint8_t TimeCnt;
	uint8_t ExtraDataSize;
	uint8_t RdExtraDataSize;
	uint8_t TimeoutPrescaller;
	uint8_t TimeoutModulator;
	uint16_t TimeoutTimer;
	uint16_t FileByteCounter;
	T_DPA_ANSWER_HANDLER DpaAnswerHandler;
	T_DPA_TIMEOUT_HANDLER DpaTimeoutHandler;
	T_DPA_PACKET *DpaRequestPacketPtr;
} T_DPA_CONTROL;

extern T_DPA_CONTROL DpaControl;

#if defined(__STORE_CODE_SUPPORT__)

#define DPA_CODE_FILE_NOT_DEFINED   0
#define DPA_CODE_FILE_HEX           1
#define DPA_CODE_FILE_IQRF          2

#define DPA_STORE_CODE_SUCCESS    111     // dpaStoreCodeToEeeprom return code (operation ended with success)
#define DPA_STORE_CODE_ERROR      222     // dpaStoreCodeToEeeprom return code (operation ended with error)

typedef struct {
	uint16_t TrAddress; // DPA address of destination TR module
	uint16_t ImageEeepromAdr; // absolute address in TR eeeprom to store code
	uint16_t ImageSize; // size of code image stored in eeeprom
	uint16_t ImageCRC; // initial CRC value (before save) / CRC of code image (after save)
	uint16_t FileSize; // size of code file on SD card
	uint8_t FileType; // file type (HEX / IQRF)
} T_DPA_CODE_FILE_INFO;

extern uint16_t DpaAditionalTimeout;

#endif

/**
 * Function initialize DPA support library
 * @param asyncPacketHandler Pointer to user call back function for asynchronous packet service
 * @return none
 */
void dpaInit(T_DPA_ANSWER_HANDLER asyncPacketHandler);

/**
 * Function provides background communication with TR module
 */
void dpaLibraryDriver(void);

/**
 * Sends DPA request packet to desired destination address
 * @param DpaRequest  pointer to DPA request packet
 * @param DataSize  number of additional data bytes in DPA request packet
 * @param Timeout operation timeout in ms
 * @return operation result (DPA_OPERATION_OK, DPA_OPERATION_IN_PROGRESS, DPA_OPERATION_TIMEOUT ... )
 */
uint8_t dpaSendRequest(T_DPA_PACKET *DpaRequest, uint8_t DataSize, uint16_t Timeout);

/**
 * Makes CRC from TR module configuration data
 * @param DpaRequest Pointer to DPA request packet
 * @return Configuration data CRC
 */
uint8_t dpaMakeConfigurationCRC(T_DPA_PACKET *DpaRequest);

/**
 * Temporary suspend DPA comunication driver
 */
void dpaSuspendDriver(void);

/**
 * Run DPA comunication driver
 */
void dpaRunDriver(void);

/**
 * Function for store HEX or IQRF code image to external EEPROM in TR module
 * @param CodeFileInfo  pointer to T_DPA_CODE_FILE_INFO structure with code file image information
 * @return  Proggess status or operation result (0 - 100 -> progress status, DPA_STORE_CODE_SUCCESS, DPA_STORE_CODE_ERROR)
 */
#if defined(__STORE_CODE_SUPPORT__)
uint8_t dpaStoreCodeToEeeprom(T_DPA_CODE_FILE_INFO *CodeFileInfo);
#endif

/**
 * Macro: increments value of file byte couter used in dpaStoreCodeToEeeprom(...) function
 */
#define dpaIncFileByteCounter() DpaControl.FileByteCounter++

/**
 * Macro: return size of extra data received from TR module
 */
#define dpaGetRxExtraDataSize() DpaControl.RdExtraDataSize

#if defined(__cplusplus)
}
#endif

#endif
