/*
 * Copyright 2015-2017 MICRORISC s.r.o.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*****************************************************************************
 *
 * DPA support library ver.1.00
 *
 *****************************************************************************
*/
#ifndef _DPA_LIBRARY_H
#define _DPA_LIBRARY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

#define __SPI_INTERFACE__				    // select for comunication via SPI
// #define __UART_INTERFACE__			  // select for comunication via UART

// #define __STORE_CODE_SUPPORT__		// uncomment for TR7xD modules code upload support

#define systemDisableInt()  noInterrupts()        // disable Interrupts (Arduino platform)
#define systemEnableInt()   interrupts()          // enable Interrupts (Arduino platform)

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

typedef struct{
	uint16_t  	NADR;
  uint8_t   	PNUM;
  uint8_t   	PCMD;
  uint16_t  	HWPID;
  uint8_t 		ResponseCode;
 	uint8_t 		DpaValue;
	TDpaMessage	DpaMessage;
} T_DPA_PACKET;

typedef void (*T_DPA_ANSWER_HANDLER)(T_DPA_PACKET *DpaAnswer);			// DPA response callback function type
typedef void (*T_DPA_TIMEOUT_HANDLER)(void);                        // DPA timeout callback function type

typedef struct{
	volatile uint8_t	Status;
  uint8_t SuspendFlag;
	uint8_t BroadcastRoutingFlag;
  uint8_t TRmoduleSelected;
	uint8_t	TimeCnt;
	uint8_t	ExtraDataSize;
	uint8_t	TimeoutPrescaller;
	uint8_t	TimeoutModulator;
	uint16_t	TimeoutTimer;
	uint16_t	FileByteCounter;
	T_DPA_ANSWER_HANDLER	DpaAnswerHandler;
	T_DPA_TIMEOUT_HANDLER   DpaTimeoutHandler;
	T_DPA_PACKET	*DpaRequestPacketPtr;
}T_DPA_CONTROL;

extern T_DPA_CONTROL	DpaControl;

#if defined (__STORE_CODE_SUPPORT__)

#define DPA_CODE_FILE_NOT_DEFINED	0
#define DPA_CODE_FILE_HEX			    1
#define DPA_CODE_FILE_IQRF			  2

#define	DPA_STORE_CODE_SUCCESS		111			// dpaStoreCodeToEeeprom return code (operation ended with success)
#define	DPA_STORE_CODE_ERROR		  222			// dpaStoreCodeToEeeprom return code (operation ended with error)

typedef struct{
	uint16_t	TrAddress;							  // DPA address of destination TR module
	uint16_t	ImageEeepromAdr;					// absolute address in TR eeeprom to store code
	uint16_t	ImageSize;							  // size of code image stored in eeeprom
	uint16_t	ImageCRC;							    // initial CRC value (before save) / CRC of code image (after save)
	uint16_t	FileSize;							    // size of code file on SD card
	uint8_t	FileType;							      // file type (HEX / IQRF)
}T_DPA_CODE_FILE_INFO;

extern uint16_t DpaAditionalTimeout;

#endif

/*
***************************************************************************************************
* Function: void dpaInit(T_DPA_ANSWER_HANDLER asyncPacketHandler)
*
* PreCondition: none
*
* Input: pointer to user call back function for asynchronous packet service
*
* Output: none
*
* Side Effects: none
*
* Overview: function initialize DPA support library
*
* Note: none
*
***************************************************************************************************
*/
void dpaInit(T_DPA_ANSWER_HANDLER asyncPacketHandler);

/*
***************************************************************************************************
* Function: void dpaLibraryDriver(void)
*
* PreCondition: dpaInit() for library initialization must be called before
*
* Input: none
*
* Output: none
*
* Side Effects: none
*
* Overview: function provides background communication with TR module
*
* Note: none
*
***************************************************************************************************
*/
void dpaLibraryDriver(void);

/*
***************************************************************************************************
* Function: uint8_t dpaSendRequest(T_DPA_PACKET *DpaRequest, uint8_t DataSize, uint16_t Timeout)
*
* PreCondition: dpaInit() for library initialization must be called before
*
* Input: DpaRequest	- pointer to DPA request packet
*        DataSize  	- number of additional data bytes in DPA request packet
*        Timeout    - operation timeout in ms
*
* Output: operation result
*           - DPA_OPERATION_OK            0
*           - DPA_OPERATION_IN_PROGRESS   1
*           - DPA_OPERATION_TIMEOUT       2
*           - DPA_CONFIRMATION_ERR        3
*           - DPA_RESPONSE_ERR            4
*           - DPA_TR_MODULE_NOT_READY     5
*
* Side Effects: none
*
* Overview: sends DPA request packet to desired destination address
*
* Note: none
*
***************************************************************************************************
*/
uint8_t dpaSendRequest(T_DPA_PACKET *DpaRequest, uint8_t DataSize, uint16_t Timeout);

/*
***************************************************************************************************
* Function: uint8_t dpaMakeConfigurationCRC(T_DPA_PACKET *DpaRequest)
*
* PreCondition: dpaInit() for library initialization must be called before
*
* Input: DpaRequest	- pointer to DPA request packet
*
* Output: configuration data CRC
*
* Side Effects: none
*
* Overview: makes CRC from TR module configuration data
*
* Note: none
*
***************************************************************************************************
*/
uint8_t dpaMakeConfigurationCRC(T_DPA_PACKET *DpaRequest);

/*
***************************************************************************************************
* Function: void dpaSuspendDriver(void)
*
* PreCondition: dpaInit(.. ) for library initialization must be called before
*
* Input: none
*
* Output: none
*
* Side Effects: none
*
* Overview: temporary suspend DPA comunication driver
*
* Note: none
*
***************************************************************************************************
*/
void dpaSuspendDriver(void);

/*
***************************************************************************************************
* Function: void dpaRunDriver(void)
*
* PreCondition: dpaInit(.. ) for library initialization must be called before
*
* Input: none
*
* Output: none
*
* Side Effects: none
*
* Overview: run DPA comunication driver
*
* Note: none
*
***************************************************************************************************
*/
void dpaRunDriver(void);

/*
***************************************************************************************************
* Function: uint8_t dpaStoreCodeToEeeprom(T_DPA_CODE_FILE_INFO *CodeFileInfo)
*
* PreCondition: dpaInit() for library initialization must be called before
*               T_DPA_CODE_FILE_INFO *CodeFileInfo must be initialized
*
* Input: 		CodeFileInfo  pointer to T_DPA_CODE_FILE_INFO structure with code file image information
*
* Output:		Progress status or operation result
*               0 - 100 -> progress status
*							  DPA_STORE_CODE_SUCCESS   -> operation ended with success
*               DPA_STORE_CODE_ERROR     -> operation ended with error
*
* Overview: Function for store HEX or IQRF code image to external EEPROM in TR module
*
* Note: function must be called periodicaly until DPA_STORE_CODE_SUCCESS or DPA_STORE_CODE_ERROR is returned
*
***************************************************************************************************
*/
#if defined (__STORE_CODE_SUPPORT__)
uint8_t dpaStoreCodeToEeeprom(T_DPA_CODE_FILE_INFO *CodeFileInfo);
#endif

/*
***************************************************************************************************
* Macro: dpaIncFileByteCounter(void)
*
* PreCondition: none
*
* Input: none
*
* Output: none
*
* Side Effects: increments value of file byte couter used in dpaStoreCodeToEeeprom(...) function
*
* Note: none
*
***************************************************************************************************
*/
#define dpaIncFileByteCounter()	DpaControl.FileByteCounter++

#ifdef __cplusplus
}
#endif

#endif
