/*
 * Copyright 2015-2016 MICRORISC s.r.o.
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
 * DPA support library ver.0.96
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

#define __STORE_CODE_SUPPORT__		  // uncomment for TR7xD modules code upload support 

#define TR7xD							          // select for TR7xD module
//#define TR5xD							          // select for TR5xD module

#define systemDisableInt()  noInterrupts()        // disable Interrupts (Arduino platform)
#define systemEnableInt()   interrupts()          // enable Interrupts (Arduino platform)

#include "DPA.h"

// Address of the DPA Custom Handler
#define  CUSTOM_HANDLER_ADDRESS    0x3A20

// Address of the DPA Custom Handler end + 1
#if defined( TR7xD )
#define CUSTOM_HANDLER_ADDRESS_END  0x3D80
#elif defined ( TR5xD )
#define CUSTOM_HANDLER_ADDRESS_END  0x3D00
#else
#error Unsupported DCTR type
#endif

typedef uint8_t  UINT8;				      // Define dpa_library data types
typedef uint16_t UINT16;

// dpaSendRequest(...  ) function return codes
#define DPA_OPERATION_OK            0
#define DPA_OPERATION_IN_PROGRESS   1
#define DPA_OPERATION_TIMEOUT       2
#define DPA_CONFIRMATION_ERR        3
#define DPA_RESPONSE_ERR            4
#define DPA_TR_MODULE_NOT_READY     5

typedef struct{
	UINT16  	NADR;
  UINT8   	PNUM;
  UINT8   	PCMD;
  UINT16  	HWPID;
  UINT8 		ResponseCode;
 	UINT8 		DpaValue;
	TDpaMessage	DpaMessage;
} T_DPA_PACKET;

typedef void (*T_DPA_ANSWER_HANDLER)(T_DPA_PACKET *DpaAnswer);			// DPA response callback function type
typedef void (*T_DPA_TIMEOUT_HANDLER)(void);                        // DPA timeout callback function type

typedef struct{
	volatile UINT8	Status;
  UINT8 SuspendFlag;
  UINT8 TRmoduleSelected;
	UINT8	TimeCnt;
	UINT8	ExtraDataSize;
	UINT8	TimeoutPrescaller;
	UINT8	TimeoutModulator;
	UINT16	TimeoutTimer;
	UINT16	FileByteCounter;						
	T_DPA_ANSWER_HANDLER	DpaAnswerHandler;
	T_DPA_TIMEOUT_HANDLER   DpaTimeoutHandler;
	T_DPA_PACKET	*DpaRequestPacketPtr;
}T_DPA_CONTROL;

extern T_DPA_CONTROL	DpaControl;

#if defined (__STORE_CODE_SUPPORT__) && defined(TR7xD)

#define DPA_CODE_FILE_NOT_DEFINED	0
#define DPA_CODE_FILE_HEX			    1
#define DPA_CODE_FILE_IQRF			  2

#define	DPA_STORE_CODE_SUCCESS		111			// dpaStoreCodeToEeeprom return code (operation ended with success)
#define	DPA_STORE_CODE_ERROR		  222			// dpaStoreCodeToEeeprom return code (operation ended with error)

typedef struct{
	UINT16	TrAddress;							  // DPA address of destination TR module
	UINT16	ImageEeepromAdr;					// absolute address in TR eeeprom to store code
	UINT16	ImageSize;							  // size of code image stored in eeeprom
	UINT16	ImageCRC;							    // initial CRC value (before save) / CRC of code image (after save)
	UINT16	FileSize;							    // size of code file on SD card
	UINT8	FileType;							      // file type (HEX / IQRF)
}T_DPA_CODE_FILE_INFO;

extern UINT16 DpaAditionalTimeout;

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
* Function: UINT8 dpaSendRequest(T_DPA_PACKET *DpaRequest, UINT8 DataSize, UINT16 Timeout)
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
UINT8 dpaSendRequest(T_DPA_PACKET *DpaRequest, UINT8 DataSize, UINT16 Timeout);

/*
***************************************************************************************************
* Function: UINT8 dpaMakeConfigurationCRC(T_DPA_PACKET *DpaRequest)
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
UINT8 dpaMakeConfigurationCRC(T_DPA_PACKET *DpaRequest);

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
* Function: UINT8 dpaStoreCodeToEeeprom(T_DPA_CODE_FILE_INFO *CodeFileInfo)
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
#if defined (__STORE_CODE_SUPPORT__) && defined(TR7xD)
UINT8 dpaStoreCodeToEeeprom(T_DPA_CODE_FILE_INFO *CodeFileInfo);
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

