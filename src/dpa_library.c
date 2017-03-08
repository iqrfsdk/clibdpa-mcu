
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

/*
 *****************************************************************************
 *
 * DPA support library ver.1.00
 *
 *****************************************************************************
*/
#include <Arduino.h>
#include <ctype.h>
#include "dpa_library.h"

#ifdef __SPI_INTERFACE__

#define SPI_TRANSFER_NONE			0
#define SPI_TRANSFER_WRITE		1
#define SPI_TRANSFER_READ			2

#define NO_MODULE           0xFF  // SPI not working (HW error)
#define SPI_BUSY            0xFE  // SPI busy in Master disabled mode
#define SPI_DATA_TRANSFER   0xFD  // SPI data transfer in progress
#define SPI_DISABLED        0x00  // SPI not working (disabled)
#define SPI_CRCM_OK         0x3F  // SPI not ready (full buffer, last CRCM ok)
#define SPI_CRCM_ERR        0x3E  // SPI not ready (full buffer, last CRCM error)
#define COMMUNICATION_MODE  0x80  // SPI ready (communication mode)
#define PROGRAMMING_MODE    0x81  // SPI ready (programming mode)
#define DEBUG_MODE          0x82  // SPI ready (debugging mode)
#define SPI_SLOW_MODE       0x83  // SPI not working in background
#define SPI_USER_STOP       0x07  // state after stopSPI();

#define SPI_CHECK  			0x00    // Master checks the SPI status of the TR module
#define SPI_WR_RD 	 		0xF0		// Master reads/writes a packet from/to TR module
#define SPI_CRCM_OK     0x3F    // SPI not ready (full buffer, last CRCM ok)
#define SPI_CRCM_ERR    0x3E    // SPI not ready (full buffer, last CRCM error)

#define SPI_STATUS_POOLING_TIME    10    // SPI status pooling time 10ms

typedef struct{                 // SPI interface control structure
	uint8_t	DLEN;
	uint8_t	CMD;
	uint8_t	PTYPE;
	uint8_t	CRCM;
	uint8_t	MyCRCS;
	uint8_t	CRCS;
	uint8_t	SpiStat;
	uint8_t	Direction;
	uint8_t	PacketLen;
	uint8_t	PacketCnt;
	uint8_t PacketRpt;
	T_DPA_PACKET	*DpaPacketPtr;
} T_DPA_SPI_INTERFACE_CONTROL;

T_DPA_SPI_INTERFACE_CONTROL		DpaSpiIfControl;

uint8_t dpaSendSpiByte(uint8_t Tx_Byte);
void dpaDeselectTRmodule(void);
void dpaSpiInterfaceDriver(void);
uint8_t dpaGetCRCM(void);

#endif

#ifdef __UART_INTERFACE__

#define UART_TRANSFER_NONE			0
#define UART_TRANSFER_WRITE			1
#define UART_TRANSFER_READ			2

#define HDLC_FRM_FLAG_SEQUENCE		0x7E
#define HDLC_FRM_CONTROL_ESCAPE		0x7D
#define HDLC_FRM_ESCAPE_BIT			0x20

typedef struct{
	uint8_t	Direction;
	uint8_t	PacketCnt;
	uint8_t	PacketLen;
	uint8_t	CRC;
	uint8_t	WasEscape;
	T_DPA_PACKET	*DpaPacketPtr;
} T_DPA_UART_INTERFACE_CONTROL;

T_DPA_UART_INTERFACE_CONTROL		DpaUartIfControl;

void dpaSendUartByte(uint8_t Tx_Byte);
uint8_t dpaReceiveUartByte(uint8_t *Rx_Byte);
void dpaUartInterfaceDriver(void);
void dpaSendDataByte(uint8_t DataByte);
uint8_t dpaDoCRC8(uint8_t InData, uint8_t Seed);

#endif

#if defined (__STORE_CODE_SUPPORT__)

uint16_t dpaFlatcherCRC16(uint8_t *DataAddress, uint8_t DataLen, uint16_t Seed);
void dpaSendDataToEeeprom(T_DPA_CODE_FILE_INFO *CodeFileInfo, uint8_t DataSize);
void dpaProcessHexCodeFile(T_DPA_CODE_FILE_INFO *CodeFileInfo);
void dpaProcessIqrfCodeFile(T_DPA_CODE_FILE_INFO *CodeFileInfo);
uint8_t dpaConvertToNum(uint8_t DataByteHi, uint8_t DataByteLo);
uint8_t dpaReadHEXFileLine(void);
uint8_t dpaReadIQRFFileLine(void);
extern uint8_t dpaReadByteFromFile(void);

T_DPA_PACKET	DpaStoreCodeRequest;
uint8_t DpaCodeLineBuffer[32];
uint8_t	DpaStartAddressDetected;
uint8_t	DpaCodeImageEndDetected;
uint8_t	DpaOutputPacketDataCnt;
uint8_t	DpaEeepromPageDataCnt;
uint8_t DpaRemainingDataCnt;
uint16_t  DpaCodeLineAddress;
uint16_t  DpaCodeImageNextAddress;

enum {
	DPA_STORE_CODE_INIT_TASK = 0,
	DPA_STORE_CODE_SEND_DATA,
  DPA_STORE_CODE_ERROR_END,
	DPA_STORE_CODE_SUCCESS_END
} DpaStoreCodeTaskSM = DPA_STORE_CODE_INIT_TASK;

#endif

#define DPA_SM_PREPARE_REQUEST      0    // internal states of DPA operation state machine
#define DPA_SM_WAIT_CONFIRMATION    1
#define DPA_SM_PROCESS_CONFIRMATION 2
#define DPA_SM_CONFIRMATION_ERR     3
#define DPA_SM_WAIT_RESPONSE        4
#define DPA_SM_PROCESS_RESPONSE     5
#define DPA_SM_RESPONSE_ERR         6
#define DPA_SM_WAIT_BROADCAST_ROUTING	7
#define DPA_SM_TIMEOUT              8

#define  DPA_READY      0x00        // DPA support library ready
#define  DPA_NOT_READY  0x01        // DPA support library not ready
#define  DPA_BUSY       0x02        // DPA request processing

#define dpaSetTimeoutTimer(A1)  DpaControl.TimeoutTimer = A1

/*
 *      function prototypes
 */
void dpaAnswerHandler(T_DPA_PACKET *dpaAnswerPkt);
void dpaTimeoutHandler(void);
uint16_t dpaGetEstimatedTimeout(void);

/*
 *		 	public variable declarations
 */
T_DPA_CONTROL		DpaControl;
T_DPA_PACKET 		DpaLibDpaAnswer;
T_DPA_ANSWER_HANDLER tempReceiverHandler;

volatile uint8_t DpaApplicationSM = DPA_SM_PREPARE_REQUEST;
uint16_t  DpaAditionalTimeout;

/**
 * Function initialize DPA support library
 * @param  asyncPacketHandler pointer to user call back function for asynchronous packet service
 * @return: none
 */
void dpaInit(T_DPA_ANSWER_HANDLER asyncPacketHandler)
{

	DpaControl.Status = DPA_READY;
	DpaControl.DpaAnswerHandler = asyncPacketHandler;
	DpaControl.DpaTimeoutHandler = dpaTimeoutHandler;
	DpaControl.TimeoutTimer = 0;
  DpaControl.TimeoutModulator = 0;
	DpaControl.TimeoutPrescaller = 7;

#ifdef __SPI_INTERFACE__
  DpaControl.TRmoduleSelected = false;
	DpaControl.TimeCnt = (SPI_STATUS_POOLING_TIME * 7) + 1;
	DpaSpiIfControl.SpiStat = 0;
	DpaSpiIfControl.Direction = SPI_TRANSFER_NONE;
#endif

#ifdef __UART_INTERFACE__
	DpaUartIfControl.Direction = UART_TRANSFER_NONE;
	DpaUartIfControl.WasEscape = 0;
#endif
}

/**
 * Function provides background communication with TR module
 */
void dpaLibraryDriver(void)
{

  if (DpaControl.SuspendFlag == true) return;

#ifdef __SPI_INTERFACE__
	if (DpaControl.Status == DPA_BUSY || !DpaControl.TimeCnt){
		dpaSpiInterfaceDriver();
	  DpaControl.TimeCnt = (SPI_STATUS_POOLING_TIME * 7) + 1;
	}
	DpaControl.TimeCnt--;
#endif

#ifdef __UART_INTERFACE__
	dpaUartInterfaceDriver();
#endif

  // prescaler 7 = 1050us , prescaler 6 = 900us
  if (--DpaControl.TimeoutPrescaller) return;

	// every third round do timeout timer correction
  if (++DpaControl.TimeoutModulator <= 2) DpaControl.TimeoutPrescaller = 7;
  else{
    DpaControl.TimeoutModulator = 0;		// 2 * 1050us + 900us = 3000us
    DpaControl.TimeoutPrescaller = 6;
  }

  // service timeout timer
  if (DpaControl.TimeoutTimer){
		// if timeout expired
    if (!(--DpaControl.TimeoutTimer)){
			// call user timeout handler
      if (DpaControl.DpaTimeoutHandler != NULL) DpaControl.DpaTimeoutHandler();
    }
  }

}

/**
 * Sends DPA request packet to desired destination address
 * @param DpaRequest  pointer to DPA request packet
 * @param DataSize  number of additional data bytes in DPA request packet
 * @param Timeout operation timeout in ms
 * @return operation result (DPA_OPERATION_OK, DPA_OPERATION_IN_PROGRESS, DPA_OPERATION_TIMEOUT ... )
 */
uint8_t dpaSendRequest(T_DPA_PACKET *DpaRequest, uint8_t DataSize, uint16_t Timeout)
{

  uint8_t OperationResult = DPA_OPERATION_IN_PROGRESS;

  // DPA operation state machine
	switch (DpaApplicationSM){

    case DPA_SM_PREPARE_REQUEST:{
			// if TR module is not ready
			if (DpaControl.Status == DPA_NOT_READY) return(DPA_TR_MODULE_NOT_READY);

      // wait until library is ready
			while (DpaControl.Status == DPA_BUSY);

      systemDisableInt();                                                       // disable interrupts
      tempReceiverHandler = DpaControl.DpaAnswerHandler;                        // save pointer to async packet user service rutine
      DpaControl.DpaAnswerHandler = dpaAnswerHandler;                           // set pointer to dpa system receiver handler

      DpaControl.DpaRequestPacketPtr = DpaRequest;                              // initialize transfer parameters
      DpaControl.ExtraDataSize = DataSize;

      dpaSetTimeoutTimer(Timeout);                                              // set operation timeout
      systemEnableInt();                                                        // enable interrupts

      // if command for coordinator or local interface,  wait for response
      if (DpaRequest->NADR==COORDINATOR_ADDRESS || DpaRequest->NADR==LOCAL_ADDRESS) DpaApplicationSM = DPA_SM_WAIT_RESPONSE;
      else DpaApplicationSM = DPA_SM_WAIT_CONFIRMATION;                         // if command for node, wait for confirmation

			DpaControl.BroadcastRoutingFlag = false;
    }
    break;

    // confimation received OK
		case DPA_SM_PROCESS_CONFIRMATION:{
      if (DpaControl.BroadcastRoutingFlag == true) DpaApplicationSM = DPA_SM_WAIT_BROADCAST_ROUTING;    // wait for broadcat routing
			else DpaApplicationSM = DPA_SM_WAIT_RESPONSE;	                       															// wait for response
    }
    break;

    // response received OK
		case DPA_SM_PROCESS_RESPONSE:{
			// copy received DPA packet to user DPA packet structure
      memcpy((uint8_t *)DpaRequest, (uint8_t *)&DpaLibDpaAnswer, sizeof(T_DPA_PACKET));
      OperationResult = DPA_OPERATION_OK;
    }
    break;

		// waiting for confirmation, response or broadcast routing
		case DPA_SM_WAIT_CONFIRMATION:
    case DPA_SM_WAIT_RESPONSE:
		case DPA_SM_WAIT_BROADCAST_ROUTING:
    break;

		// confirmation error
		case DPA_SM_CONFIRMATION_ERR: OperationResult = DPA_CONFIRMATION_ERR; break;
		// response error
		case DPA_SM_RESPONSE_ERR: OperationResult = DPA_RESPONSE_ERR; break;
		// operation timeout
    case DPA_SM_TIMEOUT:{
			if (DpaControl.BroadcastRoutingFlag == true) OperationResult = DPA_OPERATION_OK;
			else OperationResult = DPA_OPERATION_TIMEOUT;
		}
		break;
  }

  if (OperationResult != DPA_OPERATION_IN_PROGRESS){
    systemDisableInt();                                                         // disable interrupts
    DpaControl.DpaAnswerHandler = tempReceiverHandler;                          // restore pointer to async packet user service rutine
    systemEnableInt();                                                          // enable interrupts
    DpaApplicationSM = DPA_SM_PREPARE_REQUEST;                                  // DPA operation end, initialize state machine for next request
  }

  return(OperationResult);
}

/**
 * Returns estimated timeout for response packet in miliseconds
 * @return Estimated timeout for response packet in ms (computed from confirmation packet data)
 */
uint16_t dpaGetEstimatedTimeout(void)
{

	uint16_t EstimatedTimeout;
  uint16_t ResponseTimeSlotLength;

	EstimatedTimeout = (uint16_t)(DpaLibDpaAnswer.DpaMessage.IFaceConfirmation.Hops + 1) * (uint16_t)DpaLibDpaAnswer.DpaMessage.IFaceConfirmation.TimeSlotLength * 10;
	if (DpaLibDpaAnswer.DpaMessage.IFaceConfirmation.TimeSlotLength == 20) ResponseTimeSlotLength = 200;			// DPA in diagnostic mode
	else{
		if (DpaLibDpaAnswer.DpaMessage.IFaceConfirmation.TimeSlotLength > 6) ResponseTimeSlotLength = 110;			// DPA in LP mode
		else ResponseTimeSlotLength = 60;																			                                  // DPA in STD mode
 	}
	EstimatedTimeout += ((uint16_t)(DpaLibDpaAnswer.DpaMessage.IFaceConfirmation.HopsResponse + 1) * ResponseTimeSlotLength + 40);
	return(EstimatedTimeout);
}

/**
 * Makes CRC from TR module configuration data
 * @param DpaRequest	pointer to DPA request packet
 * @return Configuration data CRC
 */
uint8_t dpaMakeConfigurationCRC(T_DPA_PACKET *DpaRequest)
{

	uint8_t	Cnt;
	uint8_t	Crc = 0x5F;

	for (Cnt=0; Cnt<31; Cnt++){
		Crc ^= DpaRequest -> DpaMessage.PerOSWriteCfg_Request.Configuration[Cnt];
	}

	return(Crc);
}

/**
 * Temporary suspend DPA comunication driver
 */
void dpaSuspendDriver(void)
{
  // wait until library is ready
  while (DpaControl.Status == DPA_BUSY);
  // set driver suspend flag
  DpaControl.SuspendFlag = true;
}

/**
 * Run DPA comunication driver
 */
void dpaRunDriver(void)
{
	// reenable DPA driver running
	DpaControl.SuspendFlag = false;
}

/**
 * Callback function for received packet from TR module check
 * @param  dpaAnswerPkt  Pointer to data structure containing received packet from TR module
 * @return  none
 */
void dpaAnswerHandler(T_DPA_PACKET *dpaAnswerPkt)
{

  dpaSetTimeoutTimer(0);                 // stop timeout timmer

  switch(DpaApplicationSM){
		// waiting for confirmation
    case DPA_SM_WAIT_CONFIRMATION:{
			// if confirmation received
      if (dpaAnswerPkt->ResponseCode == STATUS_CONFIRMATION){
				// process confirmation
        DpaApplicationSM = DPA_SM_PROCESS_CONFIRMATION;
				// set timeout timmer to estimated timeout
        dpaSetTimeoutTimer(dpaGetEstimatedTimeout() + DpaAditionalTimeout);
				// check if we send broadcast request
				if (dpaAnswerPkt->NADR == BROADCAST_ADDRESS) DpaControl.BroadcastRoutingFlag = true;
      }
      else DpaApplicationSM = DPA_SM_CONFIRMATION_ERR;          // process error in confirmation
    }
    break;

		// waiting for response
    case DPA_SM_WAIT_RESPONSE:{
			// if no error received, process response
      if (dpaAnswerPkt->ResponseCode == STATUS_NO_ERROR) DpaApplicationSM = DPA_SM_PROCESS_RESPONSE;
      else DpaApplicationSM = DPA_SM_RESPONSE_ERR;              // process error in response
    }
    break;

    default:;
  }
}

/**
 * Callback function for service timeout elapsed
 */
void dpaTimeoutHandler(void)
{
  DpaApplicationSM = DPA_SM_TIMEOUT;
}

#ifdef __SPI_INTERFACE__
/**
 * Function implements IQRF packet communication over SPI with TR module
 */
void dpaSpiInterfaceDriver(void)
{

	uint8_t LastSpiStat;
	uint8_t TempData;

  // is anything to send / receive
	if (DpaSpiIfControl.Direction != SPI_TRANSFER_NONE){

		if (DpaSpiIfControl.Direction == SPI_TRANSFER_WRITE){
			switch(DpaSpiIfControl.PacketCnt){
				case 0: dpaSendSpiByte(DpaSpiIfControl.CMD); DpaSpiIfControl.MyCRCS = 0x5F; break; 							                // send SPI CMD
				case 1: dpaSendSpiByte(DpaSpiIfControl.PTYPE); DpaSpiIfControl.MyCRCS ^= DpaSpiIfControl.PTYPE; break; 		      // send PTYPE
				case 2: DpaSpiIfControl.MyCRCS ^= dpaSendSpiByte((DpaSpiIfControl.DpaPacketPtr->NADR) & 0x00FF); break;		      // send LOW(NADR)
				case 3: DpaSpiIfControl.MyCRCS ^= dpaSendSpiByte((DpaSpiIfControl.DpaPacketPtr->NADR) >> 8); break;			        // send HIGH(NADR)
				case 4: DpaSpiIfControl.MyCRCS ^= dpaSendSpiByte(DpaSpiIfControl.DpaPacketPtr->PNUM); break;					          // send PNUM
				case 5: DpaSpiIfControl.MyCRCS ^= dpaSendSpiByte(DpaSpiIfControl.DpaPacketPtr->PCMD); break;					          // send PCMD
				case 6: DpaSpiIfControl.MyCRCS ^= dpaSendSpiByte((DpaSpiIfControl.DpaPacketPtr->HWPID) & 0x00FF); break;		    // send LOW(HWPID)
				case 7: DpaSpiIfControl.MyCRCS ^= dpaSendSpiByte((DpaSpiIfControl.DpaPacketPtr->HWPID) >> 8); break;			      // send HIGH(HWPID)
				default:{
					if (DpaSpiIfControl.PacketCnt == DpaSpiIfControl.PacketLen-2){												                        // send CRCM / receive CRCS
						DpaSpiIfControl.CRCS = dpaSendSpiByte(DpaSpiIfControl.CRCM);
					}
					else{
						if (DpaSpiIfControl.PacketCnt == DpaSpiIfControl.PacketLen-1){											                        // receive SPI_STATUS
							DpaSpiIfControl.SpiStat = dpaSendSpiByte(0);
						}
						else{																									                                                      // send additional data
							DpaSpiIfControl.MyCRCS ^= dpaSendSpiByte(DpaSpiIfControl.DpaPacketPtr->DpaMessage.Request.PData[DpaSpiIfControl.PacketCnt-8]);
						}
					}
				}
			}
		}
		else{
			if (DpaSpiIfControl.PacketCnt>=2 && DpaSpiIfControl.PacketCnt<DpaSpiIfControl.PacketLen-2){
				TempData = dpaSendSpiByte(0);
				DpaSpiIfControl.MyCRCS ^= TempData;
			}

			switch(DpaSpiIfControl.PacketCnt){

				case 0: dpaSendSpiByte(DpaSpiIfControl.CMD); DpaSpiIfControl.MyCRCS = 0x5F; break; 							              // send SPI CMD
				case 1: dpaSendSpiByte(DpaSpiIfControl.PTYPE); DpaSpiIfControl.MyCRCS ^= DpaSpiIfControl.PTYPE; break; 		    // send PTYPE
				case 2: DpaSpiIfControl.DpaPacketPtr->NADR = TempData; break;													                        // receive LOW(NADR)
				case 3: DpaSpiIfControl.DpaPacketPtr->NADR |= ((uint16_t)TempData << 8); break;									              // receive HIGH(NADR)
				case 4: DpaSpiIfControl.DpaPacketPtr->PNUM = TempData; break;													                        // receive PNUM
				case 5: DpaSpiIfControl.DpaPacketPtr->PCMD = TempData; break;													                        // receive PCMD
				case 6: DpaSpiIfControl.DpaPacketPtr->HWPID = TempData; break;													                      // receive LOW(HWPID)
				case 7: DpaSpiIfControl.DpaPacketPtr->HWPID |= ((uint16_t)TempData << 8); break;									            // receive HIGH(HWPID)
				default:{
					if (DpaSpiIfControl.PacketCnt == DpaSpiIfControl.PacketLen-2){												                      // send CRCM / receive CRCS
						DpaSpiIfControl.CRCS = dpaSendSpiByte(DpaSpiIfControl.CRCM);
					}
					else{
						if (DpaSpiIfControl.PacketCnt == DpaSpiIfControl.PacketLen-1){											                      // receive SPI_STATUS
							DpaSpiIfControl.SpiStat = dpaSendSpiByte(0);
						}
						else{
							if (DpaSpiIfControl.PacketCnt == 8) DpaSpiIfControl.DpaPacketPtr->ResponseCode = TempData;			        // receive ResponseCode
							else{
								if (DpaSpiIfControl.PacketCnt == 9) DpaSpiIfControl.DpaPacketPtr->DpaValue = TempData;			          // receive DpaValue
								else DpaSpiIfControl.DpaPacketPtr->DpaMessage.Response.PData[DpaSpiIfControl.PacketCnt-10] = TempData;// receive additional data
							}
						}
					}
				}
			}
		}
		DpaSpiIfControl.PacketCnt++;											                                                                // counts number of send/receive bytes

		if (DpaSpiIfControl.PacketCnt == DpaSpiIfControl.PacketLen){			                                                // sent everything ?
			dpaDeselectTRmodule();
			if ((DpaSpiIfControl.SpiStat == SPI_CRCM_OK) && (DpaSpiIfControl.CRCS == DpaSpiIfControl.MyCRCS)){	            // CRC ok ?
				if (DpaSpiIfControl.Direction == SPI_TRANSFER_READ && DpaControl.DpaAnswerHandler != NULL){
					DpaControl.DpaAnswerHandler(DpaSpiIfControl.DpaPacketPtr);                                                  // call user response handler
				}
				DpaControl.Status = DPA_READY;									                                                              // library is ready for next packet
				DpaSpiIfControl.Direction = SPI_TRANSFER_NONE;					                                                      // stop data transfer
			}
			else{																                                                                            // CRC error
				if (--DpaSpiIfControl.PacketRpt){								                                                              // pktRepeats - must be set on packet preparing
					DpaSpiIfControl.PacketCnt = 0;								                                                              // another attempt to send data
				}
				else{
					DpaControl.Status = DPA_READY;								                                                              // library is ready for next packet
					DpaSpiIfControl.Direction = SPI_TRANSFER_NONE;				                                                      // stop data transfer
				}
			}

			DpaSpiIfControl.SpiStat = 0;										                                                                // current SPI status must be updated
		}
	}
	else{																		                                                                            // no data to send => SPI status will be updated

		LastSpiStat = dpaSendSpiByte(SPI_CHECK);								                                                          // get SPI status of TR module
		dpaDeselectTRmodule();

 		if (DpaSpiIfControl.SpiStat != LastSpiStat){							                                                        // the status must be 2x the same
    		DpaSpiIfControl.SpiStat = LastSpiStat;
  			return;
		}

    if (DpaSpiIfControl.SpiStat == COMMUNICATION_MODE) DpaControl.Status = DPA_READY;                                 // if TR module is in communication mode, DPA library is ready
    else DpaControl.Status = DPA_NOT_READY;

 	  if ((DpaSpiIfControl.SpiStat & 0xC0) == 0x40){    						                                                    // if the status is dataready, prepare packet to read it
		  if (DpaSpiIfControl.SpiStat == 0x40) DpaSpiIfControl.DLEN = 64;                                                 // stav 0x40 znamena nabidku 64B
		  else DpaSpiIfControl.DLEN = DpaSpiIfControl.SpiStat & 0x3F; 		                                                // clear bit 7,6 - rest is length (1 az 63B)
			DpaSpiIfControl.DpaPacketPtr = &DpaLibDpaAnswer;					                                                      // set pointer to DPA receive structure
      DpaSpiIfControl.CMD = SPI_WR_RD;									                                                              // read / write data
      DpaSpiIfControl.PTYPE = DpaSpiIfControl.DLEN;
      DpaSpiIfControl.CRCM = 0x5F ^ DpaSpiIfControl.CMD ^ DpaSpiIfControl.PTYPE;		                                  // CRCM

      DpaSpiIfControl.PacketLen = DpaSpiIfControl.DLEN + 4;				                                                    // length of whole packet + (CMD, PTYPE, CRCM, 0)
			DpaSpiIfControl.PacketCnt = 0;										                                                              // counter of sent bytes
			DpaSpiIfControl.PacketRpt = 1;										                                                              // number of attempts to send data

			DpaSpiIfControl.Direction = SPI_TRANSFER_READ;						                                                      // reading from buffer COM of TR module
			DpaSpiIfControl.SpiStat = 0;										                                                                // current SPI status must be updated
			DpaControl.Status = DPA_BUSY;										                                                                // library si busy
			return;
		}

		if (DpaControl.DpaRequestPacketPtr != NULL){							                                                        // check if packet to send is ready
			DpaSpiIfControl.DpaPacketPtr = DpaControl.DpaRequestPacketPtr;		                                              // set pointer to DpaRequest packet
			DpaControl.DpaRequestPacketPtr = NULL;

			DpaSpiIfControl.DLEN = DpaControl.ExtraDataSize + 6;				                                                    // NADR + PNUM + PCMD + HWPID + Data
      DpaSpiIfControl.CMD = SPI_WR_RD;
      DpaSpiIfControl.PTYPE = (DpaSpiIfControl.DLEN | 0x80);				                                                  // PBYTE set bit7 - write to buffer COM of TR module
			DpaSpiIfControl.CRCM = dpaGetCRCM();								                                                            // CRCM

      DpaSpiIfControl.PacketLen = DpaSpiIfControl.DLEN + 4;				                                                    // length of whole packet + (CMD, PTYPE, CRCM, 0)
			DpaSpiIfControl.PacketCnt = 0;										                                                              // counter of sent bytes
			DpaSpiIfControl.PacketRpt = 3;										                                                              // number of attempts to send data

			DpaSpiIfControl.Direction = SPI_TRANSFER_WRITE;						                                                      // reading from buffer COM of TR module
			DpaSpiIfControl.SpiStat = 0;										                                                                // current SPI status must be updated
			DpaControl.Status = DPA_BUSY;										                                                                // library si busy
		}
  }
}

/**
 * Calculate CRC before master's send (see IQRF SPI user manual)
 * @param none
 * @return CrcVal
 */
uint8_t dpaGetCRCM(void)
{

 	unsigned char Cnt, CrcVal, DataSize;

	CrcVal = 0x5F;													                                    // initialize CRC
	CrcVal ^= DpaSpiIfControl.CMD;									                            // add SPI CMD
	CrcVal ^= DpaSpiIfControl.PTYPE;								                            // add PTYPE
	CrcVal ^= DpaSpiIfControl.DpaPacketPtr->NADR & 0x00FF;			                // add LOW(NADR)
	CrcVal ^= DpaSpiIfControl.DpaPacketPtr->NADR >> 8;				                  // add HIGH(NADR)
	CrcVal ^= DpaSpiIfControl.DpaPacketPtr->PNUM;					                      // add PNUM
	CrcVal ^= DpaSpiIfControl.DpaPacketPtr->PCMD;					                      // add PCMD
	CrcVal ^= DpaSpiIfControl.DpaPacketPtr->HWPID & 0x00FF;		                  // add LOW(HWPID)
	CrcVal ^= DpaSpiIfControl.DpaPacketPtr->HWPID >> 8;			                    // add HIGH(HWPID)

	DataSize = DpaSpiIfControl.DLEN - 6;							                          // number of extra data bytes (except NADR, PNUM, PCMD, HWPID)
	for (Cnt=0; Cnt<DataSize; Cnt++){
		CrcVal ^= DpaSpiIfControl.DpaPacketPtr->DpaMessage.Request.PData[Cnt];
	}

	return CrcVal;
}

#endif


#ifdef __UART_INTERFACE__
/**
 * Function implements IQRF packet communication over UART with TR module
 */
void dpaUartInterfaceDriver(void)
{

	uint8_t TempData;

  // is anything to send / receive
	if (DpaUartIfControl.Direction != UART_TRANSFER_NONE){

		if (DpaUartIfControl.Direction == UART_TRANSFER_READ){
			// no data in Rx buffer
      if (dpaReceiveUartByte(&TempData)==false) return;

			// end of packet or DPA structure is full
			if (TempData == HDLC_FRM_FLAG_SEQUENCE || DpaUartIfControl.PacketCnt == DpaUartIfControl.PacketLen){
				if (DpaUartIfControl.CRC == 0 && DpaControl.DpaAnswerHandler != NULL){
					// call user response handler
					DpaControl.DpaAnswerHandler(DpaUartIfControl.DpaPacketPtr);
				}
				// library is ready for next packet
				DpaControl.Status = DPA_READY;
				// stop data transfer
				DpaUartIfControl.Direction = UART_TRANSFER_NONE;
				return;
			}

			// discard received ESCAPE character
			if (TempData == HDLC_FRM_CONTROL_ESCAPE){
				DpaUartIfControl.WasEscape = 1;
				return;
			}

			// previous character was ESCAPE
			if (DpaUartIfControl.WasEscape){
				DpaUartIfControl.WasEscape = 0;
				TempData ^= HDLC_FRM_ESCAPE_BIT;
			}

			// add Rx byte to CRC
			DpaUartIfControl.CRC = dpaDoCRC8(TempData, DpaUartIfControl.CRC);

			switch(DpaUartIfControl.PacketCnt){
				case 0: DpaUartIfControl.DpaPacketPtr->NADR = TempData; break;   				                            // receive LOW(NADR)
				case 1: DpaUartIfControl.DpaPacketPtr->NADR |= ((uint16_t)TempData << 8); break;	                  // receive HIGH(NADR)
				case 2: DpaUartIfControl.DpaPacketPtr->PNUM = TempData; break;					                            // receive PNUM
				case 3: DpaUartIfControl.DpaPacketPtr->PCMD = TempData;	break;					                            // receive PCMD
				case 4: DpaUartIfControl.DpaPacketPtr->HWPID = TempData; break;					                            // receive LOW(HWPID)
				case 5: DpaUartIfControl.DpaPacketPtr->HWPID |= ((uint16_t)TempData << 8); break;	                  // receive HIGH(HWPID)
				case 6: DpaUartIfControl.DpaPacketPtr->ResponseCode = TempData; break;			                        // receive ResponseCode
				case 7: DpaUartIfControl.DpaPacketPtr->DpaValue = TempData; break;				                          // receive DpaValue
				default: DpaUartIfControl.DpaPacketPtr->DpaMessage.Response.PData[DpaUartIfControl.PacketCnt-8] = TempData;	// receive additional data
			}
			// counts number of send/receive byte
			DpaUartIfControl.PacketCnt++;
		}
		else{
			switch(DpaUartIfControl.PacketCnt){
				case 0: dpaSendDataByte((DpaUartIfControl.DpaPacketPtr->NADR) & 0x00FF); break; 	                  // send LOW(NADR)
				case 1: dpaSendDataByte((DpaUartIfControl.DpaPacketPtr->NADR) >> 8); break;		                      // send HIGH(NADR)
				case 2: dpaSendDataByte(DpaUartIfControl.DpaPacketPtr->PNUM); break;				                        // send PNUM
				case 3: dpaSendDataByte(DpaUartIfControl.DpaPacketPtr->PCMD); break;				                        // send PCMD
				case 4: dpaSendDataByte((DpaUartIfControl.DpaPacketPtr->HWPID) & 0x00FF); break; 	                  // send LOW(HWPID)
				case 5: dpaSendDataByte((DpaUartIfControl.DpaPacketPtr->HWPID) >> 8); break;		                    // send HIGH(HWPID)
				default: dpaSendDataByte(DpaUartIfControl.DpaPacketPtr->DpaMessage.Request.PData[DpaUartIfControl.PacketCnt-6]); // send additional data
			}
			DpaUartIfControl.PacketCnt++;															                                            // counts number of send/receive bytes
			if (DpaUartIfControl.PacketCnt == DpaUartIfControl.PacketLen){						                            // sent everything ?
				dpaSendDataByte(DpaUartIfControl.CRC);											                                        // send CRC
				dpaSendUartByte(HDLC_FRM_FLAG_SEQUENCE);										                                        // send stop of packet character
				DpaControl.Status = DPA_READY;													                                            // library is ready for next packet
				DpaUartIfControl.Direction = UART_TRANSFER_NONE;								                                    // stop data transfer
			}
		}
	}
	else{														                                                                          // no data to send / receive => check for new data
    if (dpaReceiveUartByte(&TempData) == true){                                                             // data in Rx buffer
			if (TempData  == HDLC_FRM_FLAG_SEQUENCE){										                                          // start of packet

				DpaUartIfControl.DpaPacketPtr = &DpaLibDpaAnswer;							                                      // set pointer to DPA receive structure

	      DpaUartIfControl.CRC = 0xFF;												                                                // initialize CRC

	      DpaUartIfControl.PacketLen = sizeof(T_DPA_PACKET);							                                    // maximal size of received data
				DpaUartIfControl.PacketCnt = 0;												                                              // counter of received bytes
				DpaUartIfControl.WasEscape = 0;												                                              // clear Escape flag

				DpaUartIfControl.Direction = UART_TRANSFER_READ;							                                      // reading from TR module
				DpaControl.Status = DPA_BUSY;												                                                // library si busy
			}
			return;
		}

		if (DpaControl.DpaRequestPacketPtr != NULL){										                                        // check if packet to send is ready
			DpaUartIfControl.DpaPacketPtr = DpaControl.DpaRequestPacketPtr;					                              // set pointer to DpaRequest packet
			DpaControl.DpaRequestPacketPtr = NULL;

			DpaUartIfControl.PacketLen = DpaControl.ExtraDataSize + 6;						                                // NADR + PNUM + PCMD + HWPID + Data

      DpaUartIfControl.CRC = 0xFF;													                                                // initialize CRC

			DpaUartIfControl.PacketCnt = 0;													                                              // counter of sent bytes
			DpaUartIfControl.Direction = UART_TRANSFER_WRITE;								                                      // write data to TR module
			DpaControl.Status = DPA_BUSY;													                                                // library si busy

			dpaSendUartByte(HDLC_FRM_FLAG_SEQUENCE);										                                          // send start of packet character
		}
  }
}

/**
 * Send data byte to TR module + make HDLC byte stuffing and comute CRC
 * @param data byte for TR module
 * @return none
 */
void dpaSendDataByte(uint8_t DataByte)
{

	if (DataByte == HDLC_FRM_FLAG_SEQUENCE || DataByte == HDLC_FRM_CONTROL_ESCAPE){
		dpaSendUartByte(HDLC_FRM_CONTROL_ESCAPE);
		dpaSendUartByte(DataByte ^ HDLC_FRM_ESCAPE_BIT);
	}
	else dpaSendUartByte(DataByte);

	DpaUartIfControl.CRC = dpaDoCRC8(DataByte, DpaUartIfControl.CRC);
}

/**
 * Compute the CRC8 value of a data set
 * @param InData  One byte of data to compute CRC from
 * @param Seed The starting value of the CRC
 * @return	The CRC8 of InData with Seed as initial value
 */
uint8_t dpaDoCRC8(uint8_t InData, uint8_t Seed)
{

    uint8_t bitsLeft;

    for (bitsLeft = 8; bitsLeft > 0; bitsLeft--){
        if (((Seed ^ InData) & 0x01) == 0) Seed >>= 1;
        else Seed = (Seed >>= 1)^0x8C;
        InData >>= 1;
    }
    return Seed;
}

#endif

#if defined (__STORE_CODE_SUPPORT__)
/**
 * Function for store HEX or IQRF code image to external EEPROM in TR module
 * @param CodeFileInfo  pointer to T_DPA_CODE_FILE_INFO structure with code file image information
 * @return  Proggess status or operation result (0 - 100 -> progress status, DPA_STORE_CODE_SUCCESS, DPA_STORE_CODE_ERROR)
 */
uint8_t dpaStoreCodeToEeeprom(T_DPA_CODE_FILE_INFO *CodeFileInfo)
{

	static uint16_t tempStoreEepromAddress;

	switch(DpaStoreCodeTaskSM){
		case DPA_STORE_CODE_INIT_TASK:{
			// initialize code file character read counter
			DpaControl.FileByteCounter = 0;

			// temporary store start address of code image in EEEPROM
			tempStoreEepromAddress = CodeFileInfo -> ImageEeepromAdr;

			if (CodeFileInfo -> FileType == DPA_CODE_FILE_IQRF) CodeFileInfo -> ImageCRC = 0x03;		  // initial value for .IQRF CRC
			else CodeFileInfo -> ImageCRC = 0x01;														                          // initial value for .HEX CRC

			// initialize file image size
			CodeFileInfo -> ImageSize = 0;

			// initialize work variables
			DpaStartAddressDetected = DpaCodeImageEndDetected = DpaOutputPacketDataCnt = DpaEeepromPageDataCnt = DpaRemainingDataCnt = 0;

			// start data sending
			DpaStoreCodeTaskSM = DPA_STORE_CODE_SEND_DATA;

			return(0);
		}

		case DPA_STORE_CODE_SEND_DATA:{
			if (CodeFileInfo -> FileType == DPA_CODE_FILE_IQRF) dpaProcessIqrfCodeFile(CodeFileInfo);	// process .IQRF file
			else dpaProcessHexCodeFile(CodeFileInfo);													                        // process .HEX file
		}
		break;

    case DPA_STORE_CODE_ERROR_END:{
			// initialize task state machine
      DpaStoreCodeTaskSM = DPA_STORE_CODE_INIT_TASK;
      return(DPA_STORE_CODE_ERROR);
    }

		case DPA_STORE_CODE_SUCCESS_END:{
			// restore start address of code image in EEEPROM
			CodeFileInfo -> ImageEeepromAdr = tempStoreEepromAddress;
			// initialize task state machine
			DpaStoreCodeTaskSM = DPA_STORE_CODE_INIT_TASK;
			return(DPA_STORE_CODE_SUCCESS);
		}
	}

	return(((uint32_t)DpaControl.FileByteCounter * 100) / CodeFileInfo->FileSize);
}

/**
 *  Compute the Flatcher-16 checksum
 * @param InData  One byte of data to compute CRC from
 * @param Seed The starting value of the CRC
 * @return	The FlatcherCRC16 of InData with Seed as initial value
 */
uint16_t dpaFlatcherCRC16(uint8_t *DataAddress, uint8_t DataLen, uint16_t Seed)
{

	uint16_t	TempL = Seed & 0xFF;
	uint16_t  TempH = Seed >> 8;

	while (DataLen--){
	  TempL += *DataAddress;
		DataAddress++;
	  if ( ( TempL & 0x100 ) != 0 ){
			TempL++;
			TempL &= 0xFF;
		}

	  TempH += TempL;
	  if ( ( TempH & 0x100 ) != 0 ){
			TempH++;
	 		TempH &= 0xFF;
		}
	}

  return ((TempH << 8) | TempL);
}

/**
 *  Send prepared data packed to eeeprom peripheral and compute FlatcherCRC16 from data
 * @param	 CodeFileInfo  pointer to T_DPA_CODE_FILE_INFO structure with code file image information
 * @param  DataSize number of data bytes in output buffer
 * @return	none
 */
void dpaSendDataToEeeprom(T_DPA_CODE_FILE_INFO *CodeFileInfo, uint8_t DataSize)
{

  uint8_t OpResult;

	// make CRC from code image block of data
	CodeFileInfo -> ImageCRC = dpaFlatcherCRC16((uint8_t *)&DpaStoreCodeRequest.DpaMessage.XMemoryRequest.ReadWrite.Write.PData[0], DataSize, CodeFileInfo -> ImageCRC);
	CodeFileInfo -> ImageSize += DataSize;									// increase code image size

	DpaStoreCodeRequest.NADR = CodeFileInfo -> TrAddress;		// set destination address
	DpaStoreCodeRequest.PNUM = PNUM_EEEPROM;								// set destination peripheral EEEPROM
	DpaStoreCodeRequest.PCMD = CMD_EEEPROM_XWRITE;					// set xwrite to eeeprom command
	DpaStoreCodeRequest.HWPID = HWPID_DoNotCheck;						// do not check HWPID

	// set destination address in eeeprom
	DpaStoreCodeRequest.DpaMessage.XMemoryRequest.Address = CodeFileInfo -> ImageEeepromAdr;
	// next write on address
	CodeFileInfo -> ImageEeepromAdr += DataSize;

  // send request and wait for result
  while((OpResult = dpaSendRequest(&DpaStoreCodeRequest,DataSize + 2, 1000)) == DPA_OPERATION_IN_PROGRESS);

  switch(OpResult){
      case DPA_OPERATION_TIMEOUT:
      case DPA_CONFIRMATION_ERR:
      case DPA_RESPONSE_ERR:
      case DPA_TR_MODULE_NOT_READY:
        DpaStoreCodeTaskSM = DPA_STORE_CODE_ERROR_END;
  }
}

/**
 * Process HEX file with custom DPA handler code
 * @param CodeFileInfo  Pointer to T_DPA_CODE_FILE_INFO structure with code file image information
 * @return none
 */
void dpaProcessHexCodeFile(T_DPA_CODE_FILE_INFO *CodeFileInfo)
{

	uint8_t	TempValue;

  // if end of code image not detected
	if (!DpaCodeImageEndDetected){
    dpaSuspendDriver();
		// read one line from .HEX file
		if ((TempValue = dpaReadHEXFileLine()) != 0){
      dpaRunDriver();
			// CRC error in hex file detected
			if (TempValue == 2){
				DpaStoreCodeTaskSM = DPA_STORE_CODE_ERROR_END;
			}
			else DpaCodeImageEndDetected = true;							 // end of file detected
			return;
		}
    dpaRunDriver();

		// line from hex file do not contain data, read next line
		if (DpaCodeLineBuffer[3] != 0) return;

		// compute destination code address
		DpaCodeLineAddress = (((uint16_t)DpaCodeLineBuffer[1] << 8) | DpaCodeLineBuffer[2]) / 2;

		if (DpaCodeLineAddress == CUSTOM_HANDLER_ADDRESS){			// if code address == custom DPA handler start address
			DpaStartAddressDetected = true;									      // set flag, start address detected
			DpaCodeImageNextAddress = CUSTOM_HANDLER_ADDRESS;			// set next code address
		}

		if (DpaStartAddressDetected == true){								    // if start address of custom DPA handler was detected
			if (DpaCodeImageNextAddress != DpaCodeLineAddress || DpaCodeLineAddress > CUSTOM_HANDLER_ADDRESS_END){	// if destination code address is not correct
				DpaCodeImageEndDetected = true;								      // set flag, code image end defected
				return;
			}

			if (DpaCodeLineBuffer[0] != 0x10){								    // if hex file line buffer contains less then 0x10 data bytes
				DpaRemainingDataCnt = DpaCodeLineBuffer[0];					// set remaining data counter
				DpaCodeImageEndDetected = true;								      // set flag, code image end defected
				return;
			}

			// copy data from hexfile line buffer to output buffer
			memcpy ((uint8_t *)&DpaStoreCodeRequest.DpaMessage.XMemoryRequest.ReadWrite.Write.PData[DpaOutputPacketDataCnt], (uint8_t *)&DpaCodeLineBuffer[4], DpaCodeLineBuffer[0]);

			DpaCodeImageNextAddress += 0x08;								      // next code address should be
			DpaOutputPacketDataCnt += 0x10;									      // increase data counter in output buffer

			if (DpaOutputPacketDataCnt == 0x20){							    // if output buffer contains 0x20 bytes
				DpaOutputPacketDataCnt = 0;									        // clear output buffer data counter
				DpaEeepromPageDataCnt += 0x20;								      // increase eeeprom page data counter
				if (DpaEeepromPageDataCnt == 0x40) DpaEeepromPageDataCnt = 0;	// if eeeprom data page contains 0x40 bytes, clear page data counter

				dpaSendDataToEeeprom(CodeFileInfo, 0x20);					  // send 0x20 data bytes of code image to eeeprom preripheral of TR module
			}
		}
	}
	else{																	                    // if end of code image detected
		// if hex file line buffer contains any data
		if (DpaRemainingDataCnt){
		  // copy it to output buffer
			memcpy ((uint8_t *)&DpaStoreCodeRequest.DpaMessage.XMemoryRequest.ReadWrite.Write.PData[DpaOutputPacketDataCnt], (uint8_t *)&DpaCodeLineBuffer[4], DpaRemainingDataCnt);
			DpaOutputPacketDataCnt += DpaRemainingDataCnt;				// increase output buffer data counter
			DpaRemainingDataCnt = 0;										          // clear hex file line buffer data counter
		}
		else{																                    // if hex file line buffer contains no data
			if (DpaEeepromPageDataCnt == 0){								      // if whole page to eeeprom was written
				DpaStoreCodeTaskSM = DPA_STORE_CODE_SUCCESS_END;	  // operation end
				return;
			}
		}

		// if output buffer contains less then 0x20 data bytes
		if (DpaOutputPacketDataCnt < 0x20){
			// fill rest of buffer with zeros
			memset ((uint8_t *)&DpaStoreCodeRequest.DpaMessage.XMemoryRequest.ReadWrite.Write.PData[DpaOutputPacketDataCnt], 0, 0x20 - DpaOutputPacketDataCnt);
			DpaOutputPacketDataCnt = 0;										        // clear outpt buffer data counter
			DpaEeepromPageDataCnt += 0x20;									      // increase eeprom page data counter
			// if eeeprom data page contains 0x40 bytes, clear page data counter
			if (DpaEeepromPageDataCnt == 0x40) DpaEeepromPageDataCnt = 0;
			// send 0x20 data bytes of code image to eeeprom preripheral of TR module
			dpaSendDataToEeeprom(CodeFileInfo, 0x20);
		}
	}
}

/**
 * Process IQRF file with IQRF plugin or new OS
 * @param CodeFileInfo  Pointer to T_DPA_CODE_FILE_INFO structure with code file image information
 * @return none
 */
void dpaProcessIqrfCodeFile(T_DPA_CODE_FILE_INFO *CodeFileInfo)
{

	uint8_t	TempValue;

	// if end of code image not detected
	if (!DpaCodeImageEndDetected){
		// if iqrf file line buffer contains any data
		if (DpaRemainingDataCnt){

			// number of bytes to full output buffer
			TempValue = 32 - DpaOutputPacketDataCnt;
			// if required bytes is more then ready bytes in line buffer
			if (TempValue > DpaRemainingDataCnt) TempValue = DpaRemainingDataCnt;
			// copy required bytes from line buffer to out buffer
			memcpy ((uint8_t *)&DpaStoreCodeRequest.DpaMessage.XMemoryRequest.ReadWrite.Write.PData[DpaOutputPacketDataCnt], (uint8_t *)&DpaCodeLineBuffer[20-DpaRemainingDataCnt], TempValue);

			DpaRemainingDataCnt -= TempValue;								// decrease remaining data counter
			DpaOutputPacketDataCnt += TempValue;						// increase output data counter

			// if output buffer contains 32 bytes
			if (DpaOutputPacketDataCnt == 32){
				DpaOutputPacketDataCnt = 0;									  // clear output buffer data counter
				// send 32 data bytes of code image to eeeprom preripheral of TR module
				dpaSendDataToEeeprom(CodeFileInfo, 32);
			}
		}
		else{
      dpaSuspendDriver();
			// read one line from .IQRF file
			if ((TempValue = dpaReadIQRFFileLine()) != 0){
        dpaRunDriver();
				// wrong format if .IQRF file detected
				if (TempValue == 2){
					DpaStoreCodeTaskSM = DPA_STORE_CODE_ERROR_END;		// end operation
				}
				else DpaCodeImageEndDetected = true;								// end of file detected
				return;
			}
      dpaRunDriver();
			// number of ready bytes in file line buffer
			DpaRemainingDataCnt = 20;
		}
	}
	else{  // if end of code image was detected
		// if any data still in output buffer
		if (DpaOutputPacketDataCnt){
			// send rest of data bytes of code image to eeeprom preripheral of TR module
			dpaSendDataToEeeprom(CodeFileInfo, DpaOutputPacketDataCnt);
			// clear output buffer data counter
			DpaOutputPacketDataCnt = 0;
		}
		else{
			DpaStoreCodeTaskSM = DPA_STORE_CODE_SUCCESS_END;
			return;
		}
	}
}

/**
 * Convert two ascii char to number
 * @param - High and Low nibble in ascii
 * @return - number
 */
uint8_t dpaConvertToNum(uint8_t DataByteHi, uint8_t DataByteLo)
{

	uint8_t Result=0;

  /* convert High nibble */
	if (DataByteHi >= '0' && DataByteHi <= '9') Result = (DataByteHi-'0') << 4;
  else if (DataByteHi >= 'a' && DataByteHi <= 'f') Result = (DataByteHi-87) << 4;
  /* convert Low nibble */
	if (DataByteLo >= '0' && DataByteLo <= '9') Result |= (DataByteLo-'0');
  else if (DataByteLo >= 'a' && DataByteLo <= 'f') Result |= (DataByteLo-87);

	return(Result);
}

/**
 * Read and process line from firmware file
 * @param none
 * @return Return code (0 - HEX file line ready, 1 - end of file, 2 - CRC error in HEX file line)
 */
uint8_t dpaReadHEXFileLine(void)
{

  uint8_t Znak;
	uint8_t DataByteHi,DataByteLo;
	uint8_t DataByte;
	uint8_t CodeLineBufferPtr=0;
	uint8_t CodeLineBufferCrc=0;

  // find start of line or end of file
	while (((Znak = dpaReadByteFromFile()) != 0) && (Znak != ':'));
	// if end of file
  if (Znak == 0) return(1);

	// read data to end of line and convert if to numbers
	for (;;){
		DataByteHi = tolower(dpaReadByteFromFile());					        // read High nibble
    if (DataByteHi==0x0A || DataByteHi==0x0D){						        // check end of line
			if (CodeLineBufferCrc != 0) return(2);						          // check line CRC
			return(0);													                        // stop reading
		}
		DataByteLo = tolower(dpaReadByteFromFile());					        // read Low nibble

		DataByte = dpaConvertToNum(DataByteHi,DataByteLo);				    // convert two ascii to number
		CodeLineBufferCrc += DataByte;									              // add to Crc
		DpaCodeLineBuffer[CodeLineBufferPtr++] = DataByte;				    // store to line buffer
  }
}

/**
 * Read and process line from plugin file
 * @param none
 * @return Return code (0 - iqrf file line ready, 1 - end of file, 2 - input file format error)
 */
uint8_t dpaReadIQRFFileLine(void)
{

	uint8_t	FirstChar;
	uint8_t	SecondChar;
	uint8_t	CodeLineBufferPtr = 0;

	repeat_read:
	// read one char from file
	FirstChar = tolower(dpaReadByteFromFile());

	// read one char from file
	if (FirstChar == '#'){
		// read data to end of line
		while (((FirstChar = dpaReadByteFromFile()) != 0) && (FirstChar != 0x0D));
	}

	// if end of line
	if (FirstChar == 0x0D){
		dpaReadByteFromFile();										               // read second code 0x0A
		if (CodeLineBufferPtr == 0) goto repeat_read;				     // read another line
		if (CodeLineBufferPtr == 20) return(0);						       // line with data readed successfully
		else return(2);												                   // wrong file format (error)
	}

	// if end of file
	if (FirstChar == 0) return(1);

	// read second character from code file
	SecondChar = tolower(dpaReadByteFromFile());
	// convert chars to number and store to buffer
	DpaCodeLineBuffer[CodeLineBufferPtr++] = dpaConvertToNum(FirstChar, SecondChar);

	goto repeat_read;		// read next data
}

#endif
