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

#include "dpa_library.h"

#if defined(__SPI_INTERFACE__)
#define SPI_TRANSFER_NONE        0
#define SPI_TRANSFER_WRITE       1
#define SPI_TRANSFER_READ        2

/// Master checks the SPI status of the TR module
#define SPI_CHECK                0x00
/// Master reads/writes a packet from/to TR module
#define SPI_WR_RD                0xF0
/// SPI not ready (full buffer, last CRCM ok)
#define SPI_CRCM_OK              0x3F
/// SPI not ready (full buffer, last CRCM error)
#define SPI_CRCM_ERR             0x3E
/// SPI status pooling time 10ms
#define SPI_STATUS_POOLING_TIME  10

typedef struct {
	uint8_t DLEN;
	uint8_t CMD;
	uint8_t PTYPE;
	uint8_t CRCM;
	uint8_t myCRCS;
	uint8_t CRCS;
	uint8_t spiStat;
	uint8_t direction;
	uint8_t packetLen;
	uint8_t packetCnt;
	uint8_t packetRpt;
	T_DPA_PACKET *dpaPacketPtr;
} T_DPA_SPI_INTERFACE_CONTROL;

T_DPA_SPI_INTERFACE_CONTROL dpaSpiIfControl;

uint8_t DPA_SendSpiByte(uint8_t rxByte);
void DPA_DeselectTRmodule(void);
void DPA_SpiInterfaceDriver(void);
uint8_t DPA_GetCRCM(void);

#elif defined(__UART_INTERFACE__)
#define UART_TRANSFER_NONE       0
#define UART_TRANSFER_WRITE      1
#define UART_TRANSFER_READ       2

#define HDLC_FRM_FLAG_SEQUENCE   0x7E
#define HDLC_FRM_CONTROL_ESCAPE  0x7D
#define HDLC_FRM_ESCAPE_BIT      0x20

typedef struct {
	uint8_t direction;
	uint8_t packetCnt;
	uint8_t packetLen;
	uint8_t CRC;
	uint8_t wasEscape;
	uint8_t rxBuffInPtr;
	uint8_t rxBuffOutPtr;
	uint8_t rxBuff[16];
	T_DPA_PACKET *dpaPacketPtr;
} T_DPA_UART_INTERFACE_CONTROL;

T_DPA_UART_INTERFACE_CONTROL dpaUartIfControl;

void DPA_SendUartByte(uint8_t rxByte);
void DPA_ReceiveUartByte(uint8_t rxByte);
void DPA_UartInterfaceDriver(void);
void DPA_SendDataByte(uint8_t dataByte);
uint8_t DPA_doCRC8(uint8_t inData, uint8_t seed);

#endif

/*
 * Public variable declarations
 */
T_DPA_CONTROL dpaControl;
T_DPA_PACKET dpaLibDpaAnswer;

/**
 * Initialize DPA support library
 */
void DPA_Init(void) {
	dpaControl.status = DPA_READY;
	dpaControl.dpaAnswerHandler = NULL;
	dpaControl.timeFlag = 0;

#ifdef __SPI_INTERFACE__
	dpaControl.timeCnt = SPI_STATUS_POOLING_TIME;
	dpaSpiIfControl.spiStat = 0;
	dpaSpiIfControl.direction = SPI_TRANSFER_NONE;
#endif

#ifdef __UART_INTERFACE__
	dpaUartIfControl.direction = UART_TRANSFER_NONE;
	dpaUartIfControl.wasEscape = 0;
	dpaUartIfControl.rxBuffInPtr = 0;
	dpaUartIfControl.rxBuffOutPtr = 0;
#endif
}

/**
 * Provide background communication with TR module
 */
void DPA_LibraryDriver(void) {
#if defined(__SPI_INTERFACE__)
	if (dpaControl.timeFlag) {
		dpaControl.timeFlag = 0;
		if (dpaControl.status == DPA_BUSY || !dpaControl.timeCnt) {
			DPA_SpiInterfaceDriver();
#if defined(TR5xD)
			dpaControl.timeCnt = SPI_STATUS_POOLING_TIME + 1;
#elif defined(TR7xD)
			dpaControl.timeCnt = (SPI_STATUS_POOLING_TIME * 7) + 1;
#endif
		}
		dpaControl.timeCnt--;
	}
#elif defined(__UART_INTERFACE__)
	DPA_UartInterfaceDriver();
#endif
}

/**
 * Send DPA request packet to desired destination address
 * @param dpaRequest Pointer to DPA request packet
 * @param dataSize Number of additional data bytes in DPA request packet
 */
void DPA_SendRequest(T_DPA_PACKET *dpaRequest, uint8_t dataSize) {
	dpaControl.dpaRequestPacketPtr = dpaRequest;
	dpaControl.extraDataSize = dataSize;
	dpaControl.status = DPA_PREPARE;
}

/**
 * Returns estimated timeout for response packet in miliseconds
 * @return estimated timeout for response packet in ms (computed from confirmation packet data)
 */
uint16_t DPA_GetEstimatedTimeout(void) {
	uint16_t responseTimeSlotLength;
	uint16_t estimatedTimeout = (uint16_t) (dpaLibDpaAnswer.DpaMessage.IFaceConfirmation.Hops + 1) * (uint16_t) dpaLibDpaAnswer.DpaMessage.IFaceConfirmation.TimeSlotLength * 10;

	if (dpaLibDpaAnswer.DpaMessage.IFaceConfirmation.TimeSlotLength == 20) {
		// DPA in diagnostic mode
		responseTimeSlotLength = 200;
	} else if (dpaLibDpaAnswer.DpaMessage.IFaceConfirmation.TimeSlotLength > 6) {
		// DPA in LP mode
		responseTimeSlotLength = 100;
	} else {
		// DPA in STD mode
		responseTimeSlotLength = 50;
	}
	estimatedTimeout += ((uint16_t) (dpaLibDpaAnswer.DpaMessage.IFaceConfirmation.HopsResponse + 1) * responseTimeSlotLength + 40);
	return (estimatedTimeout);
}

#if defined(__SPI_INTERFACE__)

/**
 * Implements IQRF packet communication over SPI with TR module
 */
void DPA_SpiInterfaceDriver(void) {
	uint8_t last_spistat;
	uint8_t tempData;

	// is anything to send / receive
	if (dpaSpiIfControl.direction != SPI_TRANSFER_NONE) {
		if (dpaSpiIfControl.direction == SPI_TRANSFER_WRITE) {
			switch (dpaSpiIfControl.packetCnt) {
				case 0:
					// send SPI CMD
					DPA_SendSpiByte(dpaSpiIfControl.CMD);
					dpaSpiIfControl.myCRCS = 0x5F;
					break;
				case 1:
					// send PTYPE
					DPA_SendSpiByte(dpaSpiIfControl.PTYPE);
					dpaSpiIfControl.myCRCS ^= dpaSpiIfControl.PTYPE;
					break;
				case 2:
					// send LOW(NADR)
					dpaSpiIfControl.myCRCS ^= DPA_SendSpiByte((dpaSpiIfControl.dpaPacketPtr->NADR) & 0x00FF);
					break;
				case 3:
					// send HIGH(NADR)
					dpaSpiIfControl.myCRCS ^= DPA_SendSpiByte((dpaSpiIfControl.dpaPacketPtr->NADR) >> 8);
					break;
				case 4:
					// send PNUM
					dpaSpiIfControl.myCRCS ^= DPA_SendSpiByte(dpaSpiIfControl.dpaPacketPtr->PNUM);
					break;
				case 5:
					// send PCMD
					dpaSpiIfControl.myCRCS ^= DPA_SendSpiByte(dpaSpiIfControl.dpaPacketPtr->PCMD);
					break;
				case 6:
					// send LOW(HWPID)
					dpaSpiIfControl.myCRCS ^= DPA_SendSpiByte((dpaSpiIfControl.dpaPacketPtr->HWPID) & 0x00FF);
					break;
				case 7:
					// send HIGH(HWPID)
					dpaSpiIfControl.myCRCS ^= DPA_SendSpiByte((dpaSpiIfControl.dpaPacketPtr->HWPID) >> 8);
					break;
				default:
					if (dpaSpiIfControl.packetCnt == dpaSpiIfControl.packetLen - 2) {
						// send CRCM / receive CRCS
						dpaSpiIfControl.CRCS = DPA_SendSpiByte(dpaSpiIfControl.CRCM);
					} else if (dpaSpiIfControl.packetCnt == dpaSpiIfControl.packetLen - 1) {
						// receive SPI_STATUS
						dpaSpiIfControl.spiStat = DPA_SendSpiByte(0);
					} else {
						// send additional data
						dpaSpiIfControl.myCRCS ^= DPA_SendSpiByte(dpaSpiIfControl.dpaPacketPtr->DpaMessage.Request.PData[dpaSpiIfControl.packetCnt - 8]);
					}
			}
		} else {
			if (dpaSpiIfControl.packetCnt >= 2 && dpaSpiIfControl.packetCnt < dpaSpiIfControl.packetLen - 2) {
				tempData = DPA_SendSpiByte(0);
				dpaSpiIfControl.myCRCS ^= tempData;
			}

			switch (dpaSpiIfControl.packetCnt) {
				case 0:
					// send SPI CMD
					DPA_SendSpiByte(dpaSpiIfControl.CMD);
					dpaSpiIfControl.myCRCS = 0x5F;
					break;
				case 1:
					// send PTYPE
					DPA_SendSpiByte(dpaSpiIfControl.PTYPE);
					dpaSpiIfControl.myCRCS ^= dpaSpiIfControl.PTYPE;
					break;
				case 2:
					// receive LOW(NADR)
					dpaSpiIfControl.dpaPacketPtr->NADR = tempData;
					break;
				case 3:
					// receive HIGH(NADR)
					dpaSpiIfControl.dpaPacketPtr->NADR |= ((uint16_t) tempData << 8);
					break;
				case 4:
					// receive PNUM
					dpaSpiIfControl.dpaPacketPtr->PNUM = tempData;
					break;
				case 5:
					// receive PCMD
					dpaSpiIfControl.dpaPacketPtr->PCMD = tempData;
					break;
				case 6:
					// receive LOW(HWPID)
					dpaSpiIfControl.dpaPacketPtr->HWPID = tempData;
					break;
				case 7:
					// receive HIGH(HWPID)
					dpaSpiIfControl.dpaPacketPtr->HWPID |= ((uint16_t) tempData << 8);
					break;
				case 8:
					// receive ResponseCode
					dpaSpiIfControl.dpaPacketPtr->ResponseCode = tempData;
					break;
				case 9:
					// receive DpaValue
					dpaSpiIfControl.dpaPacketPtr->DpaValue = tempData;
					break;
				default:
					if (dpaSpiIfControl.packetCnt == dpaSpiIfControl.packetLen - 2) {
						// send CRCM / receive CRCS
						dpaSpiIfControl.CRCS = DPA_SendSpiByte(dpaSpiIfControl.CRCM);
					} else if (dpaSpiIfControl.packetCnt == dpaSpiIfControl.packetLen - 1) {
						// receive SPI_STATUS
						dpaSpiIfControl.spiStat = DPA_SendSpiByte(0);
					} else {
						// receive additional data
						dpaSpiIfControl.dpaPacketPtr->DpaMessage.Response.PData[dpaSpiIfControl.packetCnt - 10] = tempData;
					}
			}
		}
		// counts number of send/receive bytes
		dpaSpiIfControl.packetCnt++;
		// sent everything ?
		if (dpaSpiIfControl.packetCnt == dpaSpiIfControl.packetLen) {
			DPA_DeselectTRmodule();
			// CRC ok ?
			if ((dpaSpiIfControl.spiStat == SPI_CRCM_OK) && (dpaSpiIfControl.CRCS == dpaSpiIfControl.myCRCS)) {
				if (dpaSpiIfControl.direction == SPI_TRANSFER_READ && dpaControl.dpaAnswerHandler != NULL) {
					// call user response handler
					dpaControl.dpaAnswerHandler(&dpaLibDpaAnswer);
				}
				// library is ready for next packet
				dpaControl.status = DPA_READY;
				// stop data transfer
				dpaSpiIfControl.direction = SPI_TRANSFER_NONE;
			} else { // CRC error
				// pktRepeats - must be set on packet preparing
				if (--dpaSpiIfControl.packetRpt) {
					// another attempt to send data
					dpaSpiIfControl.packetCnt = 0;
				} else {
					// library is ready for next packet
					dpaControl.status = DPA_READY;
					// stop data transfer
					dpaSpiIfControl.direction = SPI_TRANSFER_NONE;
				}
			}
			// current SPI status must be updated
			dpaSpiIfControl.spiStat = 0;
		}
	} else { // no data to send => SPI status will be updated
		// get SPI status of TR module
		last_spistat = DPA_SendSpiByte(SPI_CHECK);
		DPA_DeselectTRmodule();
		// the status must be 2x the same
		if (dpaSpiIfControl.spiStat != last_spistat) {
			dpaSpiIfControl.spiStat = last_spistat;
			return;
		}
		// if the status is dataready, prepare packet to read it
		if ((dpaSpiIfControl.spiStat & 0xC0) == 0x40) {
			if (dpaSpiIfControl.spiStat == 0x40) {
				// status 0x40 means offering 64B
				dpaSpiIfControl.DLEN = 64;
			} else {
				// clear bit 7,6 - rest is length (from 1B to 63B)
				dpaSpiIfControl.DLEN = dpaSpiIfControl.spiStat & 0x3F;
			}// set pointer to DPA receive structure
			dpaSpiIfControl.dpaPacketPtr = &dpaLibDpaAnswer;
			// read / write data
			dpaSpiIfControl.CMD = SPI_WR_RD;
			dpaSpiIfControl.PTYPE = dpaSpiIfControl.DLEN;
			// CRCM
			dpaSpiIfControl.CRCM = 0x5F ^ dpaSpiIfControl.CMD ^ dpaSpiIfControl.PTYPE;
			// length of whole packet + (CMD, PTYPE, CRCM, 0)
			dpaSpiIfControl.packetLen = dpaSpiIfControl.DLEN + 4;
			// counter of sent bytes
			dpaSpiIfControl.packetCnt = 0;
			// number of attempts to send data
			dpaSpiIfControl.packetRpt = 1;
			// reading from buffer COM of TR module
			dpaSpiIfControl.direction = SPI_TRANSFER_READ;
			// current SPI status must be updated
			dpaSpiIfControl.spiStat = 0;
			// library si busy
			dpaControl.status = DPA_BUSY;
			return;
		}
		// check if packet to send is ready
		if (dpaControl.dpaRequestPacketPtr != NULL) {
			// set pointer to DpaRequest packet
			dpaSpiIfControl.dpaPacketPtr = dpaControl.dpaRequestPacketPtr;
			dpaControl.dpaRequestPacketPtr = NULL;
			// NADR + PNUM + PCMD + HWPID + Data
			dpaSpiIfControl.DLEN = dpaControl.extraDataSize + 6;
			// read / write data
			dpaSpiIfControl.CMD = SPI_WR_RD;
			// PBYTE set bit7 - write to buffer COM of TR module
			dpaSpiIfControl.PTYPE = (dpaSpiIfControl.DLEN | 0x80);
			// CRCM
			dpaSpiIfControl.CRCM = DPA_GetCRCM();
			// length of whole packet + (CMD, PTYPE, CRCM, 0)
			dpaSpiIfControl.packetLen = dpaSpiIfControl.DLEN + 4;
			// counter of sent bytes
			dpaSpiIfControl.packetCnt = 0;
			// number of attempts to send data
			dpaSpiIfControl.packetRpt = 3;
			// reading from buffer COM of TR module
			dpaSpiIfControl.direction = SPI_TRANSFER_WRITE;
			// current SPI status must be updated
			dpaSpiIfControl.spiStat = 0;
			// library si busy
			dpaControl.status = DPA_BUSY;
		}
	}
}

/**
 * Calculate CRC before master's send
 * @return CRC
 */
uint8_t DPA_GetCRCM(void) {
	unsigned char i, crc_val, dataSize;
	// initialize CRC
	crc_val = 0x5F;
	// add SPI CMD
	crc_val ^= dpaSpiIfControl.CMD;
	// add PTYPE
	crc_val ^= dpaSpiIfControl.PTYPE;
	// add LOW(NADR)
	crc_val ^= dpaSpiIfControl.dpaPacketPtr->NADR & 0x00FF;
	// add HIGH(NADR)
	crc_val ^= dpaSpiIfControl.dpaPacketPtr->NADR >> 8;
	// add PNUM
	crc_val ^= dpaSpiIfControl.dpaPacketPtr->PNUM;
	// add PCMD
	crc_val ^= dpaSpiIfControl.dpaPacketPtr->PCMD;
	// add LOW(HWPID)
	crc_val ^= dpaSpiIfControl.dpaPacketPtr->HWPID & 0x00FF;
	// add HIGH(HWPID)
	crc_val ^= dpaSpiIfControl.dpaPacketPtr->HWPID >> 8;
	// number of extra data bytes (except NADR, PNUM, PCMD, HWPID)
	dataSize = dpaSpiIfControl.DLEN - 6;
	for (i = 0; i < dataSize; i++) {
		crc_val ^= dpaSpiIfControl.dpaPacketPtr->DpaMessage.Request.PData[i];
	}

	return crc_val;
}

#elif defined(__UART_INTERFACE__)

/**
 * Implements IQRF packet communication over UART with TR module
 */
void DPA_UartInterfaceDriver(void) {
	uint8_t tempData;
	// is anything to send / receive
	if (dpaUartIfControl.direction != UART_TRANSFER_NONE) {
		if (dpaUartIfControl.direction == UART_TRANSFER_READ) {
			// no data in Rx buffer
			if (dpaUartIfControl.rxBuffInPtr == dpaUartIfControl.rxBuffOutPtr) {
				return;
			}
			// read byte from Rx buffer
			tempData = dpaUartIfControl.rxBuff[dpaUartIfControl.rxBuffOutPtr++];
			// this makes 16 bytes circle buffer
			dpaUartIfControl.rxBuffOutPtr &= 0x0F;
			// end of packet or DPA structure is full
			if (tempData == HDLC_FRM_FLAG_SEQUENCE || dpaUartIfControl.packetCnt == dpaUartIfControl.packetLen) {
				if (dpaUartIfControl.CRC == 0 && dpaControl.dpaAnswerHandler != NULL) {
					// call user response handler
					dpaControl.dpaAnswerHandler(&dpaLibDpaAnswer);
				}
				// library is ready for next packet
				dpaControl.status = DPA_READY;
				// stop data transfer
				dpaUartIfControl.direction = UART_TRANSFER_NONE;
				return;
			}
			// discard received ESCAPE character
			if (tempData == HDLC_FRM_CONTROL_ESCAPE) {
				dpaUartIfControl.wasEscape = 1;
				return;
			}
			// previous character was ESCAPE
			if (dpaUartIfControl.wasEscape) {
				dpaUartIfControl.wasEscape = 0;
				tempData ^= HDLC_FRM_ESCAPE_BIT;
			}
			// add Rx byte to CRC
			dpaUartIfControl.CRC = DPA_doCRC8(tempData, dpaUartIfControl.CRC);

			switch (dpaUartIfControl.packetCnt) {
				case 0:
					// receive LOW(NADR)
					dpaUartIfControl.dpaPacketPtr->NADR = tempData;
					break;
				case 1:
					// receive HIGH(NADR)
					dpaUartIfControl.dpaPacketPtr->NADR |= ((uint16_t) tempData << 8);
					break;
				case 2:
					// receive PNUM
					dpaUartIfControl.dpaPacketPtr->PNUM = tempData;
					break;
				case 3:
					// receive PCMD
					dpaUartIfControl.dpaPacketPtr->PCMD = tempData;
					break;
				case 4:
					// receive LOW(HWPID)
					dpaUartIfControl.dpaPacketPtr->HWPID = tempData;
					break;
				case 5:
					// receive HIGH(HWPID)
					dpaUartIfControl.dpaPacketPtr->HWPID |= ((uint16_t) tempData << 8);
					break;
				case 6:
					// receive ResponseCode
					dpaUartIfControl.dpaPacketPtr->ResponseCode = tempData;
					break;
				case 7:
					// receive DpaValue
					dpaUartIfControl.dpaPacketPtr->DpaValue = tempData;
					break;
				default:
					// receive additional data
					dpaUartIfControl.dpaPacketPtr->DpaMessage.Response.PData[dpaUartIfControl.packetCnt - 8] = tempData;
			}
			// counts number of send/receive byte
			dpaUartIfControl.packetCnt++;
		} else {
			switch (dpaUartIfControl.packetCnt) {
				case 0:
					// send LOW(NADR)
					DPA_SendDataByte((dpaUartIfControl.dpaPacketPtr->NADR) & 0x00FF);
					break;
				case 1:
					// send HIGH(NADR)
					DPA_SendDataByte((dpaUartIfControl.dpaPacketPtr->NADR) >> 8);
					break;
				case 2:
					// send PNUM
					DPA_SendDataByte(dpaUartIfControl.dpaPacketPtr->PNUM);
					break;
				case 3:
					// send PCMD
					DPA_SendDataByte(dpaUartIfControl.dpaPacketPtr->PCMD);
					break;
				case 4:
					// send LOW(HWPID)
					DPA_SendDataByte((dpaUartIfControl.dpaPacketPtr->HWPID) & 0x00FF);
					break;
				case 5:
					// send HIGH(HWPID)
					DPA_SendDataByte((dpaUartIfControl.dpaPacketPtr->HWPID) >> 8);
					break;
				default:
					// send additional data
					DPA_SendDataByte(dpaUartIfControl.dpaPacketPtr->DpaMessage.Request.PData[dpaUartIfControl.packetCnt - 6]);
			}
			// counts number of send/receive bytes
			dpaUartIfControl.packetCnt++;
			// sent everything ?
			if (dpaUartIfControl.packetCnt == dpaUartIfControl.packetLen) {
				// send CRC
				DPA_SendDataByte(dpaUartIfControl.CRC);
				// send stop of packet character
				DPA_SendUartByte(HDLC_FRM_FLAG_SEQUENCE);
				// library is ready for next packet
				dpaControl.status = DPA_READY;
				// stop data transfer
				dpaUartIfControl.direction = UART_TRANSFER_NONE;
			}
		}
	} else { // no data to send / receive => check for new data
		// data in Rx buffer
		if (dpaUartIfControl.rxBuffInPtr != dpaUartIfControl.rxBuffOutPtr) {
			// read byte from Rx buffer
			tempData = dpaUartIfControl.rxBuff[dpaUartIfControl.rxBuffOutPtr++];
			// this makes 16 bytes circle buffer
			dpaUartIfControl.rxBuffOutPtr &= 0x0F;
			// start of packet
			if (tempData == HDLC_FRM_FLAG_SEQUENCE) {
				// set pointer to DPA receive structure
				dpaUartIfControl.dpaPacketPtr = &dpaLibDpaAnswer;
				// initialize CRC
				dpaUartIfControl.CRC = 0xFF;
				// maximal size of received data
				dpaUartIfControl.packetLen = sizeof (T_DPA_PACKET);
				// counter of received bytes
				dpaUartIfControl.packetCnt = 0;
				// clear Escape flag
				dpaUartIfControl.wasEscape = 0;
				// reading from TR module
				dpaUartIfControl.direction = UART_TRANSFER_READ;
				// library si busy
				dpaControl.status = DPA_BUSY;
			}
			return;
		}
		// check if packet to send is ready
		if (dpaControl.dpaRequestPacketPtr != NULL) {
			// set pointer to DpaRequest packet
			dpaUartIfControl.dpaPacketPtr = dpaControl.dpaRequestPacketPtr;
			dpaControl.dpaRequestPacketPtr = NULL;
			// NADR + PNUM + PCMD + HWPID + Data
			dpaUartIfControl.packetLen = dpaControl.extraDataSize + 6;
			// initialize CRC
			dpaUartIfControl.CRC = 0xFF;
			// counter of sent bytes
			dpaUartIfControl.packetCnt = 0;
			// write data to TR module
			dpaUartIfControl.direction = UART_TRANSFER_WRITE;
			// library si busy
			dpaControl.status = DPA_BUSY;
			// send start of packet character
			DPA_SendUartByte(HDLC_FRM_FLAG_SEQUENCE);
		}
	}
}

/**
 * Send data byte to TR module + make HDLC byte stuffing and comute CRC
 * @param dataByte Data byte for TR module
 */
void DPA_SendDataByte(uint8_t dataByte) {
	if (dataByte == HDLC_FRM_FLAG_SEQUENCE || dataByte == HDLC_FRM_CONTROL_ESCAPE) {
		DPA_SendUartByte(HDLC_FRM_CONTROL_ESCAPE);
		DPA_SendUartByte(dataByte ^ HDLC_FRM_ESCAPE_BIT);
	} else {
		DPA_SendUartByte(dataByte);
	}
	dpaUartIfControl.CRC = DPA_doCRC8(dataByte, dpaUartIfControl.CRC);
}

/**
 * Calculate the CRC8 value of a data set
 * @param inData One byte of data to compute CRC
 * @param seed The starting value of the CRC
 * @return The CRC8 of inData with seed as initial value
 */
uint8_t DPA_doCRC8(uint8_t inData, uint8_t seed) {
	for (uint8_t bitsLeft = 8; bitsLeft > 0; bitsLeft--) {
		if (((seed ^ inData) & 0x01) == 0) {
			seed >>= 1;
		} else {
			seed = (seed >>= 1)^0x8C;
		}
		inData >>= 1;
	}
	return seed;
}

#endif
