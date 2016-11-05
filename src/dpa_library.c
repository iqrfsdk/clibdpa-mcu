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

uint8_t DPA_SendSpiByte(uint8_t txByte);
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

void DPA_SendUartByte(uint8_t txByte);
void DPA_ReceiveUartByte(uint8_t rxByte);
void DPA_UartInterfaceDriver(void);
void DPA_SendDataByte(uint8_t dataByte);
uint8_t DPA_doCRC8(uint8_t inData, uint8_t seed);

#endif

#if defined (__STORE_CODE_SUPPORT__) && defined(TR7xD)

void DPA_StoreCodeAnswerHandler(T_DPA_PACKET *dpaAnswerPkt);
void DPA_StoreCodeTimeoutHandler(void);
uint16_t DPA_FlatcherCRC16(uint8_t *dataAddress, uint8_t dataLen, uint16_t seed);
void DPA_SendDataToEeeprom(T_DPA_CODE_FILE_INFO *codeFileInfo, uint8_t dataSize);
void DPA_ProcessHexCodeFile(T_DPA_CODE_FILE_INFO *codeFileInfo);
void DPA_ProcessIqrfCodeFile(T_DPA_CODE_FILE_INFO *codeFileInfo);
uint8_t DPA_ConvertToNum(uint8_t dataByteHi, uint8_t dataByteLo);
uint8_t DPA_ReadHEXFileLine(void);
uint8_t DPA_ReadIQRFFileLine(void);
extern uint8_t DPA_ReadByteFromFile(void);

T_DPA_PACKET dpaStoreCodeRequest;
uint8_t dpaCodeLineBuffer[32];
uint8_t dpaStartAddressDetected;
uint8_t dpaCodeImageEndDetected;
uint8_t dpaOutputPacketDataCnt;
uint8_t dpaEeepromPageDataCnt;
uint8_t dpaRemainingDataCnt;
uint8_t dpaOperationResult;
uint16_t dpaCodeLineAddress;
uint16_t dpaCodeImageNextAddress;

enum {
    DPA_STORE_CODE_INIT_TASK = 0,
    DPA_STORE_CODE_SEND_DATA,
    DPA_STORE_CODE_WAIT_CONFIRMATION,
    DPA_STORE_CODE_CONFIRMATION_ERR,
    DPA_STORE_CODE_WAIT_RESPONSE,
    DPA_STORE_CODE_RESPONSE_ERR,
    DPA_STORE_CODE_TIMEOUT,
    DPA_STORE_CODE_END
} dpaStoreCodeTaskSM = DPA_STORE_CODE_INIT_TASK;

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
    dpaControl.dpaTimeoutHandler = NULL;
    dpaControl.timeFlag = 0;
    dpaControl.timeoutTimer = 0;
    dpaControl.timeoutModulator = 0;
    dpaControl.timeoutPrescaller = 7;

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
    dpaControl.status = DPA_PREPARE; // ?
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

/**
 * Setting time flag for DPA_LibraryDriver function
 * Must be called periodicaly every 1 ms (TR5xD) or 150us (TR7xD)
 */
void DPA_SetTimmingFlag(void) {
    // internal library timming flag
    dpaControl.timeFlag = 1;

#if defined(TR7xD)
    if (--dpaControl.timeoutPrescaller) { // prescaler 7 = 1050us , prescaler 6 = 900us
        return;
    } else if (++dpaControl.timeoutModulator <= 2) { // every third round do timeout timer correction
        dpaControl.timeoutPrescaller = 7;
    } else { // 2 * 1050us + 900us = 3000us
        dpaControl.timeoutModulator = 0;
        dpaControl.timeoutPrescaller = 6;
    }
#endif
    if (dpaControl.timeoutTimer && (!(--dpaControl.timeoutTimer))) { // if service timeout expired
        // call user timeout handler
        dpaControl.dpaTimeoutHandler();
    }
}

/**
 * Calculate CRC from TR module configuration data
 * @param dpaRequest Pointer to DPA request packet
 * @return CRC from TR module configuration data
 */
uint8_t DPA_MakeConfigurationCRC(T_DPA_PACKET *dpaRequest) {
    uint8_t cnt;
    uint8_t crc = 0x5F;
    for (cnt = 0; cnt < 31; cnt++) {
        crc ^= dpaRequest->DpaMessage.PerOSWriteCfg_Request.Configuration[cnt];
    }
    return (crc);
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
            }
            // set pointer to DPA receive structure
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
    unsigned char crc_val, dataSize, i;
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
 * Transfers received byte from UART to DPA support library
 * @param rxByte Received byte
 */
void DPA_ReceiveUartByte(uint8_t rxByte) {
    // Write received byte to Rx FiFo
    dpaUartIfControl.rxBuff[dpaUartIfControl.rxBuffInPtr++] = rxByte;
    // This makes 16 bytes circle buffer
    dpaUartIfControl.rxBuffInPtr &= 0x0F;
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

#if defined(__STORE_CODE_SUPPORT__) && defined(TR7xD)

/**
 * Function for store HEX or IQRF code image to external EEPROM in TR module
 * Function must be called periodicaly until DPA_STORE_CODE_SUCCESS or DPA_STORE_CODE_ERROR is returned
 * @param codeFileInfo Pointer to T_DPA_CODE_FILE_INFO structure with code file image information
 * @return Progress status or operation result
 *       Return code      |        Description
 * ---------------------- | ----------------------------
 *          0-100         |       Progress status
 * DPA_STORE_CODE_SUCCESS | Operation ended with success
 *  DPA_STORE_CODE_ERROR  |  Operation ended with error
 */
uint8_t DPA_StoreCodeToEeeprom(T_DPA_CODE_FILE_INFO *codeFileInfo) {
    static T_DPA_ANSWER_HANDLER tempStoreAnswerHandler;
    static T_DPA_TIMEOUT_HANDLER tempStoreTimeoutHandler;
    static uint16_t tempStoreEepromAddress;

    switch (dpaStoreCodeTaskSM) {
    case DPA_STORE_CODE_INIT_TASK:
        // Initialize code file character read counter
        dpaControl.fileByteCounter = 0;
        // Temporary store users DPA handlers setting
        tempStoreAnswerHandler = dpaControl.dpaAnswerHandler;
        tempStoreTimeoutHandler = dpaControl.dpaTimeoutHandler;
        // Temporary store start address of code image in EEEPROM
        tempStoreEepromAddress = codeFileInfo->imageEeepromAdr;
        // Set new handlers for this task
        DPA_SetAnswerHandler(DPA_StoreCodeAnswerHandler);
        DPA_SetTimeoutHandler(DPA_StoreCodeTimeoutHandler);
        if (codeFileInfo->fileType == DPA_CODE_FILE_IQRF) {
            // Initial value for .IQRF CRC
            codeFileInfo->imageCRC = 0x03;
        } else {
            // Initial value for .HEX CRC
            codeFileInfo -> imageCRC = 0x01;
        }
        // Initialize file image size
        codeFileInfo->imageSize = 0;
        // Initialize work variables
        dpaStartAddressDetected = dpaCodeImageEndDetected = dpaOutputPacketDataCnt = dpaEeepromPageDataCnt = dpaRemainingDataCnt = 0;
        // Start data sending
        dpaStoreCodeTaskSM = DPA_STORE_CODE_SEND_DATA;
        return (0);
    case DPA_STORE_CODE_SEND_DATA:
        if (codeFileInfo->fileType == DPA_CODE_FILE_IQRF) {
            // Process .IQRF file
            DPA_ProcessIqrfCodeFile(codeFileInfo);
        } else {
            // Process .HEX file
            DPA_ProcessHexCodeFile(codeFileInfo);
        }
        break;
    case DPA_STORE_CODE_WAIT_CONFIRMATION: // Wait for confirmation or response states (do nothing)
    case DPA_STORE_CODE_WAIT_RESPONSE:
        break;
    case DPA_STORE_CODE_CONFIRMATION_ERR: // Service error states
    case DPA_STORE_CODE_RESPONSE_ERR:
    case DPA_STORE_CODE_TIMEOUT:
        dpaOperationResult = DPA_STORE_CODE_ERROR;
    case DPA_STORE_CODE_END:
        // Answer handler = original user answer handler
        DPA_SetAnswerHandler(tempStoreAnswerHandler);
        // Timeout handler = original user timeout handler
        DPA_SetTimeoutHandler(tempStoreTimeoutHandler);
        // Restore start address of code image in EEEPROM
        codeFileInfo->imageEeepromAdr = tempStoreEepromAddress;
        // Initialize task state machine
        dpaStoreCodeTaskSM = DPA_STORE_CODE_INIT_TASK;
        return (dpaOperationResult);

    }
    return (((uint32_t) dpaControl.fileByteCounter * 100) / codeFileInfo->fileSize);
}

/**
 * Answer handler for function DPA_StoreCodeToEeeprom(...)
 * @param dpaAnswerPkt Pointer to T_DPA_PACKET structure containing answer from TR module
 */
void DPA_StoreCodeAnswerHandler(T_DPA_PACKET *dpaAnswerPkt) {
    // stop timeout timmer
    DPA_SetTimeoutTimer(0);

    switch (dpaStoreCodeTaskSM) {
        // Waiting for confirmation
    case DPA_STORE_CODE_WAIT_CONFIRMATION:
        // If confirmation received
        if (dpaAnswerPkt->ResponseCode == STATUS_CONFIRMATION) {
            // Wait for response
            dpaStoreCodeTaskSM = DPA_STORE_CODE_WAIT_RESPONSE;
            // Set timeout timmer to estimated timeout
            DPA_SetTimeoutTimer(DPA_GetEstimatedTimeout());
        } else {
            // Process error in confirmation
            dpaStoreCodeTaskSM = DPA_STORE_CODE_CONFIRMATION_ERR;
        }
        break;
        // Waiting for response
    case DPA_STORE_CODE_WAIT_RESPONSE:
        // If with no error received
        if (dpaAnswerPkt->ResponseCode == STATUS_NO_ERROR) {
            // Prepare data and send next packet
            dpaStoreCodeTaskSM = DPA_STORE_CODE_SEND_DATA;
        } else {
            // Process error in response
            dpaStoreCodeTaskSM = DPA_STORE_CODE_RESPONSE_ERR;
        }
        break;
    }
}

/**
 * Timeout handler for function DPA_StoreCodeToEeeprom(...)
 */
void DPA_StoreCodeTimeoutHandler(void) {
    // Set store code task to timeout state
    dpaStoreCodeTaskSM = DPA_STORE_CODE_TIMEOUT;
}

/**
 * Compute the Flatcher-16 checksum
 * @param inData One byte of data to compute CRC from
 * @param seed The starting value of the CRC
 * @return The FlatcherCRC16 of inData with seed as initial value
 */
uint16_t DPA_FlatcherCRC16(uint8_t *dataAddress, uint8_t dataLen, uint16_t seed) {
    uint16_t tempL = seed & 0xFF;
    uint16_t tempH = seed >> 8;

    while (dataLen--) {
        tempL += *dataAddress;
        dataAddress++;
        if ((tempL & 0x100) != 0) {
            tempL++;
            tempL &= 0xFF;
        }

        tempH += tempL;
        if ((tempH & 0x100) != 0) {
            tempH++;
            tempH &= 0xFF;
        }
    }
    return ((tempH << 8) | tempL);
}

/**
 * Send prepared data packed to eeeprom peripheral and compute FlatcherCRC16 from data
 * @param codeFileInfo Pointer to T_DPA_CODE_FILE_INFO structure with code file image information
 * @param dataSize Number of data bytes in output buffer
 */
void DPA_SendDataToEeeprom(T_DPA_CODE_FILE_INFO *codeFileInfo, uint8_t dataSize) {
    // make CRC from code image block of data
    codeFileInfo->imageCRC = DPA_FlatcherCRC16((uint8_t *) & dpaStoreCodeRequest.DpaMessage.XMemoryRequest.ReadWrite.Write.PData[0], dataSize, codeFileInfo->imageCRC);
    // increase code image size
    codeFileInfo->imageSize += dataSize;
    // set destination address
    dpaStoreCodeRequest.NADR = codeFileInfo->trAddress;
    // set destination peripheral EEEPROM
    dpaStoreCodeRequest.PNUM = PNUM_EEEPROM;
    // set xwrite to eeeprom command
    dpaStoreCodeRequest.PCMD = CMD_EEEPROM_XWRITE;
    // do not check HWPID
    dpaStoreCodeRequest.HWPID = HWPID_DoNotCheck;
    // set destination address in EEEPROM
    dpaStoreCodeRequest.DpaMessage.XMemoryRequest.Address = codeFileInfo->imageEeepromAdr;
    // next write on address
    codeFileInfo->imageEeepromAdr += dataSize;
    // send DPA command
    DPA_SendRequest(&dpaStoreCodeRequest, dataSize + 2);
    // if command for coordinator wait for response
    if (dpaStoreCodeRequest.NADR == COORDINATOR_ADDRESS) {
        dpaStoreCodeTaskSM = DPA_STORE_CODE_WAIT_RESPONSE;
    } else { // if command for node wait for confirmation
        dpaStoreCodeTaskSM = DPA_STORE_CODE_WAIT_CONFIRMATION;
    }
    // Set timeout timmer to 1000 ms
    DPA_SetTimeoutTimer(1000);

}

/**
 * Process HEX file with custom DPA handler code
 * @param codeFileInfo Pointer to T_DPA_CODE_FILE_INFO structure with code file image information
 */
void DPA_ProcessHexCodeFile(T_DPA_CODE_FILE_INFO *codeFileInfo) {
    uint8_t tempValue;
    // if end of code image not detected
    if (!dpaCodeImageEndDetected) {
        // read one line from .HEX file
        if ((tempValue = DPA_ReadHEXFileLine()) != 0) {
            // CRC error in hex file detected
            if (tempValue == 2) {
                // operation ends with error
                dpaOperationResult = DPA_STORE_CODE_ERROR;
                // end operation
                dpaStoreCodeTaskSM = DPA_STORE_CODE_END;
            } else {
                // end of file detected
                dpaCodeImageEndDetected = TRUE;
            }
            return;
        }
        // line from hex file do not contain data, read next line
        if (dpaCodeLineBuffer[3] != 0) {
            return;
        }
        // compute destination code address
        dpaCodeLineAddress = (((uint16_t) dpaCodeLineBuffer[1] << 8) | dpaCodeLineBuffer[2]) / 2;
        // if code address == custom DPA handler start address
        if (dpaCodeLineAddress == CUSTOM_HANDLER_ADDRESS) {
            // set flag, start address detected
            dpaStartAddressDetected = TRUE;
            // set next code address
            dpaCodeImageNextAddress = CUSTOM_HANDLER_ADDRESS;
        }
        // if start address of custom DPA handler was detected
        if (dpaStartAddressDetected == TRUE) {
            // if destination code address is not correct
            if (dpaCodeImageNextAddress != dpaCodeLineAddress || dpaCodeLineAddress > CUSTOM_HANDLER_ADDRESS_END) {
                // set flag, code image end defected
                dpaCodeImageEndDetected = TRUE;
                return;
            }
            // if hex file line buffer contains less then 0x10 data bytes
            if (dpaCodeLineBuffer[0] != 0x10) {
                // set remaining data counter
                dpaRemainingDataCnt = dpaCodeLineBuffer[0];
                // set flag, code image end defected
                dpaCodeImageEndDetected = TRUE;
                return;
            }

            // copy data from hexfile line buffer to output buffer
            memcpy((uint8_t *) & dpaStoreCodeRequest.DpaMessage.XMemoryRequest.ReadWrite.Write.PData[dpaOutputPacketDataCnt], (uint8_t *) & dpaCodeLineBuffer[4], dpaCodeLineBuffer[0]);
            // next code address should be
            dpaCodeImageNextAddress += 0x08;
            // increase data counter in output buffer
            dpaOutputPacketDataCnt += 0x10;
            // if output buffer contains 0x20 bytes
            if (dpaOutputPacketDataCnt == 0x20) {
                // clear output buffer data counter
                dpaOutputPacketDataCnt = 0;
                // increase eeeprom page data counter
                dpaEeepromPageDataCnt += 0x20;
                // if eeeprom data page contains 0x40 bytes, clear page data counter
                if (dpaEeepromPageDataCnt == 0x40) {
                    dpaEeepromPageDataCnt = 0;
                }
                // send 0x20 data bytes of code image to eeeprom preripheral of TR module
                DPA_SendDataToEeeprom(codeFileInfo, 0x20);
            }
        }
    } else { // if end of code image detected
        if (dpaRemainingDataCnt) { // if hex file line buffer contains any data
            // copy it to output buffer
            memcpy((uint8_t *) & dpaStoreCodeRequest.DpaMessage.XMemoryRequest.ReadWrite.Write.PData[dpaOutputPacketDataCnt], (uint8_t *) & dpaCodeLineBuffer[4], dpaRemainingDataCnt);
            // increase output buffer data counter
            dpaOutputPacketDataCnt += dpaRemainingDataCnt;
            // clear hex file line buffer data counter
            dpaRemainingDataCnt = 0;
        } else { // if hex file line buffer contains no data
            // if whole page to eeeprom was written
            if (dpaEeepromPageDataCnt == 0) {
                // code image was written successfully
                dpaOperationResult = DPA_STORE_CODE_SUCCESS;
                // operation end
                dpaStoreCodeTaskSM = DPA_STORE_CODE_END;
                return;
            }
        }
        // if output buffer contains less then 0x20 data bytes
        if (dpaOutputPacketDataCnt < 0x20) {
            // fill rest of buffer with zeros
            memset((uint8_t *) & dpaStoreCodeRequest.DpaMessage.XMemoryRequest.ReadWrite.Write.PData[dpaOutputPacketDataCnt], 0, 0x20 - dpaOutputPacketDataCnt);
            // clear outpt buffer data counter
            dpaOutputPacketDataCnt = 0;
            // increase eeprom page data counter
            dpaEeepromPageDataCnt += 0x20;
            // if eeeprom data page contains 0x40 bytes, clear page data counter
            if (dpaEeepromPageDataCnt == 0x40) {
                dpaEeepromPageDataCnt = 0;
            }
            // send 0x20 data bytes of code image to eeeprom preripheral of TR module
            DPA_SendDataToEeeprom(codeFileInfo, 0x20);
        }
    }
}

/**
 * Process IQRF file with IQRF plugin or new OS
 * @param codeFileInfo Pointer to T_DPA_CODE_FILE_INFO structure with code file image information
 */
void DPA_ProcessIqrfCodeFile(T_DPA_CODE_FILE_INFO *codeFileInfo) {
    uint8_t tempValue;
    // if end of code image not detected
    if (!dpaCodeImageEndDetected) {
        // if iqrf file line buffer contains any data
        if (dpaRemainingDataCnt) {
            // number of bytes to full output buffer
            tempValue = 32 - dpaOutputPacketDataCnt;
            // if required bytes is more then ready bytes in line buffer
            if (tempValue > dpaRemainingDataCnt) {
                tempValue = dpaRemainingDataCnt;
            }
            // copy required bytes from line buffer to out buffer
            memcpy((uint8_t *) & dpaStoreCodeRequest.DpaMessage.XMemoryRequest.ReadWrite.Write.PData[dpaOutputPacketDataCnt], (uint8_t *) & dpaCodeLineBuffer[20 - dpaRemainingDataCnt], tempValue);
            // decrease remaining data counter
            dpaRemainingDataCnt -= tempValue;
            // increase output data counter
            dpaOutputPacketDataCnt += tempValue;
            // if output buffer contains 32 bytes
            if (dpaOutputPacketDataCnt == 32) {
                // clear output buffer data counter
                dpaOutputPacketDataCnt = 0;
                // send 32 data bytes of code image to eeeprom preripheral of TR module
                DPA_SendDataToEeeprom(codeFileInfo, 32);
            }
        } else {
            // read one line from .IQRF file
            if ((tempValue = DPA_ReadIQRFFileLine()) != 0) {
                // wrong format if .IQRF file detected
                if (tempValue == 2) {
                    // operation ends with error
                    dpaOperationResult = DPA_STORE_CODE_ERROR;
                    // end operation
                    dpaStoreCodeTaskSM = DPA_STORE_CODE_END;
                } else {
                    // end of file detected
                    dpaCodeImageEndDetected = TRUE;
                }
                return;
            }
            // number of ready bytes in file line buffer
            dpaRemainingDataCnt = 20;
        }
    } else { // if end of code image was detected
        // if any data still in output buffer
        if (dpaOutputPacketDataCnt) {
            // send rest of data bytes of code image to eeeprom preripheral of TR module
            DPA_SendDataToEeeprom(codeFileInfo, dpaOutputPacketDataCnt);
            // clear output buffer data counter
            dpaOutputPacketDataCnt = 0;
        } else {
            // code image was written successfully
            dpaOperationResult = DPA_STORE_CODE_SUCCESS;
            // operation end
            dpaStoreCodeTaskSM = DPA_STORE_CODE_END;
            return;
        }
    }
}

/**
 * Convert two ASCII char to number
 * @param dataByteHi High nibble in ascii
 * @param dataByteLo Low nibble in ascii
 * @return Number
 */
uint8_t DPA_ConvertToNum(uint8_t dataByteHi, uint8_t dataByteLo) {
    uint8_t result = 0;

    /* convert High nibble */
    if (dataByteHi >= '0' && dataByteHi <= '9') {
        result = (dataByteHi - '0') << 4;
    } else if (dataByteHi >= 'a' && dataByteHi <= 'f') {
        result = (dataByteHi - 87) << 4;
    }
    /* convert Low nibble */
    if (dataByteLo >= '0' && dataByteLo <= '9') {
        result |= (dataByteLo - '0');
    } else if (dataByteLo >= 'a' && dataByteLo <= 'f') {
        result |= (dataByteLo - 87);
    }

    return (result);
}

/**
 * Read and process line from firmware file
 * @return Status code
 * Code |                Description
 * ---- | -------------------------------------------
 *   0  | HEX file line processed and ready in buffer
 *   1  |                End of file
 *   2  |         CRC error in HEX file line
 */
uint8_t DPA_ReadHEXFileLine(void) {
    uint8_t sign;
    uint8_t dataByteHi, dataByteLo;
    uint8_t dataByte;
    uint8_t codeLineBufferPtr = 0;
    uint8_t codeLineBufferCrc = 0;
    // find start of line or end of file
    while (((sign = DPA_ReadByteFromFile()) != 0) && (sign != ':'));
    if (sign == 0) {
        // end of file
        return (1);
    }
    // read data to end of line and convert if to numbers
    for (;;) {
        // read High nibble
        dataByteHi = tolower(DPA_ReadByteFromFile());
        // check end of line
        if (dataByteHi == 0x0A || dataByteHi == 0x0D) {
            // check line CRC
            if (codeLineBufferCrc != 0) {
                return (2);
            }
            // stop reading
            return (0);
        }
        // read Low nibble
        dataByteLo = tolower(DPA_ReadByteFromFile());
        // convert two ascii to number
        dataByte = DPA_ConvertToNum(dataByteHi, dataByteLo);
        // add to CRC
        codeLineBufferCrc += dataByte;
        // store to line buffer
        dpaCodeLineBuffer[codeLineBufferPtr++] = dataByte;
    }
}

/**
 * Read and process line from plugin file
 * @return Status code
 * Code |                 Description
 * ---- | --------------------------------------------
 *   0  | IQRF file line processed and ready in buffer
 *   1  |                 End of file
 *   2  |            Input file format error
 */
uint8_t DPA_ReadIQRFFileLine(void) {
    uint8_t firstChar;
    uint8_t secondChar;
    uint8_t codeLineBufferPtr = 0;

repeat_read:
    // read one char from file
    firstChar = tolower(DPA_ReadByteFromFile());
    // line do not contain code
    if (firstChar == '#') {
        // read data to end of line
        while (((firstChar = DPA_ReadByteFromFile()) != 0) && (firstChar != 0x0D));
    }

    // end of line
    if (firstChar == 0x0D) {
        // read secon code 0x0A
        DPA_ReadByteFromFile();
        if (codeLineBufferPtr == 0) {
            // read another line
            goto repeat_read;
        }
        if (codeLineBufferPtr == 20) {
            // line with data readed successfully
            return (0);
        } else {
            // wrong file format (error)
            return (2);
        }
    }
    // end of file
    if (firstChar == 0) {
        return (1);
    }
    // read second character from code file
    secondChar = tolower(DPA_ReadByteFromFile());
    // convert chars to number and store to buffer
    dpaCodeLineBuffer[codeLineBufferPtr++] = DPA_ConvertToNum(firstChar, secondChar);
    // read next data
    goto repeat_read;
}
#endif
