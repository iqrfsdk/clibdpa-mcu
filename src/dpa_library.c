/**
 * @file DPA support library
 * @version 1.6.0
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

#include <Arduino.h>
#include <ctype.h>
#include "dpa_library.h"

#if defined (__DPA_DEVELOPER_MODE__)

#define MSG_DATA_TO_SEND          0
#define MSG_CONFIRMATION_OK       1
#define MSG_CONFIRMATION_ERROR    2
#define MSG_RESPONSE_OK           3
#define MSG_RESPONSE_ERROR        4

void packetPrinter(uint8_t Message, uint8_t *Buffer, uint8_t DataSize);

#endif

#if defined(__SPI_INTERFACE__)

#define NO_MODULE           0xFF  // SPI not working (HW error)
#define SPI_BUSY            0xFE  // SPI busy in Master disabled mode
#define SPI_DATA_TRANSFER   0xFD  // SPI data transfer in progress
#define SPI_DISABLED        0x00  // SPI not working (disabled)
#define SPI_CRCM_OK         0x3F  // SPI not ready (full buffer, last CRCM OK)
#define SPI_CRCM_ERR        0x3E  // SPI not ready (full buffer, last CRCM error)
#define COMMUNICATION_MODE  0x80  // SPI ready (communication mode)
#define PROGRAMMING_MODE    0x81  // SPI ready (programming mode)
#define DEBUG_MODE          0x82  // SPI ready (debugging mode)
#define SPI_SLOW_MODE       0x83  // SPI not working in background
#define SPI_USER_STOP       0x07  // state after stopSPI();

#define SPI_CHECK           0x00  // Master checks the SPI status of the TR module
#define SPI_WR_RD           0xF0  // Master reads/writes a packet from/to TR module
#define SPI_CRCM_OK         0x3F  // SPI not ready (full buffer, last CRCM OK)
#define SPI_CRCM_ERR        0x3E  // SPI not ready (full buffer, last CRCM error)

#define SPI_STATUS_POOLING_TIME  10  // SPI status pooling time 10ms

enum{
    SPI_TRANSFER_NONE = 0,
    SPI_TRANSFER_WRITE,
    SPI_TRANSFER_READ
};

typedef struct { // SPI interface control structure
    uint8_t DLEN;
    uint8_t CMD;
    uint8_t PTYPE;
    uint8_t CRCM;
    uint8_t MyCRCS;
    uint8_t CRCS;
    uint8_t SpiStat;
    uint8_t Direction;
    uint8_t PacketLen;
    uint8_t PacketCnt;
    uint8_t PacketRpt;
    T_DPA_PACKET *DpaPacketPtr;
} T_DPA_SPI_INTERFACE_CONTROL;

T_DPA_SPI_INTERFACE_CONTROL DpaSpiIfControl;

uint8_t dpaSendSpiByte(uint8_t Tx_Byte);
void dpaDeselectTRmodule(void);
void dpaSpiInterfaceDriver(void);
uint8_t dpaGetCRCM(void);

#elif defined(__UART_INTERFACE__)

#define HDLC_FRM_FLAG_SEQUENCE    0x7E
#define HDLC_FRM_CONTROL_ESCAPE   0x7D
#define HDLC_FRM_ESCAPE_BIT       0x20

enum{
    UART_TRANSFER_NONE = 0,
    UART_TRANSFER_WRITE,
    UART_TRANSFER_READ
};

typedef struct {
    uint8_t Direction;
    uint8_t PacketCnt;
    uint8_t PacketLen;
    uint8_t CRC;
    uint8_t WasEscape;
    T_DPA_PACKET *DpaPacketPtr;
} T_DPA_UART_INTERFACE_CONTROL;

T_DPA_UART_INTERFACE_CONTROL DpaUartIfControl;

void dpaSendUartByte(uint8_t Tx_Byte);
uint8_t dpaReceiveUartByte(uint8_t *Rx_Byte);
void dpaUartInterfaceDriver(void);
void dpaSendDataByte(uint8_t DataByte);
uint8_t dpaDoCRC8(uint8_t InData, uint8_t Seed);

#endif

#if defined(__STORE_CODE_SUPPORT__)

#define HEX_FILE_LINE_READY  0
#define HEX_FILE_CRC_ERR  1
#define HEX_END_OF_FILE 2

#define IQRF_FILE_LINE_READY  0
#define IQRF_FILE_FORMAT_ERR  1
#define IQRF_END_OF_FILE 2

uint16_t dpaFlatcherCRC16(uint8_t *DataAddress, uint8_t DataLen, uint16_t Seed);
void dpaSendDataToEeeprom(T_DPA_CODE_FILE_INFO *CodeFileInfo, uint8_t DataSize);
void dpaProcessHexCodeFile(T_DPA_CODE_FILE_INFO *CodeFileInfo);
void dpaProcessIqrfCodeFile(T_DPA_CODE_FILE_INFO *CodeFileInfo);
uint8_t dpaReadHEXFileLine(void);
uint8_t dpaReadIQRFFileLine(void);
extern uint8_t dpaReadByteFromFile(void);

T_DPA_PACKET DpaStoreCodeRequest;
uint8_t DpaCodeLineBuffer[32];
uint8_t DpaStartAddressDetected;
uint8_t DpaCodeImageEndDetected;
uint8_t DpaOutputPacketDataCnt;
uint8_t DpaEeepromPageDataCnt;
uint8_t DpaRemainingDataCnt;
uint16_t DpaCodeLineAddress;
uint16_t DpaCodeImageNextAddress;

typedef enum {
    DPA_STORE_CODE_INIT_TASK = 0,
    DPA_STORE_CODE_SEND_DATA,
    DPA_STORE_CODE_ERROR_END,
    DPA_STORE_CODE_SUCCESS_END
} DPA_STORE_CODE_TASK_SM;

DPA_STORE_CODE_TASK_SM DpaStoreCodeTaskSM = DPA_STORE_CODE_INIT_TASK;

#endif

 // internal states of DPA operation state machine
typedef enum{
    DPA_SM_PREPARE_REQUEST = 0,
    DPA_SM_WAIT_CONFIRMATION,
    DPA_SM_PROCESS_CONFIRMATION,
    DPA_SM_CONFIRMATION_ERR,
    DPA_SM_WAIT_RESPONSE,
    DPA_SM_PROCESS_RESPONSE,
    DPA_SM_RESPONSE_ERR,
    DPA_SM_WAIT_BROADCAST_ROUTING,
    DPA_SM_TIMEOUT
} DPA_SM;

enum{
    DPA_READY = 0x00,                     // DPA support library ready
    DPA_NOT_READY,                        // DPA support library not ready
    DPA_BUSY                              // DPA request processing
};

#define dpaSetTimeoutTimer(A1)  DpaControl.TimeoutTimer = A1

/* Function prototypes */
void dpaAnswerHandler(T_DPA_PACKET *dpaAnswerPkt);
void dpaTimeoutHandler(void);
uint16_t dpaGetEstimatedTimeout(void);

/* Public variable declarations */
T_DPA_CONTROL DpaControl;
T_DPA_PACKET DpaLibDpaAnswer;
T_DPA_ANSWER_HANDLER tempReceiverHandler;
uint8_t LastConfirmationData[11];

volatile DPA_SM DpaApplicationSM = DPA_SM_PREPARE_REQUEST;
uint16_t DpaAditionalTimeout;

/**
 * Function initialize DPA support library
 * @param asyncPacketHandler Pointer to user call back function for asynchronous packet service
 */
void dpaInit(T_DPA_ANSWER_HANDLER asyncPacketHandler)
{
    DpaControl.Status = DPA_READY;
    DpaControl.DpaAnswerHandler = asyncPacketHandler;
    DpaControl.DpaTimeoutHandler = dpaTimeoutHandler;
    DpaControl.TimeoutTimer = 0;
    DpaControl.TimeoutPrescaller = 5;

#if defined(__SPI_INTERFACE__)
    DpaControl.TRmoduleSelected = false;
    DpaControl.TimeCnt = (SPI_STATUS_POOLING_TIME * 7) + 1;
    DpaSpiIfControl.SpiStat = 0;
    DpaSpiIfControl.Direction = SPI_TRANSFER_NONE;
#elif defined(__UART_INTERFACE__)
    DpaUartIfControl.Direction = UART_TRANSFER_NONE;
    DpaUartIfControl.WasEscape = 0;
#endif
}

/**
 * Function provides background communication with TR module
 */
void dpaLibraryDriver(void)
{
    if (DpaControl.SuspendFlag)
        return;

#if defined(__SPI_INTERFACE__)
    if (DpaControl.Status == DPA_BUSY || !DpaControl.TimeCnt) {
        dpaSpiInterfaceDriver();
        DpaControl.TimeCnt = (SPI_STATUS_POOLING_TIME * 5) + 1;
    }
    DpaControl.TimeCnt--;
#elif defined(__UART_INTERFACE__)
    dpaUartInterfaceDriver();
#endif

    // prescaler 5 = 1000us
    if (--DpaControl.TimeoutPrescaller)
        return;
    DpaControl.TimeoutPrescaller = 5;

    if (DpaControl.TimeoutTimer)                        // service timeout timer
        if (!(--DpaControl.TimeoutTimer))               // if timeout expired
            if (DpaControl.DpaTimeoutHandler != NULL)   // call user timeout handler
                DpaControl.DpaTimeoutHandler();
}

/**
 * Sends DPA request packet to desired destination address
 * @param DpaRequest Pointer to DPA request packet
 * @param DataSize Number of additional data bytes in DPA request packet
 * @param Timeout Operation timeout in ms
 * @return Operation result (DPA_OPERATION_OK, DPA_OPERATION_IN_PROGRESS, DPA_OPERATION_TIMEOUT ... )
 */
DPA_OPERATION_RESULT dpaSendRequest(T_DPA_PACKET *DpaRequest, uint8_t DataSize, uint16_t Timeout)
{
    DPA_OPERATION_RESULT OperationResult = DPA_OPERATION_IN_PROGRESS;

    // DPA operation state machine
    switch (DpaApplicationSM) {
    case DPA_SM_PREPARE_REQUEST:
        // if TR module is not ready
        if (DpaControl.Status == DPA_NOT_READY)
            return(DPA_TR_MODULE_NOT_READY);

        // wait until library is ready
        while (DpaControl.Status == DPA_BUSY)
            ; /* void */

#if defined (__DPA_DEVELOPER_MODE__)
        packetPrinter(MSG_DATA_TO_SEND, (uint8_t *)DpaRequest, 6 + DataSize);
#endif
        systemDisableInt(); // disable interrupts
        // save pointer to async packet user service routine
        tempReceiverHandler = DpaControl.DpaAnswerHandler;
        // set pointer to DPA system receiver handler
        DpaControl.DpaAnswerHandler = dpaAnswerHandler;
        // initialize transfer parameters
        DpaControl.DpaRequestPacketPtr = DpaRequest;
        DpaControl.ExtraDataSize = DataSize;
        DpaControl.RequestTs = millis();
        DpaControl.WasConfirmed = false;
        DpaControl.ConfirmationTs = 0;
        DpaControl.ResponseTs = 0;
        // set operation timeout
        dpaSetTimeoutTimer(Timeout);
        // enable interrupts
        systemEnableInt();

        // if command for coordinator or local interface,  wait for response
        if (DpaRequest->NADR == COORDINATOR_ADDRESS || DpaRequest->NADR == LOCAL_ADDRESS)
            DpaApplicationSM = DPA_SM_WAIT_RESPONSE;
        else  // if command for node, wait for confirmation
            DpaApplicationSM = DPA_SM_WAIT_CONFIRMATION;
        DpaControl.BroadcastRoutingFlag = false;
        break;

    // confirmation received OK
    case DPA_SM_PROCESS_CONFIRMATION:
#if defined (__DPA_DEVELOPER_MODE__)
        packetPrinter(MSG_CONFIRMATION_OK, (uint8_t *)LastConfirmationData, 11);
#endif
        if (DpaControl.BroadcastRoutingFlag)
            DpaApplicationSM = DPA_SM_WAIT_BROADCAST_ROUTING;   // wait for broadcast routing
        else
            DpaApplicationSM = DPA_SM_WAIT_RESPONSE;            // wait for response
        break;

    // response received OK
    case DPA_SM_PROCESS_RESPONSE:
#if defined (__DPA_DEVELOPER_MODE__)
        packetPrinter(MSG_RESPONSE_OK, (uint8_t *)&DpaLibDpaAnswer, 8 + DpaControl.RdExtraDataSize);
#endif
        // copy received DPA response packet to user DPA packet structure
        memcpy((uint8_t *)DpaRequest, (uint8_t *)&DpaLibDpaAnswer, sizeof(T_DPA_PACKET));
        OperationResult = DPA_OPERATION_OK;
        break;

    // waiting for confirmation, response or broadcast routing
    case DPA_SM_WAIT_CONFIRMATION:
    case DPA_SM_WAIT_RESPONSE:
    case DPA_SM_WAIT_BROADCAST_ROUTING:
        break;

    // confirmation error
    case DPA_SM_CONFIRMATION_ERR:
#if defined (__DPA_DEVELOPER_MODE__)
        packetPrinter(MSG_CONFIRMATION_ERROR, (uint8_t *)LastConfirmationData, 11);
#endif
        OperationResult = DPA_CONFIRMATION_ERR;
        break;

    // response error
    case DPA_SM_RESPONSE_ERR:
#if defined (__DPA_DEVELOPER_MODE__)
        packetPrinter(MSG_RESPONSE_ERROR, (uint8_t *)&DpaLibDpaAnswer, 8 + DpaControl.RdExtraDataSize);
#endif
        OperationResult = DPA_RESPONSE_ERR;
        memcpy((uint8_t *)DpaRequest, (uint8_t *)&DpaLibDpaAnswer, sizeof(T_DPA_PACKET));
        break;

    // operation timeout
    case DPA_SM_TIMEOUT:
        if (DpaControl.BroadcastRoutingFlag)
            OperationResult = DPA_OPERATION_OK;
        else
            OperationResult = DPA_OPERATION_TIMEOUT;
        break;
    }

    if (OperationResult != DPA_OPERATION_IN_PROGRESS) {
        systemDisableInt(); // disable interrupts
        // restore pointer to async packet user service routine
        DpaControl.DpaAnswerHandler = tempReceiverHandler;
        systemEnableInt(); // enable interrupts
        // DPA operation end, initialize state machine for next request
        DpaApplicationSM = DPA_SM_PREPARE_REQUEST;
    }

    return(OperationResult);
}

/**
 * Returns estimated timeout for response packet in milliseconds
 * @return Estimated timeout for response packet in ms (computed from confirmation packet data)
 */
uint16_t dpaGetEstimatedTimeout(void)
{
    uint16_t ResponseTimeSlotLength;
    uint16_t EstimatedTimeout = (uint16_t) (DpaLibDpaAnswer.DpaMessage.IFaceConfirmation.Hops + 1) * (uint16_t) DpaLibDpaAnswer.DpaMessage.IFaceConfirmation.TimeSlotLength * 10;
    // DPA in diagnostic mode
    if (DpaLibDpaAnswer.DpaMessage.IFaceConfirmation.TimeSlotLength == 20) {
        ResponseTimeSlotLength = 200;
    } else {
        if (DpaLibDpaAnswer.DpaMessage.IFaceConfirmation.TimeSlotLength > 6)
            ResponseTimeSlotLength = 110;       // DPA in LP mode
        else
            ResponseTimeSlotLength = 60;        // DPA in STD mode
    }
    EstimatedTimeout += ((uint16_t) (DpaLibDpaAnswer.DpaMessage.IFaceConfirmation.HopsResponse + 1) * ResponseTimeSlotLength + 40);
    return(EstimatedTimeout);
}

/**
 * Makes CRC from TR module configuration data
 * @param DpaRequest Pointer to DPA request packet
 * @return Configuration data CRC
 */
uint8_t dpaMakeConfigurationCRC(T_DPA_PACKET *DpaRequest)
{
    uint8_t i;
    uint8_t crc = 0x5F;

    for (i = 0; i < 31; i++)
        crc ^= DpaRequest->DpaMessage.PerOSWriteCfg_Request.Configuration[i];
    return(crc);
}

/**
 * Temporary suspend DPA communication driver
 */
void dpaSuspendDriver(void)
{
    // wait until library is ready
    while (DpaControl.Status == DPA_BUSY)
        ; /* void */

    // set driver suspend flag
    DpaControl.SuspendFlag = true;
}

/**
 * Run DPA communication driver
 */
void dpaRunDriver(void)
{
    // re-enable DPA driver running
    DpaControl.SuspendFlag = false;
}

/**
 * Callback function for received packet from TR module check
 * @param dpaAnswerPkt Pointer to data structure containing received packet from TR module
 */
void dpaAnswerHandler(T_DPA_PACKET *dpaAnswerPkt)
{
    // stop timeout timer
    dpaSetTimeoutTimer(0);

    switch (DpaApplicationSM) {
    // waiting for confirmation
    case DPA_SM_WAIT_CONFIRMATION:
        memcpy(LastConfirmationData, dpaAnswerPkt, 11);
        DpaControl.ConfirmationTs = millis();
        DpaControl.WasConfirmed = true;
        // if confirmation received
        if (dpaAnswerPkt->ResponseCode == STATUS_CONFIRMATION) {
            // process confirmation
            DpaApplicationSM = DPA_SM_PROCESS_CONFIRMATION;
            // set timeout timer to estimated timeout
            dpaSetTimeoutTimer(dpaGetEstimatedTimeout() + DpaAditionalTimeout);
            // check if we send broadcast request
            if (dpaAnswerPkt->NADR == BROADCAST_ADDRESS)
                DpaControl.BroadcastRoutingFlag = true;
        } else {
            // process error in confirmation
            DpaApplicationSM = DPA_SM_CONFIRMATION_ERR;
        }
        break;

    // waiting for response
    case DPA_SM_WAIT_RESPONSE:
        DpaControl.ResponseTs = millis();
        // if no error received, process response
        if (dpaAnswerPkt->ResponseCode == STATUS_NO_ERROR)
            DpaApplicationSM = DPA_SM_PROCESS_RESPONSE;
        else
            DpaApplicationSM = DPA_SM_RESPONSE_ERR;
        break;

    default:
        break;
    }
}

/**
 * Callback function for service timeout elapsed
 */
void dpaTimeoutHandler(void)
{
    DpaApplicationSM = DPA_SM_TIMEOUT;
}

/**
 * Convert two ASCII chars to number
 * @param dataByteHi High nibble in ASCII
 * @param dataByteLo Low nibble in ASCII
 * @return Number
 */
uint8_t dpaConvertToNum(uint8_t dataByteHi, uint8_t dataByteLo)
{
    uint8_t result = 0;

    dataByteHi = tolower(dataByteHi);
    dataByteLo = tolower(dataByteLo);

    /* convert High nibble */
    if (dataByteHi >= '0' && dataByteHi <= '9')
        result = (dataByteHi - '0') << 4;
    else if (dataByteHi >= 'a' && dataByteHi <= 'f')
        result = (dataByteHi - 87) << 4;

    /* convert Low nibble */
    if (dataByteLo >= '0' && dataByteLo <= '9')
        result |= (dataByteLo - '0');
    else if (dataByteLo >= 'a' && dataByteLo <= 'f')
        result |= (dataByteLo - 87);

    return(result);
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
    if (DpaSpiIfControl.Direction != SPI_TRANSFER_NONE) {

        if (DpaSpiIfControl.Direction == SPI_TRANSFER_WRITE) {
            switch (DpaSpiIfControl.PacketCnt) {
            case 0:
                // send SPI CMD
                dpaSendSpiByte(DpaSpiIfControl.CMD);
                DpaSpiIfControl.MyCRCS = 0x5F;
                break;

            case 1:
                // send PTYPE
                dpaSendSpiByte(DpaSpiIfControl.PTYPE);
                DpaSpiIfControl.MyCRCS ^= DpaSpiIfControl.PTYPE;
                break;

            case 2:
                // send LOW(NADR)
                DpaSpiIfControl.MyCRCS ^= dpaSendSpiByte((DpaSpiIfControl.DpaPacketPtr->NADR) & 0x00FF);
                break;

            case 3:
                // send HIGH(NADR)
                DpaSpiIfControl.MyCRCS ^= dpaSendSpiByte((DpaSpiIfControl.DpaPacketPtr->NADR) >> 8);
                break;

            case 4:
                // send PNUM
                DpaSpiIfControl.MyCRCS ^= dpaSendSpiByte(DpaSpiIfControl.DpaPacketPtr->PNUM);
                break;

            case 5:
                // send PCMD
                DpaSpiIfControl.MyCRCS ^= dpaSendSpiByte(DpaSpiIfControl.DpaPacketPtr->PCMD);
                break;

            case 6:
                // send LOW(HWPID)
                DpaSpiIfControl.MyCRCS ^= dpaSendSpiByte((DpaSpiIfControl.DpaPacketPtr->HWPID) & 0x00FF);
                break;

            case 7:
                // send HIGH(HWPID)
                DpaSpiIfControl.MyCRCS ^= dpaSendSpiByte((DpaSpiIfControl.DpaPacketPtr->HWPID) >> 8);
                break;

            default:
                // send CRCM / receive CRCS
                if (DpaSpiIfControl.PacketCnt == DpaSpiIfControl.PacketLen - 2) {
                    DpaSpiIfControl.CRCS = dpaSendSpiByte(DpaSpiIfControl.CRCM);
                } else {
                    // receive SPI_STATUS
                    if (DpaSpiIfControl.PacketCnt == DpaSpiIfControl.PacketLen - 1)
                        DpaSpiIfControl.SpiStat = dpaSendSpiByte(0);
                    else // send additional data
                        DpaSpiIfControl.MyCRCS ^= dpaSendSpiByte(DpaSpiIfControl.DpaPacketPtr->DpaMessage.Request.PData[DpaSpiIfControl.PacketCnt - 8]);
                }
            }
        } else {
            if (DpaSpiIfControl.PacketCnt >= 2 && DpaSpiIfControl.PacketCnt < DpaSpiIfControl.PacketLen - 2) {
                TempData = dpaSendSpiByte(0);
                DpaSpiIfControl.MyCRCS ^= TempData;
            }

            switch (DpaSpiIfControl.PacketCnt) {
            case 0:
                // send SPI CMD
                dpaSendSpiByte(DpaSpiIfControl.CMD);
                DpaSpiIfControl.MyCRCS = 0x5F;
                break;

            case 1:
                // send PTYPE
                dpaSendSpiByte(DpaSpiIfControl.PTYPE);
                DpaSpiIfControl.MyCRCS ^= DpaSpiIfControl.PTYPE;
                break;

            case 2:
                // receive LOW(NADR)
                DpaSpiIfControl.DpaPacketPtr->NADR = TempData;
                break;

            case 3:
                // receive HIGH(NADR)
                DpaSpiIfControl.DpaPacketPtr->NADR |= ((uint16_t) TempData << 8);
                break;

            case 4:
                // receive PNUM
                DpaSpiIfControl.DpaPacketPtr->PNUM = TempData;
                break;

            case 5:
                // receive PCMD
                DpaSpiIfControl.DpaPacketPtr->PCMD = TempData;
                break;

            case 6:
                // receive LOW(HWPID)
                DpaSpiIfControl.DpaPacketPtr->HWPID = TempData;
                break;

            case 7:
                // receive HIGH(HWPID)
                DpaSpiIfControl.DpaPacketPtr->HWPID |= ((uint16_t) TempData << 8);
                break;

            default:
                if (DpaSpiIfControl.PacketCnt == DpaSpiIfControl.PacketLen - 2)         // send CRCM / receive CRCS
                    DpaSpiIfControl.CRCS = dpaSendSpiByte(DpaSpiIfControl.CRCM);
                else if (DpaSpiIfControl.PacketCnt == DpaSpiIfControl.PacketLen - 1)    // receive SPI_STATUS
                    DpaSpiIfControl.SpiStat = dpaSendSpiByte(0);
                else if (DpaSpiIfControl.PacketCnt == 8)                                // receive ResponseCode
                    DpaSpiIfControl.DpaPacketPtr->ResponseCode = TempData;
                else if (DpaSpiIfControl.PacketCnt == 9)                                // receive DpaValue
                    DpaSpiIfControl.DpaPacketPtr->DpaValue = TempData;
                else                                                                    // receive additional data
                    DpaSpiIfControl.DpaPacketPtr->DpaMessage.Response.PData[DpaSpiIfControl.PacketCnt - 10] = TempData;
            }
        }
        // counts number of send/receive bytes
        DpaSpiIfControl.PacketCnt++;
        // sent everything ?
        if (DpaSpiIfControl.PacketCnt == DpaSpiIfControl.PacketLen) {
            dpaDeselectTRmodule();
            // CRC ok ?
            if ((DpaSpiIfControl.SpiStat == SPI_CRCM_OK) && (DpaSpiIfControl.CRCS == DpaSpiIfControl.MyCRCS)) {
                if (DpaSpiIfControl.Direction == SPI_TRANSFER_READ && DpaControl.DpaAnswerHandler != NULL) {
                    // remember size of received extra data
                    if (DpaSpiIfControl.PacketCnt >= 12)
                        DpaControl.RdExtraDataSize = DpaSpiIfControl.PacketCnt - 12;
                    else
                        DpaControl.RdExtraDataSize = 0;
                    // call user response handler
                    DpaControl.DpaAnswerHandler(DpaSpiIfControl.DpaPacketPtr);
                }
                // library is ready for next packet
                DpaControl.Status = DPA_READY;
                // stop data transfer
                DpaSpiIfControl.Direction = SPI_TRANSFER_NONE;
            } else { // CRC error
                // pktRepeats - must be set on packet preparing
                if (--DpaSpiIfControl.PacketRpt) {
                    // another attempt to send data
                    DpaSpiIfControl.PacketCnt = 0;
                }
                else {
                    // library is ready for next packet
                    DpaControl.Status = DPA_READY;
                    // stop data transfer
                    DpaSpiIfControl.Direction = SPI_TRANSFER_NONE;
                }
            }

            DpaSpiIfControl.SpiStat = 0; // current SPI status must be updated
        }
    } else { // no data to send => SPI status will be updated
        // get SPI status of TR module
        LastSpiStat = dpaSendSpiByte(SPI_CHECK);
        dpaDeselectTRmodule();

        // the status must be 2x the same
        if (DpaSpiIfControl.SpiStat != LastSpiStat) {
            DpaSpiIfControl.SpiStat = LastSpiStat;
            return;
        }

        // if TR module is in communication mode, DPA library is ready
        if (DpaSpiIfControl.SpiStat == COMMUNICATION_MODE)
            DpaControl.Status = DPA_READY;
        else
            DpaControl.Status = DPA_NOT_READY;

        // if the status is data ready, prepare packet to read it
        if ((DpaSpiIfControl.SpiStat & 0xC0) == 0x40) {
            // stav 0x40 znamena nabidku 64B
            if (DpaSpiIfControl.SpiStat == 0x40)
                DpaSpiIfControl.DLEN = 64;
            else
                // clear bit 7,6 - rest is length (1 to 63B)
                DpaSpiIfControl.DLEN = DpaSpiIfControl.SpiStat & 0x3F;

            // set pointer to DPA receive structure
            DpaSpiIfControl.DpaPacketPtr = &DpaLibDpaAnswer;
            DpaSpiIfControl.CMD = SPI_WR_RD;  // read / write data
            DpaSpiIfControl.PTYPE = DpaSpiIfControl.DLEN;
            // CRCM
            DpaSpiIfControl.CRCM = 0x5F ^ DpaSpiIfControl.CMD ^ DpaSpiIfControl.PTYPE;
            // length of whole packet + (CMD, PTYPE, CRCM, 0)
            DpaSpiIfControl.PacketLen = DpaSpiIfControl.DLEN + 4;
            // counter of sent bytes
            DpaSpiIfControl.PacketCnt = 0;
            // number of attempts to send data
            DpaSpiIfControl.PacketRpt = 1;
            // reading from buffer COM of TR module
            DpaSpiIfControl.Direction = SPI_TRANSFER_READ;
            // current SPI status must be updated
            DpaSpiIfControl.SpiStat = 0;
            // library is busy
            DpaControl.Status = DPA_BUSY;
            return;
        }
        // check if packet to send is ready
        if (DpaControl.DpaRequestPacketPtr != NULL) {
            // set pointer to DpaRequest packet
            DpaSpiIfControl.DpaPacketPtr = DpaControl.DpaRequestPacketPtr;
            DpaControl.DpaRequestPacketPtr = NULL;
            // NADR + PNUM + PCMD + HWPID + Data
            DpaSpiIfControl.DLEN = DpaControl.ExtraDataSize + 6;
            DpaSpiIfControl.CMD = SPI_WR_RD;
            // PBYTE set bit7 - write to buffer COM of TR module
            DpaSpiIfControl.PTYPE = (DpaSpiIfControl.DLEN | 0x80);
            DpaSpiIfControl.CRCM = dpaGetCRCM(); // CRCM
            // length of whole packet + (CMD, PTYPE, CRCM, 0)
            DpaSpiIfControl.PacketLen = DpaSpiIfControl.DLEN + 4;
            // counter of sent bytes
            DpaSpiIfControl.PacketCnt = 0;
            // number of attempts to send data
            DpaSpiIfControl.PacketRpt = 3;
            // reading from buffer COM of TR module
            DpaSpiIfControl.Direction = SPI_TRANSFER_WRITE;
            // current SPI status must be updated
            DpaSpiIfControl.SpiStat = 0;
            // library is busy
            DpaControl.Status = DPA_BUSY;
        }
    }
}

/**
 * Calculate CRC before master's send (see IQRF SPI user manual)
 * @return Value of CRC
 */
uint8_t dpaGetCRCM(void)
{
    unsigned char Cnt, CrcVal, DataSize;

    // initialize CRC
    CrcVal = 0x5F;
    // add SPI CMD
    CrcVal ^= DpaSpiIfControl.CMD;
    // add PTYPE
    CrcVal ^= DpaSpiIfControl.PTYPE;
    // add LOW(NADR)
    CrcVal ^= DpaSpiIfControl.DpaPacketPtr->NADR & 0x00FF;
    // add HIGH(NADR)
    CrcVal ^= DpaSpiIfControl.DpaPacketPtr->NADR >> 8;
    // add PNUM
    CrcVal ^= DpaSpiIfControl.DpaPacketPtr->PNUM;
    // add PCMD
    CrcVal ^= DpaSpiIfControl.DpaPacketPtr->PCMD;
    // add LOW(HWPID)
    CrcVal ^= DpaSpiIfControl.DpaPacketPtr->HWPID & 0x00FF;
    // add HIGH(HWPID)
    CrcVal ^= DpaSpiIfControl.DpaPacketPtr->HWPID >> 8;

    // number of extra data bytes (except NADR, PNUM, PCMD, HWPID)
    DataSize = DpaSpiIfControl.DLEN - 6;
    for (Cnt = 0; Cnt < DataSize; Cnt++)
        CrcVal ^= DpaSpiIfControl.DpaPacketPtr->DpaMessage.Request.PData[Cnt];

    return CrcVal;
}

#elif defined(__UART_INTERFACE__)

/**
 * Function implements IQRF packet communication over UART with TR module
 */
void dpaUartInterfaceDriver(void)
{
    uint8_t TempData;
    // is anything to send / receive
    if (DpaUartIfControl.Direction != UART_TRANSFER_NONE) {
        if (DpaUartIfControl.Direction == UART_TRANSFER_READ) {
            // no data in Rx buffer
            if (!dpaReceiveUartByte(&TempData))
                return;

            // end of packet or DPA structure is full
            if (TempData == HDLC_FRM_FLAG_SEQUENCE || DpaUartIfControl.PacketCnt == DpaUartIfControl.PacketLen) {
                if (DpaUartIfControl.CRC == 0 && DpaControl.DpaAnswerHandler != NULL) {
                    // remember size of received extra data
                    if (DpaUartIfControl.PacketCnt >= 8)
                        DpaControl.RdExtraDataSize = DpaUartIfControl.PacketCnt - 8;
                    else
                        DpaControl.RdExtraDataSize = 0;
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
            if (TempData == HDLC_FRM_CONTROL_ESCAPE) {
                DpaUartIfControl.WasEscape = 1;
                return;
            }

            // previous character was ESCAPE
            if (DpaUartIfControl.WasEscape) {
                DpaUartIfControl.WasEscape = 0;
                TempData ^= HDLC_FRM_ESCAPE_BIT;
            }

            // add Rx byte to CRC
            DpaUartIfControl.CRC = dpaDoCRC8(TempData, DpaUartIfControl.CRC);

            switch (DpaUartIfControl.PacketCnt) {
            case 0:
                // receive LOW(NADR)
                DpaUartIfControl.DpaPacketPtr->NADR = TempData;
                break;

            case 1:
                // receive HIGH(NADR)
                DpaUartIfControl.DpaPacketPtr->NADR |= ((uint16_t) TempData << 8);
                break;

            case 2:
                // receive PNUM
                DpaUartIfControl.DpaPacketPtr->PNUM = TempData;
                break;

            case 3:
                // receive PCMD
                DpaUartIfControl.DpaPacketPtr->PCMD = TempData;
                break;

            case 4:
                // receive LOW(HWPID)
                DpaUartIfControl.DpaPacketPtr->HWPID = TempData;
                break;

            case 5:
                // receive HIGH(HWPID)
                DpaUartIfControl.DpaPacketPtr->HWPID |= ((uint16_t) TempData << 8);
                break;

            case 6:
                // receive ResponseCode
                DpaUartIfControl.DpaPacketPtr->ResponseCode = TempData;
                break;

            case 7:
                // receive DpaValue
                DpaUartIfControl.DpaPacketPtr->DpaValue = TempData;
                break;

            default:
                // receive additional data
                DpaUartIfControl.DpaPacketPtr->DpaMessage.Response.PData[DpaUartIfControl.PacketCnt - 8] = TempData;
            }
            // counts number of send/receive byte
            DpaUartIfControl.PacketCnt++;
        } else {
            switch (DpaUartIfControl.PacketCnt) {
            case 0:
                // send LOW(NADR)
                dpaSendDataByte((DpaUartIfControl.DpaPacketPtr->NADR) & 0x00FF);
                break;

            case 1:
                // send HIGH(NADR)
                dpaSendDataByte((DpaUartIfControl.DpaPacketPtr->NADR) >> 8);
                break;

            case 2:
                // send PNUM
                dpaSendDataByte(DpaUartIfControl.DpaPacketPtr->PNUM);
                break;

            case 3:
                // send PCMD
                dpaSendDataByte(DpaUartIfControl.DpaPacketPtr->PCMD);
                break;

            case 4:
                // send LOW(HWPID)
                dpaSendDataByte((DpaUartIfControl.DpaPacketPtr->HWPID) & 0x00FF);
                break;

            case 5:
                // send HIGH(HWPID)
                dpaSendDataByte((DpaUartIfControl.DpaPacketPtr->HWPID) >> 8);
                break;

            default:
                // send additional data
                dpaSendDataByte(DpaUartIfControl.DpaPacketPtr->DpaMessage.Request.PData[DpaUartIfControl.PacketCnt - 6]);
            }
            // counts number of send/receive bytes
            DpaUartIfControl.PacketCnt++;
            // sent everything ?
            if (DpaUartIfControl.PacketCnt == DpaUartIfControl.PacketLen) {
                // send CRC
                dpaSendDataByte(DpaUartIfControl.CRC);
                // send stop of packet character
                dpaSendUartByte(HDLC_FRM_FLAG_SEQUENCE);
                // library is ready for next packet
                DpaControl.Status = DPA_READY;
                // stop data transfer
                DpaUartIfControl.Direction = UART_TRANSFER_NONE;
            }
        }
    } else { // no data to send / receive => check for new data
        // data in Rx buffer
        if (dpaReceiveUartByte(&TempData)) {
            // start of packet
            if (TempData == HDLC_FRM_FLAG_SEQUENCE) {
                // set pointer to DPA receive structure
                DpaUartIfControl.DpaPacketPtr = &DpaLibDpaAnswer;
                // initialize CRC
                DpaUartIfControl.CRC = 0xFF;
                // maximal size of received data
                DpaUartIfControl.PacketLen = sizeof(T_DPA_PACKET);
                // counter of received bytes
                DpaUartIfControl.PacketCnt = 0;
                // clear Escape flag
                DpaUartIfControl.WasEscape = 0;
                // reading from TR module
                DpaUartIfControl.Direction = UART_TRANSFER_READ;
                // library is busy
                DpaControl.Status = DPA_BUSY;
            }
            return;
        }

        // check if packet to send is ready
        if (DpaControl.DpaRequestPacketPtr != NULL) {
            // set pointer to DpaRequest packet
            DpaUartIfControl.DpaPacketPtr = DpaControl.DpaRequestPacketPtr;
            DpaControl.DpaRequestPacketPtr = NULL;
            // NADR + PNUM + PCMD + HWPID + Data
            DpaUartIfControl.PacketLen = DpaControl.ExtraDataSize + 6;
            // initialize CRC
            DpaUartIfControl.CRC = 0xFF;
            // counter of sent bytes
            DpaUartIfControl.PacketCnt = 0;
            // write data to TR module
            DpaUartIfControl.Direction = UART_TRANSFER_WRITE;
            // library is busy
            DpaControl.Status = DPA_BUSY;
            // send start of packet character
            dpaSendUartByte(HDLC_FRM_FLAG_SEQUENCE);
        }
    }
}

/**
 * Send data byte to TR module + make HDLC byte stuffing and compute CRC
 * @param DataByte Data byte for TR module
 */
void dpaSendDataByte(uint8_t DataByte)
{
    if (DataByte == HDLC_FRM_FLAG_SEQUENCE || DataByte == HDLC_FRM_CONTROL_ESCAPE) {
        dpaSendUartByte(HDLC_FRM_CONTROL_ESCAPE);
        dpaSendUartByte(DataByte ^ HDLC_FRM_ESCAPE_BIT);
    } else {
        dpaSendUartByte(DataByte);
    }

    DpaUartIfControl.CRC = dpaDoCRC8(DataByte, DpaUartIfControl.CRC);
}

/**
 * Compute the CRC8 value of a data set
 * @param InData One byte of data to compute CRC from
 * @param Seed The starting value of the CRC
 * @return The CRC8 of InData with Seed as initial value
 */
uint8_t dpaDoCRC8(uint8_t InData, uint8_t Seed)
{
  	for (uint8_t bitsLeft = 8; bitsLeft > 0; bitsLeft--) {
        if (((Seed ^ InData) & 0x01) == 0)
            Seed >>= 1;
        else
            Seed = (Seed >>= 1)^0x8C;
        InData >>= 1;
    }
    return (Seed);
}

#endif

#if defined (__STORE_CODE_SUPPORT__)

/**
 * Function for store HEX or IQRF code image to external EEPROM in TR module
 * @param CodeFileInfo Pointer to T_DPA_CODE_FILE_INFO structure with code file image information
 * @return Progress status or operation result (0 - 100 -> progress status, DPA_STORE_CODE_SUCCESS, DPA_STORE_CODE_ERROR)
 */
uint8_t dpaStoreCodeToEeeprom(T_DPA_CODE_FILE_INFO *CodeFileInfo)
{
    static uint16_t tempStoreEepromAddress;

    switch (DpaStoreCodeTaskSM) {
    case DPA_STORE_CODE_INIT_TASK:
        // initialize code file character read counter
        DpaControl.FileByteCounter = 0;

        // temporary store start address of code image in EEEPROM
        tempStoreEepromAddress = CodeFileInfo->ImageEeepromAdr;

        if (CodeFileInfo->FileType == DPA_CODE_FILE_IQRF)
            CodeFileInfo->ImageCRC = 0x03;  // initial value for .IQRF CRC
        else
            CodeFileInfo->ImageCRC = 0x01;  // initial value for .HEX CRC
        // initialize file image size
        CodeFileInfo->ImageSize = 0;
        // initialize work variables
        DpaStartAddressDetected = DpaCodeImageEndDetected = DpaOutputPacketDataCnt = DpaEeepromPageDataCnt = DpaRemainingDataCnt = 0;
        // start data sending
        DpaStoreCodeTaskSM = DPA_STORE_CODE_SEND_DATA;
        return(0);
        break;

    case DPA_STORE_CODE_SEND_DATA:
        if (CodeFileInfo->FileType == DPA_CODE_FILE_IQRF)
            dpaProcessIqrfCodeFile(CodeFileInfo);   // process .IQRF file
        else
            dpaProcessHexCodeFile(CodeFileInfo);    // process .HEX file
        break;

    case DPA_STORE_CODE_ERROR_END:
        // initialize task state machine
        DpaStoreCodeTaskSM = DPA_STORE_CODE_INIT_TASK;
        return(DPA_STORE_CODE_ERROR);
        break;

    case DPA_STORE_CODE_SUCCESS_END:
        // restore start address of code image in EEEPROM
        CodeFileInfo->ImageEeepromAdr = tempStoreEepromAddress;
        // initialize task state machine
        DpaStoreCodeTaskSM = DPA_STORE_CODE_INIT_TASK;
        return(DPA_STORE_CODE_SUCCESS);
        break;
    }

    return(((uint32_t) DpaControl.FileByteCounter * 100) / CodeFileInfo->FileSize);
}

/**
 * Compute the Flatcher-16 checksum
 * @param DataAddress
 * @param DataLen
 * @param Seed The starting value of the CRC
 * @return The FlatcherCRC16 of input with Seed as initial value
 */
uint16_t dpaFlatcherCRC16(uint8_t *DataAddress, uint8_t DataLen, uint16_t Seed)
{
    uint16_t TempL = Seed & 0xFF;
    uint16_t TempH = Seed >> 8;

    while (DataLen--) {
        TempL += *DataAddress;
        DataAddress++;
        if ((TempL & 0x100) != 0) {
            TempL++;
            TempL &= 0xFF;
        }

        TempH += TempL;
        if ((TempH & 0x100) != 0) {
            TempH++;
            TempH &= 0xFF;
        }
    }

    return((TempH << 8) | TempL);
}

/**
 * Send prepared data packed to eeeprom peripheral and compute FlatcherCRC16 from data
 * @param CodeFileInfo Pointer to T_DPA_CODE_FILE_INFO structure with code file image information
 * @param DataSize Number of data bytes in output buffer
 */
void dpaSendDataToEeeprom(T_DPA_CODE_FILE_INFO *CodeFileInfo, uint8_t DataSize)
{
    uint8_t OpResult;

    // make CRC from code image block of data
    CodeFileInfo->ImageCRC = dpaFlatcherCRC16((uint8_t *) & DpaStoreCodeRequest.DpaMessage.XMemoryRequest.ReadWrite.Write.PData[0], DataSize, CodeFileInfo->ImageCRC);
    // increase code image size
    CodeFileInfo->ImageSize += DataSize;
    // set destination address
    DpaStoreCodeRequest.NADR = CodeFileInfo->TrAddress;
    // set destination peripheral EEEPROM
    DpaStoreCodeRequest.PNUM = PNUM_EEEPROM;
    // set xwrite to eeeprom command
    DpaStoreCodeRequest.PCMD = CMD_EEEPROM_XWRITE;
    // do not check HWPID
    DpaStoreCodeRequest.HWPID = HWPID_DoNotCheck;
    // set destination address in eeeprom
    DpaStoreCodeRequest.DpaMessage.XMemoryRequest.Address = CodeFileInfo->ImageEeepromAdr;
    // next write on address
    CodeFileInfo->ImageEeepromAdr += DataSize;

    // send request and wait for result
    while ((OpResult = dpaSendRequest(&DpaStoreCodeRequest, DataSize + 2, 1000)) == DPA_OPERATION_IN_PROGRESS)
        ; /* void */

    switch (OpResult) {
    case DPA_OPERATION_TIMEOUT:
    case DPA_CONFIRMATION_ERR:
    case DPA_RESPONSE_ERR:
    case DPA_TR_MODULE_NOT_READY:
        DpaStoreCodeTaskSM = DPA_STORE_CODE_ERROR_END;
    }
}

/**
 * Process HEX file with custom DPA handler code
 * @param CodeFileInfo Pointer to T_DPA_CODE_FILE_INFO structure with code file image information
 */
void dpaProcessHexCodeFile(T_DPA_CODE_FILE_INFO *CodeFileInfo)
{
    uint8_t TempValue;

    // if end of code image not detected
    if (!DpaCodeImageEndDetected) {
        dpaSuspendDriver();
        // read one line from .HEX file
        if ((TempValue = dpaReadHEXFileLine()) != HEX_FILE_LINE_READY) {
            dpaRunDriver();
            // CRC error in hex file detected
            if (TempValue == HEX_FILE_CRC_ERR)
                DpaStoreCodeTaskSM = DPA_STORE_CODE_ERROR_END;
            else // end of file detected
                DpaCodeImageEndDetected = true;
            return;
        }
        dpaRunDriver();

        // line from hex file do not contain data, read next line
        if (DpaCodeLineBuffer[3] != 0)
            return;

        // compute destination code address
        DpaCodeLineAddress = (((uint16_t) DpaCodeLineBuffer[1] << 8) | DpaCodeLineBuffer[2]) / 2;

        // if code address == custom DPA handler start address
        if (DpaCodeLineAddress == CUSTOM_HANDLER_ADDRESS) {
            // set flag, start address detected
            DpaStartAddressDetected = true;
            // set next code address
            DpaCodeImageNextAddress = CUSTOM_HANDLER_ADDRESS;
        }

        // if start address of custom DPA handler was detected
        if (DpaStartAddressDetected) {
            // if destination code address is not correct
            if (DpaCodeImageNextAddress != DpaCodeLineAddress || DpaCodeLineAddress > CUSTOM_HANDLER_ADDRESS_END) {
                // set flag, code image end defected
                DpaCodeImageEndDetected = true;
                return;
            }

            // if hex file line buffer contains less then 0x10 data bytes
            if (DpaCodeLineBuffer[0] != 0x10) {
                // set remaining data counter
                DpaRemainingDataCnt = DpaCodeLineBuffer[0];
                // set flag, code image end defected
                DpaCodeImageEndDetected = true;
                return;
            }

            // copy data from hex file line buffer to output buffer
            memcpy((uint8_t *) & DpaStoreCodeRequest.DpaMessage.XMemoryRequest.ReadWrite.Write.PData[DpaOutputPacketDataCnt], (uint8_t *) & DpaCodeLineBuffer[4], DpaCodeLineBuffer[0]);

            // next code address should be
            DpaCodeImageNextAddress += 0x08;
            // increase data counter in output buffer
            DpaOutputPacketDataCnt += 0x10;

            // if output buffer contains 0x20 bytes
            if (DpaOutputPacketDataCnt == 0x20) {
                // clear output buffer data counter
                DpaOutputPacketDataCnt = 0;
                // increase eeeprom page data counter
                DpaEeepromPageDataCnt += 0x20;
                // if eeeprom data page contains 0x40 bytes, clear page data counter
                if (DpaEeepromPageDataCnt == 0x40)
                    DpaEeepromPageDataCnt = 0;
                // send 0x20 data bytes of code image to eeeprom peripheral of TR module
                dpaSendDataToEeeprom(CodeFileInfo, 0x20);
            }
        }
    } else { // if end of code image detected
        // if hex file line buffer contains any data
        if (DpaRemainingDataCnt) {
            // copy it to output buffer
            memcpy((uint8_t *) & DpaStoreCodeRequest.DpaMessage.XMemoryRequest.ReadWrite.Write.PData[DpaOutputPacketDataCnt], (uint8_t *) & DpaCodeLineBuffer[4], DpaRemainingDataCnt);
            // increase output buffer data counter
            DpaOutputPacketDataCnt += DpaRemainingDataCnt;
            // clear hex file line buffer data counter
            DpaRemainingDataCnt = 0;
        } else { // if hex file line buffer contains no data
            // if whole page to eeeprom was written
            if (DpaEeepromPageDataCnt == 0) {
                // operation end
                DpaStoreCodeTaskSM = DPA_STORE_CODE_SUCCESS_END;
                return;
            }
        }

        // if output buffer contains less then 0x20 data bytes
        if (DpaOutputPacketDataCnt < 0x20) {
            // fill rest of buffer with zeros
            memset((uint8_t *) & DpaStoreCodeRequest.DpaMessage.XMemoryRequest.ReadWrite.Write.PData[DpaOutputPacketDataCnt], 0, 0x20 - DpaOutputPacketDataCnt);
            // clear output buffer data counter
            DpaOutputPacketDataCnt = 0;
            // increase eeprom page data counter
            DpaEeepromPageDataCnt += 0x20;
            // if eeeprom data page contains 0x40 bytes, clear page data counter
            if (DpaEeepromPageDataCnt == 0x40)
                DpaEeepromPageDataCnt = 0;
            // send 0x20 data bytes of code image to eeeprom peripheral of TR module
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
    uint8_t TempValue;

    // if end of code image not detected
    if (!DpaCodeImageEndDetected) {
        // if iqrf file line buffer contains any data
        if (DpaRemainingDataCnt) {
            // number of bytes to full output buffer
            TempValue = 32 - DpaOutputPacketDataCnt;
            // if required bytes is more then ready bytes in line buffer
            if (TempValue > DpaRemainingDataCnt)
                TempValue = DpaRemainingDataCnt;
            // copy required bytes from line buffer to out buffer
            memcpy((uint8_t *) & DpaStoreCodeRequest.DpaMessage.XMemoryRequest.ReadWrite.Write.PData[DpaOutputPacketDataCnt], (uint8_t *) & DpaCodeLineBuffer[20 - DpaRemainingDataCnt], TempValue);
            // decrease remaining data counter
            DpaRemainingDataCnt -= TempValue;
            // increase output data counter
            DpaOutputPacketDataCnt += TempValue;

            // if output buffer contains 32 bytes
            if (DpaOutputPacketDataCnt == 32) {
                // clear output buffer data counter
                DpaOutputPacketDataCnt = 0;
                // send 32 data bytes of code image to eeeprom peripheral of TR module
                dpaSendDataToEeeprom(CodeFileInfo, 32);
            }
        } else {
            dpaSuspendDriver();
            // read one line from .IQRF file
            if ((TempValue = dpaReadIQRFFileLine()) != IQRF_FILE_LINE_READY) {
                dpaRunDriver();
                // wrong format if .IQRF file detected
                if (TempValue == IQRF_FILE_FORMAT_ERR)
                    DpaStoreCodeTaskSM = DPA_STORE_CODE_ERROR_END;      // end operation
                else
                    DpaCodeImageEndDetected = true;                     // end of file detected
                return;
            }
            dpaRunDriver();
            // number of ready bytes in file line buffer
            DpaRemainingDataCnt = 20;
        }
    } else { // if end of code image was detected
        // if any data still in output buffer
        if (DpaOutputPacketDataCnt) {
            // send rest of data bytes of code image to eeeprom peripheral of TR module
            dpaSendDataToEeeprom(CodeFileInfo, DpaOutputPacketDataCnt);
            // clear output buffer data counter
            DpaOutputPacketDataCnt = 0;
        } else {
            DpaStoreCodeTaskSM = DPA_STORE_CODE_SUCCESS_END;
            return;
        }
    }
}

/**
 * Read and process line from firmware file
 * @param none
 * @return Return code (HEX_FILE_LINE_READY, HEX_FILE_CRC_ERR, HEX_END_OF_FILE)
 */
uint8_t dpaReadHEXFileLine(void)
{
    uint8_t sign;
    uint8_t DataByteHi, DataByteLo;
    uint8_t DataByte;
    uint8_t CodeLineBufferPtr = 0;
    uint8_t CodeLineBufferCrc = 0;

    // find start of line or end of file
    while (((sign = dpaReadByteFromFile()) != 0) && (sign != ':'))
        ; /* void */

    // if end of file
    if (sign == 0)
        return(HEX_END_OF_FILE);

    // read data to end of line and convert if to numbers
    for ( ; ; ) {
        // read High nibble
        DataByteHi = tolower(dpaReadByteFromFile());
        // check end of line
        if (DataByteHi == 0x0A || DataByteHi == 0x0D) {
            if (CodeLineBufferCrc != 0)
                return(HEX_FILE_CRC_ERR);   // check line CRC
            return(HEX_FILE_LINE_READY);    // stop reading
        }
        // read Low nibble
        DataByteLo = tolower(dpaReadByteFromFile());
        // convert two ascii to number
        DataByte = dpaConvertToNum(DataByteHi, DataByteLo);
        // add to Crc
        CodeLineBufferCrc += DataByte;
        // store to line buffer
        DpaCodeLineBuffer[CodeLineBufferPtr++] = DataByte;
    }
}

/**
 * Read and process line from plugin file
 * @return Return code (IQRF_FILE_LINE_READY, IQRF_FILE_FORMAT_ERR, 2 - IQRF_END_OF_FILE)
 */
uint8_t dpaReadIQRFFileLine(void)
{
    uint8_t firstChar;
    uint8_t secondChar;
    uint8_t codeLineBufferPtr = 0;

repeat_read:
    // read one char from file
    firstChar = tolower(dpaReadByteFromFile());

    // read one char from file
    if (firstChar == '#') {
        // read data to end of line
        while (((firstChar = dpaReadByteFromFile()) != 0) && (firstChar != 0x0D))
            ; /* void */
    }

    // if end of line
    if (firstChar == 0x0D) {
        // read second code 0x0A
        dpaReadByteFromFile();
        if (codeLineBufferPtr == 0)
            goto repeat_read;               // read another line

        if (codeLineBufferPtr == 20)
            return(IQRF_FILE_LINE_READY);   // line with data readed successfully
        else
            return(IQRF_FILE_FORMAT_ERR);   // wrong file format (error)
    }

    // if end of file
    if (firstChar == 0)
        return(IQRF_END_OF_FILE);

    // read second character from code file
    secondChar = tolower(dpaReadByteFromFile());
    // convert chars to number and store to buffer
    DpaCodeLineBuffer[codeLineBufferPtr++] = dpaConvertToNum(firstChar, secondChar);
    // read next data
    goto repeat_read;
}

#endif
