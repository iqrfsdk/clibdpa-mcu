/**
 * Copyright 2015-2020 IQRF Tech s.r.o.
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

#include <stddef.h>
#include <stdint.h>
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <TimerOne.h>

#include "ccp.h"
#include "console.h"
#include "dpa_library.h"

#define TR_SS           8           // TR module chip select pin
#define PWR             9           // TR module power control pin
#define SD_SS           4           // SD card chip select pin

// MCU type of TR module
#define MCU_UNKNOWN           0
#define PIC16LF819            1     // TR-xxx-11A not supported
#define PIC16LF88             2     // TR-xxx-21A
#define PIC16F886             3     // TR-31B, TR-52B, TR-53B
#define PIC16LF1938           4     // TR-52D, TR-54D

// TR module types
#define TR_52D              0
#define TR_58D_RJ           1
#define TR_72D              2
#define TR_53D              3
#define TR_54D              8
#define TR_55D              9
#define TR_56D              10
#define TR_76D              11

/*
 * C prototypes
 */
#if defined(__SPI_INTERFACE__)
    extern "C" uint8_t dpaSendSpiByte(uint8_t Tx_Byte);
    extern "C" void dpaDeselectTRmodule(void);
#elif defined(__UART_INTERFACE__)
    extern "C" void dpaSendUartByte(uint8_t Tx_Byte);
    extern "C" uint8_t dpaReceiveUartByte(uint8_t *Rx_Byte);
#endif
#if defined (__STORE_CODE_SUPPORT__)
    extern "C" uint8_t dpaReadByteFromFile(void);
#endif


/*
 * C++ prototypes
 */
void myAsyncPacketHandler(T_DPA_PACKET *dpaAnswerPkt);
char bToHexa_high(uint8_t B);
char bToHexa_low(uint8_t B);
uint8_t sendMyDpaRequest(T_DPA_PACKET *DpaRequest, uint8_t DataSize, uint32_t Timeout);
void printDiscBondInfo(void);
void printOsInfo(uint8_t *infoBuffer);

/*
 * Variables
 */
uint8_t SDCardReady;
uint8_t AsyncPacketFlag;

T_DPA_PACKET MyDpaPacket;

#if defined (__STORE_CODE_SUPPORT__)
    File CodeFile;
    T_DPA_CODE_FILE_INFO  MyDpaCodeFileInfo;
#endif

/**
 * Setup peripherals
 */
void setup()
{
    Serial.begin(9600);

    SDCardReady = SD.begin(SD_SS);        // initialize SD card

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    pinMode(PWR, OUTPUT);                 // TR module power off (make TR module RESET)
    digitalWrite(PWR, HIGH);

#ifdef __SPI_INTERFACE__
    pinMode(TR_SS, OUTPUT);
    digitalWrite(TR_SS, LOW);
    delay(500);

    digitalWrite(PWR, LOW);               // TR module power on
    digitalWrite(TR_SS, HIGH);            // deselect TR module
    SPI.begin();                          // start SPI peripheral
    delay(500);                           // pause
#endif

#ifdef __UART_INTERFACE__
    delay(500);
    digitalWrite(PWR, LOW);               // TR module power on
    Serial1.begin(57600);                 // start UART1 peripheral
#endif

    dpaInit(myAsyncPacketHandler);        // initialize DPA library

    Timer1.initialize(200);                             // initialize timer1, call DPA driver every 200us
    Timer1.attachInterrupt(dpaLibraryDriver);           // attaches callback() as a timer overflow interrupt
}

/**
 * Main loop
 */
void loop()
{
    // console command processor
    ccp();

    // asynchronous packet indication
    if (AsyncPacketFlag == true) {
        AsyncPacketFlag = false;
        Serial.println();
        Serial.println("!!! Asynchronous DPA packet received !!!");
        Serial.println();
    }
}

#ifdef __SPI_INTERFACE__
/**
 * Send DPA byte over SPI
 *
 * @param Tx_Byte to send
 * @return Received Rx_Byte
 *
 */
uint8_t dpaSendSpiByte(uint8_t Tx_Byte)
{
    uint8_t Rx_Byte;

    if (!DpaControl.TRmoduleSelected) {
        SPI.beginTransaction(SPISettings(250000, MSBFIRST, SPI_MODE0));
        DpaControl.TRmoduleSelected = true;
        digitalWrite(TR_SS, LOW);
        delayMicroseconds(15);
    }

    Rx_Byte = SPI.transfer(Tx_Byte);

    return (Rx_Byte);
}

/**
 * DPA deselect module
 *
 * @param   none
 * @return   none
 *
 */
void dpaDeselectTRmodule(void)
{
    digitalWrite(TR_SS, HIGH);
    DpaControl.TRmoduleSelected = false;
    SPI.endTransaction();
}
#endif

#ifdef __UART_INTERFACE__
/**
 * Send DPA byte over SCI
 *
 * @param  Tx_Byte to send
 * @return none
 *
 */
void dpaSendUartByte(uint8_t Tx_Byte)
{
    Serial1.write(Tx_Byte);
}

/**
 * Read DPA byte over SCI
 *
 * @param  *Rx_Byte pointer to char to transfer received byte to DPA library
 * @return  false - no char to read; true - character ready
 *
 */
uint8_t dpaReceiveUartByte(uint8_t *Rx_Byte)
{
    if (Serial1.available() > 0) {
        *Rx_Byte = Serial1.read();
        return(true);
    }
    return(false);
}
#endif

#if defined (__STORE_CODE_SUPPORT__)
/**
 * Read byte from code file
 *
 * @param - none
 * @return - byte from firmware file or 0 = end of file
 *
 */
uint8_t dpaReadByteFromFile(void)
{
    if (CodeFile.available()){
        dpaIncFileByteCounter();
        return (CodeFile.read());
    } else {
        return(0);
    }
}
#endif

/**
 * asynchronous packet service routine
 *
 * @param - dpaAnswerPkt pointer to T_DPA_PACKET structure with data from TR module
 * @return - none
 *
 */
void myAsyncPacketHandler(T_DPA_PACKET *dpaAnswerPkt)
{
    /* here you can add any code for asynchronous packet service */

    AsyncPacketFlag = true;
}

/**
 * Converts the upper nibble of a binary value to a hexadecimal ASCII byte
 *
 * @param - B binary value to conversion
 * @return - ASCII char converted from upper nibble
 *
 */
char bToHexa_high(uint8_t B)
{
    B >>= 4;
    return (B>0x9) ? B+'A'-10:B+'0';
}

/**
 * Converts the lower nibble of a binary value to a hexadecimal ASCII byte.
 *
 * @param - B binary value to conversion
 * @return - ASCII char converted from lower nibble
 *
 */
char bToHexa_low(uint8_t B)
{
    B &= 0x0F;
    return (B>0x9) ? B+'A'-10:B+'0';
}

/**
 * Send DPA request and process DPA response
 * @param DpaRequest Pointer to DPA request
 * @param DataSize Size of data
 * @param Timeout Timeout
 * @return Operation result
 */
uint8_t sendMyDpaRequest(T_DPA_PACKET *DpaRequest, uint8_t DataSize, uint32_t Timeout)
{

    uint8_t Message;
    uint8_t OpResult;

    // "Sending request" message
    sysMsgPrinter(CCP_SENDING_REQUEST);

    // send request and wait for result
    while((OpResult = dpaSendRequest(DpaRequest, DataSize, Timeout)) == DPA_OPERATION_IN_PROGRESS)
        ; /* void */

    switch(OpResult){
    case DPA_OPERATION_OK:                              // operation OK
        Message = CCP_RESPONSE_OK;
        break;
    case DPA_OPERATION_TIMEOUT:                         // operation timeout
        Message = CCP_COMMAND_TIMEOUT;
        break;
    case DPA_CONFIRMATION_ERR:                          // confirmation error
        Message = CCP_CONFIRMATION_ERR;
        break;
    case DPA_RESPONSE_ERR:                              // response error
        Message = CCP_RESPONSE_ERR;
        break;
    case DPA_TR_MODULE_NOT_READY:                       // TR module not ready
        Message = CCP_TR_NOT_READY;
        break;
    }
    sysMsgPrinter(Message);

    return(OpResult);
}

const char DBInfoHead[] PROGMEM = {"    0 1 2 3 4 5 6 7 8 9 A B C D E F"};
/**
 * print result of discovery get or bond get commands
 */
void printDiscBondInfo(void)
{
    char MsgString[36];
    uint16_t Mask;
    uint16_t Data;
    uint8_t X, Y;

    Serial.print(CrLf);
    strcpy_P(MsgString, DBInfoHead);
    Serial.println(MsgString);

    for (X=0; X<16; X++) {
        Mask = 0x0001;
        Data = ((uint16_t)MyDpaPacket.DpaMessage.Response.PData[2*X+1]<<8) | MyDpaPacket.DpaMessage.Response.PData[2*X];
        Serial.print(X, HEX);
        Serial.print(" - ");
        for (Y=0; Y<16; Y++) {
            if (Data & Mask)
                Serial.print("x ");
            else
                Serial.print(". ");
            Mask <<= 1;
        }
        Serial.print(CrLf);
    }
}

const char OSModuleType[] PROGMEM = {"Module type    : "};
const char OSModuleId[] PROGMEM = {"Module ID      : "};
const char OSModuleIbk[] PROGMEM = {"Module IBK     : "};
const char OSModuleOsVer[] PROGMEM = {"OS version     : "};
const char OSModuleSupply[] PROGMEM = {"Supply voltage : "};
const char OSModuleRssi[] PROGMEM = {"Last RSSI      : "};
/**
 * Print results of OS info command
 * @param infoBuffer buffer with TR module and OS information data
 * @return none
 */
void printOsInfo(uint8_t *infoBuffer)
{
    uint8_t ModuleType = infoBuffer[5] >> 4;
    uint8_t McuType = infoBuffer[5] & 0x07;
    uint8_t OsVersionMajor = infoBuffer[4] / 16;
    uint8_t OsVersionMinor = infoBuffer[4] % 16;
    uint16_t OsBuild = ((uint16_t)infoBuffer[7] << 8) | infoBuffer[6];
    uint16_t SupplyVoltage;

    uint8_t Ptr, I;
    char TempString[24];

    // decode and print module type
    Serial.print(CrLf);
    strcpy_P(TempString, OSModuleType);
    Serial.print(TempString);
    Ptr=0;
    if(infoBuffer[3] & 0x80) {
        TempString[Ptr++] = 'D';
        TempString[Ptr++] = 'C';
    }
    TempString[Ptr++] = 'T';
    TempString[Ptr++] = 'R';
    TempString[Ptr++] = '-';

    TempString[Ptr++] = '5';

    switch(ModuleType) {
    case TR_52D:
        TempString[Ptr++] = '2';
        break;
    case TR_58D_RJ:
        TempString[Ptr++] = '8';
        break;
    case TR_72D:
        TempString[Ptr-1] = '7';
        TempString[Ptr++] = '2';
        break;
    case TR_53D:
        TempString[Ptr++] = '3';
        break;
    case TR_54D:
        TempString[Ptr++] = '4';
        break;
    case TR_55D:
        TempString[Ptr++] = '5';
        break;
    case TR_56D:
        TempString[Ptr++] = '6';
        break;
    case TR_76D:
        TempString[Ptr-1] = '7';
        TempString[Ptr++] = '6';
        break;
    default :
        TempString[Ptr++] = 'x';
        break;
    }

    if (McuType == PIC16LF1938)
        TempString[Ptr++] = 'D';
    TempString[Ptr++] = 'x';
    TempString[Ptr++] = 0;
    Serial.println(TempString);

    // print module ID
    strcpy_P(TempString, OSModuleId);
    Serial.print(TempString);
    for (I=3, Ptr=0; Ptr<8; I--) {
        TempString[Ptr++] = bToHexa_high(infoBuffer[I]);
        TempString[Ptr++] = bToHexa_low(infoBuffer[I]);
    }
    TempString[Ptr] = 0;
    Serial.println(TempString);

    // print module IBK
    strcpy_P(TempString, OSModuleIbk);
    Serial.print(TempString);
    // if OS version is 4.03 and more, print IBK
    if ((OsVersionMajor > 4) || ((OsVersionMajor == 4) && (OsVersionMinor >= 3))) {
        for (I=0; I<16; I++) {
            TempString[0] = bToHexa_high(infoBuffer[I+12]);
            TempString[1] = bToHexa_low(infoBuffer[I+12]);
            TempString[2] = 0x20;
            TempString[3] = 0x00;
            Serial.print(TempString);
            if (I == 15)
                Serial.println();
        }
    } else {
        Serial.println("---");
    }

    // print OS version string
    strcpy_P(TempString, OSModuleOsVer);
    Serial.print(TempString);
    Ptr = 0;

    // major version
    TempString[Ptr++] = bToHexa_low(OsVersionMajor);
    TempString[Ptr++] = '.';

    // minor version
    if (OsVersionMinor < 10) {
        TempString[Ptr++] = '0';
        TempString[Ptr++] = OsVersionMinor + '0';
    } else {
        TempString[Ptr++] = '1';
        TempString[Ptr++] = OsVersionMinor - 10 + '0';
    }

    if(McuType == PIC16LF1938)
        TempString[Ptr++] = 'D';

    // OS build
    TempString[Ptr++] = ' ';
    TempString[Ptr++] = '(';
    TempString[Ptr++] = bToHexa_high(OsBuild >> 8);
    TempString[Ptr++] = bToHexa_low(OsBuild >> 8);
    TempString[Ptr++] = bToHexa_high(OsBuild & 0x00FF);
    TempString[Ptr++] = bToHexa_low(OsBuild & 0x00FF);
    TempString[Ptr++] = ')';
    TempString[Ptr] = 0;
    Serial.println(TempString);

    // supply voltage
    if (ModuleType == TR_72D || ModuleType == TR_76D)
        SupplyVoltage = (26112 / (127 - infoBuffer[9]));
    else
        SupplyVoltage = (225 + infoBuffer[9] * 10);

    Ptr = SupplyVoltage / 100;
    I = SupplyVoltage % 100;

    // print supply voltage
    strcpy_P(TempString, OSModuleSupply);
    Serial.print(TempString);
    Serial.print(Ptr);
    Serial.print(".");
    Serial.print(I);
    Serial.println("V");

    // print last RSSI
    strcpy_P(TempString, OSModuleRssi);
    Serial.print(TempString);
    Serial.println(infoBuffer[8]);
}

const char CoordinatorNumBNodes[] PROGMEM = {"Number of bonded nodes = "};
const char CoordinatorNumDNodes[] PROGMEM = {"Number of discovered nodes = "};
const char CoordinatorNodesClear[] PROGMEM = {"All nodes cleared successfully"};
const char CoordinatorNodeAdr[] PROGMEM = {"Bonded node address = "};
const char CoordinatorNodeNum[] PROGMEM = {"Number of bonded nodes = "};
/**
 * coordinator DPA commands service
 * @param CommandTabParameter parameter from CCP command table
 * @return none
 */
void ccpCoordinatorCmd (uint16_t CommandTabParameter)
{
    uint8_t ReqAddr = 0;
    uint8_t TxPower = 7;
    uint8_t MaxAddr = 32;
    uint8_t ErrFlag = 0;
    uint8_t BondingTestRetries = 0;
    char TempString[32];

    // DPA header
    MyDpaPacket.NADR = COORDINATOR_ADDRESS;
    MyDpaPacket.PNUM = PNUM_COORDINATOR;
    MyDpaPacket.HWPID = HWPID_DoNotCheck;
    MyDpaPacket.PCMD = 0xFF;                                         // dummy command

    switch(CommandTabParameter) {

    case CMD_COORDINATOR_REMOVE_BOND:
        MyDpaPacket.PCMD = CommandTabParameter;                    // set peripheral cmd
        if (ccpFindCmdParameter(CcpCommandParameter))              // is any command parameter ?
            ReqAddr = atoi(CcpCommandParameter);                   // read command parameter
        else
            ReqAddr = 0;                                           // bad parameter

        if (ReqAddr == 0) {
            sysMsgPrinter(CCP_BAD_PARAMETER);                      // if bad parameter, print error message
        } else {
            MyDpaPacket.DpaMessage.PerCoordinatorRemoveBond_Request.BondAddr = ReqAddr;
            if (sendMyDpaRequest(&MyDpaPacket, sizeof(TPerCoordinatorRemoveBond_Request), 2000) == DPA_OPERATION_OK) {
                strcpy_P(TempString, CoordinatorNumBNodes);
                Serial.print(TempString);
                Serial.println(MyDpaPacket.DpaMessage.PerCoordinatorRemoveBond_Response.DevNr);
            }
        }
        break;

    case CMD_COORDINATOR_DISCOVERY:
        if (ccpFindCmdParameter(CcpCommandParameter)) {           // read first command parameter
            if (strcmp("get",CcpCommandParameter) == 0) {         // GET subcommand ?
                MyDpaPacket.PCMD = CMD_COORDINATOR_DISCOVERED_DEVICES;
                if (sendMyDpaRequest(&MyDpaPacket, 0, 5000) == DPA_OPERATION_OK)
                    printDiscBondInfo();                          // successfully received response
            } else {                                              // TxPower & MaxAddr parameters
                TxPower = atoi(CcpCommandParameter);              // set TxPower
                if (TxPower > 7)
                    ErrFlag = 1;                                  // check max TxPower
                if (ccpFindCmdParameter(CcpCommandParameter)) {   // read MaxAddr
                    MaxAddr = atoi(CcpCommandParameter);          // set MaxAddr
                    if (MaxAddr > 239)
                        ErrFlag = 1;                              // check MaxAddr
                }
            }
        }

        if (ErrFlag == 1) {
            sysMsgPrinter(CCP_BAD_PARAMETER);                     // if bad parameter, print error message
        } else {
            if (MyDpaPacket.PCMD == 0xFF) {                       // if command not defined
                MyDpaPacket.PCMD = CMD_COORDINATOR_DISCOVERY;     // discovery command
                MyDpaPacket.DpaMessage.PerCoordinatorDiscovery_Request.TxPower = TxPower;  // Command parameters
                MyDpaPacket.DpaMessage.PerCoordinatorDiscovery_Request.MaxAddr = MaxAddr;

                if (sendMyDpaRequest(&MyDpaPacket, sizeof(TPerCoordinatorDiscovery_Request), 30000) == DPA_OPERATION_OK) {
                    strcpy_P(TempString, CoordinatorNumDNodes);
                    Serial.print(TempString);
                    Serial.println(MyDpaPacket.DpaMessage.PerCoordinatorDiscovery_Response.DiscNr);
                }
            }
        }
        break;

    default:
        if (ccpFindCmdParameter(CcpCommandParameter)) {           // read first command parameter
            if (strcmp("clear",CcpCommandParameter) == 0) {
                MyDpaPacket.PCMD = CMD_COORDINATOR_CLEAR_ALL_BONDS;
                if (sendMyDpaRequest(&MyDpaPacket, 0, 2000) == DPA_OPERATION_OK) {
                    strcpy_P(TempString, CoordinatorNodesClear);
                    Serial.println(TempString);
                }
            } else {
                if (strcmp("get",CcpCommandParameter) == 0) {
                    MyDpaPacket.PCMD = CMD_COORDINATOR_BONDED_DEVICES;
                    if (sendMyDpaRequest(&MyDpaPacket, 0, 5000) == DPA_OPERATION_OK)
                        printDiscBondInfo();                        // successfully received response
                } else {
                    ReqAddr = atoi(CcpCommandParameter);
                    if (ccpFindCmdParameter(CcpCommandParameter))   // read bonding mask
                        BondingTestRetries = atoi(CcpCommandParameter);
                }
            }
        }

        if (MyDpaPacket.PCMD == 0xFF) {                             // if command not defined
            MyDpaPacket.PCMD = CMD_COORDINATOR_BOND_NODE;
            MyDpaPacket.DpaMessage.PerCoordinatorBondNode_Request.ReqAddr = ReqAddr;   // Command parameters
            MyDpaPacket.DpaMessage.PerCoordinatorBondNode_Request.BondingTestRetries = BondingTestRetries;
            if (sendMyDpaRequest(&MyDpaPacket, sizeof(TPerCoordinatorBondNode_Request), 5000) == DPA_OPERATION_OK) {
                strcpy_P(TempString, CoordinatorNodeAdr);
                Serial.print(TempString);
                Serial.println(MyDpaPacket.DpaMessage.PerCoordinatorBondNodeSmartConnect_Response.BondAddr);
                strcpy_P(TempString, CoordinatorNodeNum);
                Serial.print(TempString);
                Serial.println(MyDpaPacket.DpaMessage.PerCoordinatorBondNodeSmartConnect_Response.DevNr);
            }
        }
    }
}

/**
 * leds dpa commands service
 * @param CommandTabParameter parameter from CCP command table
 * @return none
 */
void ccpLedCmd (uint16_t CommandTabParameter)
{
    uint8_t Message = 0;

    // processing of command input parameters
    // read required operation
    if (ccpFindCmdParameter(CcpCommandParameter)) {
        if (strcmp("on",CcpCommandParameter) == 0)
            MyDpaPacket.PCMD = CMD_LED_SET_ON;
        else if (strcmp("off",CcpCommandParameter) == 0)
            MyDpaPacket.PCMD = CMD_LED_SET_OFF;
        else if (strcmp("flash",CcpCommandParameter) == 0)
            MyDpaPacket.PCMD = CMD_LED_FLASHING;
        else if (strcmp("pulse",CcpCommandParameter) == 0)
            MyDpaPacket.PCMD = CMD_LED_PULSE;
        else Message = CCP_BAD_PARAMETER;
    } else {
        Message = CCP_BAD_PARAMETER;
    }

    // read destination address
    if (ccpFindCmdParameter(CcpCommandParameter))
        MyDpaPacket.NADR = atoi(CcpCommandParameter);     // set destination address
    else
        Message = CCP_BAD_PARAMETER;                      // if address not exist, print error MSG

    // sending DPA request and processing response
    if (Message) {
        sysMsgPrinter(Message);                           // command parameters ERROR
    } else {
        MyDpaPacket.PNUM = CommandTabParameter;           // select RED or GREEN led
        MyDpaPacket.HWPID = HWPID_DoNotCheck;             // do not check HWPID
        sendMyDpaRequest(&MyDpaPacket, 0, 2000);          // send request
    }
}

/**
 * node DPA commands service
 * @param CommandTabParameter parameter from CCP command table
 * @return none
 */
void ccpNodeCmd (uint16_t CommandTabParameter)
{
    // Send Batch command
    MyDpaPacket.NADR = 0;
    MyDpaPacket.PNUM = PNUM_OS;
    MyDpaPacket.PCMD = CMD_OS_BATCH;
    MyDpaPacket.HWPID = HWPID_DoNotCheck;

    // Remove bond
    MyDpaPacket.DpaMessage.Request.PData[0] = 5;
    MyDpaPacket.DpaMessage.Request.PData[1] = PNUM_NODE;
    MyDpaPacket.DpaMessage.Request.PData[2] = CMD_NODE_REMOVE_BOND;
    MyDpaPacket.DpaMessage.Request.PData[3] = HWPID_DoNotCheck & 0xff;
    MyDpaPacket.DpaMessage.Request.PData[4] = HWPID_DoNotCheck >> 0x08;
    // Restart
    MyDpaPacket.DpaMessage.Request.PData[5] = 5;
    MyDpaPacket.DpaMessage.Request.PData[6] = PNUM_OS;
    MyDpaPacket.DpaMessage.Request.PData[7] = CMD_OS_RESTART;
    MyDpaPacket.DpaMessage.Request.PData[8] = HWPID_DoNotCheck & 0xff;
    MyDpaPacket.DpaMessage.Request.PData[9] = HWPID_DoNotCheck >> 0x08;
    // EndBatch
    MyDpaPacket.DpaMessage.Request.PData[10] = 0;

    if (ccpFindCmdParameter(CcpCommandParameter))            // read destination address
        MyDpaPacket.NADR = atoi(CcpCommandParameter);        // set destination address

    if (MyDpaPacket.NADR != 0)                               // parameter OK
        sendMyDpaRequest(&MyDpaPacket, 11, 50000);           // send request and process response
    else
        sysMsgPrinter(CCP_BAD_PARAMETER);                    // bad parameter
}

/**
 * OS DPA commands service
 * @param CommandTabParameter parameter from CCP command table
 * @return none
 */
void ccpOsCmd (uint16_t CommandTabParameter)
{
    MyDpaPacket.NADR = 0;
    MyDpaPacket.PNUM = PNUM_OS;                            // peripheral OS
    MyDpaPacket.PCMD = CommandTabParameter;                // set peripheral command
    MyDpaPacket.HWPID = HWPID_DoNotCheck;                  // do not check HWPID

    if (ccpFindCmdParameter(CcpCommandParameter))          // read destination address
        MyDpaPacket.NADR = atoi(CcpCommandParameter);      // set destination address

    if (sendMyDpaRequest(&MyDpaPacket, 0, 2000) == DPA_OPERATION_OK) {
        if (MyDpaPacket.PCMD == (RESPONSE_FLAG | CMD_OS_READ))                           // response to command CMD_OS_READ
            printOsInfo((uint8_t *)&MyDpaPacket.DpaMessage.PerOSRead_Response.MID[0]);   // decode received data
    }
}

/**
 * DPA load configuration command service
 * @param CommandTabParameter parameter from CCP command table
 * @return none
 */
void ccpLoadConfigCmd (uint16_t CommandTabParameter)
{
    uint8_t Message = 0;
    uint8_t CfgCrc;
    char Filename[16];
    File CfgFile;

    // if SD card is not ready print error msg
    if (!SDCardReady) {
        Message = CCP_SD_CARD_ERR;
    } else {
        // processing of command input parameters
        if (ccpFindCmdParameter(CcpCommandParameter)) {                  // read configuration file filename
            if (strlen(CcpCommandParameter) > 12)
                Message = CCP_BAD_PARAMETER;                             // check size of filename
            else
                strcpy(Filename, CcpCommandParameter);                   // copy
        } else {
            Message = CCP_BAD_PARAMETER;                                 // if filename not exist, print error msg
        }

        if (ccpFindCmdParameter(CcpCommandParameter))                    // read destination address
            MyDpaPacket.NADR = atoi(CcpCommandParameter);                // set destination address
        else
            Message = CCP_BAD_PARAMETER;                                 // if address not exist, print error msg
    }

    // sending DPA request and processing response
    if (Message) {
        sysMsgPrinter(Message);                                         // command parameters ERROR
    } else {
        dpaSuspendDriver();                                             // suspend SPI communication with TR module
        CfgFile = SD.open(Filename);                                    // open configuration file
        if (CfgFile) {                                                  // if configuration file exist
            if (CfgFile.size() != 33) {
                Message = CCP_CONFIG_FILE_ERR;                          // check size of configuration file
            } else {                                                    // if size is OK
                CfgFile.read((uint8_t *)&MyDpaPacket.DpaMessage.PerOSWriteCfg_Request.Undefined, 33);   // read configuration data from file
                CfgCrc = dpaMakeConfigurationCRC(&MyDpaPacket);                                         // make CRC from configuration data
                if (CfgCrc != MyDpaPacket.DpaMessage.PerOSWriteCfg_Request.Undefined)
                    Message = CCP_CONFIG_FILE_ERR;                      // configuration file wrong CRC
            }
            CfgFile.close();                                            // close file
        } else {
            Message = CCP_FILE_NOT_FOUND;                               // if configuration file not exist, print err MSG
        }

        dpaRunDriver();                                                 // start SPI communication with TR module

        if (Message) {
            sysMsgPrinter(Message);                                     // print error message, if exist
        } else {
            MyDpaPacket.PNUM = PNUM_OS;                                 // peripheral OS
            MyDpaPacket.PCMD = CMD_OS_WRITE_CFG;                        // write configuration command
            MyDpaPacket.HWPID = HWPID_DoNotCheck;                       // do not check HWPID

            sendMyDpaRequest(&MyDpaPacket, sizeof(TPerOSWriteCfg_Request), 3000);   // send request with configuration data and process response
        }
    }
}

#if defined (__STORE_CODE_SUPPORT__)

const char ProgressBar[] PROGMEM = {'-','-','-','-','-','-','-','-','-','-',0x0D,0x00};
const char StoreCodeImgAdr[] PROGMEM = {"Code image address : 0x"};
const char StoreCodeImgSize[] PROGMEM = {"Code image size : 0x"};
const char StoreCodeImgCrc[] PROGMEM = {"Code image CRC : 0x"};
/**
 * dpa store code command service
 * @param CommandTabParameter parameter from CCP command table
 * @return none
 */
void ccpDpaStoreCodeCmd (uint16_t CommandTabParameter)
{
    uint8_t Message = 0;
    uint8_t TempVariable;
    uint8_t StoreProgress;
    char Filename[26];

    if (!SDCardReady) {
        Message = CCP_SD_CARD_ERR;                                      // if SD card is not ready print error MSG
    } else {
        if (ccpFindCmdParameter(CcpCommandParameter)) {                 // read code file filename
            if (strlen(CcpCommandParameter) > 12)
                Message = CCP_BAD_PARAMETER;                            // check size of filename
            else
                strcpy(Filename, CcpCommandParameter);                  // copy filename
        } else {
            Message = CCP_BAD_PARAMETER;                                // if filename not exist, print error MSG
        }

        TempVariable = strlen(Filename);                                // filename size
        if (TempVariable >= 5)
            TempVariable -= 3;                                          // check filename extension
        else
            TempVariable = 0;

        if (strcmp(&Filename[TempVariable],"hex") == 0)
            MyDpaCodeFileInfo.FileType = DPA_CODE_FILE_HEX;             // extension is .hex -> HEX file type
        else
            MyDpaCodeFileInfo.FileType = DPA_CODE_FILE_IQRF;            // IQRF file type

        MyDpaCodeFileInfo.ImageEeepromAdr = 0;                          // absolute eeeprom address of code image in TR module
        if (ccpFindCmdParameter(CcpCommandParameter))                   // read eeeprom address
            MyDpaCodeFileInfo.ImageEeepromAdr = strtol(CcpCommandParameter,NULL,0);

        MyDpaCodeFileInfo.TrAddress = 0;
        if (ccpFindCmdParameter(CcpCommandParameter))                   // read destination tr module address
            MyDpaCodeFileInfo.TrAddress = atoi(CcpCommandParameter);

        // if eeeprom address of TR module address is out of range, print error MSG
        if (MyDpaCodeFileInfo.ImageEeepromAdr < 0x0700 || MyDpaCodeFileInfo.ImageEeepromAdr > 0x3FFF ||
            MyDpaCodeFileInfo.ImageEeepromAdr % 64 || MyDpaCodeFileInfo.TrAddress > 0xFF)
        {
            Message = CCP_BAD_PARAMETER;
        }

        if (Message) {
            sysMsgPrinter(Message);                                     // command parameters ERROR
        } else {
            dpaSuspendDriver();                                         // suspend SPI communication with TR module
            CodeFile = SD.open(Filename);                               // open code file
            dpaRunDriver();                                             // start SPI communication with TR module

            if (CodeFile) {                                             // if code file exist
                MyDpaCodeFileInfo.FileSize = CodeFile.size();           // read size of file
                sysMsgPrinter(CCP_UPLOADING);                           // message "Uploading..."
                strcpy_P(Filename, ProgressBar);
                Serial.print(Filename);                                 // print progress bar
                StoreProgress = 0;

                while ((TempVariable = dpaStoreCodeToEeeprom(&MyDpaCodeFileInfo)) <= 100) {    // call dpaStoreCodeToEeeprom function, until code file is stored
                    while (StoreProgress < TempVariable / 10) {         // during code file storing, print progress bar
                        Serial.write('*');
                        StoreProgress++;
                    }
                }

                if (TempVariable == DPA_STORE_CODE_SUCCESS) {
                     while (StoreProgress < 10) {                       // if progress bar is not full, fill it to full
                        Serial.write('*');
                        StoreProgress++;
                    }
                    Serial.println();                                   // new line
                    sysMsgPrinter(CCP_CODE_STORED);                     // print results of code store operation
                    strcpy_P(Filename, StoreCodeImgAdr);
                    Serial.print(Filename);
                    Serial.println(MyDpaCodeFileInfo.ImageEeepromAdr, HEX);
                    strcpy_P(Filename, StoreCodeImgSize);
                    Serial.print(Filename);
                    Serial.println(MyDpaCodeFileInfo.ImageSize, HEX);
                    strcpy_P(Filename, StoreCodeImgCrc);
                    Serial.print(Filename);
                    Serial.println(MyDpaCodeFileInfo.ImageCRC, HEX);
                } else {                                               // code store operation error
                    Serial.println();
                    sysMsgPrinter(CCP_CODE_STORE_ERR);                 // print error MSG
                }

                CodeFile.close();                                      // close file
            } else {
                sysMsgPrinter(CCP_FILE_NOT_FOUND);                     // if code file not exist, print error MSG
            }
        }
    }
}

const char LoadCodeImgOk[] PROGMEM = {"Code image load OK"};
const char LoadCodeImgErr[] PROGMEM = {"Code image load ERROR"};
const char VerifyCodeImgOk[] PROGMEM = {"Code image verification OK"};
const char VerifyCodeImgErr[] PROGMEM = {"Code image verification ERROR"};
/**
 * DPA load / verify code command service
 * @param CommandTabParameter parameter from CCP command table
 * @return none
 */
void ccpDpaLoadVerifyCodeCmd (uint16_t CommandTabParameter)
{
    uint8_t Message = 0;
    char MsgString[32];

    if (ccpFindCmdParameter(CcpCommandParameter)) {                   // set type of code image (HEX / IQRF)
        if (strcmp(CcpCommandParameter,"iqrf") == 0) {
            MyDpaCodeFileInfo.FileType = DPA_CODE_FILE_IQRF;
        } else {
            if (strcmp(CcpCommandParameter,"hex") == 0)
                MyDpaCodeFileInfo.FileType = DPA_CODE_FILE_HEX;
            else
                Message = CCP_BAD_PARAMETER;                          // wrong type of code image
        }
    } else {
        Message = CCP_BAD_PARAMETER;                                 // missing parameter
    }

    if (ccpFindCmdParameter(CcpCommandParameter))
        MyDpaCodeFileInfo.ImageEeepromAdr = strtol(CcpCommandParameter,NULL,0);   // set eeeprom address of code image
    else
        Message = CCP_BAD_PARAMETER;                                 // missing parameter

    if (ccpFindCmdParameter(CcpCommandParameter))
        MyDpaCodeFileInfo.ImageSize = strtol(CcpCommandParameter,NULL,0);         // set size of code image
    else
        Message = CCP_BAD_PARAMETER;                                 // missing parameter

    if (ccpFindCmdParameter(CcpCommandParameter))
        MyDpaCodeFileInfo.ImageCRC = strtol(CcpCommandParameter,NULL,0);          // set CRC of code image
    else
        Message = CCP_BAD_PARAMETER;                                 // missing parameter

    if (ccpFindCmdParameter(CcpCommandParameter))
        MyDpaCodeFileInfo.TrAddress = atoi(CcpCommandParameter);                  // set destination TR module address
    else
        Message = CCP_BAD_PARAMETER;                                 // missing parameter

    if (Message) {
        sysMsgPrinter(Message);                                      // if command parameters ERROR
    } else {
        MyDpaPacket.NADR = MyDpaCodeFileInfo.TrAddress;
        MyDpaPacket.PNUM = PNUM_OS;                                  // peripheral OS
        MyDpaPacket.PCMD = CMD_OS_LOAD_CODE;                         // load code peripheral command
        MyDpaPacket.HWPID = HWPID_DoNotCheck;                        // do not check HWPID

        if (MyDpaCodeFileInfo.FileType == DPA_CODE_FILE_IQRF)
            CommandTabParameter |= 0x02;                             // set IQRF plug-in type of code image

        MyDpaPacket.DpaMessage.PerOSLoadCode_Request.Flags = CommandTabParameter;
        MyDpaPacket.DpaMessage.PerOSLoadCode_Request.Address = MyDpaCodeFileInfo.ImageEeepromAdr;
        MyDpaPacket.DpaMessage.PerOSLoadCode_Request.Length = MyDpaCodeFileInfo.ImageSize;
        MyDpaPacket.DpaMessage.PerOSLoadCode_Request.CheckSum = MyDpaCodeFileInfo.ImageCRC;

        DpaAditionalTimeout = MyDpaCodeFileInfo.ImageSize;           // every byte is 1ms of additional timeout

        if (sendMyDpaRequest(&MyDpaPacket, sizeof(TPerOSLoadCode_Request), DpaAditionalTimeout) == DPA_OPERATION_OK) {
            if (CommandTabParameter & 0x01) {                        // result of load code operation
                if (MyDpaPacket.DpaMessage.Response.PData[0] == 1)
                    strcpy_P(MsgString, LoadCodeImgOk);
                else
                    strcpy_P(MsgString, LoadCodeImgErr);
                Serial.println(MsgString);
            }
            else{                                                     // result of verify code operation
                if (MyDpaPacket.DpaMessage.Response.PData[0] == 1)
                    strcpy_P(MsgString, VerifyCodeImgOk);
                else
                    strcpy_P(MsgString, VerifyCodeImgErr);
                Serial.println(MsgString);
            }
        }
        DpaAditionalTimeout = 0;
    }
}

const char CustomHandlerOn[] PROGMEM = {"Custom DPA handler is ON"};
const char CustomHandlerOff[] PROGMEM = {"Custom DPA handler is OFF"};
/**
 * DPA custom handler on/off command service
 * @param CommandTabParameter parameter from CCP command table
 * @return none
 */
void ccpDpaCustomHandlerOnOffCmd (uint16_t CommandTabParameter)
{
    uint8_t Message = 0;
    uint8_t CDpaHandlerSet;
    char MsgString[32];

    if (ccpFindCmdParameter(CcpCommandParameter)) {                       // set ON / OFF for custom DPA handler
        if (strcmp(CcpCommandParameter,"on") == 0) {
            CDpaHandlerSet = 1;                                           // set custom DPA handler ON
        } else {
            if (strcmp(CcpCommandParameter,"off") == 0)
                CDpaHandlerSet = 0;                                       // set custom DPA handler OFF
            else
                Message = CCP_BAD_PARAMETER;                              // wrong command parameter
        }
    } else {
        Message = CCP_BAD_PARAMETER;                                     // missing command parameter
    }

    if (ccpFindCmdParameter(CcpCommandParameter))
        MyDpaPacket.NADR = atoi(CcpCommandParameter);                    // set destination TR module address
    else
        Message = CCP_BAD_PARAMETER;                                     // missing TR module address

    if (Message) {
        sysMsgPrinter(Message);                                          // if command parameters ERROR
    } else {
        MyDpaPacket.PNUM = PNUM_OS;                                      // peripheral OS
        MyDpaPacket.PCMD = CMD_OS_WRITE_CFG_BYTE;                        // write configuration byte peripheral command
        MyDpaPacket.HWPID = HWPID_DoNotCheck;                            // do not check HWPID

        MyDpaPacket.DpaMessage.PerOSWriteCfgByte_Request.Triplets[0].Address = 5;
        MyDpaPacket.DpaMessage.PerOSWriteCfgByte_Request.Triplets[0].Value = CDpaHandlerSet;
        MyDpaPacket.DpaMessage.PerOSWriteCfgByte_Request.Triplets[0].Mask = 0x01;

        if (sendMyDpaRequest(&MyDpaPacket, 3, 2000) == DPA_OPERATION_OK) {
            if (CDpaHandlerSet == 1)
                strcpy_P(MsgString, CustomHandlerOn);                     // print status of custom DPA handler
            else
                strcpy_P(MsgString, CustomHandlerOff);
            Serial.println(MsgString);
        }
    }
}

#endif
