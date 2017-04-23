
/**
 * Copyright 2015-2017 MICRORISC s.r.o.
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
#include <TimerOne.h>

#include "autonetwork_core.h"
#include "autonetwork.h"
#include "dpa_library.h"
#include "ccp.h"

#define TR_SS           8           // TR module chip select pin
#define PWR             9           // TR module power control pin
#define SD_SS           4           // SD card chip select pin

#define SERIAL_BUFFER_SIZE  64

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

/*
 * C++ prototypes
 */
void myAsyncPacketHandler(T_DPA_PACKET *dpaAnswerPkt);

/*
 * Variables
 */
char SerialBufferOut[SERIAL_BUFFER_SIZE];
T_AN_PARAMS MyAutonetworkParams;

const char EventStrings[28][80] PROGMEM = {
    // Event EVT_AN_PROCESS_STARTED
    "Automatic network construction started\r\n\0",
    // Event EVT_AN_PROCESS_STOPPED
    "Automatic network construction stopped\r\n\0",
    // Event EVT_GET_NETWORK_INFO
    "Getting initial network info\r\n\0",
    // Event EVT_ROUND_START
    "\r\nRound=%u, Nodes=%u, New nodes=%u\r\n\0",
    // Event EVT_COOR_ENABLE_PREBONDING
    "Enable prebonding at coordinator, mask=%u, time=%u\r\n\0",
    // Event EVT_COOR_DISABLE_PREBONDING
    "Disable prebonding at coordinator\r\n\0",
    // Event EVT_COOR_READ_MID
    "Coordinator prebonded MID=%08lX, UserData=%08lX\r\n\0",
    // Event EVT_COOR_REMOVING_BOND
    "Removing node %u\r\n\0",
    // Event EVT_NODE_ENABLE_PREBONDING
    "Enable prebonding at nodes, mask=%u, time=%u, LEDR=1\r\n\0",
    // Event EVT_NODE_DISABLE_PREBONDING
    "Disable prebonding at nodes\r\n\0",
    // Event EVT_NODE_READ_MID
    "Node %u prebonded MID=%08lX, UserData=%08lX\r\n\0",
    // Event EVT_AUTHORIZE_BOND
    "Authorizing node MID=%08lX, address %u\r\n\0",
    // Event EVT_AUTHORIZE_BOND_OK
    "OK, nodes count=%u\r\n\0",
    // Event EVT_DISCOVERY
    "Running discovery\r\n\0",
    // Event EVT_DISCOVERY_WAIT
    "Waiting for coordinator to finish discovery\r\n\0",
    // Event EVT_DISCOVERY_OK
    "Discovered nodes=%u\r\n\0",
    // Event EVT_WAIT_PREBONDING
    "Waiting for prebonding for %u seconds\r\n\0",
    // Event EVT_FRC_DISABLE_PREBONDING
    "Running FRC to disable and check for prebonding\r\n\0",
    // Event EVT_FRC_DISABLE_PREBONDING_BIT0_ERR
    "Error @ FRC bit.0 is set, but node %u not bonded\r\n\0",
    // Event EVT_FRC_DISABLE_PREBONDING_BIT1_ERR
    "Error @ FRC bit.1, set, but node %u is already bonded\r\n\0",
    // Event EVT_FRC_CHECK_NEW_NODES
    "Running FRC to check new nodes and removing 0xFE nodes\r\n\0",
    // Event EVT_NO_FREE_ADDRESS
    "No free address\r\n\0",
    // Event EVT_NO_NEW_NODE_PREBONDED
    "No new node prebonded\r\n\0",
    // Event EVT_DPA_CONFIRMATION_TIMEOUT
    "DPA confirmation timeout\r\n\0",
    // Event EVT_DPA_RESPONSE_TIMEOUT
    "DPA response timeout\r\n\0",
    // Event EVT_REMOVE_ALL_BONDS
    "Remove all bonds at nodes and coordinator, restart nodes\r\n\0",
    // Event EVT_MAX_NODES_PREBONDED
    "Maximum prebonded nodes reached\r\n\0",
    // Event EVT_NODE_REMOTE_UNBOND
    "Remote unbonding node %u\r\n\0"
};

/**
 * Setup peripherals
 */
void setup()
{
    Serial.begin(9600);

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
    autonetworkInit(autonetworkHandler);  // initialize autonetwork

    Timer1.initialize(150);                             // initialize timer1, call dpa driver every 150us
    Timer1.attachInterrupt(dpaLibraryDriver);           // attaches callback() as a timer overflow interrupt
}

/**
 * Main loop
 */
void loop()
{
    ccp();
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

    if (!DpaControl.TRmoduleSelected){
        SPI.beginTransaction(SPISettings(250000, MSBFIRST, SPI_MODE0));
        DpaControl.TRmoduleSelected = true;
        digitalWrite(TR_SS, LOW);
        delayMicroseconds(15);
    }

    Rx_Byte = SPI.transfer(Tx_Byte);

    return Rx_Byte;
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
 * @param  *Rx_Byte pointer to char to transfer received byte to dpa library
 * @return  false - no char to read; true - character ready
 *
 */
uint8_t dpaReceiveUartByte(uint8_t *Rx_Byte)
{
    if (Serial1.available() > 0){
        *Rx_Byte = Serial1.read();
        return(true);
    }
    return(false);
}
#endif

/**
 * asynchronous packet service rutine
 *
 * @param - dpaAnswerPkt pointer to T_DPA_PACKET structure with data from TR module
 * @return - none
 *
 */
void myAsyncPacketHandler(T_DPA_PACKET *dpaAnswerPkt)
{
    /* here you can add any code for asynchronous packet service */
    Serial.println("!!! Asynchronous DPA packet received !!!");
}

/**
 * autonetwork message handler
 *
 * @param - EventCode number of desired message
 * @param - State pointer to T_AN_STATE structure with autonetwork information
 * @return - none
 *
 */
void autonetworkHandler(uint8_t EventCode, T_AN_STATE *State)
{
    // Cpoy event description string to serial_buffer_out
    strcpy_P(SerialBufferOut, &EventStrings[EventCode][0]);

    // Event with parameters ?
    if (State != NULL){
        // Yes, event with parameters
        char Buffer[SERIAL_BUFFER_SIZE];
        switch (EventCode){

            case EVT_ROUND_START:
                sprintf(Buffer, SerialBufferOut, State->PrebondingInfo->Round, State->NewtorkInfo->BondedNodesCount, State->NewtorkInfo->BondedNodesCount - State->PrebondingInfo->OrigNodesCount);
                break;

            case EVT_COOR_ENABLE_PREBONDING:
                sprintf(Buffer, SerialBufferOut, State->PrebondingInfo->BondingMask, State->Params->TemporaryAddressTimeout);
                break;

            case EVT_NODE_ENABLE_PREBONDING:
                sprintf(Buffer, SerialBufferOut, State->PrebondingInfo->BondingMask, State->Params->TemporaryAddressTimeout);
                break;

            case EVT_WAIT_PREBONDING:
                sprintf(Buffer, SerialBufferOut, State->PrebondingInfo->WaitBonding/*Delay*/);
                break;

            case EVT_COOR_READ_MID:
                sprintf(Buffer, SerialBufferOut, State->PrebondingInfo->MID, State->PrebondingInfo->UserData);
                break;

            case EVT_NODE_READ_MID:
                sprintf(Buffer, SerialBufferOut, State->PrebondingInfo->Param, State->PrebondingInfo->MID, State->PrebondingInfo->UserData);
                break;

            case EVT_AUTHORIZE_BOND:
                sprintf(Buffer, SerialBufferOut, State->PrebondingInfo->MID, State->PrebondingInfo->NextAddr);
                break;

            case EVT_AUTHORIZE_BOND_OK:
            case EVT_FRC_DISABLE_PREBONDING_BIT0_ERR:
            case EVT_FRC_DISABLE_PREBONDING_BIT1_ERR:
            case EVT_DISCOVERY_OK:
            case EVT_COOR_REMOVING_BOND:
            case EVT_NODE_REMOTE_UNBOND:
                sprintf(Buffer, SerialBufferOut, State->PrebondingInfo->Param);
                break;

            // Unknown message
            default:
                strcpy(Buffer, "Unknown event\r\n");
                break;
        }
        Serial.print(Buffer);
    }
    else
    {
        // No parameters, send text
        Serial.print(SerialBufferOut);
    }
}

/**
 * autontw commands service
 *
 * @param - CommandTabParameter command parameter from decode command table
 * @return - none
 *
 */
void ccpAutonetworkCmd (uint16_t CommandTabParameter)
{
    // set default autonetwork parametters
    MyAutonetworkParams.TemporaryAddressTimeout = 60;
    MyAutonetworkParams.AuthorizeRetries = 1;
    MyAutonetworkParams.DiscoveryRetries = 1;
    MyAutonetworkParams.PrebondingInterval = 35;
    MyAutonetworkParams.DiscoveryTxPower = 1;

    // read first parameter
    if (ccpFindCmdParameter(CcpCommandParameter)){
        MyAutonetworkParams.TemporaryAddressTimeout = atoi(CcpCommandParameter);
    }

    // read second parameter
    if (ccpFindCmdParameter(CcpCommandParameter)){
        MyAutonetworkParams.AuthorizeRetries = atoi(CcpCommandParameter);
    }

    // read third parameter
    if (ccpFindCmdParameter(CcpCommandParameter)){
        MyAutonetworkParams.DiscoveryRetries = atoi(CcpCommandParameter);
    }

    // read fourth parameter
    if (ccpFindCmdParameter(CcpCommandParameter)){
        MyAutonetworkParams.PrebondingInterval = atoi(CcpCommandParameter);
    }

    // read fifth parameter
    if (ccpFindCmdParameter(CcpCommandParameter)){
        MyAutonetworkParams.DiscoveryTxPower = atoi(CcpCommandParameter);
    }

    autonetwork(&MyAutonetworkParams);
}

/**
 * clrbonds commands service
 *
 * @param - CommandTabParameter command parameter from decode command table
 * @return - none
 *
 */
void ccpClrBondsCmd (uint16_t CommandTabParameter)
{
    removeAllBonds();
}

/**
 * ledr commands service
 *
 * @param - CommandTabParameter command parameter from decode command table
 * @return - none
 *
 */
void ccpLedCmd (uint16_t CommandTabParameter)
{
    uint16_t Addr;
    uint8_t Cmd;

    // processing of command input parameters
    // read required operation
    if (ccpFindCmdParameter(CcpCommandParameter)){
        if (strcmp("on",CcpCommandParameter)==0) Cmd = CMD_LED_SET_ON;
        else if (strcmp("off",CcpCommandParameter)==0) Cmd = CMD_LED_SET_OFF;
             else Cmd = CMD_LED_PULSE;
    }

    // read destination address
    if (ccpFindCmdParameter(CcpCommandParameter)){
        Addr = atoi(CcpCommandParameter);         // set destination address
    }
    else Addr = 0;

    ledR(Addr, Cmd);                              // send ledR DPA command
}
