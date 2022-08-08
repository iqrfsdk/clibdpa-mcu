/**
 * Copyright 2015-2021 IQRF Tech s.r.o.
 * Copyright 2019-2021 MICRORISC s.r.o.
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
#include <dpa_library.h>

#if defined(__PIC32MX__)
#include <WProgram.h>
#else
#include <Arduino.h>
#endif

#if defined(__AVR__) || defined(CORE_TEENSY)
#include <TimerOne.h>
#elif defined(__SAM3X8E__)
#include <DueTimer.h>
#endif

#if defined(__SPI_INTERFACE__)
#include <SPI.h>
#endif

#define TR_SS       8           // TR module chip select pin
#define PWR         9           // TR module power control pin

/*
 * Addresses
 */
#define COORDINATOR 0
#define NODE1       1
#define NODE2       2
#define NODE3       3
#define NODE4       4
#define NODE5       5
#define LOCAL       0xFC

/*
 * C prototypes
 */
#if defined(__SPI_INTERFACE__)
extern "C" uint8_t dpaSendSpiByte(uint8_t Tx_Byte);
extern "C" void dpaDeselectTRmodule(void);
#elif defined(__UART_INTERFACE__)
#if defined (__AVR_ATmega168__) || defined (__AVR_ATmega168P__) || \
    defined (__AVR_ATmega328__) || defined (__AVR_ATmega328P__)
#include <SoftwareSerial.h>
// RX: D2, TX: D3
SoftwareSerial Serial1(2, 3);
#endif
extern "C" void dpaSendUartByte(uint8_t Tx_Byte);
extern "C" uint8_t dpaReceiveUartByte(uint8_t *Rx_Byte);
#endif

/*
 * C++ prototypes
 */
void myAsyncPacketHandler(T_DPA_PACKET *dpaAnswerPkt);
uint8_t sendMyDpaRequest(T_DPA_PACKET *DpaRequest, uint8_t DataSize, uint32_t Timeout);
void simpleDpaCmd(uint16_t Addr, uint8_t Peripheral, uint8_t Cmd);
void systemTimerIterruptHandler(void);
#if defined(__PIC32MX__)
uint32_t systemTimerIterruptHandlerPic32(uint32_t CurrentTime);
#endif

/*
 * Variables
 */
typedef struct {
    // Timer
    volatile uint16_t SwTimer;
    uint16_t SwTimerPreset;
    volatile bool SwTimerAck;
#if defined(__PIC32MX__)
    uint32_t SwTimerTimmingInterval;
#endif
    // DPA
    uint8_t State;
    uint8_t AsyncPacketFlag;
    T_DPA_PACKET MyDpaRequest;
} APP_VARS;
APP_VARS AppVars;

/**
 * Setup peripherals
 */
void setup()
{
    Serial.begin(9600);

    AppVars.State = 0;
    AppVars.AsyncPacketFlag = false;

    // Set TR power pin as output
    pinMode(PWR, OUTPUT);
    // TR module power off (make TR module RESET)
    digitalWrite(PWR, HIGH);

#if defined(__SPI_INTERFACE__)
    // Set SPI slave select pin as output
    pinMode(TR_SS, OUTPUT);
    digitalWrite(TR_SS, LOW);
    // pause
    delay(500);

    // TR module power on
    digitalWrite(PWR, LOW);
    // deselect TR module
    digitalWrite(TR_SS, HIGH);
    // start SPI peripheral
    SPI.begin();
    // pause
    delay(500);
#endif

#if defined(__UART_INTERFACE__)
    delay(500);

    digitalWrite(PWR, LOW); // TR module power on
    Serial1.begin(57600);   // start UART1 peripheral
#endif

    // initialize DPA library
    dpaInit(myAsyncPacketHandler);

#if defined(__AVR__) || defined(CORE_TEENSY)
    // initialize Timer1, call DPA driver every 200us
    Timer1.initialize(200);
    // time 1.5s for DPA request sending
    AppVars.SwTimer = AppVars.SwTimerPreset = 7500;
    // attaches callback() as a timer overflow interrupt
    Timer1.attachInterrupt(systemTimerIterruptHandler);
#endif

#if defined(__SAM3X8E__)
    // initialize DueTimer (6), call DPA driver every 200us
    Timer6.attachInterrupt(systemTimerIterruptHandler).start(200);
    // time 1.5s for DPA request sending
    AppVars.SwTimer = AppVars.SwTimerPreset = 7500;
#endif

#if defined(__PIC32MX__)
    AppVars.SwTimerTimmingInterval = CORE_TICK_RATE / 5;
    // time 1.5s for DPA request sending
    AppVars.SwTimer = AppVars.SwTimerPreset = 7500;
    attachCoreTimerService(systemTimerIterruptHandlerPic32);
#endif

}

/**
 * Main loop
 */
void loop()
{
    // Is DPA request time flag set?
    if (AppVars.SwTimerAck) {
        AppVars.SwTimerAck = false;

        // select which request is going to send
        switch (AppVars.State) {
        case 0: // coordinator - LED R pulse
            Serial.println("LEDR pulse on Coordinator");
            simpleDpaCmd(COORDINATOR, PNUM_LEDR, CMD_LED_PULSE);
            AppVars.State++;
            break;

        case 1: // node 1 - LED G pulse
            Serial.println("LEDG pulse on Node 1");
            simpleDpaCmd(NODE1, PNUM_LEDG, CMD_LED_PULSE);
            AppVars.State++;
            break;

        case 2: // node 2 - LED G pulse
            Serial.println("LEDG pulse on Node 2");
            simpleDpaCmd(NODE2, PNUM_LEDG, CMD_LED_PULSE);
            AppVars.State = 0;
            break;
        }
    }

    // asynchronous packet indication
    if (AppVars.AsyncPacketFlag) {
        AppVars.AsyncPacketFlag = false;
        Serial.println();
        Serial.println("!!! Asynchronous DPA packet received !!!");
        Serial.println();
    }
}

/**
 * System timer Interrupt handler (200us)
 */
void systemTimerIterruptHandler(void)
{
    if (AppVars.SwTimer) {
        AppVars.SwTimer--;
    } else {
        AppVars.SwTimer = AppVars.SwTimerPreset;
        AppVars.SwTimerAck = true;
    }

    dpaLibraryDriver();
}

#if defined(__PIC32MX__)

/**
 * System timer Interrupt handler (200us) for PIC32
 * @param CurrentTime Current time
 * @return Time of next calling this function
 */
uint32_t systemTimerIterruptHandlerPic32(uint32_t CurrentTime)
{
    systemTimerIterruptHandler();
    return(CurrentTime + AppVars.SwTimerTimmingInterval);
}
#endif

#if defined(__SPI_INTERFACE__)

/**
 * Send DPA byte over SPI
 * @param Tx_Byte Byte to send
 * @return Received byte
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

    return(Rx_Byte);
}

/**
 * SPI deselect TR module
 */
void dpaDeselectTRmodule(void)
{
    digitalWrite(TR_SS, HIGH);
    DpaControl.TRmoduleSelected = false;
    SPI.endTransaction();
}

#elif defined(__UART_INTERFACE__)

/**
 * Send DPA byte over UART
 * @param Tx_Byte Byte to send
 */
void dpaSendUartByte(uint8_t Tx_Byte)
{
    Serial1.write(Tx_Byte);
}

/**
 * Read DPA byte over UART
 * @param Rx_Byte Pointer to char to transfer received byte to DPA library
 * @return false - no char to read; true - character ready
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

/**
 * Asynchronous packet service rutine
 * @param dpaAnswerPkt Pointer to DPA answer packet
 */
void myAsyncPacketHandler(T_DPA_PACKET *dpaAnswerPkt)
{
    /* here you can add any code for asynchronous packet service */

    AppVars.AsyncPacketFlag = true;
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
    uint8_t OpResult;

    Serial.println("Sending request");

    // send request and wait for result
    while ((OpResult = dpaSendRequest(DpaRequest, DataSize, Timeout)) == DPA_OPERATION_IN_PROGRESS)
        ; /* void */

    switch (OpResult) {
    case DPA_OPERATION_OK:                    // operation OK
        Serial.println("Operation OK");
        break;
    case DPA_OPERATION_TIMEOUT:               // operation timeout
        Serial.println("Command timeout");
        break;
    case DPA_CONFIRMATION_ERR:                // confirmation error
        Serial.println("Confirmation ERROR");
        break;
    case DPA_RESPONSE_ERR:                    // response error
        Serial.println("Response ERROR");
        break;
    case DPA_TR_MODULE_NOT_READY:             // TR module not ready
        Serial.println("TR module not ready");
        break;
    }

    Serial.println();

    return(OpResult);
}

/**
 * Node DPA commands service
 * @param Addr Address of a node
 * @param Peripheral Peripheral
 * @param Cmd Command
 */
void simpleDpaCmd(uint16_t Addr, uint8_t Peripheral, uint8_t Cmd)
{
    // set address
    AppVars.MyDpaRequest.NADR = Addr;
    // set peripheral
    AppVars.MyDpaRequest.PNUM = Peripheral;
    // set peripheral command
    AppVars.MyDpaRequest.PCMD = Cmd;
    // do not check HWPID
    AppVars.MyDpaRequest.HWPID = HWPID_DoNotCheck;

    // send request and process response
    sendMyDpaRequest(&AppVars.MyDpaRequest, 0, 2000);
}
