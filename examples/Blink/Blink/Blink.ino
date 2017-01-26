
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

#define TR_SS           10          // TR module chip select pin
#define PWR             9           // TR module power control pin

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
  extern "C" void dpaSendUartByte(uint8_t Tx_Byte);
  extern "C" uint8_t dpaReceiveUartByte(uint8_t *Rx_Byte);
#endif

/*
 * C++ prototypes
 */
void myAsyncPacketHandler(T_DPA_PACKET *dpaAnswerPkt);
uint8_t sendMyDpaRequest(T_DPA_PACKET *DpaRequest, UINT8 DataSize, UINT16 Timeout);
void simpleDpaCmd (uint16_t Addr, uint8_t Peripheral, uint8_t Cmd);
#if defined(__PIC32MX__)
  uint32_t systemTimerIterruptHandler(uint32_t CurrentTime);
#else
  void systemTimerIterruptHandler(void);
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

//=============================================================================
void setup() {
  Serial.begin(9600);

  AppVars.State = 0;
  AppVars.AsyncPacketFlag = false;

  #ifdef __SPI_INTERFACE__
  pinMode(PWR, OUTPUT);                 // TR module power off (make TR module RESET)
  digitalWrite(PWR, HIGH);
  pinMode(TR_SS, OUTPUT);
  digitalWrite(TR_SS, LOW);
  delay(500);

  digitalWrite(PWR, LOW);               // TR module power on
  digitalWrite(TR_SS, HIGH);            // deselect TR module
  SPI.begin();                          // start SPI peripheral
  delay(500);                           // pause
  #endif

  #ifdef __UART_INTERFACE__
  pinMode(PWR, OUTPUT);                 // TR module power off (make TR module RESET)
  digitalWrite(PWR, HIGH);
  delay(500);

  digitalWrite(PWR, LOW);               // TR module power on
  Serial1.begin(57600);                 // start UART1 peripheral
  #endif

  dpaInit(myAsyncPacketHandler);        // initialize DPA library

  #if defined(__AVR__) || defined(CORE_TEENSY)
    #if defined(TR7xD) || defined(__UART_INTERFACE__)
      Timer1.initialize(150);                                     // initialize timer1, call dpa driver every 150us
      AppVars.SwTimer = AppVars.SwTimerPreset = 10000;            // time 1.5s for DPA request sending
    #else
      Timer1.initialize(1000);                                    // initialize timer1, call dpa driver every 1000us
      AppVars.SwTimer = AppVars.SwTimerPreset = 1500;             // time 1.5s for DPA request sending
    #endif
    Timer1.attachInterrupt(systemTimerIterruptHandler);           // attaches callback() as a timer overflow interrupt
  #endif

  #if defined(__SAM3X8E__)
    #if defined(TR7xD) || defined(__UART_INTERFACE__)
      Timer6.attachInterrupt(systemTimerIterruptHandler).start(150); // initialize timer1, call dpa driver every 150us
      AppVars.SwTimer = AppVars.SwTimerPreset = 10000;            // time 1.5s for DPA request sending
    #else
      Timer6.attachInterrupt(systemTimerIterruptHandler).start(1000);// initialize timer1, call dpa driver every 1000us
      AppVars.SwTimer = AppVars.SwTimerPreset = 1500;             // time 1.5s for DPA request sending
    #endif
  #endif

  #if defined(__PIC32MX__)
    #if defined(TR7xD) || defined(__UART_INTERFACE__)
      AppVars.SwTimerTimmingInterval = CORE_TICK_RATE / 6.7;
      AppVars.SwTimer = AppVars.SwTimerPreset = 10000;            // time 1.5s for DPA request sending
    #else
      AppVars.SwTimerTimmingInterval = CORE_TICK_RATE;
      AppVars.SwTimer = AppVars.SwTimerPreset = 1500;             // time 1.5s for DPA request sending
    #endif
    attachCoreTimerService(systemTimerIterruptHandler);
  #endif

}
//=============================================================================

void loop() {

  if (AppVars.SwTimerAck == true){                              // DPA request time flag is set
    AppVars.SwTimerAck = false;

    switch (AppVars.State){                                     // select which request is going to send
      case 0:                                                   // coordinator - LED R pulse
        Serial.println("LEDR pulse on Coordinator");
        simpleDpaCmd (COORDINATOR, PNUM_LEDR, CMD_LED_PULSE);
        AppVars.State++;
        break;

      case 1:                                                   // node 1 - LED G pulse
        Serial.println("LEDG pulse on Node 1");
        simpleDpaCmd (NODE1, PNUM_LEDG, CMD_LED_PULSE);
        AppVars.State++;
        break;

      case 2:                                                   // node 1 - LED G pulse
        Serial.println("LEDG pulse on Node 2");
        simpleDpaCmd (NODE2, PNUM_LEDG, CMD_LED_PULSE);
        AppVars.State = 0;
        break;
    }

  }

  if (AppVars.AsyncPacketFlag == true){                         // asynchronous packet indication
    AppVars.AsyncPacketFlag = false;
    Serial.println();
    Serial.println("!!! Asynchronous DPA packet received !!!");
    Serial.println();
  }

}
//=============================================================================

/**
* System timer Interrupt handler  (1ms or 150us)
*
* @param   none
* @return   none
*
**/
#if defined(__PIC32MX__)
  uint32_t systemTimerIterruptHandler(uint32_t CurrentTime){

    if (AppVars.SwTimer) AppVars.SwTimer--;
    else{
      AppVars.SwTimer = AppVars.SwTimerPreset;
      AppVars.SwTimerAck = true;
    }

    dpaLibraryDriver();
    return(CurrentTime + AppVars.SwTimerTimmingInterval);
  }
#else
  void systemTimerIterruptHandler(void){

    if (AppVars.SwTimer) AppVars.SwTimer--;
    else{
      AppVars.SwTimer = AppVars.SwTimerPreset;
      AppVars.SwTimerAck = true;
    }

    dpaLibraryDriver();
  }
#endif
//=============================================================================

#ifdef __SPI_INTERFACE__
/**
* Send DPA byte over SPI
*
* @param       Tx_Byte to send
* @return      Received Rx_Byte
*
**/
uint8_t dpaSendSpiByte(uint8_t Tx_Byte){
    uint8_t Rx_Byte;

    if (!DpaControl.TRmoduleSelected){
        SPI.beginTransaction(SPISettings(250000, MSBFIRST, SPI_MODE0));
        DpaControl.TRmoduleSelected = true;
        digitalWrite(TR_SS, LOW);
        delayMicroseconds(15);
    }

    Rx_Byte = SPI.transfer(Tx_Byte);

    #ifdef TR5xD
        delayMicroseconds(15);
        digitalWrite(TR_SS, HIGH);
        DpaControl.TRmoduleSelected = false;
        SPI.endTransaction();
    #endif

    return Rx_Byte;
}
//=============================================================================

/**
* DPA deselect module
*
* @param   none
* @return   none
*
**/
void dpaDeselectTRmodule(void){
    digitalWrite(TR_SS, HIGH);
    DpaControl.TRmoduleSelected = false;
    SPI.endTransaction();
}
//=============================================================================
#endif

#ifdef __UART_INTERFACE__
/**
* Send DPA byte over SCI
*
* @param       Tx_Byte to send
* @return      none
*
**/
void dpaSendUartByte(uint8_t Tx_Byte){
  Serial1.write(Tx_Byte);
}
//=============================================================================

/**
* Read DPA byte over SCI
*
* @param       pointer to char Rx_Byte to transfer received byte to dpa library
* @return      false - no char to read
*              true - character ready
*
**/
uint8_t dpaReceiveUartByte(uint8_t *Rx_Byte){
  if (Serial1.available() > 0){
    *Rx_Byte = Serial1.read();
    return(true);
  }
  return(false);
}
//=============================================================================
#endif

/*
 *------------------------------------------------------------
 *           asynchronous packet service rutine
 *------------------------------------------------------------
 */
void myAsyncPacketHandler(T_DPA_PACKET *dpaAnswerPkt){

  /* here you can add any code for asynchronous packet service */

  AppVars.AsyncPacketFlag = true;
}

/*
*------------------------------------------------------------
*           send DPA request and process response
*------------------------------------------------------------
*/
uint8_t sendMyDpaRequest(T_DPA_PACKET *DpaRequest, UINT8 DataSize, UINT16 Timeout){

  uint8_t OpResult;

  Serial.println("Sending request");

  while((OpResult = dpaSendRequest(DpaRequest, DataSize, Timeout)) == DPA_OPERATION_IN_PROGRESS); // send request and wait for result

  switch(OpResult){
      case DPA_OPERATION_OK: Serial.println("Operation OK"); break;             // operation OK
      case DPA_OPERATION_TIMEOUT: Serial.println("Command timeout"); break;     // operation timeout
      case DPA_CONFIRMATION_ERR: Serial.println("Confirmation ERROR"); break;   // confirmation error
      case DPA_RESPONSE_ERR: Serial.println("Response ERROR"); break;           // response error
      case DPA_TR_MODULE_NOT_READY: Serial.println("TR module not ready"); break; // TR module not ready
  }

  Serial.println();

  return(OpResult);
}
//=============================================================================


/*
*------------------------------------------------------------
*                 node dpa commands service
*------------------------------------------------------------
*/
void simpleDpaCmd (uint16_t Addr, uint8_t Peripheral, uint8_t Cmd){

  AppVars.MyDpaRequest.NADR = Addr;                                 // set address
  AppVars.MyDpaRequest.PNUM = Peripheral;                           // set peripheral
  AppVars.MyDpaRequest.PCMD = Cmd;                                  // set peripheral command
  AppVars.MyDpaRequest.HWPID = HWPID_DoNotCheck;                    // do not check HWPID

  sendMyDpaRequest(&AppVars.MyDpaRequest, 0, 2000);                 // send request and process response
}
//=============================================================================
