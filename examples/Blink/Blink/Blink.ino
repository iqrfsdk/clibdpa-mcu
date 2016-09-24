/**
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

#include <stddef.h>
#include <stdint.h>
#include <dpa_library.h>
#if defined(__PIC32MX__)
#include <WProgram.h>
#else
#include <Arduino.h>
#endif
#if defined(__AVR__)
#include <MsTimer2.h>
#include <TimerOne.h>
#elif defined(__SAM3X8E__)
#include <DueTimer.h>
#endif
#if defined(__SPI_INTERFACE__)
#include <SPI.h>
#endif

/*
 * C prototypes
 */
#if defined(__SPI_INTERFACE__)
extern "C" void DPA_DeselectTRmodule();
extern "C" uint8_t DPA_SendSpiByte(uint8_t txByte);
#elif defined(__UART_INTERFACE__)
extern "C" void DPA_SendUartByte(uint8_t txByte);
#endif

/*
 * C++ prototypes 
 */
void setup();
void loop();
void greenLed(uint16_t addr, uint8_t cmd);
void redLed(uint16_t addr, uint8_t cmd);
void dpaRequests();
void dpaAnswerHandler(T_DPA_PACKET *dpaAnswerPacket);
void dpaTimeoutHandler();
void swTimeoutHandler();
void msTimerCallback();
#if defined(TR7xD)
void usTimerCallback();
#endif
#if defined(__PIC32MX__)
uint32_t msTimerCallbackPic32(uint32_t currentTime);
#if defined(TR7xD)
uint32_t usTimerCallbackPic32(uint32_t currentTime);
#endif
#endif

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
 * Message types
 */

#define CONFIRMATION  1
#define RESPONSE      2

/*
 * Statuses
 */
#define NO_WAITING            0
#define CONFIRMATION_WAITING  1
#define RESPONSE_WAITING      2

/*
 * Variables
 */
#if defined(__SPI_INTERFACE__)
/// SPI SS (Slave select) pin of IQRF DCTR module
uint8_t iqrfSs = 10;
#endif
/// User timer period in ms
uint16_t userTimerPeriod = 1000; // 1000@1ms = 1s

typedef enum {
  GLED_PULSE,
  RLED_PULSE
} DPA_RQ;

typedef struct {
  // Timer
  volatile uint16_t swTimer;
  volatile bool swTimerAck;
  volatile uint8_t ticks;

  // DPA
  volatile uint16_t dpaTimeoutCnt;
  uint16_t dpaStep;
  uint8_t state;
  T_DPA_PACKET myDpaRequest;
  DPA_RQ cmds;
} app_vars_t;
app_vars_t app_vars;

/**
 * Init peripherals
 */
void setup() {
  Serial.begin(9600);
#if defined(__UART_INTERFACE__)
  Serial1.begin(115200);
#elif defined(__SPI_INTERFACE__)
  pinMode(iqrfSs, OUTPUT);
  digitalWrite(iqrfSs, HIGH);
  SPI.begin();
#endif
#if defined(__AVR__)
  MsTimer2::set(1, msTimerCallback);
  MsTimer2::start();
#if defined(TR7xD)
  Timer1.initialize(150);
  Timer1.attachInterrupt(usTimerCallback);
#endif
#elif defined(__SAM3X8E__)
  Timer6.attachInterrupt(msTimerCallback).start(1000);
#if defined(TR7xD)
  Timer7.attachInterrupt(usTimerCallback).start(150);
#endif
#elif defined(__PIC32MX__)
  attachCoreTimerService(msTimerCallbackPic32);
#if defined(TR7xD)
  attachCoreTimerService(usTimerCallbackPic32);
#endif
#endif
  // Clear local variable
  memset(&app_vars, 0, sizeof(app_vars_t));
  app_vars.swTimer = userTimerPeriod;
  app_vars.state = 1;
  // DPA library initialization
  DPA_Init();
  // Set DPA response handler
  DPA_SetAnswerHandler(dpaAnswerHandler);
  // Wait 1s
  delay(1000);
  Serial.println("Peripheral and DPA init done");
}

/**
 * Main loop
 */
void loop() {
#if defined(__UART_INTERFACE__)
  // Check and read data from UART
  DPA_ReadUartByte();
#endif
  // Sending DPA requests
  if (app_vars.swTimerAck) {
    // Function for sending DPA requests over SPI/UART
    dpaRequests();
    app_vars.swTimerAck = false;
  }
  // Run DPA library handler
  DPA_LibraryDriver();
}

/**
 * DPA request to drive green LED peripheral
 * @param addr Address
 * &param cmd Command
 */
void greenLed(uint16_t addr, uint8_t cmd) {
  app_vars.myDpaRequest.NADR = addr;
  app_vars.myDpaRequest.PNUM = PNUM_LEDG;
  app_vars.myDpaRequest.PCMD = cmd;
  app_vars.myDpaRequest.HWPID = HWPID_DoNotCheck; // replace HW_PROFILE_DO_NOT_CHECK
  // Initial timeout 1s for either confirmation or response
  app_vars.dpaTimeoutCnt = 1000;
  DPA_SendRequest(&app_vars.myDpaRequest, 0);
}

/**
 * DPA request to drive red LED peripheral
 * @param addr Address
 * &param cmd Command
 */
void redLed(uint16_t addr, uint8_t cmd) {
  app_vars.myDpaRequest.NADR = addr;
  app_vars.myDpaRequest.PNUM = PNUM_LEDR;
  app_vars.myDpaRequest.PCMD = cmd;
  app_vars.myDpaRequest.HWPID = HWPID_DoNotCheck; // replace HW_PROFILE_DO_NOT_CHECK
  // Initial timeout 1s for either confirmation or response
  app_vars.dpaTimeoutCnt = 1000;
  DPA_SendRequest(&app_vars.myDpaRequest, 0);
}

/**
 * Send DPA requests
 */
void dpaRequests() {
  // if DPA library is busy, or dpa operation in progress, then return
  if (DPA_GetStatus() != DPA_READY || app_vars.dpaStep != NO_WAITING) {
    return;
  }
  // 100ms before sending another request
  app_vars.ticks = 100;
  while (app_vars.ticks) {
  }
  switch (app_vars.state) {
    case 1:
      redLed(COORDINATOR, CMD_LED_PULSE);
      Serial.println("LEDR pulse");
      app_vars.state = 2;
      break;
    case 2:
      greenLed(NODE1, CMD_LED_PULSE);
      Serial.println("LEDG pulse");
      app_vars.state = 1;
      break;
  }
  if (app_vars.myDpaRequest.NADR == 0x00 || app_vars.myDpaRequest.NADR == 0xFC) {
    // DPA answer would be response
    app_vars.dpaStep = RESPONSE_WAITING;
  } else {
    // DPA answer would be confirmation
    app_vars.dpaStep = CONFIRMATION_WAITING;
  }
}

/**
 * DPA answer handler
 * @param dpaAnswerPacket Pointer to DPA answer packet
 */
void dpaAnswerHandler(T_DPA_PACKET *dpaAnswerPacket) {
  if (app_vars.dpaStep == CONFIRMATION_WAITING &&
      dpaAnswerPacket->ResponseCode == STATUS_CONFIRMATION) {
        app_vars.dpaStep = RESPONSE_WAITING;
        app_vars.dpaTimeoutCnt = DPA_GetEstimatedTimeout();
        return;
    }
    if (app_vars.dpaStep == RESPONSE_WAITING &&
        dpaAnswerPacket->ResponseCode == STATUS_NO_ERROR) {
        app_vars.dpaStep = NO_WAITING;
        app_vars.dpaTimeoutCnt = 0;
        // ANY USER CODE FOR DPA RESPONSE PROCESSING
        return;
    }
    // ANY USER CODE FOR ASYNCHRONOUS DPA MESSAGES PROCESSING
}

/**
 * DPA response timeout handler
 */
void dpaTimeoutHandler() {
  app_vars.dpaStep = NO_WAITING;
  // ANY USER CODE FOR OPERATION TIMEOUT HANDLING
}

/**
 * User timeout handler
 */
void swTimeoutHandler() {
  app_vars.swTimerAck = true;
}

/**
 * 1ms timer callback
 */
void msTimerCallback() {
#if defined(TR5xD)
  DPA_SetTimmingFlag();
#endif
  // Timeout handler
  if (app_vars.dpaTimeoutCnt) {
    if ((--app_vars.dpaTimeoutCnt) == 0) {
      // If timeout is expired it call timeout handler
      dpaTimeoutHandler();
    }
  }
  // SW timer, call timeout handler
  if (app_vars.swTimer) {
    if ((--app_vars.swTimer) == 0) {
      swTimeoutHandler();
      app_vars.swTimer = userTimerPeriod;
    }
  }
  // Delay timer
  if (app_vars.ticks) {
     --app_vars.ticks;
  }
}

/**
 * 150us timer callback for DCTR-7x modules
 */
void usTimerCallback() {
  DPA_SetTimmingFlag();
}

#if defined(__PIC32MX__)
/**
 * 1ms timer callback
 * @param currentTime Current time
 * @return Next time
 */
uint32_t msTimerCallbackPic32(uint32_t currentTime) {
  msTimerCallback();
  return(currentTime + CORE_TICK_RATE);
}
#if defined(TR7xD)
/**
 * 150us timer callback for DCTR-7x modules
 * @param currentTime Current time
 * @return Next time
 */
uint32_t usTimerCallbackPic32(uint32_t currentTime) {
  usTimerCallback();
  return(currentTime + CORE_TICK_RATE);
}
#endif
#endif

#if defined(__SPI_INTERFACE__)

/**
 * DPA SPI deselect module
 */
void DPA_DeselectTRmodule() {
  pinMode(iqrfSs, OUTPUT);
  digitalWrite(iqrfSs, HIGH);
}

/**
 * Send DPA byte over SPI
 * @param txByte Byte to send
 * @return Received byte
 */
uint8_t DPA_SendSpiByte(uint8_t txByte) {
  digitalWrite(iqrfSs, LOW);
  delayMicroseconds(15);
  SPI.beginTransaction(SPISettings(250000, MSBFIRST, SPI_MODE0));
  uint8_t rxByte = SPI.transfer(txByte);
  SPI.endTransaction();
  delayMicroseconds(15);
  digitalWrite(iqrfSs, HIGH);
  return rxByte;
}
#elif defined(__UART_INTERFACE__)

/**
 * Send DPA byte over UART
 * @param txByte Byte to send
 */
void DPA_SendUartByte(uint8_t txByte) {
  Serial1.write(txByte);
}

/**
 * Read DPA byte from UART
 */
void DPA_ReadUartByte() {
  while(Serial1.available()) {
    // Read received byte
    uint8_t rxByte = Serial1.read();
    // Load that byte to DPA library
    DPA_ReceiveUartByte(rxByte);
  }
}
#endif
