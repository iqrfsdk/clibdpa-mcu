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

#include <stdio.h>

#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>

#include <MsTimer2.h>
#include <TimerOne.h>

//dpa library
#include "dpa_library.h"

//=========================== defines =========================================

#define LEONARDO
#define SS                  10

#define USER_TIMER_PERIOD		1000		// 1000@1ms = 1s
#define TRUE                1
#define FALSE               0

#define CONFIRMATION		    1
#define RESPONSE			      2

#define NO_WAITING			      0
#define CONFIRMATION_WAITING	1
#define RESPONSE_WAITING		  2

#define COORDINATOR			0
#define NODE1				    1
#define NODE2				    2
#define NODE3				    3
#define NODE4				    4
#define NODE5				    5
#define LOCAL           0xFC

//=========================== variables =======================================
typedef enum
{
  GLED_PULSE,
  RLED_PULSE
}DPA_RQ;

typedef struct
{
	// timer
	volatile uint16_t swTimer;
	volatile uint8_t swTimerAck;
	volatile uint8_t ticks;

	// dpa
	volatile uint16_t dpaTimeoutCnt;
	uint16_t dpaStep;
  uint8_t state;
	T_DPA_PACKET myDpaRequest;
	DPA_RQ cmds;
} app_vars_t;

app_vars_t app_vars;

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network.
// gateway and subnet are optional:
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};

IPAddress ip(192, 168, 1, 177);
IPAddress server(192, 168, 1, 1);
IPAddress mydns(192, 168, 1, 1);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

EthernetClient ethClient;
PubSubClient client(ethClient);

//=========================== prototypes ======================================

//dpa handlers
void MyDpaAnswerHandler(T_DPA_PACKET *MyDpaAnswer);
void MyDpaLibTimeoutHandler(void);
void MyDpaLibRequests(void);

//dpa requests
void DpaLedG(uint16_t addr, uint8_t cmd);
void DpaLedR(uint16_t addr, uint8_t cmd);

//dpa driver timing
#ifdef LEONARDO
void cb_timer1ms(void);
#ifdef TR7xD
void cb_timer150us(void);
#endif
#endif

//user timer
void MySwTimerTimeoutHandler(void);

//serial check and read
void MySerialEvent(void);

//mqtt connection
void mqtt_callback(void);
void mqtt_reconnect(void);

//=========================== callbacks ======================================
void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  for (int i=0; i<length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

//=========================== init peripherals ===============================
void setup()
{
    //up - PC
    Serial.begin(9600);

    //down - DCTR
#ifdef __UART_INTERFACE__
    Serial1.begin(9600);
#endif
#ifdef __SPI_INTERFACE__
#ifdef LEONARDO
    pinMode( SS, OUTPUT );
    digitalWrite( SS, HIGH );

    SPI.begin();
#endif
#endif

#ifdef LEONARDO
    //timer 1ms
	MsTimer2::set(1, cb_timer1ms);
  MsTimer2::start();

	//timer 150us for fast spi on 7x modules
  Timer1.initialize(150);
  Timer1.attachInterrupt(cb_timer150us);
#endif

  //clear local variable
  memset(&app_vars, 0, sizeof(app_vars_t));
  app_vars.swTimer = USER_TIMER_PERIOD;
  app_vars.state = 1;

  //dpa library initialization
  DPA_Init();
  DPA_SetAnswerHandler(MyDpaAnswerHandler);   // set DPA response handler

  // mqtt
  client.setServer(server, 1883);
  client.setCallback(mqtt_callback);

  // initialize the ethernet device
  Ethernet.begin(mac, ip, mydns, gateway, subnet);

  // Allow the hardware to sort itself out
  delay(1000);

  //done here
  Serial.println(Ethernet.localIP());
  Serial.println("Peripheral and DPA init done");
}

//=========================== main ============================================
void loop()
{
    //check and read serial data
#ifdef __UART_INTERFACE__
    MySerialEvent();
#endif

    //sending dpa requests
    if (app_vars.swTimerAck == TRUE)
    {
      MyDpaLibRequests();				    // function for sending DPA requests - SPI/UART
      app_vars.swTimerAck = FALSE;
    }

    //run dpa driver
    DPA_LibraryDriver();					// DPA library handler

    //run mqtt driver
    if (!client.connected()) {
      mqtt_reconnect();
    }
    client.loop();
}
//=============================================================================

/**
* Check mqtt connection and reconnect
*
* @param     none
* @return   none
*
**/
void mqtt_reconnect(void) {

  Serial.print("Attempting MQTT connection...");

  // Attempt to connect
  if (client.connect("iqrfClient")) {
    Serial.println("connected");
    // Once connected, publish an announcement...
    client.publish("outTopic","hello world");
    // ... and resubscribe
    client.subscribe("inTopic");
  } else {
    Serial.print("failed, rc=");
    Serial.print(client.state());
    Serial.println(" try again in 5 seconds");
  }
}

/**
* Serial checking and reading
*
* @param     none
* @return   none
*
**/
void MySerialEvent(void)
{
    uint8_t byte;

    while(Serial1.available())
    {
        // read received byte
        byte = Serial1.read();

        // load that byte to dpa library
        DPA_ReceiveUartByte(byte);
    }
}
//=============================================================================

/**
* DPA requests sending
*
* @param 	none
* @return 	none
*
**/
void MyDpaLibRequests(void)
{
    // if DPA library is busy, or dpa operation in progress, then return
    if (DPA_GetStatus() != DPA_READY || app_vars.dpaStep != NO_WAITING) {
      return;
    }

    // 100ms before sending another request
    app_vars.ticks = 100;
    while (app_vars.ticks)
        ;

    switch (app_vars.state) {
      case 1:
        DpaLedR(COORDINATOR, CMD_LED_PULSE);
        Serial.println("C LEDR pulse");

        if (client.connected()) {
          client.publish("outTopic","C LEDR pulse");
        }

        app_vars.state = 2;
	    break;

      case 2:
	      DpaLedG(NODE1, CMD_LED_PULSE);
        Serial.println("N1 LEDG pulse");

        if (client.connected()) {
          client.publish("outTopic","N1 LEDG pulse");
        }

        app_vars.state = 1;
	    break;

      default:
      break;
    }

    if (app_vars.myDpaRequest.NADR == 0x00 || app_vars.myDpaRequest.NADR == 0xFC)
        app_vars.dpaStep = RESPONSE_WAITING;			   // dpa answer would be response
    else
        app_vars.dpaStep = CONFIRMATION_WAITING;		 // dpa answer would be confirmation
}
//=============================================================================

/**
* DPA answer handler
*
* @param 	pointer to DPA answer packet
* @return 	none
*
**/
void MyDpaAnswerHandler(T_DPA_PACKET *dpaAnswerPkt)
{
    if (app_vars.dpaStep == CONFIRMATION_WAITING
        && dpaAnswerPkt->ResponseCode
        == STATUS_CONFIRMATION)
    {
        app_vars.dpaStep = RESPONSE_WAITING;
        app_vars.dpaTimeoutCnt = DPA_GetEstimatedTimeout();
        return;
    }

    if (app_vars.dpaStep == RESPONSE_WAITING
        && dpaAnswerPkt->ResponseCode == STATUS_NO_ERROR)
    {
        app_vars.dpaStep = NO_WAITING;
        app_vars.dpaTimeoutCnt = 0;

        // ANY USER CODE FOR DPA RESPONSE PROCESSING
        return;
    }

    // ANY USER CODE FOR ASYNCHRONOUS DPA MESSAGES PROCESSING
}
//=============================================================================

/**
* DPA response timeout handler
*
* @param 	none
* @return 	none
*
**/
void MyDpaLibTimeoutHandler(void)
{
    app_vars.dpaStep = NO_WAITING;

    // ANY USER CODE FOR OPERATION TIMEOUT HANDLING
}
//=============================================================================

/**
* DPA deselect module
*
* @param   none
* @return   none
*
**/
void DPA_DeselectTRmodule(void)
{
    pinMode( SS, OUTPUT );
    digitalWrite( SS, HIGH );
}
//=============================================================================

/**
* DPA request to drive ledG peripheral
*
* @param 	addr
* @param 	cmd
* @return
*
**/
void DpaLedG(uint16_t addr, uint8_t cmd)
{
    app_vars.myDpaRequest.NADR = addr;
    app_vars.myDpaRequest.PNUM = PNUM_LEDG;
    app_vars.myDpaRequest.PCMD = cmd;
    app_vars.myDpaRequest.HWPID = HWPID_DoNotCheck; // replace HW_PROFILE_DO_NOT_CHECK

    // initial timeout 1s for either confirmation or response
    app_vars.dpaTimeoutCnt = 1000;

    DPA_SendRequest(&app_vars.myDpaRequest, 0);
}
//=============================================================================

/**
* DPA request to drive ledR peripheral
*
* @param 	addr
* @param 	cmd
* @return
*
**/
void DpaLedR(uint16_t addr, uint8_t cmd)
{
    app_vars.myDpaRequest.NADR = addr;
    app_vars.myDpaRequest.PNUM = PNUM_LEDR;
    app_vars.myDpaRequest.PCMD = cmd;
    app_vars.myDpaRequest.HWPID = HWPID_DoNotCheck; // replace HW_PROFILE_DO_NOT_CHECK

    // initial timeout 1s for either confirmation or response
    app_vars.dpaTimeoutCnt = 1000;

    DPA_SendRequest(&app_vars.myDpaRequest, 0);
}
//=============================================================================

/**
* User timer timeout handler
*
* @param 	none
* @return 	none
*
**/
void MySwTimerTimeoutHandler(void)
{
    app_vars.swTimerAck = TRUE;
}
//=============================================================================

/**
* 1ms timer callback
*
* @param 	none or currentTime
* @return 	none or nextTime
*
**/
#ifdef LEONARDO
void cb_timer1ms(void)
#endif
{
#ifdef TR5xD
  // dpa timing
  DPA_SetTimmingFlag();	// set 1ms flag
#endif

  // timeout handler
  if (app_vars.dpaTimeoutCnt)
  {
    if ((--app_vars.dpaTimeoutCnt) == 0)
    {
      MyDpaLibTimeoutHandler();  // if timeout expired, call timeout handler
    }
  }

  // sw timer, call timeout handler
  if (app_vars.swTimer)
  {
    if ((--app_vars.swTimer) == 0)
    {
      MySwTimerTimeoutHandler();
      app_vars.swTimer = USER_TIMER_PERIOD;
    }
  }

  // delay timer
  if (app_vars.ticks)
  {
    --app_vars.ticks;
  }
}
//=============================================================================

/**
* 150us timer callback for 7x modules spi
*
* @param 	none
* @return none
*
**/
#ifdef TR7xD
void cb_timer150us(void)
{
  // dpa timing
  DPA_SetTimmingFlag();	// set 150us flag
}
#endif
//=============================================================================

/**
* Send DPA byte over UART
*
* @param     Tx_Byte to send
* @return   none
*
**/
void DPA_SendUartByte(uint8_t Tx_Byte)
{
  Serial1.write(Tx_Byte);
}
//=============================================================================

/**
* Send DPA byte over SPI
*
* @param       Tx_Byte to send
* @return      Received Rx_Byte
*
**/
uint8_t DPA_SendSpiByte(uint8_t Tx_Byte)
{
  uint8_t Rx_Byte;

#ifdef LEONARDO
  digitalWrite( SS, LOW );
  delayMicroseconds( 15 );

  SPI.beginTransaction(SPISettings(250000, MSBFIRST, SPI_MODE0));
  Rx_Byte = SPI.transfer( Tx_Byte );
  SPI.endTransaction();

  delayMicroseconds( 15 );
  digitalWrite( SS, HIGH );
#endif

  return Rx_Byte;
}
//=============================================================================
