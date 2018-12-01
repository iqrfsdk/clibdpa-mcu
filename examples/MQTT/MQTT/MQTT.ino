/**
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

#include <stddef.h>
#include <stdint.h>
#include <dpa_library.h>
#include <dpa_json.h>

#if defined(__AVR__) || defined(CORE_TEENSY)
#include <TimerOne.h>
#endif

#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>

#define TR_SS       8           // TR module chip select pin
#define PWR         9           // TR module power control pin

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
void mqttCallback(char *Topic, byte *Payload, unsigned int Length);
void mqttReconnect();

void myAsyncPacketHandler(T_DPA_PACKET *dpaAnswerPkt);
void systemTimerIterruptHandler(void);

/*
 * Instances
 */
EthernetClient ethClient;
PubSubClient client(ethClient);

/*
 * Variables
 */
T_DPA_PACKET MyDpaPacket;
uint8_t MyMqttMessage[JSON_OBJECT_BUFF_SIZE];
uint8_t DpaDataReady = false;
uint8_t AsyncPacketReceived = false;

/// LAN Settings
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFA, 0xCE};
/// My IP address
IPAddress myIp(192, 168, 100, 90);
/// MQTT server IP address
IPAddress server(192, 168, 100, 170);

char MqttSubscribeTopic[] = {"dpaGateIn"};
char MqttPublishTopic[] = {"dpaGateOut"};

/**
 * Setup peripherals
 */
void setup()
{
    Serial.begin(9600);

    // MQTT Client library initialization
    client.setServer(server, 1883);
    client.setCallback(mqttCallback);
    Ethernet.begin(mac, myIp);
    SPI.end();

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

#elif defined(__UART_INTERFACE__)
    delay(500);

    // TR module power on
    digitalWrite(PWR, LOW);
    // start UART1 peripheral
    Serial1.begin(57600);
#endif

    // initialize DPA library
    dpaInit(myAsyncPacketHandler);

#if defined(__AVR__) || defined(CORE_TEENSY)
    // initialize Timer1, call DPA driver every 200us
    Timer1.initialize(200);
    // attaches callback() as a timer overflow interrupt
    Timer1.attachInterrupt(systemTimerIterruptHandler);
#endif

    // JSON initialization
    jsonInit();

    delay(1000);
    Serial.println(F("Initialization OK"));
}

/**
 * Main loop
 */
void loop()
{
    DPA_OPERATION_RESULT OpResult;
    uint16_t TempValue;

    // release SPI bus
    dpaSuspendDriver();
    // Run MQTT driver
    if (!client.connected())
        mqttReconnect();
    client.loop();
    // occupy SPI bus by DPA driver
    dpaRunDriver();

    // MQTT received data processing
    if (DpaDataReady == true || AsyncPacketReceived == true) {
        // send request and wait for result
        if (AsyncPacketReceived == false) {
            TempValue = jsonGetTimeout();
            if (TempValue == 0)
                TempValue = 1000;
          	while ((OpResult = dpaSendRequest(&MyDpaPacket, jsonGetDataSize(), TempValue)) == DPA_OPERATION_IN_PROGRESS)
                ; /* void */
            if (OpResult == DPA_OPERATION_OK)
                Serial.println(F("DPA operation OK"));
            else
                Serial.println(F("DPA operation ERROR"));
        } else {
            OpResult = DPA_OPERATION_OK;
        }

        AsyncPacketReceived = false;
        DpaDataReady = false;

        // create JSON answer or notification packet
        TempValue = jsonCreate(MyMqttMessage, &MyDpaPacket, OpResult);
        Serial.print(F("Size of JSON message - "));
        Serial.println(TempValue);

        // release SPI bus
        dpaSuspendDriver();
        // send MQTT JSON data
        if (client.connected() && TempValue)
            client.publish(MqttPublishTopic, MyMqttMessage, TempValue);
        // occupy SPI bus by DPA driver
        dpaRunDriver();
    }
}

/**
 * System timer Interrupt handler (200us)
 */
void systemTimerIterruptHandler(void)
{
    dpaLibraryDriver();
}

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
 * Asynchronous packet service routine
 * @param dpaAnswerPkt Pointer to DPA answer packet
 */
void myAsyncPacketHandler(T_DPA_PACKET *dpaAnswerPkt)
{
    memcpy((uint8_t *)&MyDpaPacket, (uint8_t *)dpaAnswerPkt, sizeof(T_DPA_PACKET));
    AsyncPacketReceived = true;
    Serial.println(F("Async packet received"));
}

/**
 * MQTT callback
 * @param Topic MQTT topic
 * @param Payload
 * @param Length
 */
void mqttCallback(char* Topic, byte* Payload, unsigned int Length)
{
    Serial.print(F("Message arrived ["));
    Serial.print(Topic);
    Serial.print("] ");
    for (uint16_t i = 0; i < Length; i++)
        Serial.print((char) Payload[i]);
    Serial.println();
    if (jsonParse(&MyDpaPacket, Payload, Length) == JSON_PARSE_OK) {
        Serial.println(F("Parse OK"));
        DpaDataReady = true;
    } else {
        Serial.println(F("Parse ERROR"));
    }
}

/**
 * Check MQTT connection and reconnect
 */
void mqttReconnect()
{
    Serial.print(F("Attempting MQTT connection..."));
    // Attempt to connect
    if (client.connect("iqrfDpaGate")) {
        Serial.println(F("connected"));
        // Once connected, publish an announcement...
        client.publish(MqttPublishTopic, "DPA gate ready");
        // ... and resubscribe
        client.subscribe(MqttSubscribeTopic);
    } else {
        Serial.print(F("failed, rc="));
        Serial.println(client.state());
        delay(1000);
    }
}
