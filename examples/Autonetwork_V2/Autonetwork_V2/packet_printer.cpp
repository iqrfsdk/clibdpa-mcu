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

/*
 *********************************************************************
 * FileName:        packet_printer.cpp
 * Platform:        Arduino
 * Company:         IQRF Tech s.r.o.
 *********************************************************************
*/

#include <Arduino.h>
#include "dpa_library.h"

#if defined (__DPA_DEVELOPER_MODE__)

extern "C" void packetPrinter(uint8_t Message, uint8_t *Buffer, uint8_t DataSize);

const char DebugMsg[5][26] PROGMEM = {       // debug messages
    "Data to send             ",   /* 0 */
    "Confirmation OK          ",   /* 1 */
    "Confirmation Error       ",   /* 2 */
    "Response OK              ",   /* 3 */
    "Response Error           ",   /* 4 */
};

/**
 * print data from DPA communication packets
 * @param Message number of desired description message
 * @param Buffer pointer to buffer with DPA data
 * @param DataSize number of DPA data bytes in buffer
 * @return none
 */
void packetPrinter(uint8_t Message, uint8_t *Buffer, uint8_t DataSize)
{
    char MessageStr[26];

    Serial.println();
    strcpy_P(MessageStr, &DebugMsg[Message][0]);
    Serial.println(MessageStr);

    // if "Data to send" are printed, 2 bytes after DPA header are skipped
    // DataSize must be increased by 2
    if (Message == 0)
        DataSize += 2;

    for (uint8_t Cnt=0; Cnt<DataSize; Cnt++) {
        Serial.print(Buffer[Cnt], HEX);
        Serial.print(" ");
        // if "Data to send" are printed, 2 bytes after DPA header are skipped
        if (Message==0 && Cnt==5)
            Cnt += 2;
    }
    Serial.println();
}

#endif
