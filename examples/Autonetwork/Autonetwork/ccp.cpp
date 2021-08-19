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

/*
 *********************************************************************
 *
 *  Arduino console command processor
 *
 *********************************************************************
 * FileName:        ccp.cpp
 * Platform:        Arduino
 * Company:         IQRF Tech s.r.o.
 *********************************************************************
*/

#include <Arduino.h>
#include "ccp.h"
#include "autonetwork.h"
#include "dpa_library.h"

/* data types */
typedef struct {                        // command decode table item structure
    char    Com[16];
    void    (*Func)(word);
    word    Param;
} COM;

/* function prototypes */
void find_command(void);
void memcpy_P (byte *Destination, byte *Source, word Count);
void ccpClsCmd (word CommandParameter);
void ccpCommandNotFound (word CommandParameter);

/* global variables */
const COM Commands[] PROGMEM = {         // command decode table
    "rst", ccpClsCmd, 'H',
    "ledr", ccpLedCmd, PNUM_LEDR,
    "clearbonds", ccpClrBondsCmd, 0,
    "autonetwork", ccpAutonetworkCmd, 0,
};

#define HEADER_LINE_SIZE    75
const char Header[][HEADER_LINE_SIZE] PROGMEM = {
    "--------------------------------------------------------------------------",
    "|                 Arduino console command processor                      |",
    "|                          IQRF Tech s.r.o                               |",
    "--------------------------------------------------------------------------"
};

const char CmdPrompt[] = {"cmd> "};
const char CrLf[] = {0x0D,0x0A,0x00};
const char Back[] = {0x08,0x20,0x08,0x00};

COM CmdWorkCopy;
void (*run_func)(word);                   // pointer to command service function
word Parameter;                           // command service function parameter
byte ComEndPos;                           // end of command position in input buffer
byte CSel;

char CcpCommandParameter[SIZE_OF_PARAM];
char InLine[SIZE_OF_IN_BUFF] = {"rst\0"};   // input buffer
byte InLinePtr = SIZE_OF_IN_BUFF-1;         // input buffer pointer
boolean RepeatInLine = true;
boolean Esc = false;                        // receiver flags
boolean Esc2 = false;

/**
 * console command processor kernel
 */
void ccp(void)
{
    char ConsoleChar;

    if (Serial.available() > 0) {
        ConsoleChar = Serial.read();

        if (Esc2 == true) {                                       // previous chars has been ESC, 0x5B -> ignore received char & clear flags
            Esc = Esc2 = false;
            return;
        }
        if (Esc == true && (ConsoleChar == 0x5B)) {               // previous char has been ESC and received char is 0x5B, ignore next char
            Esc2 = true;
            return;
        }
        if (ConsoleChar == 0x1B) {                                // received char is ESC
            Esc = true;
            return;
        }
        if (ConsoleChar == 0x0A)                                  // ignore LF
            return;

        Esc = false;

        if (ConsoleChar==0x0D) {                                  // Enter pressed
            InLine[InLinePtr]=0;                                  // set end of input string

            if (RepeatInLine)
                Serial.println(InLine);                           // repeat previous command if new don't exist
            else
                Serial.print(CrLf);                               // print new line
            Serial.print(CrLf);

            find_command();                                       // decode entered command
            run_func(Parameter);                                  // execute command service function

            Serial.print(CrLf);                                   // new line
            Serial.print(CmdPrompt);                              // print prompt

            RepeatInLine = true;                                  // repeat command if ENTER is pressed
        } else {
            if (RepeatInLine) {                                   // if first char of new command hes been received
                RepeatInLine = false;
                InLinePtr=0;                                      // set pointer to start of input buffer
            }
            if (ConsoleChar == 0x08) {                            // back space char has been received
                if (InLinePtr) {                                  // if any chars are in input buffer
                    InLinePtr--;                                  // one char back in input buffer
                    Serial.print(Back);                           // send string to clear last char on console
                }
            } else {
                if (InLinePtr<SIZE_OF_IN_BUFF) {                  // input buffer is not full
                    InLine[InLinePtr++] = ConsoleChar;            // write received char to input buffer
                    Serial.write(ConsoleChar);                    // loop back received char to console
                }
            }
        }
    }
}

/**
 * find command in input buffer
 *   - set pointer to command service function
 *   - initialize parameter of command service function
 *   - set position in input buffer to command parameter reading
 */
void find_command(void)
{
    uint8_t x,y;

    for (CSel=0; CSel < (sizeof(Commands)/sizeof(COM));CSel++) {    // compare input buffer with existing command patterns
        memcpy_P(&CmdWorkCopy, &Commands[CSel], sizeof(CmdWorkCopy));
        x=0;                                                        // initialize start position in input buffer
        while (InLine[x] == ' ' && x < SIZE_OF_IN_BUFF)
            x++;                                                    // find start of command
        if (x == SIZE_OF_IN_BUFF)
            goto err_find_out;                                      // if command not exist, decode end
        for (y=0; y<sizeof(CmdWorkCopy.Com); y++) {                 // compare input command with command patterns
            if (CmdWorkCopy.Com[y]==0 && (InLine[x]==' ' || InLine[x]==0)) { // if command exist
                run_func = CmdWorkCopy.Func;                        // initialize pointer to function to run
                Parameter = CmdWorkCopy.Param;                      // initialize function parameter
                ComEndPos = x;                                      // set position in input buffer for command parameter reading
                return;                                             // decode end
            }
            if (InLine[x] != CmdWorkCopy.Com[y])
                break;                                              // compare with next command pattern
            x++;                                                    // compare next char in command
        }
    }

err_find_out:
    run_func = ccpCommandNotFound;                                  // in case of error, run error service function
    ComEndPos = 0;
}

/**
 * find string in ccpInBuff
 * @param DestinationString pointer to string where parameter going to be placed
 * @return size of parameter string
 */
uint8_t ccpFindCmdParameter(char *DestinationString)
{
    uint8_t  TempCnt = 0;
    uint8_t  TempChar;

    InLine[SIZE_OF_IN_BUFF-1] = 0;

    do {
        TempChar = InLine[ComEndPos];
        if (TempChar == ' ')
            ComEndPos++;
    } while (TempChar == ' ');

    if (TempChar == 0) {
        DestinationString[0] = 0;
        return(0);
    }

    do {
        TempChar = InLine[ComEndPos++];
        DestinationString[TempCnt++] = TempChar;
    } while(TempChar!=' ' && TempChar!=0 && ComEndPos<SIZE_OF_IN_BUFF && TempCnt<SIZE_OF_PARAM);

    DestinationString[--TempCnt] = 0;
    return(TempCnt);
}

/**
 * copy data from FLASH to RAM
 * @param Destination pointer to destination position in RAM
 * @param Source pointer to source position in FLASH
 * @param Count number of data bytes to be copied
 * @return none
 */
void memcpy_P (uint8_t *Destination, uint8_t *Source, uint16_t Count)
{
    while (Count--)
        *Destination = pgm_read_byte_near(Source++);
}

/**
 * clear screen command service
 * @param CommandParameter parameter from CCP command table
 * @return none
 */
void ccpClsCmd (uint16_t CommandParameter)
{
    uint8_t x,y;
    char HeaderChar;

    Serial.print(CrLf);
    Serial.print(CrLf);

    if (CommandParameter == 'H') {
        for (y=0; y<sizeof(Header)/HEADER_LINE_SIZE; y++) {
            for (x=0; x<HEADER_LINE_SIZE; x++) {
                HeaderChar = pgm_read_byte_near(&Header[y][x]);
                Serial.write(HeaderChar);
            }
            Serial.print(CrLf);
        }
    }
}

/**
 * print message "Command not found" in case of bad command
 * @param CommandParameter parameter from CCP command table
 * @return none
 */
void ccpCommandNotFound (uint16_t CommandParameter)
{
    Serial.println("Command not found");
}
