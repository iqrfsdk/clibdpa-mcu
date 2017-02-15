
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

/*
 *********************************************************************
 *
 *  Arduino console command processor
 *
 *********************************************************************
 * FileName:        ccp.cpp
 * Platform:        Arduino
 * Company:         Microrisc s.r.o.
 *********************************************************************
*/

#include <Arduino.h>
#include <SD.h>
#include "ccp.h"
#include "console.h"
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
void ccpLsCmd (word CommandParameter);

/* global variables */
const COM Commands[] PROGMEM ={          // command decode table
  "rst",ccpClsCmd,'H',
  "ls",ccpLsCmd,0,
  "bond",ccpCoordinatorCmd,0,
  "unbondc",ccpCoordinatorCmd,CMD_COORDINATOR_REMOVE_BOND,
  "rebondc",ccpCoordinatorCmd,CMD_COORDINATOR_REBOND_NODE,
  "discovery",ccpCoordinatorCmd,CMD_COORDINATOR_DISCOVERY,
  "ledr",ccpLedCmd,PNUM_LEDR,
  "ledg",ccpLedCmd,PNUM_LEDG,
  "unbondn",ccpNodeCmd,CMD_NODE_REMOVE_BOND,
  "osreset",ccpOsCmd,CMD_OS_RESET,
  "osrestart",ccpOsCmd,CMD_OS_RESTART,
  "osinfo",ccpOsCmd,CMD_OS_READ,
  "loadcfg",ccpLoadConfigCmd,0,

  #if defined (__STORE_CODE_SUPPORT__)
    "storecode",ccpDpaStoreCodeCmd,0,
    "verifycode",ccpDpaLoadVerifyCodeCmd,0,
    "loadcode",ccpDpaLoadVerifyCodeCmd,1,
    "customhandler",ccpDpaCustomHandlerOnOffCmd,0,
  #endif
};

#define HEADER_LINE_SIZE    75
const char Header[][HEADER_LINE_SIZE] PROGMEM ={
  "--------------------------------------------------------------------------",
  "|                 Arduino console command processor                      |",
  "|                          Microrisc s.r.o                               |",
  "--------------------------------------------------------------------------"
};

const char SystemMsg[17][26] PROGMEM ={        // error messages
  "Command not found        ",   /* 0 */
  "Done !!!                 ",   /* 1 */
  "SD card operation ERROR  ",   /* 2 */
  "Uploading ....           ",   /* 3 */
  "Bad command parameter    ",   /* 4 */
  "File not found           ",   /* 5 */
  "Directory not found      ",   /* 6 */
  "Sending request          ",   /* 7 */
  "Confirmation OK          ",   /* 8 */
  "Confirmation ERROR       ",   /* 9 */
  "Response OK              ",   /* 10 */
  "Response ERROR           ",   /* 11 */
  "Command timeout          ",   /* 12 */
  "Code stored successfully ",   /* 13 */
  "Code store ERROR         ",   /* 14 */
  "TR module not ready      ",   /* 15 */
  "Configuration file ERROR ",   /* 16 */
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

/*
*------------------------------------------------------------
*              console command processor kernel
*------------------------------------------------------------
*/
void ccp(void) {

  char ConsoleChar;

  if (Serial.available() > 0){
    ConsoleChar = Serial.read();

    if (Esc2 == true) {Esc = Esc2 = false; return;}           // previous chars has been ESC, 0x5B -> ignore received char & clear flags
    if (Esc == true && (ConsoleChar == 0x5B)) {Esc2 = true; return;}   // previous char has been ESC and received char is 0x5B, ignore next char
    if (ConsoleChar == 0x1B) {Esc = true; return;}            // received char is ESC
    if (ConsoleChar == 0x0A) return;                          // ignore LF

    Esc = false;

    if (ConsoleChar==0x0D){                                   // Enter pressed
      InLine[InLinePtr]=0;                                    // set end of input string

      if (RepeatInLine) Serial.println(InLine);               // repeat previous command if new don't exist
      else Serial.print(CrLf);                                // print new line
      Serial.print(CrLf);

      find_command();                                         // decode entered command
      run_func(Parameter);                                    // execute command service function

      Serial.print(CrLf);                                     // new line
      Serial.print(CmdPrompt);                                // print prompt

      RepeatInLine = true;                                    // repeat command if ENTER is pressed
    }
    else{
      if (RepeatInLine){                                      // if first char of new command hes been received
        RepeatInLine = false;
        InLinePtr=0;                                          // set pointer to start of inpit buffer
      }
      if (ConsoleChar == 0x08){                               // back space char has been received
        if (InLinePtr){                                       // if any chars are in input buffer
          InLinePtr--;                                        // one char back in input buffer
          Serial.print(Back);                                 // send string to clear last char on console
        }
      }
      else{
        if (InLinePtr<SIZE_OF_IN_BUFF){                       // input buffer is not full
          InLine[InLinePtr++] = ConsoleChar;                  // write received char to input buffer
          Serial.write(ConsoleChar);                          // loop back received char to console
        }
      }
    }
  }
}

/*
*-----------------------------------------------------------------------
*                      find command in input buffer
*   - set pointer to command service function
*   - initialize parameter of command service function
*   - set position in input buffer for command parameter reading
*-----------------------------------------------------------------------
*/
void find_command(void){  /* find command in input buffer */
  byte x,y;

  for (CSel=0; CSel < (sizeof(Commands)/sizeof(COM));CSel++){    // compare input buffer with existing command patterns
    memcpy_P(&CmdWorkCopy, &Commands[CSel], sizeof(CmdWorkCopy));
    x=0;                                                        // initialize start position in input buffer
    while (InLine[x] == ' ' && x < SIZE_OF_IN_BUFF) x++;        // find start of command
    if (x==SIZE_OF_IN_BUFF) goto err_find_out;                  // if command not exist, decode end
    for (y=0;y<sizeof(CmdWorkCopy.Com);y++){                    // compare input command with command patterns
      if (CmdWorkCopy.Com[y]==0 && (InLine[x]==' ' || InLine[x]==0)){ // if command exist
        run_func=CmdWorkCopy.Func;                              // initialize pointer to function to run
        Parameter=CmdWorkCopy.Param;                            // initialize function parameter
        ComEndPos=x;                                            // set position in input buffer for command parameter reading
        return;                                                 // decode end
      }
      if (InLine[x] != CmdWorkCopy.Com[y]) break;               // compare with next command pattern
      x++;                                                      // compare next char in command
    }
  }

  err_find_out:
  run_func=sysMsgPrinter;                                       // in case of error, run error service function
  Parameter=CCP_COMMAND_NOT_FOUND;
  ComEndPos=0;
}

/*
*------------------------------------------------------------
*                  find string in ccpInBuff
*------------------------------------------------------------
*/
byte ccpFindCmdParameter(char *DestinationString){

  byte  TempCnt = 0;
  byte  TempChar;

  InLine[SIZE_OF_IN_BUFF-1] = 0;

  do{
    TempChar = InLine[ComEndPos];
    if (TempChar==' ') ComEndPos++;
  }while (TempChar==' ');

  if (TempChar == 0){
    DestinationString[0] = 0;
    return(0);
  }

  do{
    TempChar = InLine[ComEndPos++];
    DestinationString[TempCnt++] = TempChar;
  }while(TempChar!=' ' && TempChar!=0 && ComEndPos<SIZE_OF_IN_BUFF && TempCnt<SIZE_OF_PARAM);

  DestinationString[--TempCnt] = 0;
  return(TempCnt);
}

/*
*------------------------------------------------------------
*               copy data from FLASH to RAM
*------------------------------------------------------------
*/
void memcpy_P (byte *Destination, byte *Source, word Count){
  while (Count--){
    *Destination = pgm_read_byte_near(Source++);
  }
}

/*
*------------------------------------------------------------
*                  system message printing
*------------------------------------------------------------
*/
void sysMsgPrinter(word Msg){
  char Message[26];

  strcpy_P(Message, &SystemMsg[Msg][0]);
  Serial.println(Message);
}

/*
*------------------------------------------------------------
*              clear screen command service
*------------------------------------------------------------
*/
void ccpClsCmd (word CommandParameter){
  byte x,y;
  char HeaderChar;

  Serial.print(CrLf);
  Serial.print(CrLf);

  if (CommandParameter == 'H'){
    for (y=0; y<sizeof(Header)/HEADER_LINE_SIZE; y++){
      for (x=0; x<HEADER_LINE_SIZE; x++){
        HeaderChar =  pgm_read_byte_near(&Header[y][x]);
        Serial.write(HeaderChar);
      }
      Serial.print(CrLf);
    }
  }
}

/*
*------------------------------------------------------------
*              root SD card file list
*------------------------------------------------------------
*/
void ccpLsCmd (word CommandParameter){
  File root;

  dpaSuspendDriver();                                                     // suspend SPI comunication with TR module

  if (!SDCardReady) sysMsgPrinter(CCP_SD_CARD_ERR);                       // if SD card is not ready print error msg
  else{
    root = SD.open("/");                                                  // open root directory
    root.rewindDirectory();                                               // set pointer to start of directory
    while (true){
      File entry =  root.openNextFile();                                  // open next file in root directory
      if (! entry) break;                                                 // no more files exist -> end
      Serial.print(entry.name());                                         // print
      if (entry.isDirectory()) {
        Serial.println("/");
      } else {
        Serial.print("\t\t");
        Serial.println(entry.size(), DEC);
      }
      entry.close();
    }
    root.close();
  }

  dpaRunDriver();                                                          // start SPI comunication with TR modul
}
