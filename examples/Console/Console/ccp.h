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
 *
 *  Arduino console command processor
 *
 *********************************************************************
 * FileName:        ccp.h
 * Platform:        Arduino
 * Company:         IQRF Tech s.r.o.
 *********************************************************************
*/

#ifndef _CCP_H
#define _CCP_H

#define SIZE_OF_IN_BUFF   48
#define SIZE_OF_PARAM     16

#define CCP_COMMAND_NOT_FOUND     0
#define CCP_DONE                  1
#define CCP_SD_CARD_ERR           2
#define CCP_UPLOADING             3
#define CCP_BAD_PARAMETER         4
#define CCP_FILE_NOT_FOUND        5
#define CCP_DIR_NOT_FOUND         6
#define CCP_SENDING_REQUEST       7
#define CCP_CONFIRMATION_OK       8
#define CCP_CONFIRMATION_ERR      9
#define CCP_RESPONSE_OK           10
#define CCP_RESPONSE_ERR          11
#define CCP_COMMAND_TIMEOUT       12
#define CCP_CODE_STORED           13
#define CCP_CODE_STORE_ERR        14
#define CCP_TR_NOT_READY          15
#define CCP_CONFIG_FILE_ERR       16

extern char CcpCommandParameter[SIZE_OF_PARAM];
extern const char CrLf[];

extern void ccp(void);
extern uint8_t ccpFindCmdParameter(char *DestinationString);
extern void sysMsgPrinter(uint16_t Msg);

#endif
