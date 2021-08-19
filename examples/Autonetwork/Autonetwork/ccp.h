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
 * FileName:        ccp.h
 * Platform:        Arduino
 * Company:         IQRF Tech s.r.o.
 *********************************************************************
*/

#ifndef _CCP_H
#define _CCP_H

#define SIZE_OF_IN_BUFF		48
#define SIZE_OF_PARAM 16

extern char CcpCommandParameter[SIZE_OF_PARAM];

extern void ccp(void);
extern uint8_t ccpFindCmdParameter(char *DestinationString);

#endif
