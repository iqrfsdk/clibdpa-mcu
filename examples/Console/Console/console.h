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

#ifndef _CONSOLE_H
#define _CONSOLE_H

void ccpCoordinatorCmd (word CommandTabParameter);
void ccpLedCmd (word CommandTabParameter);
void ccpNodeCmd (word CommandTabParameter);
void ccpOsCmd (word CommandTabParameter);
void ccpLoadConfigCmd (word CommandTabParameter);
void ccpDpaStoreCodeCmd (word CommandTabParameter);
void ccpDpaLoadVerifyCodeCmd (word CommandTabParameter);
void ccpDpaCustomHandlerOnOffCmd (word CommandTabParameter);

extern uint8_t   SDCardReady;

#endif
