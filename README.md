# cLibDPA-MCU

[![Build Status](https://travis-ci.org/iqrfsdk/clibdpa-mcu.svg?branch=develop)](https://travis-ci.org/iqrfsdk/clibdpa-mcu)
[![Apache License](https://img.shields.io/badge/license-APACHE2-blue.svg)](https://github.com/iqrfsdk/clibdpa-mcu/blob/master/LICENSE)

The cLibDPA-MCU library intended usage is to provide programming language means for communication between MCU and TR module, equipped with the firmware supporting the DPA framework.
  
The library implementation is based on encapsulation of DPA commands, sent from MCU to TR module, into programming language's functions. More detailed information about DPA framework and commands are in document ["IQRF DPA framework, Technical Guide"](http://www.iqrf.org/support/download&kat=54&ids=481).

These boards [1](http://iqrf.org/weben/index.php?sekce=products&id=iqrf-bb-01&ot=development-tools&ot2=development-kits), [2](http://iqrf.org/weben/index.php?sekce=products&id=iqrf-bb-02&ot=development-tools&ot2=development-kits) can be used to interface DCTR modules to a control unit.

## Features

- intended for communication with DCTR-72D and DCTR-52D modules
- intended for both DPA coordinator and node
- supported DPA frameworks: 2.x
- supported communication interfaces: SPI, UART
- supported programming languages: C for MCU
- lightweight and easy to use
- well documented with examples for SPI and UART

## Installation

The best way how to install this library is to [download a latest package](https://github.com/iqrfsdk/clibdpa-mcu/releases) or use a [platformio](http://platformio.org/lib/show/1169/IQRF%20DPA):

```bash
platformio lib install "IQRF DPA"
```

## Compatibility table

|         MCU        |  Works |            Boards            |
| ------------------ | :----: | ---------------------------- |
| Atmega328 @ 16MHz  |    ✓   | Arduino Uno, Nano            |
| Atmega32u4 @ 16MHz |    ✓   | Arduino Leonardo, Micro      |
| Atmega2560 @ 16MHz |    ✓   | Arduino Mega 2560            |
| ATSAM3X8E          |    ✓   | Arduino Due                  |
| PIC32MX320F128H    |    ✓   | chipKIT Uno32                |
| PIC32MX340F512H    |    ✓   | chipKIT uC32                 |
| MK66FX1M0VMD18     |    ✓   | Teensy 3.6                   |

## Integration

The pointer to struct ```T_DPA_PACKET``` is used for communication between user's application and the library. The definition of ```T_DPA_PACKET``` can be found in the file [```dpa_library.h```](https://github.com/iqrfsdk/clibdpa-mcu/blob/develop/src/dpa_library.h). If the user wishes to use the services of the library the files [```dpa_library.c```](https://github.com/iqrfsdk/clibdpa-mcu/blob/develop/src/dpa_library.c) and [```dpa_library.h```](https://github.com/iqrfsdk/clibdpa-mcu/blob/develop/src/dpa_library.h) must be included in user's project and following conditions must be met.

- select the communication interface in the library header file

| Bus  |              Macro               |
| :--: | -------------------------------- |
| SPI  | ```#define __SPI_INTERFACE__```  |
| UART | ```#define __UART_INTERFACE__``` |

- select module type

|  Module  |        Macro       |
| :------: | ------------------ |
| DCTR-52D | ```#define TR5xD```|
| DCTR-72D | ```#define TR7xD```|

- implement functions to transfer of 1B to TR module via selected communication interface and deselect module if using SPI interface

| Bus  |                    Function                    |
| :--: | ---------------------------------------------- |
| SPI  | ```uint8_t DPA_SendSpiByte(uint8_t txByte)```  |
| SPI  | ```void DPA_DeselectTRmodule(void)```          |
| UART | ```void DPA_SendUartByte(uint8_t txByte)```    |
| UART | ```void DPA_ReceiveUartByte(uint8_t rxByte)``` |	

- call the function void DPA_SetTimmingFlag(void) with 1ms period. It is recommended to call the function in the interrupt.
- call the function void DPA_LibraryDriver(void) in the main loop of the user's application. The calling period is not strictly defined. If the function is called in the interrupt then it is necessary to bear in mind that the function takes time at least of 1B transfer via selected interface.
- initialize the library by calling following functions first:
```c
void DPA_Init(void);
void DPA_SetAnswerHandler(T_DPA_ANSWER_HANDLER dpaAnswerHandler); // dpaAnswerHandler is user's function which is called by the library after packet reception from DPA framework.
```

## API functions
- ```void DPA_Init(void)```
- ```DPA_SetAnswerHandler(T_DPA_ANSWER_HANDLER dpaAnswerHandler)```
- ```DPA_SetTimmingFlag(void)```
- ```void DPA_LibraryDriver(void)``` - The brief description of these functions is in the paragraph Integration.
- ```void DPA_SendRequest(T_DPA_PACKET *dpaRequest, uint8_t dataSize)``` - The function sends DPA request to TR module via selected interface. The user fills the ```T_DPA_PACKET``` struct and defines size of additional data (if any) in the DPA request. By additional data are meant bytes which follows after DPA request header NAdr, PNum, PCmd and HwProfile. Some DPA requests require the additional data.
- ```uint16_t DPA_GetEstimatedTimeout(void)``` - The function returns estimated time for delivery of DPA response for sent DPA request. The timeout is calculated from the bytes in DPA confirmation and is in miliseconds.
- ```DPA_GetStatus()``` The function returns a state in which the library is found. It is recommended to call this macro before calling DPA_SendRequest(...).
    - ```DPA_READY``` - the library is ready for new requests
    - ```DPA_BUSY```  - the library is processing the request 

## License
This library is licensed under Apache License 2.0:

 > Copyright 2015-2016 MICRORISC s.r.o.
 > 
 > Licensed under the Apache License, Version 2.0 (the "License");
 > you may not use this file except in compliance with the License.
 > You may obtain a copy of the License at
 > 
 >     http://www.apache.org/licenses/LICENSE-2.0
 > 
 > Unless required by applicable law or agreed to in writing, software
 > distributed under the License is distributed on an "AS IS" BASIS,
 > WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 > See the License for the specific language governing permissions and
 > limitations under the License.
