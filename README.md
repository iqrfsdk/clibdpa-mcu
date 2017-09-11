# cLibDPA-MCU

[![Build Status](https://travis-ci.org/iqrfsdk/clibdpa-mcu.svg?branch=develop)](https://travis-ci.org/iqrfsdk/clibdpa-mcu)
[![Apache License](https://img.shields.io/badge/license-APACHE2-blue.svg)](https://github.com/iqrfsdk/clibdpa-mcu/blob/master/LICENSE)

The cLibDPA-MCU library intended usage is to provide programming language means for communication between MCU and TR module, equipped with the firmware supporting the DPA framework.

The library implementation is based on encapsulation of DPA commands, sent from MCU to TR module, into programming language's functions. More detailed information about DPA framework and commands are in document ["IQRF DPA framework, Technical Guide"](http://www.iqrf.org/support/download&kat=54&ids=481).

These boards [1](http://iqrf.org/weben/index.php?sekce=products&id=iqrf-bb-01&ot=development-tools&ot2=development-kits), [2](http://iqrf.org/weben/index.php?sekce=products&id=iqrf-bb-02&ot=development-tools&ot2=development-kits) can be used to interface DCTR modules to a control unit.

## Features

-   intended for communication with DCTR-7xD modules
-   intended for both DPA coordinator and node
-   supported DPA frameworks: 3.x
-   supported communication interfaces: SPI, UART
-   supported programming languages: C for MCU
-   lightweight and easy to use
-   well documented with examples for SPI and UART

## Installation

The best way how to install this library is to [download a latest package](https://github.com/iqrfsdk/clibdpa-mcu/releases) or use a [platformio](http://platformio.org/lib/show/1169/IQRF%20DPA):

```bash
platformio lib install "cLibDPA-MCU"
```


## Compatibility table

| MCU                     | Works  | Boards                         |
| ----------------------- | :----: | ------------------------------ |
| Atmega168 @ 16MHz       |    ✓   | Arduino Duemilanove, Decimalia |
| Atmega328 @ 16MHz       |    ✓   | Arduino Uno, Nano, Mini        |
| Atmega32u4 @ 16MHz      |    ✓   | Arduino Leonardo, Micro        |
| Atmega2560 @ 16MHz      |    ✓   | Arduino Mega 2560              |
| ATSAM3X8E @ 84MHz       |    ✓   | Arduino Due                    |
| PIC32MX320F128H @ 80MHz |    ✓   | chipKIT Uno32                  |
| PIC32MX340F512H @ 80MHz |    ✓   | chipKIT uC32                   |
| Atmega32u4 @ 16MHz      |    ✓   | Teensy 2.0                     |
| AT90USB1286 @ 16MHZ     |    ✓   | Teensy 2.0++                   |
| MK20DX256 @ 72MHZ       |    ✓   | Teensy 3.1 / 3.2               |

## Integration

The pointer to struct ```T_DPA_PACKET``` is used for communication between user's application and the library. The definition of ```T_DPA_PACKET``` can be found in the file [```dpa_library.h```](https://github.com/iqrfsdk/clibdpa-mcu/blob/develop/src/dpa_library.h). If the user wishes to use the services of the library the files [```dpa_library.c```](https://github.com/iqrfsdk/clibdpa-mcu/blob/develop/src/dpa_library.c), [```dpa_library.h```](https://github.com/iqrfsdk/clibdpa-mcu/blob/develop/src/dpa_library.h) and [```dpa.h```](https://github.com/iqrfsdk/clibdpa-mcu/blob/develop/src/DPA.h) must be included in user's project and following conditions must be met.

-   select the communication interface in the dpa_library.h header file

| Bus  | Macro                            |
| :--: | -------------------------------- |
| SPI  | ```#define __SPI_INTERFACE__```  |
| UART | ```#define __UART_INTERFACE__``` |

-   enable or disable library extension for uploading new custom DPA handler of firmware to DCTR-7xD modules

| Extension   | Macro                                |
| :---------: | ------------------------------------ |
|  STORE CODE | ```#define __STORE_CODE_SUPPORT__``` |

-   implement functions to transfer of 1B to TR module via selected communication interface and deselect module if using SPI interface

| Bus  | Function                                          |
| :--: | ------------------------------------------------- |
| SPI  | ```uint8_t dpaSendSpiByte(uint8_t TxByte)```      |
| SPI  | ```void dpaDeselectTRmodule(void)```              |
| UART | ```void dpaSendUartByte(uint8_t TxByte)```        |
| UART | ```uint8_t dpaReceiveUartByte(uint8_t *RxByte)``` |

-   in case of using STORE CODE extension, implement function to read 1B from selected open file on storage media. Inside this function, user should call ```dpaIncFileByteCounter()``` macro, whenever a byte is read from the file.

| Extension   | Function                                |
| :---------: | --------------------------------------- |
|  STORE CODE | ```uint8_t dpaReadByteFromFile(void)``` |

-   call the function ```void dpaLibraryDriver(void)``` with 150us period. It is recommended to call the function in the interrupt.
-   initialize the library by calling following function first:
```c
void dpaInit(T_DPA_ANSWER_HANDLER dpaAnswerHandler); // dpaAnswerHandler is user's function which is called by the library after asynchronous packet reception from DPA framework
```

## API functions
-   ```void dpaInit(T_DPA_ANSWER_HANDLER dpaAnswerHandler)``` - ```dpaAnswerHandler``` is user's function which is called by the library after asynchronous packet reception from DPA framework
-   ```void dpaLibraryDriver(void)``` - The brief description of this function is in the paragraph Integration.
-   ```uint8_t dpaSendRequest(T_DPA_PACKET *DpaRequest, uint8_t DataSize, uint16_t Timeout)``` - The function sends DPA request to TR module via selected interface. The user fills the ```T_DPA_PACKET``` struct, defines size of additional data in the DPA request (if any) and Timeout of opperation in ms. By additional data are meant bytes which follows after DPA request header NAdr, PNum, PCmd and HwProfile. Some DPA requests require the additional data. The function must be called periodically, if returns code ```DPA_OPERATION_IN_PROGRESS```. Periodically function calling is necessary end, when returns one of the following return codes:
    -   ```DPA_OPERATION_OK``` - operation OK, answer to our request is in ```T_DPA_PACKET``` struct
    -   ```DPA_OPERATION_TIMEOUT```  - operation timeout
    -   ```DPA_CONFIRMATION_ERR```  - operation ERROR, coordinator doesn't confirm our request
    -   ```DPA_RESPONSE_ERR```  - operation ERROR, TR module returned an unexpected response
    -   ```DPA_TR_MODULE_NOT_READY```  - operation ERROR, TR module is not ready    

-   ```uint8_t dpaMakeConfigurationCRC(T_DPA_PACKET *DpaRequest)``` - Function calculates the CRC of the new configuration data for TR module
-   ```void dpaIncFileByteCounter(void)``` - The brief description of this function is in the paragraph Integration.
-   ```uint8_t dpaGetRxExtraDataSize(void)``` - Function returns size of additional data block in DPA response packet, received from TR module.
-   ```uint8_t dpaStoreCodeToEeeprom(T_DPA_CODE_FILE_INFO *CodeFileInfo)``` - Function ensures the storing of code image from IQRF or HEX file with the new custom DPA handler, or firmware for DCTR-7xD module. The user fills the ```T_DPA_CODE_FILE_INFO``` struct, with the information necessary to store code image. Then the function must be called periodically, until returns ```DPA_STORE_CODE_SUCCESS```, or ```DPA_STORE_CODE_ERROR```.

## License
This library is licensed under Apache License 2.0:

 > Copyright 2015-2017 IQRF Tech s.r.o.
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
