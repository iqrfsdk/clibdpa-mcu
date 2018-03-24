/**
 * @file DPA support library (JSON support)
 * @version 1.3.1
 *
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

#include <Arduino.h>
#include <ctype.h>
#include <stdio.h>
#include "dpa_json.h"


/* Data types */
typedef struct {
  uint8_t Type;
  uint8_t MsgIdSize;
  uint16_t Timeout;
  uint8_t Rdata;
  uint8_t Request;
  uint8_t RequestSize;
  uint8_t RequestTs;
  uint8_t Confirmation;
  uint8_t ConfirmationTs;
  uint8_t Response;
  uint8_t ResponseTs;
  uint16_t DataSize;
} T_JSON_CONTROL;

enum{
  JSON_TYPE_NOT_DEFINED = 0,
  JSON_TYPE_RAW,
  JSON_TYPE_RAW_HDP
};

/* Function prototypes */
PARSE_RESULT jsonCheckFormat(uint8_t *DataBuffer, uint8_t DataBufferSize);
uint8_t jsonFindString(char *FindingString, uint8_t *DataBuffer, uint8_t DataBufferSize);
uint16_t jsonReadValue(char *FindingString, uint8_t *DataBuffer, uint8_t DataBufferSize, PARSE_RESULT *ReadingStatus);
uint16_t jsonReadHexaField(uint8_t *DataBuffer, uint8_t *JsonBuffer, uint16_t JsonBufferSize);
uint16_t jsonWriteHexaField(uint8_t *JsonBuffer, uint8_t *DataBuffer, uint8_t DataSize);
void jsonWriteAsciiByte(uint8_t *JsonBuffer, uint8_t DataByte);

/* Public variable declarations */
T_JSON_CONTROL	JsonControl;
uint8_t	JsonMsgIdBuff[JSON_MSG_ID_BUFF_SIZE];
uint8_t JsonRequestBuff[JSON_REQUEST_BUFF_SIZE];
char JsonFindStringBuffer[20];

const char JsonCtypeStr[] PROGMEM = {"\"ctype\":"};
const char JsonCtypeDpaStr[] PROGMEM = {"\"dpa\""};
const char JsonTypeStr[] PROGMEM = {"\"type\":"};
const char JsonTypeRawStr[] PROGMEM = {"\"raw\""};
const char JsonTypeRawHdpStr[] PROGMEM = {"\"raw-hdp\""};
const char JsonMsgIdStr[] PROGMEM = {"\"msgid\":"};
const char JsonTimeoutStr[] PROGMEM = {"\"timeout\":"};
const char JsonNadrStr[] PROGMEM = {"\"nadr\":"};
const char JsonPnumStr[] PROGMEM = {"\"pnum\":"};
const char JsonPcmdStr[] PROGMEM = {"\"pcmd\":"};
const char JsonHwpidStr[] PROGMEM = {"\"hwpid\":"};
const char JsonRdataStr[] PROGMEM = {"\"rdata\":"};
const char JsonRcodeStr[] PROGMEM = {"\"rcode\":"};
const char JsonDpavalStr[] PROGMEM = {"\"dpaval\":"};
const char JsonRequestStr[] PROGMEM = {"\"request\":"};
const char JsonRequestTsStr[] PROGMEM = {"\"request_ts\":"};
const char JsonConfirmationStr[] PROGMEM = {"\"confirmation\":"};
const char JsonConfirmationTsStr[] PROGMEM = {"\"confirmation_ts\":"};
const char JsonResponseStr[] PROGMEM = {"\"response\":"};
const char JsonResponseTsStr[] PROGMEM = {"\"response_ts\":"};
const char JsonStatusStr[] PROGMEM = {"\"status\":"};
const char JsonStatusNoErrStr[] PROGMEM = {"\"STATUS_NO_ERROR\"}"};
const char JsonStatusTimeoutStr[] PROGMEM = {"\"ERROR_TIMEOUT\"}"};
const char JsonStatusErrFailStr[] PROGMEM = {"\"ERROR_FAIL\"}"};
const char JsonPacketStartStr[] PROGMEM = {"{\"ctype\":\"dpa\",\"type\":"};

/**
 * Initialize support of JSON format in DPA library
 */
void jsonInit(void)
{
  memset((uint8_t *)&JsonControl, 0, sizeof(JsonControl));
  JsonControl.Type = JSON_TYPE_NOT_DEFINED;
}

/**
 * Check the basic format of MQTT JSON message (mandatory parameters)
 * @param DataBuffer pointer to buffer, with received MQTT JSON message
 * @param DataBufferSize size of MQTT JSON message
 * @return operation result (JSON_PARSE_OK or JSON_PARSE_ERROR)
 */
PARSE_RESULT jsonCheckFormat(uint8_t *DataBuffer, uint8_t DataBufferSize)
{
  uint8_t Index;
  uint8_t LeftBracketCnt, RightBracketCnt;
  uint8_t TempData;

  LeftBracketCnt = RightBracketCnt = 0;

  for (Index=0; Index<DataBufferSize; Index++){
    TempData = DataBuffer[Index];
    if (TempData=='{') LeftBracketCnt++;
    if (TempData=='}') RightBracketCnt++;
  }

  if (LeftBracketCnt==0 || RightBracketCnt==0 || LeftBracketCnt!=RightBracketCnt) return(JSON_PARSE_ERROR);

  strcpy_P(JsonFindStringBuffer, JsonCtypeStr);
  if (jsonFindString(JsonFindStringBuffer, DataBuffer, DataBufferSize) == 0) return(JSON_PARSE_ERROR);
  strcpy_P(JsonFindStringBuffer, JsonCtypeDpaStr);
  if (jsonFindString(JsonFindStringBuffer, DataBuffer, DataBufferSize) == 0) return(JSON_PARSE_ERROR);
  strcpy_P(JsonFindStringBuffer, JsonTypeStr);
  if (jsonFindString(JsonFindStringBuffer, DataBuffer, DataBufferSize) == 0) return(JSON_PARSE_ERROR);

  strcpy_P(JsonFindStringBuffer, JsonTypeRawStr);
  if (jsonFindString(JsonFindStringBuffer, DataBuffer, DataBufferSize) == 0){
    strcpy_P(JsonFindStringBuffer, JsonTypeRawHdpStr);
    if (jsonFindString(JsonFindStringBuffer, DataBuffer, DataBufferSize) == 0) return(JSON_PARSE_ERROR);
    else JsonControl.Type = JSON_TYPE_RAW_HDP;
  }

  return(JSON_PARSE_OK);
}

/**
 * Find specific string in MQTT JSON message
 * @param FindingString pointer to buffer, with string what we looking for
 * @param DataBuffer pointer to buffer, with received MQTT JSON message
 * @param DataBufferSize size of MQTT JSON message
 * @return 0 = string not found, xxx = index to DataBuffer, pointing to last char of finding string
 */
uint8_t jsonFindString(char *FindingString, uint8_t *DataBuffer, uint8_t DataBufferSize)
{
  uint8_t Index, CntS;
  uint8_t Len;

  Len = strlen(FindingString);
  CntS = 0;

  for (Index=0; Index<DataBufferSize; Index++){
    if (FindingString[CntS] == DataBuffer[Index]){
      if (++CntS == Len) return(Index);
    }
    else CntS = 0;
  }
  return(0);
}

/**
 * Read a specific value from the JSON message
 * @param FindingString pointer to buffer, with name of value what we looking for
 * @param DataBuffer pointer to buffer, with received MQTT JSON message
 * @param DataBufferSize size of MQTT JSON message
 * @param ReadingStatus pointer to variable with status of operation (JSON_PARSE_ERROR = variable not found)
 * @return content of value
 */
uint16_t jsonReadValue(char *FindingString, uint8_t *DataBuffer, uint8_t DataBufferSize, PARSE_RESULT *ReadingStatus)
{
  uint16_t Result = 0;
  uint16_t Index;
  char *TempStr;

  Index = jsonFindString(FindingString, DataBuffer, DataBufferSize);
  if (Index){
    while(DataBuffer[Index] != '"' && Index<DataBufferSize) Index++;
    Index++;
    if (DataBuffer[Index] == '"') *ReadingStatus = JSON_PARSE_ERROR;
    Result = strtol((char *)&DataBuffer[Index], &TempStr, 16);
  }
  else *ReadingStatus = JSON_PARSE_ERROR;

  return(Result);
}

/**
 * Read a field of hexadecimal values from the JSON message
 * @param DataBuffer pointer to buffer to store the read data
 * @param JsonBuffer pointer to specific position in buffer, with received MQTT JSON message
 * @param DataBufferSize size of MQTT JSON message
 * @param ReadingStatus pointer to variable with status of operation (JSON_PARSE_ERROR = variable not found)
 * @return number of readed values
 */
uint16_t jsonReadHexaField(uint8_t *DataBuffer, uint8_t *JsonBuffer, uint16_t JsonBufferSize)
{
  uint16_t Index, Cnt;

  Index = Cnt = 0;
  while (JsonBuffer[Index]!='"' && Index<JsonBufferSize) Index++;
  if (JsonBuffer[Index+1] == '"') return(0);
  do{
    Index++;
    DataBuffer[Cnt++] = dpaConvertToNum(JsonBuffer[Index], JsonBuffer[Index+1]);
    Index += 2;
  }while (JsonBuffer[Index]!='"' && ((DataBuffer+Cnt)<(JsonRequestBuff+sizeof(JsonRequestBuff))) && Index<JsonBufferSize);
  return(Cnt);
}

/**
 * Write a field of hexadecimal values to the creating JSON message
 * @param JsonBuffer pointer to specific position in buffer, with creating MQTT JSON message
 * @param DataBuffer pointer to buffer with data we want to write
 * @param DataSize number of values to write
 * @return number of ASCII chars written to creating JSON message
 */
uint16_t jsonWriteHexaField(uint8_t *JsonBuffer, uint8_t *DataBuffer, uint8_t DataSize)
{
  uint16_t  Cnt = 0;

  JsonBuffer[Cnt++] = '"';
  while (DataSize){
    jsonWriteAsciiByte((uint8_t *)&JsonBuffer[Cnt], *DataBuffer);
    DataBuffer++;
    Cnt += 3;
    if (DataSize == 1) Cnt--;
    DataSize--;
  }
  JsonBuffer[Cnt++] = '"';
  JsonBuffer[Cnt++] = ',';
  return(Cnt);
}

/**
 * Write a hexadecimal values to the creating JSON message
 * @param JsonBuffer pointer to specific position in buffer, with creating MQTT JSON message
 * @param DataByte value we want to write
 */
void jsonWriteAsciiByte(uint8_t *JsonBuffer, uint8_t DataByte)
{
  char TempBuff[4];
  sprintf(TempBuff, "%02X.", DataByte);
  strcpy((char *)JsonBuffer, TempBuff);
}

/**
 * Parse DPA request packet from MQTT JSON message
 * @param DpaPacket pointer to DPA request packet, filled with data from received MQTT JSON message
 * @param DataBuffer pointer to buffer, with received MQTT JSON message
 * @param DataBufferSize size of MQTT JSON message
 * @return operation result (JSON_PARSE_OK or JSON_PARSE_ERROR)
 */
PARSE_RESULT jsonParse(T_DPA_PACKET *DpaPacket, uint8_t *DataBuffer, uint8_t DataBufferSize)
{
  PARSE_RESULT ParseResult = JSON_PARSE_OK;
  uint8_t Index, Cnt;
  uint16_t TempValue;
  char *TempPtr;

  if (jsonCheckFormat(DataBuffer, DataBufferSize) == JSON_PARSE_ERROR) ParseResult = JSON_PARSE_ERROR;
  else{

    /* MSG ID */
    strcpy_P(JsonFindStringBuffer, JsonMsgIdStr);
    Index = jsonFindString(JsonFindStringBuffer, DataBuffer, DataBufferSize);
    if (Index == 0) ParseResult = JSON_PARSE_ERROR;
    else{
      Cnt = 0;
      while (DataBuffer[Index]!='"') Index++;
      Index++;
      while (DataBuffer[Index]!='"' && Cnt<sizeof(JsonMsgIdBuff)) JsonMsgIdBuff[Cnt++] = DataBuffer[Index++];
      JsonControl.MsgIdSize = Cnt;
    }

    /* Timeout */
    strcpy_P(JsonFindStringBuffer, JsonTimeoutStr);
    Index = jsonFindString(JsonFindStringBuffer, DataBuffer, DataBufferSize);
    if (Index == 0) JsonControl.Timeout = 0;
    else JsonControl.Timeout = strtol((char *)&DataBuffer[++Index], &TempPtr, 10);

    /* Request_ts */
    strcpy_P(JsonFindStringBuffer, JsonRequestTsStr);
    JsonControl.RequestTs = jsonFindString(JsonFindStringBuffer, DataBuffer, DataBufferSize);

    /* Confirmation */
    strcpy_P(JsonFindStringBuffer, JsonConfirmationStr);
    JsonControl.Confirmation = jsonFindString(JsonFindStringBuffer, DataBuffer, DataBufferSize);

    /* Confirmation_ts */
    strcpy_P(JsonFindStringBuffer, JsonConfirmationTsStr);
    JsonControl.ConfirmationTs = jsonFindString(JsonFindStringBuffer, DataBuffer, DataBufferSize);

    /* Response */
    strcpy_P(JsonFindStringBuffer, JsonResponseStr);
    JsonControl.Response = jsonFindString(JsonFindStringBuffer, DataBuffer, DataBufferSize);

    /* Response_ts */
    strcpy_P(JsonFindStringBuffer, JsonResponseTsStr);
    JsonControl.ResponseTs = jsonFindString(JsonFindStringBuffer, DataBuffer, DataBufferSize);

    /* Request */
    strcpy_P(JsonFindStringBuffer, JsonRequestStr);
    JsonControl.Request = jsonFindString(JsonFindStringBuffer, DataBuffer, DataBufferSize);

    if (JsonControl.Type == JSON_TYPE_RAW){
      if (JsonControl.Request == 0) ParseResult = JSON_PARSE_ERROR;
      else JsonControl.RequestSize = jsonReadHexaField(&JsonRequestBuff[0], &DataBuffer[Index], DataBufferSize-Index);
    }

    if (JsonControl.Type == JSON_TYPE_RAW_HDP){
      /* nadr */
      strcpy_P(JsonFindStringBuffer, JsonNadrStr);
      TempValue = jsonReadValue(JsonFindStringBuffer, DataBuffer, DataBufferSize, &ParseResult);
      JsonRequestBuff[0] = TempValue;
      JsonRequestBuff[1] = TempValue >> 8;
      /* pnum */
      strcpy_P(JsonFindStringBuffer, JsonPnumStr);
      JsonRequestBuff[2] = jsonReadValue(JsonFindStringBuffer, DataBuffer, DataBufferSize, &ParseResult);
      /* pcmd */
      strcpy_P(JsonFindStringBuffer, JsonPcmdStr);
      JsonRequestBuff[3] = jsonReadValue(JsonFindStringBuffer, DataBuffer, DataBufferSize, &ParseResult);
      /* hwpid */
      strcpy_P(JsonFindStringBuffer, JsonHwpidStr);
      TempValue = jsonReadValue(JsonFindStringBuffer, DataBuffer, DataBufferSize, &ParseResult);
      JsonRequestBuff[4] = TempValue;
      JsonRequestBuff[5] = TempValue >> 8;

      JsonControl.RequestSize = 6;

      /* rdata */
      strcpy_P(JsonFindStringBuffer, JsonRdataStr);
      Index = jsonFindString(JsonFindStringBuffer, DataBuffer, DataBufferSize);
      if (Index){
        JsonControl.Rdata = 1;
        JsonControl.RequestSize += jsonReadHexaField(&JsonRequestBuff[6], &DataBuffer[Index], DataBufferSize-Index);
      }
    }

    /* fill DPA PACKET structure with data */
    if (JsonControl.RequestSize < 6) ParseResult = JSON_PARSE_ERROR;
    else{
      DpaPacket -> NADR = ((uint16_t)JsonRequestBuff[1] << 8) + JsonRequestBuff[0];
      DpaPacket -> PNUM = JsonRequestBuff[2];
      DpaPacket -> PCMD = JsonRequestBuff[3];
      DpaPacket -> HWPID = ((uint16_t)JsonRequestBuff[5] << 8) + JsonRequestBuff[4];
      JsonControl.DataSize = JsonControl.RequestSize - 6;
      if (JsonControl.DataSize > 0) memcpy((uint8_t *)&DpaPacket -> DpaMessage, &JsonRequestBuff[6], JsonControl.DataSize);
    }
  }

  if (ParseResult == JSON_PARSE_ERROR) jsonInit();

  return(ParseResult);
}

/**
 * Create JSON message from received DPA packet
 * @param DataBuffer  - pointer to buffer with gererated JSON message
 * @param DpaResponse - pointer to T_DPA_PACKET with DPA notification or response data
 * @param DpaOperationResult - result of dpaSendRequest(... ) function
 * @return size of created JSON message (0 = message was not created, due to its size is over the size of DataBuffer)
 */
uint16_t jsonCreate(uint8_t *DataBuffer, T_DPA_PACKET *DpaResponse, DPA_OPERATION_RESULT DpaOperationResult)
{
  uint16_t TempData;

  /* JSON type is not defined in case of receiving asynchronous DPA packet (notification) */
  if (JsonControl.Type == JSON_TYPE_NOT_DEFINED){
    memset(JsonMsgIdBuff, '0', 10);     // fill MGS ID buffer
    JsonControl.MsgIdSize = 10;
    JsonControl.Type = JSON_TYPE_RAW;   // set JSON type to raw format
    JsonControl.Request = 1;            // request is mandatory
    JsonControl.Response = 1;           // notification data as response
  }

  /* check size of new JSON message */
  // start + type
  JsonControl.DataSize = strlen(JsonPacketStartStr) + 1;
  if (JsonControl.Type == JSON_TYPE_RAW) JsonControl.DataSize += strlen(JsonTypeRawStr);
  else JsonControl.DataSize += strlen(JsonTypeRawHdpStr);
  // MsgId
  JsonControl.DataSize += (strlen(JsonMsgIdStr) + JsonControl.MsgIdSize + 3);
  // Timeout
  if (JsonControl.Timeout) JsonControl.DataSize += (strlen(JsonTimeoutStr) + 6);
  if (JsonControl.Type == JSON_TYPE_RAW_HDP){
    /* nadr */
    JsonControl.DataSize += (strlen(JsonNadrStr) + 7);
    /* pnum */
    JsonControl.DataSize += (strlen(JsonPnumStr) + 5);
    /* pcmd */
    JsonControl.DataSize += (strlen(JsonPcmdStr) + 5);
    /* hwpid */
    JsonControl.DataSize += (strlen(JsonHwpidStr) + 7);
    /* rcode */
    JsonControl.DataSize += (strlen(JsonRcodeStr) + 5);
    /* dpaval */
    JsonControl.DataSize += (strlen(JsonDpavalStr) + 5);
    /* rdata */
    if (JsonControl.Rdata) JsonControl.DataSize += (strlen(JsonRdataStr) + (dpaGetRxExtraDataSize() * 3) + 3);
  }
  // Request
  if (JsonControl.Request) JsonControl.DataSize += (strlen(JsonRequestStr) + (JsonControl.RequestSize * 3) + 3);
  // RequestTs
  if (JsonControl.RequestTs) JsonControl.DataSize += (strlen(JsonRequestTsStr) + 10 + 3);
  // Confirmation
  if (JsonControl.Confirmation) JsonControl.DataSize += (strlen(JsonConfirmationStr) + (11 * 3) + 3);
  // ConfirmationTs
  if (JsonControl.ConfirmationTs) JsonControl.DataSize += (strlen(JsonConfirmationTsStr) + 10 + 3);
  // Response
  if (JsonControl.Response) JsonControl.DataSize += (strlen(JsonResponseStr) + ((dpaGetRxExtraDataSize() + 8) * 3) + 3);
  // ResponseTs
  if (JsonControl.ResponseTs) JsonControl.DataSize += (strlen(JsonResponseTsStr) + 10 + 3);
  // STATUS_NO_ERROR
  JsonControl.DataSize += (strlen(JsonStatusStr) + strlen(JsonStatusNoErrStr) + 1);
  // compare final message size with size of buffer
  if (JsonControl.DataSize > JSON_OBJECT_BUFF_SIZE) return(0);

  JsonControl.DataSize = 0;

  /* start creation of new JSON message */
  strcpy_P((char *)&DataBuffer[0], JsonPacketStartStr);
  JsonControl.DataSize += strlen(JsonPacketStartStr);
  /* type */
  if (JsonControl.Type == JSON_TYPE_RAW){
    strcpy_P((char *)&DataBuffer[JsonControl.DataSize], JsonTypeRawStr);
    JsonControl.DataSize += strlen(JsonTypeRawStr);
  }
  else{
    strcpy_P((char *)&DataBuffer[JsonControl.DataSize], JsonTypeRawHdpStr);
    JsonControl.DataSize += strlen(JsonTypeRawHdpStr);
  }
  DataBuffer[JsonControl.DataSize++] = ',';
  /* MSGID */
  strcpy_P((char *)&DataBuffer[JsonControl.DataSize], JsonMsgIdStr);
  JsonControl.DataSize += strlen(JsonMsgIdStr);
  DataBuffer[JsonControl.DataSize++] = '"';
  memcpy(&DataBuffer[JsonControl.DataSize], JsonMsgIdBuff, JsonControl.MsgIdSize);
  JsonControl.DataSize += JsonControl.MsgIdSize;
  DataBuffer[JsonControl.DataSize++] = '"';
  DataBuffer[JsonControl.DataSize++] = ',';
  /* Timeout */
  if (JsonControl.Timeout){
    strcpy_P((char *)&DataBuffer[JsonControl.DataSize], JsonTimeoutStr);
    JsonControl.DataSize += strlen(JsonTimeoutStr);
    sprintf(JsonFindStringBuffer, "%d,", JsonControl.Timeout);
    strcpy((char *)&DataBuffer[JsonControl.DataSize], JsonFindStringBuffer);
    JsonControl.DataSize += strlen(JsonFindStringBuffer);
  }
  /* raw-hdp format */
  if (JsonControl.Type == JSON_TYPE_RAW_HDP){
    /* nadr */
    strcpy_P((char *)&DataBuffer[JsonControl.DataSize], JsonNadrStr);
    JsonControl.DataSize += strlen(JsonNadrStr);
    sprintf((char *)&DataBuffer[JsonControl.DataSize], "\"%04X\",", DpaResponse->NADR);
    JsonControl.DataSize += 7;
    /* pnum */
    strcpy_P((char *)&DataBuffer[JsonControl.DataSize], JsonPnumStr);
    JsonControl.DataSize += strlen(JsonPnumStr);
    sprintf((char *)&DataBuffer[JsonControl.DataSize], "\"%02X\",", DpaResponse->PNUM);
    JsonControl.DataSize += 5;
    /* pcmd */
    strcpy_P((char *)&DataBuffer[JsonControl.DataSize], JsonPcmdStr);
    JsonControl.DataSize += strlen(JsonPcmdStr);
    sprintf((char *)&DataBuffer[JsonControl.DataSize], "\"%02X\",", DpaResponse->PCMD);
    JsonControl.DataSize += 5;
    /* hwpid */
    strcpy_P((char *)&DataBuffer[JsonControl.DataSize], JsonHwpidStr);
    JsonControl.DataSize += strlen(JsonHwpidStr);
    sprintf((char *)&DataBuffer[JsonControl.DataSize], "\"%04X\",", DpaResponse->HWPID);
    JsonControl.DataSize += 7;
    /* rcode */
    strcpy_P((char *)&DataBuffer[JsonControl.DataSize], JsonRcodeStr);
    JsonControl.DataSize += strlen(JsonRcodeStr);
    sprintf((char *)&DataBuffer[JsonControl.DataSize], "\"%02X\",", DpaResponse->ResponseCode);
    JsonControl.DataSize += 5;
    /* dpaval */
    strcpy_P((char *)&DataBuffer[JsonControl.DataSize], JsonDpavalStr);
    JsonControl.DataSize += strlen(JsonDpavalStr);
    sprintf((char *)&DataBuffer[JsonControl.DataSize], "\"%02X\",", DpaResponse->DpaValue);
    JsonControl.DataSize += 5;
    /* rdata */
    if (JsonControl.Rdata){
      strcpy_P((char *)&DataBuffer[JsonControl.DataSize], JsonRdataStr);
      JsonControl.DataSize += strlen(JsonRdataStr);
      if (dpaWasResponsed() && DpaOperationResult==DPA_OPERATION_OK) TempData = dpaGetRxExtraDataSize();
      else TempData = 0;
      JsonControl.DataSize += jsonWriteHexaField(&DataBuffer[JsonControl.DataSize], (uint8_t *)DpaResponse + 8, TempData);
    }
  }
  /* Request */
  if (JsonControl.Request){
    strcpy_P((char *)&DataBuffer[JsonControl.DataSize], JsonRequestStr);
    JsonControl.DataSize += strlen(JsonRequestStr);
    JsonControl.DataSize += jsonWriteHexaField(&DataBuffer[JsonControl.DataSize], JsonRequestBuff, JsonControl.RequestSize);
  }
  /* RequestTs */
  if (JsonControl.RequestTs){
    strcpy_P((char *)&DataBuffer[JsonControl.DataSize], JsonRequestTsStr);
    JsonControl.DataSize += strlen(JsonRequestTsStr);
    sprintf((char *)&DataBuffer[JsonControl.DataSize], "\"%010lu\",", dpaGetRequestTs());
    JsonControl.DataSize += 13;
  }
  /* Confirmation */
  if (JsonControl.Confirmation){
    strcpy_P((char *)&DataBuffer[JsonControl.DataSize], JsonConfirmationStr);
    JsonControl.DataSize += strlen(JsonConfirmationStr);
    if (dpaWasConfirmed()) TempData = 11;
    else TempData = 0;
    JsonControl.DataSize += jsonWriteHexaField(&DataBuffer[JsonControl.DataSize], dpaGetConfirmationData(), TempData);
  }
  /* ConfirmationTs */
  if (JsonControl.ConfirmationTs){
    strcpy_P((char *)&DataBuffer[JsonControl.DataSize], JsonConfirmationTsStr);
    JsonControl.DataSize += strlen(JsonConfirmationTsStr);
    if (dpaWasConfirmed()){
      sprintf((char *)&DataBuffer[JsonControl.DataSize], "\"%010lu\",", dpaGetConfirmationTs());
      JsonControl.DataSize += 13;
    }
    else{
      DataBuffer[JsonControl.DataSize++] = '"';
      DataBuffer[JsonControl.DataSize++] = '"';
      DataBuffer[JsonControl.DataSize++] = ',';
    }
  }
  /* Response */
  if (JsonControl.Response){
    strcpy_P((char *)&DataBuffer[JsonControl.DataSize], JsonResponseStr);
    JsonControl.DataSize += strlen(JsonResponseStr);
    if (dpaWasResponsed() && DpaOperationResult==DPA_OPERATION_OK) TempData = 8 + dpaGetRxExtraDataSize();
    else TempData = 0;
    JsonControl.DataSize += jsonWriteHexaField(&DataBuffer[JsonControl.DataSize], (uint8_t *)DpaResponse, TempData);
  }
  /* ResponseTs */
  if (JsonControl.ResponseTs){
    strcpy_P((char *)&DataBuffer[JsonControl.DataSize], JsonResponseTsStr);
    JsonControl.DataSize += strlen(JsonResponseTsStr);
    if (dpaWasResponsed() && DpaOperationResult==DPA_OPERATION_OK){
      sprintf((char *)&DataBuffer[JsonControl.DataSize], "\"%010lu\",", dpaGetResponseTs());
      JsonControl.DataSize += 13;
    }
    else{
      DataBuffer[JsonControl.DataSize++] = '"';
      DataBuffer[JsonControl.DataSize++] = '"';
      DataBuffer[JsonControl.DataSize++] = ',';
    }
  }
  /* Status */
  strcpy_P((char *)&DataBuffer[JsonControl.DataSize], JsonStatusStr);
  JsonControl.DataSize += strlen(JsonStatusStr);
  switch(DpaOperationResult){
    case DPA_OPERATION_OK:{
      strcpy_P((char *)&DataBuffer[JsonControl.DataSize], JsonStatusNoErrStr);
      JsonControl.DataSize += strlen(JsonStatusNoErrStr);
    }
    break;

    case DPA_OPERATION_TIMEOUT:{
      strcpy_P((char *)&DataBuffer[JsonControl.DataSize], JsonStatusTimeoutStr);
      JsonControl.DataSize += strlen(JsonStatusTimeoutStr);
    }
    break;

    default:{
      strcpy_P((char *)&DataBuffer[JsonControl.DataSize], JsonStatusErrFailStr);
      JsonControl.DataSize += strlen(JsonStatusErrFailStr);
    }
  }

  TempData = JsonControl.DataSize;
  jsonInit();
  return(TempData);
}

/**
 * Get the timeout parameter from JSON message ( used in dpaSendRequest(... ) function )
 * @return  timeout parameter from JSON message (0 = timeout parameter not defined)
 */
uint16_t jsonGetTimeout(void)
{
  return (JsonControl.Timeout);
}

/**
 * Get the number of additional data bytes of DPA request in JSON message ( used in dpaSendRequest(... ) function )
 * @return  number of additional data bytes in DPA request message
 */
uint8_t jsonGetDataSize(void)
{
  return (JsonControl.DataSize);
}
