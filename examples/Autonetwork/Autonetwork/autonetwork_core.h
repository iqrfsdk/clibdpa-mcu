/**
 * Copyright 2015-2017 IQRF Tech s.r.o.
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
 *  Arduino IQRF DPA autonetwork implementation
 *
 *********************************************************************
 * FileName:        autonetwork_core.h
 * Platform:        Arduino
 * Company:         IQRF Tech s.r.o.
 *********************************************************************
*/

#ifndef _AUTONETWORK_CORE_H
#define _AUTONETWORK_CORE_H

#include <Arduino.h>
#include <dpa_library.h>

//----------------------
// Autonetwork constants
//----------------------
#define AN_MIN_PREBONDING_TIME 15

//------------------------
// Autonetwork event codes
//------------------------
#define EVT_AN_PROCESS_STARTED              0x00
#define EVT_AN_PROCESS_STOPPED              0x01
#define EVT_GET_NETWORK_INFO                0x02
#define EVT_ROUND_START                     0x03
#define EVT_COOR_ENABLE_PREBONDING          0x04
#define EVT_COOR_DISABLE_PREBONDING         0x05
#define EVT_COOR_READ_MID                   0x06
#define EVT_COOR_REMOVING_BOND              0x07
#define EVT_NODE_ENABLE_PREBONDING          0x08
#define EVT_NODE_DISABLE_PREBONDING         0x09
#define EVT_NODE_READ_MID                   0x0a
#define EVT_AUTHORIZE_BOND                  0x0b
#define EVT_AUTHORIZE_BOND_OK               0x0c
#define EVT_DISCOVERY                       0x0d
#define EVT_DISCOVERY_WAIT                  0x0e
#define EVT_DISCOVERY_OK                    0x0f
#define EVT_WAIT_PREBONDING                 0x10
#define EVT_FRC_DISABLE_PREBONDING          0x11
#define EVT_FRC_DISABLE_PREBONDING_BIT0_ERR 0x12
#define EVT_FRC_DISABLE_PREBONDING_BIT1_ERR 0x13
#define EVT_FRC_CHECK_NEW_NODES             0x14
#define EVT_NO_FREE_ADDRESS                 0x15
#define EVT_NO_NEW_NODE_PREBONDED           0x16
#define EVT_DPA_CONFIRMATION_TIMEOUT        0x17
#define EVT_DPA_RESPONSE_TIMEOUT            0x18
#define EVT_REMOVE_ALL_BONDS                0x19
#define EVT_MAX_NODES_PREBONDED             0x1a
#define EVT_NODE_REMOTE_UNBOND              0x1b
#define EVT_WITH_PARAM                      0x01
#define EVT_WITHOUT_PARAM                   0x00

//-------------------------
// Autonetwork return codes
//-------------------------
#define ERR_PROCESS_TERMINATED              0xff
#define ERR_NO_FREE_ADDRESS                 0xfe
#define ERR_MAX_NODES_PREBONDED             0xfd
#define AN_OK                               0x00
#define AN_ERROR                            0xff

//--------------------
// Buffer size, offset
//--------------------
#define NODE_BITMAP_SIZE                    32        // Bitmap for 256 nodes
#define MID_BUFFER_SIZE                     128       // Buffer for 128 MIDs
#define FRC_BUFFER_SIZE                     64
#define FRC_DATA_OFFSET                     0
#define FRC_DATA_SIZE                       55
#define FRC_EXTRA_DATA_OFFSET               55
#define FRC_EXTRA_DATA_SIZE                 9

//-----------------------
// Autonetwork parameters
//-----------------------
typedef struct
{
    uint16_t TemporaryAddressTimeout;
    uint16_t AuthorizeRetries;
    uint16_t DiscoveryRetries;
    uint16_t PrebondingInterval;
    uint16_t DiscoveryTxPower;
}T_AN_PARAMS;

//-------------
// Network info
//-------------
typedef struct
{
    uint8_t BondedNodesMap[NODE_BITMAP_SIZE];
    uint8_t BondedNodesCount;
    uint8_t DiscoveredNodesMap[NODE_BITMAP_SIZE];
    uint8_t DiscoveredNodesCount;
    uint8_t DID;
}T_NETWORK_INFO;

//----------------
// Prebonding info
//----------------
typedef struct
{
    uint32_t MID;
    uint32_t UserData;
    uint32_t PrebondedMID[7];
    uint32_t PrebondedUserData[7];
    uint32_t MIDlist[MID_BUFFER_SIZE];
    uint16_t WTimeout;
    uint16_t WaitBonding;
    uint16_t WaitBonding10ms;
    uint8_t PrebondedMIDcount;
    uint8_t MIDcount;
    uint8_t NewNodesMap[NODE_BITMAP_SIZE];
    uint8_t FrcData[FRC_BUFFER_SIZE];
    uint8_t Round;
    uint8_t Param;
    uint8_t OrigNodesCount;
    uint8_t BondingMask;
    uint8_t NextAddr;
    uint8_t NewNode;
    uint8_t DiscoveredNodesCount;
}T_PREBONDING_INFO;

//------------------
// Autonetwork state
//------------------
typedef struct
{
    T_NETWORK_INFO *NewtorkInfo;
    T_PREBONDING_INFO *PrebondingInfo;
    T_AN_PARAMS *Params;
}T_AN_STATE;

//-------------------------------
// Autonetwork callback functions
//-------------------------------
typedef void (*AUTONETWORK_NOTIFY_CB)(uint8_t EventCode, T_AN_STATE *State);

//-----------
// Prototypes
//-----------
void removeAllBonds(void);
void autonetworkInit(AUTONETWORK_NOTIFY_CB notify_CB);
uint8_t autonetwork(T_AN_PARAMS *Parameters);
uint8_t ledG(uint16_t Addr, uint8_t Cmd);
uint8_t ledR(uint16_t Addr, uint8_t Cmd);

extern T_DPA_PACKET MyDpaPacket;

#endif
