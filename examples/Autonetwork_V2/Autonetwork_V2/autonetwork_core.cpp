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
 *  Arduino IQRF DPA autonetwork implementation
 *
 *********************************************************************
 * FileName:        autonetwork_core.cpp
 * Platform:        Arduino
 * Company:         IQRF Tech s.r.o.
 *********************************************************************
*/

#include <Arduino.h>
#include "dpa_library.h"
#include "autonetwork_core.h"
#include "autonetwork_V2.h"

/*
 * definitions
 */
#define SHORT_TIMEOUT   2000
#define LONG_TIMEOUT    60000

/*
 * Variables
 */
AUTONETWORK_NOTIFY_CB notify_cb = NULL;

T_PREBONDING_INFO Prebonding;
T_AN_STATE AutonetworkState;
T_AN_PARAMS ANparams;
T_NETWORK_INFO NetworkInfo;
T_DPA_PACKET MyDpaPacket;

volatile uint8_t Terminate = 0;

/*
 * C prototypes
 */
uint8_t getBitValue(uint8_t *BitField, uint8_t BitAddr);
void setBitValue(uint8_t *BitField, uint8_t BitAddr);
uint8_t isNodeBonded(uint8_t Addr);
uint8_t nextFreeAddr(void);
void delayMS(uint32_t Time);
void notifyMainApp(uint8_t EventCode, uint8_t Param);
uint8_t sendMyDpaRequest(T_DPA_PACKET *DpaRequest, uint8_t DataSize, uint16_t Timeout);
uint8_t getAddrInfo(void);
uint8_t getDiscoveredNodes(void);
uint8_t getBondedNodes(void);
uint8_t clearAllBond(void);
uint8_t removeBondedNode(uint8_t BondAddr);
uint8_t discovery(uint8_t TxPower, uint8_t MaxAddr);
uint8_t setDPAParam(uint8_t Param);
uint8_t setHops(uint8_t ReqHops, uint8_t RespHops);
uint8_t sendBatch(uint16_t Addr, uint8_t Len);
uint8_t readPrebondedMID(void);
uint8_t frcSend(uint8_t Len, uint8_t *FrcStatus);
uint8_t frcSelectiveSend(uint8_t Len);
uint8_t frcExtraResult(void);
uint8_t authorizeBond(uint8_t ReqAddr, uint32_t MID);
uint8_t removeBondRestart(uint16_t Addr);
uint8_t smartConnectPrebonding(void);

/**
 * Get the value of specified bit in bitField array
 *
 * @param - BitField pointer to bitField array
 * @param - BitAddr specified bit
 * @return - value of specified bit in bitField array
 *
 */
uint8_t getBitValue(uint8_t *BitField, uint8_t BitAddr)
{
    return((BitField[BitAddr / 8] >> (BitAddr % 8)) & 0x01);
}

/**
 * Set the specified bit in bitField array
 *
 * @param - BitField pointer to bitField array
 * @param - BitAddr specified bit
 * @return - none
 *
 */
void setBitValue(uint8_t *BitField, uint8_t BitAddr)
{
    BitField[BitAddr / 8] |= (1 << (BitAddr % 8));
}

/**
 * Check the specified node is bonded
 *
 * @param - Addr specified node address
 * @return - actual bond state of specified node
 *
 */
uint8_t isNodeBonded(uint8_t Addr)
{
    return(getBitValue(NetworkInfo.BondedNodesMap, Addr));
}

/**
 * Get next free address to bond new node
 *
 * @param - From starting address to start search for next free node address
 * @return - next free address to bond new node
 *
 */
uint8_t nextFreeAddr(void)
{
    uint8_t CheckAddr;

    for(CheckAddr=1; CheckAddr<MAX_ADDRESS; CheckAddr++)
        if(isNodeBonded(CheckAddr) == 0)
            return (CheckAddr);

    // No free address
    return(ERR_NO_FREE_ADDRESS);
}

/**
 * Delay function
 *
 * @param - Time specified delay time in ms
 * @return - none
 *
 */
void delayMS(uint32_t Time)
{
    static uint8_t InputCharMem = 0;
    uint8_t InputChar;

    if (Time != 0) {

        while (Time--) {
            delay(1);

            // Check for process termination by user
            if (Serial.available() > 0) {
                InputChar = Serial.read();
                if ((InputChar == '\r') || (InputChar == '\n')) {
                    if (InputCharMem == 'q' || InputCharMem == 'Q') {
                        InputCharMem = 0;
                        ANparams.BondingWaves = 0;
                        Prebonding.ReturnCode = ERR_PROCESS_TERMINATED;
                        return;
                    }
                } else {
                    InputCharMem = InputChar;
                }
            }
        }
    }
}

/**
 * Notify main application
 *
 * @param - EventCode number of desired message
 * @param - Param print message with or without parameters
 * @return - none
 *
 */
void notifyMainApp(uint8_t EventCode, uint8_t Param)
{
    if(notify_cb != NULL) {
        if(Param == EVT_WITHOUT_PARAM)
            notify_cb(EventCode, NULL);
        else
            notify_cb(EventCode, &AutonetworkState);
    }
}

/**
 * Send DPA request and process DPA response
 *
 * @param DpaRequest Pointer to DPA request
 * @param DataSize Size of data
 * @param Timeout Timeout
 * @return Operation result
 *
 */
uint8_t sendMyDpaRequest(T_DPA_PACKET *DpaRequest, uint8_t DataSize, uint16_t Timeout)
{
    uint8_t OpResult;
    uint8_t NotifyException = false;

    if (DpaRequest->NADR == TEMPORARY_ADDRESS || DpaRequest->PCMD == CMD_COORDINATOR_DISCOVERY)
        NotifyException = true;

    // send request and wait for result
    while((OpResult = dpaSendRequest(DpaRequest, DataSize, Timeout)) == DPA_OPERATION_IN_PROGRESS)
        ; /* void */

    if (NotifyException == false) {
        switch (OpResult) {
        case DPA_OPERATION_TIMEOUT:
            notifyMainApp(EVT_DPA_OPERATION_TIMEOUT, EVT_WITHOUT_PARAM);
            break;

        case DPA_CONFIRMATION_ERR:
            notifyMainApp(EVT_DPA_CONFIRMATION_TIMEOUT, EVT_WITHOUT_PARAM);
            break;

        case DPA_RESPONSE_ERR:
            notifyMainApp(EVT_DPA_RESPONSE_TIMEOUT, EVT_WITHOUT_PARAM);
            break;
        }
    }

    return(OpResult);
}

/**
 * Get coordinator addressing information
 *
 * @param - none
 * @return - result of DPA operation
 *
 */
uint8_t getAddrInfo(void)
{
    uint8_t OpResult;

    // Get addressing info
    MyDpaPacket.NADR = COORDINATOR_ADDRESS;
    MyDpaPacket.HWPID = HWPID_DoNotCheck;
    MyDpaPacket.PNUM = PNUM_COORDINATOR;
    MyDpaPacket.PCMD = CMD_COORDINATOR_ADDR_INFO;
    OpResult = sendMyDpaRequest(&MyDpaPacket, 0, SHORT_TIMEOUT);

    if (OpResult == DPA_OPERATION_OK) {
        NetworkInfo.BondedNodesCount = MyDpaPacket.DpaMessage.PerCoordinatorAddrInfo_Response.DevNr;
        NetworkInfo.DID = MyDpaPacket.DpaMessage.PerCoordinatorAddrInfo_Response.DID;
    }

    return(OpResult);
}

/**
 * Get discovered nodes
 *
 * @param - none
 * @return - result of DPA operation
 *
 */
uint8_t getDiscoveredNodes(void)
{
    uint8_t OpResult;

    // Get discovered nodes
    MyDpaPacket.NADR = COORDINATOR_ADDRESS;
    MyDpaPacket.HWPID = HWPID_DoNotCheck;
    MyDpaPacket.PNUM = PNUM_COORDINATOR;
    MyDpaPacket.PCMD = CMD_COORDINATOR_DISCOVERED_DEVICES;
    OpResult = sendMyDpaRequest(&MyDpaPacket, 0, SHORT_TIMEOUT);

    if (OpResult == DPA_OPERATION_OK)
        memcpy(NetworkInfo.DiscoveredNodesMap, (void*)&MyDpaPacket.DpaMessage.Response.PData[0], NODE_BITMAP_SIZE);

    return(OpResult);
}

/**
 * Get bonded nodes
 *
 * @param - none
 * @return - result of DPA operation
 *
 */
uint8_t getBondedNodes(void)
{
    uint8_t OpResult;

    // Get bonded nodes
    MyDpaPacket.NADR = COORDINATOR_ADDRESS;
    MyDpaPacket.HWPID = HWPID_DoNotCheck;
    MyDpaPacket.PNUM = PNUM_COORDINATOR;
    MyDpaPacket.PCMD = CMD_COORDINATOR_BONDED_DEVICES;
    OpResult = sendMyDpaRequest(&MyDpaPacket, 0, SHORT_TIMEOUT);

    if (OpResult == DPA_OPERATION_OK)
        memcpy(NetworkInfo.BondedNodesMap, (void*)&MyDpaPacket.DpaMessage.Response.PData[0], NODE_BITMAP_SIZE);

    return(OpResult);
}

/**
 * Clear all bonds
 *
 * @param - none
 * @return - result of DPA operation
 *
 */
uint8_t clearAllBond(void)
{
    // Clear all bonds
    MyDpaPacket.NADR = COORDINATOR_ADDRESS;
    MyDpaPacket.HWPID = HWPID_DoNotCheck;
    MyDpaPacket.PNUM = PNUM_COORDINATOR;
    MyDpaPacket.PCMD = CMD_COORDINATOR_CLEAR_ALL_BONDS;
    return(sendMyDpaRequest(&MyDpaPacket, 0, SHORT_TIMEOUT));
}

/**
 * Remove bonded node
 *
 * @param - BondAddr specified node address to remove
 * @return - result of DPA operation
 *
 */
uint8_t removeBondedNode(uint8_t BondAddr)
{
    // Remove bonded node
    MyDpaPacket.NADR = COORDINATOR_ADDRESS;
    MyDpaPacket.HWPID = HWPID_DoNotCheck;
    MyDpaPacket.PNUM = PNUM_COORDINATOR;
    MyDpaPacket.PCMD = CMD_COORDINATOR_REMOVE_BOND;
    MyDpaPacket.DpaMessage.PerCoordinatorRemoveBond_Request.BondAddr = BondAddr;

    return(sendMyDpaRequest(&MyDpaPacket, sizeof(TPerCoordinatorRemoveBond_Request), SHORT_TIMEOUT));
}

/**
 * Discovery;
 *
 * @param - TxPower discovery Tx power
 * @param - MaxAddr maximal address for discovering nodes
 * @return - result of DPA operation
 *
 */
uint8_t discovery(uint8_t TxPower, uint8_t MaxAddr)
{
    uint8_t OpResult;

    // Discovery
    MyDpaPacket.NADR = COORDINATOR_ADDRESS;
    MyDpaPacket.HWPID = HWPID_DoNotCheck;
    MyDpaPacket.PNUM = PNUM_COORDINATOR;
    MyDpaPacket.PCMD = CMD_COORDINATOR_DISCOVERY;
    MyDpaPacket.DpaMessage.PerCoordinatorDiscovery_Request.TxPower = TxPower;
    MyDpaPacket.DpaMessage.PerCoordinatorDiscovery_Request.MaxAddr = MaxAddr;

    OpResult = sendMyDpaRequest(&MyDpaPacket, sizeof(TPerCoordinatorDiscovery_Request), LONG_TIMEOUT);

    if (OpResult == DPA_OPERATION_OK)
        Prebonding.Param = MyDpaPacket.DpaMessage.PerCoordinatorDiscovery_Response.DiscNr;

    return(OpResult);
}

/**
 * Set DPA Param
 *
 * @param - Param specified DPA parameter
 * @return - result of DPA operation
 *
 */
uint8_t setDPAParam(uint8_t Param)
{
    // Set DPA param
    MyDpaPacket.NADR = COORDINATOR_ADDRESS;
    MyDpaPacket.HWPID = HWPID_DoNotCheck;
    MyDpaPacket.PNUM = PNUM_COORDINATOR;
    MyDpaPacket.PCMD = CMD_COORDINATOR_SET_DPAPARAMS;
    MyDpaPacket.DpaMessage.PerCoordinatorSetDpaParams_Request_Response.DpaParam = Param;

    return(sendMyDpaRequest(&MyDpaPacket, sizeof(TPerCoordinatorSetDpaParams_Request_Response), SHORT_TIMEOUT));
}

/**
 * Set Hops
 *
 * @param - ReqHops specified number of hops for DPA request
 * @param - RespHops specified number of hops for DPA response
 * @return - result of DPA operation
 *
 */
uint8_t setHops(uint8_t ReqHops, uint8_t RespHops)
{
    // Set hops
    MyDpaPacket.NADR = COORDINATOR_ADDRESS;
    MyDpaPacket.HWPID = HWPID_DoNotCheck;
    MyDpaPacket.PNUM = PNUM_COORDINATOR;
    MyDpaPacket.PCMD = CMD_COORDINATOR_SET_HOPS;
    MyDpaPacket.DpaMessage.PerCoordinatorSetHops_Request_Response.RequestHops = ReqHops;
    MyDpaPacket.DpaMessage.PerCoordinatorSetHops_Request_Response.ResponseHops = RespHops;

    return(sendMyDpaRequest(&MyDpaPacket, sizeof(TPerCoordinatorSetHops_Request_Response), SHORT_TIMEOUT));
}

/**
 * Send batch
 *
 * @param - Addr destination address of specified TR module
 * @param - Len length of batch command
 * @return - result of DPA operation
 *
 */
uint8_t sendBatch(uint16_t Addr, uint8_t Len)
{
    // Send Batch command
    MyDpaPacket.NADR = Addr;
    MyDpaPacket.HWPID = HWPID_DoNotCheck;
    MyDpaPacket.PNUM = PNUM_OS;
    MyDpaPacket.PCMD = CMD_OS_BATCH;

    return(sendMyDpaRequest(&MyDpaPacket, Len, LONG_TIMEOUT));
}

/**
 * Read remotely bonded module ID
 *
 * @return - number of re-added MIDS
 *
 */
uint8_t readPrebondedMID(void)
{
    uint8_t BufferPtr;
    uint8_t Cnt;
    uint8_t MIDcount;
    uint32_t TemporaryMID;

    MIDcount = 0;
    for (Cnt=0; Cnt<15; Cnt++) {
        BufferPtr = 4 * Cnt;
        TemporaryMID = (uint32_t)Prebonding.FrcData[BufferPtr];
        TemporaryMID |= (uint32_t)Prebonding.FrcData[BufferPtr + 1] << 8;
        TemporaryMID |= (uint32_t)Prebonding.FrcData[BufferPtr + 2] << 16;
        TemporaryMID |= (uint32_t)Prebonding.FrcData[BufferPtr + 3] << 24;

        if (TemporaryMID != 0)
            Prebonding.MIDlist[MIDcount++] = TemporaryMID - 1;
    }

    return (MIDcount);
}

/**
 * Send DPA command for green LED
 *
 * @param - Addr destination address of specified TR module
 * @param - Cmd specified DPA command for LED
 * @return - result of DPA operation
 *
 */
uint8_t ledG(uint16_t Addr, uint8_t Cmd)
{
    // LEDG peripheral
    MyDpaPacket.NADR = Addr;
    MyDpaPacket.HWPID = HWPID_DoNotCheck;
    MyDpaPacket.PNUM = PNUM_LEDG;
    MyDpaPacket.PCMD = Cmd;

    return(sendMyDpaRequest(&MyDpaPacket, 0, SHORT_TIMEOUT));
}

/**
 * Send DPA command for red LED
 *
 * @param - Addr destination address of specified TR module
 * @param - Cmd specified DPA command for LED
 * @return - result of DPA operation
 *
 */
uint8_t ledR(uint16_t Addr, uint8_t Cmd)
{
    // LEDR peripheral
    MyDpaPacket.NADR = Addr;
    MyDpaPacket.HWPID = HWPID_DoNotCheck;
    MyDpaPacket.PNUM = PNUM_LEDR;
    MyDpaPacket.PCMD = Cmd;

    return(sendMyDpaRequest(&MyDpaPacket, 0, SHORT_TIMEOUT));
}

/**
 * Send Smart connect command to prebond new nodes
 *
 * @param - none
 * @return - result of DPA operation
 *
 */
uint8_t smartConnectPrebonding(void)
{
    MyDpaPacket.NADR = COORDINATOR_ADDRESS;
    MyDpaPacket.HWPID = HWPID_DoNotCheck;
    MyDpaPacket.PNUM = PNUM_COORDINATOR;
    MyDpaPacket.PCMD = CMD_COORDINATOR_SMART_CONNECT;

    memset((uint8_t *)&MyDpaPacket.DpaMessage.PerCoordinatorSmartConnect_Request, 0, sizeof(MyDpaPacket.DpaMessage.PerCoordinatorSmartConnect_Request));
    MyDpaPacket.DpaMessage.PerCoordinatorSmartConnect_Request.ReqAddr = TEMPORARY_ADDRESS;

    return(sendMyDpaRequest(&MyDpaPacket, sizeof(MyDpaPacket.DpaMessage.PerCoordinatorSmartConnect_Request), LONG_TIMEOUT));
}


/**
 * FRC Send
 *
 * @param - Len - length of FRC command
 * @param - FrcStatus - pointer to variable where the FRC status will be returned
 * @return - result of DPA operation
 *
 */
uint8_t frcSend(uint8_t Len, uint8_t *FrcStatus)
{
    uint8_t OpResult;

    // Send FRC
    MyDpaPacket.NADR = COORDINATOR_ADDRESS;
    MyDpaPacket.HWPID = HWPID_DoNotCheck;
    MyDpaPacket.PNUM = PNUM_FRC;
    MyDpaPacket.PCMD = CMD_FRC_SEND;

    OpResult = sendMyDpaRequest(&MyDpaPacket, Len, LONG_TIMEOUT);

    if (OpResult == DPA_OPERATION_OK) {
        memcpy((void*)&Prebonding.FrcData[FRC_DATA_OFFSET], (void*)&MyDpaPacket.DpaMessage.PerFrcSend_Response.FrcData[0], FRC_DATA_SIZE);
        *FrcStatus = MyDpaPacket.DpaMessage.PerFrcSend_Response.Status;
    }

    return(OpResult);
}

/**
 * FRC selective Send
 *
 * @param - Len - length of FRC command
 * @return - result of DPA operation
 *
 */
uint8_t frcSelectiveSend(uint8_t Len)
{
    uint8_t OpResult;

    // Send selective FRC
    MyDpaPacket.NADR = COORDINATOR_ADDRESS;
    MyDpaPacket.HWPID = HWPID_DoNotCheck;
    MyDpaPacket.PNUM = PNUM_FRC;
    MyDpaPacket.PCMD = CMD_FRC_SEND_SELECTIVE;

    OpResult = sendMyDpaRequest(&MyDpaPacket, Len, LONG_TIMEOUT);

    if (OpResult == DPA_OPERATION_OK)
        memcpy((void*)&Prebonding.FrcData[FRC_DATA_OFFSET], (void*)&MyDpaPacket.DpaMessage.PerFrcSend_Response.FrcData[0], FRC_DATA_SIZE);

    return(OpResult);
}

/**
 * FRC extra result
 *
 * @param - none
 * @return - result of DPA operation
 *
 */
uint8_t frcExtraResult(void)
{
    uint8_t OpResult;

    // FRC Extra result
    MyDpaPacket.NADR = COORDINATOR_ADDRESS;
    MyDpaPacket.HWPID = HWPID_DoNotCheck;
    MyDpaPacket.PNUM = PNUM_FRC;
    MyDpaPacket.PCMD = CMD_FRC_EXTRARESULT;

    OpResult = sendMyDpaRequest(&MyDpaPacket, 0, SHORT_TIMEOUT);

    if (OpResult == DPA_OPERATION_OK)
        memcpy((void*)&Prebonding.FrcData[FRC_EXTRA_DATA_OFFSET], (void*)&MyDpaPacket.DpaMessage.Response.PData[0], FRC_EXTRA_DATA_SIZE);

    return(OpResult);
}

/**
 * Authorize bond
 *
 * @param - ReqAddr required address of authorized node
 * @param - MID module ID of authorized node
 * @return - result of DPA operation
 *
 */
uint8_t authorizeBond(uint8_t ReqAddr, uint32_t MID)
{
    uint8_t OpResult;

    // Authorize node
    MyDpaPacket.NADR = COORDINATOR_ADDRESS;
    MyDpaPacket.HWPID = HWPID_DoNotCheck;
    MyDpaPacket.PNUM = PNUM_COORDINATOR;
    MyDpaPacket.PCMD = CMD_COORDINATOR_AUTHORIZE_BOND;
    MyDpaPacket.DpaMessage.PerCoordinatorAuthorizeBond_Request.ReqAddr = ReqAddr;
    MyDpaPacket.DpaMessage.PerCoordinatorAuthorizeBond_Request.MID[0] = MID & 0xFF;
    MyDpaPacket.DpaMessage.PerCoordinatorAuthorizeBond_Request.MID[1] = (MID >> 8) & 0xFF;
    MyDpaPacket.DpaMessage.PerCoordinatorAuthorizeBond_Request.MID[2] = (MID >> 16) & 0xFF;
    MyDpaPacket.DpaMessage.PerCoordinatorAuthorizeBond_Request.MID[3] = (MID >> 24) & 0xFF;

    OpResult = sendMyDpaRequest(&MyDpaPacket, sizeof(TPerCoordinatorAuthorizeBond_Request), LONG_TIMEOUT);

    // Authorization OK ?
    if (OpResult == DPA_OPERATION_OK) {
        // Yes
        Prebonding.Param = MyDpaPacket.DpaMessage.PerCoordinatorAuthorizeBond_Response.DevNr;
        Prebonding.NewNodesCount++;
        setBitValue(NetworkInfo.BondedNodesMap, ReqAddr);
        setBitValue(Prebonding.NewNodesMap, ReqAddr);
    }

    return(OpResult);
}

/**
 * Remove bond and restart using batch command
 *
 * @param - Addr address of removed node
 * @return - result of DPA operation
 *
 */
uint8_t removeBondRestart(uint16_t Addr)
{
    // Remove bond
    MyDpaPacket.DpaMessage.Request.PData[0] = 5;
    MyDpaPacket.DpaMessage.Request.PData[1] = PNUM_NODE;
    MyDpaPacket.DpaMessage.Request.PData[2] = CMD_NODE_REMOVE_BOND;
    MyDpaPacket.DpaMessage.Request.PData[3] = HWPID_DoNotCheck & 0xff;
    MyDpaPacket.DpaMessage.Request.PData[4] = HWPID_DoNotCheck >> 0x08;
    // Reset
    MyDpaPacket.DpaMessage.Request.PData[5] = 5;
    MyDpaPacket.DpaMessage.Request.PData[6] = PNUM_OS;
    MyDpaPacket.DpaMessage.Request.PData[7] = CMD_OS_RESTART;
    MyDpaPacket.DpaMessage.Request.PData[8] = HWPID_DoNotCheck & 0xff;
    MyDpaPacket.DpaMessage.Request.PData[9] = HWPID_DoNotCheck >> 0x08;
    // EndBatch
    MyDpaPacket.DpaMessage.Request.PData[10] = 0;
    // Batch command
    return(sendBatch(Addr, 11));
}

/**
 * Remove all bond at nodes and coordinator
 *
 * @param - none
 * @return - none
 *
 */
void removeAllBonds(void)
{
    // Send broadcast batch command to remove bonds and reset nodes
    notifyMainApp(EVT_REMOVE_ALL_BONDS, EVT_WITHOUT_PARAM);
    removeBondRestart(BROADCAST_ADDRESS);
    // Remove bonds at coordinator too
    clearAllBond();
}

/**
 * Autonetwork initialization
 *
 * @param - notify_CB pointer to user notification callback function
 * @return - none
 *
 */
void autonetworkInit(AUTONETWORK_NOTIFY_CB notify_CB)
{
    // Set callback functions
    if(notify_CB != NULL)
        notify_cb = notify_CB;
}

/**
 * Autonetwork function
 *
 * @param - Parameters pointer to structure with autonetwork parameters
 * @return - none
 *
 */
uint8_t autonetwork(T_AN_PARAMS *Parameters)
{
    uint8_t Cnt;

    // initialization
    memcpy((void*)&ANparams, (void*)Parameters, sizeof(T_AN_PARAMS));
    AutonetworkState.NewtorkInfo = &NetworkInfo;
    AutonetworkState.PrebondingInfo = &Prebonding;
    AutonetworkState.Params = &ANparams;

    Prebonding.ReturnCode = AN_OK;

    Prebonding.WaveCnt = 0;
    Prebonding.EmtyWaveCnt = 0;

    // Notify main application
    notifyMainApp(EVT_AN_PROCESS_STARTED, EVT_WITHOUT_PARAM);

    // Get addressing info
    notifyMainApp(EVT_GET_NETWORK_INFO, EVT_WITHOUT_PARAM);
    getAddrInfo();
    Prebonding.OrigNodesCount = NetworkInfo.BondedNodesCount;

    // Get bonded nodes bitmap
    getBondedNodes();

    // Get discovered nodes bitmap
    getDiscoveredNodes();

    for(;;) {
        // check termination conditions of autonetwork process
        if (Prebonding.WaveCnt >= ANparams.BondingWaves || Prebonding.EmtyWaveCnt >= ANparams.EmptyWaves)
            break;

        Prebonding.WaveCnt++;
        Prebonding.NewNodesCount = 0;

        // Set DPA hops
        setHops(0xff, 0xff);
        // Set DPA parameter
        setDPAParam(0x00);
        // Reset bitmap of nodes provided prebonding
        memset(Prebonding.NewNodesMap, 0, NODE_BITMAP_SIZE);
        memset(Prebonding.PrebondedNodesMap, 0, NODE_BITMAP_SIZE);
        // Notify main application
        notifyMainApp(EVT_ROUND_START, EVT_WITH_PARAM);

        // Smart connect prebonding
        notifyMainApp(EVT_SMART_CONNECT_PREBONDING, EVT_WITHOUT_PARAM);
        smartConnectPrebonding();

        // Prebonded alive FRC
        notifyMainApp(EVT_FRC_CHECK_NEW_NODES, EVT_WITHOUT_PARAM);
        MyDpaPacket.DpaMessage.PerFrcSend_Request.FrcCommand = FRC_PrebondedAlive;
        MyDpaPacket.DpaMessage.PerFrcSend_Request.UserData[0] = Prebonding.WaveCnt;
        MyDpaPacket.DpaMessage.PerFrcSend_Request.UserData[1] = 0;
        if (frcSend(sizeof(MyDpaPacket.DpaMessage.PerFrcSend_Request) - sizeof(MyDpaPacket.DpaMessage.PerFrcSend_Request.UserData) + 2, &Prebonding.PrebondedNodesCount) == DPA_OPERATION_OK) {
            memcpy(&Prebonding.PrebondedNodesMap[0], &Prebonding.FrcData[0], NODE_BITMAP_SIZE);
            if (Prebonding.PrebondedNodesCount == 0) {
                notifyMainApp(EVT_NO_NEW_NODE_PREBONDED, EVT_WITHOUT_PARAM);
            } else {
                Prebonding.Param = Prebonding.PrebondedNodesCount;
                notifyMainApp(EVT_PREBONDED_NEW_NODE, EVT_WITH_PARAM);
            }
        } else {
            Prebonding.PrebondedNodesCount = 0;
        }

        // if any new nodes are prebonded
        if (Prebonding.PrebondedNodesCount != 0) {

            Prebonding.PrebondedMIDReadOffset = 0;
            Prebonding.PrebondedMIDcount = 0;

            // read MIDs all prebonded nodes
            while (Prebonding.PrebondedMIDcount < Prebonding.PrebondedNodesCount && Prebonding.PrebondedMIDReadOffset != 240) {
                memcpy((void*)&MyDpaPacket.DpaMessage.PerFrcSendSelective_Request.SelectedNodes, Prebonding.PrebondedNodesMap, NODE_BITMAP_SIZE);
                MyDpaPacket.DpaMessage.PerFrcSendSelective_Request.FrcCommand = FRC_PrebondedMemoryReadPlus1;
                MyDpaPacket.DpaMessage.PerFrcSendSelective_Request.UserData[0] = Prebonding.WaveCnt;
                MyDpaPacket.DpaMessage.PerFrcSendSelective_Request.UserData[1] = Prebonding.PrebondedMIDReadOffset;
                MyDpaPacket.DpaMessage.PerFrcSendSelective_Request.UserData[2] = 0xA0;
                MyDpaPacket.DpaMessage.PerFrcSendSelective_Request.UserData[3] = 0x04;
                MyDpaPacket.DpaMessage.PerFrcSendSelective_Request.UserData[4] = PNUM_OS;
                MyDpaPacket.DpaMessage.PerFrcSendSelective_Request.UserData[5] = CMD_OS_READ;
                MyDpaPacket.DpaMessage.PerFrcSendSelective_Request.UserData[6] = 0;
                if (frcSelectiveSend(sizeof(MyDpaPacket.DpaMessage.PerFrcSendSelective_Request) - sizeof(MyDpaPacket.DpaMessage.PerFrcSendSelective_Request.UserData) + 7) == DPA_OPERATION_OK) {
                    frcExtraResult();
                    Prebonding.ReadedMIDCount = readPrebondedMID();
                    if (Prebonding.ReadedMIDCount > Prebonding.PrebondedNodesCount)
                        Prebonding.ReadedMIDCount = Prebonding.PrebondedNodesCount;
                } else {
                    Prebonding.ReadedMIDCount = 0;
                }

                if (Prebonding.ReadedMIDCount != 0) {
                    Prebonding.PrebondedMIDcount += Prebonding.ReadedMIDCount;
                    notifyMainApp(EVT_FRC_READ_NEW_NODES_MID, EVT_WITHOUT_PARAM);
                }
                Prebonding.PrebondedMIDReadOffset += 15;

                // authorize prebonded nodes
                for (Cnt=0; Cnt<Prebonding.ReadedMIDCount; Cnt++) {
                    Prebonding.MID = Prebonding.MIDlist[Cnt];

                    // OK, Get next free address
                    Prebonding.NextAddr = nextFreeAddr();
                    if (Prebonding.NextAddr == 0xff) {
                        // Error, no free address, terminate process
                        notifyMainApp(EVT_NO_FREE_ADDRESS, EVT_WITHOUT_PARAM);
                        Prebonding.PrebondedMIDcount = Prebonding.PrebondedNodesCount;
                        ANparams.BondingWaves = 0;
                        Prebonding.ReturnCode = ERR_NO_FREE_ADDRESS;
                        break;
                    } else {
                        // Authorize node
                        notifyMainApp(EVT_AUTHORIZE_BOND, EVT_WITH_PARAM);
                        if (authorizeBond(Prebonding.NextAddr, Prebonding.MID) == DPA_OPERATION_OK)
                            notifyMainApp(EVT_AUTHORIZE_BOND_OK, EVT_WITH_PARAM);
                    }
                    delayMS(100);
                }
            }
        }

        // check new authorized nodes
        if (Prebonding.NewNodesCount != 0) {
            notifyMainApp(EVT_FRC_CHECK_AUTHORIZED_NODES, EVT_WITHOUT_PARAM);
            memcpy((void*)&MyDpaPacket.DpaMessage.PerFrcSendSelective_Request.SelectedNodes, Prebonding.NewNodesMap, NODE_BITMAP_SIZE);
            MyDpaPacket.DpaMessage.PerFrcSendSelective_Request.FrcCommand = FRC_Prebonding;
            MyDpaPacket.DpaMessage.PerFrcSendSelective_Request.UserData[0] = 0;
            MyDpaPacket.DpaMessage.PerFrcSendSelective_Request.UserData[1] = 0;
            if (frcSelectiveSend(sizeof(MyDpaPacket.DpaMessage.PerFrcSendSelective_Request) - sizeof(MyDpaPacket.DpaMessage.PerFrcSendSelective_Request.UserData) + 2) == DPA_OPERATION_OK) {
                for (Cnt=0; Cnt<NODE_BITMAP_SIZE; Cnt++)
                    Prebonding.NewNodesMap[Cnt] ^= Prebonding.FrcData[Cnt];
                for (Cnt=1; Cnt<=MAX_ADDRESS; Cnt++) {
                    if (getBitValue(Prebonding.NewNodesMap, Cnt) == 1) {
                        if (Prebonding.NewNodesCount)
                            Prebonding.NewNodesCount--;
                        // No response from the node
                        Prebonding.Param = Cnt;
                        // Remove and reset the node
                        notifyMainApp(EVT_NODE_REMOTE_UNBOND, EVT_WITH_PARAM);
                        removeBondRestart(Cnt);
                        // Remove node at coordinator too
                        notifyMainApp(EVT_COOR_REMOVING_BOND, EVT_WITH_PARAM);
                        removeBondedNode(Cnt);
                    }
                }
            }
        }

        // Remove and reset node with temporary address 0xfe
        removeBondRestart(TEMPORARY_ADDRESS);

        // if any new nodes authorized
        if (Prebonding.NewNodesCount != 0) {
            Prebonding.EmtyWaveCnt = 0;

            // Run Discovery
            for (uint8_t DiscoveryRetry = ANparams.DiscoveryRetries; DiscoveryRetry != 0; DiscoveryRetry--) {
                notifyMainApp(EVT_DISCOVERY, EVT_WITHOUT_PARAM);
                if (discovery(ANparams.DiscoveryTxPower, 0) == DPA_OPERATION_OK)
                    notifyMainApp(EVT_DISCOVERY_OK, EVT_WITH_PARAM);
                // Get addressing info
                getAddrInfo();
                // Get bonded nodes bitmap
                getBondedNodes();
                // Get discovered nodes bitmap
                getDiscoveredNodes();

                NetworkInfo.DiscoveredNodesCount = 0;
                for (Cnt = 1; Cnt <= MAX_ADDRESS; Cnt++)
                    if (getBitValue(NetworkInfo.DiscoveredNodesMap, Cnt) == 1)
                        NetworkInfo.DiscoveredNodesCount++;

                if(NetworkInfo.DiscoveredNodesCount == NetworkInfo.BondedNodesCount)
                    break;
            }
        } else {
            Prebonding.EmtyWaveCnt++;
        }
    }

    notifyMainApp(EVT_AN_PROCESS_STOPPED, EVT_WITHOUT_PARAM);
    return(Prebonding.ReturnCode);
}
