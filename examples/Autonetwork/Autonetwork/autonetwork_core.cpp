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
#include "autonetwork.h"

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
uint8_t readPrebondedMID(uint16_t Addr, uint32_t *MID, uint32_t *UserData);
uint8_t disableRemoteBonding(uint16_t Addr);
uint8_t frcSend(uint8_t Len);
uint8_t frcExtraResult(void);
uint8_t authorizeBond(uint8_t ReqAddr, uint32_t MID);
uint8_t removeBond(uint16_t Addr);
uint8_t removeBondRestart(uint16_t Addr);
uint8_t checkNewMID(uint32_t NewMID);
void terminateProcess(void);


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

    for(CheckAddr=1; CheckAddr<MAX_ADDRESS; CheckAddr++){
        if(isNodeBonded(CheckAddr) == 0) return CheckAddr;
    }

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

    if (Time != 0){

        while (Time--){
            delay(1);

            // Check for process termination by user
            if (Serial.available() > 0){
                InputChar = Serial.read();
                if ((InputChar == '\r') || (InputChar == '\n')){
                    if (InputCharMem == 'q' || InputCharMem == 'Q'){
                        InputCharMem = 0;
                        Terminate = ERR_PROCESS_TERMINATED;
                        return;
                    }
                }
                else InputCharMem = InputChar;
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
    if(notify_cb != NULL){
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
    while((OpResult = dpaSendRequest(DpaRequest, DataSize, Timeout)) == DPA_OPERATION_IN_PROGRESS);

    if (NotifyException == false){
        if (OpResult == DPA_CONFIRMATION_ERR)
            notifyMainApp(EVT_DPA_CONFIRMATION_TIMEOUT, EVT_WITHOUT_PARAM);

        if (OpResult == DPA_RESPONSE_ERR)
            notifyMainApp(EVT_DPA_RESPONSE_TIMEOUT, EVT_WITHOUT_PARAM);
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

    // Get addresing info
    MyDpaPacket.NADR = COORDINATOR_ADDRESS;
    MyDpaPacket.HWPID = HWPID_DoNotCheck;
    MyDpaPacket.PNUM = PNUM_COORDINATOR;
    MyDpaPacket.PCMD = CMD_COORDINATOR_ADDR_INFO;
    OpResult = sendMyDpaRequest(&MyDpaPacket, 0, 500);

    if (OpResult == DPA_OPERATION_OK){
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
    OpResult = sendMyDpaRequest(&MyDpaPacket, 0, 500);

    if (OpResult == DPA_OPERATION_OK){
        memcpy(NetworkInfo.DiscoveredNodesMap, (void*)&MyDpaPacket.DpaMessage.Response.PData[0], NODE_BITMAP_SIZE);
    }

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
    OpResult = sendMyDpaRequest(&MyDpaPacket, 0, 500);

    if (OpResult == DPA_OPERATION_OK){
        memcpy(NetworkInfo.BondedNodesMap, (void*)&MyDpaPacket.DpaMessage.Response.PData[0], NODE_BITMAP_SIZE);
    }

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
    return(sendMyDpaRequest(&MyDpaPacket, 0, 500));
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
    MyDpaPacket.DpaMessage.PerCoordinatorRemoveRebondBond_Request.BondAddr = BondAddr;

    return(sendMyDpaRequest(&MyDpaPacket, sizeof(TPerCoordinatorRemoveRebondBond_Request), 500));
}

/**
 * Discovery;
 *
 * @param - TxPower discovery Tx power
 * @param - MaxAddr maxinal address for discovering nodes
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

    OpResult = sendMyDpaRequest(&MyDpaPacket, sizeof(TPerCoordinatorDiscovery_Request), (uint16_t)(NetworkInfo.BondedNodesCount) * 40 + 10000);

    if (OpResult == DPA_OPERATION_OK){
        Prebonding.Param = MyDpaPacket.DpaMessage.PerCoordinatorDiscovery_Response.DiscNr;
    }

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

    return(sendMyDpaRequest(&MyDpaPacket, sizeof(TPerCoordinatorSetDpaParams_Request_Response), 500));
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

    return(sendMyDpaRequest(&MyDpaPacket, sizeof(TPerCoordinatorSetHops_Request_Response), 500));
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
    return(sendMyDpaRequest(&MyDpaPacket, Len, (NetworkInfo.BondedNodesCount + 1) * 40 + 500));
}

/**
 * Read remotely bonded module ID
 *
 * @param - Addr destination address of specified TR module
 * @param - MID pointer to list of prebonded modules ID
 * @param - UserData pointer to list of prebonded modules UserData
 * @return - result of DPA operation
 *
 */
uint8_t readPrebondedMID(uint16_t Addr, uint32_t *MID, uint32_t *UserData)
{
    uint8_t OpResult;
    uint8_t Cnt;

    // Read prebonded MID from [C] or [N]
    MyDpaPacket.NADR = Addr;
    MyDpaPacket.HWPID = HWPID_DoNotCheck;
    if (Addr == COORDINATOR_ADDRESS){
        // Read MID from coordinator
        MyDpaPacket.PNUM = PNUM_COORDINATOR;
        MyDpaPacket.PCMD = CMD_COORDINATOR_READ_REMOTELY_BONDED_MID;
    }
    else{
        // Read MID from node
        MyDpaPacket.PNUM = PNUM_NODE;
        MyDpaPacket.PCMD = CMD_NODE_READ_REMOTELY_BONDED_MID;
    }

    // send request and wait for result
    while ((OpResult = dpaSendRequest(&MyDpaPacket, 0, 500)) == DPA_OPERATION_IN_PROGRESS);

    if (OpResult == DPA_OPERATION_OK){
        // read number of prebonded nodes
        Prebonding.PrebondedMIDcount = dpaGetRxExtraDataSize() / 8;
        // read prebonded MIDs and UserData
        for (Cnt=0; Cnt<Prebonding.PrebondedMIDcount; Cnt++){
            MID[Cnt] = (uint32_t)MyDpaPacket.DpaMessage.PerCoordinatorNodeReadRemotelyBondedMID_Response.PrebondedNodes[Cnt].MID[0];
            MID[Cnt] |= ((uint32_t)MyDpaPacket.DpaMessage.PerCoordinatorNodeReadRemotelyBondedMID_Response.PrebondedNodes[Cnt].MID[1] << 8);
            MID[Cnt] |= ((uint32_t)MyDpaPacket.DpaMessage.PerCoordinatorNodeReadRemotelyBondedMID_Response.PrebondedNodes[Cnt].MID[2] << 16);
            MID[Cnt] |= ((uint32_t)MyDpaPacket.DpaMessage.PerCoordinatorNodeReadRemotelyBondedMID_Response.PrebondedNodes[Cnt].MID[3] << 24);

            UserData[Cnt] = (uint32_t)MyDpaPacket.DpaMessage.PerCoordinatorNodeReadRemotelyBondedMID_Response.PrebondedNodes[Cnt].UserData[0];
            UserData[Cnt] |= ((uint32_t)MyDpaPacket.DpaMessage.PerCoordinatorNodeReadRemotelyBondedMID_Response.PrebondedNodes[Cnt].UserData[1] << 8);
            UserData[Cnt] |= ((uint32_t)MyDpaPacket.DpaMessage.PerCoordinatorNodeReadRemotelyBondedMID_Response.PrebondedNodes[Cnt].UserData[2] << 16);
            UserData[Cnt] |= ((uint32_t)MyDpaPacket.DpaMessage.PerCoordinatorNodeReadRemotelyBondedMID_Response.PrebondedNodes[Cnt].UserData[3] << 24);
        }
    }
    return(OpResult);
}

/**
 * Disable remote bonding
 *
 * @param - Addr destination address of specified TR module
 * @return - result of DPA operation
 *
 */
uint8_t disableRemoteBonding(uint16_t Addr)
{
    // Enable remote bonding
    MyDpaPacket.NADR = Addr;
    MyDpaPacket.HWPID = HWPID_DoNotCheck;
    if (Addr == COORDINATOR_ADDRESS){
        // Coordinator
        MyDpaPacket.PNUM = PNUM_COORDINATOR;
        MyDpaPacket.PCMD = CMD_COORDINATOR_ENABLE_REMOTE_BONDING;
    }
    else{
        // Node
        MyDpaPacket.PNUM = PNUM_NODE;
        MyDpaPacket.PCMD = CMD_NODE_ENABLE_REMOTE_BONDING;
    }
    MyDpaPacket.DpaMessage.PerCoordinatorNodeEnableRemoteBonding_Request.BondingMask = 0;
    MyDpaPacket.DpaMessage.PerCoordinatorNodeEnableRemoteBonding_Request.Control = 0;
    MyDpaPacket.DpaMessage.PerCoordinatorNodeEnableRemoteBonding_Request.UserData[0] = 0;
    MyDpaPacket.DpaMessage.PerCoordinatorNodeEnableRemoteBonding_Request.UserData[1] = 0;
    MyDpaPacket.DpaMessage.PerCoordinatorNodeEnableRemoteBonding_Request.UserData[2] = 0;
    MyDpaPacket.DpaMessage.PerCoordinatorNodeEnableRemoteBonding_Request.UserData[3] = 0;

    return(sendMyDpaRequest(&MyDpaPacket, sizeof(TPerCoordinatorNodeEnableRemoteBonding_Request), 500));
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
    return(sendMyDpaRequest(&MyDpaPacket, 0, 500));
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
    return(sendMyDpaRequest(&MyDpaPacket, 0, 500));
}

/**
 * FRC Send
 *
 * @param - Len length of FRC command
 * @return - result of DPA operation
 *
 */
uint8_t frcSend(uint8_t Len)
{
    uint8_t OpResult;

    // Send FRC
    MyDpaPacket.NADR = COORDINATOR_ADDRESS;
    MyDpaPacket.HWPID = HWPID_DoNotCheck;
    MyDpaPacket.PNUM = PNUM_FRC;
    MyDpaPacket.PCMD = CMD_FRC_SEND;

    // Timeout for IQRF = Bonded Nodes x 130 + _RESPONSE_FRC_TIME_xxx_MS + 250 [ms]
    // + overhead for the = 2000 [ms]
    OpResult = sendMyDpaRequest(&MyDpaPacket, Len, (uint16_t)(NetworkInfo.BondedNodesCount) * 130 + 40 + 250 + 2000);

    if (OpResult == DPA_OPERATION_OK)
        memcpy((void*)&Prebonding.FrcData[FRC_DATA_OFFSET], (void*)&MyDpaPacket.DpaMessage.Response.PData[1], FRC_DATA_SIZE);

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

    OpResult = sendMyDpaRequest(&MyDpaPacket, 0, 500);

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

    for (uint8_t AuthorizeRetry = ANparams.AuthorizeRetries; AuthorizeRetry != 0; AuthorizeRetry--){
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

        OpResult = sendMyDpaRequest(&MyDpaPacket, sizeof(TPerCoordinatorAuthorizeBond_Request), (uint16_t)(NetworkInfo.BondedNodesCount) * 50 + 500);

        delayMS((uint32_t)(NetworkInfo.BondedNodesCount) * 50 + 150);

        if(AuthorizeRetry != 1){
            Prebonding.Param = ReqAddr;
            notifyMainApp(EVT_COOR_REMOVING_BOND, EVT_WITH_PARAM);
            removeBondedNode(ReqAddr);
        }
        else{
            // Authorization OK ?
            if (OpResult == DPA_OPERATION_OK){
                // Yes
                Prebonding.Param = MyDpaPacket.DpaMessage.PerCoordinatorAuthorizeBond_Response.DevNr;
                setBitValue(NetworkInfo.BondedNodesMap, ReqAddr);
                return(OpResult);
            }
        }
    }

    return(OpResult);
}

/**
 * Remove bond
 *
 * @param - Addr address of removed node
 * @return - result of DPA operation
 *
 */
uint8_t removeBond(uint16_t Addr)
{
    // Remove bond
    MyDpaPacket.NADR = Addr;
    MyDpaPacket.HWPID = HWPID_DoNotCheck;
    MyDpaPacket.PNUM = PNUM_NODE;
    MyDpaPacket.PCMD = CMD_NODE_REMOVE_BOND;

    return(sendMyDpaRequest(&MyDpaPacket, 0, 500));
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
 * Check new MID
 *
 * @param - NewMID new MID to check if it is already in MID list
 * @return - AN_OK / AN_ERROR
 *
 */
uint8_t checkNewMID(uint32_t NewMID)
{
    for (uint8_t i = 0; i < Prebonding.MIDcount; i++){
        // Is the newMID in prebondedMIDs list ?
        if (Prebonding.MIDlist[i] == NewMID)
            return(AN_ERROR);
    }

    // MID OK
    return(AN_OK);
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
 * Terminate autonetwork process
 *
 * @param - none
 * @return - none
 *
 */
void terminateProcess(void)
{
    // Disable bonding at nodes and coordinator
    notifyMainApp(EVT_NODE_DISABLE_PREBONDING, EVT_WITHOUT_PARAM);
    disableRemoteBonding(BROADCAST_ADDRESS);
    notifyMainApp(EVT_COOR_DISABLE_PREBONDING, EVT_WITHOUT_PARAM);
    disableRemoteBonding(COORDINATOR_ADDRESS);
    notifyMainApp(EVT_AN_PROCESS_STOPPED, EVT_WITHOUT_PARAM);
    ledR(BROADCAST_ADDRESS, CMD_LED_SET_OFF);
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
    uint32_t  Delay_ms;
    uint8_t Cnt;

    // initialization
    Terminate = 0x00;
    memcpy((void*)&ANparams, (void*)Parameters, sizeof(T_AN_PARAMS));
    AutonetworkState.NewtorkInfo = &NetworkInfo;
    AutonetworkState.PrebondingInfo = &Prebonding;
    AutonetworkState.Params = &ANparams;

    // Notify main application
    notifyMainApp(EVT_AN_PROCESS_STARTED, EVT_WITHOUT_PARAM);

    // Get addresing info
    notifyMainApp(EVT_GET_NETWORK_INFO, EVT_WITHOUT_PARAM);
    getAddrInfo();
    Prebonding.OrigNodesCount = NetworkInfo.BondedNodesCount;

    // Get bonded nodes bitmap
    getBondedNodes();

    // Get discovered nodes bitmap
    getDiscoveredNodes();

    // Set DPA hops
    setHops(0xff, 0xff);

    // Set DPA prarameter
    setDPAParam(0x00);
    Prebonding.NextAddr = MAX_ADDRESS;
    for (Prebonding.Round = 1; NetworkInfo.BondedNodesCount != MAX_ADDRESS; Prebonding.Round++){

        // Initialize variables
        Prebonding.MIDcount = 0;
        Prebonding.NewNode = 0x00;
        // Reset bitmap of nodes provided prebonding, new adrress and MID
        memset(Prebonding.NewNodesMap, 0, NODE_BITMAP_SIZE);
        memset((uint8_t *)Prebonding.MIDlist, 0, MID_BUFFER_SIZE * sizeof(uint32_t));

        // Set bonding mask
        Prebonding.BondingMask = 0;                // Default value for 0 nodes
        if(NetworkInfo.BondedNodesCount > 0){
            uint8_t X = NetworkInfo.BondedNodesCount;
            Prebonding.BondingMask = 0xff;         // Default value for > 127 nodes
            while((X & 0x80) == 0){
                X <<= 1;
                Prebonding.BondingMask >>= 1;
            }
        }

        // 100ms unit
        Prebonding.WTimeout = (uint16_t)(((long)ANparams.TemporaryAddressTimeout * 1000) / 100);
        // Check prebondingInterval, minimal interval is 15 sec.
        if(ANparams.PrebondingInterval < AN_MIN_PREBONDING_TIME)
            ANparams.PrebondingInterval = AN_MIN_PREBONDING_TIME;
        Prebonding.WaitBonding = ANparams.PrebondingInterval;
        Prebonding.WaitBonding10ms = Prebonding.WaitBonding * 100;

        // Notify main application
        notifyMainApp(EVT_ROUND_START, EVT_WITH_PARAM);

        // Send batch command to all nodes, if any
        if(NetworkInfo.BondedNodesCount > 0){
            // Notify main application
            notifyMainApp(EVT_NODE_ENABLE_PREBONDING, EVT_WITH_PARAM);

            // LEDR = 1
            MyDpaPacket.DpaMessage.Request.PData[0] = 5;
            MyDpaPacket.DpaMessage.Request.PData[1] = PNUM_LEDR;
            MyDpaPacket.DpaMessage.Request.PData[2] = CMD_LED_SET_ON;
            MyDpaPacket.DpaMessage.Request.PData[3] = HWPID_DoNotCheck & 0xff;
            MyDpaPacket.DpaMessage.Request.PData[4] = HWPID_DoNotCheck >> 0x08;
            // Enable prebonding
            MyDpaPacket.DpaMessage.Request.PData[5] = 11;
            MyDpaPacket.DpaMessage.Request.PData[6] = PNUM_NODE;
            MyDpaPacket.DpaMessage.Request.PData[7] = CMD_NODE_ENABLE_REMOTE_BONDING;
            MyDpaPacket.DpaMessage.Request.PData[8] = HWPID_DoNotCheck & 0xff;
            MyDpaPacket.DpaMessage.Request.PData[9] = HWPID_DoNotCheck >> 0x08;
            MyDpaPacket.DpaMessage.Request.PData[10] = Prebonding.BondingMask;
            MyDpaPacket.DpaMessage.Request.PData[11] = 1;
            MyDpaPacket.DpaMessage.Request.PData[12] = Prebonding.WTimeout & 0xff;
            MyDpaPacket.DpaMessage.Request.PData[13] = Prebonding.WTimeout >> 8;
            MyDpaPacket.DpaMessage.Request.PData[14] = 0;
            MyDpaPacket.DpaMessage.Request.PData[15] = 0;
            // Send P2P packet to allow prebonding
            MyDpaPacket.DpaMessage.Request.PData[16] = 9;
            MyDpaPacket.DpaMessage.Request.PData[17] = PNUM_USER,
            MyDpaPacket.DpaMessage.Request.PData[18] = 0;
            MyDpaPacket.DpaMessage.Request.PData[19] = HWPID_DoNotCheck & 0xff;
            MyDpaPacket.DpaMessage.Request.PData[20] = HWPID_DoNotCheck >> 0x08;
            MyDpaPacket.DpaMessage.Request.PData[21] = 0x55;
            MyDpaPacket.DpaMessage.Request.PData[22] = Prebonding.WaitBonding10ms & 0xff;
            MyDpaPacket.DpaMessage.Request.PData[23] = Prebonding.WaitBonding10ms >> 8 ;
            MyDpaPacket.DpaMessage.Request.PData[24] = NetworkInfo.BondedNodesCount + 3;
            // EndBatch
            MyDpaPacket.DpaMessage.Request.PData[25] = 0;
            // Broadcast batch command
            sendBatch(BROADCAST_ADDRESS, 26);
            delayMS(((uint32_t)(NetworkInfo.BondedNodesCount) + 1) * 40 + (uint32_t)(NetworkInfo.BondedNodesCount) * 60);
            // Terminate process ?
            if(Terminate == ERR_PROCESS_TERMINATED){
                terminateProcess();
                return(ERR_PROCESS_TERMINATED);
            }
        }

        // Send batch command to coordinator
        // Enable prebonding
        MyDpaPacket.DpaMessage.Request.PData[0] = 11;
        MyDpaPacket.DpaMessage.Request.PData[1] = PNUM_COORDINATOR;
        MyDpaPacket.DpaMessage.Request.PData[2] = CMD_COORDINATOR_ENABLE_REMOTE_BONDING;
        MyDpaPacket.DpaMessage.Request.PData[3] = HWPID_DoNotCheck & 0xff;
        MyDpaPacket.DpaMessage.Request.PData[4] = HWPID_DoNotCheck >> 0x08;
        MyDpaPacket.DpaMessage.Request.PData[5] = Prebonding.BondingMask;
        MyDpaPacket.DpaMessage.Request.PData[6] = 1;
        MyDpaPacket.DpaMessage.Request.PData[7] = Prebonding.WTimeout & 0xff;
        MyDpaPacket.DpaMessage.Request.PData[8] = Prebonding.WTimeout >> 8;
        MyDpaPacket.DpaMessage.Request.PData[9] = 0;
        MyDpaPacket.DpaMessage.Request.PData[10] = 0;
        // Send P2P packet to allow prebonding
        MyDpaPacket.DpaMessage.Request.PData[11] = 9;
        MyDpaPacket.DpaMessage.Request.PData[12] = PNUM_USER;
        MyDpaPacket.DpaMessage.Request.PData[13] = 0;
        MyDpaPacket.DpaMessage.Request.PData[14] = HWPID_DoNotCheck & 0xff;
        MyDpaPacket.DpaMessage.Request.PData[15] = HWPID_DoNotCheck >> 0x08;
        MyDpaPacket.DpaMessage.Request.PData[16] = 0x55;
        MyDpaPacket.DpaMessage.Request.PData[17] = Prebonding.WaitBonding10ms & 0xff;
        MyDpaPacket.DpaMessage.Request.PData[18] = Prebonding.WaitBonding10ms >> 8;
        MyDpaPacket.DpaMessage.Request.PData[19] = 1;
        // EndBatch
        MyDpaPacket.DpaMessage.Request.PData[20] = 0;

        // Notify main application
        notifyMainApp(EVT_COOR_ENABLE_PREBONDING, EVT_WITH_PARAM);

        // Send the batch command to [C]
        sendBatch(COORDINATOR_ADDRESS, 21);

        // Waiting for prebonding
        notifyMainApp(EVT_WAIT_PREBONDING, EVT_WITH_PARAM);
        Delay_ms = (uint32_t)(Prebonding.WaitBonding) * 1000 + 1000;
        delayMS(Delay_ms);
        // Terminate process ?
        if(Terminate == ERR_PROCESS_TERMINATED){
            terminateProcess();
            return(ERR_PROCESS_TERMINATED);
        }

        // Disable remote bonding at coordinator
        notifyMainApp(EVT_COOR_DISABLE_PREBONDING, EVT_WITHOUT_PARAM);
        disableRemoteBonding(COORDINATOR_ADDRESS);

        // Try to read prebonded MID from coordinator
        if (readPrebondedMID(COORDINATOR_ADDRESS, &Prebonding.PrebondedMID[0], &Prebonding.PrebondedUserData[0]) == DPA_OPERATION_OK){
            for (Cnt=0; Cnt<Prebonding.PrebondedMIDcount; Cnt++){
                Prebonding.MID = Prebonding.PrebondedMID[Cnt];
                Prebonding.UserData = Prebonding.PrebondedUserData[Cnt];
                // Check the new MID
                if(checkNewMID(Prebonding.MID) == 0){
                    // Notify main application
                    notifyMainApp(EVT_COOR_READ_MID, EVT_WITH_PARAM);
                    Prebonding.MIDlist[Prebonding.MIDcount++] = Prebonding.MID;

                    // Terminate process ?
                    if(Terminate == ERR_PROCESS_TERMINATED){
                        terminateProcess();
                        return(ERR_PROCESS_TERMINATED);
                    }
                }
            }
        }

        // Any bonded nodes ?
        if(NetworkInfo.BondedNodesCount){
            // Send FRC command Prebonding to get bitmap of nodes provided prebonding
            notifyMainApp(EVT_FRC_DISABLE_PREBONDING, EVT_WITHOUT_PARAM);
            MyDpaPacket.DpaMessage.Request.PData[0] = 0;       // Command 0 - Prebonding
            MyDpaPacket.DpaMessage.Request.PData[1] = 0x01;
            MyDpaPacket.DpaMessage.Request.PData[2] = 0x00;
            frcSend(3);
            frcExtraResult();

            // Get bitmap of nodes provided prebonding, read MID of prebonded nodes
            for (uint8_t Addr = 1; Addr <= MAX_ADDRESS; Addr++){
                // Terminate process ?
                if(Terminate == ERR_PROCESS_TERMINATED){
                    terminateProcess();
                    return(ERR_PROCESS_TERMINATED);
                }

                // Check the node is bonded
                Prebonding.Param = Addr;
                uint8_t NodeBonded = isNodeBonded(Addr);
                uint8_t Bit0OK = 0x00;

                // Bit0 is set (node sent response to FRC) ?
                if(getBitValue(Prebonding.FrcData, Addr) == 0x01){
                    if(NodeBonded)
                        Bit0OK = 0xff;
                    else
                        notifyMainApp(EVT_FRC_DISABLE_PREBONDING_BIT0_ERR, EVT_WITH_PARAM);
                }

                // Is Bit1 set (node provided the prebonding) ?
                if(getBitValue(Prebonding.FrcData + 32, Addr) == 0x01){
                    // Is node providing the prebonding already bonded ?
                    if(NodeBonded){
                        // Node sent response to FRC ?
                        if(Bit0OK){
                            // Read the MID of prebonded node
                            if(readPrebondedMID(Addr, &Prebonding.PrebondedMID[0], &Prebonding.PrebondedUserData[0]) == DPA_OPERATION_OK){
                              for (Cnt=0; Cnt<Prebonding.PrebondedMIDcount; Cnt++){
                                  Prebonding.MID = Prebonding.PrebondedMID[Cnt];
                                  Prebonding.UserData = Prebonding.PrebondedUserData[Cnt];
                                    // Check the current MID
                                    if(checkNewMID(Prebonding.MID) == 0x00){
                                        notifyMainApp(EVT_NODE_READ_MID, EVT_WITH_PARAM);
                                        Prebonding.MIDlist[Prebonding.MIDcount++] = Prebonding.MID;
                                        if(Prebonding.MIDcount >= (MID_BUFFER_SIZE - 1)){
                                            // Error, maximum prebonded nodes reached
                                            notifyMainApp(EVT_MAX_NODES_PREBONDED, EVT_WITHOUT_PARAM);
                                            terminateProcess();
                                            return(ERR_MAX_NODES_PREBONDED);
                                        }
                                    }
                                }
                            }
                        }
                    }
                    else
                        notifyMainApp(EVT_FRC_DISABLE_PREBONDING_BIT1_ERR, EVT_WITH_PARAM);
                }
            }
        }

        // Authorize prebonded nodes
        for (uint8_t Addr = 0; Addr < Prebonding.MIDcount; Addr++){
            Prebonding.MID = Prebonding.MIDlist[Addr];

            // OK, Get next free address
            Prebonding.NextAddr = nextFreeAddr();
            if (Prebonding.NextAddr == 0xff){
                // Error, no free address, terminate process
                notifyMainApp(EVT_NO_FREE_ADDRESS, EVT_WITHOUT_PARAM);
                terminateProcess();
                return(ERR_NO_FREE_ADDRESS);
            }

            // Authorize node
            notifyMainApp(EVT_AUTHORIZE_BOND, EVT_WITH_PARAM);
            if (authorizeBond(Prebonding.NextAddr, Prebonding.MIDlist[Addr]) == DPA_OPERATION_OK){
                setBitValue(Prebonding.NewNodesMap, Prebonding.NextAddr);
                Prebonding.NewNode = 0xff;
                notifyMainApp(EVT_AUTHORIZE_BOND_OK, EVT_WITH_PARAM);
            }

            delayMS(100);
        }

        // Any new node ?
        if(Prebonding.NewNode){
          delayMS(500);
            // Send FRC command Prebonding to check the new nodes
            notifyMainApp(EVT_FRC_CHECK_NEW_NODES, EVT_WITHOUT_PARAM);
            MyDpaPacket.DpaMessage.Request.PData[0] = 0;       // Command 0 - Prebonding
            MyDpaPacket.DpaMessage.Request.PData[1] = 0x01;
            MyDpaPacket.DpaMessage.Request.PData[2] = 0x00;
            frcSend(3);

            // Check new nodes
            for(uint8_t Addr = 1; Addr <= MAX_ADDRESS; Addr++){
                if(getBitValue(Prebonding.NewNodesMap, Addr)){
                    // Bit0 is cleared (new node isn't responding to FRC) ?
                    if(getBitValue(Prebonding.FrcData, Addr) == 0x00){
                        // No response from the node
                        Prebonding.Param = Addr;
                        // Remove and reset the node
                        notifyMainApp(EVT_NODE_REMOTE_UNBOND, EVT_WITH_PARAM);
                        if(removeBondRestart(Addr) != DPA_OPERATION_OK){
                             Delay_ms = (uint32_t)(NetworkInfo.BondedNodesCount + 1) * 80;
                             delayMS(Delay_ms);
                        }
                        // Remove node at coordinator too
                        notifyMainApp(EVT_COOR_REMOVING_BOND, EVT_WITH_PARAM);
                        removeBondedNode(Addr);
                    }
                }
            }

            // Remove and reset node with temporary address 0xfe
            removeBond(TEMPORARY_ADDRESS);

            // Run Discovery
            for (uint8_t DiscoveryRetry = ANparams.DiscoveryRetries; DiscoveryRetry != 0; DiscoveryRetry--){
                notifyMainApp(EVT_DISCOVERY, EVT_WITHOUT_PARAM);
                if (discovery(ANparams.DiscoveryTxPower, 0) == DPA_OPERATION_OK)
                    notifyMainApp(EVT_DISCOVERY_OK, EVT_WITH_PARAM);
                notifyMainApp(EVT_DISCOVERY_WAIT, EVT_WITHOUT_PARAM);
                for( ; ; ){
                    // Wait for finish the Discovery
                     delayMS(1000);
                    // Terminate process ?
                    if(Terminate == ERR_PROCESS_TERMINATED){
                        terminateProcess();
                        return(ERR_PROCESS_TERMINATED);
                    }
                    // Get addresing info
                    if(getAddrInfo() == DPA_OPERATION_OK)
                        break;
                }

                // Get bonded nodes bitmap
                getBondedNodes();

                // Get discovered nodes bitmap
                getDiscoveredNodes();

                NetworkInfo.DiscoveredNodesCount = 0;
                for (uint8_t Addr = 1; Addr <= MAX_ADDRESS; Addr++){
                    if (getBitValue(NetworkInfo.DiscoveredNodesMap, Addr) == 1)
                        NetworkInfo.DiscoveredNodesCount++;
                }
                // Terminate process ?
                if(Terminate == ERR_PROCESS_TERMINATED){
                    terminateProcess();
                    return(ERR_PROCESS_TERMINATED);
                }

                if(NetworkInfo.DiscoveredNodesCount == NetworkInfo.BondedNodesCount)
                    break;
            }
        }
        else
            notifyMainApp(EVT_NO_NEW_NODE_PREBONDED, EVT_WITHOUT_PARAM);
    }

    terminateProcess();
    return(AN_OK);
}
