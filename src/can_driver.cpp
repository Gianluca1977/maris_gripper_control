/*
 * can_driver.cpp
 *
 *  Created on: Aug 2, 2012
 *      Author: lar
 *
 *	- Parsing of new tactile sensors.
 */

#include "can_driver.h"

std::queue<TPCANMsg> CanInterface::msg_outqueue;
OpenCAN::OpenCANSchedulerHelper CanDriver::schedHelper_;
OpenCAN::CANOpenMsg CanDriver::tmpCANMsg;

TPCANMsg CanInterface::command[] = {
    //    #define CMD_OPENCAN_NMT_RESET 				0
    {0x000, 0x0, 0x2, 0x81 , 0x00 , 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    
    //    #define CMD_OPENCAN_NMT_PREOP  				1
    {0x000, 0x0, 0x2, 0x80 , 0x00 , 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},

    //    #define CMD_OPENCAN_NMT_STOP 				2
    {0x000, 0x0, 0x2, 0x02 , 0x00 , 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},

    //    #define CMD_OPENCAN_NMT_STOP2               3
    {0x000, 0x0, 0x2, 0x02 , 0x00 , 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},

    //    #define CMD_OPENCAN_NMT_START				4
    {0x000, 0x0, 0x2, 0x01 , 0x00 , 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},

    //    #define CMD_OPENCAN_SYNC					5
    {0x080, 0x0, 0x0, 0x00 , 0x00 , 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},

    //    #define CMD_OPENCAN_NMT_RSTCOMM 			6
    {0x000, 0x0, 0x2, 0x82 , 0x00 , 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},

    //    #define CMD_OPENCAN_NMT_BOOTUP_MSG			7
    {0x700, 0x0, 0x1, 0x00 , 0x00 , 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},

    //    #define CMD_OPENCAN_SET_TPDO3_TRACE_CONF	8
    {0x400, 0x0, 0x5, 0xC8 , 0x04 , 0x01, 0x01, 0x01, 0x00, 0x00, 0x00},

    //    #define CMD_OPENCAN_SET_TPDO3_SYNC_CONF 	9
    {0x600, 0x0, 0x8, 0x2F , 0x02 , 0x18, 0x02, 0x01, 0x00, 0x00, 0x00},

    //    #define CMD_OPENCAN_SET_TPDO1_SYNC_CONF		10
    {0x600, 0x0, 0x8, 0x2F , 0x00 , 0x18, 0x02, 0x01, 0x00, 0x00, 0x00},

    //    #define CMD_OPENCAN_SET_RPDO2_SYNC_CONF		11
    {0x600, 0x0, 0x8, 0x2F , 0x01 , 0x14, 0x02, 0x01, 0x00, 0x00, 0x00},

    //    #define CMD_OPENCAN_RPDO2					12
    {0x300, 0x0, 0x5, 0x00 , 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},

    //    #define CMD_OPENCAN_SET_FAULHABER_MODE 		13
    {0x300, 0x0, 0x5, 0xFD , 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00},

    //    #define CMD_OPENCAN_SET_FAULHABER_ENABLE	14
    {0x300, 0x0, 0x5, 0xF0 , 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},

    //    #define CMD_OPENCAN_SET_FAULHABER_DISABLE  	15
    {0x300, 0x0, 0x5, 0x08 , 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},

    //    #define CMD_OPENCAN_SET_FAULHABER_VELOCITY  16
    {0x300, 0x0, 0x5, 0x93 , 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},

    //    #define CMD_OPENCAN_SET_CIA402_POS_MODE 	17
    {0x600, 0x0, 0x8, 0x2F , 0x60, 0x60, 0x00, 0x01, 0x00, 0x00, 0x00},

    //    #define CMD_OPENCAN_SET_CIA402_VEL_MODE		18
    {0x600, 0x0, 0x8, 0x2F , 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00},

    //    #define CMD_OPENCAN_SET_CIA402_HOME_MODE	19
    {0x600, 0x0, 0x8, 0x2F , 0x60, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00},

    //    #define CMD_OPENCAN_SET_CIA402_TARGET_VEL	20
    {0x600, 0x0, 0x8, 0x23 , 0xFF, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00},

    //    #define CMD_OPENCAN_TSPDO  					21
    {0x600, 0x0, 0x8, 0x00 , 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},

    //    #define CMD_OPENCAN_TPDO1_REQ				22
    {0x180, 0x0, 0x0, 0x00 , 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},

    //    #define CMD_OPENCAN_TPDO2_REQ				23
    {0x280, 0x0, 0x0, 0x00 , 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},

    //    #define CMD_OPENCAN_TPDO3_REQ				24
    {0x380, 0x0, 0x0, 0x00 , 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},

    //    #define CMD_OPENCAN_SPDO1_READ				25
    {0x600, 0x0, 0x8, 0x40 , 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},

    //    #define CMD_OPENCAN_SPDO1_READ_REP			26
    {0x580, 0x0, 0x8, 0x40 , 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},

    //    #define CMD_OPENCAN_SPDO1_WRITE				27
    {0x600, 0x0, 0x8, 0x20 , 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},

    //    #define CMD_OPENCAN_SPDO1_WRITE_REP 		28
    {0x580, 0x0, 0x8, 0x60 , 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},

    //    #define	CMD_OPENCAN_SHUTDOWN				29
    {0x200, 0x0, 0x2, 0x06 , 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},

    //    #define	CMD_OPENCAN_SWITCHON				30
    {0x200, 0x0, 0x2, 0x07 , 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},

    //    #define	CMD_OPENCAN_ENABLEOP				31
    {0x200, 0x0, 0x2, 0x0F , 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},

    //    #define	CMD_OPENCAN_DISABLEOP				32
    {0x200, 0x0, 0x2, 0x07 , 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},

    //    #define CMD_OPENCAN_FAULTRESET				33
    {0x200, 0x0, 0x2, 0x80 , 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},

    //    #define CMD_OPENCAN_SET_FAULHABER_GET_ACTUAL_VELOCITY	34
    {0x300, 0x0, 0x5, 0x2B , 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},

    //    #define CMD_OPENCAN_TPDO1_REQ_ON_CHANGE			35
    {0x600, 0x0, 0x8, 0x2F , 0x00, 0x18, 0x02, 0x00, 0x00, 0x00, 0x00},
};

CanDriver::CanDriver()
{
    char mbxName[WF_TNAME_SIZE];
    uint8_t canPayload[8];

    canPayload[0] = 0x81;
    canPayload[1] = 0x00;

    // Declare recv mailbox
    sprintf(mbxName, "CANSampleMB");

    KAL::DebugConsole::Write(LOG_LEVEL_WARNING, "CanDriver", "Calling CanDriver::CanDriver()");

#ifndef VM_TEST
    schedHelper_.Declare(mbxName);

    //
    // Subscribe to cobIds
    //

    for (int32_t nodeId = HAND_START_NODE_ID; nodeId < HAND_END_NODE_ID; nodeId++)
    {
        // Subscribe to node boot message
        schedHelper_.Subscribe(BOOTUP_COBID + nodeId);

        // Subscribe to node txPDO1 (Statusword)
        schedHelper_.Subscribe(TPDO1_COBID + nodeId);

        // Subscribe to node txPDO2 (FAULHABER data)
        schedHelper_.Subscribe(TPDO2_COBID + nodeId);

        // Subscribe to node txPDO3 (Trace data)
        schedHelper_.Subscribe(TPDO3_COBID + nodeId);
    }

    schedHelper_.AddAction(OPENCAN_MSG_KILL_ACTION, 0x000, 0x02, canPayload);

#endif
}

CanDriver::~CanDriver()
{

}

void CanInterface::send_can_msg(TPCANMsg* msg)
{
    msg_outqueue.push(*msg);
}

void CanDriver::flush_msg_queue()
{
    /* send all commands in the msg_outqueue */
    while(!msg_outqueue.empty()){
        TPCANMsg* CANMsg = &(msg_outqueue.front());
        //KAL::DebugConsole::Write(LOG_LEVEL_INFO, "CANSampleMB", "Send CAN msg ID = %03X TYPE = %02X LEN = %d DATA = %02X %02X %02X %02X %02X %02X %02X %02X", CANMsg->ID, CANMsg->MSGTYPE, CANMsg->LEN, CANMsg->DATA[0], CANMsg->DATA[1], CANMsg->DATA[2], CANMsg->DATA[3], CANMsg->DATA[4], CANMsg->DATA[5], CANMsg->DATA[6], CANMsg->DATA[7]);
#ifndef VM_TEST
        schedHelper_.Send(CANMsg, sizeof(TPCANMsg));
#endif
        msg_outqueue.pop();
    }
}

void CanDriver::send_sync_msg()
{
    //KAL::DebugConsole::Write(LOG_LEVEL_INFO, "CANSampleMB", "Send CAN SYNC msg ID = %03X TYPE = %02X LEN = %d DATA = %02X %02X %02X %02X %02X %02X %02X %02X", sync_msg.ID, sync_msg.MSGTYPE, sync_msg.LEN, sync_msg.DATA[0], sync_msg.DATA[1], sync_msg.DATA[2], sync_msg.DATA[3], sync_msg.DATA[4], sync_msg.DATA[5], sync_msg.DATA[6], sync_msg.DATA[7]);
#ifndef VM_TEST
    schedHelper_.Send(&(command[CMD_OPENCAN_SYNC]), sizeof(TPCANMsg));
#endif
}

int CanDriver::receive_can_msg()
{
    int canRet;

    memset((char *) &tmpCANMsg, 0x00, sizeof(OpenCAN::CANOpenMsg));

    //KAL::DebugConsole::Write(LOG_LEVEL_INFO, "CANSampleMB", "Waiting for CAN packets...");
#ifndef VM_TEST
    canRet = schedHelper_.Receive(&tmpCANMsg, sizeof(OpenCAN::CANOpenMsg));
#endif
    //KAL::DebugConsole::Write(LOG_LEVEL_INFO, "CANSampleMB", "CAN packet Received ID = %03X TYPE = %02X LEN = %d DATA = %02X %02X %02X %02X %02X %02X %02X %02X", tmpCANMsg.msg.ID, tmpCANMsg.msg.MSGTYPE, tmpCANMsg.msg.LEN, tmpCANMsg.msg.DATA[0], tmpCANMsg.msg.DATA[1], tmpCANMsg.msg.DATA[2], tmpCANMsg.msg.DATA[3], tmpCANMsg.msg.DATA[4], tmpCANMsg.msg.DATA[5], tmpCANMsg.msg.DATA[6], tmpCANMsg.msg.DATA[7]);

    return canRet;
}

int CanDriver::receive_can_msg(TPCANMsg &msg)
{
    int canRet = receive_can_msg();

    memcpy((void*)&msg,(void*)&tmpCANMsg.msg,sizeof(TPCANMsg));

    return canRet;
}

#define DEBUG printf

void CanInterface::print_can_msg(TPCANMsg* m)
{
    int i;

    //DEBUG("Get new can message\n");
    DEBUG("Message ID: %x\n",m->ID);
    DEBUG("Message type: %x\n",m->MSGTYPE);
    DEBUG("Message Lenght: %x\n",m->LEN);
    DEBUG("Message data: ");

    for(i=0;i<m->LEN;i++) DEBUG(" %x", m->DATA[i]);

    DEBUG("\n");

}

union tmp_data_t
{
    long long int long_data;
    char char_data[8];
};

void CanInterface::send_faulhaber_cmd_to_node(int cmd, long long int data, int ID)
{
    int i;
    TPCANMsg tmp_msg;
    
    tmp_data_t tmp_data;
    
    tmp_data.long_data = data;
    
    memcpy(&tmp_msg,&(command[CMD_OPENCAN_RPDO2]),sizeof(TPCANMsg));
    
    tmp_msg.ID += ID;

    tmp_msg.DATA[0] = cmd;
    
    for(i=0; i<4;i++) memcpy(&(tmp_msg.DATA[1+i]),tmp_data.char_data+i,1);

    //DEBUG("RPDO2_msg = \n");
    //print_can_msg(&tmp_msg);
    send_can_msg(&tmp_msg);
}

void CanInterface::send_faulhaber_disable_to_node(int ID)
{
    TPCANMsg tmp_msg;

    memcpy(&tmp_msg,&(command[CMD_OPENCAN_SET_FAULHABER_DISABLE]),sizeof(TPCANMsg));
    
    tmp_msg.ID += ID;

    send_can_msg(&tmp_msg);
}

void CanInterface::send_faulhaber_enable_to_node(int ID)
{
    TPCANMsg tmp_msg;

    memcpy(&tmp_msg,&(command[CMD_OPENCAN_SET_FAULHABER_ENABLE]),sizeof(TPCANMsg));
    
    tmp_msg.ID += ID;

    send_can_msg(&tmp_msg);
}

void CanInterface::send_cmd_to_node(int cmd, int len, long long int data, int ID)
{
    int i;
    TPCANMsg tmp_msg;

    tmp_data_t tmp_data;
    
    tmp_data.long_data = data;
    
    memcpy(&tmp_msg,&(command[cmd]),sizeof(TPCANMsg));
    
    tmp_msg.ID += ID;
    
    tmp_msg.LEN = len;
    
    for(i=0; i<len;i++) memcpy(&(tmp_msg.DATA[i]),tmp_data.char_data+i,1);

    send_can_msg(&tmp_msg);
}

void CanInterface::send_cmd_to_node(int cmd, long long int data, int ID)
{
    int i;
    TPCANMsg tmp_msg;
    
    tmp_data_t tmp_data;

    tmp_data.long_data = data;
    
    memcpy(&tmp_msg,&(command[cmd]),sizeof(TPCANMsg));
    
    tmp_msg.ID += ID;
    
    for(i=0; i < 8;i++) memcpy(&(tmp_msg.DATA[i]),tmp_data.char_data+i,1);

    send_can_msg(&tmp_msg);
}

void CanInterface::send_cmd_to_node(int cmd, int ID)
{
    TPCANMsg tmp_msg;

    memcpy(&tmp_msg,&(command[cmd]),sizeof(TPCANMsg));
    
    tmp_msg.ID += ID;
    
    send_can_msg(&tmp_msg);
}

void CanInterface::send_NMT_to_node(int cmd, int ID)
{
    TPCANMsg tmp_msg;

    memcpy(&tmp_msg,&(command[cmd]),sizeof(TPCANMsg));
    
    tmp_msg.DATA[1] = ID;
    
    send_can_msg(&tmp_msg);
}

