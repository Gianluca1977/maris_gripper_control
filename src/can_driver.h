/*
 * can_driver.h
 *
 *  Created on: Aug 2, 2012
 *      Author: lar
 */

#ifndef CAN_DRIVER_H_
#define CAN_DRIVER_H_

#include <unistd.h>  // exit
#include <vector>
#include <queue>
#include <math.h>
#include <cmath>
#include <sstream>
#include <string>
#include <boost/algorithm/string.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdlib.h>   // strtoul
#include <fcntl.h>    // O_RDWR
#include <ctype.h>

#include "wf.h"
#include "opencan.h"
#include "opencan_scheduler_helper.h"
#include "controllerdata.h"
#include "can_define.h"

#define HAND_START_NODE_ID		0x00A
#define HAND_END_NODE_ID        0x01A

#define MAX_NODE                0x20

#define SIGN_MASK		0x80000000
#define VALUE_MASK		0x0FFFFFFFF

#define COMMAND_NUM             50

#define COB_ID_MASK             0x0F80 
#define NODE_ID_MASK            0x007F 

#define STATUS_WORD_MASK        0x00FF
#define STATUS_WORD_ERR_MASK    0xFF00

#define STATE_NONE		0x0000
#define SWITCH_ON_DISABLED      0x0060
#define READY_TO_SWITCH_ON      0x0021
#define SWITCHED_ON             0x0023
#define OPERATION_ENABLED       0x0027
#define FAULT_STATE             0x0008
#define QUICKSTOP               0x0020

#define TARGET_MASK             0x0400

#define TPDO1_COBID             0x180
#define TPDO2_COBID             0x280
#define TPDO3_COBID             0x380
#define TSDO_COBID              0x580
#define BOOTUP_COBID            0x700

#define TSDO_DOWNLOAD_REPLY     0x60
#define TSDO_UPLOAD_REPLY_1B    0x4F
#define TSDO_UPLOAD_REPLY_2B    0x4B
#define TSDO_UPLOAD_REPLY_3B    0x47
#define TSDO_UPLOAD_REPLY_4B    0x43

#define FAULHABER_VELOCITY_CMD  	0x93
#define FAULHABER_ABSOLUTE_POSITION_CMD 0xB4
#define FAULHABER_RELATIVE_POSITION_CMD 0xB6
#define FAULHABER_IN_MOTION_CMD 	0x3C
#define FAULHABER_SET_HOME_POSITION_CMD 0xB8

#define FAULHABER_SET_MAX_SPEED     0x8F
#define FAULHABER_SET_MAX_ACC       0x65
#define FAULHABER_SET_MAX_DEC       0x6D
#define FAULHABER_SET_MAX_PEAK_CURR 0x81
#define FAULHABER_SET_MAX_CONT_CURR 0x80

#define FAULHABER_GET_ACUTAL_VELOCITY  0x2B

#define TARGET_VELOCITY         0x23FF6000
#define MAX_VELOCITY            0x237F6000
#define MAX_ACCELERATION        0x23836000
#define MAX_DECELERATION        0x23846000

#define CMD_OPENCAN_NMT_RESET 				0
#define CMD_OPENCAN_NMT_PREOP  				1
#define CMD_OPENCAN_NMT_STOP 				2
#define CMD_OPENCAN_NMT_STOP2               3
#define CMD_OPENCAN_NMT_START				4
#define CMD_OPENCAN_SYNC					5
#define CMD_OPENCAN_NMT_RSTCOMM 			6
#define CMD_OPENCAN_NMT_BOOTUP_MSG			7
#define CMD_OPENCAN_SET_TPDO3_TRACE_CONF	8
#define CMD_OPENCAN_SET_TPDO3_SYNC_CONF 	9
#define CMD_OPENCAN_SET_TPDO1_SYNC_CONF		10
#define CMD_OPENCAN_SET_RPDO2_SYNC_CONF		11
#define CMD_OPENCAN_RPDO2					12
#define CMD_OPENCAN_SET_FAULHABER_MODE 		13
#define CMD_OPENCAN_SET_FAULHABER_ENABLE	14
#define CMD_OPENCAN_SET_FAULHABER_DISABLE  	15
#define CMD_OPENCAN_SET_FAULHABER_VELOCITY  16
#define CMD_OPENCAN_SET_CIA402_POS_MODE 	17
#define CMD_OPENCAN_SET_CIA402_VEL_MODE		18
#define CMD_OPENCAN_SET_CIA402_HOME_MODE	19
#define CMD_OPENCAN_SET_CIA402_TARGET_VEL	20
#define CMD_OPENCAN_TSPDO  					21
#define CMD_OPENCAN_TPDO1_REQ				22
#define CMD_OPENCAN_TPDO2_REQ				23
#define CMD_OPENCAN_TPDO3_REQ				24
#define CMD_OPENCAN_SPDO1_READ				25
#define CMD_OPENCAN_SPDO1_READ_REP			26
#define CMD_OPENCAN_SPDO1_WRITE				27
#define CMD_OPENCAN_SPDO1_WRITE_REP 		28
#define	CMD_OPENCAN_SHUTDOWN				29
#define	CMD_OPENCAN_SWITCHON				30
#define	CMD_OPENCAN_ENABLEOP				31
#define	CMD_OPENCAN_DISABLEOP				32
#define CMD_OPENCAN_FAULTRESET				33
#define CMD_OPENCAN_SET_FAULHABER_GET_ACTUAL_VELOCITY	34
#define CMD_OPENCAN_TPDO1_REQ_ON_CHANGE			35

class CanInterface
{
protected:
    static std::queue<TPCANMsg> msg_outqueue; // out msg queue (to the devices)
    static TPCANMsg command[];

public:

    void send_can_msg(TPCANMsg* msg);
    void print_can_msg(TPCANMsg* m);

    void send_faulhaber_cmd_to_node(int cmd, long long int data, int ID);
    void send_faulhaber_disable_to_node(int ID);
    void send_faulhaber_enable_to_node(int ID);

    void send_cmd_to_node(int cmd, int len, long long int data, int ID);
    void send_cmd_to_node(int cmd, long long int data, int ID);
    void send_cmd_to_node(int cmd, int ID);
    void send_NMT_to_node(int cmd, int ID);
};

class CanDriver : virtual private CanInterface
{
    static OpenCAN::CANOpenMsg tmpCANMsg;
    static TPCANMsg sync_msg;

public:
    CanDriver();
	~CanDriver();

    static OpenCAN::OpenCANSchedulerHelper schedHelper_;

    void flush_msg_queue();
    void send_sync_msg();
    int receive_can_msg();
    int receive_can_msg(TPCANMsg &msg);		
};

#endif /* CAN_DRIVER_H_ */
