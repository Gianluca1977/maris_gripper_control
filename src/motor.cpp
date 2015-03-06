#include "motor.h"
#include "can_driver.h"
#include "controller.h"

#include <iostream>

Motor::Motor()
{
    clear();
    //std::cout << "Motor ID = " << ID << std::endl;
}

Motor::Motor(int id) : ID(id)
{
    Motor();
}

Motor::~Motor()
{
}

void Motor::setID(int id)
{
    ID = id;
    //std::cout << "Motor ID = " << ID << std::endl;
}

void Motor::setLimits(long maxVel, long maxAcc, long maxDeacc, long maxPCurr, long maxCCurr)
{
    MaxVel = maxVel;
    MaxAcc = maxAcc;
    MaxDeacc = maxDeacc;
    MaxPeakCurr = maxPCurr;
    MaxContCurr = maxCCurr;
}

void Motor::stop()
{
    send_faulhaber_cmd_to_node(FAULHABER_VELOCITY_CMD, 0, ID);
}

void Motor::emergencyStop()
{
    send_cmd_to_node(CMD_OPENCAN_DISABLE_VOLTAGE,ID);
    //send_NMT_to_node(CMD_OPENCAN_NMT_STOP, ID);
}

bool Motor::checkStopped()
{
    if(Velocity >= -1 && Velocity <=1 ) return true;
    else return false;
}

void Motor::disable()
{
    send_faulhaber_disable_to_node(ID);
}

void Motor::enable()
{
    send_faulhaber_enable_to_node(ID);
}

void Motor::reset()
{
    //disable();
    //enable();
    clear();
    send_NMT_to_node(CMD_OPENCAN_NMT_RESET, ID);
}

void Motor::clear()
{
    Control = 0;
    Operational = false;
    TargetReached = false;
    Fault = false;
    Position = ERR_VAL;
    Velocity = ERR_VAL;
    MaxPosGrad = ERR_VAL;
    BootUp = false;
    State = STATE_NONE;
    Old_State = STATE_NONE;
}

void Motor::queryAsyncVel()
{
    send_faulhaber_cmd_to_node(FAULHABER_GET_ACUTAL_VELOCITY,0,ID);
    send_cmd_to_node(CMD_OPENCAN_TPDO2_REQ,5,FAULHABER_GET_ACUTAL_VELOCITY,ID);
}

void Motor::moveVel(long vel)
{
    send_faulhaber_cmd_to_node(FAULHABER_VELOCITY_CMD, vel, ID);
}

void Motor::movePosAbs(long absValue)
{
    send_faulhaber_cmd_to_node(FAULHABER_ABSOLUTE_POSITION_CMD, absValue, ID);
    send_faulhaber_cmd_to_node(FAULHABER_IN_MOTION_CMD, 0, ID);
}

void Motor::movePosRel(long relValue)
{
    send_faulhaber_cmd_to_node(FAULHABER_RELATIVE_POSITION_CMD, relValue, ID);
    send_faulhaber_cmd_to_node(FAULHABER_IN_MOTION_CMD, 0, ID);
}

void Motor::loadPosAbs(long absValue)
{
    send_faulhaber_cmd_to_node(FAULHABER_ABSOLUTE_POSITION_CMD, absValue, ID);
}

void Motor::startMovePos(){
    send_faulhaber_cmd_to_node(FAULHABER_IN_MOTION_CMD, 0, ID);
}

void Motor::setHomePosition()
{
    send_faulhaber_cmd_to_node(FAULHABER_SET_HOME_POSITION_CMD, 0, ID);
}

void Motor::setMaxVel(long maxVel)
{
    MaxVel = maxVel;
    send_faulhaber_cmd_to_node(FAULHABER_SET_MAX_SPEED, maxVel, ID);
}

void Motor::setMaxAcc(long maxAcc)
{
    MaxAcc = maxAcc;
    send_faulhaber_cmd_to_node(FAULHABER_SET_MAX_ACC, maxAcc, ID);
}

void Motor::setMaxDeacc(long maxDeacc)
{
    MaxDeacc = maxDeacc;
    send_faulhaber_cmd_to_node(FAULHABER_SET_MAX_DEC, maxDeacc, ID);
}

void Motor::setMaxPeakCurr(long maxPCurr)
{
    MaxPeakCurr = maxPCurr;
    send_faulhaber_cmd_to_node(FAULHABER_SET_MAX_PEAK_CURR, maxPCurr, ID);
}

void Motor::setMaxContCurr(long maxCCurr)
{
    MaxContCurr = maxCCurr;
    send_faulhaber_cmd_to_node(FAULHABER_SET_MAX_CONT_CURR, maxCCurr, ID);
}

void Motor::init(int phase) {

    switch(phase){
    case INIT_BOOTUP_DEV:
        send_NMT_to_node(CMD_OPENCAN_NMT_RSTCOMM, ID);
        break;

    case INIT_START_DEV:
        send_NMT_to_node(CMD_OPENCAN_NMT_START, ID);
        break;

    case INIT_SHUTDOWN_DEV:
        send_cmd_to_node(CMD_OPENCAN_SHUTDOWN, ID);
        break;

    case INIT_SWITCH_ON_DEV:
        send_cmd_to_node(CMD_OPENCAN_SWITCHON, ID);
        break;

    case INIT_ENABLE_OP_DEV:
        send_cmd_to_node(CMD_OPENCAN_ENABLEOP, ID);
        break;

    case INIT_ENABLED:
        send_cmd_to_node(CMD_OPENCAN_SET_FAULHABER_MODE, ID);
        break;

    case INIT_TRACE_CONF:
        // setting trace CMD_OPENCAN_SET_TPDO3_TRACE_CONF
        setMaxVel(MaxVel);	//#Set standard max Speed

        setMaxAcc(MaxAcc);	//#Set standard max Acceleration

        setMaxDeacc(MaxDeacc);	//#Set standard max Deceleration

        setMaxPeakCurr(MaxPeakCurr); //#Set max Peak Current
        setMaxContCurr(MaxContCurr); //#Set max Continuous Current

        send_cmd_to_node(CMD_OPENCAN_SET_TPDO3_TRACE_CONF, ID);
        break;
    case INIT_SYNC_CONF:
        //configuring TxPDO3 to response on sync CMD_OPENCAN_SET_TPDO3_SYNC_CONF
        send_cmd_to_node(CMD_OPENCAN_SET_TPDO3_SYNC_CONF, ID);
        break;
    case INIT_TPDO1_CONF:
        //configuring TxPDO1 to response on change
        send_cmd_to_node(CMD_OPENCAN_TPDO1_REQ_ON_CHANGE, ID);
        Operational = true;
        break;
    case INIT_FAULHABER_CONF:
        //enable all
        //for(unsigned int i=0; i < DOF_ ; i++) if(ID != 0)
        send_cmd_to_node(CMD_OPENCAN_SET_FAULHABER_ENABLE, ID);
        break;
    case INIT_FAULT:
        //device is in fault state CMD_OPENCAN_FAULTRESET
        send_cmd_to_node(CMD_OPENCAN_FAULTRESET, ID);
        reset();
        break;

    case INIT_CLOSE:	//exit condition
    default:
        break;
    }
}

void Motor::stateUpdate(unsigned char data[])
{
    Old_State = State;

    State_byte[0] = data[0];
    State_byte[1] = data[1];

    TargetReached = State & TARGET_MASK;

    //KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, "States Update", "ID = %X,  State = %X%X, %X, Old_State = %X", ID, msg.DATA[1], msg.DATA[0], State, Old_State);

    Operational = ((State & STATUS_WORD_MASK) == OPERATION_ENABLED);
    Fault = State & FAULT_STATE;
}








