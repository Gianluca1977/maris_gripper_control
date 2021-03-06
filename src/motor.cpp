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
    send_cmd_to_node(CMD_OPENCAN_SWITCHON, ID);
    send_cmd_to_node(CMD_OPENCAN_ENABLEOP, ID);
    //send_faulhaber_enable_to_node(ID);
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
    PositionGrad = 0;
    PositionRaw = 0;
    OldPosition = 0;
    updateTime = 0;
    Velocity = ERR_VAL;
    VelocityGrad = 0;
    VelocityRaw = 0;
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
    //send_faulhaber_cmd_to_node(FAULHABER_VELOCITY_CMD, vel*jointReduction/360, ID);
    send_faulhaber_cmd_to_node(FAULHABER_VELOCITY_CMD, vel, ID);
}

void Motor::movePosAbs(long absValue)
{
    send_faulhaber_cmd_to_node(FAULHABER_ABSOLUTE_POSITION_CMD, absValue, ID);
    send_faulhaber_cmd_to_node(FAULHABER_IN_MOTION_CMD, 0, ID);
}

void Motor::movePosAbsGrad(double absPos)
{
    movePosAbs(absPos*jointReduction/360);
}

void Motor::movePosAbsRad(double absPos)
{
    movePosAbs(absPos*jointReduction/(2*PI));
}

void Motor::movePosRel(long relValue)
{
    send_faulhaber_cmd_to_node(FAULHABER_RELATIVE_POSITION_CMD, relValue*jointReduction/360, ID);
    send_faulhaber_cmd_to_node(FAULHABER_IN_MOTION_CMD, 0, ID);
}

void Motor::loadPosAbs(long absValue)
{
    send_faulhaber_cmd_to_node(FAULHABER_ABSOLUTE_POSITION_CMD, absValue*jointReduction/360, ID);
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

        send_cmd_to_node(CMD_OPENCAN_SET_RPDO2_SYNC_CONF, ID); // configure acyclic FAULHABER command feedback
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

void Motor::processCanMsg(int cmdID, unsigned char data[])
{
    if(cmdID == TPDO3_COBID) //0x380(896D) - trace response
    {
        long tmp_curr = data[5];
        Current = (tmp_curr << 8) + data[4];

        PositionRaw = pcanData2Double(data,0);
        PositionGrad = PositionRaw*360/jointReduction;
        Position = ((double) PositionGrad)*PI/180.0;

        long long actualTime = llround(KAL::GetTime());
        VelocityRaw = (long)(PositionRaw - OldPositionRaw)/((actualTime - updateTime)*1E-9);
        Velocity = VelocityRaw*2*PI/jointReduction;

        OldPositionRaw = PositionRaw;
        OldPosition = Position;
        updateTime = actualTime;
        //KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, "States Update", "ID = %X,  curr = %X%X, pos = %X%X%X%X", ID, data[5], data[4], data[3], data[2], data[1], data[0] );

    }//if(cmdID == TPDO3_COBID)
    else if(cmdID == TPDO2_COBID) //0x280(640D) - TxPDO2 response
    {
        if(data[0] == 0x2B)
        {
            Velocity = pcanData2Double(data,1)*360/jointReduction;
            KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, "States Update", "ID = %X,  vel = %X%X%X%X", ID, data[4], data[3], data[2], data[1]);
        }
        else if(data[0] == 0x40)
        {
            PositionGrad = pcanData2Double(data,1)*360/jointReduction;
            KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, "States Update", "ID = %X,  pos = %X%X%X%X", ID, data[4], data[3], data[2], data[1]);
        }
    }//else if(cmdID == TPDO2_COBID)
    else if(cmdID == TPDO1_COBID) //0x180(384D) - TxPDO1 (statusWord)
    {
        stateUpdate(data);

        if(stateChanged())
            switch(State & STATUS_WORD_MASK){
            case SWITCH_ON_DISABLED:
                KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, TRIGGERTASK_NAME, "Motor %d Switch On Disabled.", ID);
                break;
            case READY_TO_SWITCH_ON:
                KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, TRIGGERTASK_NAME, "Motor %d Ready to Switch On.", ID);
                break;
            case SWITCHED_ON:
                KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, TRIGGERTASK_NAME, "Motor %d Switched On.", ID);
                break;
            case OPERATION_ENABLED:
                KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, TRIGGERTASK_NAME, "Motor %d Operation Enabled.", ID);
                break;
            case FAULT_STATE:
                KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, TRIGGERTASK_NAME, "Motor %d Fault Detected. Reset Motor.", ID);
                break;
            case QUICKSTOP:
                break;
            default:
                break;
            }

    } else if(cmdID == BOOTUP_COBID) //0x700 (1972D) - Boot up message
    {
        BootUp = true;
        KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, TRIGGERTASK_NAME, "Motor %d booted up.", ID);
    }
}

void Motor::stateUpdate(unsigned char data[])
{
    Old_State = State;

    State_byte[0] = data[0];
    State_byte[1] = data[1];

    TargetReached = State & TARGET_MASK;

    //KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, "States Update", "ID = %X,  State = %X%X, %X, Old_State = %X", ID, data[1], data[0], State, Old_State);

    Operational = ((State & STATUS_WORD_MASK) == OPERATION_ENABLED);
    Fault = State & FAULT_STATE;
}

double Motor::pcanData2Double(unsigned char data[], int offset)
{
    long long int tmp_data = data[3 + offset];
    tmp_data = (tmp_data << 8) + data[2 + offset];
    tmp_data = (tmp_data << 8) + data[1 + offset];
    tmp_data = (tmp_data << 8) + data[0 + offset];


    if(tmp_data & SIGN_MASK) {
        //DEBUG("changing pos sign..\n");
        //DEBUG("~(tmp_data-1) = 0x%X\n",(( VALUE_MASK & ~tmp_data) +1));
        return -1.0*(double)(( VALUE_MASK & ~tmp_data ) +1);
    }
    else return (double) tmp_data;
}






