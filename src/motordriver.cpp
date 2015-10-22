#include "motordriver.h"

Timer MotorDriver::driverTimer = Timer();

MotorDriver::MotorDriver() : StateMachine(MotorGuard::ST_MAX_STATES)
{
    command = CMD_NONE;
}

MotorDriver::MotorDriver(int id) : StateMachine(MotorGuard::ST_MAX_STATES), Motor(id)
{
    command = CMD_NONE;
}

void MotorDriver::movePosAbs(long absValue)
{
    MotorDriverData* cmdData = new MotorDriverData({CMD_MOVE_POS_ABS, absValue});

    executeCommand(cmdData);
}

void MotorDriver::moveVel(long value)
{
    MotorDriverData* cmdData = new MotorDriverData({CMD_MOVE_VEL, value});

    executeCommand(cmdData);
}

void MotorDriver::executeCommand(MotorDriverData* cmdData)
{
    BEGIN_TRANSITION_MAP
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_PREBOOT
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_BOOTUP_DEV
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_START_DEV
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_SHUTDOWN_DEV
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_SWITCH_ON_DEV
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_ENABLE_OP_DEV
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_ENABLED
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_TRACE_CONF
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_SYNC_CONF
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_TPDO1_CONF
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_FAULHABER_CONF
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_DONE
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_FAULT
        TRANSITION_MAP_ENTRY(ST_SWITCH_ON_DEV) // ST_STOP
        TRANSITION_MAP_ENTRY(ST_COMMANDRECEIVED)
        TRANSITION_MAP_ENTRY(ST_COMMANDRECEIVED)
        TRANSITION_MAP_ENTRY(ST_COMMANDRECEIVED)
        TRANSITION_MAP_ENTRY(ST_COMMANDRECEIVED)
    END_TRANSITION_MAP(cmdData)
}

void MotorDriver::updateDriver()
{
    BEGIN_TRANSITION_MAP
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_PREBOOT
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_BOOTUP_DEV
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_START_DEV
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_SHUTDOWN_DEV
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_SWITCH_ON_DEV
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_ENABLE_OP_DEV
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_ENABLED
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_TRACE_CONF
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_SYNC_CONF
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_TPDO1_CONF
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_FAULHABER_CONF
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_DONE
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_FAULT
        TRANSITION_MAP_ENTRY(ST_SWITCH_ON_DEV) // ST_STOP
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)
        TRANSITION_MAP_ENTRY(ST_COMMANDPENDING)
        TRANSITION_MAP_ENTRY(ST_COMMANDPENDING)
        TRANSITION_MAP_ENTRY(ST_MOVING)
    END_TRANSITION_MAP(NULL)
}

void MotorDriver::ST_CommandReceived(MotorDriverData* cmdData)
{
    command = cmdData->cmd;
    goal = cmdData->data;

    switch(command)
    {
    case CMD_MOVE_POS_ABS:
        Motor::movePosAbsGrad(goal);
        break;
    case CMD_MOVE_VEL:
        Motor::moveVel(goal);
        break;
    case CMD_NONE:
        break;
    default:
        break;
    }

    InternalEvent(ST_COMMANDPENDING);
}

void MotorDriver::ST_CommandPending()
{
    switch(command)
    {
    case CMD_MOVE_POS_ABS:
        if(fabs(goal-PositionGrad) > POS_THRESHOLD && !TargetReached) ;
        else Motor::movePosAbsGrad(goal);
        break;
    case CMD_MOVE_VEL:
        Motor::moveVel(goal);
        break;
    case CMD_NONE:
        break;
    default:
        break;
    }
}

void MotorDriver::processCanMsg(int cmdID, unsigned char data[])
{
    if(cmdID == TPDO3_COBID) //0x380(896D) - trace response
    {
        long tmp_curr = data[5];
        Current = (tmp_curr << 8) + data[4];

        PositionRaw = pcandata2Double(msg,0);
        PositionGrad = PositionRaw*360/jointReduction;
        Position = ((double) PositionGrad)*PI/180.0;

        long long actualTime = llround(KAL::GetTime());
        VelocityRaw = (long)(PositionRaw - OldPositionRaw)/((actualTime - updateTime)*1E-9);
        Velocity = (Position - OldPosition)/((actualTime - updateTime)*1E-9);

        OldPositionRaw = PositionRaw;
        OldPosition = Position;
        updateTime = actualTime;
        //KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, "States Update", "ID = %X,  curr = %X%X, pos = %X%X%X%X", ID, data[5], data[4], data[3], data[2], data[1], data[0] );

    }//if(cmdID == TPDO3_COBID)
    else if(cmdID == TPDO2_COBID) //0x280(640D) - TxPDO2 response
    {
        if(data[0] == 0x2B)
        {
            Velocity = pcandata2Double(msg,1)*360/jointReduction;
            //    KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, "States Update", "ID = %X,  vel = %X%X%X%X", ID, data[4], data[3], data[2], data[1]);
        }
        else if(data[0] == 0x40)
        {
            PositionGrad = pcandata2Double(msg,1)*360/jointReduction;
            //   KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, "States Update", "ID = %X,  pos = %X%X%X%X", ID, data[4], data[3], data[2], data[1]);
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


