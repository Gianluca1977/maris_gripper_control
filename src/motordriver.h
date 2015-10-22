#ifndef MOTORDRIVER_H
#define MOTORDRIVER_H

#include "motor.h"
#include "state_machine.h"
#include "timer.h"

#define CMD_NONE            0
#define CMD_MOVE_POS_ABS    1
#define CMD_MOVE_VEL        2

#define POS_THRESHOLD       1.0

struct MotorDriverData : public EventData
{
    int cmd;
    long long int data;
};

class MotorDriver : public StateMachine, public Motor
{
    static Timer driverTimer;

    int command;
    long long int goal;

    void executeCommand(MotorDriverData* cmdData);

    // state machine state functions
    void ST_PreBoot();
    void ST_BootedUp();
    void ST_SwitchOnDisabled();
    void ST_ReadyToSwitchOn();
    void ST_SwitchedOn();
    void ST_OperationEnabled();
    void ST_QuickStopActive();
    void ST_Limit_Conf();
    void ST_Sync_Conf();
    void ST_TPDO1_Conf();
    void ST_TPDO2_Conf();
    void ST_Faulhaber_Conf();
    void ST_Idle();
    void ST_CommandReceived();
    void ST_CommandPending();
    void ST_Moving();
    void ST_Stop();
    void ST_Fault();

    // state map to define state function order
    BEGIN_STATE_MAP
        STATE_MAP_ENTRY(&MotorDriver::ST_PreBoot)
        STATE_MAP_ENTRY(&MotorDriver::ST_BootedUp)
        STATE_MAP_ENTRY(&MotorDriver::ST_SwitchOnDisabled)
        STATE_MAP_ENTRY(&MotorDriver::ST_ReadyToSwitchOn)
        STATE_MAP_ENTRY(&MotorDriver::ST_SwitchedOn)
        STATE_MAP_ENTRY(&MotorDriver::ST_OperationEnabled)
        STATE_MAP_ENTRY(&MotorDriver::ST_QuickStopActive)
        STATE_MAP_ENTRY(&MotorDriver::ST_Limit_Conf)
        STATE_MAP_ENTRY(&MotorDriver::ST_Sync_Conf)
        STATE_MAP_ENTRY(&MotorDriver::ST_TPDO1_Conf)
        STATE_MAP_ENTRY(&MotorDriver::ST_TPDO2_Conf)
        STATE_MAP_ENTRY(&MotorDriver::ST_Faulhaber_Conf)
        STATE_MAP_ENTRY(&MotorDriver::ST_Idle)
        STATE_MAP_ENTRY(&MotorDriver::ST_CommandReceived)
        STATE_MAP_ENTRY(&MotorDriver::ST_CommandPending)
        STATE_MAP_ENTRY(&MotorDriver::ST_Moving)
        STATE_MAP_ENTRY(&MotorDriver::ST_Stop)
        STATE_MAP_ENTRY(&MotorDriver::ST_Fault)
    END_STATE_MAP

    // state enumeration order must match the order of state
    // method entries in the state map
    enum E_States {
        ST_PREBOOT = 0,
        ST_BOOTEDUP,
        ST_SWITCHONDISABLED,
        ST_READYTOSWTICHON,
        ST_SWITCHEDON,
        ST_OPERATIONENABLED,
        ST_QUICKSTOPACTIVE,
        ST_TRACE_CONF,
        ST_SYNC_CONF,
        ST_TPDO1_CONF,
        ST_FAULHABER_CONF,
        ST_DONE,
        ST_FAULT,
        ST_STOP,
        ST_IDLE,
        ST_COMMANDRECEIVED,
        ST_COMMANDPENDING,
        ST_MOVING,
        ST_MAX_STATES
    };

public:
    MotorDriver(int id);
    MotorDriver();
    ~MotorDriver();

    void processCanMsg(int cmdID, unsigned char data[]);

    void movePosAbsGrad(long absValue = 0);
    void moveVel(long value = 0);
    void updateDriver();
};

#endif // MOTORDRIVER_H
