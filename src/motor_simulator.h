#ifndef MOTORSIMULATOR_H
#define MOTORSIMULATOR_H

#include "state_machine.h"

// structure to hold event data passed into state machine
struct MotorSimulatorData : public EventData
{
    int speed;
};

// the Motor state machine class
class MotorSimulator : public StateMachine
{
public:
    MotorSimulator() : StateMachine(ST_MAX_STATES) {}

    void setID(int id);

    // external events taken by this state machine

private:

    int ID;

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
        STATE_MAP_ENTRY(&MotorSimulator::ST_PreBoot)
        STATE_MAP_ENTRY(&MotorSimulator::ST_BootedUp)
        STATE_MAP_ENTRY(&MotorSimulator::ST_SwitchOnDisabled)
        STATE_MAP_ENTRY(&MotorSimulator::ST_ReadyToSwitchOn)
        STATE_MAP_ENTRY(&MotorSimulator::ST_SwitchedOn)
        STATE_MAP_ENTRY(&MotorSimulator::ST_OperationEnabled)
        STATE_MAP_ENTRY(&MotorSimulator::ST_QuickStopActive)
        STATE_MAP_ENTRY(&MotorSimulator::ST_Limit_Conf)
        STATE_MAP_ENTRY(&MotorSimulator::ST_Sync_Conf)
        STATE_MAP_ENTRY(&MotorSimulator::ST_TPDO1_Conf)
        STATE_MAP_ENTRY(&MotorSimulator::ST_TPDO2_Conf)
        STATE_MAP_ENTRY(&MotorSimulator::ST_Faulhaber_Conf)
        STATE_MAP_ENTRY(&MotorSimulator::ST_Idle)
        STATE_MAP_ENTRY(&MotorSimulator::ST_CommandReceived)
        STATE_MAP_ENTRY(&MotorSimulator::ST_CommandPending)
        STATE_MAP_ENTRY(&MotorSimulator::ST_Moving)
        STATE_MAP_ENTRY(&MotorSimulator::ST_Stop)
        STATE_MAP_ENTRY(&MotorSimulator::ST_Fault)
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
};

#endif // MOTORSIMULATOR_H
