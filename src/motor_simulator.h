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

    // external events taken by this state machine
    void Halt();
    void SetSpeed(MotorSimulatorData*);

private:
    // state machine state functions
    void ST_Idle();
    void ST_Stop();
    void ST_Start(MotorSimulatorData*);
    void ST_ChangeSpeed(MotorSimulatorData*);

    // state map to define state function order
    BEGIN_STATE_MAP
        STATE_MAP_ENTRY(ST_Idle)
        STATE_MAP_ENTRY(ST_Stop)
        STATE_MAP_ENTRY(ST_Start)
        STATE_MAP_ENTRY(ST_ChangeSeed)
    END_STATE_MAP

    // state enumeration order must match the order of state
    // method entries in the state map
    enum E_States {
        ST_IDLE = 0,
        ST_STOP,
        ST_START,
        ST_CHANGE_SPEED,
        ST_MAX_STATES
    };
};

#endif // MOTORSIMULATOR_H
