#ifndef MOTORGUARD_H
#define MOTORGUARD_H

#include "state_machine.h"
#include "motor.h"

// structure to hold event data passed into state machine
struct MotorGuardData : public EventData
{

};

class MotorGuard : public StateMachine
{
    int motor_index;

    Motor *GripperMotors;

    // state machine state functions

    void ST_Idle();

    // state map to define state function order
    BEGIN_STATE_MAP
        STATE_MAP_ENTRY(&MotorConfigurator::ST_Idle)
    END_STATE_MAP

    // state enumeration order must match the order of state
    // method entries in the state map
    enum E_States {
        ST_IDLE = 0,
        ST_MAX_STATES
    };

public:
    MotorGuard();
    MotorGuard(Motor (&motor)[NUM_MOT]);

    void Init(Motor *motor){GripperMotors = motor;}

    void timerExpired(void);
};

#endif // MOTORGUARD_H
