#include <assert.h>
#include "motor_simulator.h"

// halt motor external event
void MotorSimulator::Halt(void)
{
    // given the Halt event, transition to a new state based upon
    // the current state of the state machine
    BEGIN_TRANSITION_MAP                      // - Current State -
        TRANSITION_MAP_ENTRY (EVENT_IGNORED)  // ST_Idle
        TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)  // ST_Stop
        TRANSITION_MAP_ENTRY (ST_STOP)        // ST_Start
        TRANSITION_MAP_ENTRY (ST_STOP)        // ST_ChangeSpeed
    END_TRANSITION_MAP(NULL)
}

// set motor speed external event
void MotorSimulator::SetSpeed(MotorSimulatorData* pData)
{
    BEGIN_TRANSITION_MAP                      // - Current State -
        TRANSITION_MAP_ENTRY (ST_START)       // ST_Idle
        TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)  // ST_Stop
        TRANSITION_MAP_ENTRY (ST_CHANGE_SPEED)// ST_Start
        TRANSITION_MAP_ENTRY (ST_CHANGE_SPEED)// ST_ChangeSpeed
    END_TRANSITION_MAP(pData)
}

// state machine sits here when motor is not running
void MotorSimulator::ST_Idle()
{
}

// stop the motor
void MotorSimulator::ST_Stop()
{
    // perform the stop motor processing here
    // transition to ST_Idle via an internal event
    InternalEvent(ST_IDLE);
}

// start the motor going
void MotorSimulator::ST_Start(MotorSimulatorData* pData)
{
    // set initial motor speed processing here
}

// changes the motor speed once the motor is moving
void MotorSimulator::ST_ChangeSpeed(MotorSimulatorData* pData)
{
    // perform the change motor speed to pData->speed here
}
