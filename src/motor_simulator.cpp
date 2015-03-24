#include <assert.h>
#include "motor_simulator.h"


// state machine sits here when motor is not running
void MotorSimulator::setID(int id)
{
        ID = id;
        //std::cout << "Motor ID = " << ID << std::endl;
}

void MotorSimulator::ST_Idle()
{

}

void MotorSimulator::ST_Bootup_Dev()
{

}

// stop the motor
void MotorSimulator::ST_Stop()
{
    // perform the stop motor processing here
    // transition to ST_Idle via an internal event
    InternalEvent(ST_IDLE);
}

