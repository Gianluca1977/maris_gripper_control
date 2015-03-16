#include "motorguard.h"

Timer MotorGuard::guardTimer = Timer();

MotorGuard::MotorGuard() : StateMachine(MotorConfigurator::ST_MAX_STATES), motor_index(0), overloaded({false, false, false})
{
    guardTimer.Init(GUARD_DELAY);
}

MotorGuard::MotorGuard(Motor (&motor)[]) : StateMachine(MotorConfigurator::ST_MAX_STATES), GripperMotors(motor), motor_index(0), overloaded({false, false, false})
{
    guardTimer.Init(GUARD_DELAY);
}

void MotorGuard::ST_Idle()
{
    CHECK_ALL_MOTORS(if(Motors[motor_index].Current > CURRENT_THRESHOLD) overloaded[motor_index] = true;)

    CHECK_ALL_MOTORS(
        if(overloaded[motor_index])
        {
            guardTimer.Start();
            InternalEvent(ST_WARNING);
            return;
        }
    )
}

void MotorGuard::ST_Warning()
{
    if(guardTimer.isExpired())
    {
        InternalEvent(ST_OVERLOAD);
        return;
    }

    CHECK_ALL_MOTORS(if(Motors[motor_index].Current < CURRENT_THRESHOLD) overloaded[motor_index] = false;)

    CHECK_ALL_MOTORS(if(overloaded[motor_index]) return;)

    InternalEvent(ST_IDLE);
}

void MotorGuard::ST_Overload()
{
    CHECK_ALL_MOTORS(
        if(overloaded[motor_index])
        {
            GripperMotors[motor_index].stop();
            GripperMotors[motor_index].disable();
            GripperMotors[motor_index].Operational = false;
        }
    )

    InternalEvent(ST_IDLE);
}

void MotorGuard::timerExpired()
{
    BEGIN_TRANSITION_MAP
            TRANSITION_MAP_ENTRY(EVENT_IGNORED)
            TRANSITION_MAP_ENTRY(ST_OVERLOAD)
            TRANSITION_MAP_ENTRY(ST_OVERLOAD)
    END_TRANSITION_MAP(NULL)
}

void MotorGuard::reset()
{
    CHECK_ALL_MOTORS(overloaded[motor_index] = false;)

    BEGIN_TRANSITION_MAP
            TRANSITION_MAP_ENTRY(EVENT_IGNORED)
            TRANSITION_MAP_ENTRY(ST_IDLE)
            TRANSITION_MAP_ENTRY(ST_IDLE)
    END_TRANSITION_MAP(NULL)
}

void MotorGuard::update()
{
    BEGIN_TRANSITION_MAP
            TRANSITION_MAP_ENTRY(ST_IDLE)
            TRANSITION_MAP_ENTRY(ST_WARNING)
            TRANSITION_MAP_ENTRY(ST_OVERLOAD)
    END_TRANSITION_MAP(NULL)
}
