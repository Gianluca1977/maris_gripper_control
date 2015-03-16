#include "motorguard.h"

Timer MotorGuard::guardTimer = Timer();

MotorGuard::MotorGuard() : StateMachine(MotorGuard::ST_MAX_STATES), overloaded(false)
{
    guardTimer.init(GUARD_DELAY);
}

MotorGuard::MotorGuard(int id) : StateMachine(MotorGuard::ST_MAX_STATES), Motor(id), overloaded(false)
{
    guardTimer.init(GUARD_DELAY);
}

void MotorGuard::ST_Idle()
{
    if(Current > CURRENT_THRESHOLD)
    {
        overloaded = true;
        guardTimer.Start();
        InternalEvent(ST_WARNING);
    }
}

void MotorGuard::ST_Warning()
{
    if(guardTimer.isExpired())
    {
        InternalEvent(ST_OVERLOAD);
        return;
    }

    if(Current <= CURRENT_THRESHOLD)
    {
        overloaded = false;
        InternalEvent(ST_IDLE);
    }
}

void MotorGuard::ST_Overload()
{    
    if(overloaded)
    {
        stop();
        disable();
        Operational = false;
    }
}

void MotorGuard::guardTimerExpired()
{
    BEGIN_TRANSITION_MAP
            TRANSITION_MAP_ENTRY(EVENT_IGNORED)
            TRANSITION_MAP_ENTRY(ST_OVERLOAD)
            TRANSITION_MAP_ENTRY(ST_OVERLOAD)
    END_TRANSITION_MAP(NULL)
}

void MotorGuard::resetGuard()
{
    overloaded = false;

    BEGIN_TRANSITION_MAP
            TRANSITION_MAP_ENTRY(EVENT_IGNORED)
            TRANSITION_MAP_ENTRY(ST_IDLE)
            TRANSITION_MAP_ENTRY(ST_IDLE)
    END_TRANSITION_MAP(NULL)
}

void MotorGuard::updateGuard()
{
    BEGIN_TRANSITION_MAP
            TRANSITION_MAP_ENTRY(ST_IDLE)
            TRANSITION_MAP_ENTRY(ST_WARNING)
            TRANSITION_MAP_ENTRY(ST_OVERLOAD)
    END_TRANSITION_MAP(NULL)
}


MotorGuard::~MotorGuard()
{

}
