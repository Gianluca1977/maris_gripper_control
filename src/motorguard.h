#ifndef MOTORGUARD_H
#define MOTORGUARD_H

#include "state_machine.h"
#include "motor.h"
#include "timer.h"

#define GUARD_DELAY              (1 * WF_TIME_ONE_S)

#define CURRENT_THRESHOLD   2000

// structure to hold event data passed into state machine
struct MotorGuardData : public EventData
{

};

class MotorGuard : public StateMachine, public Motor
{
    bool overloaded;

    static Timer guardTimer;

    // state machine state functions
    void ST_Idle(); 
    void ST_Warning();
    void ST_Overload();

    // state map to define state function order
    BEGIN_STATE_MAP
        STATE_MAP_ENTRY(&MotorGuard::ST_Idle)        
        STATE_MAP_ENTRY(&MotorGuard::ST_Warning)
        STATE_MAP_ENTRY(&MotorGuard::ST_Overload)
    END_STATE_MAP

    // state enumeration order must match the order of state
    // method entries in the state map
    enum E_States {
        ST_IDLE = 0,        
        ST_WARNING,
        ST_OVERLOAD,
        ST_MAX_STATES
    };

public:
    MotorGuard();    
    MotorGuard(int id);
    ~MotorGuard();

    void guardTimerExpired(void);
    void resetGuard(void);
    void updateGuard(void);
};

#endif // MOTORGUARD_H
