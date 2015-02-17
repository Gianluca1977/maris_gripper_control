#ifndef TIMER_H
#define TIMER_H

#include "state_machine.h"

// structure to hold event data passed into state machine
struct TimerData : public EventData
{
    long long expire_time;
};


class Timer : public StateMachine
{
    bool running;
    bool expired;
    long long timeout;
    long long actualTime, startTime;

    // state machine state functions
    void ST_Idle();
    void ST_Running();
    void ST_Expired();

    // state map to define state function order
    BEGIN_STATE_MAP
        STATE_MAP_ENTRY(&ST_Idle)
        STATE_MAP_ENTRY(&ST_Running)
        STATE_MAP_ENTRY(&ST_Expired)
    END_STATE_MAP

    // state enumeration order must match the order of state
    // method entries in the state map
    enum E_States {
        ST_IDLE = 0,
        ST_RUNNING,
        ST_EXPIRED,
        ST_MAX_STATES
    };

public:
    Timer();
    Timer(long long expire_time);

    void update(void);

    void expireCallback(void);

    bool Start(void);
    bool Start(TimerData*);

    void Stop(void);
    void Reset(void);

    bool isRunning(void){return (currentState == ST_RUNNING);}
    bool isExpired(void){return (currentState == ST_EXPIRED);}

};

#endif // TIMER_H
