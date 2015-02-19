#include "wf.h"
#include "timer.h"

Timer::Callback Timer::CallbackFunc;

Timer::Timer() : timeout(0), StateMachine(Timer::ST_MAX_STATES)
{
    Reset();
}

Timer::Timer(long long expire_time) : timeout(expire_time), StateMachine(Timer::ST_MAX_STATES)
{
    KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, "TIMER", "Calling Constructor");
    Reset();
}

void Timer::Update()
{
    BEGIN_TRANSITION_MAP                   // - Current State -
        TRANSITION_MAP_ENTRY (ST_IDLE)     // ST_Idle
        TRANSITION_MAP_ENTRY (ST_RUNNING)  // ST_Running
        TRANSITION_MAP_ENTRY (ST_EXPIRED)  // ST_Expired
    END_TRANSITION_MAP(NULL)
}

bool Timer::Start()
{
    if(timeout <= 0) return false;

    return Start(timeout);
}

bool Timer::Start(long long expire_time)
{
    if((currentState == ST_RUNNING) || (currentState == ST_EXPIRED)) return false;

    timeout = expire_time;

    Reset();

    startTime = llround(KAL::GetTime());

    BEGIN_TRANSITION_MAP                    // - Current State -
        TRANSITION_MAP_ENTRY (ST_RUNNING)      // ST_Idle
        TRANSITION_MAP_ENTRY (EVENT_IGNORED)      // ST_Running
        TRANSITION_MAP_ENTRY (EVENT_IGNORED)      // ST_Expired
    END_TRANSITION_MAP(NULL)

    KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, "TIMER", "Timer Started");

    return true;
}

void Timer::Stop()
{
    BEGIN_TRANSITION_MAP                    // - Current State -
        TRANSITION_MAP_ENTRY (ST_IDLE)      // ST_Idle
        TRANSITION_MAP_ENTRY (ST_IDLE)      // ST_Running
        TRANSITION_MAP_ENTRY (ST_IDLE)      // ST_Expired
    END_TRANSITION_MAP(NULL)
}

void Timer::Reset()
{
    Stop();
    startTime = 0;
}

void Timer::ST_Idle()
{

}

void Timer::ST_Running()
{
    if((startTime + timeout - llround(KAL::GetTime())) <= 0){
        InternalEvent(ST_EXPIRED);
        (this->*Timer::CallbackFunc)();
    }
}

void Timer::ST_Expired()
{

}

bool Timer::Restart()
{
    Reset();
    return Start();
}
