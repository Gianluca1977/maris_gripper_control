#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H
#include <stdio.h>
#include "event_data.h"

struct StateStruct;

// base class for state machines
class StateMachine
{
public:
    StateMachine(int maxStates);
    virtual ~StateMachine() {}

protected:
    enum { EVENT_IGNORED = 0xFE, CANNOT_HAPPEN };
    unsigned char currentState;
    unsigned char previousState;
    unsigned char futureState;
    void ExternalEvent(unsigned char, EventData* = NULL);
    void InternalEvent(unsigned char, EventData* = NULL);
    virtual const StateStruct* GetStateMap() = 0;

private:
    const int _maxStates;
    bool _eventGenerated;
    EventData* _pEventData;
    void StateEngine(void);
};

typedef void (StateMachine::*InFunc)(EventData *);
typedef void (StateMachine::*StateFunc)(EventData *);
typedef void (StateMachine::*OutFunc)(EventData *);

struct StateStruct
{
//    InFunc pInFunc;
    StateFunc pStateFunc;
//    OutFunc pOutFunc;
};

#define BEGIN_STATE_MAP \
    public:\
    const StateStruct* GetStateMap() {\
    static const StateStruct StateMap[] = {

#define STATE_MAP_ENTRY(state)\
    { reinterpret_cast<StateFunc>(state) },

#define END_STATE_MAP \
    { reinterpret_cast<StateFunc>((StateFunc)NULL) }\
        }; \
    return &StateMap[0]; }

#define BEGIN_TRANSITION_MAP \
    static const unsigned char TRANSITIONS[] = {\

#define TRANSITION_MAP_ENTRY(entry)\
    entry,

#define END_TRANSITION_MAP(data) \
    0 };\
    ExternalEvent(TRANSITIONS[currentState], data);

#endif //STATE_MACHINE_H
