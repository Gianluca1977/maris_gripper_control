#include <assert.h>
#include "state_machine.h"
#include "wf.h"

StateMachine::StateMachine(int maxStates) :
    _maxStates(maxStates),
    currentState(0),
    _eventGenerated(false),
    _pEventData(NULL)
{    
    //KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, "STATE_MACHINE", "Created by %p with currentState = %d, _maxStates = %d", this, currentState, _maxStates);
}

// generates an external event. called once per external event
// to start the state machine executing
void StateMachine::ExternalEvent(unsigned char newState, EventData* pData)
{
    // if we are supposed to ignore this event
    if (newState == EVENT_IGNORED)
    {
        // just delete the event data, if any
        if (pData) delete pData;
    }
    else
    {
        // generate the event and execute the state engine
        InternalEvent(newState, pData);
        StateEngine();
    }
}

// generates an internal event. called from within a state
// function to transition to a new state
void StateMachine::InternalEvent(unsigned char newState, EventData* pData)
{
    _pEventData = pData;
    _eventGenerated = true;
    currentState = newState;
}

// the state engine executes the state machine states
void StateMachine::StateEngine(void)
{
    EventData* pDataTemp = NULL;

    // TBD - lock semaphore here
    // while events are being generated keep executing states
    while (_eventGenerated) {
        pDataTemp = _pEventData;  // copy of event data pointer
        _pEventData = NULL;       // event data used up, reset ptr
        _eventGenerated = false;  // event used up, reset flag        

        // execute the state passing in event data, if any
        const StateStruct* pStateMap = GetStateMap();
#ifdef _DEBUG_
        KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, "STATE_MACHINE", "Called by %s %p with currentState = %d, _maxStates = %d, pStateFunc = %p", ST_name, this, currentState, _maxStates, &pStateMap[currentState]);
#endif
        if(&pStateMap[currentState] != (const StateStruct*) NULL) (this->*pStateMap[currentState].pStateFunc)(pDataTemp);
        else  KAL::DebugConsole::Write(LOG_LEVEL_ERROR, "STATE_MACHINE", "Pointer to state %d function is NULL", currentState);

        // if event data was used, then delete it
        if (pDataTemp) {
            delete pDataTemp;
            pDataTemp = NULL;
        }
    }
    // TBD - unlock semaphore here
}
