#ifndef MOTOR_CONFIGURATOR_H
#define MOTOR_CONFIGURATOR_H

#include "motor.h"
#include "timer.h"

#define EVALUATE_TRANSITION(condition, initphase) \
    if(motor_index == NUM_MOT) \
    {\
        motor_index = 0;\
        Conf_StepUp();\
    }\
    else\
    {\
        if(condition)\
        {\
            GripperMotors[motor_index].init(initphase);\            
        }\
        else\
        {\
            motor_index++;\
            InternalEvent(currentState);\
        }\
    }\

#define SEND_CONFIGURATION(initphase) \
    if(motor_index == NUM_MOT) \
    {\
        motor_index = 0;\
        Conf_StepUp();\
    }\
    else\
    {\
        GripperMotors[motor_index].init(initphase);\        
        motor_index++;\
    }\

// structure to hold event data passed into state machine
struct MotorConfiguratorData : public EventData
{

};

class MotorConfigurator : public StateMachine
{

    int motor_index;    

    bool Configured;

    Motor *GripperMotors;

    // state machine state functions

    void Conf_StepUp();

    void ST_Idle();
    void ST_Bootup_Dev();
    void ST_Start_Dev();
    void ST_Shutdown_Dev();
    void ST_Switch_On_Dev();
    void ST_Enable_Op_Dev();
    void ST_Enabled();
    void ST_Trace_Conf();
    void ST_Sync_Conf();
    void ST_TPDO1_Conf();
    void ST_Faulhaber_Conf();
    void ST_Done();
    void ST_Fault();
    void ST_Stop();

    // state map to define state function order
    BEGIN_STATE_MAP
        STATE_MAP_ENTRY(&MotorConfigurator::ST_Idle)
        STATE_MAP_ENTRY(&MotorConfigurator::ST_Bootup_Dev)
        STATE_MAP_ENTRY(&MotorConfigurator::ST_Start_Dev)
        STATE_MAP_ENTRY(&MotorConfigurator::ST_Shutdown_Dev)
        STATE_MAP_ENTRY(&MotorConfigurator::ST_Switch_On_Dev)
        STATE_MAP_ENTRY(&MotorConfigurator::ST_Enable_Op_Dev)
        STATE_MAP_ENTRY(&MotorConfigurator::ST_Enabled)
        STATE_MAP_ENTRY(&MotorConfigurator::ST_Trace_Conf)
        STATE_MAP_ENTRY(&MotorConfigurator::ST_Sync_Conf)
        STATE_MAP_ENTRY(&MotorConfigurator::ST_TPDO1_Conf)
        STATE_MAP_ENTRY(&MotorConfigurator::ST_Faulhaber_Conf)
        STATE_MAP_ENTRY(&MotorConfigurator::ST_Done)
        STATE_MAP_ENTRY(&MotorConfigurator::ST_Fault)
        STATE_MAP_ENTRY(&MotorConfigurator::ST_Stop)
    END_STATE_MAP

    // state enumeration order must match the order of state
    // method entries in the state map
    enum E_States {
        ST_IDLE = 0,
        ST_BOOTUP_DEV,
        ST_START_DEV,
        ST_SHUTDOWN_DEV,
        ST_SWITCH_ON_DEV,
        ST_ENABLE_OP_DEV,
        ST_ENABLED,
        ST_TRACE_CONF,
        ST_SYNC_CONF,
        ST_TPDO1_CONF,
        ST_FAULHABER_CONF,
        ST_DONE,
        ST_FAULT,
        ST_STOP,
        ST_MAX_STATES
    };

public:

    //Timer WaitTime;

    bool fromStop;

    MotorConfigurator(Motor (&motor)[NUM_MOT]);

    MotorConfigurator();

    void init(Motor *motor){GripperMotors = motor;}

    void timerExpired(void);
    void StartConfiguration(void);
    bool isConfigured(void){return Configured;}
    void Stop(void);
    void Restart(void);
};

#endif // MOTOR_CONFIGURATOR_H
