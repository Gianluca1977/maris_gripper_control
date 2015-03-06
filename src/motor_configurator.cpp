#include "motor_configurator.h"
#include "timer.h"
#include "controller.h"
#include "motor.h"

//const Callback MotorConfigurator::CallbackFunc = reinterpret_cast<Callback>(&MotorConfigurator::timerExpired);

void MotorConfigurator::timerExpired()
{
    //KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, "MOTOR_CONFIGURATOR", "Calling timerExpired of %s %p", ST_name, this);

    BEGIN_TRANSITION_MAP
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)
        TRANSITION_MAP_ENTRY(ST_BOOTUP_DEV)
        TRANSITION_MAP_ENTRY(ST_START_DEV)
        TRANSITION_MAP_ENTRY(ST_SHUTDOWN_DEV)
        TRANSITION_MAP_ENTRY(ST_SWITCH_ON_DEV)
        TRANSITION_MAP_ENTRY(ST_ENABLE_OP_DEV)
        TRANSITION_MAP_ENTRY(ST_ENABLED)
        TRANSITION_MAP_ENTRY(ST_TRACE_CONF)
        TRANSITION_MAP_ENTRY(ST_SYNC_CONF)
        TRANSITION_MAP_ENTRY(ST_TPDO1_CONF)
        TRANSITION_MAP_ENTRY(ST_FAULHABER_CONF)
        TRANSITION_MAP_ENTRY(ST_DONE)
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)
    END_TRANSITION_MAP(NULL)            
}

void MotorConfigurator::StartConfiguration()
{
    KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, "MOTOR_CONFIGURATOR", "Calling Start Configuration of %s %p", ST_name, this);

    BEGIN_TRANSITION_MAP
        TRANSITION_MAP_ENTRY(ST_BOOTUP_DEV) // ST_IDLE
        TRANSITION_MAP_ENTRY(ST_FAULT) // ST_BOOTUP_DEV
        TRANSITION_MAP_ENTRY(ST_FAULT) // ST_START_DEV
        TRANSITION_MAP_ENTRY(ST_FAULT) // ST_SHUTDOWN_DEV
        TRANSITION_MAP_ENTRY(ST_FAULT) // ST_SWITCH_ON_DEV
        TRANSITION_MAP_ENTRY(ST_FAULT) // ST_ENABLE_OP_DEV
        TRANSITION_MAP_ENTRY(ST_FAULT) // ST_ENABLED
        TRANSITION_MAP_ENTRY(ST_FAULT) // ST_TRACE_CONF
        TRANSITION_MAP_ENTRY(ST_FAULT) // ST_SYNC_CONF
        TRANSITION_MAP_ENTRY(ST_FAULT) // ST_TPDO1_CONF
        TRANSITION_MAP_ENTRY(ST_FAULT) // ST_FAULHABER_CONF
        TRANSITION_MAP_ENTRY(ST_FAULT) // ST_DONE
        TRANSITION_MAP_ENTRY(ST_FAULT) // ST_FAULT
        TRANSITION_MAP_ENTRY(ST_FAULT) // ST_CLOSE
    END_TRANSITION_MAP(NULL)
}

void MotorConfigurator::Conf_StepUp()
{
    // KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, "MOTOR_CONFIGURATOR", "Calling Conf_StepUp() of %s %p", ST_name, this);

    BEGIN_TRANSITION_MAP
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)
        TRANSITION_MAP_ENTRY(ST_START_DEV)
        TRANSITION_MAP_ENTRY(ST_SHUTDOWN_DEV)
        TRANSITION_MAP_ENTRY(ST_SWITCH_ON_DEV)
        TRANSITION_MAP_ENTRY(ST_ENABLE_OP_DEV)
        TRANSITION_MAP_ENTRY(ST_ENABLED)
        TRANSITION_MAP_ENTRY(ST_TRACE_CONF)
        TRANSITION_MAP_ENTRY(ST_SYNC_CONF)
        TRANSITION_MAP_ENTRY(ST_TPDO1_CONF)
        TRANSITION_MAP_ENTRY(ST_FAULHABER_CONF)
        TRANSITION_MAP_ENTRY(ST_DONE)
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)
    END_TRANSITION_MAP(NULL)
}

MotorConfigurator::MotorConfigurator() : StateMachine(MotorConfigurator::ST_MAX_STATES, "MotorConfigurator")
{
     KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, "MOTOR_CONFIGURATOR", "Calling default constructor of %p", this);
     //WaitTime.CallbackFunc = reinterpret_cast<Timer::Callback>(&MotorConfigurator::timerExpired);
     motor_index = 0;
     Configured = false;
}

MotorConfigurator::MotorConfigurator(Motor (&motor)[NUM_MOT]) : StateMachine(MotorConfigurator::ST_MAX_STATES, "MotorConfigurator"), GripperMotors(motor)
{
    KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, "MOTOR_CONFIGURATOR", "Calling Constructor of %p", this);
    //WaitTime.CallbackFunc = reinterpret_cast<Timer::Callback>(&MotorConfigurator::timerExpired);
    motor_index = 0;
    Configured = false;
}

void MotorConfigurator::ST_Bootup_Dev()
{
    //KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, "MOTOR_CONFIGURATOR", "In ST_Bootup_Dev");
    Configured = false;
    EVALUATE_TRANSITION(!GripperMotors[motor_index].BootUp, INIT_BOOTUP_DEV)
}

void MotorConfigurator::ST_Start_Dev()
{
     EVALUATE_TRANSITION((GripperMotors[motor_index].State & STATUS_WORD_MASK) != SWITCH_ON_DISABLED, INIT_START_DEV)
}

void MotorConfigurator::ST_Shutdown_Dev()
{
    EVALUATE_TRANSITION((GripperMotors[motor_index].State & STATUS_WORD_MASK) != READY_TO_SWITCH_ON, INIT_SHUTDOWN_DEV)
}

void MotorConfigurator::ST_Switch_On_Dev()
{
    EVALUATE_TRANSITION((GripperMotors[motor_index].State & STATUS_WORD_MASK) != SWITCHED_ON , INIT_SWITCH_ON_DEV)
}

void MotorConfigurator::ST_Enable_Op_Dev()
{
    EVALUATE_TRANSITION((GripperMotors[motor_index].State & STATUS_WORD_MASK) != OPERATION_ENABLED, INIT_ENABLE_OP_DEV)
}

void MotorConfigurator::ST_Enabled()
{
    if(motor_index == 0) KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, TRIGGERTASK_NAME, "All devices has been initialized! Configuring it... ");
    SEND_CONFIGURATION(INIT_ENABLED)
}

void MotorConfigurator::ST_Trace_Conf()
{
    SEND_CONFIGURATION(INIT_TRACE_CONF)
}

void MotorConfigurator::ST_Sync_Conf()
{
    SEND_CONFIGURATION(INIT_SYNC_CONF)
}

void MotorConfigurator::ST_TPDO1_Conf()
{
    SEND_CONFIGURATION(INIT_TPDO1_CONF)
}

void MotorConfigurator::ST_Faulhaber_Conf()
{
    SEND_CONFIGURATION(INIT_FAULHABER_CONF)
}

void MotorConfigurator::ST_Done()
{
    KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, TRIGGERTASK_NAME, "Done. We are ready to roll! ");
    Configured = true;
    //if (armPresent == true) S2.Signal();//risvegliamo il braccio - torna a dare il sync
    InternalEvent(ST_IDLE);
}

void MotorConfigurator::ST_Fault()
{
    //fromFault = true;
    Configured = false;
    for(motor_index = 0; motor_index < NUM_MOT; motor_index++) GripperMotors[motor_index].init(INIT_FAULT);
    motor_index = 0;
    InternalEvent(ST_BOOTUP_DEV);
}

void MotorConfigurator::ST_Close()
{

}

void MotorConfigurator::ST_Idle()
{

}



