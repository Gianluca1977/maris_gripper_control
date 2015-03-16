//============================================================================
// Name        : controller.cpp
// Author      : Gianluca Palli
// Version     :
// Copyright   : 
// Description :
//============================================================================

#include "main.h"
#include "ros_interface.h"
#include "controller.h"
#include "timer.h"

#include <boost/algorithm/string.hpp>

MotorConfigurator CtrlHandler::configurator = MotorConfigurator();
Timer CtrlHandler::configuratorTimer = Timer();

MotorConfigurator CtrlHandler::guard = MotorGuard();
Timer CtrlHandler::guardTimer = Timer();

WF::Thread CtrlHandler::if_thread;
WF::Task* CtrlHandler::if_task;
void* CtrlHandler::returnValue;

WF::BinarySemaphore CtrlHandler::S1;
WF::BinarySemaphore CtrlHandler::S2;
WF::BinarySemaphore CtrlHandler::S3;

CtrlHandler::CtrlHandler() : StateMachine(CtrlHandler::ST_MAX_STATES), Gripper(nodeIds)//, Configurator(Motors)
{
    KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, CONTROLTASK_NAME, "Calling Constructor of %p", this);

    configurator.Init(Motors);
    configuratorTimer.Init(INIT_PHASEDELAY);

    KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, CONTROLTASK_NAME, "Address of configurator = %p", &configurator);

#ifdef ROS_IF

#endif        

    if(sockTCP.active) TcpActive = true;
    else TcpActive = false;

    int ret;

    if (armPresent == true){
        ret = S1.Request("S1");
        if (ret != WF_RV_OK){
        }
        ret = S2.Request("S2");
        if (ret != WF_RV_OK){
        }
        ret = S3.Request("S3");
        if (ret != WF_RV_OK){
        }
    }

    ret = MsgSem.Create("MSGSEM", 1);
    if (ret != WF_RV_OK){
        KAL::DebugConsole::Write(LOG_LEVEL_ERROR, CONTROLTASK_NAME, "Unable to create MSGSEM");\
    }


    KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, CONTROLTASK_NAME, "Calling thread_func");\
    //if_thread.Create(thread_func, this);
    if_thread.Create(rt_thread_handler, this);
}

CtrlHandler::~CtrlHandler()
{
    if_thread.Join();
    MsgSem.Release();
    //DEBUG("%s thread joined\n",get_label());
}
Timer CtrlHandler::getGuardTimer()
{
    return guardTimer;
}

void CtrlHandler::setGuardTimer(const Timer &value)
{
    guardTimer = value;
}


void CtrlHandler::ST_Start_Controller()
{
    //KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, CONTROLTASK_NAME, "Calling Configurator.StartConfiguration() of %s %p %p from %p", this->Configurator.ST_name, &(this->Configurator), GetStateMap(), this);
    configurator.StartConfiguration(); // calling within ST_START_CONTROLLER, then transit to ST_WAIT_CONFIGURATION
    configuratorTimer.Start();
    KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, CONTROLTASK_NAME, "Configurator Started");
    InternalEvent(ST_WAIT_CONFIGURATION);
}

void CtrlHandler::ST_Wait_Configuration()
{
    if(!armPresent && configurator.fromStop)
    {
        send_sync_msg();
    }

    configuratorTimer.Update(); // calling within ST_WAIT_CONFIGURATION
    if(configurator.isConfigured())
    {
        KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, CONTROLTASK_NAME, "Motors are Configured");
        if (armPresent) S2.Signal();//risvegliamo il braccio - torna a dare il sync
        InternalEvent(ST_RUNNING);
        KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, CONTROLTASK_NAME, "Controller in Running state");
    }
    else
    {
        if(configuratorTimer.isExpired())
        {
            configurator.timerExpired();
            if(!configuratorTimer.Restart()) KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, "TIMER", "Error restarting timer");\
        }
    }
}

void CtrlHandler::ST_Running()
{
    if(!isOperative() || !configurator.isConfigured())
    {
        KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, CONTROLTASK_NAME, "Controller in Emergency state");
        InternalEvent(ST_EMERGENCY);
        return;
    }

    // call motor guard
    guard.update();

    int semRet = WF_RV_OK;
    if (armPresent == true) semRet = S3.Wait_If(); //check for operative sem

    if(semRet == WF_RV_OK)
    {
        semRet = MsgSem.Wait_If();
        if(semRet == WF_RV_OK)
        {
            switch(Request.command)
            {
            case DO_NOTHING:
                resetCommand();
                break;
            case RECOVER:
                resetCommand();
                KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, CONTROLTASK_NAME, "Controller in driven in Emergency state by user request");
                for(int i = 0; i < NUM_MOT; i++) do Motors[i].emergencyStop(); while((Motors[i].State & STATUS_WORD_MASK) == SWITCH_ON_DISABLED);
                InternalEvent(ST_EMERGENCY);
                return;
                break;
            case EMERGENCY:
                resetCommand();
                KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, CONTROLTASK_NAME, "Emergency Request by the user");
                break;
            case GO_POSITION:
                resetCommand();
                for(int i = 0; i < NUM_MOT; i++) Motors[i].movePosAbs(Request.req_pos[i]*jointReduction/360);
                Status.lastCommandAccomplished = false;
                break;
            case GO_FINAL_POS:
                resetCommand();
                for(int i = 0; i < NUM_MOT; i++) if(Request.motor_selection[i]) Motors[i].movePosAbs(Motors[i].MaxPosGrad*jointReduction/360);
                Status.lastCommandAccomplished = false;
                break;
            case GO_VELOCITY:
                resetCommand();
                for(int i = 0; i < NUM_MOT; i++) Motors[i].moveVel(Request.req_vel[i]);
                Status.lastCommandAccomplished = false;
                break;
            case SET_INIT_POS:
                resetCommand();
                for(int i = 0; i < NUM_MOT; i++) if(Request.motor_selection[i]) Motors[i].setHomePosition();
                break;
            case SET_FINAL_POS:
                resetCommand();
                for(int i = 0; i < NUM_MOT; i++) if(Request.motor_selection[i]) Motors[i].MaxPosGrad = Motors[i].PositionGrad;
                break;
            case PRESHAPE:
                resetCommand();
                KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, CONTROLTASK_NAME, "Preshape request %d", Request.preshape);
                if(Request.preshape >= 0 && Request.preshape < finger_conf_num)
                {
                    for(int i = 0; i < NUM_MOT; i++) Motors[i].movePosAbs(finger_confs[Request.preshape][i]*jointReduction/360);
                    Status.lastCommandAccomplished = false;
                }
                else KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, CONTROLTASK_NAME, "Preshape request out of range");
                break;
            case STOP:
                resetCommand();
                //for(int i = 0; i < NUM_MOT; i++) Motors[i].disable();
                configurator.Stop();
                break;
            case RUN:
                resetCommand();
                //for(int i = 0; i < NUM_MOT; i++) Motors[i].enable();
                break;
            case SET_HOME:
                resetCommand();
                for(int i = 0; i < NUM_MOT; i++) Motors[i].setHomePosition();
                break;
            default:
                resetCommand();
                break;
            }
        }
    }//ifsemRet
}

void CtrlHandler::ST_Emergency()
{
    int semRet = WF_RV_OK;
    if (armPresent == true) semRet = S3.Wait_If(); //check for operative sem

    if(semRet == WF_RV_OK)
    {
        semRet = MsgSem.Wait_If();
        if(semRet == WF_RV_OK)
        {
            switch(Request.command)
            {
            case RECOVER:
                resetCommand();
                configurator.fromStop = false;
                KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, CONTROLTASK_NAME, "Controller driven in Emergency state by user request");
                InternalEvent(ST_START_CONTROLLER);
                break;
            case STOP:
                resetCommand();
                //for(int i = 0; i < NUM_MOT; i++) Motors[i].disable();
                configurator.Stop();
                break;
            case RUN:
                resetCommand();
                //for(int i = 0; i < NUM_MOT; i++) Motors[i].enable();
                configurator.Restart();
                InternalEvent(ST_WAIT_CONFIGURATION);
                break;
            default:
                resetCommand();
                break;
            }
        }
    }
}

void CtrlHandler::Start()
{
    BEGIN_TRANSITION_MAP
            TRANSITION_MAP_ENTRY(ST_START_CONTROLLER)
            TRANSITION_MAP_ENTRY(EVENT_IGNORED)
            TRANSITION_MAP_ENTRY(EVENT_IGNORED)
            TRANSITION_MAP_ENTRY(EVENT_IGNORED)
    END_TRANSITION_MAP(NULL)
}

void CtrlHandler::Update()
{
    BEGIN_TRANSITION_MAP
            TRANSITION_MAP_ENTRY(EVENT_IGNORED)
            TRANSITION_MAP_ENTRY(ST_WAIT_CONFIGURATION)
            TRANSITION_MAP_ENTRY(ST_RUNNING)
            TRANSITION_MAP_ENTRY(ST_EMERGENCY)
    END_TRANSITION_MAP(NULL)
}

void CtrlHandler::Recover()
{
    BEGIN_TRANSITION_MAP
            TRANSITION_MAP_ENTRY(EVENT_IGNORED)
            TRANSITION_MAP_ENTRY(EVENT_IGNORED)
            TRANSITION_MAP_ENTRY(EVENT_IGNORED)
            TRANSITION_MAP_ENTRY(ST_START_CONTROLLER)
     END_TRANSITION_MAP(NULL)
}

void CtrlHandler::resetCommand()
{
    Request.command = DO_NOTHING;
    int semRet = MsgSem.Signal();
    if(semRet != WF_RV_OK) KAL::DebugConsole::Write(LOG_LEVEL_ERROR, CONTROLTASK_NAME, "Error in MsgSem.Signal()");
}

void* CtrlHandler::rt_thread_handler(void *p)
{
    CtrlHandler *current_ctrl = (CtrlHandler*) p;

    int ret = 0;
    //action_done = false;

    /* task initialization */
    if_task = WF::Task::GetInstance();
    if ((ret=if_task->CreateSync(CONTROLTASK_NAME, CONTROLTASK_SAMPLETIME, WF_TASK_TYPE_USER)) != WF_RV_OK)
    {
        KAL::DebugConsole::Write(LOG_LEVEL_ERROR, CONTROLTASK_NAME, "CreateSync fallita. (valore ritorno %d)", ret);
        WF::Task::Exit();
    }

    KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, CONTROLTASK_NAME, "CtrlHandler Created");// %s %p", ST_name, this);

    if_task->SetReadyUntilPostInit();
    if_task->WaitRunning();

    // Enter hard realtime
    if_task->GotoHard();

    int semRet = WF_RV_OK;

    /* wait for arm configuration and homing */
    if (armPresent == true)
    {
        KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, CONTROLTASK_NAME, "Waiting for arm configuration and homing");
        S2.Wait();
        KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, CONTROLTASK_NAME, "Arm configuration and homing done");
    }

    KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, CONTROLTASK_NAME, "Start gripper configuration");

    /* start controller state machine */
    current_ctrl->Start();

    /* main hard real time loop */
    while (if_task->Continue() && !GLOBALSTOP){

        semRet = 1;
        if (armPresent == true) S1.GetValue(semRet); // check for arm init done

        /* update state machine */
        if ( semRet > 0)
        {
            current_ctrl->Update();
        }

        /* send all commands in the msg_outqueue */
        current_ctrl->flush_msg_queue();

        /* send sync command */
        if(!armPresent && current_ctrl->configurator.isConfigured())
        {
            current_ctrl->send_sync_msg();
        }

        if_task->WaitPeriod();

    }//while loop

    //  action_free = true;		//free the ros action

    // Enable this if your task was Hard Real Time
    if_task->GotoSoft();
    if_task->Release();
    WF::Task::Exit();

    returnValue = (void *) WF_RV_OK;
}

PubJointState::PubJointState() : Gripper(nodeIds), RosInterface(argc, argv, nodeName)
{
    if_thread.Create(thread_func, this);
}

void PubJointState::rt_thread_handler()
{       
    if_task = WF::Task::GetInstance();

    // Init call for a synchronous task
    if ((if_task->CreateSync(PUBJSTASK_NAME, PUBJTASK_SAMPLETIME)) != WF_RV_OK)
    {
        std::cerr << "Cannot create pubJointStates task." << std::endl;
        WF::Task::Exit();
    }

    KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, PUBJSTASK_NAME, "PubJointState Created");

    if_task->SetReadyUntilPostInit();

    if_task->WaitRunning();
    if_task->GotoSoft();

    while(if_task->Continue())
    {
        // read and publish joint angles and velocities
        for(unsigned int i=0; i < NUM_MOT; i++)
        {
            /* aggiorna dati interfaccia */
            Status.Velocity[i] = Motors[i].Velocity;
            Status.Position[i] = Motors[i].Position;
            Status.Current[i] = Motors[i].Current;
            Status.Control[i] = Motors[i].Control;
            Status.MotorFault[i] = Motors[i].Fault;
            Status.MaxPosGrad[i] = Motors[i].MaxPosGrad;
            Status.State[i] = Motors[i].State;
            Status.Operational[i] = Motors[i].Operational;
            Status.emerg_stop = emerg_stop;
            Status.lastCommandAccomplished = commandExecuted();

#ifdef ROS_IF
            if(rosOk())
            {
                rosPublish();
                rosSpinOnce();
            }
#endif
        }

        if_task->WaitPeriod();
    }

    GLOBALSTOP = true;

    if_task->Release();
    WF::Task::Exit();

    returnValue = (void *) WF_RV_OK;
}

/* ******************************** */
// Gripper Class Contructor
Controller::Controller(int arg_c, char** arg_v, std::string node_Name) : Gripper(nodeIds), ControllerData(arg_c, arg_v, node_Name), RosInterface(arg_c, arg_v, node_Name)//, StateMachine(CtrlHandler::ST_MAX_STATES, "Controller")
{
#ifdef _DEBUG_
    KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, "CONTROLLER", "Calling Constructor of %p", this);
    KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, "CONTROLLER", "Address of configurator = %p", &Configurator);
#endif

}

// Gripper Class Destructor
Controller::~Controller()
{
    //delete this->rosInter;
    if(TcpActive){
    }

    //if_thread.Join();
    //DEBUG("%s thread joined\n",get_label());
}

void Controller::process_message(TPCANMsg msg)
{
    if(hasID(msg.ID & NODE_ID_MASK)) updateStates(msg);
}

