//============================================================================
// Name        : controller.cpp
// Author      : Gianluca Palli
// Version     :
// Copyright   : 
// Description :
//============================================================================

#include "controller.h"
#include "timer.h"

#include <boost/algorithm/string.hpp>

MotorConfigurator CtrlHandler::Configurator = MotorConfigurator();

Timer CtrlHandler::WaitTimer = Timer();

WF::Thread CtrlHandler::if_thread;
WF::Task* CtrlHandler::if_task;
void* CtrlHandler::returnValue;

WF::BinarySemaphore CtrlHandler::S1;
WF::BinarySemaphore CtrlHandler::S2;
WF::BinarySemaphore CtrlHandler::S3;

CtrlHandler::CtrlHandler() : StateMachine(CtrlHandler::ST_MAX_STATES, "CtrlHandler"), Gripper(nodeIds)//, Configurator(Motors)
{
    KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, CONTROLTASK_NAME, "Calling Constructor of %p", this);

    Configurator.Init(Motors);
    WaitTimer.Init(INIT_PHASEDELAY);

    KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, CONTROLTASK_NAME, "Address of configurator = %p", &Configurator);

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

    ret = MsgSem.Create("MSGSEM", WF_SEMAPHORE_COUNTING, 0);
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
    //DEBUG("%s thread joined\n",get_label());
}

void CtrlHandler::ST_Start_Controller()
{
    //KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, CONTROLTASK_NAME, "Calling Configurator.StartConfiguration() of %s %p %p from %p", this->Configurator.ST_name, &(this->Configurator), GetStateMap(), this);
    Configurator.StartConfiguration(); // calling within ST_START_CONTROLLER, then transit to ST_WAIT_CONFIGURATION
    WaitTimer.Start();
    KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, CONTROLTASK_NAME, "Configurator Started");
    InternalEvent(ST_WAIT_CONFIGURATION);
}

void CtrlHandler::ST_Wait_Configuration()
{
    if(!armPresent && Configurator.fromStop)
    {
        send_sync_msg();
    }

    WaitTimer.Update(); // calling within ST_WAIT_CONFIGURATION
    if(Configurator.isConfigured())
    {
        KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, CONTROLTASK_NAME, "Motors are Configured");
        if (armPresent) S2.Signal();//risvegliamo il braccio - torna a dare il sync        
        InternalEvent(ST_RUNNING);
        KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, CONTROLTASK_NAME, "Controller in Running state");
    }
    else
    {
        if(WaitTimer.isExpired())
        {
            Configurator.timerExpired();
            if(!WaitTimer.Restart()) KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, "TIMER", "Error restarting timer");\
        }
    }
}

void CtrlHandler::ST_Running()
{
    if(!isOperative() || !Configurator.isConfigured())
    {
        KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, CONTROLTASK_NAME, "Controller in Emergency state");
        InternalEvent(ST_EMERGENCY);
        return;
    }

    int semRet = WF_RV_OK;
    if (armPresent == true) semRet = S3.Wait_If(); //check for operative sem

    if(semRet == WF_RV_OK)
    {
        /* *********** via TCP Using the grafic interface ***********/
        if(TcpActive)
        {
            semRet = MsgSem.Wait_If();
            if(semRet == WF_RV_OK)
            {
                switch(Request.command)
                {
                case DO_NOTHING:
                    break;
                case RECOVER:
                    Request.command = DO_NOTHING;
                    KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, CONTROLTASK_NAME, "Controller in driven in Emergency state by user request");
                    for(int i = 0; i < NUM_MOT; i++) do Motors[i].emergencyStop(); while((Motors[i].State & STATUS_WORD_MASK) == SWITCH_ON_DISABLED);
                    InternalEvent(ST_EMERGENCY);
                    return;
                    break;
                case EMERGENCY:
                    Request.command = DO_NOTHING;
                    KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, CONTROLTASK_NAME, "Emergency Request by the user");
                    break;
                case GO_POSITION:
                    Request.command = DO_NOTHING;
                    for(int i = 0; i < NUM_MOT; i++) Motors[i].movePosAbs(Request.req_pos[i]*jointReduction/360);
                    break;
                case GO_FINAL_POS:
                    Request.command = DO_NOTHING;
                    for(int i = 0; i < NUM_MOT; i++) if(Request.motor_selection[i]) Motors[i].movePosAbs(Motors[i].MaxPosGrad*jointReduction/360);
                    break;
                case GO_VELOCITY:
                    Request.command = DO_NOTHING;
                    for(int i = 0; i < NUM_MOT; i++) Motors[i].moveVel(Request.req_vel[i]);
                    break;
                case SET_INIT_POS:
                    Request.command = DO_NOTHING;
                    for(int i = 0; i < NUM_MOT; i++) if(Request.motor_selection[i]) Motors[i].setHomePosition();
                    break;
                case SET_FINAL_POS:
                    Request.command = DO_NOTHING;
                    for(int i = 0; i < NUM_MOT; i++) if(Request.motor_selection[i]) Motors[i].MaxPosGrad = Motors[i].PositionGrad;
                    break;
                case PRESHAPE:
                    Request.command = DO_NOTHING;
                    KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, CONTROLTASK_NAME, "Preshape request %d", Request.preshape);
                    if(Request.preshape >= 0 && Request.preshape < finger_conf_num) for(int i = 0; i < NUM_MOT; i++) Motors[i].movePosAbs(finger_confs[Request.preshape][i]*jointReduction/360);
                    else KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, CONTROLTASK_NAME, "Preshape request out of range");
                    break;
                case STOP:
                    Request.command = DO_NOTHING;
                    //for(int i = 0; i < NUM_MOT; i++) Motors[i].disable();
                    Configurator.Stop();
                    break;
                case RUN:
                    Request.command = DO_NOTHING;
                    //for(int i = 0; i < NUM_MOT; i++) Motors[i].enable();
                    break;
                default:
                    break;
                }

                /*
                semRet = MsgSem.Signal();
                if(semRet != WF_RV_OK) KAL::DebugConsole::Write(LOG_LEVEL_ERROR, CONTROLTASK_NAME, "Error in MsgSem.Signal()");
                */
            }
        }//if tcp
        /******** end of via TCP ******/

        /************* big control if using ROS interface*************/
        /*
        if(isHomeDone &&  !fault && !emerg_stop){
            //fKinematics();
            switch(srv_preshape){
            case 0:

                break;
            case 1:

                break;
            default:
                break;
            }//switch((srv_preshape)

            if(emerg_stop){
                data.assign(dof,0);
                srv_preshape = -1;
                auxInternalCall = -1;
                action_free = true;
                action_done = false;
            }
            if(fault)//se qualche Motore in fault fermi tutti.
                data.assign(dof,0);

            //moveVel(data);

        }//big control if
        */
    }//ifsemRet
}

void CtrlHandler::ST_Emergency()
{
    int semRet = WF_RV_OK;
    if (armPresent == true) semRet = S3.Wait_If(); //check for operative sem

    if(semRet == WF_RV_OK)
    {
        /* *********** via TCP Using the grafic interface ***********/
        if(TcpActive)
        {
            semRet = MsgSem.Wait_If();
            if(semRet == WF_RV_OK)
            {
                switch(Request.command)
                {
                case RECOVER:
                    Request.command = DO_NOTHING;
                    Configurator.fromStop = false;
                    KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, CONTROLTASK_NAME, "Controller driven in Emergency state by user request");
                    InternalEvent(ST_START_CONTROLLER);                    
                    break;
                case STOP:
                    Request.command = DO_NOTHING;
                    //for(int i = 0; i < NUM_MOT; i++) Motors[i].disable();
                    Configurator.Stop();
                    break;
                case RUN:
                    Request.command = DO_NOTHING;
                    //for(int i = 0; i < NUM_MOT; i++) Motors[i].enable();
                    Configurator.Restart();
                    InternalEvent(ST_WAIT_CONFIGURATION);
                    break;
                default:
                    break;
                }
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

void* CtrlHandler::rt_thread_handler(void *p)
{
    CtrlHandler *current_ctrl = (CtrlHandler*) p;

    int ret = 0;
    //action_done = false;

    /* task initialization */
    if_task = WF::Task::GetInstance();
    if ((ret=if_task->CreateSync(CONTROLTASK_NAME, CONTROLTASK_SAMPLETIME, WF_TASK_TYPE_USER)) != WF_RV_OK){
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
        if(!armPresent && current_ctrl->Configurator.isConfigured())
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

PubJointState::PubJointState() : Gripper(nodeIds)
{
    if_thread.Create(thread_func, this);
}

void PubJointState::rt_thread_handler()
{
    std::string msg;
    std::stringstream ss;
    int dof = NUM_MOT;
    std::vector<double> Position(dof,0);
    double auxDouble;

    std::vector<long> req_velocity(dof,0);

    /*
    sensor_msgs::JointState ros_msg;

    ros_msg.name.resize(dof);
    ros_msg.position.resize(dof);
    ros_msg.velocity.resize(dof);
    ros_msg.effort.resize(dof);
    */

    if_task = WF::Task::GetInstance();

    // Init call for a synchronous task
    if ((if_task->CreateSync(PUBJSTASK_NAME, PUBJTASK_SAMPLETIME)) != WF_RV_OK){
        std::cerr << "Cannot create pubJointStates task." << std::endl;
        WF::Task::Exit();
    }

    KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, PUBJSTASK_NAME, "PubJointState Created");

    if_task->SetReadyUntilPostInit();

    if_task->WaitRunning();
    if_task->GotoSoft();


    while(if_task->Continue()){
#ifdef ROS_IF
        if(rosInter->okInterface()) rosInter->spinInterface();
#endif        
        // read and publish joint angles and velocities
        for(unsigned int i=0; i<NUM_MOT;i++)
        {
#ifdef ROS_IF
            if(rosInter->okInterface())
            {
                ros_msg.position[i] = roundToSignificant(Motors[i].Position_grad, 4);
                ros_msg.effort[i] = roundToSignificant(Motors[i].last_curr, 4);
            }
#endif
            auxDouble = PUBJTASK_SAMPLETIME*pow(10,-11);
            auxDouble = (Motors[i].PositionGrad-Position[i])/auxDouble;

            //ros_msg.velocity[i]	= auxDouble*3.35;

            Motors[i].Velocity = roundToSignificant(auxDouble*3.35, 4);
            Position[i] = roundToSignificant(Motors[i].PositionGrad,4);

            /* aggiorna interfaccia tcp */
            if(TcpActive){
                Status.Velocity[i] = Motors[i].Velocity;//auxDouble*3.35;
                Status.PositionGrad[i] = Motors[i].PositionGrad;
                Status.Current[i] = Motors[i].Current;
                Status.Control[i] = Motors[i].Control;
                Status.MotorFault[i] = Motors[i].Fault;
                Status.MaxPosGrad[i] = Motors[i].MaxPosGrad;
                Status.State[i] = Motors[i].State;
                Status.Operational[i] = Motors[i].Operational;
                Status.emerg_stop = emerg_stop;
            }
        }

        /* ***************************  */

        //rosInter->pubJointStates(ros_msg);
        if_task->WaitPeriod();
    }

    GLOBALSTOP = true;

    if_task->Release();
    WF::Task::Exit();

    returnValue = (void *) WF_RV_OK;
}

/* ******************************** */
// Gripper Class Contructor
Controller::Controller() : Gripper(nodeIds)//, StateMachine(CtrlHandler::ST_MAX_STATES, "Controller")
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

