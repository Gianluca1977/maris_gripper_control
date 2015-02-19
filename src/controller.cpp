//============================================================================
// Name        : gripper_control.cpp
// Author      : Gianluca Palli
// Version     :
// Copyright   : 
// Description :
//============================================================================

#include "controller.h"
#include "timer.h"

#include <boost/algorithm/string.hpp>

CtrlHandler::CtrlHandler() : Gripper(nodeIds), Configurator(Motors), StateMachine(ST_MAX_STATES)
{
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

    if_thread.Create(thread_func, this);
}

void CtrlHandler::ST_Start_Controller()
{
    Configurator.StartConfiguration(); // calling within ST_START_CONTROLLER, then transit to ST_WAIT_CONFIGURATION
    InternalEvent(ST_WAIT_CONFIGURATION);
}

void CtrlHandler::ST_Wait_Configuration()
{
    Configurator.WaitTime.Update(); // calling within ST_WAIT_CONFIGURATION
    if(Configurator.isConfigured())
    {
        if (armPresent == true) S2.Signal();//risvegliamo il braccio - torna a dare il sync
        srv_mode = SRV_MODE_DO_NOTHING;
        InternalEvent(ST_RUNNING);
    }
}

void CtrlHandler::ST_Running()
{

}

void CtrlHandler::rt_thread_handler()
{
    long long timeNow, timeIni;
    bool initTime = true;
    bool fromFault = false;
    int initPhase = INIT_BOOTUP_DEV;
    int nextPhase = INIT_BOOTUP_DEV;

    bool start_moving = false;

    int ret = 0;

    unsigned int cnt = 0;

    std::vector<int> auxNode;

    auxNode.reserve(NUM_MOT);
    for(cnt = 0; cnt < NUM_MOT; cnt++)
        auxNode.push_back(Motors[cnt].ID);

    std::vector<long> data(NUM_MOT,0);

    action_done = false;

    /* task initialization */
    if_task = WF::Task::GetInstance();
    if ((ret=if_task->CreateSync(CONTROLTASK_NAME, CONTROLTASK_SAMPLETIME, WF_TASK_TYPE_USER)) != WF_RV_OK){
        KAL::DebugConsole::Write(LOG_LEVEL_ERROR, CONTROLTASK_NAME, "CreateSync fallita. (valore ritorno %d)", ret);
        WF::Task::Exit();
    }

    KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, CONTROLTASK_NAME, "CtrlHandler Created");

    if_task->SetReadyUntilPostInit();
    if_task->WaitRunning();

    // Enter hard realtime
    if_task->GotoHard();

    int semRet = WF_RV_OK;
    unsigned int dof = NUM_MOT;

    if (armPresent == true) S2.Wait();

    /* main hard real time loop */

    ExternalEvent(ST_START_CONTROLLER);

    while (if_task->Continue() && !GLOBALSTOP){

        //check if everyone are operative.
        for(cnt = 0;cnt < dof ; cnt++){
            if(!Motors[cnt].Operational && !doHome && isHomeDone){
                if(!emerg_stop){
                    //srv_mode = SRV_MODE_RECOVER;
                    //fault = true;	//Signalizes to trigger thread to recover fault motor
                    fault = false;	//Signalizes to trigger thread to recover fault motor
                }
                break;
            }
        }

        if(emerg_stop){
            srv_mode = SRV_MODE_DO_NOTHING;
            auxInternalCall = -1;
            doHome = false;
            action_free = true;
            tcpDoHome = false;
            //homePhase = 0;
            data.assign(NUM_MOT,0);
            //moveVel(data);
        }

        switch(srv_mode){

        case SRV_MODE_INIT_DEVICES: //init all devices

            semRet = 1;
            if (armPresent == true) S1.GetValue(semRet); // check for arm init done

            if ( semRet > 0)
            {

                /*
                if(initTime){
                    timeIni = llround(KAL::GetTime());
                    for(cnt = 0; cnt < dof; cnt++) Motors[cnt].init(initPhase);
                    initTime = false;
                    initPhase = INIT_WAIT;
                }

                switch(initPhase){

                case INIT_WAIT:
                    timeNow = llround(KAL::GetTime());

                    if(timeNow - timeIni > INIT_PHASEDELAY){
                        for(cnt = 0; cnt < dof; cnt++) auxNode.at(cnt) = Motors[cnt].ID;
                        initPhase = nextPhase;
                    }

                    break;

                case INIT_BOOTUP_DEV:

                    initTime = true;
                    nextPhase = INIT_START_DEV;

                    for(cnt = 0; cnt < dof; cnt++){
                        if(!Motors[cnt].BootUp) nextPhase = INIT_BOOTUP_DEV;
                        else auxNode.at(cnt) = 0;
                    }

                    if(initPhase != nextPhase) for(cnt = 0; cnt < dof; cnt++) auxNode.at(cnt) = Motors[cnt].ID;
                    initPhase = nextPhase;
                    break;

                case INIT_START_DEV:

                    initTime = true;
                    nextPhase = INIT_SHUTDOWN_DEV;

                    for(cnt = 0; cnt < dof; cnt++){
                        if((Motors[cnt].State & STATUS_WORD_MASK) != SWITCH_ON_DISABLED) nextPhase = INIT_START_DEV;
                        else auxNode.at(cnt) = 0;
                    }

                    if(initPhase != nextPhase) for(cnt = 0; cnt < dof; cnt++) auxNode.at(cnt) = Motors[cnt].ID;
                    initPhase = nextPhase;
                    break;

                case INIT_SHUTDOWN_DEV:

                    initTime = true;
                    nextPhase = INIT_SWITCH_ON_DEV;

                    for(cnt = 0; cnt < dof; cnt++){
                        if((Motors[cnt].State & STATUS_WORD_MASK) != READY_TO_SWITCH_ON) nextPhase = INIT_SHUTDOWN_DEV;
                        else auxNode.at(cnt) = 0;
                    }

                    if(initPhase != nextPhase) for(cnt = 0; cnt < dof; cnt++) auxNode.at(cnt) = Motors[cnt].ID;
                    initPhase = nextPhase;
                    break;

                case INIT_SWITCH_ON_DEV:

                    initTime = true;
                    nextPhase = INIT_ENABLE_OP_DEV;
                    for(cnt = 0; cnt < dof; cnt++){
                        if((Motors[cnt].State & STATUS_WORD_MASK) != SWITCHED_ON) nextPhase = INIT_SWITCH_ON_DEV;
                        else auxNode.at(cnt) = 0;
                    }

                    if(initPhase != nextPhase) for(cnt = 0; cnt < dof; cnt++) auxNode.at(cnt) = Motors[cnt].ID;
                    initPhase = nextPhase;
                    break;

                case INIT_ENABLE_OP_DEV:

                    initTime = true;
                    nextPhase = INIT_ENABLED;
                    for(cnt = 0; cnt < dof; cnt++){
                        if((Motors[cnt].State & STATUS_WORD_MASK) != OPERATION_ENABLED) nextPhase = INIT_ENABLE_OP_DEV;
                        else auxNode.at(cnt) = 0;
                    }

                    if(initPhase != nextPhase) for(cnt = 0; cnt < dof; cnt++) auxNode.at(cnt) = Motors[cnt].ID;
                    initPhase = nextPhase;
                    break;

                case INIT_ENABLED:
                    if(fromFault) nextPhase = INIT_DONE;
                    else{
                        KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, TRIGGERTASK_NAME, "All devices has been initialized! Configuring it... ");
                        initTime = true;
                        nextPhase = INIT_TRACE_CONF;
                        for(cnt = 0; cnt < dof; cnt++) auxNode.at(cnt) = Motors[cnt].ID;
                    }

                    initPhase = nextPhase;
                    break;

                case INIT_TRACE_CONF:
                    initTime = true;
                    nextPhase = INIT_SYNC_CONF;
                    initPhase = nextPhase;
                    break;

                case INIT_SYNC_CONF:
                    initTime = true;
                    nextPhase = INIT_TPDO1_CONF;
                    initPhase = nextPhase;
                    break;

                case INIT_TPDO1_CONF:
                    initTime = true;
                    nextPhase = INIT_FAULHABER_CONF;
                    initPhase = nextPhase;
                    break;

                case INIT_FAULHABER_CONF:
                    initTime = true;
                    nextPhase = INIT_DONE;
                    initPhase = nextPhase;
                    break;

                case INIT_DONE:
                    if(!fromFault) KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, TRIGGERTASK_NAME, "Done. We are ready to roll! ");
                    initPhase = INIT_NONE;
                    initTime = false;

                    srv_mode = SRV_MODE_DO_NOTHING;
                    for(cnt = 0; cnt < dof; cnt++) auxNode.at(cnt) = Motors[cnt].ID; //recover auxVector

                    if (armPresent == true) S2.Signal();//risvegliamo il braccio - torna a dare il sync

                    break;

                case INIT_FAULT:	//reset fault state
                    fromFault = true;
                    nextPhase = INIT_SHUTDOWN_DEV;
                    initTime = true;
                    for(cnt = 0; cnt < dof; cnt++){
                        auxNode.at(cnt) = Motors[cnt].ID;
                        if(!Motors[cnt].Fault) auxNode.at(cnt) = 0;
                    }

                    initPhase = nextPhase;
                    break;
                case INIT_NONE:
                default:
                    break;
                }
                */
            }
            break;

        case SRV_MODE_RECOVER:
            //recover devices;
            for(cnt = 0; cnt < dof; cnt++) if(Motors[cnt].Fault)
            {
                initPhase = INIT_FAULT;
                break;
            }
            srv_mode = SRV_MODE_INIT_DEVICES;
            break;

        case SRV_MODE_TACTILE_OFFSET:
            if(!setTactOffset) setTactOffset = true;
            break;

        case SRV_MODE_STOP:
            emerg_stop = false;
            initPhase = INIT_BOOTUP_DEV;
            action_free = true;
            break;

        case SRV_MODE_HOMING:
            doHome = true;
            isHomeDone = false;
            break;

        case SRV_MODE_DO_NOTHING:
        default:
            srv_mode = SRV_MODE_DO_NOTHING;
            break;
        }//switch(srv_mode)



        semRet = WF_RV_OK;
        if (armPresent == true) semRet = S3.Wait_If(); //check for operative sem

        if(semRet == WF_RV_OK){
            /* *********** via TCP Using the grafic interface ***********/

            if(!emerg_stop && TcpActive)
            {
                if(Request.recover){
                    srv_mode = SRV_MODE_RECOVER;
                    continue;
                }

                if(!Request.pos && !doHome && !tcpDoHome){	//pure velocity input
                    for(unsigned int k=0;k<dof;k++){
                        data[k] = Request.req_vel[k];
                    }
                    //moveVel(data);
                }
                //set home position button
                if(Request.setIniPos){
                    if(Request.butNum == 3){
                        Motors[0].setHomePosition();
                    }
                    else if(Request.butNum == 4){
                        Motors[1].setHomePosition();
                    }
                    else if(Request.butNum == 5){
                        Motors[2].setHomePosition();
                    }
                }
                //set max range button
                else if(Request.setFinPos){
                    if(Request.butNum == 6)
                        Motors[0].MaxPosGrad = Motors[0].PositionGrad;
                    if(Request.butNum == 7)
                        Motors[1].MaxPosGrad = Motors[1].PositionGrad;
                    if(Request.butNum == 8)
                        Motors[2].MaxPosGrad = Motors[2].PositionGrad;
                }
                //position control button
                if(Request.pos && !doHome && !tcpDoHome){
                    if(Request.goIniPos){
                        //for(unsigned int i=0;i<dof;i++) des_pos[i] = Motors[i].Position_grad;
                        if(Request.butNum == 9){
                            //des_pos[0] = 0;
                            Motors[0].movePosAbs(0);
                        }
                        else if(Request.butNum == 10){
                            //des_pos[1] = 0;
                            Motors[1].movePosAbs(0);
                        }
                        else if(Request.butNum == 11){
                            //des_pos[2] = 0;
                            Motors[2].movePosAbs(0);
                        }
                        //loadPosAbs(des_pos);
                        start_moving = true;

                    } else if(Request.goFinPos){
                        //for(unsigned int i=0;i<dof;i++) des_pos[i] = Motors[i].Position_grad;
                        if(Request.butNum == 12){
                            //des_pos[0] = Motors[0].max_range_grad;
                            Motors[0].movePosAbs(Motors[0].MaxPosGrad*jointReduction/360);
                        }
                        else if(Request.butNum == 13){
                            //des_pos[1] = Motors[1].max_range_grad;
                            Motors[1].movePosAbs(Motors[1].MaxPosGrad*jointReduction/360);
                        }
                        else if(Request.butNum == 14){
                            //des_pos[2] = Motors[2].max_range_grad;
                            Motors[2].movePosAbs(Motors[2].MaxPosGrad*jointReduction/360);
                        }
                        //loadPosAbs(des_pos);
                        start_moving = true;

                    } else {
                        int req_pose = Request.desired_conf;
                        for(int i = 0; i < 3; i++) Motors[i].movePosAbs(Motors[i].MaxPosGrad*jointReduction/360);
                    }

                    //ret = calcPID(des_pos,data);
                    //moveVel(data);

                }//if pos

                //if(tcp->request->doHome && start_moving) {
                /*if(start_moving) {
                    for(unsigned i=0;i<3; i++) startMovePos();
                    start_moving = false;
                    KAL::DebugConsole::Write(LOG_LEVEL_INFO, CONTROLTASK_NAME, "Start Motion");

                }*/

            }//if tcp


            /******** end of via TCP ******/


            /************* big control if using ROS interface*************/
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

        }//ifsemRet

        /* send all commands in the msg_outqueue */
        flush_msg_queue();

        /* send sync command */
        if(!armPresent && isOperative()){
            send_sync_msg();
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
                Status.PositionGrad[i] = Motors[i].Position;
                Status.Current[i] = Motors[i].Current;
                Status.Control[i] = Motors[i].Control;
                Status.MotorFault[i] = Motors[i].Fault;
                Status.MaxPosGrad[i] = Motors[i].MaxPosGrad;
                Status.State[i] = Motors[i].State;
                //Status.statusword_low[i] = Motors[i].statusword_low;
                Status.Operational[i] = Motors[i].Operational;
                /*
                Status.homePhase = homePhase;
                Status.initPhase = initPhase;
                */
                Status.srv_mode = srv_mode;
                Status.srv_preshape = srv_preshape;
                //Status.isInitialized_ = ;
                Status.emerg_stop = emerg_stop;
                //Status.resetSensors = ;
            }
        }

        /* ***************************  */

        if(TcpActive){
            emerg_stop = Request.emerg_stop;
            Status.actCycle = 0;

            if(Request.parking)
                srv_preshape = 5;

            if(Request.doHome)
                //tcpDoHome = true;
                if(Request.manualHomeDone)
                    isHomeDone = true;
            if(Request.highLevel)
                setTactOffset = true;
        }
        // if(tcp!=NULL) TcpActive = Request.tcpActive;

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
Controller::Controller() : Gripper(nodeIds)
{
}

// Gripper Class Destructor
Controller::~Controller()
{
    //delete this->rosInter;
    if(TcpActive){
    }
}

void Controller::process_message(TPCANMsg msg)
{
    if(hasID(msg.ID & NODE_ID_MASK)) updateStates(msg);
}

