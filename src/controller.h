#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "can_driver.h"
#include "interface_data.h"
#include "tcp_interface.h"
#include "motor.h"
#include "gripper.h"
#include "supervisor.h"
#include "controllerdata.h"
#include "rt_thread.h"
#include "motor_configurator.h"
#include "state_machine.h"
#include "timer.h"
#include "ros_interface.h"

#include "wf.h"

//#include "thread"

// ROS msg 
//#include <sensor_msgs/JointState.h>


//****************************************************************************
// INCLUDES

#include <sys/mman.h>
#include <sys/poll.h>
#include <libconfig.h++>

//****************************************************************************

// DEFINES

#define STATE_FILE_OPENED         1
#define STATE_TASK_CREATED        2
#define SET_INIT_STATE(new_state) current_state |= new_state
#define RESET_INIT_STATE(new_state) current_state &= ~new_state

//****************************************************************************

#define ALLRANGE

#define BIT16_	1U<<15	//0x8000
#define BIT32_	1U<<31	//0x80000000
#define ERR_VAL BIT32_

#define TRAJ_POINTS			(100)
#define RAD_TO_GRAD			(180/M_PI)

#define GRAD_TO_RAD			(M_PI/180)
#define BASEJOINT_OFFSET	(30)
#define FINGERJOINT_OFFSET	(90)

//#define HOME_MAXPEAK				3000
//#define HOME_MAXCONT				2300
//#define MAX_GRASPCURR				10000	//maximo assorbimento totale
#define CD_MAXCURR_H				2000

#define MAINTASK_NAME				("GRIPPER_MAIN_TASK")
#define CONTROLTASK_NAME			("GRIPPER_CTRL_TASK")
#define TRIGGERTASK_NAME			("GRIPPER_TRIG_TASK")
#define PUBJSTASK_NAME				("GRIPPER_PUBJS_TASK")
#define MBXSENDTASK_NAME			("GRIPPER_MBXSEND_TASK")
#define MBXRECVTASK_NAME			("GRIPPER_MBXRECV_TASK")

#define MAINTASK_PRIORITY			(8)
#define OPENCAN_QUEUE_DIM			(2)

#define CONTROLTASK_SAMPLETIME		(10 * WF_TIME_ONE_MS)
#define PUBJTASK_SAMPLETIME     	(500 * WF_TIME_ONE_MS)
#define INIT_PHASEDELAY             (100 * WF_TIME_ONE_MS)//(1 * WF_TIME_ONE_S)//
#define HOME_TIMEOUT                (30 * WF_TIME_ONE_S)
#define HOME_BLINDELAY              (3 * WF_TIME_ONE_S)

#define SRV_MODE_DO_NOTHING	-1
#define SRV_MODE_INIT_DEVICES	0
#define SRV_MODE_STOP		1
#define SRV_MODE_RECOVER	2
#define SRV_MODE_HOMING		3
#define SRV_MODE_TACTILE_OFFSET	4

class PubJointState : virtual public CanDriver, virtual public Gripper, virtual public ControllerData, virtual public Supervisor, virtual public TcpData, private rt_thread, virtual public RosInterface
{
public:
    PubJointState();

private:
    void rt_thread_handler(void);
};

class CtrlHandler:  virtual public CanDriver, virtual public Gripper, virtual public ControllerData, virtual public Supervisor, virtual public TcpData, public StateMachine//, private rt_thread
{
public:
    CtrlHandler();
    ~CtrlHandler();

    static WF::Thread if_thread;
    static WF::Task *if_task;
    static void* returnValue;

    static MotorConfigurator Configurator;
    static Timer WaitTimer;

    static WF::BinarySemaphore S1;
    static WF::BinarySemaphore S2;
    static WF::BinarySemaphore S3;

    //static StateStruct StateMap[10];

    // state machine state functions
    void ST_Start_Controller();
    void ST_Wait_Configuration();
    void ST_Running();
    void ST_Emergency();

    void Start();
    void Update();
    void Recover();

    // state map to define state function order
    //const StateStruct* GetStateMap(){return (const StateStruct*) &StateMap[0];}

    BEGIN_STATE_MAP
        STATE_MAP_ENTRY(&CtrlHandler::ST_Start_Controller)
        STATE_MAP_ENTRY(&CtrlHandler::ST_Wait_Configuration)
        STATE_MAP_ENTRY(&CtrlHandler::ST_Running)
        STATE_MAP_ENTRY(&CtrlHandler::ST_Emergency)
    END_STATE_MAP

    // state enumeration order must match the order of state
    // method entries in the state map
    enum E_States {
        ST_START_CONTROLLER = 0,
        ST_WAIT_CONFIGURATION,
        ST_RUNNING,
        ST_EMERGENCY,
        ST_MAX_STATES
    };

private:
    static void* rt_thread_handler(void*);
};

/* Gripper Controller Class */
class Controller : virtual public CtrlHandler, virtual public PubJointState, virtual public TcpSend, virtual public TcpReceive
{
public:
    Controller(int arg_c, char** arg_v, std::string node_Name);
    ~Controller();

    void process_message();
    void process_message(TPCANMsg msg);

	//void initStateMachine(int& initPhase, void* p);

	void install_signal(void);
	void signal_handler(int unused);
};

#endif /* CONTROLLER_H_ */
