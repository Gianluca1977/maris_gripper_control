#include "main.h"
#include "ros_interface.h"
#include "interface_data.h"
#include "tcp_interface.h"
#include "controller.h"

#include <ros/ros.h>

//#define SIMULATION

RosInterface::RosInterface(int argc, char** argv, std::string name) : nodeName(name)
{
#ifdef ROS_IF
    KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, ROS_INTERFACE_NAME, "Initialize ROS interface");

    ros::init(argc,argv,nodeName.c_str());

    ros::NodeHandle nh;

    // implementation of topics to publish
    ros_publisher_pos = nh.advertise<gripper_control::GripperJointPosition>("gripper/q", 1);
    ros_publisher_vel = nh.advertise<gripper_control::GripperJointVelocity>("gripper/qdot", 1);
    ros_publisher_status = nh.advertise<gripper_control::GripperStatus>("gripper/status", 1);

    ros_service_shape = nh.advertiseService("gripper/shape", &RosInterface::selectShape, this);

    actionInterface = new ShapeActionInterface(nh, "gripper/actionshape", &Status, &Request);

    rt_thread_create();
#endif
}

RosInterface::~RosInterface()
{

}

void RosInterface::rt_thread_handler()
{
    int ret;

    /* task initialization */
    if_task = WF::Task::GetInstance();
    if ((ret = if_task->CreateSync(ROS_INTERFACE_NAME, ROS_INTERFACE_SAMPLETIME, WF_TASK_TYPE_USER)) != WF_RV_OK){
        KAL::DebugConsole::Write(LOG_LEVEL_ERROR, ROS_INTERFACE_NAME, "CreateSync fallita. (valore ritorno %d)", ret);
        WF::Task::Exit();
    }

    KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, ROS_INTERFACE_NAME, "%s Created %d", ROS_INTERFACE_NAME, sizeof(SystemStatus));

    if_task->SetReadyUntilPostInit();
    if_task->WaitRunning();

    while (if_task->Continue() && !GLOBALSTOP){
        //inviare dati alla maixbox grafica

        ret = StatusSem.Wait();
        if(ret != WF_RV_OK) KAL::DebugConsole::Write(LOG_LEVEL_ERROR, ROS_INTERFACE_NAME, "Error in StatusSem.Wait()");

        if(rosOk())
        {
            rosPublish();
            rosSpinOnce();
        }

        ret = StatusSem.Signal();
        if(ret != WF_RV_OK) KAL::DebugConsole::Write(LOG_LEVEL_ERROR, ROS_INTERFACE_NAME, "Error in StatusSem.Signal()");

        if_task->WaitPeriod();

    }
    // Enable this if your task was Hard Real Time
    // delete Status;
    if_task->Release();
    WF::Task::Exit();

    returnValue = (void *) WF_RV_OK;
}

void RosInterface::rosSpinOnce()
{
    ros::spinOnce();
}

bool RosInterface::rosOk()
{
    return ros::ok();
}

void RosInterface::rosPublish()
{
    gripper_control::GripperJointPosition rosJointPosition;
    gripper_control::GripperJointVelocity rosJointVelocity;
    gripper_control::GripperStatus rosStatus;

    rosStatus.status = -1;

    for(int i = 0; i < NUM_MOT; i++)
    {
        rosJointPosition.q[i] = Status.Position[i];
        rosJointVelocity.qdot[i] = Status.Velocity[i];
        rosStatus.status &= Status.State[i];
    }

    ros_publisher_pos.publish(rosJointPosition);
    ros_publisher_vel.publish(rosJointVelocity);
    ros_publisher_status.publish(rosStatus);
}

bool RosInterface::selectShape(gripper_control::GripperSelectShape::Request &req, gripper_control::GripperSelectShape::Response &res)
{
    int ret = RequestSem.Wait();
    if(ret != WF_RV_OK) KAL::DebugConsole::Write(LOG_LEVEL_ERROR, ROS_INTERFACE_NAME, "Error in RequestSem.Signal()");

    Request.command = PRESHAPE;
    Request.preshape = req.shape;

    ret = RequestSem.Signal();
    if(ret != WF_RV_OK) KAL::DebugConsole::Write(LOG_LEVEL_ERROR, ROS_INTERFACE_NAME, "Error in RequestSem.Signal()");

    res.result = req.shape;

    // non necessario nell'interfaccia ROS
    /*
    ret =  if_task->Sleep(100 * WF_TIME_ONE_MS);
    if(ret != WF_RV_OK) KAL::DebugConsole::Write(LOG_LEVEL_ERROR, ROS_INTERFACE_NAME, "Error in if_task->Sleep()");
    */
}

void ShapeActionInterface::executeCB(const gripper_control::GripperSelectShapeGoalConstPtr &goal)
{
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // push_back the seeds for the fibonacci sequence
    feedback_.progress = 0;
    result_.done = false;

    // publish info to the console for the user
    ROS_INFO("%s: Executing, moving to shape %i", actionName.c_str(), goal->shape);

    //int ret = RequestSem.Wait();
    //if(ret != WF_RV_OK) KAL::DebugConsole::Write(LOG_LEVEL_ERROR, ROS_INTERFACE_NAME, "Error in RequestSem.Signal()");

    Request->command = PRESHAPE;
    Request->preshape = goal->shape;

    //ret = RequestSem.Signal();
    //if(ret != WF_RV_OK) KAL::DebugConsole::Write(LOG_LEVEL_ERROR, ROS_INTERFACE_NAME, "Error in RequestSem.Signal()");

    // start executing the action
    while(!Status->lastCommandAccomplished)
    {
        // check that preempt has not been requested by the client
        if (as_.isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("%s: Preempted", actionName.c_str());
            // set the action state to preempted
            as_.setPreempted();
            success = false;
            break;
        }
        feedback_.progress = Status->Position[0];
        // publish the feedback
        as_.publishFeedback(feedback_);
        // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
        r.sleep();
    }

    if(success)
    {
        result_.done = Status->lastCommandAccomplished;
        ROS_INFO("%s: Succeeded", actionName.c_str());
        // set the action state to succeeded
        as_.setSucceeded(result_);
    }
}






