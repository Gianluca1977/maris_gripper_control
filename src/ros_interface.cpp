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

    actionInterface = new ShapeActionInterface(nh, "gripper/actionshape");
#endif
}

RosInterface::~RosInterface()
{

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

    rosStatus.status = 0xFFFF;

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
    int ret = MsgSem.Wait();
    if(ret != WF_RV_OK) KAL::DebugConsole::Write(LOG_LEVEL_ERROR, ROS_INTERFACE_NAME, "Error in MsgSem.Signal()");

    Request.command = PRESHAPE;
    Request.preshape = req.shape;

    ret = MsgSem.Signal();
    if(ret != WF_RV_OK) KAL::DebugConsole::Write(LOG_LEVEL_ERROR, ROS_INTERFACE_NAME, "Error in MsgSem.Signal()");

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
    feedback_.done = false;
    result_.done = false;

    // publish info to the console for the user
    ROS_INFO("%s: Executing, moving to shape %i", nodeName.c_str(), goal->shape);

    int ret = MsgSem.Wait();
    if(ret != WF_RV_OK) KAL::DebugConsole::Write(LOG_LEVEL_ERROR, ROS_INTERFACE_NAME, "Error in MsgSem.Signal()");

    Request.command = PRESHAPE;
    Request.preshape = goal->shape;

    ret = MsgSem.Signal();
    if(ret != WF_RV_OK) KAL::DebugConsole::Write(LOG_LEVEL_ERROR, ROS_INTERFACE_NAME, "Error in MsgSem.Signal()");

    // start executing the action
    while(!Status.lastCommandAccomplished)
    {
        // check that preempt has not been requested by the client
        if (as_.isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("%s: Preempted", nodeName.c_str());
            // set the action state to preempted
            as_.setPreempted();
            success = false;
            break;
        }
        feedback_.done = Status.lastCommandAccomplished;
        // publish the feedback
        as_.publishFeedback(feedback_);
        // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
        r.sleep();
    }

    if(success)
    {
        result_.done = feedback_.done;
        ROS_INFO("%s: Succeeded", nodeName.c_str());
        // set the action state to succeeded
        as_.setSucceeded(result_);
    }
}






