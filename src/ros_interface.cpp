#include "main.h"
#include "ros_interface.h"
#include "interface_data.h"
#include "tcp_interface.h"
#include "controller.h"

#include <ros/ros.h>
#include <gripper_control/GripperJointPosition.h>
#include <gripper_control/GripperJointVelocity.h>

//#define SIMULATION

RosInterface::RosInterface(int argc, char** argv, std::string nodeName)
{
#ifdef ROS_IF
	ros::init(argc,argv,nodeName.c_str());
	ros::NodeHandle nh;

	// implementation of topics to publish
    ros_publisher_pos = nh.advertise<gripper_control::GripperJointPosition>("Gripper/JointPosition", 1);
    ros_publisher_vel = nh.advertise<gripper_control::GripperJointVelocity>("Gripper/JointVelocity", 1);

	// implementation of service server callback   
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

    for(int i = 0; i < NUM_MOT; i++)
    {
        rosJointPosition.q[i] = Status.PositionGrad[i];
        rosJointVelocity.qdot[i] = Status.Velocity[i];
    }

    ros_publisher_pos.publish(rosJointPosition);
    ros_publisher_vel.publish(rosJointVelocity);
}







