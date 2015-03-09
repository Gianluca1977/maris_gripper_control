#include "ros_interface.h"
#include "controller.h"

#include <ros/ros.h>
#include <std_msgs/GripperJointPosition.h>

//#define SIMULATION

RosInterface::RosInterface(int argc, char** argv, Controller* gripper, std::string nodeName) : gripper_app(gripper)
{
	ros::init(argc,argv,nodeName.c_str());
	ros::NodeHandle nh;

	gripper_app = gripper;

	// implementation of topics to publish
    ros_publisher = nh.advertise<std_msg::GripperJointPosition>("Gripper/JointPosition", 1);

	// implementation of service server callback
}

RosInterface::~RosInterface() {

}


void RosInterface::spinInterface()
{
	ros::spinOnce();
}

bool RosInterface::okInterface()
{
	return ros::ok();
}







