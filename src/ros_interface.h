/*
 * RosInterface.h
 *
 *  Created on: 24/feb/2012
 *      Author: robotics
 */

#ifndef ROSINTERFACE_H_
#define ROSINTERFACE_H_

#include "gripper_control.h"
#include "can_driver.h"

// ROS includes
#include <ros/ros.h>
#include <string>


// ROS message includes
//#include <sensor_msgs/JointState.h>
#include <gripper_app_WF/Trigger.h>
#include <gripper_app_WF/setPreshape.h>

#include <actionlib/server/simple_action_server.h>
#include <gripper_app_WF/graspAction.h>

class GraspActionInterface;
class GripperApp;
//class CanDriver;

class RosInterface {
public:
	RosInterface(int argc, char** argv, GripperApp* gripper, std::string name);
	virtual ~RosInterface();

	void init();
	void spinInterface();
	bool okInterface();
	//bool pubJointStates(sensor_msgs::JointState& msg);

#ifdef SIMULATION
	sensor_msgs::JointState act_sim_jst;
#endif

private:
	//declaration of service server callback

	bool srv_Callback_Setup(gripper_app_WF::Trigger::Request& req,
			gripper_app_WF::Trigger::Response& resp);

	bool srv_Callback_setPreshape(gripper_app_WF::setPreshape::Request& req,
			gripper_app_WF::setPreshape::Response& resp);

#ifdef SIMULATION
	// declaration of topics to subscribe
	ros::Subscriber sub_JointState_sim;
	bool sim_Callback(const sensor_msgs::JointState::ConstPtr& act_jst);
#endif

	// declaration of topics to publish
	ros::Publisher pub_JointState_;
	// service servers
	ros::ServiceServer srv_setup;
	ros::ServiceServer srv_setPreshape;

	bool isInitialized_;

	int srv_mode;
	//int stdVel;
	//int srv_preshape;
	std::vector<float> srv_geom_params;

protected:
	GripperApp* gripper_app;

	GraspActionInterface* actionInterface;
	std::string nodeName;
};


class GraspActionInterface
{
protected:
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<gripper_app_WF::graspAction> as_;
	std::string action_name_;
	// create messages that are used to published feedback/result
	gripper_app_WF::graspFeedback graspFeedback;
	gripper_app_WF::graspResult graspResult;
	GripperApp* gripper_app;

public:
	void executeGrasp(const gripper_app_WF::graspGoalConstPtr& goal);
	GraspActionInterface(std::string name, GripperApp* gripper ) :
		as_(nh_, name, boost::bind(&GraspActionInterface::executeGrasp, this, _1), false),
				action_name_(name)
				{
					as_.start();
					this->gripper_app = gripper;
				}

	~GraspActionInterface(void){}
	int preshape;
};


#endif /* ROSINTERFACE_H_ */
