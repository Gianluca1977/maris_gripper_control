/*
 * RosInterface.cpp
 *
 *  Created on: 24/feb/2012
 *      Author: robotics
 */

#include "RosInterface.h"
#include "gripper_app_WF.h"

//#define SIMULATION

RosInterface::RosInterface(int argc, char** argv, GripperApp* gripper, std::string nodeName)
{

	ros::init(argc,argv,nodeName.c_str());
	ros::NodeHandle nh;

	gripper_app = gripper;

	// implementation of topics to publish
	//pub_JointState_ = nh.advertise<sensor_msgs::JointState>("Gripper/joint_states", 1);
	// implementation of service server callback
	srv_setup = nh.advertiseService("Gripper/config",&RosInterface::srv_Callback_Setup,this);
	srv_setPreshape = nh.advertiseService("Gripper/setPreshape",&RosInterface::srv_Callback_setPreshape,this);
											
#ifdef SIMULATION
	// implementation of topics to subscribe
	sub_JointState_sim = nh.subscribe("Gripper/joint_states_act", 1000, &RosInterface::sim_Callback, this);
	act_sim_jst.name.resize(gripper_app->DOF_);
	act_sim_jst.position.resize(gripper_app->DOF_);
	act_sim_jst.velocity.resize(gripper_app->DOF_);
	act_sim_jst.effort.resize(gripper_app->DOF_);
#endif

	actionInterface = new GraspActionInterface(nodeName, gripper);
	gripper_app->srv_preshape = -1;	//invalid service set_preshape
	gripper_app->srv_radius = 0;
	actionInterface->preshape = -1;
}

RosInterface::~RosInterface() {
	delete actionInterface;
}

#ifdef SIMULATION
bool RosInterface::sim_Callback(const sensor_msgs::JointState::ConstPtr& act_jst){
	act_sim_jst = act_jst;
}
#endif


bool RosInterface::srv_Callback_Setup(gripper_app_WF::Trigger::Request& req,
							gripper_app_WF::Trigger::Response& resp)
{
	std::string errMsg;
	bool success;

	success = false;
	srv_mode = req.mode;
	errMsg.clear();
	if(srv_mode == SRV_MODE_INIT_DEVICES)
	{
		try
		{
			gripper_app->srv_mode = srv_mode;
			success = true;
			errMsg.append("Init process finished successfully!");
		}
		catch (ros::Exception& e)
		{
			errMsg.append("An exception was caught: %s ",e.what());
			isInitialized_ = false;
			success = false;
		}//catch (ros::Exception& e)
	}//if mode == 0
	else if(srv_mode == SRV_MODE_STOP)
	{
		try{
			gripper_app->stop();
			if(gripper_app->emerg_stop){
				gripper_app->emerg_stop = false;
				errMsg.append("Emergency stop released.");
			}
			else{
				gripper_app->emerg_stop = true;
				errMsg.append("Motors Stopped.");
			}
			gripper_app->srv_preshape = -1;
			success = true;
		}
		catch (ros::Exception& e)
		{
			errMsg.append("An exception was caught: %s ",e.what());
			success = false;
		}
	}//if mode == 1
	else if(srv_mode == SRV_MODE_RECOVER)
	{
		try{
			gripper_app->srv_mode = srv_mode;
			errMsg.append("Recover request has been sent");
			success = true;
		}
		catch (ros::Exception& e)
		{
			errMsg.append("An exception was caught: %s ",e.what());
			success = false;
		}
	}//if mode == 2
	else if(srv_mode == SRV_MODE_HOMING)
	{
		try{
			if(!gripper_app->candriver->isInitialized_){
				errMsg.append("Please, init the Gripper First!");
				success = false;
			}
			else{
				gripper_app->emerg_stop = false;
				gripper_app->action_free = true;
				gripper_app->srv_mode = srv_mode;

				errMsg.append("Gripper home process initialized");
				success = true;
			}
		}
		catch (ros::Exception& e)
		{
			errMsg.append("An exception was caught: %s ",e.what());
			success = false;
		}
	}// if mode == 3
	
	else if(srv_mode == SRV_MODE_TACTILE_OFFSET)
	{
		try{
			if(!gripper_app->candriver->isInitialized_){
				errMsg.append("Please, init the Gripper First!");
				success = false;
			}
			else{
				gripper_app->srv_mode = srv_mode;

				errMsg.append("Tactile sensors offset has been set");
				success = true;
			}
		}
		catch (ros::Exception& e)
		{
			errMsg.append("An exception was caught: %s ",e.what());
			success = false;
		}
	}// if mode == 4	
	
	

        else if(srv_mode == 10 || srv_mode == 11 || srv_mode == 12 || srv_mode == 13 || srv_mode == 14 || srv_mode == 15)		//DANGEROUS!!!
        {
                try{
                        gripper_app->moveVel(srv_mode,600);
                        errMsg.append("moveVel 600 finished");
                        success = true;
                }
                catch (ros::Exception& e)
                {
                        errMsg.append("An exception was caught: %s ",e.what());
                        success = false;
                }
        }// if mode == 5

	else if(srv_mode == 16 || srv_mode == 17 || srv_mode == 18 || srv_mode == 19 || srv_mode == 20 || srv_mode == 21)		//DANGEROUS!!!
       {
                try{
                        gripper_app->moveVel(srv_mode-6,-600);
                        errMsg.append("moveVel -600 finished");
                        success = true;
                }
                catch (ros::Exception& e)
                {
                        errMsg.append("An exception was caught: %s ",e.what());
                        success = false;
                }
        }// if mode == 5

	else
	{
		success = false;
		errMsg.append("Unable to send request. Invalid argument!");
	}

	resp.success = success;
	resp.error_message.data.clear();
	resp.error_message.data.append(errMsg.c_str());
	return true;
}

bool RosInterface::srv_Callback_setPreshape(gripper_app_WF::setPreshape::Request& req,
										gripper_app_WF::setPreshape::Response& resp)
{
	/*
	   int8 PRESHAPE_SPHERICAL=0
	   int8 PRESHAPE_CYLINDRICAL=1
       int8 PRESHAPE_PRECISION=2
       int8 PRESHAPE_HOOK=3
       int8 PRESHAPE_FINGERTIP=4
	*/
	std::string errMsg;
	bool success;

	int pre_sp = req.preshape;
	errMsg.clear();

	try{
		if(!gripper_app->isHomeDone)
		{
			errMsg.append("Please, Home the Gripper First!");
			std::cout << "Please, Home the Gripper First!" << std::endl;			
			success = false;
		}
		else{
			if(pre_sp < 0 || pre_sp > 30){
				errMsg.append("Error: invalid preshape");
				success = false;
			}
			else
			{
				actionInterface->preshape = pre_sp;
				errMsg.append("Preshape has been set successfully");
				success = true;
			}
		}
	}
	catch(ros::Exception& e)
	{
		errMsg.append("An exception was caught: %s ",e.what());
		success = false;
	}
	resp.error_message.data.clear();
	resp.error_message.data.append(errMsg.c_str());
	resp.success = success;
	return success;
}

/*
bool RosInterface::pubJointStates(sensor_msgs::JointState& msg)
{
	msg.header.stamp = ros::Time::now();
	pub_JointState_.publish(msg);
	return true;
}
*/

void RosInterface::spinInterface()
{
	ros::spinOnce();
}

bool RosInterface::okInterface()
{
	return ros::ok();
}


void GraspActionInterface::executeGrasp(const gripper_app_WF::graspGoalConstPtr& goal)
{
	bool success;
	float fingerOffset = goal->finger_offset;
	float radius = goal->perc_closure;
	float force = goal->force;
	float radius_palm = goal->palm_closure;

	if(preshape == -1 && gripper_app->srv_mode!=3){
		ROS_WARN("%s: Rejected. Invalid preShape. See Gripper/setPreshape service.", action_name_.c_str());
		as_.setAborted();
		success = false;
	}
	else{
		gripper_app->srv_preshape = preshape;
		gripper_app->fingerOffset = fingerOffset;
		gripper_app->srv_radius = radius;
		gripper_app->action_force = force;
		gripper_app->srv_radius_palm = radius_palm;
		gripper_app->action_perc_complete = 0;
		while(!gripper_app->action_free){
			if(as_.isPreemptRequested() || !ros::ok()){
				as_.setPreempted();
				success = false;
				gripper_app->action_free = true;
				break;
			}
			graspFeedback.percent_complete = gripper_app->action_perc_complete;
			//ROS_INFO("perc_complete: %f", graspFeedback.percent_complete);
			as_.publishFeedback(graspFeedback);
		}//while
		success = gripper_app->action_done;
		if(success)
			ROS_INFO("%s: Succeeded", action_name_.c_str());
		else
			ROS_INFO("%s: Failed", action_name_.c_str());
	}
	graspResult.done = success;
	as_.setSucceeded(graspResult);
}







