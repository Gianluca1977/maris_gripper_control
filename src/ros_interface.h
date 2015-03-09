#ifndef ROSINTERFACE_H_
#define ROSINTERFACE_H_

#include "controller.h"
#include "can_driver.h"

// ROS includes
#include <ros/ros.h>
#include <string>


// ROS message includes
//#include <sensor_msgs/JointState.h>

class RosInterface {
public:
    RosInterface(int argc, char** argv, Controller* gripper, std::string name);
	virtual ~RosInterface();

	void init();
	void spinInterface();
	bool okInterface();

private:
    //declaration of service server callback

	// declaration of topics to publish
    ros::Publisher ros_publisher;

	// service servers

protected:
    Controller* gripper_app;

    std::string nodeName;
};


#endif /* ROSINTERFACE_H_ */
