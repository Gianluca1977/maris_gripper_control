#ifndef ROSINTERFACE_H_
#define ROSINTERFACE_H_

#include "interface_data.h"
#include "tcp_interface.h"

// ROS includes
#include <ros/ros.h>
#include <string>

#define ROS_IF

class RosInterface : virtual protected TcpData
{
public:
    RosInterface(int argc, char** argv, std::string name);
	virtual ~RosInterface();

    void rosSpinOnce();
    bool rosOk();
    void rosPublish();

private:
    //declaration of service server callback

	// declaration of topics to publish
    ros::Publisher ros_publisher_pos;
    ros::Publisher ros_publisher_vel;

	// service servers

protected:

    std::string nodeName;
};


#endif /* ROSINTERFACE_H_ */
