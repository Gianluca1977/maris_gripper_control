#ifndef ROSINTERFACE_H_
#define ROSINTERFACE_H_

#include "interface_data.h"
#include "tcp_interface.h"
#include "rt_thread.h"
#include "wf.h"

// ROS includes
#include <ros/ros.h>
#include <string>

#include <actionlib/server/simple_action_server.h>

// ROS Messages
#include <gripper_control/GripperJointPosition.h>
#include <gripper_control/GripperJointVelocity.h>
#include <gripper_control/GripperStatus.h>

// ROS Services
#include <gripper_control/GripperSelectShape.h>

// ROS Actions
#include <gripper_control/GripperSelectShapeAction.h>

#define ROS_INTERFACE_NAME "ROS_INTERFACE"
#define ROS_INTERFACE_SAMPLETIME    (500 * WF_TIME_ONE_MS)

#define ROS_IF

class ShapeActionInterface;

class RosInterface : virtual protected TcpData, virtual protected Supervisor, private rt_thread
{
public:
    RosInterface(int argc, char** argv, std::string name);
    virtual ~RosInterface();

    void rosSpinOnce();
    bool rosOk();
    void rosPublish();

private:    
    void rt_thread_handler(void);

    // declaration of topics to publish
    ros::Publisher ros_publisher_pos;
    ros::Publisher ros_publisher_vel;
    ros::Publisher ros_publisher_status;

    ros::ServiceServer ros_service_shape;

    //declaration of service server callback
    bool selectShape(gripper_control::GripperSelectShape::Request  &req, gripper_control::GripperSelectShape::Response &res);

    // service servers

protected:
    std::string nodeName;

    ShapeActionInterface* actionInterface;
};

class ShapeActionInterface
{
public:
    ShapeActionInterface(ros::NodeHandle &nh, std::string name, SystemStatus* status, SystemRequest *request) : as_(nh, name, boost::bind(&ShapeActionInterface::executeCB, this, _1), false), actionName(name), nh_(nh), Status(status), Request(request)
    {
        as_.start();
    }

    ~ShapeActionInterface(void)
    {

    }

    //declaration of action server callback
    void executeCB(const gripper_control::GripperSelectShapeGoalConstPtr &goal);

protected:

    SystemStatus* Status;
    SystemRequest* Request;

    ros::NodeHandle nh_;

    std::string actionName;

    actionlib::SimpleActionServer<gripper_control::GripperSelectShapeAction> as_;

    // create messages that are used to published feedback/result
    gripper_control::GripperSelectShapeFeedback feedback_;
    gripper_control::GripperSelectShapeResult result_;
};

#endif /* ROSINTERFACE_H_ */
