
#include "msf_localization_ros/ros_robot_interface.h"

RosRobotInterface::RosRobotInterface(ros::NodeHandle* nh, tf::TransformBroadcaster *tf_transform_broadcaster) :
    RosInterface()
{
    // Node Handle
    this->nh=nh;

    // Tf broadcaster
    this->tf_transform_broadcaster_=tf_transform_broadcaster;

    return;
}

RosRobotInterface::~RosRobotInterface()
{
    return;
}

int RosRobotInterface::setRobotName(const std::string robot_name)
{
    this->robot_name_=robot_name;
    return 0;
}

std::string RosRobotInterface::getRobotName() const
{
    return this->robot_name_;
}
