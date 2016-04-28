
#include "msf_localization_ros/ros_sensor_interface.h"


RosSensorInterface::RosSensorInterface(ros::NodeHandle* nh) :
    RosInterface()
{
    // Node Handle
    this->nh=nh;

    return;
}

RosSensorInterface::~RosSensorInterface()
{
    return;
}
