
#include "msf_localization_ros/ros_input_interface.h"

RosInputInterface::RosInputInterface(ros::NodeHandle* nh) :
    RosInterface()
{
    // Node Handle
    this->nh=nh;

    return;
}

RosInputInterface::~RosInputInterface()
{
    return;
}
