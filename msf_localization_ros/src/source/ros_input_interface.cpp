
#include "msf_localization_ros/ros_input_interface.h"

RosInputInterface::RosInputInterface(ros::NodeHandle* nh, tf::TransformBroadcaster *tf_transform_broadcaster) :
    RosInterface()
{
    // Node Handle
    this->nh=nh;

    // Tf
    this->tf_transform_broadcaster_=tf_transform_broadcaster;

    // End
    return;
}

RosInputInterface::~RosInputInterface()
{
    return;
}
