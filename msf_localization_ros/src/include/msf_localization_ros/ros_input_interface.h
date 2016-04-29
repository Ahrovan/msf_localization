
#ifndef _ROS_INPUT_INTERFACE_H
#define _ROS_INPUT_INTERFACE_H


#include "msf_localization_ros/ros_interface.h"


class RosInputInterface : public RosInterface
{
protected:
    RosInputInterface(ros::NodeHandle* nh, tf::TransformBroadcaster *tf_transform_broadcaster);
public:
    virtual ~RosInputInterface();

public:
    virtual int publish()=0;

};


#endif
