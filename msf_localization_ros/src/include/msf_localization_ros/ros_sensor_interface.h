
#ifndef _ROS_SENSOR_INTERFACE_H
#define _ROS_SENSOR_INTERFACE_H


#include "msf_localization_ros/ros_interface.h"


class RosSensorInterface : public RosInterface
{
protected:
    RosSensorInterface(ros::NodeHandle* nh);
public:
    virtual ~RosSensorInterface();

public:
    virtual int publish()=0;

};



#endif
