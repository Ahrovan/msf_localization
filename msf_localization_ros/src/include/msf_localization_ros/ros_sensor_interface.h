
#ifndef _ROS_SENSOR_INTERFACE_H
#define _ROS_SENSOR_INTERFACE_H


#include "msf_localization_ros/ros_interface.h"
#include "msf_localization_ros/ros_robot_interface.h"

#include "msf_localization_core/time_stamp.h"
#include "msf_localization_core/sensor_core.h"
#include "msf_localization_core/sensor_state_core.h"

class RosSensorInterface : public RosInterface
{
protected:
    RosSensorInterface(ros::NodeHandle* nh, tf::TransformBroadcaster* tf_transform_broadcaster);
public:
    virtual ~RosSensorInterface();



public:
    virtual int publish(const TimeStamp& time_stamp, const std::shared_ptr<RosRobotInterface> robot_core, const std::shared_ptr<SensorStateCore> sensor_state_core)=0;

protected:
    int publishTfPoseSensorWrtRobot(const TimeStamp& time_stamp, const std::shared_ptr<RosRobotInterface> robot_core, const std::shared_ptr<SensorStateCore> sensor_state_core);

};



#endif
