
#ifndef _ROS_SENSOR_INTERFACE_H
#define _ROS_SENSOR_INTERFACE_H


#include "msf_localization_ros/ros_interface.h"
#include "msf_localization_ros/ros_robot_interface.h"

#include "msf_localization_core/time_stamp.h"
#include "msf_localization_core/sensor_state_core.h"

class RosSensorInterface : public RosInterface
{
protected:
    RosSensorInterface(ros::NodeHandle* nh, tf::TransformBroadcaster* tf_transform_broadcaster);
public:
    virtual ~RosSensorInterface();


    //// Name
protected:
    std::string sensor_name_;
public:
    int setSensorName(std::string sensor_name);
    std::string getSensorName() const;


public:
    virtual int publish(TimeStamp time_stamp, std::shared_ptr<RosRobotInterface> robot_core, std::shared_ptr<SensorStateCore> sensor_state_core)=0;

protected:
    int publishTfPoseSensorWrtRobot(TimeStamp time_stamp, std::shared_ptr<RosRobotInterface> robot_core, std::shared_ptr<SensorStateCore> sensor_state_core);

};



#endif
