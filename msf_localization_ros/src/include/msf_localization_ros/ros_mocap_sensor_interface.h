
#ifndef _ROS_MOCAP_SENSOR_INTERFACE_H
#define _ROS_MOCAP_SENSOR_INTERFACE_H



// ROS Msg
#include <geometry_msgs/PoseStamped.h>


// Time Stamp
#include "msf_localization_core/time_stamp.h"

// Sensor Core
#include "msf_localization_core/absolute_pose_sensor_core.h"

// Sensor measurement core
#include "msf_localization_core/absolute_pose_sensor_measurement_core.h"


// ROS Sensor Interface
#include "msf_localization_ros/ros_sensor_interface.h"


#include "pugixml/pugixml.hpp"



class RosMocapSensorInterface : public RosSensorInterface, public AbsolutePoseSensorCore
{
public:
    RosMocapSensorInterface(ros::NodeHandle* nh, tf::TransformBroadcaster *tf_transform_broadcaster, const std::weak_ptr<MsfStorageCore> the_msf_storage_core);



public:
    int setMeasurementRos(const geometry_msgs::PoseStampedPtr& msg);


    // Subscriber
protected:
    std::string measurement_mocap_sensor_wrt_mocap_world_topic_name_;
protected:
    ros::Subscriber measurement_mocap_sensor_wrt_mocap_world_list_sub_;
    void measurementMocapSensorWrtMocapWorldCallback(const geometry_msgs::PoseStampedPtr& msg);
public:
    int setMeasurementMocapSensorWrtMocapWorldTopicName(const std::string measurement_mocap_sensor_wrt_mocap_world_topic_name);


public:
    int open();


public:
    int publish(const TimeStamp& time_stamp, const std::shared_ptr<RosRobotInterface> robot_core, const std::shared_ptr<SensorStateCore> sensor_state_core);


public:
    int readConfig(const pugi::xml_node& sensor, unsigned int sensorId, std::shared_ptr<AbsolutePoseSensorStateCore>& SensorInitStateCore);

};



#endif
