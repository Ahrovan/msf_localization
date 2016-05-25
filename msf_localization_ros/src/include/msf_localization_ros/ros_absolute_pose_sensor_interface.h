
#ifndef _ROS_ABSOLUTE_POSE_SENSOR_INTERFACE_H
#define _ROS_ABSOLUTE_POSE_SENSOR_INTERFACE_H



// ROS Msg
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


// Time Stamp
#include "msf_localization_core/time_stamp.h"

// Sensor Core
#include "msf_localization_core/absolute_pose_sensor_core.h"

// Sensor measurement core
#include "msf_localization_core/absolute_pose_sensor_measurement_core.h"


// ROS Sensor Interface
#include "msf_localization_ros/ros_sensor_interface.h"


#include "pugixml/pugixml.hpp"


enum class AbsolutePoseSensorMeasurementMessageTypes
{
    undefined=0,
    geometry_msgs_PoseStamped=1,
    geometry_msgs_PoseWithCovarianceStamped
};


class RosAbsolutePoseSensorInterface : public RosSensorInterface, public AbsolutePoseSensorCore
{
public:
    RosAbsolutePoseSensorInterface(ros::NodeHandle* nh, tf::TransformBroadcaster *tf_transform_broadcaster, const std::weak_ptr<MsfStorageCore> the_msf_storage_core);



public:
    int setMeasurementRos(const geometry_msgs::PoseStampedPtr& msg);
    int setMeasurementRos(const geometry_msgs::PoseWithCovarianceStampedPtr& msg);


    // Message Type
protected:
    AbsolutePoseSensorMeasurementMessageTypes sensor_measurement_message_type_;
protected:
    void setSensorMeasurementMessageType(AbsolutePoseSensorMeasurementMessageTypes sensor_measurement_message_type);

    // Subscriber
protected:
    std::string measurement_mocap_sensor_wrt_mocap_world_topic_name_;
public:
    int setMeasurementMocapSensorWrtMocapWorldTopicName(const std::string measurement_mocap_sensor_wrt_mocap_world_topic_name);
protected:
    ros::Subscriber measurement_mocap_sensor_wrt_mocap_world_list_sub_;
protected:
    // Callback for geometry_msgs::PoseStamped
    void measurementMocapSensorWrtMocapWorldCallbackPoseStamped(const geometry_msgs::PoseStampedPtr& msg);
    // Callback for geometry_msgs::PoseWithCovarianceStamped
    void measurementMocapSensorWrtMocapWorldCallbackPoseWithCovarianceStamped(const geometry_msgs::PoseWithCovarianceStampedPtr& msg);


public:
    int open();


public:
    int publish(const TimeStamp& time_stamp,
                const std::shared_ptr<RosRobotInterface>& robot_core,
                const std::shared_ptr<SensorStateCore>& sensor_state_core);


public:
    int readConfig(const pugi::xml_node& sensor, unsigned int sensorId, std::shared_ptr<AbsolutePoseSensorStateCore>& SensorInitStateCore);

};



#endif
