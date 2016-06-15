


#ifndef _ROS_IMU_SENSOR_INTEFACE_H
#define _ROS_IMU_SENSOR_INTEFACE_H





// ROS Msg sensor_msgs::Imu
#include <sensor_msgs/Imu.h>

// ROS Msg for estimated biases
#include <geometry_msgs/Vector3Stamped.h>


// Time Stamp
#include "msf_localization_core/time_stamp.h"

// Imu Sensor Core
#include "msf_localization_core/imu_sensor_core.h"

// Sensor measurement core
#include "msf_localization_core/imu_sensor_measurement_core.h"


// ROS Sensor Interface
#include "msf_localization_ros/ros_sensor_interface.h"


#include "pugixml/pugixml.hpp"



class RosImuSensorInterface : public RosSensorInterface, public ImuSensorCore
{
public:
    RosImuSensorInterface(ros::NodeHandle* nh, tf::TransformBroadcaster *tf_transform_broadcaster, MsfLocalizationCore* msf_localization_core_ptr);



public:
    int setMeasurementRos(const sensor_msgs::ImuConstPtr& msg);


    // Frequencies
protected:
    ros::Time previous_time_stamp_;
    double frequency_desired_;


    // Subscriber
protected:
    std::string ImuTopicName;
protected:
    ros::Subscriber ImuTopicSub;
    void imuTopicCallback(const sensor_msgs::ImuConstPtr& msg);
public:
    int setImuTopicName(const std::string SensorImuTopicName);


    // Publishers for biases
protected:
    std::string estimated_bias_linear_acceleration_topic_name_;
public:
    int setEstimatedBiasLinearAccelerationTopicName(const std::string& estimated_bias_linear_acceleration_topic_name);
protected:
    ros::Publisher estimated_bias_linear_acceleration_pub_;
    geometry_msgs::Vector3Stamped estimated_bias_linear_acceleration_msg_;
protected:
    int publishEstimatedBiasLinearAcceleration(const TimeStamp& time_stamp,
                                               const std::shared_ptr<ImuSensorStateCore>& sensor_state_core);



public:
    int open();


public:
    int publish(const TimeStamp& time_stamp,
                const std::shared_ptr<RosRobotInterface>& robot_core,
                const std::shared_ptr<SensorStateCore>& sensor_state_core);

public:
    int readConfig(const pugi::xml_node& sensor, unsigned int sensorId, std::shared_ptr<ImuSensorStateCore>& SensorInitStateCore);


};




#endif
