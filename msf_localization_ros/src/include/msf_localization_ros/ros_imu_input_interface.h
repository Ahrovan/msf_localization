
#ifndef _ROS_IMU_INPUT_INTERFACE_H
#define _ROS_IMU_INPUT_INTERFACE_H



// ROS Msg sensor_msgs::Imu
#include <sensor_msgs/Imu.h>


// Time Stamp
#include "msf_localization_core/time_stamp.h"


#include "msf_localization_ros/ros_interface.h"

#include "msf_localization_core/imu_input_core.h"


#include "pugixml/pugixml.hpp"


class RosImuInputInterface : public RosInterface, public ImuInputCore
{
public:
    RosImuInputInterface(ros::NodeHandle* nh, std::weak_ptr<MsfStorageCore> the_msf_storage_core);



public:
    int setInputRos(const sensor_msgs::ImuConstPtr& msg);


    // Subscriber
protected:
    std::string imu_topic_name_;
protected:
    ros::Subscriber imu_topic_subs_;
    void imuTopicCallback(const sensor_msgs::ImuConstPtr& msg);
public:
    int setImuTopicName(std::string imu_topic_name);


public:
    int open();


public:
    //int readConfig(pugi::xml_node sensor, unsigned int sensorId, std::shared_ptr<ImuSensorStateCore>& SensorInitStateCore);



};



#endif