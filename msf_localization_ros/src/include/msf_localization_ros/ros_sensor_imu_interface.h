


#ifndef _ROS_SENSOR_IMU_INTEFACE_H
#define _ROS_SENSOR_IMU_INTEFACE_H





// ROS Msg sensor_msgs::Imu
#include <sensor_msgs/Imu.h>


// Time Stamp
#include "msf_localization_core/time_stamp.h"

// Imu Sensor Core
#include "msf_localization_core/imu_sensor_core.h"

// Sensor measurement core
#include "msf_localization_core/imu_sensor_measurement_core.h"


// ROS Sensor Interface
#include "msf_localization_ros/ros_sensor_interface.h"


#include "pugixml/pugixml.hpp"



class RosSensorImuInterface : public RosSensorInterface, public ImuSensorCore
{
public:
    RosSensorImuInterface(ros::NodeHandle* nh, std::weak_ptr<MsfStorageCore> the_msf_storage_core);



public:
    int setMeasurementRos(const sensor_msgs::ImuConstPtr& msg);


    // Subscriber
protected:
    std::string ImuTopicName;
protected:
    ros::Subscriber ImuTopicSub;
    void imuTopicCallback(const sensor_msgs::ImuConstPtr& msg);
public:
    int setImuTopicName(std::string SensorImuTopicName);


public:
    int open();


public:
    int publish();

public:
    int readConfig(pugi::xml_node sensor, unsigned int sensorId, std::shared_ptr<ImuSensorStateCore>& SensorInitStateCore);


};




#endif
