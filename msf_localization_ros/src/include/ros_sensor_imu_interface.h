


#ifndef _ROS_SENSOR_IMU_INTEFACE_H
#define _ROS_SENSOR_IMU_INTEFACE_H





// ROS Msg sensor_msgs::Imu
#include <sensor_msgs/Imu.h>



// ROS Sensor Interface
#include "ros_sensor_interface.h"





class RosSensorImuInterface : public RosSensorInterface, public ImuSensorCore
{
public:
    RosSensorImuInterface(ros::NodeHandle* nh);


protected:
    ros::NodeHandle* nh;


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


};




#endif
