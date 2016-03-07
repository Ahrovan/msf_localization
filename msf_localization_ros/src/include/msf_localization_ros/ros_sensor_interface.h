

#ifndef _ROS_SENSOR_INTEFACE_H
#define _ROS_SENSOR_INTEFACE_H


// string
#include <string>


// ROS
#include <ros/ros.h>


// std_msgs
#include <std_msgs/Header.h>





class RosSensorInterface
{
    //
protected:
    RosSensorInterface();
public:
    virtual ~RosSensorInterface();


protected:
    ros::NodeHandle* nh;


    // Sensor Core
protected:
    //SensorCore* TheSensorCore;
public:
    //SensorCore* getTheSensorCore() const;



    // Sensor Type
protected:


    // Open
public:
    virtual int open()=0;

    // Close
public:
    virtual int close();



};







#endif