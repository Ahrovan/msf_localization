

#ifndef _ROS_INTEFACE_H
#define _ROS_INTEFACE_H


// string
#include <string>


// ROS
#include <ros/ros.h>


// tf
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>


// std_msgs
#include <std_msgs/Header.h>





class RosInterface
{
    //
protected:
    RosInterface();
public:
    virtual ~RosInterface();


protected:
    ros::NodeHandle* nh;


    // Tf
protected:
    tf::TransformBroadcaster* tf_transform_broadcaster_;


    // Open
public:
    virtual int open()=0;

    // Close
public:
    virtual int close();



};







#endif
