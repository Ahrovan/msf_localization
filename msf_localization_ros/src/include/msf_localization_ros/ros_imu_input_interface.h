
#ifndef _ROS_IMU_INPUT_INTERFACE_H
#define _ROS_IMU_INPUT_INTERFACE_H



// ROS Msg sensor_msgs::Imu
#include <sensor_msgs/Imu.h>


// Time Stamp
#include "time_stamp/time_stamp.h"


#include "msf_localization_ros/ros_input_interface.h"

#include "msf_localization_core/imu_input_core.h"


#include "pugixml/pugixml.hpp"


class RosImuInputInterface : public RosInputInterface, public ImuInputCore
{
public:
    RosImuInputInterface(ros::NodeHandle* nh, tf::TransformBroadcaster *tf_transform_broadcaster, MsfLocalizationCore* msf_localization_core_ptr);
    ~RosImuInputInterface();


public:
    int setInputCommandRos(const sensor_msgs::ImuConstPtr& msg);


    // Subscriber
protected:
    std::string imu_topic_name_;
protected:
    ros::Subscriber imu_topic_subs_;
    void imuTopicCallback(const sensor_msgs::ImuConstPtr& msg);
public:
    int setImuTopicName(const std::string imu_topic_name);


public:
    int open();

public:
    int publish();

public:
    int readConfig(const pugi::xml_node& input, std::shared_ptr<ImuInputStateCore>& init_state_core);



};



#endif
