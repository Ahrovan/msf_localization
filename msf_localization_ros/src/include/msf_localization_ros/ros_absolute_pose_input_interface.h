
#ifndef _ROS_ABSOLUTE_POSE_INPUT_INTERFACE_H
#define _ROS_ABSOLUTE_POSE_INPUT_INTERFACE_H




// ROS Msg
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// ROS Srvs
#include "msf_localization_ros_srvs/GetPoseWithCovarianceByStamp.h"


// Time Stamp
#include "time_stamp/time_stamp.h"

// Input Core
#include "msf_localization_core/absolute_pose_input_core.h"

// Input Command core
#include "msf_localization_core/absolute_pose_input_command_core.h"


// ROS Input Interface
#include "msf_localization_ros/ros_input_interface.h"


#include "pugixml/pugixml.hpp"



enum class AbsolutePoseInputCommandMessageTypes
{
    undefined=0,
    geometry_msgs_PoseStamped=1,
    geometry_msgs_PoseWithCovarianceStamped
};



class RosAbsolutePoseInputInterface : public RosInputInterface, public AbsolutePoseInputCore
{
public:
    RosAbsolutePoseInputInterface(ros::NodeHandle* nh, tf::TransformBroadcaster *tf_transform_broadcaster, MsfLocalizationCore* msf_localization_core_ptr);



public:
    int setInputCommandRos(const geometry_msgs::PoseStampedPtr& msg);
    int setInputCommandRos(const geometry_msgs::PoseWithCovarianceStampedPtr& msg);


    // Message Type
protected:
    AbsolutePoseInputCommandMessageTypes input_command_message_type_;
protected:
    void setInputCommandMessageType(AbsolutePoseInputCommandMessageTypes input_command_message_type);

    // Topic Subscriber / Service Client
protected:
    std::string input_command_pose_input_wrt_input_world_topic_name_;
public:
    void setInputCommandPoseInputWrtInputWorldTopicName(const std::string input_command_pose_input_wrt_input_world_topic_name);


    // Subscriber
protected:
    ros::Subscriber input_command_pose_input_wrt_input_world_sub_;
protected:
    // Callback for geometry_msgs::PoseStamped
    void inputCommandPoseInputWrtInputWorldCallbackPoseStamped(const geometry_msgs::PoseStampedPtr& msg);
    // Callback for geometry_msgs::PoseWithCovarianceStamped
    void inputCommandPoseInputWrtInputWorldCallbackPoseWithCovarianceStamped(const geometry_msgs::PoseWithCovarianceStampedPtr& msg);


    // Service client
protected:
    ros::ServiceClient input_command_pose_input_wrt_input_world_srv_cli_;
protected:
    msf_localization_ros_srvs::GetPoseWithCovarianceByStamp input_command_pose_with_covariance_input_wrt_input_world_srv_;



    // Active input command
public:
    int getInputCommand(const TimeStamp& requested_time_stamp,
                        TimeStamp& received_time_stamp,
                        std::shared_ptr<InputCommandCore>& received_input_command);



public:
    int open();


public:
    int publish();


public:
    int readConfig(const pugi::xml_node& input, std::shared_ptr<AbsolutePoseInputStateCore>& init_state_core);

};



#endif
