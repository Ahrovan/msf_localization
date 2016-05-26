
#include "msf_localization_ros/ros_absolute_pose_input_interface.h"


RosAbsolutePoseInputInterface::RosAbsolutePoseInputInterface(ros::NodeHandle* nh, tf::TransformBroadcaster *tf_transform_broadcaster, const std::weak_ptr<MsfStorageCore> the_msf_storage_core) :
    RosInputInterface(nh, tf_transform_broadcaster),
    AbsolutePoseInputCore(the_msf_storage_core)
{
    this->input_command_message_type_=AbsolutePoseInputCommandMessageTypes::undefined;

    return;
}

int RosAbsolutePoseInputInterface::setInputCommandRos(const geometry_msgs::PoseStampedPtr& msg)
{
    if(!isInputEnabled())
        return 0;

    // PROVISIONAL! -> Dischart part of the input commands
    if(msg->header.seq  % 3 != 0)
        return 0;


    // Time Stamp
    TimeStamp time_stamp(msg->header.stamp.sec, msg->header.stamp.nsec);


    // Input Command
    std::shared_ptr<AbsolutePoseInputCommandCore> input_command=std::make_shared<AbsolutePoseInputCommandCore>(std::dynamic_pointer_cast<InputCore>(this->getMsfElementCoreSharedPtr()));


    // Covariance
    // No covariance subscribed


    // Position if enabled
    if(this->isInputCommandPositionInputWrtInputWorldEnabled())
    {
        Eigen::Vector3d position_input_wrt_input_world(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        input_command->setPositionInputWrtInputWorld(position_input_wrt_input_world);
    }


    // Attitude if enabled
    if(this->isInputCommandAttitudeInputWrtInputWorldEnabled())
    {
        Eigen::Vector4d attitude_input_wrt_input_world(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
        input_command->setAttitudeInputWrtInputWorld(attitude_input_wrt_input_world);
    }


    // Set
    this->setInputCommand(time_stamp, input_command);

    return 0;
}

int RosAbsolutePoseInputInterface::setInputCommandRos(const geometry_msgs::PoseWithCovarianceStampedPtr& msg)
{
    if(!isInputEnabled())
        return 0;

    // PROVISIONAL! -> Dischart part of the input commands
    if(msg->header.seq  % 3 != 0)
        return 0;


    // Time Stamp
    TimeStamp time_stamp(msg->header.stamp.sec, msg->header.stamp.nsec);


    // Input Command
    std::shared_ptr<AbsolutePoseInputCommandCore> input_command=std::make_shared<AbsolutePoseInputCommandCore>(std::dynamic_pointer_cast<InputCore>(this->getMsfElementCoreSharedPtr()));


    // Covariance
    if(this->hasInputCommandPoseInputWrtInputWorldCovariance())
    {
        Eigen::MatrixXd noise_input_command_pose_input_wrt_input_world=
                Eigen::Map<Eigen::MatrixXd>(msg->pose.covariance.c_array(), 6, 6);
        input_command->setNoiseInputCommandPoseInputWrtInputWorld(noise_input_command_pose_input_wrt_input_world);
    }


    // Position if enabled
    if(this->isInputCommandPositionInputWrtInputWorldEnabled())
    {
        Eigen::Vector3d position_input_wrt_input_world(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        input_command->setPositionInputWrtInputWorld(position_input_wrt_input_world);
    }

    // Attitude if enabled
    if(this->isInputCommandAttitudeInputWrtInputWorldEnabled())
    {
        Eigen::Vector4d attitude_input_wrt_input_world(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        input_command->setAttitudeInputWrtInputWorld(attitude_input_wrt_input_world);
    }


    // Set
    this->setInputCommand(time_stamp, input_command);

    return 0;
}

void RosAbsolutePoseInputInterface::setInputCommandPoseInputWrtInputWorldTopicName(const std::string input_command_pose_input_wrt_input_world_topic_name)
{
    this->input_command_pose_input_wrt_input_world_topic_name_=input_command_pose_input_wrt_input_world_topic_name;
    return;
}

void RosAbsolutePoseInputInterface::setInputCommandMessageType(AbsolutePoseInputCommandMessageTypes input_command_message_type)
{
    this->input_command_message_type_=input_command_message_type;
    return;
}

// Callback for geometry_msgs::PoseStamped
void RosAbsolutePoseInputInterface::inputCommandPoseInputWrtInputWorldCallbackPoseStamped(const geometry_msgs::PoseStampedPtr& msg)
{
    this->setInputCommandRos(msg);
}

// Callback for geometry_msgs::PoseWithCovarianceStamped
void RosAbsolutePoseInputInterface::inputCommandPoseInputWrtInputWorldCallbackPoseWithCovarianceStamped(const geometry_msgs::PoseWithCovarianceStampedPtr& msg)
{
    this->setInputCommandRos(msg);
}

int RosAbsolutePoseInputInterface::open()
{
    // Input Command
    switch(input_command_message_type_)
    {
        case AbsolutePoseInputCommandMessageTypes::geometry_msgs_PoseStamped:
        {
            input_command_pose_input_wrt_input_world_sub_=nh->subscribe(input_command_pose_input_wrt_input_world_topic_name_, 10, &RosAbsolutePoseInputInterface::inputCommandPoseInputWrtInputWorldCallbackPoseStamped, this);
            break;
        }
        case AbsolutePoseInputCommandMessageTypes::geometry_msgs_PoseWithCovarianceStamped:
        {
            input_command_pose_input_wrt_input_world_sub_=nh->subscribe(input_command_pose_input_wrt_input_world_topic_name_, 10, &RosAbsolutePoseInputInterface::inputCommandPoseInputWrtInputWorldCallbackPoseWithCovarianceStamped, this);
            break;
        }
        default:
        {
            return -1;
            break;
        }
    }


    return 0;
}

int RosAbsolutePoseInputInterface::publish()
{
    // TODO

    return 0;
}

int RosAbsolutePoseInputInterface::readConfig(const pugi::xml_node& input, std::shared_ptr<AbsolutePoseInputStateCore>& init_state_core)
{
    /// Input Configs
    int error_read_config=this->AbsolutePoseInputCore::readConfig(input, init_state_core);

    if(error_read_config)
        return error_read_config;


    /// Ros Configs
    // Input Command Topic
    std::string input_command_topic=input.child_value("ros_topic");
    this->setInputCommandPoseInputWrtInputWorldTopicName(input_command_topic);

    // Message Type Input Command Topic
    std::string type_input_command_topic=input.child_value("ros_topic_type");
    if(type_input_command_topic=="geometry_msgs::PoseStamped")
        this->setInputCommandMessageType(AbsolutePoseInputCommandMessageTypes::geometry_msgs_PoseStamped);
    else if(type_input_command_topic=="geometry_msgs::PoseWithCovarianceStamped")
        this->setInputCommandMessageType(AbsolutePoseInputCommandMessageTypes::geometry_msgs_PoseWithCovarianceStamped);


    /// Finish

    // Open
    this->open();

    // End
    return 0;
}
