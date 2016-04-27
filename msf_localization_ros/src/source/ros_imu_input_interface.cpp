
#include "msf_localization_ros/ros_imu_input_interface.h"


RosImuInputInterface::RosImuInputInterface(ros::NodeHandle* nh, std::weak_ptr<MsfStorageCore> the_msf_storage_core) :
    RosInterface(),
    ImuInputCore(the_msf_storage_core)
{
    // Node Handle
    this->nh=nh;

    return;
}

int RosImuInputInterface::setInputRos(const sensor_msgs::ImuConstPtr& msg)
{

    return 0;
}

void RosImuInputInterface::imuTopicCallback(const sensor_msgs::ImuConstPtr& msg)
{
    this->setInputRos(msg);

    return;
}

int RosImuInputInterface::setImuTopicName(std::string imu_topic_name)
{
    this->imu_topic_name_=imu_topic_name;
    return 0;
}

int RosImuInputInterface::open()
{
    // Subscriber
    imu_topic_subs_=nh->subscribe(imu_topic_name_, 10, &RosImuInputInterface::imuTopicCallback, this);

    return 0;
}

