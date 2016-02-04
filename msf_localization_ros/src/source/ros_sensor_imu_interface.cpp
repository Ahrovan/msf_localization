
#include "ros_sensor_imu_interface.h"



int RosSensorImuInterface::setImuTopicName(std::string ImuTopicName)
{
    this->ImuTopicName=ImuTopicName;
    return 0;
}


void RosSensorImuInterface::imuTopicCallback(const sensor_msgs::ImuConstPtr& msg)
{
    //std::cout<<"Imu Measured"<<std::endl;

    return;
}


int RosSensorImuInterface::open()
{
    // Node handler
    ros::NodeHandle nh;

    // Subscriber
    ImuTopicSub=nh.subscribe(ImuTopicName, 10, &RosSensorImuInterface::imuTopicCallback, this);


    return 0;
}
