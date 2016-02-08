
#include "ros_sensor_imu_interface.h"





RosSensorImuInterface::RosSensorImuInterface()
{
    // Create the variable in the MSF Localization Core
    //TheSensorCore = new ImuSensorCore;

    //ImuSensorCore TheImuSensorCore;
    //TheMsfLocalizationCore->TheListOfSensorCore.push_back(TheImuSensorCore);

    return;
}



int RosSensorImuInterface::setImuTopicName(std::string ImuTopicName)
{
    this->ImuTopicName=ImuTopicName;
    return 0;
}


int RosSensorImuInterface::setMeasurementRos(const sensor_msgs::ImuConstPtr& msg)
{

    //std::cout<<"Imu Measured"<<std::endl;

    //ImuSensorCore* TheSensorCore=dynamic_cast<ImuSensorCore*>(this->TheSensorCore);
    //TheSensorCore->setMeasurement();

    // Time Stamp
    TimeStamp TheTimeStamp(msg->header.stamp.sec, msg->header.stamp.nsec);

    // Value
    std::shared_ptr<ImuSensorMeasurementCore> TheImuSensorMeasurementCore=std::make_shared<ImuSensorMeasurementCore>();

    // ID
    // TODO

    // Orientation if enabled
    msg->orientation;
    msg->orientation_covariance;

    // Angular velocity if enabled
    msg->angular_velocity;
    msg->angular_velocity_covariance;

    // Linear acceleration if enabled
    msg->linear_acceleration;
    msg->linear_acceleration_covariance;


    // Set
    this->setMeasurement(TheTimeStamp, TheImuSensorMeasurementCore);

    return 0;
}


void RosSensorImuInterface::imuTopicCallback(const sensor_msgs::ImuConstPtr& msg)
{

    this->setMeasurementRos(msg);


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
