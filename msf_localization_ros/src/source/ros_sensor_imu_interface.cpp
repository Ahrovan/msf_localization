
#include "ros_sensor_imu_interface.h"





RosSensorImuInterface::RosSensorImuInterface()
{
    // Create the variable in the MSF Localization Core
    TheSensorCore = new ImuSensorCore;

    //ImuSensorCore TheImuSensorCore;
    //TheMsfLocalizationCore->TheListOfSensorCore.push_back(TheImuSensorCore);

    return;
}



int RosSensorImuInterface::setImuTopicName(std::string ImuTopicName)
{
    this->ImuTopicName=ImuTopicName;
    return 0;
}


void RosSensorImuInterface::imuTopicCallback(const sensor_msgs::ImuConstPtr& msg)
{
    //std::cout<<"Imu Measured"<<std::endl;

    ImuSensorCore* TheSensorCore=dynamic_cast<ImuSensorCore*>(this->TheSensorCore);
    TheSensorCore->setMeasurement();



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
