
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
    if(!isSensorEnabled())
        return 0;


    //std::cout<<"ROS Imu Measured"<<std::endl;

    //ImuSensorCore* TheSensorCore=dynamic_cast<ImuSensorCore*>(this->TheSensorCore);
    //TheSensorCore->setMeasurement();


    // Sensor Id
    //std::cout<<"Sensor Id="<<this->sensorId<<std::endl;


    // Time Stamp
    TimeStamp TheTimeStamp(msg->header.stamp.sec, msg->header.stamp.nsec);

    // Value
    std::shared_ptr<ImuSensorMeasurementCore> TheImuSensorMeasurementCore=std::make_shared<ImuSensorMeasurementCore>();

    // Set the sensor core -> Associate it to the SensorCore
    //TheImuSensorMeasurementCore->setTheSensorCore(std::dynamic_pointer_cast<ImuSensorCore>(this));
    //std::weak_ptr<const ImuSensorCore> TheImuSensorCorePtrAux=std::static_pointer_cast<const ImuSensorCore>((this));
    //TheImuSensorMeasurementCore->setTheSensorCore(TheImuSensorCorePtrAux);
    TheImuSensorMeasurementCore->setTheSensorCore(this->TheSensorCorePtr);


    // Orientation if enabled
    if(this->isOrientationEnabled())
    {
        Eigen::Vector4d orientation(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

        if(TheImuSensorMeasurementCore->setOrientation(orientation))
            std::cout<<"Error setting orientation"<<std::endl;
    }

    msg->orientation_covariance;


    // Angular velocity if enabled
    if(this->isAngularVelocityEnabled())
    {
        Eigen::Vector3d angular_velocity(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

        if(TheImuSensorMeasurementCore->setAngularVelocity(angular_velocity))
            std::cout<<"Error setting angular_velocity"<<std::endl;
    }

    msg->angular_velocity_covariance;


    // Linear acceleration if enabled
    if(this->isLinearAccelerationEnabled())
    {
        Eigen::Vector3d linear_acceleration(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);

        if(TheImuSensorMeasurementCore->setLinearAcceleration(linear_acceleration))
            std::cout<<"Error setting linear_acceleration"<<std::endl;
    }
    msg->linear_acceleration_covariance;


    // Set
    this->setMeasurement(TheTimeStamp, TheImuSensorMeasurementCore);

    return 0;
}


void RosSensorImuInterface::imuTopicCallback(const sensor_msgs::ImuConstPtr& msg)
{
    //logFile<<"RosSensorImuInterface::imuTopicCallback()"<<std::endl;

    this->setMeasurementRos(msg);


    //logFile<<"RosSensorImuInterface::imuTopicCallback() ended"<<std::endl;
    return;
}


int RosSensorImuInterface::open()
{
     std::cout<<"RosSensorImuInterface::open()"<<std::endl;

    // Node handler
    ros::NodeHandle nh;

    // Subscriber
    ImuTopicSub=nh.subscribe(ImuTopicName, 10, &RosSensorImuInterface::imuTopicCallback, this);


    return 0;
}
