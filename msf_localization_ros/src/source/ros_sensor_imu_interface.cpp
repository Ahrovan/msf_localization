
#include "msf_localization_ros/ros_sensor_imu_interface.h"





RosSensorImuInterface::RosSensorImuInterface(ros::NodeHandle* nh, tf::TransformBroadcaster *tf_transform_broadcaster, std::weak_ptr<MsfStorageCore> the_msf_storage_core) :
    RosSensorInterface(nh, tf_transform_broadcaster),
    ImuSensorCore(the_msf_storage_core)
{

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

    // PROVISIONAL! -> Dischart part of the measurements
    if(msg->header.seq  % 4 != 0)
        return 0;

    //std::cout<<"ROS Imu Measured"<<std::endl;

    //ImuSensorCore* TheSensorCore=dynamic_cast<ImuSensorCore*>(this->TheSensorCore);
    //TheSensorCore->setMeasurement();


    // Sensor Id
    //std::cout<<"Sensor Id="<<this->sensorId<<std::endl;


    // Time Stamp
    TimeStamp TheTimeStamp(msg->header.stamp.sec, msg->header.stamp.nsec);


    // Sensor Core
    std::shared_ptr<ImuSensorCore> TheImuSensorCore=std::dynamic_pointer_cast<ImuSensorCore>(this->getMsfElementCoreSharedPtr());

    // Value
    std::shared_ptr<ImuSensorMeasurementCore> TheImuSensorMeasurementCore=std::make_shared<ImuSensorMeasurementCore>(TheImuSensorCore);


    // Orientation if enabled
    if(this->isMeasurementOrientationEnabled())
    {
        Eigen::Vector4d orientation(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

        if(TheImuSensorMeasurementCore->setOrientation(orientation))
            std::cout<<"Error setting orientation"<<std::endl;
    }

    //msg->orientation_covariance;
    // TODO


    // Angular velocity if enabled
    if(this->isMeasurementAngularVelocityEnabled())
    {
        Eigen::Vector3d angular_velocity(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
        if(TheImuSensorMeasurementCore->setAngularVelocity(angular_velocity))
            std::cout<<"Error setting angular_velocity"<<std::endl;
    }

    //msg->angular_velocity_covariance;
    // TODO


    // Linear acceleration if enabled
    if(this->isMeasurementLinearAccelerationEnabled())
    {
        Eigen::Vector3d linear_acceleration(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);

        if(TheImuSensorMeasurementCore->setLinearAcceleration(linear_acceleration))
            std::cout<<"Error setting linear_acceleration"<<std::endl;
    }

    //msg->linear_acceleration_covariance;
    // TODO

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
    // Subscriber
    ImuTopicSub=nh->subscribe(ImuTopicName, 10, &RosSensorImuInterface::imuTopicCallback, this);


    return 0;
}

int RosSensorImuInterface::publish(TimeStamp time_stamp, std::shared_ptr<RosRobotInterface> robot_core, std::shared_ptr<SensorStateCore> sensor_state_core)
{
    // tf pose sensor wrt robot
    this->publishTfPoseSensorWrtRobot(time_stamp, robot_core, sensor_state_core);

    // end
    return 0;
}

int RosSensorImuInterface::readConfig(pugi::xml_node sensor, unsigned int sensorId, std::shared_ptr<ImuSensorStateCore>& SensorInitStateCore)
{
    /// Imu Sensor Configs
    int errorReadConfig=this->ImuSensorCore::readConfig(sensor, sensorId, SensorInitStateCore);

    if(errorReadConfig)
        return errorReadConfig;


    /// Ros Configs
    // Sensor Topic
    std::string sensor_topic=sensor.child_value("ros_topic");
    this->setImuTopicName(sensor_topic);

    // Name
    std::string sensor_name=sensor.child_value("name");
    this->setSensorName(sensor_name);


    /// Finish

    // Open
    this->open();

    // End
    return 0;
}
