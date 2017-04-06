
#include "msf_localization_ros/ros_imu_sensor_interface.h"





RosImuSensorInterface::RosImuSensorInterface(ros::NodeHandle* nh, tf::TransformBroadcaster *tf_transform_broadcaster, MsfLocalizationCore* msf_localization_core_ptr) :
    RosSensorInterface(nh, tf_transform_broadcaster),
    ImuSensorCore(msf_localization_core_ptr)
{

    // Frequencies
    frequency_desired_=30.0; // Hz
    previous_time_stamp_=ros::Time(0,0);

    return;
}



int RosImuSensorInterface::setImuTopicName(const std::string ImuTopicName)
{
    this->ImuTopicName=ImuTopicName;
    return 0;
}


int RosImuSensorInterface::setMeasurementRos(const sensor_msgs::ImuConstPtr& msg)
{
    if(!isSensorEnabled())
        return 0;


    // Frequency
    if(frequency_desired_>0)
    {
        // Set time stamps
        ros::Time current_time_stamp=msg->header.stamp;

        ros::Duration diference_time_stamps=current_time_stamp-previous_time_stamp_;


        if(diference_time_stamps.toSec() < 1/frequency_desired_)
            return 0;

        // Update for the next iteration
        previous_time_stamp_=current_time_stamp;
    }


    // PROVISIONAL! -> Dischart part of the measurements
    // Remainder operator: header.seq % num == 0 (se descartan 1 de cada num mensajes)
//    if(msg->header.seq  % 2 == 0)
//        return 0;

    //std::cout<<"ROS Imu Measured"<<std::endl;

    //ImuSensorCore* TheSensorCore=dynamic_cast<ImuSensorCore*>(this->TheSensorCore);
    //TheSensorCore->setMeasurement();


    // Sensor Id
    //std::cout<<"Sensor Id="<<this->sensorId<<std::endl;


    // Time Stamp
    TimeStamp TheTimeStamp(msg->header.stamp.sec, msg->header.stamp.nsec);


    // Sensor Core
    //std::shared_ptr<ImuSensorCore> TheImuSensorCore=std::dynamic_pointer_cast<ImuSensorCore>(this->getMsfElementCoreSharedPtr());

    // Value
    std::shared_ptr<ImuSensorMeasurementCore> TheImuSensorMeasurementCore=std::make_shared<ImuSensorMeasurementCore>(this->getSensorCoreWeakPtr());


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


void RosImuSensorInterface::imuTopicCallback(const sensor_msgs::ImuConstPtr& msg)
{
    //logFile<<"RosImuSensorInterface::imuTopicCallback()"<<std::endl;

    this->setMeasurementRos(msg);


    //logFile<<"RosImuSensorInterface::imuTopicCallback() ended"<<std::endl;
    return;
}


int RosImuSensorInterface::setEstimatedBiasLinearAccelerationTopicName(const std::string& estimated_bias_linear_acceleration_topic_name)
{
    this->estimated_bias_linear_acceleration_topic_name_=estimated_bias_linear_acceleration_topic_name;
    //std::cout<<"estimated_bias_linear_acceleration_topic_name_="<<estimated_bias_linear_acceleration_topic_name_<<std::endl;
    return 0;
}

int RosImuSensorInterface::publishEstimatedBiasLinearAcceleration(const TimeStamp& time_stamp, const std::shared_ptr<ImuSensorStateCore> &sensor_state_core)
{
    if(estimated_bias_linear_acceleration_pub_.getNumSubscribers() > 0)
    {
        if(this->isEstimationBiasLinearAccelerationEnabled())
        {
            // Fill message
            // Header
            estimated_bias_linear_acceleration_msg_.header.frame_id="NA";
            estimated_bias_linear_acceleration_msg_.header.stamp=ros::Time(time_stamp.sec, time_stamp.nsec);
            // Value
            Eigen::Vector3d estimated_bias_linear_acceleration=sensor_state_core->getBiasesLinearAcceleration();
            estimated_bias_linear_acceleration_msg_.vector.x=estimated_bias_linear_acceleration(0);
            estimated_bias_linear_acceleration_msg_.vector.y=estimated_bias_linear_acceleration(1);
            estimated_bias_linear_acceleration_msg_.vector.z=estimated_bias_linear_acceleration(2);

            // Publish
            estimated_bias_linear_acceleration_pub_.publish(this->estimated_bias_linear_acceleration_msg_);
        }
    }
    return 0;
}

int RosImuSensorInterface::open()
{
    // Subscriber
    ImuTopicSub=nh->subscribe(ImuTopicName, 10, &RosImuSensorInterface::imuTopicCallback, this);

    // Publishers
    estimated_bias_linear_acceleration_pub_=nh->advertise<geometry_msgs::Vector3Stamped>(estimated_bias_linear_acceleration_topic_name_, 1, true);


    return 0;
}

int RosImuSensorInterface::publish(const TimeStamp& time_stamp, const std::shared_ptr<RosRobotInterface> &robot_core, const std::shared_ptr<SensorStateCore> &sensor_state_core)
{
    // tf pose sensor wrt robot
    this->publishTfPoseSensorWrtRobot(time_stamp, robot_core, sensor_state_core);

    // Estimated biases
    this->publishEstimatedBiasLinearAcceleration(time_stamp, std::dynamic_pointer_cast<ImuSensorStateCore>(sensor_state_core));

    // end
    return 0;
}

int RosImuSensorInterface::readConfig(const pugi::xml_node& sensor, unsigned int sensorId, std::shared_ptr<ImuSensorStateCore>& SensorInitStateCore)
{
    /// Imu Sensor Configs
    int errorReadConfig=this->ImuSensorCore::readConfig(sensor, sensorId, SensorInitStateCore);

    if(errorReadConfig)
        return errorReadConfig;


    /// Ros Configs
    // Sensor Topic
    std::string sensor_topic=sensor.child_value("ros_topic");
    this->setImuTopicName(sensor_topic);


    // Estimated states topic name
    std::string estimated_bias_linear_acceleration_topic_name=sensor.child("parameters").child("linear_acceleration").child("biases").child_value("ros_topic");
    this->setEstimatedBiasLinearAccelerationTopicName(estimated_bias_linear_acceleration_topic_name);

    /// Finish

    // Open
    this->open();

    // End
    return 0;
}
