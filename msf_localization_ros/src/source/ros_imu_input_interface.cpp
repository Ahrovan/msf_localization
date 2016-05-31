
#include "msf_localization_ros/ros_imu_input_interface.h"


RosImuInputInterface::RosImuInputInterface(ros::NodeHandle* nh, tf::TransformBroadcaster *tf_transform_broadcaster, MsfLocalizationCore* msf_localization_core_ptr) :
    RosInputInterface(nh, tf_transform_broadcaster),
    ImuInputCore(msf_localization_core_ptr)
{

    return;
}

RosImuInputInterface::~RosImuInputInterface()
{
    return;
}

int RosImuInputInterface::setInputCommandRos(const sensor_msgs::ImuConstPtr& msg)
{
    if(!isInputEnabled())
        return 0;

    // PROVISIONAL! -> Dischart part of the measurements
    if(msg->header.seq  % 4 != 0)
        return 0;


    // Time Stamp
    TimeStamp time_stamp(msg->header.stamp.sec, msg->header.stamp.nsec);


    // Sensor Core
    std::shared_ptr<ImuInputCore> imu_input_core=std::dynamic_pointer_cast<ImuInputCore>(this->getMsfElementCoreSharedPtr());

    // Value
    std::shared_ptr<ImuInputCommandCore> imu_input_command=std::make_shared<ImuInputCommandCore>(imu_input_core);



    // Orientation if enabled
    if(this->isInputCommandOrientationEnabled())
    {
        Eigen::Vector4d orientation(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

        if(imu_input_command->setOrientation(orientation))
            std::cout<<"Error setting orientation"<<std::endl;
    }

    //msg->orientation_covariance;
    // TODO


    // Angular velocity if enabled
    if(this->isInputCommandAngularVelocityEnabled())
    {
        Eigen::Vector3d angular_velocity(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
        if(imu_input_command->setAngularVelocity(angular_velocity))
            std::cout<<"Error setting angular_velocity"<<std::endl;
    }

    //msg->angular_velocity_covariance;
    // TODO


    // Linear acceleration if enabled
    if(this->isInputCommandLinearAccelerationEnabled())
    {
        Eigen::Vector3d linear_acceleration(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);

        if(imu_input_command->setLinearAcceleration(linear_acceleration))
            std::cout<<"Error setting linear_acceleration"<<std::endl;
    }

    //msg->linear_acceleration_covariance;
    // TODO

    // Set
    this->setInputCommand(time_stamp, imu_input_command);

    return 0;
}

void RosImuInputInterface::imuTopicCallback(const sensor_msgs::ImuConstPtr& msg)
{
    this->setInputCommandRos(msg);

    return;
}

int RosImuInputInterface::setImuTopicName(const std::string imu_topic_name)
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

int RosImuInputInterface::publish()
{
    // TODO

    return 0;
}

int RosImuInputInterface::readConfig(const pugi::xml_node &input, std::shared_ptr<ImuInputStateCore>& init_state_core)
{
    /// Imu Sensor Configs
    int error_read_config=this->ImuInputCore::readConfig(input, init_state_core);

    if(error_read_config)
        return error_read_config;


    /// Ros Configs
    // Input Topic
    std::string input_topic=input.child_value("ros_topic");
    this->setImuTopicName(input_topic);


    /// Finish

    // Open
    this->open();

    // End
    return 0;
}

