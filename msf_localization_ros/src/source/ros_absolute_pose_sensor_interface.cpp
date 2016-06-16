#include "msf_localization_ros/ros_absolute_pose_sensor_interface.h"


RosAbsolutePoseSensorInterface::RosAbsolutePoseSensorInterface(ros::NodeHandle* nh, tf::TransformBroadcaster *tf_transform_broadcaster, MsfLocalizationCore* msf_localization_core_ptr) :
    RosSensorInterface(nh, tf_transform_broadcaster),
    AbsolutePoseSensorCore(msf_localization_core_ptr)
{
    this->sensor_measurement_message_type_=AbsolutePoseSensorMeasurementMessageTypes::undefined;


    return;
}

int RosAbsolutePoseSensorInterface::setMeasurementMocapSensorWrtMocapWorldTopicName(std::string measurement_mocap_sensor_wrt_mocap_world_topic_name)
{
    this->measurement_mocap_sensor_wrt_mocap_world_topic_name_=measurement_mocap_sensor_wrt_mocap_world_topic_name;
    return 0;
}


int RosAbsolutePoseSensorInterface::setMeasurementRos(const geometry_msgs::PoseStampedPtr& msg)
{
    if(!isSensorEnabled())
        return 0;

//    if(msg->header.seq  % 3 != 0)
//        return 0;


    // Time Stamp
    TimeStamp the_time_stamp(msg->header.stamp.sec, msg->header.stamp.nsec);


    // Value with sensor core
    std::shared_ptr<AbsolutePoseSensorCore> sensor_core=std::dynamic_pointer_cast<AbsolutePoseSensorCore>(this->getMsfElementCoreSharedPtr());
    std::shared_ptr<AbsolutePoseSensorMeasurementCore> measurement_core=std::make_shared<AbsolutePoseSensorMeasurementCore>(sensor_core);


    // Measurement Position
    if(this->isMeasurementPositionMocapSensorWrtMocapWorldEnabled())
    {
        Eigen::Vector3d position(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

        if(measurement_core->setPositionMocapSensorWrtMocapWorld(position))
            std::cout<<"Error setting position"<<std::endl;

    }

    // Measurement Attitude
    if(this->isMeasurementAttitudeMocapSensorWrtMocapWorldEnabled())
    {
        Eigen::Vector4d orientation;
        Eigen::Vector4d orientation_aux;

        orientation_aux<<msg->pose.orientation.w,
                    msg->pose.orientation.x,
                    msg->pose.orientation.y,
                    msg->pose.orientation.z;

        if(orientation_aux[0]<0)
        {
            orientation=-orientation_aux;
        }
        else
        {
            orientation=orientation_aux;
        }

        if(measurement_core->setAttitudeMocapSensorWrtMocapWorld(orientation))
            std::cout<<"Error setting orientation"<<std::endl;
    }

    // Covariance
    // No covariance subscribed


    // Set
    this->setMeasurement(the_time_stamp, measurement_core);



    return 0;
}

int RosAbsolutePoseSensorInterface::setMeasurementRos(const geometry_msgs::PoseWithCovarianceStampedPtr& msg)
{
    if(!isSensorEnabled())
        return 0;

//    if(msg->header.seq  % 3 != 0)
//        return 0;


    // Time Stamp
    TimeStamp the_time_stamp(msg->header.stamp.sec, msg->header.stamp.nsec);


    // Value with sensor core
    std::shared_ptr<AbsolutePoseSensorCore> sensor_core=std::dynamic_pointer_cast<AbsolutePoseSensorCore>(this->getMsfElementCoreSharedPtr());
    std::shared_ptr<AbsolutePoseSensorMeasurementCore> measurement_core=std::make_shared<AbsolutePoseSensorMeasurementCore>(sensor_core);


    // Measurement Position
    if(this->isMeasurementPositionMocapSensorWrtMocapWorldEnabled())
    {
        Eigen::Vector3d position(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

        if(measurement_core->setPositionMocapSensorWrtMocapWorld(position))
            std::cout<<"Error setting position"<<std::endl;

    }

    // Measurement Attitude
    if(this->isMeasurementAttitudeMocapSensorWrtMocapWorldEnabled())
    {
        Eigen::Vector4d orientation;
        Eigen::Vector4d orientation_aux;

        orientation_aux<<msg->pose.pose.orientation.w,
                    msg->pose.pose.orientation.x,
                    msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z;

        if(orientation_aux[0]<0)
        {
            orientation=-orientation_aux;
        }
        else
        {
            orientation=orientation_aux;
        }

        if(measurement_core->setAttitudeMocapSensorWrtMocapWorld(orientation))
            std::cout<<"Error setting orientation"<<std::endl;
    }

    // Covariance
    if(this->hasSensorMeasurementPoseSensorWrtSensorWorldCovariance())
    {
        Eigen::MatrixXd noise_sensor_measurement_pose_sensor_wrt_sensor_world=
                Eigen::Map<Eigen::MatrixXd>(msg->pose.covariance.c_array(), 6, 6);
        measurement_core->setNoiseSensorMeasurementPoseSensorWrtSensorWorld(noise_sensor_measurement_pose_sensor_wrt_sensor_world);
    }

    // Set
    this->setMeasurement(the_time_stamp, measurement_core);



    return 0;
}

void RosAbsolutePoseSensorInterface::setSensorMeasurementMessageType(AbsolutePoseSensorMeasurementMessageTypes sensor_measurement_message_type)
{
    this->sensor_measurement_message_type_=sensor_measurement_message_type;
    return;
}

void RosAbsolutePoseSensorInterface::measurementMocapSensorWrtMocapWorldCallbackPoseStamped(const geometry_msgs::PoseStampedPtr& msg)
{
    this->setMeasurementRos(msg);

    return;
}

void RosAbsolutePoseSensorInterface::measurementMocapSensorWrtMocapWorldCallbackPoseWithCovarianceStamped(const geometry_msgs::PoseWithCovarianceStampedPtr& msg)
{
    this->setMeasurementRos(msg);

    return;
}

int RosAbsolutePoseSensorInterface::open()
{
    // Sensor Measurement
    switch(sensor_measurement_message_type_)
    {
        case AbsolutePoseSensorMeasurementMessageTypes::geometry_msgs_PoseStamped:
        {
            measurement_mocap_sensor_wrt_mocap_world_list_sub_=nh->subscribe(measurement_mocap_sensor_wrt_mocap_world_topic_name_, 10, &RosAbsolutePoseSensorInterface::measurementMocapSensorWrtMocapWorldCallbackPoseStamped, this);
            break;
        }
        case AbsolutePoseSensorMeasurementMessageTypes::geometry_msgs_PoseWithCovarianceStamped:
        {
            measurement_mocap_sensor_wrt_mocap_world_list_sub_=nh->subscribe(measurement_mocap_sensor_wrt_mocap_world_topic_name_, 10, &RosAbsolutePoseSensorInterface::measurementMocapSensorWrtMocapWorldCallbackPoseWithCovarianceStamped, this);
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

int RosAbsolutePoseSensorInterface::publish(const TimeStamp& time_stamp, const std::shared_ptr<RosRobotInterface> &robot_core, const std::shared_ptr<SensorStateCore> &sensor_state_core)
{
    // tf pose sensor wrt robot
    this->publishTfPoseSensorWrtRobot(time_stamp, robot_core, sensor_state_core);

    // end
    return 0;
}

int RosAbsolutePoseSensorInterface::readConfig(const pugi::xml_node& sensor, unsigned int sensorId, std::shared_ptr<AbsolutePoseSensorStateCore>& SensorInitStateCore)
{
    /// Sensor General Configs
    int errorReadConfig=this->AbsolutePoseSensorCore::readConfig(sensor, sensorId, SensorInitStateCore);

    if(errorReadConfig)
        return errorReadConfig;



    /// ROS Configs

    // Sensor Topic
    std::string sensor_topic=sensor.child_value("ros_topic");
    this->setMeasurementMocapSensorWrtMocapWorldTopicName(sensor_topic);


    // Message Type Sensor Measurement Topic
    std::string type_sensor_measurement_topic=sensor.child_value("ros_topic_type");
    if(type_sensor_measurement_topic=="geometry_msgs::PoseStamped")
        this->setSensorMeasurementMessageType(AbsolutePoseSensorMeasurementMessageTypes::geometry_msgs_PoseStamped);
    else if(type_sensor_measurement_topic=="geometry_msgs::PoseWithCovarianceStamped")
        this->setSensorMeasurementMessageType(AbsolutePoseSensorMeasurementMessageTypes::geometry_msgs_PoseWithCovarianceStamped);



    /// Finish

    // Open
    this->open();

    // End
    return 0;
}
