#include "msf_localization_ros/ros_mocap_sensor_interface.h"


RosMocapSensorInterface::RosMocapSensorInterface(ros::NodeHandle* nh, tf::TransformBroadcaster *tf_transform_broadcaster, const std::weak_ptr<MsfStorageCore> the_msf_storage_core) :
    RosSensorInterface(nh, tf_transform_broadcaster),
    AbsolutePoseSensorCore(the_msf_storage_core)
{


    return;
}

int RosMocapSensorInterface::setMeasurementMocapSensorWrtMocapWorldTopicName(std::string measurement_mocap_sensor_wrt_mocap_world_topic_name)
{
    this->measurement_mocap_sensor_wrt_mocap_world_topic_name_=measurement_mocap_sensor_wrt_mocap_world_topic_name;
    return 0;
}


int RosMocapSensorInterface::setMeasurementRos(const geometry_msgs::PoseStampedPtr& msg)
{
    if(!isSensorEnabled())
        return 0;

    if(msg->header.seq  % 4 != 0)
        return 0;


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


    // Set
    this->setMeasurement(the_time_stamp, measurement_core);



    return 0;
}


void RosMocapSensorInterface::measurementMocapSensorWrtMocapWorldCallback(const geometry_msgs::PoseStampedPtr& msg)
{
    this->setMeasurementRos(msg);

    return;
}


int RosMocapSensorInterface::open()
{
    // Subscriber
    measurement_mocap_sensor_wrt_mocap_world_list_sub_=nh->subscribe(measurement_mocap_sensor_wrt_mocap_world_topic_name_, 10, &RosMocapSensorInterface::measurementMocapSensorWrtMocapWorldCallback, this);


    return 0;
}

int RosMocapSensorInterface::publish(const TimeStamp& time_stamp, const std::shared_ptr<RosRobotInterface> robot_core, const std::shared_ptr<SensorStateCore> sensor_state_core)
{
    // tf pose sensor wrt robot
    this->publishTfPoseSensorWrtRobot(time_stamp, robot_core, sensor_state_core);

    // end
    return 0;
}

int RosMocapSensorInterface::readConfig(const pugi::xml_node& sensor, unsigned int sensorId, std::shared_ptr<AbsolutePoseSensorStateCore>& SensorInitStateCore)
{
    /// Sensor General Configs
    int errorReadConfig=this->AbsolutePoseSensorCore::readConfig(sensor, sensorId, SensorInitStateCore);

    if(errorReadConfig)
        return errorReadConfig;



    /// ROS Configs

    // Sensor Topic
    std::string sensor_topic=sensor.child_value("ros_topic");
    this->setMeasurementMocapSensorWrtMocapWorldTopicName(sensor_topic);



    /// Finish

    // Open
    this->open();

    // End
    return 0;
}
