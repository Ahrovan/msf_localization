
#include "msf_localization_ros/ros_px4flow_sensor_interface.h"





RosPx4FlowSensorInterface::RosPx4FlowSensorInterface(ros::NodeHandle* nh, tf::TransformBroadcaster *tf_transform_broadcaster, MsfLocalizationCore* msf_localization_core_ptr) :
    RosSensorInterface(nh, tf_transform_broadcaster),
    Px4FlowSensorCore(msf_localization_core_ptr)
{

    // Frequencies
    frequency_desired_=-1.0; // Hz
    previous_time_stamp_=ros::Time(0,0);

    return;
}



int RosPx4FlowSensorInterface::setPx4FlowMeasTopicName(const std::string px4flow_meas_topic_name)
{
    this->px4flow_meas_topic_name_=px4flow_meas_topic_name;
    return 0;
}


int RosPx4FlowSensorInterface::setMeasurementRos(const px_comm::OpticalFlowConstPtr& msg)
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



    // Time Stamp
    TimeStamp time_stamp(msg->header.stamp.sec, msg->header.stamp.nsec);


    // Sensor Core
    //std::shared_ptr<Px4FlowSensorCore> sensor_core=std::dynamic_pointer_cast<Px4FlowSensorCore>(this->getMsfElementCoreSharedPtr());

    // Value
    std::shared_ptr<Px4FlowSensorMeasurementCore> sensor_measurement_core=std::make_shared<Px4FlowSensorMeasurementCore>(this->getSensorCoreWeakPtr());


    // Values
    int quality=msg->quality;
    double ground_distance=msg->ground_distance;


    // Velocity
    if(this->isMeasurementVelocityEnabled())
    {
        // We avoid to use bad optical flow measurements
        if(quality >= 200 && ground_distance >= 0.30001)
        {
            Eigen::Vector2d velocity(msg->velocity_x, msg->velocity_y);
            sensor_measurement_core->setVelocity(velocity);
        }
    }

    // Ground Distance
    if(this->isMeasurementGroundDistanceEnabled())
    {
        // We avoid to use erroneous measurements
        if(ground_distance > 0.30001)
        {
            sensor_measurement_core->setGroundDistance(ground_distance);
        }
    }


    // Set measurements
    this->setMeasurement(time_stamp, sensor_measurement_core);


    return 0;
}


void RosPx4FlowSensorInterface::px4FlowMeasCallback(const px_comm::OpticalFlowConstPtr& msg)
{
    this->setMeasurementRos(msg);

    return;
}


int RosPx4FlowSensorInterface::open()
{
    // Subscriber
    px4flow_meas_sub_=nh->subscribe(px4flow_meas_topic_name_, 10, &RosPx4FlowSensorInterface::px4FlowMeasCallback, this);

    // Publishers


    return 0;
}

int RosPx4FlowSensorInterface::publish(const TimeStamp& time_stamp, const std::shared_ptr<RosRobotInterface> &robot_core, const std::shared_ptr<SensorStateCore> &sensor_state_core)
{
    // tf pose sensor wrt robot
    this->publishTfPoseSensorWrtRobot(time_stamp, robot_core, sensor_state_core);


    // end
    return 0;
}

int RosPx4FlowSensorInterface::readConfig(const pugi::xml_node& sensor, unsigned int sensorId, std::shared_ptr<Px4FlowSensorStateCore>& SensorInitStateCore)
{
    /// Imu Sensor Configs
    int errorReadConfig=this->Px4FlowSensorCore::readConfig(sensor, sensorId, SensorInitStateCore);

    if(errorReadConfig)
        return errorReadConfig;


    /// Ros Configs
    // Sensor Topic
    std::string sensor_topic=sensor.child_value("ros_topic");
    this->setPx4FlowMeasTopicName(sensor_topic);



    /// Finish

    // Open
    this->open();

    // End
    return 0;
}
