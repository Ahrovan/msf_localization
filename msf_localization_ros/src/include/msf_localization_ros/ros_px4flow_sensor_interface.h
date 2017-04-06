

#ifndef _ROS_PX4FLOW_SENSOR_INTEFACE_H
#define _ROS_PX4FLOW_SENSOR_INTEFACE_H





// ROS Msg
#include "px_comm/OpticalFlow.h"

// ROS Msg for estimated
//#include <geometry_msgs/Vector3Stamped.h>


// Time Stamp
#include "time_stamp/time_stamp.h"

// Px4Flow Sensor Core
#include "msf_localization_core/px4flow_sensor_core.h"

// Sensor measurement core
#include "msf_localization_core/px4flow_sensor_measurement_core.h"

// ROS Sensor Interface
#include "msf_localization_ros/ros_sensor_interface.h"

// Pugixml
#include "pugixml/pugixml.hpp"



class RosPx4FlowSensorInterface : public RosSensorInterface, public Px4FlowSensorCore
{
public:
    RosPx4FlowSensorInterface(ros::NodeHandle* nh, tf::TransformBroadcaster *tf_transform_broadcaster, MsfLocalizationCore* msf_localization_core_ptr);



public:
    int setMeasurementRos(const px_comm::OpticalFlowConstPtr& msg);


    // Frequencies
protected:
    ros::Time previous_time_stamp_;
    double frequency_desired_;


    // Subscriber
protected:
    std::string px4flow_meas_topic_name_;
protected:
    ros::Subscriber px4flow_meas_sub_;
    void px4FlowMeasCallback(const px_comm::OpticalFlowConstPtr& msg);
public:
    int setPx4FlowMeasTopicName(const std::string px4flow_meas_topic_name);



public:
    int open();


public:
    int publish(const TimeStamp& time_stamp,
                const std::shared_ptr<RosRobotInterface>& robot_core,
                const std::shared_ptr<SensorStateCore>& sensor_state_core);

public:
    int readConfig(const pugi::xml_node& sensor, unsigned int sensorId, std::shared_ptr<Px4FlowSensorStateCore>& SensorInitStateCore);


};




#endif
