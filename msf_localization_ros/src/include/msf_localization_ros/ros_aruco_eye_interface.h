


#ifndef _ROS_ARUCO_EYE_INTEFACE_H
#define _ROS_ARUCO_EYE_INTEFACE_H





// ROS Msg
#include "aruco_eye_msgs/MarkerList.h"


// Time Stamp
#include "msf_localization_core/time_stamp.h"

// Imu Sensor Core
#include "msf_localization_core/coded_visual_marker_eye_core.h"

// Sensor measurement core
#include "msf_localization_core/coded_visual_marker_measurement_core.h"


// ROS Sensor Interface
#include "msf_localization_ros/ros_sensor_interface.h"


#include "pugixml/pugixml.hpp"



class RosArucoEyeInterface : public RosSensorInterface, public CodedVisualMarkerEyeCore
{
public:
    RosArucoEyeInterface(ros::NodeHandle* nh, tf::TransformBroadcaster *tf_transform_broadcaster, std::weak_ptr<MsfStorageCore> the_msf_storage_core);



public:
    int setMeasurementRos(const aruco_eye_msgs::MarkerListPtr& msg);


    // Subscriber
protected:
    std::string marker_list_topic_name_;
protected:
    ros::Subscriber marker_list_sub_;
    void markerListCallback(const aruco_eye_msgs::MarkerListPtr& msg);
public:
    int setMarkerListTopicName(std::string marker_list_topic_name);


public:
    int open();


public:
    int publish(TimeStamp time_stamp, std::shared_ptr<RosRobotInterface> robot_core, std::shared_ptr<SensorStateCore> sensor_state_core);


public:
    int readConfig(pugi::xml_node sensor, unsigned int sensorId, std::shared_ptr<CodedVisualMarkerEyeStateCore>& SensorInitStateCore);

};




#endif
