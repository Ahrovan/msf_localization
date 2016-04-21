//////////////////////////////////////////////////////
//  droneMsfLocalizationROSModule.h
//
//  Created on: Feb, 2015
//      Author: joselusl
//
//  Last modification on:
//      Author: joselusl
//
//////////////////////////////////////////////////////


#ifndef _MSF_ODOMETRY_ROS_H
#define _MSF_ODOMETRY_ROS_H




//I/O stream
//std::cout
#include <iostream>

//String
//std::string, std::getline()
#include <string>

//String stream
//std::istringstream
#include <sstream>

//File Stream
//std::ofstream, std::ifstream
#include <fstream>


//Vector
//std::vector
#include <vector>

// List
#include <list>


//PUGIXML
#include <pugixml/pugixml.hpp>


// Boost
#include <boost/filesystem.hpp>

// Thread
#include <thread>

// Exceptions
#include <exception>


//ROS
#include <ros/ros.h>


// ROS msg
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>


// ROS Msg geometry_msgs::Vector3
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Vector3.h>


// tf
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

// Ros service
#include <msf_localization_ros_srvs/SetBool.h>


// Robot
#include "msf_localization_core/free_model_robot_core.h"
#include "msf_localization_core/free_model_robot_state_core.h"


// ROS Sensor Interface
#include "msf_localization_ros/ros_sensor_interface.h"


// ROS IMU Interface
#include "msf_localization_ros/ros_sensor_imu_interface.h"

// ROS Aruco Eye
#include "msf_localization_ros/ros_aruco_eye_interface.h"

// MSF Core
#include "msf_localization_core/msfLocalization.h"


#include "msf_localization_core/global_parameters_core.h"
#include "msf_localization_core/global_parameters_state_core.h"


#define _DEBUG_TIME_MSF_LOCALIZATION_ROS 1

#define _DEBUG_MODE 0
#define _DEBUG_MSF_LOCALIZATION_ROBOT_POSE_THREAD 0


class MsfLocalizationROS : public MsfLocalizationCore
{
public:
    MsfLocalizationROS(int argc,char **argv);
    ~MsfLocalizationROS();

private:
    ros::NodeHandle* nh;


protected:
    int readParameters();

protected:
    int init();
    int close();

public:
    int open();
    int run();


    // Config File
protected:
    std::string configFile;
protected:
    int setConfigFile(std::string configFile);
    int readConfigFile();


    // Particular config readings
protected:
    int readGlobalParametersConfig(pugi::xml_node global_parameters, std::shared_ptr<MsfStorageCore> TheMsfStorageCore, std::shared_ptr<GlobalParametersCore>& TheGlobalParametersCoreAux, std::shared_ptr<GlobalParametersStateCore>& GlobalParametersInitStateCore);

protected:
    int readFreeModelRobotConfig(pugi::xml_node robot, std::shared_ptr<MsfStorageCore> TheMsfStorageCore, std::shared_ptr<FreeModelRobotCore>& TheRobotCoreAux, std::shared_ptr<FreeModelRobotStateCore>& RobotInitStateCore);

protected:
    int readImuConfig(pugi::xml_node sensor, unsigned int sensorId, std::shared_ptr<MsfStorageCore> TheMsfStorageCore, std::shared_ptr<RosSensorImuInterface>& TheRosSensorImuInterface, std::shared_ptr<ImuSensorStateCore>& SensorInitStateCore);
    int readArucoEyeConfig(pugi::xml_node sensor, unsigned int sensorId, std::shared_ptr<MsfStorageCore> TheMsfStorageCore, std::shared_ptr<RosArucoEyeInterface>& TheRosArucoEyeInterface, std::shared_ptr<CodedVisualMarkerEyeStateCore>& SensorInitStateCore);

protected:
    int readCodedVisualMarkerConfig(pugi::xml_node map_element, std::shared_ptr<MsfStorageCore> TheMsfStorageCore, std::shared_ptr<CodedVisualMarkerLandmarkCore>& TheMapElementCore, std::shared_ptr<CodedVisualMarkerLandmarkStateCore>& InitStateCore);



    // Robot Pose Publisher
protected:
    // Robot Pose with Covariance Stamped
    ros::Publisher robotPoseWithCovarianceStampedPub;
    std::string robotPoseWithCovarianceStampedTopicName;
    geometry_msgs::PoseWithCovarianceStamped robotPoseWithCovarianceStampedMsg;

    // Robot Pose Stamped
    ros::Publisher robotPoseStampedPub;
    std::string robotPoseStampedTopicName;
    geometry_msgs::PoseStamped robotPoseStampedMsg;

    // Robot Linear Speed
    ros::Publisher robotLinearSpeedStampedPub;
    std::string robotLinearSpeedStampedTopicName;
    geometry_msgs::Vector3Stamped robotLinearSpeedStampedMsg;

    // Robot Linear Acceleration
    ros::Publisher robotLinearAccelerationStampedPub;
    std::string robotLinearAccelerationStampedTopicName;
    geometry_msgs::Vector3Stamped robotLinearAccelerationStampedMsg;

    // Robot Angular Velocity
    ros::Publisher robotAngularVelocityStampedPub;
    std::string robotAngularVelocityStampedTopicName;
    geometry_msgs::Vector3Stamped robotAngularVelocityStampedMsg;

    // Robot Angular Acceleration
    ros::Publisher robotAngularAccelerationStampedPub;
    std::string robotAngularAccelerationStampedTopicName;
    geometry_msgs::Vector3Stamped robotAngularAccelerationStampedMsg;


    // Tf
protected:
    tf::TransformBroadcaster* tfTransformBroadcaster;


protected:
    double robotPoseRateVal;
    ros::Rate* robotPoseRate;
protected:
    std::thread* robotPoseThread;
//    SyncThreadState robotPoseThreadState;
protected:
    int robotPoseThreadFunction();


    // Service to start state estimation
protected:
    std::string setStateEstimationEnabledServiceName;
    ros::ServiceServer setStateEstimationEnabledSrv;
    bool setStateEstimationEnabledCallback(msf_localization_ros_srvs::SetBool::Request  &req, msf_localization_ros_srvs::SetBool::Response &res);


    // Time Stamp
public:
    TimeStamp getTimeStamp();


    // Algorithm Core threads with ROS time
protected:
    int predictThreadFunction();


    // Buffer thread manager
protected:
    int bufferManagerThreadFunction();


};






#endif
