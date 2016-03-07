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


// ROS Msg geometry_msgs::Vector3
#include <geometry_msgs/Vector3Stamped.h>


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


// MSF Core
#include "msf_localization_core/msfLocalization.h"


#include "msf_localization_core/global_parameters_core.h"
#include "msf_localization_core/global_parameters_state_core.h"


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
    int readFreeModelRobotConfig(pugi::xml_node robot, std::shared_ptr<MsfStorageCore> TheMsfStorageCore, std::shared_ptr<FreeModelRobotCore>& TheRobotCoreAux, std::shared_ptr<FreeModelRobotStateCore>& RobotInitStateCore);
    int readImuConfig(pugi::xml_node sensor, unsigned int sensorId, std::shared_ptr<MsfStorageCore> TheMsfStorageCore, std::shared_ptr<RosSensorImuInterface>& TheRosSensorImuInterface, std::shared_ptr<ImuSensorStateCore>& SensorInitStateCore);




    // Robot Pose Publisher
protected:
    ros::Publisher robotPosePub;
    std::string robotPoseTopicName;

    geometry_msgs::PoseWithCovarianceStamped robotPoseMsg;


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