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







// Ros service
#include <msf_localization_ros_srvs/SetBool.h>



// ROS Interface
#include "msf_localization_ros/ros_interface.h"


/// ROS Sensors
// ROS IMU Interface
#include "msf_localization_ros/ros_sensor_imu_interface.h"
// ROS Aruco Eye
#include "msf_localization_ros/ros_aruco_eye_interface.h"


/// ROS Inputs
// ROS IMU Input
#include "msf_localization_ros/ros_imu_input_interface.h"



// MSF Core
#include "msf_localization_core/msfLocalization.h"



// Global Parameters (World)
#include "msf_localization_core/global_parameters_core.h"
#include "msf_localization_core/global_parameters_state_core.h"

// Robot
#include "msf_localization_ros/ros_free_model_robot_interface.h"
#include "msf_localization_core/free_model_robot_state_core.h"




#define _DEBUG_TIME_MSF_LOCALIZATION_ROS 1

#define _DEBUG_MODE 0
#define _DEBUG_MSF_LOCALIZATION_ROBOT_POSE_THREAD 0


class MsfLocalizationROS : public MsfLocalizationCore, public RosInterface
{
public:
    MsfLocalizationROS(int argc,char **argv);
    ~MsfLocalizationROS();


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




protected:
    double robotPoseRateVal;
    ros::Rate* robotPoseRate;
protected:
    std::thread* robotPoseThread;
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
