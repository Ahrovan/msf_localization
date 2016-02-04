

#ifndef _MSF_LOCALIZATION_CORE_H
#define _MSF_LOCALIZATION_CORE_H


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
#include "pugixml.hpp"


// Boost
#include <boost/filesystem.hpp>

// Thread
#include <thread>


// Estimator Cores

// Robot
#include "robot_core.h"

// Sensor
#include "sensor_core.h"
// IMU
#include "imu_sensor_core.h"



class MsfLocalizationCore
{

public:
    MsfLocalizationCore();
    ~MsfLocalizationCore();


protected:
    int init();
    int close();

public:
    int open();
    int run();


    // Robot Component
protected:
    RobotCore TheRobotCore;



    // Sensors Components
protected:
    std::list<SensorCore*> TheListOfSensorCore;



    // Covariances Matrixes
protected:
    // TODO Use Ring-Buffers
    // TODO k|k
    // TODO k+1|k
    // TODO K+1|k+1


    // Predict Thread
protected:
    bool predictEnabled;
    int setPredictEnabled(bool predictEnabled);
protected:
    double predictRate;
    std::thread* predictThread;
protected:
    int predictThreadFunction();


    // Predict Functions
protected:
    int predict();



};




#endif
