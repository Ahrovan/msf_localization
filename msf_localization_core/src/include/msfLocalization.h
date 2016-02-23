

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

//  shared_ptr
#include <memory>


//PUGIXML
//#include "pugixml.hpp"


// Boost
#include <boost/filesystem.hpp>

// Thread
#include <thread>

// Chrono
#include <chrono>


// Estimator Cores

// Robot
#include "robot_core.h"

// Sensor
#include "sensor_core.h"
// IMU
#include "imu_sensor_core.h"



// Robot
#include "robot_state_core.h"
// Free Model Robot Core
#include "free_model_robot_core.h"


#include "sensor_state_core.h"
#include "sensor_measurement_core.h"


#include "state_estimation_core.h"


#include "msf_storage_core.h"


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


public:
    std::shared_ptr<MsfStorageCore> TheMsfStorageCore;


    // Robot Component
protected:
    std::shared_ptr<RobotCore> TheRobotCore;



    // Sensors Components
protected:
//public:
    std::list< std::shared_ptr<SensorCore> > TheListOfSensorCore;
protected:
    unsigned int firstAvailableId;

public:
    // Sensor Measures
    //std::list<SensorMeasurementCore*> TheListOfSensorMeasurementsCores;



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
    double predictRateVale;
    std::thread* predictThread;
protected:
    // TODO Finish
    virtual int predictThreadFunction();



    // Buffer Manager Thread
protected:
    std::thread* bufferManagerThread;
protected:
    virtual int bufferManagerThreadFunction();



    // Start threads
public:
    int startThreads();


    // Predict Functions
protected:
    int predict(TimeStamp TheTimeStamp);





};




#endif
