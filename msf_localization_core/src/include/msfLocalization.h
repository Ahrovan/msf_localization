

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




    // Time Stamps. Pure virtual function
public:
    virtual TimeStamp getTimeStamp();



    // State Estimation
protected:
    bool stateEstimationEnabled;
public:
    int setStateEstimationEnabled(bool predictEnabled);
    bool isStateEstimationEnabled() const;



    // Predict Functions
protected:
    int predict(TimeStamp TheTimeStamp, std::shared_ptr<StateEstimationCore>& PredictedState);



    // Update functions
protected:
    int update(TimeStamp TheTimeStamp, std::shared_ptr<StateEstimationCore>& UpdatedState);






    // Predict Thread
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
    // TODO finish
    virtual int bufferManagerThreadFunction();





    // Start threads
public:
    int startThreads();



    //// Helper functions

    // Helper functions
protected:
    int findSensorStateCoreFromList(std::list<std::shared_ptr<SensorStateCore> > TheListSensorStateCore, std::shared_ptr<SensorCore> TheSensorCore, std::shared_ptr<SensorStateCore>& TheSensorStateCore);


    // Helper functions
protected:
    struct MatrixPoint
    {
        unsigned int row;
        unsigned int col;
    };


    // Concrete functions for covariances
protected:
    int predictedFreeModelRobotCovariance(TimeStamp DeltaTime, std::shared_ptr<FreeModelRobotCore> robotCore, std::shared_ptr<FreeModelRobotStateCore> predictedStateRobot, Eigen::MatrixXd* previousStateCovarianceMatrix, Eigen::MatrixXd* predictedStateCovarianceMatrix);

    int predictedFreeModelRobotImuCovariance(TimeStamp DeltaTime, std::shared_ptr<FreeModelRobotCore> robotCore, std::shared_ptr<ImuSensorCore> TheImuSensor1Core, std::shared_ptr<FreeModelRobotStateCore> predictedStateRobot, std::shared_ptr<ImuSensorStateCore> predictedImuStateSensor1, Eigen::MatrixXd* previousStateCovarianceMatrix, MatrixPoint InitPoint, Eigen::MatrixXd* predictedStateCovarianceMatrix, MatrixPoint& EndPoint);

    int predictedImuImuCovariance(TimeStamp DeltaTime, std::shared_ptr<ImuSensorCore> TheImuSensor1Core, std::shared_ptr<ImuSensorCore> TheImuSensor2Core, std::shared_ptr<ImuSensorStateCore> predictedImuStateSensor1, std::shared_ptr<ImuSensorStateCore> predictedImuStateSensor2, Eigen::MatrixXd* previousStateCovarianceMatrix, MatrixPoint InitPoint, Eigen::MatrixXd* predictedStateCovarianceMatrix, MatrixPoint& EndPoint);


    //// Debug log
protected:
    std::string logPath;
    std::ofstream logFile;

};




#endif
