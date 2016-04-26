

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

// Mutex
#include <mutex>


//Vector
//std::vector
#include <vector>

// List
#include <list>

//  shared_ptr
#include <memory>

// Thread
#include <thread>



// Boost
#include <boost/filesystem.hpp>

// Thread
#include <thread>

// Chrono
#include <chrono>


#include <limits>


#include <cmath>        // std::abs


// Estimator Cores

// Robot
#include "msf_localization_core/robot_core.h"

// Sensor
#include "msf_localization_core/sensor_core.h"
// IMU
#include "msf_localization_core/imu_sensor_core.h"
// Visual Marker Eye
#include "msf_localization_core/coded_visual_marker_eye_core.h"



// Robot
#include "msf_localization_core/robot_state_core.h"
// Free Model Robot Core
#include "msf_localization_core/free_model_robot_core.h"


#include "msf_localization_core/sensor_state_core.h"
#include "msf_localization_core/sensor_measurement_core.h"


#include "msf_localization_core/state_estimation_core.h"


#include "msf_localization_core/msf_storage_core.h"

#include "msf_localization_core/input_core.h"


#include "msf_localization_core/global_parameters_core.h"

#include "msf_localization_core/map_element_core.h"
#include "msf_localization_core/coded_visual_marker_landmark_core.h"

#include "msf_localization_core/coded_visual_marker_landmark_state_core.h"



/*
class SyncThreadState
{
protected:
    std::mutex protectionMutex;

protected:
    bool flagIsWorking;
    TimeStamp procesingTimeStamp;
public:
    SyncThreadState();

public:
    bool isWorking();
    TimeStamp getProcessingTimeStamp();

public:
    int setProcessing(TimeStamp procesingTimeStamp);
    int setNotProcessing();
};
*/


#define _DEBUG_MSF_LOCALIZATION_CORE 0
#define _DEBUG_MSF_LOCALIZATION_ALGORITHM 0

#define _DEBUG_TIME_MSF_LOCALIZATION_CORE 0

#define _DEBUG_ERROR_MSF_LOCALIZATION_CORE 1


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


    // Commands
protected:
    std::list< std::shared_ptr<InputCore> > TheListOfInputCore;


    // Global Parameters
protected:
    std::shared_ptr<GlobalParametersCore> TheGlobalParametersCore;


    // Robot Component
protected:
    std::shared_ptr<RobotCore> TheRobotCore;



    // Sensors Components
protected:
//public:
    std::list< std::shared_ptr<SensorCore> > TheListOfSensorCore;
protected:
    unsigned int firstAvailableSensorId;


    // Map Elements
protected:
    std::list< std::shared_ptr<MapElementCore> > TheListOfMapElementCore;
protected:
    unsigned int firstAvailableMapElementId;






    // Time Stamps. Pure virtual function
public:
    virtual TimeStamp getTimeStamp();



    // State Estimation
protected:
    bool stateEstimationEnabled;
public:
    int setStateEstimationEnabled(bool predictEnabled);
    bool isStateEstimationEnabled() const;




    // Get previous state
protected:
    int getPreviousState(TimeStamp TheTimeStamp, TimeStamp& ThePreviousTimeStamp, std::shared_ptr<StateEstimationCore>& ThePreviousState);


    // Predict Functions
protected:
    int predict(TimeStamp TheTimeStamp);

    int predictNoAddBuffer(TimeStamp TheTimeStamp, std::shared_ptr<StateEstimationCore>& ThePredictedState);

private:
    int predictSemiCore(TimeStamp ThePredictedTimeStamp, std::shared_ptr<StateEstimationCore>& ThePredictedState);
    int predictCore(TimeStamp ThePreviousTimeStamp, TimeStamp ThePredictedTimeStamp, std::shared_ptr<StateEstimationCore> ThePreviousState, std::shared_ptr<StateEstimationCore>& ThePredictedState);



    // Update functions
protected:
    int update(TimeStamp TheTimeStamp);

private:
    int updateCore(TimeStamp TheTimeStamp, std::shared_ptr<StateEstimationCore> OldState, std::shared_ptr<StateEstimationCore>& UpdatedState);






    // Predict Thread
protected:
    double predictRateValue;
    std::thread* predictThread;
    //SyncThreadState predictThreadState;
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

    int findMapElementStateCoreWithMapCoreFromList(std::list< std::shared_ptr<MapElementStateCore> > TheListMapElementStateCore, std::shared_ptr<MapElementCore> TheMapElementCore, std::shared_ptr<MapElementStateCore>& TheMapElementStateCore);

    int findMapElementStateCoreWithMeasurementFromList(std::list< std::shared_ptr<MapElementStateCore> > TheListMapElementStateCore, std::shared_ptr<SensorMeasurementCore> TheSensorMeasurementCore, std::shared_ptr<MapElementStateCore>& TheMapElementStateCore);

    int findMapElementCoreWithMeasurementFromList(std::list< std::shared_ptr<MapElementCore> > TheListMapElementCore, std::shared_ptr<SensorMeasurementCore> TheSensorMeasurementCore, std::shared_ptr<MapElementCore>& TheMapElementCore);


    // Helper functions
protected:
    struct MatrixPoint
    {
        unsigned int row;
        unsigned int col;
    };


    // Concrete functions for covariances
protected:
    //int predictedFreeModelRobotCovariance(TimeStamp DeltaTime, std::shared_ptr<FreeModelRobotCore> robotCore, std::shared_ptr<FreeModelRobotStateCore> predictedStateRobot, Eigen::MatrixXd* previousStateCovarianceMatrix, Eigen::MatrixXd* predictedStateCovarianceMatrix);

    //int predictedFreeModelRobotImuCovariance(TimeStamp DeltaTime, std::shared_ptr<FreeModelRobotCore> robotCore, std::shared_ptr<ImuSensorCore> TheImuSensor1Core, std::shared_ptr<FreeModelRobotStateCore> predictedStateRobot, std::shared_ptr<ImuSensorStateCore> predictedImuStateSensor1, Eigen::MatrixXd* previousStateCovarianceMatrix, MatrixPoint InitPoint, Eigen::MatrixXd* predictedStateCovarianceMatrix, MatrixPoint& EndPoint);

    //int predictedImuImuCovariance(TimeStamp DeltaTime, std::shared_ptr<ImuSensorCore> TheImuSensor1Core, std::shared_ptr<ImuSensorCore> TheImuSensor2Core, std::shared_ptr<ImuSensorStateCore> predictedImuStateSensor1, std::shared_ptr<ImuSensorStateCore> predictedImuStateSensor2, Eigen::MatrixXd* previousStateCovarianceMatrix, MatrixPoint InitPoint, Eigen::MatrixXd* predictedStateCovarianceMatrix, MatrixPoint& EndPoint);


    //// Debug log
protected:
    std::string logPath;
    std::ofstream logFile;
    // mutex to protect the log file
protected:
    std::mutex TheLogFileMutex;
public:
    int log(std::string logString);

};




#endif
