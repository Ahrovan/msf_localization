

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



#include "msf_localization_core/block_matrix.h"


#include "msf_localization_core/state_estimation_core.h"


#include "msf_localization_core/msf_storage_core.h"




//// Estimator Cores

/// Robot
#include "msf_localization_core/robot_core.h"
/// Global Parameters
#include "msf_localization_core/global_parameters_core.h"
/// Sensor
#include "msf_localization_core/sensor_core.h"
/// Input
#include "msf_localization_core/input_core.h"
/// Map
#include "msf_localization_core/map_element_core.h"


//// State
#include "msf_localization_core/state_core.h"
// TODO REMOVE
#include "msf_localization_core/map_element_state_core.h"


//// Measurement
#include "msf_localization_core/sensor_measurement_core.h"


//// Input Command
#include "msf_localization_core/input_command_core.h"





#define _DEBUG_MSF_LOCALIZATION_CORE 0
#define _DEBUG_MSF_LOCALIZATION_ALGORITHM 0

#define _DEBUG_TIME_MSF_LOCALIZATION_CORE 0
#define _DEBUG_TIME_MSF_LOCALIZATION_THREAD 1

#define _DEBUG_ERROR_MSF_LOCALIZATION_CORE 1

#define _BUFFER_PROPAGATION_MULTI_THREADING 0

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


    // Storage
public:
    std::shared_ptr<MsfStorageCore> TheMsfStorageCore;



    /// Components

    // Global Parameters
protected:
    std::shared_ptr<GlobalParametersCore> TheGlobalParametersCore;


    // Robot Component
protected:
    std::shared_ptr<RobotCore> TheRobotCore;


    // Inputs
protected:
    std::list< std::shared_ptr<InputCore> > TheListOfInputCore;


    // Sensors Components
protected:
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

    // isAlive
public:
    virtual bool isAlive();


    // State Estimation
protected:
    bool stateEstimationEnabled;
public:
    int setStateEstimationEnabled(bool predictEnabled);
    bool isStateEstimationEnabled() const;




    // Get previous state
protected:
    int getPreviousState(const TimeStamp& TheTimeStamp,
                         TimeStamp& ThePreviousTimeStamp,
                         std::shared_ptr<StateEstimationCore>& ThePreviousState);

    // Fill inputs
protected:
    int findInputCommands(const TimeStamp& TheTimeStamp,
                          //const std::shared_ptr<StateEstimationCore> ThePreviousState,
                          std::shared_ptr<InputCommandComponent>& input_command);



    /// Predict Step Functions
protected:
    int predict(const TimeStamp& TheTimeStamp);

    int predictNoAddBuffer(const TimeStamp& TheTimeStamp,
                           std::shared_ptr<StateEstimationCore>& ThePredictedState);

private:
    int predictSemiCore(const TimeStamp& ThePredictedTimeStamp,
                        std::shared_ptr<StateEstimationCore>& ThePredictedState);

    int predictCore(const TimeStamp& ThePreviousTimeStamp, const TimeStamp& ThePredictedTimeStamp,
                    // Previous State
                    const std::shared_ptr<StateEstimationCore>& ThePreviousState,
                    // Inputs
                    const std::shared_ptr<InputCommandComponent>& inputCommand,
                    // Predicted State
                    std::shared_ptr<StateEstimationCore>& ThePredictedState);



    /// Update Step functions
protected:
    int update(const TimeStamp& TheTimeStamp);

private:
    int updateCore(const TimeStamp& TheTimeStamp,
                   const std::shared_ptr<StateEstimationCore>& OldState,
                   std::shared_ptr<StateEstimationCore>& UpdatedState);






    // Remove unneded current state
public:
    int removeUnnecessaryStateFromBuffer(const TimeStamp& time_stamp);


    // Find next element in buffer and add to outdated list
public:
    int findNextElementInBufferAndAddOutdatedList(const TimeStamp& time_stamp);




    // Time what is required betwwwn two prediction steps
protected:
    double predict_model_time_; // in seconds


    // Predict Thread
protected:
    std::thread* predictThread;
protected:
    // TODO Finish
    virtual int predictThreadFunction();

protected:
    int predictThreadStep();




    // Buffer Manager Thread
protected:
    std::thread* bufferManagerThread;
protected:
    int bufferManagerThreadFunction();
protected:
    int bufferPropagationStep(const TimeStamp& time_stamp);

#if _BUFFER_PROPAGATION_MULTI_THREADING
protected:
    int num_buffer_propagation_threads=0;
#endif



    // Start threads
public:
    int startThreads();







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
