

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


#include <iomanip>
#include <ctime>



#include "msf_localization_core/block_matrix.h"


#include "msf_localization_core/state_estimation_core.h"


#include "msf_localization_core/msf_storage_core.h"




//// Estimator Cores

//
#include "msf_localization_core/msf_element_core.h"

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
#define _DEBUG_MSF_LOCALIZATION_ALGORITHM_PREDICT 0
#define _DEBUG_MSF_LOCALIZATION_ALGORITHM_UPDATE 0
#define _DEBUG_MSF_LOCALIZATION_ALGORITHM_MAPPING 0

#define _DEBUG_TIME_MSF_LOCALIZATION_CORE 0
#define _DEBUG_TIME_MSF_LOCALIZATION_THREAD 1

#define _DEBUG_ERROR_MSF_LOCALIZATION_CORE 1


#define _USE_BUFFER_IN_STATE_ESTIMATION 1
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


    // Storage for states, measurements and input commands
#if _USE_BUFFER_IN_STATE_ESTIMATION
protected:
    std::shared_ptr<MsfStorageCore> TheMsfStorageCore;
#else
protected:
    // Previous State
    const TimeStamp previous_time_stamp_;
    const std::shared_ptr<StateEstimationCore> previous_state_;
    // Inputs (if any)
    const std::shared_ptr<InputCommandComponent> input_commands_;
    // Measurements
    // TODO
    // std::list<std::shared_ptr<SensorMeasurementCore> > sensor_mesurements_;
#endif



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



    // Time Stamp getter
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



    //// Buffer Querries

    /// Measurements

    // Set a measurement in the buffer with given time stamp
public:
    int setMeasurement(const TimeStamp& time_stamp,
                       const std::shared_ptr<SensorMeasurementCore>& sensor_measurement);
    int setMeasurementList(const TimeStamp& time_stamp,
                           const std::list< std::shared_ptr<SensorMeasurementCore> >& list_sensor_measurement);

    // New measurement notification
public:
    int semaphoreNewMeasurementWait(TimeStamp& new_measurement_time_stamp);
    int semaphoreNewMeasurementNotify(const TimeStamp &new_measurement_time_stamp);
protected:
    // TODO Posible race problem
    TimeStamp new_measurement_time_stamp_;
protected:
    std::mutex new_measurement_mutex_;             // mutex for critical section
    std::condition_variable new_measurement_condition_variable_; // condition variable for critical section
    std::unique_lock<std::mutex>* new_measurement_lock_;


    /// Input Command

    // Set an input command in the buffer with given time stamp
public:
    int setInputCommand(const TimeStamp& time_stamp,
                        const std::shared_ptr<InputCommandCore>& input_command);
    int setInputCommandList(const TimeStamp& time_stamp,
                            const std::list< std::shared_ptr<InputCommandCore> >& list_input_command);


    // Get state
public:
    int getStateByStamp(const TimeStamp& requested_time_stamp,
                        TimeStamp& received_time_stamp,
                        std::shared_ptr<StateEstimationCore>& received_state);


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





    // Remove unneded current state
public:
    int removeUnnecessaryStateFromBuffer(const TimeStamp& time_stamp);


    // Find next element in buffer and add to outdated list
public:
    int findNextElementInBufferAndAddOutdatedList(const TimeStamp& time_stamp);




    /// Predict Step Functions
#if _USE_BUFFER_IN_STATE_ESTIMATION
protected:
    int predictInBuffer(const TimeStamp& TheTimeStamp);
protected:
    int predictInBufferAddBuffer(const TimeStamp& TheTimeStamp,
                                 std::shared_ptr<StateEstimationCore>& ThePredictedState);

    int predictInBufferNoAddBuffer(const TimeStamp& TheTimeStamp,
                                   std::shared_ptr<StateEstimationCore>& ThePredictedState);
private:
    int predictInBufferSemiCore(const TimeStamp& ThePredictedTimeStamp,
                                std::shared_ptr<StateEstimationCore>& ThePredictedState);
#endif

private:
    int predictCore(const TimeStamp& previous_time_stamp, const TimeStamp& predicted_time_stamp,
                    // Previous State
                    const std::shared_ptr<StateEstimationCore>& previous_state,
                    // Inputs
                    const std::shared_ptr<InputCommandComponent>& input_commands,
                    // Predicted State
                    std::shared_ptr<StateEstimationCore>& predicted_state);




    /// Update Step functions

#if _USE_BUFFER_IN_STATE_ESTIMATION
protected:
    int updateInBuffer(const TimeStamp& TheTimeStamp);
#endif

private:
    int updateCore(const TimeStamp& TheTimeStamp,
                   const std::shared_ptr<StateEstimationCore>& OldState,
                   const std::shared_ptr<SensorMeasurementComponent>& sensor_measurement_component,
                   std::shared_ptr<StateEstimationCore>& UpdatedState);




    /////

    // Time what is required between two prediction steps
protected:
    double predict_model_time_; // in seconds
    // if > 0 -> predict with period
    // if == 0 -> predict only if measurements or input commands
    // if < 0 -> predict only if measurements


    // Predict Thread
protected:
    std::thread* predictThread;
protected:
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



    // Publish Thread
protected:
    double publish_rate_val_; // in Hz
    // if > 0 -> publish at constant rate
    // if == 0 -> publish only when the buffer is updated

protected:
    std::thread* publish_thread_;
protected:
    virtual int publishThreadFunction();



    // New measurement notification thread
protected:
    std::thread* new_measurement_notification_thread_;
protected:
    int publishNewMeasurementNotificationThreadFunction();
protected:
    virtual int publishNewMeasurementNotification(const TimeStamp& measurement_time_stamp);



    // Start/Stop threads
public:
    int startThreads();
    int stopThreads(); // TODO FIX!






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
