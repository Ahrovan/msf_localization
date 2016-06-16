

#ifndef _MSF_STORAGE_CORE_H
#define _MSF_STORAGE_CORE_H


//I/O stream
//std::cout
#include <iostream>

#include <fstream>

//String
//std::string, std::getline()
#include <string>

// Environment variable
#include <cstdlib>


//Vector
//std::vector
#include <vector>

// List
#include <list>



// Thread
#include <thread>


#include <memory>


// Mutex
#include <mutex>

// Condition variable
#include <condition_variable>


#include "msf_localization_core/stamped_ring_buffer.h"

#include "msf_localization_core/state_estimation_core.h"



/// Estimator Cores
// Robot
//#include "msf_localization_core/free_model_robot_core.h"
// Global Parameters
//#include "msf_localization_core/global_parameters_core.h"
// Sensor
//#include "msf_localization_core/imu_sensor_core.h"
// Input
//#include "msf_localization_core/input_core.h"
//#include "msf_localization_core/imu_input_core.h"
// Map
//#include "msf_localization_core/map_element_core.h"


/// State
#include "msf_localization_core/state_core.h"
// Global Parameters

// Robot
//#include "msf_localization_core/robot_state_core.h"
//#include "msf_localization_core/free_model_robot_state_core.h"

// Sensor
//#include "msf_localization_core/sensor_state_core.h"
//#include "msf_localization_core/imu_sensor_state_core.h"

// Input

// Map
//#include "msf_localization_core/map_element_state_core.h"


/// Measurement
#include "msf_localization_core/sensor_measurement_core.h"
// Imu
//#include "msf_localization_core/imu_sensor_measurement_core.h"


/// Input Command
#include "msf_localization_core/input_command_core.h"
//#include "msf_localization_core/imu_input_command_core.h"






#define _DEBUG_MSF_STORAGE 0



class MsfStorageCore : protected StampedRingBuffer< std::shared_ptr<StateEstimationCore> >
{
public:
    MsfStorageCore();
    ~MsfStorageCore();

    // mutex to protect the buffer
protected:
    std::mutex TheRingBufferMutex;



    // Get element in the ring buffer (safe)
public:
    int getElement(const TimeStamp& timeStamp, std::shared_ptr<StateEstimationCore>& TheElement);

    // Get the last element in the ring buffer which has a state estimate (safe)
    // Unuseful for the predict -> We need the previous to a defined timestamp!
public:
    int getLastElementWithStateEstimate(TimeStamp& TheTimeStamp, std::shared_ptr<StateEstimationCore>& PreviousState);

    // Get the element of a timestamp in the ring buffer which has a state estimate (safe)
public:
    int getElementWithStateEstimateByStamp(const TimeStamp& ThePreviousTimeStamp, TimeStamp& TheTimeStamp, std::shared_ptr<StateEstimationCore>& PreviousState);

    // Get the previous element of a timestamp in the ring buffer which has a state estimate (safe)
public:
    int getPreviousElementWithStateEstimateByStamp(const TimeStamp& ThePreviousTimeStamp, TimeStamp& TheTimeStamp, std::shared_ptr<StateEstimationCore>& PreviousState);

    // Get next timestap (safe)
public:
    int getNextTimeStamp(const TimeStamp& currentTimeStamp, TimeStamp& nextTimeStamp);

    // Get previous timestamp
public:
    int getPreviousTimeStamp(const TimeStamp& currentTimeStamp, TimeStamp& previousTimeStamp);

    // Get the previous input by stamp using the input core
public:
    int getPreviousInputCommandByStampAndInputCore(const TimeStamp& time_stamp,
                                                   const std::shared_ptr<InputCore>& input_core,
                                                   std::shared_ptr<InputCommandCore>& input_command_core);

    // Get oldest time stamp
public:
    int getOldestTimeStamp(TimeStamp& oldest_time_stamp);


    // Add element in the ring buffer by stamp (safe)
public:
    int addElement(const TimeStamp& TheTimeStamp, const std::shared_ptr<StateEstimationCore>& TheStateEstimationCore);


    /// Measurements

    // New Measurement Set
    // Set a measurement in the ring buffer with given time stamp (safe)
public:
    int setMeasurement(const TimeStamp& TheTimeStamp,
                       const std::shared_ptr<SensorMeasurementCore>& TheSensorMeasurement);
    int setMeasurementList(const TimeStamp& TheTimeStamp,
                           const std::list< std::shared_ptr<SensorMeasurementCore> >& TheListSensorMeasurement);





    // New measurement Flag
    // TODO posible race problem!
protected:
    bool flag_new_measurement_;



    /// Input Commands
    // Set an input command in the ring buffer with given time stamp (safe)
public:
    int setInputCommand(const TimeStamp& time_stamp,
                        const std::shared_ptr<InputCommandCore>& input_command_core);
    int setInputCommandList(const TimeStamp& time_stamp,
                            const std::list< std::shared_ptr<InputCommandCore> >& list_input_command_core);



    /// Purge buffer
    // Purge Ring Buffer (safe)
public:
    int purgeRingBuffer(int numElementsFrom);

    int purgeElementRingBuffer(const TimeStamp& TheTimeStamp);



    /// Display buffer
    // Display Elements in the ring buffer (safe)
public:
    int displayRingBuffer();
    int displayStateEstimationElement(const TimeStamp& TheTimeStamp,
                                      const std::shared_ptr<StateEstimationCore>&  TheStateEstimationCore);







    // List with the timestamp of the outdated elements of the buffer
protected:
    std::list<TimeStamp> outdatedBufferElements;
public:
    int addOutdatedElement(const TimeStamp& TheTimeStamp);
    int getOldestOutdatedElement(TimeStamp &TheOutdatedTimeStamp, bool sleep_if_empty=true);
public:
    int displayOutdatedBufferElements();
    std::string getDisplayOutdatedElements();
    // Buffer waiting
protected:
    std::mutex outdatedBufferElementsMutex;             // mutex for critical section
    std::condition_variable outdatedBufferElementsConditionVariable; // condition variable for critical section
    std::unique_lock<std::mutex>* outdatedBufferElementsLock;

    // Buffer updated: Async publishing
public:
    int semaphoreBufferUpdated();
protected:
    std::mutex updated_buffer_mutex_;             // mutex for critical section
    std::condition_variable updated_buffer_condition_variable_; // condition variable for critical section
    std::unique_lock<std::mutex>* updated_buffer_lock_;



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
