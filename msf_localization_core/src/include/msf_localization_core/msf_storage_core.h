

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

// Chrono
#include <chrono>


#include <boost/filesystem.hpp>


#include "buffer/stamped_ring_buffer.h"

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
#include "msf_localization_core/sensor_measurement_component.h"
#include "msf_localization_core/sensor_measurement_core.h"
// Imu
//#include "msf_localization_core/imu_sensor_measurement_core.h"


/// Input Command
#include "msf_localization_core/input_command_core.h"
//#include "msf_localization_core/imu_input_command_core.h"






#define _DEBUG_MSF_STORAGE 0

#define _DEBUG_MSF_STORAGE_BUFFER_TIME 1




class MsfStorageCore : protected StampedRingBuffer< std::shared_ptr<StateEstimationCore> >
{
public:
    MsfStorageCore();
    ~MsfStorageCore();

    // mutex to protect the buffer
protected:
private:
    std::timed_mutex buffer_mutex_;
    //std::unique_lock<std::timed_mutex>* lock_buffer_mutex_;
    //std::recursive_mutex TheRingBufferMutex;
protected:
    // timeout for the mutex of the buffer in us.
    // Not working with C++11 unless you have a real-time OS.
    // std::timed_mutex uses a steady_clock (precision ~ms):
    // http://en.cppreference.com/w/cpp/thread/timed_mutex/try_lock_for
    int buffer_mutex_timeout_us_; //500;


    // Get number elements in the buffer (safe)
public:
    int getNumElements();

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

    // Get previous timestamp (safe)
public:
    int getPreviousTimeStamp(const TimeStamp& currentTimeStamp, TimeStamp& previousTimeStamp);

    // Get the previous input by stamp using the input core (safe)
public:
    int getPreviousInputCommandByStampAndInputCore(const TimeStamp& time_stamp,
                                                   const std::shared_ptr<InputCore>& input_core,
                                                   std::shared_ptr<InputCommandCore>& input_command_core);

    // Get oldest time stamp (safe)
public:
    int getOldestTimeStamp(TimeStamp& oldest_time_stamp);


    // Add element in the ring buffer by stamp (safe)
public:
    int addElement(const TimeStamp& TheTimeStamp, const std::shared_ptr<StateEstimationCore>& TheStateEstimationCore, bool flag_wait_before_overwrite=true);


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
public:
    int displayStateEstimationElement(const TimeStamp& TheTimeStamp,
                                      const std::shared_ptr<StateEstimationCore>&  TheStateEstimationCore);






    //// Outdated elements buffer
    ///
    // List with the timestamp of the outdated elements of the buffer
protected:
    std::list<TimeStamp> outdatedBufferElements;
public:
    // (safe)
    int addOutdatedElement(const TimeStamp& TheTimeStamp);
    // (safe)
    int getOldestOutdatedElement(TimeStamp &TheOutdatedTimeStamp, bool sleep_if_empty=true);
public:
    // get number of outdated elements (safe)
    int getNumOutdatedElements();
    // (safe)
    int displayOutdatedBufferElements();
    // (safe)
    std::string getDisplayOutdatedElements();
    // Protector mutex
protected:
private:
    //std::recursive_timed_mutex outdated_buffer_elements_protector_mutex_;
    std::timed_mutex outdated_buffer_elements_protector_mutex_;
    // timeout for the mutex of the buffer in us
    int outdated_elements_buffer_mutex_timeout_us_;

    // Buffer waiting
protected:
private:
    std::mutex outdatedBufferElementsMutex;             // mutex for critical section
    std::condition_variable outdatedBufferElementsConditionVariable; // condition variable for critical section
    //std::unique_lock<std::mutex>* outdatedBufferElementsLock;

    // Buffer updated: Async publishing
public:
    int semaphoreBufferUpdated();
protected:
private:
    std::mutex updated_buffer_mutex_;             // mutex for critical section
    std::condition_variable updated_buffer_condition_variable_; // condition variable for critical section
    //std::unique_lock<std::mutex>* updated_buffer_lock_;



    //// Debug log
protected:
    std::string logPath;
    std::ofstream logFile;
    // mutex to protect the log file
protected:
private:
    std::timed_mutex TheLogFileMutex;
public:
    int log(std::string logString);

};





#endif
