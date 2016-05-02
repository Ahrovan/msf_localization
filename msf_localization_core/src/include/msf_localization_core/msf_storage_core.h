

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
#include "msf_localization_core/free_model_robot_core.h"
// Global Parameters
#include "msf_localization_core/global_parameters_core.h"
// Sensor
#include "msf_localization_core/imu_sensor_core.h"
// Input
#include "msf_localization_core/input_core.h"
#include "msf_localization_core/imu_input_core.h"
// Map


/// State
// Robot
#include "msf_localization_core/robot_state_core.h"
#include "msf_localization_core/free_model_robot_state_core.h"
// Global Parameters

// Sensor
#include "msf_localization_core/sensor_state_core.h"
#include "msf_localization_core/imu_sensor_state_core.h"
// Input

// Map


/// Measurement
#include "msf_localization_core/sensor_measurement_core.h"
// Imu
#include "msf_localization_core/imu_sensor_measurement_core.h"


/// Input Command
#include "msf_localization_core/input_command_core.h"
#include "msf_localization_core/imu_input_command_core.h"






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
    int getElement(const TimeStamp timeStamp, std::shared_ptr<StateEstimationCore>& TheElement);

    // Get the last element in the ring buffer which has a state estimate (safe)
    // Unuseful for the predict -> We need the previous to a defined timestamp!
public:
    int getLastElementWithStateEstimate(TimeStamp& TheTimeStamp, std::shared_ptr<StateEstimationCore>& PreviousState);

    // Get the previous element of a timestamp in the ring buffer which has a state estimate (safe)
public:
    int getPreviousElementWithStateEstimateByStamp(TimeStamp ThePreviousTimeStamp, TimeStamp& TheTimeStamp, std::shared_ptr<StateEstimationCore>& PreviousState);

    // Get next timestap (safe)
public:
    int getNextTimeStamp(const TimeStamp previousTimeStamp, TimeStamp& nextTimeStamp);

    // Get the previous input by stamp using the input core
public:
    int getPreviousInputCommandByStampAndInputCore(const TimeStamp time_stamp, const std::shared_ptr<InputCore> input_core, std::shared_ptr<InputCommandCore>& input_command_core);



    // Add element in the ring buffer by stamp (safe)
public:
    int addElement(const TimeStamp TheTimeStamp, const std::shared_ptr<StateEstimationCore> TheStateEstimationCore);

    // Set a measurement in the ring buffer with given time stamp (safe)
public:
    int setMeasurement(const TimeStamp TheTimeStamp, const std::shared_ptr<SensorMeasurementCore> TheSensorMeasurement);
    int setMeasurementList(const TimeStamp TheTimeStamp, const std::list< std::shared_ptr<SensorMeasurementCore> > TheListSensorMeasurement);


    // Set an input command in the ring buffer with given time stamp (safe)
public:
    int setInputCommand(const TimeStamp time_stamp, const std::shared_ptr<InputCommandCore> input_command_core);
    int setInputCommandList(const TimeStamp time_stamp, const std::list< std::shared_ptr<InputCommandCore> > list_input_command_core);



    // Purge Ring Buffer (safe)
public:
    int purgeRingBuffer(int numElementsFrom);

    int purgeElementRingBuffer(const TimeStamp TheTimeStamp);



    // Display Elements in the ring buffer (safe)
public:
    int displayRingBuffer();
    int displayStateEstimationElement(const TimeStamp TheTimeStamp, const std::shared_ptr<StateEstimationCore>  TheStateEstimationCore);




    // List with the timestamp of the outdated elements of the buffer
protected:
    std::list<TimeStamp> outdatedBufferElements;
public:
    int addOutdatedElement(TimeStamp TheTimeStamp);
    int getOldestOutdatedElement(TimeStamp &TheOutdatedTimeStamp);
public:
    int displayOutdatedBufferElements();
    std::string getDisplayOutdatedElements();
protected:
    std::mutex outdatedBufferElementsMutex;             // mutex for critical section
    std::condition_variable outdatedBufferElementsConditionVariable; // condition variable for critical section
    std::unique_lock<std::mutex>* outdatedBufferElementsLock;



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
