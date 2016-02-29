

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



// Estimator Cores

// Robot
#include "robot_state_core.h"
#include "free_model_robot_core.h"
#include "free_model_robot_state_core.h"


// Sensor
#include "sensor_state_core.h"
#include "sensor_measurement_core.h"

// Imu
#include "imu_sensor_core.h"
#include "imu_sensor_measurement_core.h"
#include "imu_sensor_state_core.h"


#include "stamped_ring_buffer.h"

#include "state_estimation_core.h"




class MsfStorageCore : protected StampedRingBuffer< std::shared_ptr<StateEstimationCore> >
{
public:
    MsfStorageCore();
    ~MsfStorageCore();

    // mutex to protect the buffer
protected:
    std::mutex TheRingBufferMutex;


    // Set a measurement in the ring buffer with given time stamp (safe)
public:
    int setMeasurement(const TimeStamp TheTimeStamp, const std::shared_ptr<SensorMeasurementCore> TheSensorMeasurement);

    // Get the last element in the ring buffer which has a state estimate (safe)
    // Unuseful for the predict -> We need the previous to a defined timestamp!
public:
    int getLastElementWithStateEstimate(TimeStamp& TheTimeStamp, std::shared_ptr<StateEstimationCore>& PreviousState);

    // Get the previous element of a timestamp in the ring buffer which has a state estimate (safe)
public:
    int getPreviousElementWithStateEstimateByStamp(TimeStamp ThePreviousTimeStamp, TimeStamp& TheTimeStamp, std::shared_ptr<StateEstimationCore>& PreviousState);


    // Get element in the ring buffer (safe)
public:
    int getElement(const TimeStamp timeStamp, std::shared_ptr<StateEstimationCore>& TheElement);

    // Get next timestap (safe)
public:
    int getNextTimeStamp(const TimeStamp previousTimeStamp, TimeStamp& nextTimeStamp);


    // Add element in the ring buffer by stamp (safe)
public:
    int addElement(const TimeStamp TheTimeStamp, const std::shared_ptr<StateEstimationCore> TheStateEstimationCore);

    // Purge Ring Buffer (safe)
public:
    int purgeRingBuffer(int numElementsFrom);



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
protected:
    std::mutex outdatedBufferElementsMutex;             // mutex for critical section
    std::condition_variable outdatedBufferElementsConditionVariable; // condition variable for critical section
    std::unique_lock<std::mutex>* outdatedBufferElementsLock;



    //// Debug log
protected:
    std::string logPath;
    std::ofstream logFile;


};





#endif
