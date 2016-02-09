

#ifndef _MSF_STORAGE_CORE_H
#define _MSF_STORAGE_CORE_H


//I/O stream
//std::cout
#include <iostream>

//String
//std::string, std::getline()
#include <string>




//Vector
//std::vector
#include <vector>

// List
#include <list>



// Thread
#include <thread>


#include <memory>


// Estimator Cores





#include "robot_state_core.h"
#include "sensor_state_core.h"

#include "sensor_measurement_core.h"
#include "imu_sensor_measurement_core.h"


#include "stamped_ring_buffer.h"

#include "state_estimation_core.h"




class MsfStorageCore : public StampedRingBuffer<StateEstimationCore>
{
public:
    MsfStorageCore();
    ~MsfStorageCore();

public:
    int setMeasurement(TimeStamp TheTimeStamp, std::shared_ptr<SensorMeasurementCore> TheSensorMeasurement);


};





#endif
