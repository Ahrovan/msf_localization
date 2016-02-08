

#ifndef _STATE_ESTIMATION_CORE_H
#define _STATE_ESTIMATION_CORE_H


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


//PUGIXML
#include "pugixml.hpp"


// Boost
#include <boost/filesystem.hpp>

// Thread
#include <thread>


#include <memory>


// Estimator Cores

// Robot
//#include "robot_core.h"

// Sensor
//#include "sensor_core.h"
// IMU
//#include "imu_sensor_core.h"





#include "robot_state_core.h"
#include "sensor_state_core.h"
#include "sensor_measurement_core.h"


#include "stamped_ring_buffer.h"



// Class that stores the information for every time instant
class StateEstimationCore
{
public:
    StateEstimationCore();
    ~StateEstimationCore();



public:
    // Robot State
    bool flagHasRobotState;
    RobotStateCore TheRobotStateCore;



public:
    // Sensors State
    bool flagHasSensorState;
    std::list<std::shared_ptr<SensorStateCore> > TheListSensorStateCore;


public:
    // Covariances Matrixes




public:
    // Available Measurements
    bool flagHasMeasurement;
    std::list<std::shared_ptr<SensorMeasurementCore> > TheListMeasurementCore;


};



class MsfStorageCore : public StampedRingBuffer<StateEstimationCore>
{
public:
    MsfStorageCore();
    ~MsfStorageCore();

public:
    int setMeasurement(TimeStamp TheTimeStamp, std::shared_ptr<SensorMeasurementCore> TheSensorMeasurement);


};





#endif
