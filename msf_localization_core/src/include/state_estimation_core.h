

#ifndef _STATE_ESTIMATION_CORE_H
#define _STATE_ESTIMATION_CORE_H


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




#include <memory>



#include "robot_state_core.h"

#include "sensor_state_core.h"

#include "sensor_measurement_core.h"




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






#endif
