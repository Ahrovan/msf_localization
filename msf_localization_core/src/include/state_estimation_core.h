

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



    /// State
public:
    bool hasState() const;


    // Robot State
public:
    std::shared_ptr<RobotStateCore> TheRobotStateCore;

    // Sensors State
public:
    std::list<std::shared_ptr<SensorStateCore> > TheListSensorStateCore;


    // Covariances Matrixes
public:
    Eigen::MatrixXd covarianceMatrix;




    /// Measurements

public:
    bool hasMeasurement() const;


    // Available Measurements
public:
    std::list<std::shared_ptr<SensorMeasurementCore> > TheListMeasurementCore;





    /// Others
    // Kalman Gain
    // TODO?


};






#endif
