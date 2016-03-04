

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


#include "msf_localization_core/global_parameters_state_core.h"

#include "msf_localization_core/robot_state_core.h"

#include "msf_localization_core/sensor_state_core.h"

#include "msf_localization_core/sensor_measurement_core.h"




// Class that stores the information for every time instant
class StateEstimationCore
{
public:
    StateEstimationCore();
    ~StateEstimationCore();



    /// State
public:
    bool hasState() const;


    // Global Parameters State
public:
    std::shared_ptr<GlobalParametersStateCore> TheGlobalParametersStateCore;

    // Robot State
public:
    std::shared_ptr<RobotStateCore> TheRobotStateCore;

    // Sensors State
public:
    std::list<std::shared_ptr<SensorStateCore> > TheListSensorStateCore;






    // Dimension total of state and error state
public:
    int getDimensionState() const;
    int getDimensionErrorState() const;


    // Covariances Matrixes
public:
    int prepareInitErrorStateVariance();


    // Covariances Matrixes of the error state
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
