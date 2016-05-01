

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

#include "msf_localization_core/map_element_state_core.h"

#include "msf_localization_core/input_state_core.h"

#include "msf_localization_core/input_command_core.h"

#include "msf_localization_core/sensor_measurement_core.h"




// Class that stores the information for every time instant
class StateEstimationCore
{
public:
    StateEstimationCore();
    ~StateEstimationCore();



    /// State / Parameters

    // Check
public:
    bool hasState() const;

    // Dimension total of state and error state
public:
    int getDimensionState() const;
    int getDimensionErrorState() const;


    // Dimension total of parameters and error parameters
public:
    int getDimensionParameters() const;
    int getDimensionErrorParameters() const;


    // Global Parameters (World) State
public:
    std::shared_ptr<GlobalParametersStateCore> TheGlobalParametersStateCore;

    // Robot State
public:
    std::shared_ptr<RobotStateCore> TheRobotStateCore;

    // Inputs State
public:
    std::list< std::shared_ptr<InputStateCore> > TheListInputStateCore;

    // Sensors State
public:
    std::list< std::shared_ptr<SensorStateCore> > TheListSensorStateCore;

    // Map State
public:
    std::list< std::shared_ptr<MapElementStateCore> > TheListMapElementStateCore;



    // Covariances Matrixes
public:
    int prepareCovarianceInitErrorState();


    // Covariances Matrixes of the error state
public:
    std::shared_ptr<Eigen::MatrixXd> covarianceMatrix;



    /// Input Commands

    // Check
public:
    bool hasInputCommand() const;


    // Dimension total of input and error input
public:
    int getDimensionInputCommand() const;
    int getDimensionErrorInputCommand() const;


    // Avaliable Inputs
public:
    std::list< std::shared_ptr<InputCommandCore> > TheListInputCommandCore;



    /// Measurements

    // Check
public:
    bool hasMeasurement() const;


    // Dimension total of measurement and error measurement
public:
    int getDimensionMeasurement() const;
    int getDimensionErrorMeasurement() const;


    // Available Measurements
public:
    std::list<std::shared_ptr<SensorMeasurementCore> > TheListMeasurementCore;








};






#endif
