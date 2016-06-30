

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


#include <Eigen/Dense>


#include "msf_localization_core/sensor_measurement_component.h"
#include "msf_localization_core/input_command_component.h"



/// Forward declarations

// States
class StateCore;



// Class that stores the information for every time instant
class StateEstimationCore
{
public:
    StateEstimationCore();
    ~StateEstimationCore();



    /// State / Parameters

    // Check
public:
    bool checkState() const;
    bool hasState() const;

    // Dimension total of state and error state
public:
    int getDimensionState() const;
    int getDimensionErrorState() const;


    // Dimension total of parameters and error parameters
public:
    int getDimensionParameters() const;
    int getDimensionErrorParameters() const;


    // World (Global Parameters) State
public:
    std::shared_ptr<StateCore> TheGlobalParametersStateCore;

    // Robot State
public:
    std::shared_ptr<StateCore> TheRobotStateCore;

    // Inputs State
public:
    std::list< std::shared_ptr<StateCore> > TheListInputStateCore;
    int getNumberInputStates() const;

    // Sensors State
public:
    std::list< std::shared_ptr<StateCore> > TheListSensorStateCore;
    int getNumberSensorStates() const;

    // Map State
public:
    std::list< std::shared_ptr<StateCore> > TheListMapElementStateCore;
    int getNumberMapElementStates() const;



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

public:
    std::shared_ptr<InputCommandComponent> input_command_component_;



    /// Measurements

public:
    bool hasMeasurement();

public:
    std::shared_ptr<SensorMeasurementComponent> sensor_measurement_component_;








};






#endif
