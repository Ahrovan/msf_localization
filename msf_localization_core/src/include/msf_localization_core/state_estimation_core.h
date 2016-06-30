

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


#include "msf_localization_core/state_component.h"
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

public:
    bool hasState() const;

public:
    std::shared_ptr<StateComponent> state_component_;



    /// Input Commands

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
