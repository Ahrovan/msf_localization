
#include "msf_localization_core/state_estimation_core.h"


// To avoid circular dependencies
#include "msf_localization_core/msf_element_core.h"
#include "msf_localization_core/state_core.h"



StateEstimationCore::StateEstimationCore()
{

    return;
}

StateEstimationCore::~StateEstimationCore()
{
    // Be tidy

    // state_component_
    this->state_component_.reset();

    // sensor_measurement_component_
    this->sensor_measurement_component_.reset();

    // input_command_component_
    this->input_command_component_.reset();


    return;
}

bool StateEstimationCore::hasState() const
{
    if(!this->state_component_)
        return false;
    if(!this->state_component_->hasState())
        return false;
    return true;
}

bool StateEstimationCore::hasInputCommand() const
{
    if(!this->input_command_component_)
        return false;
    if(!this->input_command_component_->hasInputCommand())
        return false;
    return true;
}

bool StateEstimationCore::hasMeasurement()
{
    if(!this->sensor_measurement_component_)
        return false;
    if(!this->sensor_measurement_component_->hasMeasurement())
        return false;
    return true;
}
