
#include "msf_localization_core/state_estimation_core.h"


// To avoid circular dependencies
#include "msf_localization_core/state_core.h"

#include "msf_localization_core/input_command_core.h"

#include "msf_localization_core/sensor_measurement_core.h"



StateEstimationCore::StateEstimationCore()
{
    if(!covarianceMatrix)
        covarianceMatrix=std::make_shared<Eigen::MatrixXd>();

    return;
}

StateEstimationCore::~StateEstimationCore()
{
    // Be tidy

    // TheListSensorStateCore
    this->TheListSensorStateCore.clear();

    // TheListInputStateCore
    this->TheListInputStateCore.clear();

    // TheListMapElementStateCore
    this->TheListMapElementStateCore.clear();

    // sensor_measurement_component_
    this->sensor_measurement_component_.reset();

    // input_command_component_
    this->input_command_component_.reset();


    return;
}

bool StateEstimationCore::checkState() const
{
    if(!TheGlobalParametersStateCore)
        return false;
    if(!TheRobotStateCore)
        return false;
    // The rest are optional!

    if(!covarianceMatrix)
        return false;

    return true;
}

bool StateEstimationCore::hasState() const
{
    // TODO check
    //if(TheListSensorStateCore.size()!=0 || TheRobotStateCore)
    if(TheRobotStateCore)
        return true;
    else
        return false;
}





int StateEstimationCore::getDimensionState() const
{
    int dimension_state=0;

    // Global Parameters
    dimension_state+=this->TheGlobalParametersStateCore->getMsfElementCoreSharedPtr()->getDimensionState();

    // Robot
    dimension_state+=this->TheRobotStateCore->getMsfElementCoreSharedPtr()->getDimensionState();

    // Sensors
    for(std::list< std::shared_ptr<StateCore> >::const_iterator itSensor=TheListSensorStateCore.begin();
        itSensor!=TheListSensorStateCore.end();
        ++itSensor)
    {
        dimension_state+=(*itSensor)->getMsfElementCoreSharedPtr()->getDimensionState();
    }

    // Inputs
    for(std::list< std::shared_ptr<StateCore> >::const_iterator itInput=TheListInputStateCore.begin();
        itInput!=TheListInputStateCore.end();
        ++itInput)
    {
        dimension_state+=(*itInput)->getMsfElementCoreSharedPtr()->getDimensionState();
    }

    // Map
    for(std::list< std::shared_ptr<StateCore> >::const_iterator itMapElement=TheListMapElementStateCore.begin();
        itMapElement!=TheListMapElementStateCore.end();
        ++itMapElement)
    {
        dimension_state+=(*itMapElement)->getMsfElementCoreSharedPtr()->getDimensionState();
    }


    // end
    return dimension_state;
}


int StateEstimationCore::getDimensionErrorState() const
{
    int dimension_error_state=0;

    // Global Parameters
    dimension_error_state+=this->TheGlobalParametersStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorState();

    // Robot
    dimension_error_state+=this->TheRobotStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorState();

    // Sensors
    for(std::list< std::shared_ptr<StateCore> >::const_iterator itSensor=TheListSensorStateCore.begin();
        itSensor!=TheListSensorStateCore.end();
        ++itSensor)
    {
        dimension_error_state+=(*itSensor)->getMsfElementCoreSharedPtr()->getDimensionErrorState();
    }

    // Inputs
    for(std::list< std::shared_ptr<StateCore> >::const_iterator itInput=TheListInputStateCore.begin();
        itInput!=TheListInputStateCore.end();
        ++itInput)
    {
        dimension_error_state+=(*itInput)->getMsfElementCoreSharedPtr()->getDimensionErrorState();
    }

    // Map
    for(std::list< std::shared_ptr<StateCore> >::const_iterator itMapElement=TheListMapElementStateCore.begin();
        itMapElement!=TheListMapElementStateCore.end();
        ++itMapElement)
    {
        dimension_error_state+=(*itMapElement)->getMsfElementCoreSharedPtr()->getDimensionErrorState();
    }


    // end
    return dimension_error_state;
}

int StateEstimationCore::getDimensionParameters() const
{
    int dimension_parameters=0;

    // Global Parameters
    dimension_parameters+=this->TheGlobalParametersStateCore->getMsfElementCoreSharedPtr()->getDimensionParameters();

    // Robot
    dimension_parameters+=this->TheRobotStateCore->getMsfElementCoreSharedPtr()->getDimensionParameters();

    // Sensors
    for(std::list< std::shared_ptr<StateCore> >::const_iterator itSensor=TheListSensorStateCore.begin();
        itSensor!=TheListSensorStateCore.end();
        ++itSensor)
    {
        dimension_parameters+=(*itSensor)->getMsfElementCoreSharedPtr()->getDimensionParameters();
    }

    // Inputs
    for(std::list< std::shared_ptr<StateCore> >::const_iterator itInput=TheListInputStateCore.begin();
        itInput!=TheListInputStateCore.end();
        ++itInput)
    {
        dimension_parameters+=(*itInput)->getMsfElementCoreSharedPtr()->getDimensionParameters();
    }

    // Map
    for(std::list< std::shared_ptr<StateCore> >::const_iterator itMapElement=TheListMapElementStateCore.begin();
        itMapElement!=TheListMapElementStateCore.end();
        ++itMapElement)
    {
        dimension_parameters+=(*itMapElement)->getMsfElementCoreSharedPtr()->getDimensionParameters();
    }


    // end
    return dimension_parameters;
}

int StateEstimationCore::getDimensionErrorParameters() const
{
    int dimension_error_parameters=0;

    // Global Parameters
    dimension_error_parameters+=this->TheGlobalParametersStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorParameters();

    // Robot
    dimension_error_parameters+=this->TheRobotStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorParameters();

    // Sensors
    for(std::list< std::shared_ptr<StateCore> >::const_iterator itSensor=TheListSensorStateCore.begin();
        itSensor!=TheListSensorStateCore.end();
        ++itSensor)
    {
        dimension_error_parameters+=(*itSensor)->getMsfElementCoreSharedPtr()->getDimensionErrorParameters();
    }

    // Inputs
    for(std::list< std::shared_ptr<StateCore> >::const_iterator itInput=TheListInputStateCore.begin();
        itInput!=TheListInputStateCore.end();
        ++itInput)
    {
        dimension_error_parameters+=(*itInput)->getMsfElementCoreSharedPtr()->getDimensionErrorParameters();
    }

    // Map
    for(std::list< std::shared_ptr<StateCore> >::const_iterator itMapElement=TheListMapElementStateCore.begin();
        itMapElement!=TheListMapElementStateCore.end();
        ++itMapElement)
    {
        dimension_error_parameters+=(*itMapElement)->getMsfElementCoreSharedPtr()->getDimensionErrorParameters();
    }


    // end
    return dimension_error_parameters;
}

int StateEstimationCore::getNumberInputStates() const
{
    return this->TheListInputStateCore.size();
}

int StateEstimationCore::getNumberSensorStates() const
{
    return this->TheListSensorStateCore.size();
}

int StateEstimationCore::getNumberMapElementStates() const
{
    return this->TheListMapElementStateCore.size();
}

int StateEstimationCore::prepareCovarianceInitErrorState()
{
    //std::cout<<"StateEstimationCore::prepareCovarianceInitErrorState()"<<std::endl;

    // Dimension
    int dimension_error_state=this->getDimensionErrorState();

    // Init
    covarianceMatrix->resize(dimension_error_state, dimension_error_state);
    covarianceMatrix->setZero();


    // Add matrix
    int pointCovMatrix=0;


    // Global Parameters
    int dimensionWorld=this->TheGlobalParametersStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorState();
    covarianceMatrix->block(pointCovMatrix, pointCovMatrix, dimensionWorld, dimensionWorld)=this->TheGlobalParametersStateCore->getMsfElementCoreSharedPtr()->getCovarianceInitErrorState();
    pointCovMatrix+=dimensionWorld;


    // Robot
    int dimensionRobot=this->TheRobotStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorState();
    covarianceMatrix->block(pointCovMatrix, pointCovMatrix, dimensionRobot, dimensionRobot)=this->TheRobotStateCore->getMsfElementCoreSharedPtr()->getCovarianceInitErrorState();
    pointCovMatrix+=dimensionRobot;


    // Sensors
    for(std::list<std::shared_ptr<StateCore> >::const_iterator itLisSensorState=TheListSensorStateCore.begin();
        itLisSensorState!=TheListSensorStateCore.end();
        ++itLisSensorState)
    {
        int dimensionSensor=(*itLisSensorState)->getMsfElementCoreSharedPtr()->getDimensionErrorState();
        covarianceMatrix->block(pointCovMatrix, pointCovMatrix, dimensionSensor, dimensionSensor)=(*itLisSensorState)->getMsfElementCoreSharedPtr()->getCovarianceInitErrorState();
        pointCovMatrix+=dimensionSensor;
    }

    // Input
    for(std::list<std::shared_ptr<StateCore> >::const_iterator itListInputState=TheListInputStateCore.begin();
        itListInputState!=TheListInputStateCore.end();
        ++itListInputState)
    {
        int dimensionInput=(*itListInputState)->getMsfElementCoreSharedPtr()->getDimensionErrorState();
        covarianceMatrix->block(pointCovMatrix, pointCovMatrix, dimensionInput, dimensionInput)=(*itListInputState)->getMsfElementCoreSharedPtr()->getCovarianceInitErrorState();
        pointCovMatrix+=dimensionInput;
    }


    // Map
    for(std::list<std::shared_ptr<StateCore> >::const_iterator itLisMapElementState=TheListMapElementStateCore.begin();
        itLisMapElementState!=TheListMapElementStateCore.end();
        ++itLisMapElementState)
    {
        int dimensionMapElement=(*itLisMapElementState)->getMsfElementCoreSharedPtr()->getDimensionErrorState();
        covarianceMatrix->block(pointCovMatrix, pointCovMatrix, dimensionMapElement, dimensionMapElement)=(*itLisMapElementState)->getMsfElementCoreSharedPtr()->getCovarianceInitErrorState();
        pointCovMatrix+=dimensionMapElement;
    }



    // End
    return 0;
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
