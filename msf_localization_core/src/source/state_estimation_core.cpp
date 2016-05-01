
#include "msf_localization_core/state_estimation_core.h"


// To avoid circular dependencies
#include "msf_localization_core/global_parameters_state_core.h"

#include "msf_localization_core/robot_state_core.h"

#include "msf_localization_core/sensor_state_core.h"

#include "msf_localization_core/map_element_state_core.h"

#include "msf_localization_core/input_state_core.h"

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

    // TheListMeasurementCore
    this->TheListMeasurementCore.clear();

    // TheListInputCommandCore
    this->TheListInputCommandCore.clear();


    return;
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

bool StateEstimationCore::hasMeasurement() const
{
    if(this->TheListMeasurementCore.size()!=0)
        return true;
    else
        return false;
}

bool StateEstimationCore::hasInputCommand() const
{
    if(this->TheListInputCommandCore.size()!=0)
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
    for(std::list< std::shared_ptr<SensorStateCore> >::const_iterator itSensor=TheListSensorStateCore.begin();
        itSensor!=TheListSensorStateCore.end();
        ++itSensor)
    {
        dimension_state+=(*itSensor)->getMsfElementCoreSharedPtr()->getDimensionState();
    }

    // Inputs
    for(std::list< std::shared_ptr<InputStateCore> >::const_iterator itInput=TheListInputStateCore.begin();
        itInput!=TheListInputStateCore.end();
        ++itInput)
    {
        dimension_state+=(*itInput)->getMsfElementCoreSharedPtr()->getDimensionState();
    }

    // Map
    for(std::list< std::shared_ptr<MapElementStateCore> >::const_iterator itMapElement=TheListMapElementStateCore.begin();
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
    for(std::list< std::shared_ptr<SensorStateCore> >::const_iterator itSensor=TheListSensorStateCore.begin();
        itSensor!=TheListSensorStateCore.end();
        ++itSensor)
    {
        dimension_error_state+=(*itSensor)->getMsfElementCoreSharedPtr()->getDimensionErrorState();
    }

    // Inputs
    for(std::list< std::shared_ptr<InputStateCore> >::const_iterator itInput=TheListInputStateCore.begin();
        itInput!=TheListInputStateCore.end();
        ++itInput)
    {
        dimension_error_state+=(*itInput)->getMsfElementCoreSharedPtr()->getDimensionErrorState();
    }

    // Map
    for(std::list< std::shared_ptr<MapElementStateCore> >::const_iterator itMapElement=TheListMapElementStateCore.begin();
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
    for(std::list< std::shared_ptr<SensorStateCore> >::const_iterator itSensor=TheListSensorStateCore.begin();
        itSensor!=TheListSensorStateCore.end();
        ++itSensor)
    {
        dimension_parameters+=(*itSensor)->getMsfElementCoreSharedPtr()->getDimensionParameters();
    }

    // Inputs
    for(std::list< std::shared_ptr<InputStateCore> >::const_iterator itInput=TheListInputStateCore.begin();
        itInput!=TheListInputStateCore.end();
        ++itInput)
    {
        dimension_parameters+=(*itInput)->getMsfElementCoreSharedPtr()->getDimensionParameters();
    }

    // Map
    for(std::list< std::shared_ptr<MapElementStateCore> >::const_iterator itMapElement=TheListMapElementStateCore.begin();
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
    for(std::list< std::shared_ptr<SensorStateCore> >::const_iterator itSensor=TheListSensorStateCore.begin();
        itSensor!=TheListSensorStateCore.end();
        ++itSensor)
    {
        dimension_error_parameters+=(*itSensor)->getMsfElementCoreSharedPtr()->getDimensionErrorParameters();
    }

    // Inputs
    for(std::list< std::shared_ptr<InputStateCore> >::const_iterator itInput=TheListInputStateCore.begin();
        itInput!=TheListInputStateCore.end();
        ++itInput)
    {
        dimension_error_parameters+=(*itInput)->getMsfElementCoreSharedPtr()->getDimensionErrorParameters();
    }

    // Map
    for(std::list< std::shared_ptr<MapElementStateCore> >::const_iterator itMapElement=TheListMapElementStateCore.begin();
        itMapElement!=TheListMapElementStateCore.end();
        ++itMapElement)
    {
        dimension_error_parameters+=(*itMapElement)->getMsfElementCoreSharedPtr()->getDimensionErrorParameters();
    }


    // end
    return dimension_error_parameters;
}

int StateEstimationCore::getDimensionMeasurement() const
{
    int dimension_measurement=0;

    for(std::list< std::shared_ptr<SensorMeasurementCore> >::const_iterator itSensorMeas=TheListMeasurementCore.begin();
        itSensorMeas!=TheListMeasurementCore.end();
        ++itSensorMeas)
    {
        dimension_measurement+=(*itSensorMeas)->getSensorCoreSharedPtr()->getDimensionMeasurement();
    }

    return dimension_measurement;
}

int StateEstimationCore::getDimensionErrorMeasurement() const
{
    int dimension_error_measurement=0;

    for(std::list< std::shared_ptr<SensorMeasurementCore> >::const_iterator itSensorMeas=TheListMeasurementCore.begin();
        itSensorMeas!=TheListMeasurementCore.end();
        ++itSensorMeas)
    {
        dimension_error_measurement+=(*itSensorMeas)->getSensorCoreSharedPtr()->getDimensionErrorMeasurement();
    }

    return dimension_error_measurement;
}

int StateEstimationCore::getDimensionInputCommand() const
{
    int dimensionInputCommand=0;

    for(std::list< std::shared_ptr<InputCommandCore> >::const_iterator itInputCommand=TheListInputCommandCore.begin();
        itInputCommand!=TheListInputCommandCore.end();
        ++itInputCommand)
    {
        dimensionInputCommand+=(*itInputCommand)->getInputCoreSharedPtr()->getDimensionInputCommand();
    }

    return dimensionInputCommand;
}

int StateEstimationCore::getDimensionErrorInputCommand() const
{
    int dimensionErrorInputCommand=0;

    for(std::list< std::shared_ptr<InputCommandCore> >::const_iterator itInputCommand=TheListInputCommandCore.begin();
        itInputCommand!=TheListInputCommandCore.end();
        ++itInputCommand)
    {
        dimensionErrorInputCommand+=(*itInputCommand)->getInputCoreSharedPtr()->getDimensionErrorInputCommand();
    }

    return dimensionErrorInputCommand;
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
    for(std::list<std::shared_ptr<SensorStateCore> >::const_iterator itLisSensorState=TheListSensorStateCore.begin();
        itLisSensorState!=TheListSensorStateCore.end();
        ++itLisSensorState)
    {
        int dimensionSensor=(*itLisSensorState)->getMsfElementCoreSharedPtr()->getDimensionErrorState();
        covarianceMatrix->block(pointCovMatrix, pointCovMatrix, dimensionSensor, dimensionSensor)=(*itLisSensorState)->getMsfElementCoreSharedPtr()->getCovarianceInitErrorState();
        pointCovMatrix+=dimensionSensor;
    }

    // Input
    for(std::list<std::shared_ptr<InputStateCore> >::const_iterator itListInputState=TheListInputStateCore.begin();
        itListInputState!=TheListInputStateCore.end();
        ++itListInputState)
    {
        int dimensionInput=(*itListInputState)->getMsfElementCoreSharedPtr()->getDimensionErrorState();
        covarianceMatrix->block(pointCovMatrix, pointCovMatrix, dimensionInput, dimensionInput)=(*itListInputState)->getMsfElementCoreSharedPtr()->getCovarianceInitErrorState();
        pointCovMatrix+=dimensionInput;
    }


    // Map
    for(std::list<std::shared_ptr<MapElementStateCore> >::const_iterator itLisMapElementState=TheListMapElementStateCore.begin();
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
