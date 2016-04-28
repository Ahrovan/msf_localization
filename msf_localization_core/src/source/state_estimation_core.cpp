
#include "msf_localization_core/state_estimation_core.h"





StateEstimationCore::StateEstimationCore()
{
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
    int dimensionState=0;

    // Global Parameters
    dimensionState+=this->TheGlobalParametersStateCore->getMsfElementCoreSharedPtr()->getDimensionState();

    // Robot
    dimensionState+=this->TheRobotStateCore->getMsfElementCoreSharedPtr()->getDimensionState();

    // Sensors
    for(std::list< std::shared_ptr<SensorStateCore> >::const_iterator itSensor=TheListSensorStateCore.begin();
        itSensor!=TheListSensorStateCore.end();
        ++itSensor)
    {
        dimensionState+=(*itSensor)->getMsfElementCoreSharedPtr()->getDimensionState();
    }

    // Inputs
    for(std::list< std::shared_ptr<InputStateCore> >::const_iterator itInput=TheListInputStateCore.begin();
        itInput!=TheListInputStateCore.end();
        ++itInput)
    {
        dimensionState+=(*itInput)->getMsfElementCoreSharedPtr()->getDimensionState();
    }

    // Map
    for(std::list< std::shared_ptr<MapElementStateCore> >::const_iterator itMapElement=TheListMapElementStateCore.begin();
        itMapElement!=TheListMapElementStateCore.end();
        ++itMapElement)
    {
        dimensionState+=(*itMapElement)->getMsfElementCoreSharedPtr()->getDimensionState();
    }


    // end
    return dimensionState;
}


int StateEstimationCore::getDimensionErrorState() const
{
    int dimensionErrorState=0;

    // Global Parameters
    dimensionErrorState+=this->TheGlobalParametersStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorState();

    // Robot
    dimensionErrorState+=this->TheRobotStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorState();

    // Sensors
    for(std::list< std::shared_ptr<SensorStateCore> >::const_iterator itSensor=TheListSensorStateCore.begin();
        itSensor!=TheListSensorStateCore.end();
        ++itSensor)
    {
        dimensionErrorState+=(*itSensor)->getMsfElementCoreSharedPtr()->getDimensionErrorState();
    }

    // Inputs
    for(std::list< std::shared_ptr<InputStateCore> >::const_iterator itInput=TheListInputStateCore.begin();
        itInput!=TheListInputStateCore.end();
        ++itInput)
    {
        dimensionErrorState+=(*itInput)->getMsfElementCoreSharedPtr()->getDimensionErrorState();
    }

    // Map
    for(std::list< std::shared_ptr<MapElementStateCore> >::const_iterator itMapElement=TheListMapElementStateCore.begin();
        itMapElement!=TheListMapElementStateCore.end();
        ++itMapElement)
    {
        dimensionErrorState+=(*itMapElement)->getMsfElementCoreSharedPtr()->getDimensionErrorState();
    }


    // end
    return dimensionErrorState;
}


int StateEstimationCore::getDimensionMeasurement() const
{
    int dimensionMeasurement=0;

    for(std::list< std::shared_ptr<SensorMeasurementCore> >::const_iterator itSensorMeas=TheListMeasurementCore.begin();
        itSensorMeas!=TheListMeasurementCore.end();
        ++itSensorMeas)
    {
        dimensionMeasurement+=(*itSensorMeas)->getSensorCoreSharedPtr()->getDimensionMeasurement();
    }

    return dimensionMeasurement;
}

int StateEstimationCore::getDimensionErrorMeasurement() const
{
    int dimensionErrorMeasurement=0;

    for(std::list< std::shared_ptr<SensorMeasurementCore> >::const_iterator itSensorMeas=TheListMeasurementCore.begin();
        itSensorMeas!=TheListMeasurementCore.end();
        ++itSensorMeas)
    {
        dimensionErrorMeasurement+=(*itSensorMeas)->getSensorCoreSharedPtr()->getDimensionErrorMeasurement();
    }

    return dimensionErrorMeasurement;
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
    //std::cout<<"StateEstimationCore::prepareInitErrorStateVariance()"<<std::endl;

    // Dimension
    int dimensionErrorState=this->getDimensionErrorState();

    // Init
    covarianceMatrix.resize(dimensionErrorState, dimensionErrorState);
    covarianceMatrix.setZero();


    // Add matrix
    int pointCovMatrix=0;


    // Global Parameters
    int dimensionWorld=this->TheGlobalParametersStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorState();
    covarianceMatrix.block(pointCovMatrix, pointCovMatrix, dimensionWorld, dimensionWorld)=this->TheGlobalParametersStateCore->getMsfElementCoreSharedPtr()->getCovarianceInitErrorState();
    pointCovMatrix+=dimensionWorld;


    // Robot
    int dimensionRobot=this->TheRobotStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorState();
    covarianceMatrix.block(pointCovMatrix, pointCovMatrix, dimensionRobot, dimensionRobot)=this->TheRobotStateCore->getMsfElementCoreSharedPtr()->getCovarianceInitErrorState();
    pointCovMatrix+=dimensionRobot;


    // Sensors
    for(std::list<std::shared_ptr<SensorStateCore> >::const_iterator itLisSensorState=TheListSensorStateCore.begin();
        itLisSensorState!=TheListSensorStateCore.end();
        ++itLisSensorState)
    {
        int dimensionSensor=(*itLisSensorState)->getMsfElementCoreSharedPtr()->getDimensionErrorState();
        covarianceMatrix.block(pointCovMatrix, pointCovMatrix, dimensionSensor, dimensionSensor)=(*itLisSensorState)->getMsfElementCoreSharedPtr()->getCovarianceInitErrorState();
        pointCovMatrix+=dimensionSensor;
    }

    // Input
    for(std::list<std::shared_ptr<InputStateCore> >::const_iterator itListInputState=TheListInputStateCore.begin();
        itListInputState!=TheListInputStateCore.end();
        ++itListInputState)
    {
        int dimensionInput=(*itListInputState)->getMsfElementCoreSharedPtr()->getDimensionErrorState();
        covarianceMatrix.block(pointCovMatrix, pointCovMatrix, dimensionInput, dimensionInput)=(*itListInputState)->getMsfElementCoreSharedPtr()->getCovarianceInitErrorState();
        pointCovMatrix+=dimensionInput;
    }


    // Map
    for(std::list<std::shared_ptr<MapElementStateCore> >::const_iterator itLisMapElementState=TheListMapElementStateCore.begin();
        itLisMapElementState!=TheListMapElementStateCore.end();
        ++itLisMapElementState)
    {
        int dimensionMapElement=(*itLisMapElementState)->getMsfElementCoreSharedPtr()->getDimensionErrorState();
        covarianceMatrix.block(pointCovMatrix, pointCovMatrix, dimensionMapElement, dimensionMapElement)=(*itLisMapElementState)->getMsfElementCoreSharedPtr()->getCovarianceInitErrorState();
        pointCovMatrix+=dimensionMapElement;
    }



    // End
    return 0;
}
