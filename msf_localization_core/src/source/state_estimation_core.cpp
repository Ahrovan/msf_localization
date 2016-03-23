
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

    // TheListMapElementStateCore
    this->TheListMapElementStateCore.clear();

    // TheListMeasurementCore
    this->TheListMeasurementCore.clear();


    return;
}


bool StateEstimationCore::hasState() const
{
    // TODO check
    if(TheListSensorStateCore.size()!=0 || TheRobotStateCore)
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



int StateEstimationCore::getDimensionState() const
{
    int dimensionState=0;

    // Global Parameters
    // TODO

    // Robot
    dimensionState+=this->TheRobotStateCore->getTheRobotCore()->getDimensionState();

    // Sensors
    for(std::list< std::shared_ptr<SensorStateCore> >::const_iterator itSensor=TheListSensorStateCore.begin();
        itSensor!=TheListSensorStateCore.end();
        ++itSensor)
    {
        dimensionState+=(*itSensor)->getTheSensorCore()->getDimensionState();
    }

    // Map
    // TODO


    // end
    return dimensionState;
}


int StateEstimationCore::getDimensionErrorState() const
{
    int dimensionErrorState=0;

    // Global Parameters
    // TODO

    // Robot
    dimensionErrorState+=this->TheRobotStateCore->getTheRobotCore()->getDimensionErrorState();

    // Sensors
    for(std::list< std::shared_ptr<SensorStateCore> >::const_iterator itSensor=TheListSensorStateCore.begin();
        itSensor!=TheListSensorStateCore.end();
        ++itSensor)
    {
        dimensionErrorState+=(*itSensor)->getTheSensorCore()->getDimensionErrorState();
    }

    // Map
    // TODO


    // end
    return dimensionErrorState;
}


int StateEstimationCore::prepareInitErrorStateVariance()
{
    // Dimension
    int dimensionErrorState=this->getDimensionErrorState();

    // Init
    covarianceMatrix.resize(dimensionErrorState, dimensionErrorState);
    covarianceMatrix.setZero();


    // Add matrix
    int pointCovMatrix=0;


    // Global Parameters
    // TODO


    // Robot
    int dimensionRobot=this->TheRobotStateCore->getTheRobotCore()->getDimensionErrorState();
    covarianceMatrix.block(pointCovMatrix, pointCovMatrix, dimensionRobot, dimensionRobot)=this->TheRobotStateCore->getTheRobotCore()->getInitErrorStateVariance();
    pointCovMatrix+=dimensionRobot;


    // Sensors
    for(std::list<std::shared_ptr<SensorStateCore> >::const_iterator itLisSensorState=TheListSensorStateCore.begin();
        itLisSensorState!=TheListSensorStateCore.end();
        ++itLisSensorState)
    {
        int dimensionSensor=(*itLisSensorState)->getTheSensorCore()->getDimensionErrorState();
        covarianceMatrix.block(pointCovMatrix, pointCovMatrix, dimensionSensor, dimensionSensor)=(*itLisSensorState)->getTheSensorCore()->getInitErrorStateVariance();
        pointCovMatrix+=dimensionSensor;
    }


    // Map
    // TODO


    // End
    return 0;
}
