
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



int StateEstimationCore::getDimensionState() const
{
    int dimensionState=0;

    // Global Parameters
    // TODO

    // Robot
    dimensionState+=this->TheRobotStateCore->getMsfElementCoreSharedPtr()->getDimensionState();

    // Sensors
    for(std::list< std::shared_ptr<SensorStateCore> >::const_iterator itSensor=TheListSensorStateCore.begin();
        itSensor!=TheListSensorStateCore.end();
        ++itSensor)
    {
        dimensionState+=(*itSensor)->getTheSensorCore()->getDimensionState();
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
    // TODO

    // Robot
    dimensionErrorState+=this->TheRobotStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorState();

    // Sensors
    for(std::list< std::shared_ptr<SensorStateCore> >::const_iterator itSensor=TheListSensorStateCore.begin();
        itSensor!=TheListSensorStateCore.end();
        ++itSensor)
    {
        dimensionErrorState+=(*itSensor)->getTheSensorCore()->getDimensionErrorState();
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


int StateEstimationCore::prepareInitErrorStateVariance()
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
    // TODO


    // Robot
    int dimensionRobot=this->TheRobotStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorState();
    covarianceMatrix.block(pointCovMatrix, pointCovMatrix, dimensionRobot, dimensionRobot)=this->TheRobotStateCore->getMsfElementCoreSharedPtr()->getCovarianceInitErrorState();
    pointCovMatrix+=dimensionRobot;


    // Sensors
    for(std::list<std::shared_ptr<SensorStateCore> >::const_iterator itLisSensorState=TheListSensorStateCore.begin();
        itLisSensorState!=TheListSensorStateCore.end();
        ++itLisSensorState)
    {
        int dimensionSensor=(*itLisSensorState)->getTheSensorCore()->getDimensionErrorState();
        covarianceMatrix.block(pointCovMatrix, pointCovMatrix, dimensionSensor, dimensionSensor)=(*itLisSensorState)->getTheSensorCore()->getCovarianceInitErrorState();
        pointCovMatrix+=dimensionSensor;
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
