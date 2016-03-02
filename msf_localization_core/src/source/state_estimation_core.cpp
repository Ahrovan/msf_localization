
#include "state_estimation_core.h"





StateEstimationCore::StateEstimationCore() //:
    //flagHasState(false),
    //flagHasRobotState(false),
    //flagHasSensorState(false),
    //flagHasMeasurement(false)
{
    return;
}

StateEstimationCore::~StateEstimationCore()
{
    // Be tidy

//    // Remove the intelliguent pointers
//    TheRobotStateCore.reset();


    // TheListSensorStateCore
//    for(std::list<SensorStateCore*>::iterator it=TheListSensorStateCore.begin(); it!=TheListSensorStateCore.end(); ++it)
//        delete *it;
    this->TheListSensorStateCore.clear();

    // TheListMeasurementCore
//    for(std::list<SensorMeasurementCore*>::iterator it=TheListMeasurementCore.begin(); it!=TheListMeasurementCore.end(); ++it)
//        delete *it;
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




