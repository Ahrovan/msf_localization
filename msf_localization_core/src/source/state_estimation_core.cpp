
#include "state_estimation_core.h"





StateEstimationCore::StateEstimationCore() :
    flagHasRobotState(false),
    flagHasSensorState(false),
    flagHasMeasurement(false)
{
    return;
}

StateEstimationCore::~StateEstimationCore()
{
    // Be tidy

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





