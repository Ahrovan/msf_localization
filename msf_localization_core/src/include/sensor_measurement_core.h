

#ifndef _SENSOR_MEASUREMENT_CORE_H
#define _SENSOR_MEASUREMENT_CORE_H


#include <memory>


#include <Eigen/Dense>


#include "sensor_core.h"



class SensorMeasurementCore
{
public:
    SensorMeasurementCore();
    ~SensorMeasurementCore();


protected:
    // It is not the owner of this Pointer. it doesn't modify the pointer
    std::weak_ptr<SensorCore> TheSensorCorePtr;
public:
    int setTheSensorCore(std::weak_ptr<SensorCore> TheSensorCorePtr);
    std::shared_ptr<SensorCore> getTheSensorCore() const;


};






#endif
