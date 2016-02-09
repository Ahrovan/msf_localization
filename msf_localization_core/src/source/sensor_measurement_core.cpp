
#include "sensor_measurement_core.h"



SensorMeasurementCore::SensorMeasurementCore()
{
    return;
}

SensorMeasurementCore::~SensorMeasurementCore()
{
    return;
}


int SensorMeasurementCore::setTheSensorCore(std::weak_ptr<const SensorCore> TheSensorCorePtr)
{
    this->TheSensorCorePtr=TheSensorCorePtr;
    return 0;
}
std::weak_ptr<const SensorCore> SensorMeasurementCore::getTheSensorCore() const
{
    return this->TheSensorCorePtr;
}
