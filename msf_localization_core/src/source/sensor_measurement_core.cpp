
#include "msf_localization_core/sensor_measurement_core.h"



SensorMeasurementCore::SensorMeasurementCore()
{
    return;
}

SensorMeasurementCore::~SensorMeasurementCore()
{
    return;
}


int SensorMeasurementCore::setTheSensorCore(std::weak_ptr<SensorCore> TheSensorCorePtr)
{
    this->TheSensorCorePtr=TheSensorCorePtr;
    return 0;
}
std::shared_ptr<SensorCore> SensorMeasurementCore::getTheSensorCore() const
{
    std::shared_ptr<SensorCore> TheSensorCore=this->TheSensorCorePtr.lock();
    return TheSensorCore;
}
