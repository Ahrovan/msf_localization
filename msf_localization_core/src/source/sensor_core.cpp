
#include "sensor_core.h"


SensorCore::SensorCore() :
    sensorType(SensorTypes::undefined)
{
    return;
}

SensorCore::~SensorCore()
{
    return;
}


int SensorCore::setSensorType(SensorTypes sensorType)
{
    this->sensorType=sensorType;
    return 0;
}
SensorTypes SensorCore::getSensorType() const
{
    return sensorType;
}


int SensorCore::setSensorId(int sensorId)
{
    this->sensorId=sensorId;
    return 0;
}

int SensorCore::getSensorId() const
{
    return sensorId;
}
