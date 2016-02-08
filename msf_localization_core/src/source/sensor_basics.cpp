
#include "sensor_core.h"


SensorBasics::SensorBasics() :
    sensorType(SensorTypes::undefined)
{
    return;
}

SensorBasics::~SensorBasics()
{
    return;
}


int SensorBasics::setSensorType(SensorTypes sensorType)
{
    this->sensorType=sensorType;
    return 0;
}
SensorTypes SensorBasics::getSensorType() const
{
    return sensorType;
}


int SensorBasics::setSensorId(int sensorId)
{
    this->sensorId=sensorId;
    return 0;
}

int SensorBasics::getSensorId() const
{
    return sensorId;
}






