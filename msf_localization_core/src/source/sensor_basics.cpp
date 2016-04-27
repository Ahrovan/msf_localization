
#include "msf_localization_core/sensor_core.h"


SensorBasics::SensorBasics() :
    sensorType(SensorTypes::undefined),
    flagSensorEnabled(false)
{
    //std::cout<<"SensorBasics::SensorBasics()"<<std::endl;

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


bool SensorBasics::isSensorEnabled() const
{
    return this->flagSensorEnabled;
}

int SensorBasics::setSensorEnabled(bool flagSensorEnabled)
{
    this->flagSensorEnabled=flagSensorEnabled;
    return 0;
}




