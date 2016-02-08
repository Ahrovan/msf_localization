
#include "sensor_core.h"





SensorCore::SensorCore() :
    SensorBasics()
{
    return;
}

SensorCore::~SensorCore()
{
    return;
}

int SensorCore::setTheMsfStorageCore(std::shared_ptr<MsfStorageCore> TheMsfStorageCore)
{
    this->TheMsfStorageCore=TheMsfStorageCore;
    return 0;
}






