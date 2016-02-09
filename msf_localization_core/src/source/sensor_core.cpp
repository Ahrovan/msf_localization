
#include "sensor_core.h"

//#include "state_estimation_core.h"

#include "msf_storage_core.h"



SensorCore::SensorCore() :
    SensorBasics()
{
    return;
}

SensorCore::~SensorCore()
{
    return;
}

int SensorCore::setTheMsfStorageCore(std::weak_ptr<MsfStorageCore> TheMsfStorageCore)
{
    this->TheMsfStorageCore=TheMsfStorageCore;
    return 0;
}






