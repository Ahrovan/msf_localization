
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


int SensorCore::setTheSensorCore(std::weak_ptr<const SensorCore> TheSensorCorePtr)
{
    this->TheSensorCorePtr=TheSensorCorePtr;
    return 0;
}
std::shared_ptr<const SensorCore> SensorCore::getTheSensorCore() const
{
    std::shared_ptr<const SensorCore> TheSensorCoreSharedPtr=this->TheSensorCorePtr.lock();
    return TheSensorCoreSharedPtr;
}


int SensorCore::setTheMsfStorageCore(std::weak_ptr<MsfStorageCore> TheMsfStorageCore)
{
    this->TheMsfStorageCore=TheMsfStorageCore;
    return 0;
}






