
#include "imu_sensor_core.h"

// Circular Dependency
#include "msf_storage_core.h"


ImuSensorCore::ImuSensorCore()
{
    // Sensor Type
    setSensorType(SensorTypes::imu);

    // Flags measurement
    flagOrientationEnabled=false;
    flagAngularVelocityEnabled=false;
    flagLinearAccelerationEnabled=false;

    return;
}

ImuSensorCore::~ImuSensorCore()
{
    return;
}

bool ImuSensorCore::isOrientationEnabled() const
{
    return flagOrientationEnabled;
}

int ImuSensorCore::enableOrientation()
{
    this->flagOrientationEnabled=true;
    return 0;
}

bool ImuSensorCore::isAngularVelocityEnabled() const
{
    return flagAngularVelocityEnabled;
}

int ImuSensorCore::enableAngularVelocity()
{
    this->flagAngularVelocityEnabled=true;
    return 0;
}

bool ImuSensorCore::isLinearAccelerationEnabled() const
{
    return flagLinearAccelerationEnabled;
}

int ImuSensorCore::enableLinearAcceleration()
{
    this->flagLinearAccelerationEnabled=true;
    return 0;
}

int ImuSensorCore::setMeasurement(const TimeStamp TheTimeStamp, std::shared_ptr<ImuSensorMeasurementCore> TheImuSensorMeasurement)
{
    std::cout<<"Imu Measurement Set"<<std::endl;

    std::shared_ptr<MsfStorageCore> TheMsfStorageCoreAux=this->TheMsfStorageCore.lock();
//    if(!TheMsfStorageCoreAux)
//        std::cout<<"Unable to lock TheMsfStorageCore"<<std::endl;

    TheMsfStorageCoreAux->setMeasurement(TheTimeStamp, TheImuSensorMeasurement);

    return 0;
}
