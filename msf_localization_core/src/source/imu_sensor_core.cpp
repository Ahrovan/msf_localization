
#include "imu_sensor_core.h"


ImuSensorCore::ImuSensorCore()
{
    setSensorType(SensorTypes::imu);
    return;
}

ImuSensorCore::~ImuSensorCore()
{
    return;
}

int ImuSensorCore::setMeasurement(const TimeStamp TheTimeStamp, std::shared_ptr<ImuSensorMeasurementCore> TheImuSensorMeasurement)
{
    std::cout<<"Imu Measurement Set"<<std::endl;

    this->TheMsfStorageCore->setMeasurement(TheTimeStamp, TheImuSensorMeasurement);

    return 0;
}
