
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

int ImuSensorCore::setMeasurement()
{
    std::cout<<"Measurement Set"<<std::endl;
    return 0;
}
