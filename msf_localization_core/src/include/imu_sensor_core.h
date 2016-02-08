
#ifndef _IMU_SENSOR_CORE_H
#define _IMU_SENSOR_CORE_H



#include "sensor_core.h"

#include "imu_sensor_measurement_core.h"



class ImuSensorCore : public virtual SensorCore
{
public:
    ImuSensorCore();
    ~ImuSensorCore();

public:
    int setMeasurement(const TimeStamp TheTimeStamp, std::shared_ptr<ImuSensorMeasurementCore> TheImuSensorMeasurement);

};




#endif
