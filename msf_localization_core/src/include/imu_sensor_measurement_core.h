
#ifndef _IMU_SENSOR_MEASUREMENT_CORE_H
#define _IMU_SENSOR_MEASUREMENT_CORE_H



#include "sensor_measurement_core.h"


class ImuSensorMeasurementCore : public SensorMeasurementCore
{
public:
    ImuSensorMeasurementCore();
    ~ImuSensorMeasurementCore();


protected:
    bool flagOrientationEnabled;


protected:
    bool flagAngularVelocityEnabled;


protected:
    bool flagLinearAccelerationEnabled;


};



#endif
