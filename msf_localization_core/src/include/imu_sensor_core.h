
#ifndef _IMU_SENSOR_CORE_H
#define _IMU_SENSOR_CORE_H



#include "sensor_core.h"

#include "imu_sensor_measurement_core.h"

#include "time_stamp.h"



class ImuSensorCore : public virtual SensorCore
{
public:
    ImuSensorCore();
    ~ImuSensorCore();


    // Info related to the measurements

    // Orientation
protected:
    bool flagOrientationEnabled;
public:
    bool isOrientationEnabled() const;
    int enableOrientation();

    // Angular Velocity
protected:
    bool flagAngularVelocityEnabled;
public:
    bool isAngularVelocityEnabled() const;
    int enableAngularVelocity();

    // Linear Acceleration
protected:
    bool flagLinearAccelerationEnabled;
public:
    bool isLinearAccelerationEnabled() const;
    int enableLinearAcceleration();


public:
    int setMeasurement(const TimeStamp TheTimeStamp, std::shared_ptr<ImuSensorMeasurementCore> TheImuSensorMeasurement);

};




#endif
