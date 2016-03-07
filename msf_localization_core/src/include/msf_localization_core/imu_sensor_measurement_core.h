
#ifndef _IMU_SENSOR_MEASUREMENT_CORE_H
#define _IMU_SENSOR_MEASUREMENT_CORE_H



#include "msf_localization_core/sensor_measurement_core.h"


class ImuSensorMeasurementCore : public SensorMeasurementCore
{
public:
    ImuSensorMeasurementCore();
    ~ImuSensorMeasurementCore();



protected:
    bool flagOrientationSet;
public:
    bool isOrientationSet() const;
protected:
    Eigen::Vector4d Orientation;
public:
    int setOrientation(const Eigen::Vector4d Orientation);
    Eigen::Vector4d getOrientation() const;



protected:
    bool flagAngularVelocitySet;
public:
    bool isAngularVelocitySet() const;
protected:
    Eigen::Vector3d AngularVelocity;
public:
    int setAngularVelocity(Eigen::Vector3d AngularVelocity);
    Eigen::Vector3d getAngularVelocity() const;



protected:
    bool flagLinearAccelerationSet;
public:
    bool isLinearAccelerationSet() const;
protected:
    Eigen::Vector3d LinearAcceleration;
public:
    int setLinearAcceleration(Eigen::Vector3d LinearAcceleration);
    Eigen::Vector3d getLinearAcceleration() const;





    //// Get the full measurement as a Eigen::VectorXd
public:
    Eigen::VectorXd getMeasurement();

};



#endif
