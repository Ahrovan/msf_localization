

#ifndef _IMU_SENSOR_STATE_CORE_H
#define _IMU_SENSOR_STATE_CORE_H


#include <Eigen/Dense>


#include "sensor_state_core.h"



class ImuSensorStateCore : public SensorStateCore
{
public:
    ImuSensorStateCore();
    ~ImuSensorStateCore();


    ///// Imu State

    // Angular Velocity Biases
protected:
    Eigen::Vector3d biasesAngularVelocity_ref;


    // Linear Acceleration Biases
protected:
    Eigen::Vector3d biasesLinearAcceleration_ref;





};








#endif
