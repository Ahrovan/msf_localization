

#ifndef _IMU_SENSOR_STATE_CORE_H
#define _IMU_SENSOR_STATE_CORE_H


#include <Eigen/Dense>


#include "sensor_state_core.h"



class ImuSensorStateCore : public SensorStateCore
{
public:
    ImuSensorStateCore();
    ~ImuSensorStateCore();


    ///// Imu State (Parameters)

    // Angular Velocity Biases
    // Estimated
protected:
public:
    Eigen::Vector3d biasesAngularVelocity;
public:
    Eigen::Vector3d getBiasesAngularVelocity() const;
    int setBiasesAngularVelocity(Eigen::Vector3d biasesAngularVelocity);


    // Reference
protected:
    Eigen::Vector3d biasesAngularVelocity_ref;


    // Error
protected:
public:
    //Eigen::Vector3d biasesAngularVelocity_error;



    // Linear Acceleration Biases   
    // Estimated
protected:
public:
    Eigen::Vector3d biasesLinearAcceleration;
public:
    Eigen::Vector3d getBiasesLinearAcceleration() const;
    int setBiasesLinearAcceleration(Eigen::Vector3d biasesLinearAcceleration);

    // Reference
protected:
public:
    Eigen::Vector3d biasesLinearAcceleration_ref;

    // Error
protected:
public:
    //Eigen::Vector3d biasesLinearAcceleration_error;


};








#endif
