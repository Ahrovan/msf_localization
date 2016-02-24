

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
protected:
public:
    Eigen::Vector3d biasesAngularVelocity;
public:
    Eigen::Vector3d getBiasesAngularVelocity() const;
    int setBiasesAngularVelocity(Eigen::Vector3d biasesAngularVelocity);



    // Linear Acceleration Biases   
protected:
public:
    Eigen::Vector3d biasesLinearAcceleration;
public:
    Eigen::Vector3d getBiasesLinearAcceleration() const;
    int setBiasesLinearAcceleration(Eigen::Vector3d biasesLinearAcceleration);





    // Error State Jacobians
public:
    struct
    {
        Eigen::Matrix3d positionSensorWrtRobot;
        Eigen::Matrix3d attitudeSensorWrtRobot;
        Eigen::Matrix3d biasesLinearAcceleration;
        Eigen::Matrix3d biasesAngularVelocity;
    } errorStateJacobian;


};








#endif
