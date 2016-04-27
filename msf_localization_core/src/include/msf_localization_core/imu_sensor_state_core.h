

#ifndef _IMU_SENSOR_STATE_CORE_H
#define _IMU_SENSOR_STATE_CORE_H


#include <Eigen/Dense>


#include "msf_localization_core/sensor_state_core.h"

#include "msf_localization_core/quaternion_algebra.h"



class ImuSensorStateCore : public SensorStateCore
{
public:
    ImuSensorStateCore();
    ImuSensorStateCore(std::weak_ptr<MsfElementCore> msf_element_core_ptr);
    ~ImuSensorStateCore();

protected:
    int init();


    ///// Imu State if enabled (or Parameters if disabled)

    // if enabled
    // State: xs=[posi_sensor_wrt_robot, att_sensor_wrt_robot, bias_lin_accel, ka, bias_ang_veloc, kw]'


    // Angular Velocity Biases
protected:
public:
    Eigen::Vector3d biasesAngularVelocity;
public:
    Eigen::Vector3d getBiasesAngularVelocity() const;
    int setBiasesAngularVelocity(Eigen::Vector3d biasesAngularVelocity);


    // Angular Velocity Scale
protected:
public:
    Eigen::Vector3d scaleAngularVelocity;
public:
    Eigen::Vector3d getScaleAngularVelocity() const;
    int setScaleAngularVelocity(Eigen::Vector3d scaleAngularVelocity);


    // Linear Acceleration Biases   
protected:
public:
    Eigen::Vector3d biasesLinearAcceleration;
public:
    Eigen::Vector3d getBiasesLinearAcceleration() const;
    int setBiasesLinearAcceleration(Eigen::Vector3d biasesLinearAcceleration);


    // Linear Acceleration Scale
protected:
public:
    Eigen::Vector3d scaleLinearAcceleration;
public:
    Eigen::Vector3d getScaleLinearAcceleration() const;
    int setScaleLinearAcceleration(Eigen::Vector3d scaleLinearAcceleration);




    ////// Jacobians

    /// Jacobians Error State

public:
    struct
    {
        Eigen::Matrix3d positionSensorWrtRobot;
        Eigen::Matrix3d attitudeSensorWrtRobot;
        Eigen::Matrix3d biasesLinearAcceleration;
        Eigen::Matrix3d scaleLinearAcceleration;
        Eigen::Matrix3d biasesAngularVelocity;
        Eigen::Matrix3d scaleAngularVelocity;
    } errorStateJacobian;


    // TODO
//public:
//    Eigen::SparseMatrix<double> jacobianErrorState;


public:
    Eigen::SparseMatrix<double> jacobianErrorStateNoise;


public:
    Eigen::MatrixXd getJacobianErrorState();
    Eigen::SparseMatrix<double> getJacobianErrorStateNoise();


public:
    int updateStateFromIncrementErrorState(Eigen::VectorXd increment_error_state);




};








#endif
