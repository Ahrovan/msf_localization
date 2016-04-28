
#ifndef _IMU_INPUT_COMMAND_CORE_H
#define _IMU_INPUT_COMMAND_CORE_H


#include <Eigen/Dense>


#include "msf_localization_core/input_command_core.h"

#include "msf_localization_core/quaternion_algebra.h"



class ImuInputCommandCore : public InputCommandCore
{
public:
    ImuInputCommandCore();
    ImuInputCommandCore(std::weak_ptr<InputCore> input_core_ptr);
    ~ImuInputCommandCore();

protected:
    int init();


    ///// Imu Input Commands

    // Orientation
protected:
    Eigen::Vector4d orientation_;
public:
    int setOrientation(const Eigen::Vector4d orientation);
    Eigen::Vector4d getOrientation() const;


    // Angular Velocity
protected:
    Eigen::Vector3d angular_velocity_;
public:
    int setAngularVelocity(Eigen::Vector3d angular_velocity);
    Eigen::Vector3d getAngularVelocity() const;


    // Linear Acceleration
protected:
    Eigen::Vector3d linear_acceleration_;
public:
    int setLinearAcceleration(Eigen::Vector3d linear_acceleration);
    Eigen::Vector3d getLinearAcceleration() const;




/*

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

*/


};








#endif
