
#ifndef _IMU_INPUT_STATE_CORE_H
#define _IMU_INPUT_STATE_CORE_H


#include <Eigen/Dense>
#include <Eigen/Sparse>


#include "msf_localization_core/input_state_core.h"


#include "msf_localization_core/quaternion_algebra.h"


class ImuInputStateCore : public InputStateCore
{
public:
    ImuInputStateCore();
    ImuInputStateCore(std::weak_ptr<MsfElementCore> msf_element_core_ptr);
    ~ImuInputStateCore();

protected:
    int init();



    ///// State and parameters
    // state xu=[p_i_r, q_i_r, ba, bw] (if enabled)
    // parameters with covariance pu=[p_i_r, q_i_r, ba, bw] (if enabled)
    // parameters without covariance pu=[Sa, Sw]


    // Pose of the input wrt robot
    // position of the input wrt robot
protected:
public:
    Eigen::Vector3d position_input_wrt_robot_;
public:
    Eigen::Vector3d getPositionInputWrtRobot() const;
    int setPositionInputWrtRobot(Eigen::Vector3d position_input_wrt_robot);


    // attitude of the input wrt robot
protected:
public:
    Eigen::Vector4d attitude_input_wrt_robot_;
public:
    Eigen::Vector4d getAttitudeInputWrtRobot() const;
    int setAttitudeInputWrtRobot(Eigen::Vector4d attitude_input_wrt_robot);

    // Linear Acceleration Biases
protected:
public:
    Eigen::Vector3d biases_linear_acceleration_;
public:
    Eigen::Vector3d getBiasesLinearAcceleration() const;
    int setBiasesLinearAcceleration(Eigen::Vector3d biases_linear_acceleration);


    // Angular Velocity Biases
protected:
public:
    Eigen::Vector3d biases_angular_velocity_;
public:
    Eigen::Vector3d getBiasesAngularVelocity() const;
    int setBiasesAngularVelocity(Eigen::Vector3d biases_angular_velocity);


    // Linear Acceleration Sensitivity
protected:
public:
    Eigen::Matrix3d sensitivity_linear_acceleration_;
public:
    Eigen::Matrix3d getSensitivityLinearAcceleration() const;
    int setSensitivityLinearAcceleration(Eigen::Matrix3d sensitivity_linear_acceleration);


    // Angular Velocity Sensitivity
protected:
public:
    Eigen::Matrix3d sensitivity_angular_velocity_;
public:
    Eigen::Matrix3d getSensitivityAngularVelocity() const;
    int setSensitivityAngularVelocity(Eigen::Matrix3d sensitivity_angular_velocity);




    // Jacobian Error State

    // Jacobian Error State Noise




public:
    int updateStateFromIncrementErrorState(Eigen::VectorXd increment_error_state);

};







#endif
