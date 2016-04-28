
#ifndef _IMU_INPUT_CORE_H
#define _IMU_INPUT_CORE_H



#include "msf_localization_core/input_core.h"


#include "msf_localization_core/imu_input_command_core.h"

#include "msf_localization_core/imu_input_state_core.h"


#include "pugixml/pugixml.hpp"


class ImuInputCore : public InputCore
{
public:
    ImuInputCore();
    ImuInputCore(std::weak_ptr<MsfStorageCore> the_msf_storage_core);
    ~ImuInputCore();

protected:
    int init();

public:
    int readConfig(pugi::xml_node input, std::shared_ptr<ImuInputStateCore>& init_state_core);




    ///// Inputs Command

    // u=[u_lin_accel, u_attit, u_ang_vel]'


    /// Orientation Input Command
protected:
    bool flag_input_command_orientation_;
public:
    bool isInputCommandOrientationEnabled() const;
    int enableInputCommandOrientation();

    // Orientation input command Covariance
protected:
    // TODO


    /// Angular Velocity Input Command
protected:
    bool flag_input_command_angular_velocity_;
public:
    bool isInputCommandAngularVelocityEnabled() const;
    int enableInputCommandAngularVelocity();

    // Angular velocity Input Command covariance
protected:
    Eigen::Matrix3d noise_input_command_angular_velocity_;
public:
    Eigen::Matrix3d getNoiseInputCommandAngularVelocity() const;
    int setNoiseInputCommandAngularVelocity(Eigen::Matrix3d noise_input_command_angular_velocity);


    /// Linear Acceleration Input Command
protected:
    bool flag_input_command_linear_acceleration_;
public:
    bool isInputCommandLinearAccelerationEnabled() const;
    int enableInputCommandLinearAcceleration();

    // Linear acceleration Input Command covariance
protected:
    Eigen::Matrix3d noise_input_command_linear_acceleration_;
public:
    Eigen::Matrix3d getNoiseInputCommandLinearAcceleration() const;
    int setNoiseInputCommandLinearAcceleration(Eigen::Matrix3d noise_input_command_linear_acceleration);


    /// Store Input Command
public:
    int setInputCommand(const TimeStamp time_stamp, std::shared_ptr<ImuInputCommandCore> imu_input_command_core);




    ///// State and parameters
    // state xu=[p_i_r, q_i_r, ba, bw] (if enabled)
    // parameters with covariance pu=[p_i_r, q_i_r, ba, bw] (if enabled)
    // parameters without covariance pu=[Sa, Sw]


    /// Position input wrt robot

    // Position input wrt robot: t_input_wrt_robot (3x1)
protected:
    bool flag_estimation_position_input_wrt_robot_;
public:
    bool isEstimationPositionInputWrtRobotEnabled() const;
    int enableEstimationPositionInputWrtRobot();
    int enableParameterPositionInputWrtRobot();

    // Position input wrt robot: covariance (if enabled estimation -> P; if no enabled estimation -> Sigma_mu)
protected:
    Eigen::Matrix3d noise_position_input_wrt_robot_;
public:
    Eigen::Matrix3d getNoisePositionInputWrtRobot() const;
    int setNoisePositionInputWrtRobot(Eigen::Matrix3d noise_position_input_wrt_robot);



    /// Attitude input wrt robot

    // Attitude input wrt robot: q_sensor_wrt_robot (4x1) [Theta_input_wrt_robot (3x1)]
protected:
    bool flag_estimation_attitude_input_wrt_robot_;
public:
    bool isEstimationAttitudeInputWrtRobotEnabled() const;
    int enableEstimationAttitudeInputWrtRobot();
    int enableParameterAttitudeInputWrtRobot();

    // Attitude input wrt robot: covariance (if enabled estimation -> P; if no enabled estimation -> Sigma_mu)
protected:
    Eigen::Matrix3d noise_attitude_input_wrt_robot_;
public:
    Eigen::Matrix3d getNoiseAttitudeInputWrtRobot() const;
    int setNoiseAttitudeInputWrtRobot(Eigen::Matrix3d noise_attitude_input_wrt_robot);



    /// Linear Acceleration biases: ba (3x1)

    // Linear Acceleration Biases: bax, bay, baz
protected:
    bool flag_estimation_bias_linear_acceleration_;
public:
    bool isEstimationBiasLinearAccelerationEnabled() const;
    int enableEstimationBiasLinearAcceleration();
    int enableParameterBiasLinearAcceleration();


    // Linear Acceleration biases: covariance (if enabled estimation -> P; if no enabled estimation -> Sigma_mu)
protected:
    Eigen::Matrix3d noise_bias_linear_acceleration_;
public:
    Eigen::Matrix3d getNoiseBiasLinearAcceleration() const;
    int setNoiseBiasLinearAcceleration(Eigen::Matrix3d noise_bias_linear_acceleration);


    // Linear Acceleration biases: estimation covariance (if enabled estimation -> used)
protected:
    Eigen::Matrix3d noise_estimation_bias_linear_acceleration_;
public:
    Eigen::Matrix3d getNoiseEstimationBiasLinearAcceleration() const;
    int setNoiseEstimationBiasLinearAcceleration(Eigen::Matrix3d noise_estimation_bias_linear_acceleration);



    /// Angular Velocity biases: bw (3x1)

    // Angular Velocity Biases: bwx, bwy, bwz
protected:
    bool flag_estimation_bias_angular_velocity_;
public:
    bool isEstimationBiasAngularVelocityEnabled() const;
    int enableEstimationBiasAngularVelocity();
    int enableParameterBiasAngularVelocity();


    // Angular Velocity biases: Covariance (if enabled estimation -> P; if no enabled estimation -> Sigma_mu)
protected:
    Eigen::Matrix3d noise_bias_angular_velocity_;
public:
    Eigen::Matrix3d getNoiseBiasAngularVelocity() const;
    int setNoiseBiasAngularVelocity(Eigen::Matrix3d noise_bias_angular_velocity);


    // Angular Velocity biases: Estimation Covariance (if enabled estimation -> used)
protected:
    Eigen::Matrix3d noise_estimation_bias_angular_velocity_;
public:
    Eigen::Matrix3d getNoiseEstimationBiasAngularVelocity() const;
    int setNoiseEstimationBiasAngularVelocity(Eigen::Matrix3d noise_estimation_bias_angular_velocity);



    /// Linear acceleration sensitivity: Sa (3x1)

    // Linear acceleration sensitivity:
    // TODO
protected:
    bool flag_estimation_sensitivity_linear_acceleration_;
public:
    bool isEstimationSensitivityLinearAccelerationEnabled() const;
    int enableEstimationSensitivityLinearAcceleration();
    int enableParameterSensitivityLinearAcceleration();


    // Linear acceleration sensitivity: Covariance (if enabled estimation -> P; if no enabled estimation -> Sigma_mu)
    // TODO
protected:
    Eigen::MatrixXd noise_sensitivity_linear_acceleration_;
public:
    Eigen::MatrixXd getNoiseSensitivityLinearAcceleration() const;
    int setNoiseSensitivityLinearAcceleration(Eigen::Matrix3d noise_sensitivity_linear_acceleration);


    // Linear acceleration sensitivity: Estimation Covariance (if enabled estimation -> used)
    // TODO



    /// Angular Velocity sensitivity: Sw (3x1)

    // Angular Velocity sensitivity
    // TODO
protected:
    bool flag_estimation_sensitivity_angular_velocity_;
public:
    bool isEstimationSensitivityAngularVelocityEnabled() const;
    int enableEstimationSensitivityAngularVelocity();
    int enableParameterSensitivityAngularVelocity();


    // Angular Velocity sensitivity: Covariance (if enabled estimation -> P; if no enabled estimation -> Sigma_mu)
    // TODO
protected:
    Eigen::Matrix3d noise_sensitivity_angular_velocity_;
public:
    Eigen::Matrix3d getNoiseSensitivityAngularVelocity() const;
    int setNoiseSensitivityAngularVelocity(Eigen::Matrix3d noise_sensitivity_angular_velocity);


    // Angular Velocity sensitivity: Estimation Covariance (if enabled estimation -> used)
    // TODO



    ////// Init error state variances -> Temporal, only for the initial configuration
public:
    int prepareCovarianceInitErrorState();





};



#endif
