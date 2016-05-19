
#ifndef _IMU_SENSOR_CORE_H
#define _IMU_SENSOR_CORE_H




#include "msf_localization_core/block_matrix.h"

// Time Stamp
#include "msf_localization_core/time_stamp.h"


// Sensor core
#include "msf_localization_core/sensor_core.h"

// IMU Measurement
#include "msf_localization_core/imu_sensor_measurement_core.h"

// IMU State
#include "msf_localization_core/imu_sensor_state_core.h"

// Robot Core
#include "msf_localization_core/robot_core.h"

// Robot state
#include "msf_localization_core/robot_state_core.h"

// Global parameters
#include "msf_localization_core/global_parameters_state_core.h"




class ImuSensorCore : public SensorCore
{
public:
    ImuSensorCore();
    ImuSensorCore(std::weak_ptr<MsfStorageCore> the_msf_storage_core);
    ~ImuSensorCore();


protected:
    int init();


public:
    int readConfig(const pugi::xml_node& sensor, const unsigned int sensorId, std::shared_ptr<ImuSensorStateCore>& SensorInitStateCore);




    ///// Measurements

    // z=[z_lin_accel, z_attit, z_ang_vel]'


    // Orientation Measurement
protected:
    bool flagMeasurementOrientation;
public:
    bool isMeasurementOrientationEnabled() const;
    int enableMeasurementOrientation();

    // Orientation measurement Covariance
protected:
    // TODO


    // Angular Velocity Measurement
protected:
    bool flagMeasurementAngularVelocity;
public:
    bool isMeasurementAngularVelocityEnabled() const;
    int enableMeasurementAngularVelocity();

    // Angular velocity measurement convariance
protected:
    Eigen::Matrix3d noiseMeasurementAngularVelocity;
public:
    Eigen::Matrix3d getNoiseMeasurementAngularVelocity() const;
    int setNoiseMeasurementAngularVelocity(const Eigen::Matrix3d& noiseMeasurementAngularVelocity);


    // Linear Acceleration Measurement
protected:
    bool flagMeasurementLinearAcceleration;
public:
    bool isMeasurementLinearAccelerationEnabled() const;
    int enableMeasurementLinearAcceleration();

    // Linear acceleration measurement covariance
protected:
    Eigen::Matrix3d noiseMeasurementLinearAcceleration;
public:
    Eigen::Matrix3d getNoiseMeasurementLinearAcceleration() const;
    int setNoiseMeasurementLinearAcceleration(const Eigen::Matrix3d& noiseMeasurementLinearAcceleration);


    // Store Measurement
public:
    int setMeasurement(const TimeStamp& TheTimeStamp, const std::shared_ptr<ImuSensorMeasurementCore> TheImuSensorMeasurement);







    ///// State estimation and parameters



    /// Angular Velocity biases: bw (3x1)

    // Angular Velocity Biases: bwx, bwy, bwz
protected:
    bool flagEstimationBiasAngularVelocity;
public:
    bool isEstimationBiasAngularVelocityEnabled() const;
    int enableEstimationBiasAngularVelocity();
    int enableParameterBiasAngularVelocity();


    // Angular Velocity biases: Covariance (if enabled estimation -> P; if no enabled estimation -> Sigma_mu)
protected:
    Eigen::Matrix3d noiseBiasAngularVelocity;
public:
    Eigen::Matrix3d getNoiseBiasAngularVelocity() const;
    int setNoiseBiasAngularVelocity(const Eigen::Matrix3d& noiseBiasAngularVelocity);


    // Angular Velocity biases: Estimation Covariance (if enabled estimation -> used)
protected:
    Eigen::Matrix3d noiseEstimationBiasAngularVelocity;
public:
    Eigen::Matrix3d getNoiseEstimationBiasAngularVelocity() const;
    int setNoiseEstimationBiasAngularVelocity(const Eigen::Matrix3d& noiseEstimationBiasAngularVelocity);



    /// Angular Velocity scale: kw (3x1)

    // Angular Velocity Scale: kwx, kwy, kwz
protected:
    bool flagEstimationScaleAngularVelocity; // TODO
public:
    bool isEstimationScaleAngularVelocityEnabled() const;
    int enableEstimationScaleAngularVelocity();
    int enableParameterScaleAngularVelocity();


    // Angular Velocity scale: Covariance (if enabled estimation -> P; if no enabled estimation -> Sigma_mu)
protected:
    Eigen::Matrix3d noiseScaleAngularVelocity;
public:
    Eigen::Matrix3d getNoiseScaleAngularVelocity() const;
    int setNoiseScaleAngularVelocity(const Eigen::Matrix3d& noiseScaleAngularVelocity);


    // Angular Velocity scale: Estimation Covariance (if enabled estimation -> used)
    // TODO


    // Angular Velocity Sensitivity
protected:
    bool flagEstimationSensitivityAngularVelocity; // TODO
public:
    bool isEstimationSensitivityAngularVelocityEnabled() const;
    int enableEstimationSensitivityAngularVelocity();
    int enableParameterSensitivityAngularVelocity();




    /// Linear Acceleration biases: ba (3x1)

    // Linear Acceleration Biases: bax, bay, baz
protected:
    bool flagEstimationBiasLinearAcceleration;
public:
    bool isEstimationBiasLinearAccelerationEnabled() const;
    int enableEstimationBiasLinearAcceleration();
    int enableParameterBiasLinearAcceleration();


    // Linear Acceleration biases: covariance (if enabled estimation -> P; if no enabled estimation -> Sigma_mu)
protected:
    Eigen::Matrix3d noiseBiasLinearAcceleration;
public:
    Eigen::Matrix3d getNoiseBiasLinearAcceleration() const;
    int setNoiseBiasLinearAcceleration(const Eigen::Matrix3d& noiseBiasLinearAcceleration);


    // Linear Acceleration biases: estimation covariance (if enabled estimation -> used)
protected:
    Eigen::Matrix3d noiseEstimationBiasLinearAcceleration;
public:
    Eigen::Matrix3d getNoiseEstimationBiasLinearAcceleration() const;
    int setNoiseEstimationBiasLinearAcceleration(const Eigen::Matrix3d& noiseEstimationBiasLinearAcceleration);



    /// Linear acceleration scale: ka (3x1)

    // Linear acceleration Scale: kax, kay, kaz
protected:
    bool flagEstimationScaleLinearAcceleration; // TODO
public:
    bool isEstimationScaleLinearAccelerationEnabled() const;
    int enableEstimationScaleLinearAcceleration();
    int enableParameterScaleLinearAcceleration();


    // Linear acceleration scale: Covariance (if enabled estimation -> P; if no enabled estimation -> Sigma_mu)
protected:
    Eigen::Matrix3d noiseScaleLinearAcceleration;
public:
    Eigen::Matrix3d getNoiseScaleLinearAcceleration() const;
    int setNoiseScaleLinearAcceleration(const Eigen::Matrix3d& noiseScaleLinearAcceleration);


    // Linear acceleration scale: Estimation Covariance (if enabled estimation -> used)
    // TODO




    ///// Covariances Getters

    // Covariance Error Measurements: Rn
public:
    Eigen::SparseMatrix<double> getCovarianceMeasurement();

    // Covariance Error Parameters: Rp = Qp
public:
    Eigen::SparseMatrix<double> getCovarianceParameters();

    // Covariance Noise Estimation: Qn
public:
    Eigen::SparseMatrix<double> getCovarianceNoise(const TimeStamp deltaTimeStamp);


    ////// Init error state variances -> Temporal, only for the initial configuration
public:
    // Covariance init Error State: P(0)
    int prepareCovarianceInitErrorStateSpecific();







    ///// Predict Step functions

    // State: xs=[posi_sensor_wrt_robot, att_sensor_wrt_robot, bias_lin_accel, ka, bias_ang_veloc, kw]'

    // Prediction state function: f
public:
    int predictState(//Time
                     const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp,
                     // Previous State
                     const std::shared_ptr<StateEstimationCore> pastState,
                     // Inputs
                     const std::shared_ptr<InputCommandComponent> inputCommand,
                     // Predicted State
                     std::shared_ptr<StateCore>& predictedState);

protected:
    int predictStateSpecific(const TimeStamp& previousTimeStamp, const TimeStamp& currentTimeStamp,
                     const std::shared_ptr<ImuSensorStateCore> pastState,
                     std::shared_ptr<ImuSensorStateCore>& predictedState);

    // Jacobian of the error state: F

public:
    int predictErrorStateJacobian(//Time
                                 const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp,
                                 // Previous State
                                 const std::shared_ptr<StateEstimationCore> past_state,
                                 // Inputs
                                 const std::shared_ptr<InputCommandComponent> input_command,
                                 // Predicted State
                                 std::shared_ptr<StateCore>& predicted_state);

protected:
    int predictErrorStateJacobiansSpecific(const TimeStamp& previousTimeStamp, const TimeStamp& currentTimeStamp,
                                            const std::shared_ptr<ImuSensorStateCore> pastState,
                                            std::shared_ptr<ImuSensorStateCore>& predictedState,
                                           // Jacobians Error State: Fx, Fp
                                           // Sensor
                                           Eigen::SparseMatrix<double>& jacobian_error_state_wrt_sensor_error_state,
                                           Eigen::SparseMatrix<double>& jacobian_error_state_wrt_sensor_error_parameters,
                                           // Jacobians Noise: Fn
                                           Eigen::SparseMatrix<double>& jacobian_error_state_wrt_noise
                                           );


protected:
    int predictErrorStateJacobiansCore(// State k: Sensor
                                       const Eigen::Vector3d& position_sensor_wrt_robot, const Eigen::Vector4d& attitude_sensor_wrt_robot,
                                       // TODO Add others
                                       // State k+1: Sensor
                                       const Eigen::Vector3d& pred_position_sensor_wrt_robot, const Eigen::Vector4d& pred_attitude_sensor_wrt_robot,
                                       // TODO add others
                                       // Jacobian: State Fx & Fp
                                       Eigen::Matrix3d& jacobian_error_sens_pos_wrt_error_state_sens_pos,  Eigen::Matrix3d& jacobian_error_sens_att_wrt_error_state_sens_att,
                                       Eigen::Matrix3d& jacobian_error_bias_lin_acc_wrt_error_bias_lin_acc,
                                       Eigen::Matrix3d& jacobian_error_bias_ang_vel_wrt_error_bias_ang_vel
                                       );





    //// Update Step functions


    /// State Correction

    // Prediction measurements: h
public:
    int predictMeasurement(// Time
                           const TimeStamp current_time_stamp,
                           // Current State
                           const std::shared_ptr<StateEstimationCore> current_state,
                           // Measurement to match
                           const std::shared_ptr<SensorMeasurementCore> measurement,
                           // Predicted Measurements
                           std::shared_ptr<SensorMeasurementCore> &predicted_measurement);

protected:
    int predictMeasurementSpecific(const TimeStamp& theTimeStamp,
                           const std::shared_ptr<GlobalParametersStateCore> TheGlobalParametersStateCore,
                           const std::shared_ptr<RobotStateCore> currentRobotState,
                           const std::shared_ptr<ImuSensorStateCore> currentImuState,
                           std::shared_ptr<ImuSensorMeasurementCore>& predictedMeasurement);


    // Jacobian of the measurements: H
public:
    int predictErrorMeasurementJacobian(// Time
                                        const TimeStamp current_time_stamp,
                                        // Current State
                                        const std::shared_ptr<StateEstimationCore> current_state,
                                        // Measurements
                                        const std::shared_ptr<SensorMeasurementCore> measurement,
                                        // Predicted Measurements
                                        std::shared_ptr<SensorMeasurementCore> &predicted_measurement);

protected:
    int predictErrorMeasurementJacobianSpecific(const TimeStamp& theTimeStamp,
                                                const std::shared_ptr<GlobalParametersStateCore> TheGlobalParametersStateCore,
                                                const std::shared_ptr<RobotStateCore> TheRobotStateCore,
                                                const std::shared_ptr<ImuSensorStateCore> TheImuStateCore,
                                                std::shared_ptr<ImuSensorMeasurementCore>& predictedMeasurement,
                                                // Jacobians State / Parameters
                                                // World
                                                Eigen::SparseMatrix<double>& jacobian_error_measurement_wrt_world_error_state,
                                                Eigen::SparseMatrix<double>& jacobian_error_measurement_wrt_world_error_parameters,
                                                // Robot
                                                Eigen::SparseMatrix<double>& jacobian_error_measurement_wrt_robot_error_state,
                                                Eigen::SparseMatrix<double>& jacobian_error_measurement_wrt_robot_error_parameters,
                                                // Sensor
                                                Eigen::SparseMatrix<double>& jacobian_error_measurement_wrt_sensor_error_state,
                                                Eigen::SparseMatrix<double>& jacobian_error_measurement_wrt_sensor_error_parameters,
                                                // Jacobians Measurement
                                                Eigen::SparseMatrix<double>& jacobian_error_measurement_wrt_error_measurement
                                                );
    int predictErrorMeasurementJacobianCore(// State: World
                                            const Eigen::Vector3d& gravity_wrt_world,
                                            // State: Robot
                                            const Eigen::Vector3d& position_robot_wrt_world, const Eigen::Vector4d& attitude_robot_wrt_world,
                                            const Eigen::Vector3d& lin_speed_robot_wrt_world, const Eigen::Vector3d& ang_velocity_robot_wrt_world,
                                            const Eigen::Vector3d& lin_accel_robot_wrt_world, const Eigen::Vector3d& ang_accel_robot_wrt_world,
                                            // State: Sensor
                                            const Eigen::Vector3d& position_sensor_wrt_robot, const Eigen::Vector4d& attitude_sensor_wrt_robot,
                                            // Parameters: Sensor
                                            const Eigen::Matrix3d& sensitivity_meas_linear_acceleration, const Eigen::Matrix3d& sensitivity_meas_angular_velocity,
                                            // Measurement
                                            const Eigen::Vector3d& meas_lin_accel_sensor_wrt_sensor, const Eigen::Vector3d& meas_attitude_sensor_wrt_sensor, const Eigen::Vector3d& meas_ang_velocity_sensor_wrt_sensor,
                                            // Predicted Measurement
                                            const Eigen::Vector3d& lin_accel_sensor_wrt_sensor, const Eigen::Vector3d& attitude_sensor_wrt_sensor, const Eigen::Vector3d& ang_velocity_sensor_wrt_sensor,
                                            // Jacobians: State and Params
                                            Eigen::Matrix3d& jacobian_error_meas_lin_acc_wrt_error_gravity,
                                            Eigen::Matrix3d& jacobian_error_meas_lin_acc_wrt_error_state_robot_lin_acc, Eigen::Matrix3d& jacobian_error_meas_lin_acc_wrt_error_state_robot_att, Eigen::Matrix3d& jacobian_error_meas_lin_acc_wrt_error_state_robot_ang_vel, Eigen::Matrix3d& jacobian_error_meas_lin_acc_wrt_error_state_robot_ang_acc,
                                            Eigen::Matrix3d& jacobian_error_meas_ang_vel_wrt_error_state_robot_att, Eigen::Matrix3d& jacobian_error_meas_ang_vel_wrt_error_state_robot_ang_vel,
                                            Eigen::Matrix3d& jacobian_error_meas_lin_acc_wrt_error_state_sensor_pos, Eigen::Matrix3d& jacobian_error_meas_lin_acc_wrt_error_state_sensor_att, Eigen::Matrix3d& jacobian_error_meas_lin_acc_wrt_error_state_sensor_bias_lin_acc,
                                            Eigen::Matrix3d& jacobian_error_meas_ang_vel_wrt_error_state_sensor_att, Eigen::Matrix3d& jacobian_error_meas_ang_vel_wrt_error_state_sensor_bias_ang_vel,
                                            // Jacobians: Noise
                                            Eigen::Matrix3d& jacobian_error_meas_lin_acc_wrt_error_meas_lin_acc, Eigen::Matrix3d& jacobian_error_meas_att_wrt_error_meas_att, Eigen::Matrix3d& jacobian_error_meas_ang_vel_wrt_error_meas_ang_vel
                                            );



    /// Reset Error State

public:
    int resetErrorStateJacobian(// Time
                                const TimeStamp& current_time_stamp,
                                // Increment Error State
                                const Eigen::VectorXd& increment_error_state,
                                // Current State
                                std::shared_ptr<StateCore>& current_state
                                );


    /// Mapping

    // None


};




#endif
