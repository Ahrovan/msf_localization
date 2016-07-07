
#ifndef _PX4FLOW_SENSOR_CORE_H
#define _PX4FLOW_SENSOR_CORE_H



// Pugixml
#include "pugixml/pugixml.hpp"

// Time Stamp
#include "msf_localization_core/time_stamp.h"


// Sensor core
#include "msf_localization_core/sensor_core.h"

// Px4flow sensor Measurement
#include "msf_localization_core/px4flow_sensor_measurement_core.h"

// Px4flow sensor State
#include "msf_localization_core/px4flow_sensor_state_core.h"

// Robot Core
#include "msf_localization_core/robot_core.h"

// Robot state
#include "msf_localization_core/robot_state_core.h"





class Px4FlowSensorCore : public SensorCore
{
public:
    Px4FlowSensorCore();
    Px4FlowSensorCore(MsfLocalizationCore* msf_localization_core_ptr);
    ~Px4FlowSensorCore();


protected:
    int init();


public:
    int readConfig(const pugi::xml_node& sensor, const unsigned int sensor_id, std::shared_ptr<Px4FlowSensorStateCore>& sensor_init_state);




    ///// Measurements

    // Meas=[v, dg]


    // Velocity: v=[vx, vy]
protected:
    bool flag_measurement_velocity_;
public:
    bool isMeasurementVelocityEnabled() const;
    void enableMeasurementVelocity();

    // Velocity Covariance
protected:
    Eigen::Matrix2d noise_measurement_velocity_;
public:
    Eigen::Matrix2d getNoiseMeasurementVelocity() const;
    void setNoiseMeasurementVelocity(const Eigen::Matrix2d& noise_measurement_velocity);


    // Ground Distance: dg
protected:
    bool flag_measurement_ground_distance_;
public:
    bool isMeasurementGroundDistanceEnabled() const;
    void enableMeasurementGroundDistance();

    // Ground Distance Covariance
protected:
    double noise_measurement_ground_distance_;
public:
    double getNoiseMeasurementGroundDistance() const;
    void setNoiseMeasurementGroundDistance(double noise_measurement_ground_distance);





    ///// State estimation and parameters


/*
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

*/






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
                     const TimeStamp& previousTimeStamp, const TimeStamp& currentTimeStamp,
                     // Previous State
                     const std::shared_ptr<StateComponent>& pastState,
                     // Inputs
                     const std::shared_ptr<InputCommandComponent>& inputCommand,
                     // Predicted State
                     std::shared_ptr<StateCore>& predictedState);

protected:
    int predictStateSpecific(const TimeStamp& previousTimeStamp, const TimeStamp& currentTimeStamp,
                            const Px4FlowSensorStateCore* pastState,
                            Px4FlowSensorStateCore*& predictedState);

    // Jacobian of the error state: F

public:
    int predictErrorStateJacobian(//Time
                                 const TimeStamp& previousTimeStamp, const TimeStamp& currentTimeStamp,
                                 // Previous State
                                 const std::shared_ptr<StateComponent>& past_state,
                                 // Inputs
                                 const std::shared_ptr<InputCommandComponent>& input_command,
                                 // Predicted State
                                 std::shared_ptr<StateCore>& predicted_state);

protected:
    int predictErrorStateJacobiansSpecific(const TimeStamp& previousTimeStamp, const TimeStamp& currentTimeStamp,
                                           const Px4FlowSensorStateCore* pastState,
                                           const Px4FlowSensorStateCore* predictedState,
                                           // Jacobians Error State: Fx, Fp
                                           // Sensor
                                           Eigen::SparseMatrix<double>& jacobian_error_state_wrt_sensor_error_state,
                                           Eigen::SparseMatrix<double>& jacobian_error_state_wrt_sensor_error_parameters,
                                           // Jacobians Noise: Fn
                                           Eigen::SparseMatrix<double>& jacobian_error_state_wrt_noise
                                           );

/*
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
*/




    //// Update Step functions


    /// State Correction

    // Prediction measurements: h
public:
    int predictMeasurement(// Time
                           const TimeStamp& current_time_stamp,
                           // Current State
                           const std::shared_ptr<StateComponent>& current_state,
                           // Measurement to match
                           const std::shared_ptr<SensorMeasurementCore>& measurement,
                           // Predicted Measurements
                           std::shared_ptr<SensorMeasurementCore> &predicted_measurement);

protected:
    int predictMeasurementSpecific(const TimeStamp& theTimeStamp,
                           const RobotStateCore* currentRobotState,
                           const Px4FlowSensorStateCore* currentImuState,
                           Px4FlowSensorMeasurementCore*& predictedMeasurement);


    // Jacobian of the measurements: H
public:
    int predictErrorMeasurementJacobian(// Time
                                        const TimeStamp& current_time_stamp,
                                        // Current State
                                        const std::shared_ptr<StateComponent>& current_state,
                                        // Measurements
                                        const std::shared_ptr<SensorMeasurementCore>& measurement,
                                        // Predicted Measurements
                                        std::shared_ptr<SensorMeasurementCore> &predicted_measurement);

protected:
    int predictErrorMeasurementJacobianSpecific(const TimeStamp& theTimeStamp,
                                                const RobotStateCore* TheRobotStateCore,
                                                const Px4FlowSensorStateCore* TheImuStateCore,
                                                Px4FlowSensorMeasurementCore*& predictedMeasurement,
                                                // Jacobians State / Parameters
                                                // Robot
                                                Eigen::SparseMatrix<double>& jacobian_error_measurement_wrt_robot_error_state,
                                                Eigen::SparseMatrix<double>& jacobian_error_measurement_wrt_robot_error_parameters,
                                                // Sensor
                                                Eigen::SparseMatrix<double>& jacobian_error_measurement_wrt_sensor_error_state,
                                                Eigen::SparseMatrix<double>& jacobian_error_measurement_wrt_sensor_error_parameters,
                                                // Jacobians Measurement
                                                Eigen::SparseMatrix<double>& jacobian_error_measurement_wrt_error_measurement
                                                );
/*
protected:
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
*/


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
