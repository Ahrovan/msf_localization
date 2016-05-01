
#ifndef _IMU_SENSOR_CORE_H
#define _IMU_SENSOR_CORE_H






// Time Stamp
#include "msf_localization_core/time_stamp.h"


// Sensor core
#include "msf_localization_core/sensor_core.h"

// IMU Measurement
#include "msf_localization_core/imu_sensor_measurement_core.h"

// IMU State
#include "msf_localization_core/imu_sensor_state_core.h"


// Robot state
#include "msf_localization_core/robot_state_core.h"
#include "msf_localization_core/free_model_robot_state_core.h"

// Global parameters
#include "msf_localization_core/global_parameters_core.h"
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
    int readConfig(pugi::xml_node sensor, unsigned int sensorId, std::shared_ptr<ImuSensorStateCore>& SensorInitStateCore);




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
    int setNoiseMeasurementAngularVelocity(Eigen::Matrix3d noiseMeasurementAngularVelocity);


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
    int setNoiseMeasurementLinearAcceleration(Eigen::Matrix3d noiseMeasurementLinearAcceleration);


    // Store Measurement
public:
    int setMeasurement(const TimeStamp TheTimeStamp, std::shared_ptr<ImuSensorMeasurementCore> TheImuSensorMeasurement);







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
    int setNoiseBiasAngularVelocity(Eigen::Matrix3d noiseBiasAngularVelocity);


    // Angular Velocity biases: Estimation Covariance (if enabled estimation -> used)
protected:
    Eigen::Matrix3d noiseEstimationBiasAngularVelocity;
public:
    Eigen::Matrix3d getNoiseEstimationBiasAngularVelocity() const;
    int setNoiseEstimationBiasAngularVelocity(Eigen::Matrix3d noiseEstimationBiasAngularVelocity);



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
    int setNoiseScaleAngularVelocity(Eigen::Matrix3d noiseScaleAngularVelocity);


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
    int setNoiseBiasLinearAcceleration(Eigen::Matrix3d noiseBiasLinearAcceleration);


    // Linear Acceleration biases: estimation covariance (if enabled estimation -> used)
protected:
    Eigen::Matrix3d noiseEstimationBiasLinearAcceleration;
public:
    Eigen::Matrix3d getNoiseEstimationBiasLinearAcceleration() const;
    int setNoiseEstimationBiasLinearAcceleration(Eigen::Matrix3d noiseEstimationBiasLinearAcceleration);



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
    int setNoiseScaleLinearAcceleration(Eigen::Matrix3d noiseScaleLinearAcceleration);


    // Linear acceleration scale: Estimation Covariance (if enabled estimation -> used)
    // TODO




    ///// Covariances Getters

    // Covariance Sensor Error Measurements: Rn
public:
    Eigen::SparseMatrix<double> getCovarianceMeasurement();

    // Covariance Sensor Error Parameters: Rp = Qp
public:
    Eigen::SparseMatrix<double> getCovarianceParameters();

    // Covariance Noise Estimation: Qn
public:
    Eigen::SparseMatrix<double> getCovarianceNoise(const TimeStamp deltaTimeStamp);


    ////// Init error state variances -> Temporal, only for the initial configuration
public:
    // Covariance init Error State: P(0)
    int prepareCovarianceInitErrorStateSpecific();







    ///// Predict functions

    // State: xs=[posi_sensor_wrt_robot, att_sensor_wrt_robot, bias_lin_accel, ka, bias_ang_veloc, kw]'

    // Prediction state function: f
public:
    int predictState(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, const std::shared_ptr<SensorStateCore> pastState, std::shared_ptr<SensorStateCore>& predictedState);

    // Jacobian of the error state: F
public:
    int predictErrorStateJacobians(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, std::shared_ptr<SensorStateCore> pastState, std::shared_ptr<SensorStateCore>& predictedState);




    //// Update functions


    /// State Correction

    // Prediction measurements: h
public:
    int predictMeasurement(const TimeStamp theTimeStamp, std::shared_ptr<GlobalParametersStateCore> TheGlobalParametersStateCore, const std::shared_ptr<RobotStateCore> currentRobotState, const std::shared_ptr<ImuSensorStateCore> currentImuState, std::shared_ptr<ImuSensorMeasurementCore>& predictedMeasurement);


    // Jacobian of the measurements: H
public:
    int jacobiansErrorMeasurements(const TimeStamp theTimeStamp, std::shared_ptr<GlobalParametersStateCore> TheGlobalParametersStateCore, std::shared_ptr<RobotStateCore> TheRobotStateCore, std::shared_ptr<ImuSensorStateCore> TheImuStateCore, std::shared_ptr<ImuSensorMeasurementCore>& predictedMeasurement);


    /// Mapping

    // None


};




#endif
