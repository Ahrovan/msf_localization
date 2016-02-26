
#ifndef _IMU_SENSOR_CORE_H
#define _IMU_SENSOR_CORE_H



#include "sensor_core.h"


// Time Stamp
#include "time_stamp.h"


// Measurement
#include "imu_sensor_measurement_core.h"

// State
#include "imu_sensor_state_core.h"






class ImuSensorCore : public virtual SensorCore
{
public:
    ImuSensorCore();
    ~ImuSensorCore();


    ///// Measurements

    // Orientation Measurement
protected:
    bool flagMeasurementOrientation;
public:
    bool isOrientationEnabled() const;
    int enableOrientation();

    // Orientation measurement Covariance
protected:


    // Angular Velocity Measurement
protected:
    bool flagMeasurementAngularVelocity;
public:
    bool isAngularVelocityEnabled() const;
    int enableAngularVelocity();

    // Angular velocity measurement convariance
protected:


    // Linear Acceleration Measurement
protected:
    bool flagMeasurementLinearAcceleration;
public:
    bool isLinearAccelerationEnabled() const;
    int enableLinearAcceleration();

    // Linear acceleration measurement covariance
protected:


    // Store Measurement
public:
    int setMeasurement(const TimeStamp TheTimeStamp, std::shared_ptr<ImuSensorMeasurementCore> TheImuSensorMeasurement);




    ///// State estimation


//public:
//    unsigned int getDimensionState() const;

//public:
//    unsigned int getDimensionErrorState() const;


    // Angular Velocity Biases: bwx, bwy, bwz
protected:
    bool flagEstimationBiasAngularVelocity;
public:
    bool isEstimationBiasAngularVelocityEnabled() const;
    int enableEstimationBiasAngularVelocity();

    // Angular Velocity Covariance
protected:
    Eigen::Matrix3d noiseBiasAngularVelocity;
public:
    Eigen::Matrix3d getNoiseBiasAngularVelocity() const;
    int setNoiseBiasAngularVelocity(Eigen::Matrix3d noiseBiasAngularVelocity);



    // Linear Acceleration Biases: bax, bay, baz
protected:
    bool flagEstimationBiasLinearAcceleration;
public:
    bool isEstimationBiasLinearAccelerationEnabled() const;
    int enableEstimationBiasLinearAcceleration();

    // Linear Acceleration covariance
protected:
    Eigen::Matrix3d noiseBiasLinearAcceleration;
public:
    Eigen::Matrix3d getNoiseBiasLinearAcceleration() const;
    int setNoiseBiasLinearAcceleration(Eigen::Matrix3d noiseBiasLinearAcceleration);




    ///// Predict functions

    // State: xs=[posi_sensor_wrt_robot, att_sensor_wrt_robot, bias_lin_accel, bias_ang_veloc]'

    // Prediction state function
public:
    int predictState(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, const std::shared_ptr<ImuSensorStateCore> pastState, std::shared_ptr<ImuSensorStateCore>& predictedState);

    // Jacobian
public:
    int predictStateErrorStateJacobians(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, std::shared_ptr<ImuSensorStateCore> pastState, std::shared_ptr<ImuSensorStateCore>& predictedState);



    // Prediction measurements
public:
    int predictMeasurement(TimeStamp theTimeStamp, const std::shared_ptr<ImuSensorStateCore> currentState, std::shared_ptr<ImuSensorMeasurementCore> predictedMeasurement);




};




#endif
