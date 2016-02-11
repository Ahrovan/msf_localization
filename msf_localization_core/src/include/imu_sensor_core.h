
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


    // Pose of the sensor wrt robot
protected:
    bool flagEstimationAttitudeSensorWrtRobot;

    // Covariance


protected:
    bool flagEstimationPositionSensorWrtRobot;

    // Covariance



    // Angular Velocity Biases: bwx, bwy, bwz
protected:
    bool flagEstimationBiasAngularVelocity;

    // Angular Velocity Covariance
protected:



    // Linear Acceleration Biases: bax, bay, baz
protected:
    bool flagEstimationBiasLinearAcceleration;

    // Linear Acceleration covariance
protected:



    ///// Predict functions

    // State

    // Prediction state function
public:
    int predictState(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, const std::shared_ptr<ImuSensorStateCore> pastState, std::shared_ptr<ImuSensorStateCore>& predictedState);

    // Jacobian
public:
    int predictStateJacobians(TimeStamp theTimeStamp, std::shared_ptr<ImuSensorStateCore> currentState);



    // Prediction measurements
public:
    int predictMeasurement(TimeStamp theTimeStamp, const std::shared_ptr<ImuSensorStateCore> currentState, std::shared_ptr<ImuSensorMeasurementCore> predictedMeasurement);




};




#endif
