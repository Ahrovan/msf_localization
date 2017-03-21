

#ifndef _IMU_SENSOR_STATE_CORE_H
#define _IMU_SENSOR_STATE_CORE_H


#include <Eigen/Dense>


#include "msf_localization_core/sensor_state_core.h"

#include "quaternion_algebra/quaternion_algebra.h"



class ImuSensorStateCore : public SensorStateCore
{
public:
    ImuSensorStateCore();
    ImuSensorStateCore(const std::weak_ptr<MsfElementCore> msf_element_core_ptr);
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
    int setBiasesAngularVelocity(const Eigen::Vector3d& biasesAngularVelocity);


    // Angular Velocity Scale
protected:
public:
    Eigen::Vector3d scaleAngularVelocity;
public:
    Eigen::Vector3d getScaleAngularVelocity() const;
    int setScaleAngularVelocity(const Eigen::Vector3d &scaleAngularVelocity);


    // Linear Acceleration Biases   
protected:
public:
    Eigen::Vector3d biasesLinearAcceleration;
public:
    Eigen::Vector3d getBiasesLinearAcceleration() const;
    int setBiasesLinearAcceleration(const Eigen::Vector3d& biasesLinearAcceleration);


    // Linear Acceleration Scale
protected:
public:
    Eigen::Vector3d scaleLinearAcceleration;
public:
    Eigen::Vector3d getScaleLinearAcceleration() const;
    int setScaleLinearAcceleration(const Eigen::Vector3d &scaleLinearAcceleration);





public:
    int updateStateFromIncrementErrorState(const Eigen::VectorXd& increment_error_state);




};








#endif
