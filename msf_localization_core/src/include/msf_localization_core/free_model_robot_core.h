
#ifndef _FREE_MODEL_ROBOT_CORE_H
#define _FREE_MODEL_ROBOT_CORE_H


// Math
#include "cmath"

// Time Stamp
#include "msf_localization_core/time_stamp.h"

// Quaternion algebra
#include "msf_localization_core/quaternion_algebra.h"

// Robot Core
#include "msf_localization_core/robot_core.h"

// State
#include "msf_localization_core/free_model_robot_state_core.h"


class FreeModelRobotCore : public RobotCore
{

public:
    FreeModelRobotCore();
    ~FreeModelRobotCore();



    ///// Prediction Noises

    // Noise Linear Acceleration
protected:
public:
    Eigen::Matrix3d noiseLinearAcceleration;
public:
    Eigen::Matrix3d getNoiseLinearAcceleration() const;
    int setNoiseLinearAcceleration(Eigen::Matrix3d noiseLinearAcceleration);


    // Noise Angular Velocity -> NOT USED!
protected:
public:
    Eigen::Matrix3d noiseAngularVelocity;
public:
    Eigen::Matrix3d getNoiseAngularVelocity() const;
    int setNoiseAngularVelocity(Eigen::Matrix3d noiseAngularVelocity);


    // Noise Angular Acceleration
protected:
public:
    Eigen::Matrix3d noiseAngularAcceleration;
public:
    Eigen::Matrix3d getNoiseAngularAcceleration() const;
    int setNoiseAngularAcceleration(Eigen::Matrix3d noiseAngularAcceleration);




public:
    Eigen::SparseMatrix<double> getCovarianceNoise(const TimeStamp deltaTimeStamp);



    ////// Init error state variances -> Temporal, only for the initial configuration

public:
    int setInitErrorStateVariancePosition(Eigen::Vector3d initVariance);
    int setInitErrorStateVarianceLinearSpeed(Eigen::Vector3d initVariance);
    int setInitErrorStateVarianceLinearAcceleration(Eigen::Vector3d initVariance);
    int setInitErrorStateVarianceAttitude(Eigen::Vector3d initVariance);
    int setInitErrorStateVarianceAngularVelocity(Eigen::Vector3d initVariance);
    int setInitErrorStateVarianceAngularAcceleration(Eigen::Vector3d initVariance);





    ///// Predict functions

    // State: xR=[pos, lin_speed, lin_accel, attit, ang_vel, ang_accel]'

    // Prediction state function
public:
    int predictState(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, const std::shared_ptr<RobotStateCore> pastState, std::shared_ptr<RobotStateCore>& predictedState);

    // Jacobian
public:
    int predictStateErrorStateJacobians(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, std::shared_ptr<RobotStateCore> pastState, std::shared_ptr<RobotStateCore>& predictedState);




};





#endif
