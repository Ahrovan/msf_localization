
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


#include "pugixml/pugixml.hpp"



class FreeModelRobotCore : public RobotCore
{

public:
    FreeModelRobotCore();
    FreeModelRobotCore(MsfLocalizationCore* msf_localization_core_ptr);
    ~FreeModelRobotCore();


protected:
    int init();


public:
    int readConfig(const pugi::xml_node& robot, std::shared_ptr<FreeModelRobotStateCore>& RobotInitStateCore);



    ///// Prediction Noises


    // Noise Position
protected:
public:
    Eigen::Matrix3d noisePosition;
public:
    Eigen::Matrix3d getNoisePosition() const;
    int setNoisePosition(const Eigen::Matrix3d& noisePosition);


    // Noise Linear Speed
protected:
public:
    Eigen::Matrix3d noiseLinearSpeed;
public:
    Eigen::Matrix3d getNoiseLinearSpeed() const;
    int setNoiseLinearSpeed(const Eigen::Matrix3d& noiseLinearSpeed);


    // Noise Linear Acceleration
protected:
public:
    Eigen::Matrix3d noiseLinearAcceleration;
public:
    Eigen::Matrix3d getNoiseLinearAcceleration() const;
    int setNoiseLinearAcceleration(const Eigen::Matrix3d& noiseLinearAcceleration);


    // Noise Attitude
protected:
public:
    Eigen::Matrix3d noiseAttitude;
public:
    Eigen::Matrix3d getNoiseAttitude() const;
    int setNoiseAttitude(const Eigen::Matrix3d& noiseAttitude);


    // Noise Angular Velocity
protected:
public:
    Eigen::Matrix3d noiseAngularVelocity;
public:
    Eigen::Matrix3d getNoiseAngularVelocity() const;
    int setNoiseAngularVelocity(const Eigen::Matrix3d& noiseAngularVelocity);


    // Noise Angular Acceleration
protected:
public:
    Eigen::Matrix3d noiseAngularAcceleration;
public:
    Eigen::Matrix3d getNoiseAngularAcceleration() const;
    int setNoiseAngularAcceleration(const Eigen::Matrix3d& noiseAngularAcceleration);




    ///// Covariances Getters

    // Covariance Error Parameters: Rp = Qp
public:
    Eigen::SparseMatrix<double> getCovarianceParameters();

    // Covariance Noise Estimation: Qn
public:
    Eigen::SparseMatrix<double> getCovarianceNoise(const TimeStamp deltaTimeStamp);



    ////// Init error state variances -> Temporal, only for the initial configuration

public:
    int prepareCovarianceInitErrorStateSpecific();

public:
    int setInitErrorStateVariancePosition(const Eigen::Vector3d& initVariance);
    int setInitErrorStateVarianceLinearSpeed(const Eigen::Vector3d &initVariance);
    int setInitErrorStateVarianceLinearAcceleration(const Eigen::Vector3d& initVariance);
    int setInitErrorStateVarianceAttitude(const Eigen::Vector3d &initVariance);
    int setInitErrorStateVarianceAngularVelocity(const Eigen::Vector3d& initVariance);
    int setInitErrorStateVarianceAngularAcceleration(const Eigen::Vector3d& initVariance);






    ///// Predict Step functions

    // State: xR=[pos, lin_speed, lin_accel, attit, ang_vel, ang_accel]'

    // Prediction state function: f()
public:
    int predictState(//Time
                     const TimeStamp& previousTimeStamp, const TimeStamp& currentTimeStamp,
                     // Previous State
                     const std::shared_ptr<StateEstimationCore>& pastState,
                     // Inputs
                     const std::shared_ptr<InputCommandComponent>& inputCommand,
                     // Predicted State
                     std::shared_ptr<StateCore>& predictedState);

protected:
    int predictStateSpecific(const TimeStamp& previousTimeStamp, const TimeStamp& currentTimeStamp,
                             const FreeModelRobotStateCore* pastState,
                             FreeModelRobotStateCore*& predictedState);

    // int predictStateSpecificCore();


    // Jacobian: F
public:
    int predictErrorStateJacobian(//Time
                                 const TimeStamp& previousTimeStamp, const TimeStamp& currentTimeStamp,
                                 // Previous State
                                 const std::shared_ptr<StateEstimationCore>& past_state,
                                  // Inputs
                                  const std::shared_ptr<InputCommandComponent>& input_command,
                                 // Predicted State
                                 std::shared_ptr<StateCore>& predicted_state);

protected:
    int predictErrorStateJacobianSpecific(const TimeStamp& previousTimeStamp, const TimeStamp& currentTimeStamp,
                                          const FreeModelRobotStateCore* pastState,
                                          const FreeModelRobotStateCore* predictedState,
                                          // Jacobians Error State: Fx, Fp
                                          // Robot
                                          Eigen::SparseMatrix<double>& jacobian_error_state_wrt_robot_error_state,
                                          Eigen::SparseMatrix<double>& jacobian_error_state_wrt_robot_error_parameters,
                                          // Jacobians Noise: Hn
                                          Eigen::SparseMatrix<double>& jacobian_error_state_wrt_noise
                                          );

    // int predictErrorStateJacobianSpecificCore();


    //// Update Step Functions

    // None

public:
    int resetErrorStateJacobian(// Time
                                const TimeStamp& current_time_stamp,
                                // Increment Error State
                                const Eigen::VectorXd& increment_error_state,
                                // Current State
                                std::shared_ptr<StateCore>& current_state
                                );


};





#endif
