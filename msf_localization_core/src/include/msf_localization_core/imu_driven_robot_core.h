
#ifndef _IMU_DRIVEN_ROBOT_CORE_H
#define _IMU_DRIVEN_ROBOT_CORE_H


// Math
#include "cmath"

// Time Stamp
#include "time_stamp/time_stamp.h"

// Quaternion algebra
#include "quaternion_algebra/quaternion_algebra.h"

// Robot Core
#include "msf_localization_core/robot_core.h"

// State
#include "msf_localization_core/imu_driven_robot_state_core.h"


// Input
#include "msf_localization_core/imu_input_command_core.h"


#include "pugixml/pugixml.hpp"



class ImuDrivenRobotCore : public RobotCore
{

public:
    ImuDrivenRobotCore();
    ImuDrivenRobotCore(MsfLocalizationCore* msf_localization_core_ptr);
    ~ImuDrivenRobotCore();


protected:
    int init();


public:
    int readConfig(const pugi::xml_node& robot, std::shared_ptr<ImuDrivenRobotStateCore>& robot_init_state);


    ///// Noise State / Parameters

    // Position Robot Wrt World
protected:
    Eigen::Matrix3d noise_position_robot_wrt_world_;
public:
    int setNoisePositionRobotWrtWorld(const Eigen::Matrix3d& noise_position_robot_wrt_world);
    Eigen::Matrix3d getNoisePositionRobotWrtWorld();



    // Linear Speed Robot Wrt World
protected:
    Eigen::Matrix3d noise_linear_speed_robot_wrt_world_;
public:
    int setNoiseLinearSpeedRobotWrtWorld(const Eigen::Matrix3d& noise_linear_speed_robot_wrt_world);
    Eigen::Matrix3d getNoiseLinearSpeedRobotWrtWorld();


    // Linear Acceleration Robot Wrt World
protected:
    Eigen::Matrix3d noise_linear_acceleration_robot_wrt_world_;
public:
    int setNoiseLinearAccelerationRobotWrtWorld(const Eigen::Matrix3d& noise_linear_acceleration_robot_wrt_world);
    Eigen::Matrix3d getNoiseLinearAccelerationRobotWrtWorld();


    // Attitude Robot Wrt World
protected:
    Eigen::Matrix3d noise_attitude_robot_wrt_world_;
public:
    int setNoiseAttitudeRobotWrtWorld(const Eigen::Matrix3d& noise_attitude_robot_wrt_world);
    Eigen::Matrix3d getNoiseAttitudeRobotWrtWorld();


    // Angular Velocity Robot Wrt World
protected:
    Eigen::Matrix3d noise_angular_velocity_robot_wrt_world_;
public:
    int setNoiseAngularVelocityRobotWrtWorld(const Eigen::Matrix3d& noise_angular_velocity_robot_wrt_world);
    Eigen::Matrix3d getNoiseAngularVelocityRobotWrtWorld();



    ///// Noises Predict
    // None



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



    ///// Predict Step Functions

    // State: xR=[pos, lin_speed, attit]'

    // TODO

    // Prediction state function: f()
public:
    // TODO
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
                             const std::shared_ptr<ImuDrivenRobotStateCore>& pastState,
                             const std::shared_ptr<ImuInputCommandCore>& input,
                             std::shared_ptr<ImuDrivenRobotStateCore>& predictedState);
    // int predictStateSpecificCore();



    // Jacobian: F
public:
    // TODO
    int predictErrorStateJacobian(//Time
                                 const TimeStamp& previousTimeStamp, const TimeStamp& currentTimeStamp,
                                 // Previous State
                                 const std::shared_ptr<StateComponent>& pastState,
                                  // Inputs
                                  const std::shared_ptr<InputCommandComponent>& inputCommand,
                                 // Predicted State
                                 std::shared_ptr<StateCore>& predictedState);

protected:
    int predictErrorStateJacobianSpecific(const TimeStamp& previousTimeStamp, const TimeStamp& currentTimeStamp,
                                           const std::shared_ptr<ImuDrivenRobotStateCore>& pastState,
                                           const std::shared_ptr<ImuInputCommandCore>& input,
                                           std::shared_ptr<ImuDrivenRobotStateCore>& predictedState);

// int predictErrorStateJacobianSpecificCore();



    ///// Update Step Functions

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
