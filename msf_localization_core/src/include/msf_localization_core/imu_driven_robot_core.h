
#ifndef _IMU_DRIVEN_ROBOT_CORE_H
#define _IMU_DRIVEN_ROBOT_CORE_H


// Math
#include "cmath"

// Time Stamp
#include "msf_localization_core/time_stamp.h"

// Quaternion algebra
#include "msf_localization_core/quaternion_algebra.h"

// Robot Core
#include "msf_localization_core/robot_core.h"

// State
#include "msf_localization_core/imu_driven_robot_state_core.h"


#include "pugixml/pugixml.hpp"



class ImuDrivenRobotCore : public RobotCore
{

public:
    ImuDrivenRobotCore();
    ImuDrivenRobotCore(std::weak_ptr<MsfStorageCore> msf_storage_core_ptr);
    ~ImuDrivenRobotCore();


protected:
    int init();


public:
    int readConfig(pugi::xml_node robot, std::shared_ptr<ImuDrivenRobotStateCore>& robot_init_state);


    ///// Noise State / Parameters

    // Position Robot Wrt World
protected:
    Eigen::Matrix3d noise_position_robot_wrt_world_;
public:
    int setNoisePositionRobotWrtWorld(Eigen::Matrix3d noise_position_robot_wrt_world);
    Eigen::Matrix3d getNoisePositionRobotWrtWorld();



    // Linear Speed Robot Wrt World
protected:
    Eigen::Matrix3d noise_linear_speed_robot_wrt_world_;
public:
    int setNoiseLinearSpeedRobotWrtWorld(Eigen::Matrix3d noise_linear_speed_robot_wrt_world);
    Eigen::Matrix3d getNoiseLinearSpeedRobotWrtWorld();


    // Linear Acceleration Robot Wrt World
protected:
    Eigen::Matrix3d noise_linear_acceleration_robot_wrt_world_;
public:
    int setNoiseLinearAccelerationRobotWrtWorld(Eigen::Matrix3d noise_linear_acceleration_robot_wrt_world);
    Eigen::Matrix3d getNoiseLinearAccelerationRobotWrtWorld();


    // Attitude Robot Wrt World
protected:
    Eigen::Matrix3d noise_attitude_robot_wrt_world_;
public:
    int setNoiseAttitudeRobotWrtWorld(Eigen::Matrix3d noise_attitude_robot_wrt_world);
    Eigen::Matrix3d getNoiseAttitudeRobotWrtWorld();


    // Angular Velocity Robot Wrt World
protected:
    Eigen::Matrix3d noise_angular_velocity_robot_wrt_world_;
public:
    int setNoiseAngularVelocityRobotWrtWorld(Eigen::Matrix3d noise_angular_velocity_robot_wrt_world);
    Eigen::Matrix3d getNoiseAngularVelocityRobotWrtWorld();



    ///// Noises Predict
    // None



    ///// Covariances Getters

    // Covariance Error Parameters: Rp = Qp
public:
    //Eigen::SparseMatrix<double> getCovarianceParameters();

    // Covariance Noise Estimation: Qn
public:
    Eigen::SparseMatrix<double> getCovarianceNoise(const TimeStamp deltaTimeStamp);



    ////// Init error state variances -> Temporal, only for the initial configuration

public:
    int prepareCovarianceInitErrorStateSpecific();



    ///// Predict Step Functions

    // State: xR=[pos, lin_speed, attit]'

    // TODO

    // Prediction state function
public:
    int predictState(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, const std::shared_ptr<RobotStateCore> pastState, std::shared_ptr<RobotStateCore>& predictedState);

    // Jacobian
public:
    int predictErrorStateJacobians(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, std::shared_ptr<RobotStateCore> pastState, std::shared_ptr<RobotStateCore>& predictedState);



    ///// Update Step Functions

    // None
};



#endif
