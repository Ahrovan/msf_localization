
#ifndef _IMU_DRIVEN_ROBOT_STATE_CORE_H
#define _IMU_DRIVEN_ROBOT_STATE_CORE_H


#include <Eigen/Dense>
#include <Eigen/Sparse>


#include "msf_localization_core/robot_state_core.h"


#include "msf_localization_core/quaternion_algebra.h"




class ImuDrivenRobotStateCore : public RobotStateCore
{
public:
    ImuDrivenRobotStateCore();
    ImuDrivenRobotStateCore(std::weak_ptr<MsfElementCore> msf_element_core_ptr);
    ~ImuDrivenRobotStateCore();

protected:
    int init();


    // State: xR=[pos (3/3), lin_speed (3/3), lin_acc (3/3), attit (4/3), ang_vel (3/3)]'

protected:
public:
    Eigen::Vector3d position_robot_wrt_world_;
public:
    Eigen::Vector3d getPositionRobotWrtWorld() const;
    int setPositionRobotWrtWorld(Eigen::Vector3d position_robot_wrt_world);


protected:
public:
    Eigen::Vector3d linear_speed_robot_wrt_world_;
public:
    Eigen::Vector3d getLinearSpeedRobotWrtWorld() const;
    int setLinearSpeedRobotWrtWorld(Eigen::Vector3d linear_speed_robot_wrt_world);


protected:
public:
    Eigen::Vector3d linear_acceleration_robot_wrt_world_;
public:
    Eigen::Vector3d getLinearAccelerationRobotWrtWorld() const;
    int setLinearAccelerationRobotWrtWorld(Eigen::Vector3d linear_acceleration_robot_wrt_world);


protected:
public:
    Eigen::Vector4d attitude_robot_wrt_world_;
public:
    Eigen::Vector4d getAttitudeRobotWrtWorld() const;
    int setAttitudeRobotWrtWorld(Eigen::Vector4d attitude_robot_wrt_world);


protected:
public:
    Eigen::Vector3d angular_velocity_robot_wrt_world_;
public:
    Eigen::Vector3d getAngularVelocityRobotWrtWorld() const;
    int setAngularVelocityRobotWrtWorld(Eigen::Vector3d angular_velocity_robot_wrt_world);




public:
    int updateStateFromIncrementErrorState(Eigen::VectorXd increment_error_state);


};



#endif
