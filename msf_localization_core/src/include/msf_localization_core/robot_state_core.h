
#ifndef _ROBOT_STATE_CORE_H
#define _ROBOT_STATE_CORE_H


#include "msf_localization_core/state_core.h"

#include <iostream>

#include <Eigen/Dense>
#include <Eigen/Sparse>



enum class RobotStateCoreTypes
{
    undefined=0,
    free_model=1,
    imu_driven
};




class RobotStateCore : public StateCore
{
public:
    RobotStateCore();
    RobotStateCore(const std::weak_ptr<MsfElementCore> msf_element_core_ptr);
    ~RobotStateCore();


protected:
    int init();


protected:
    RobotStateCoreTypes robot_state_core_type_;
public:
    int setRobotStateType(RobotStateCoreTypes robot_state_core_type);
    RobotStateCoreTypes getRobotStateType();



public:
    virtual Eigen::Vector3d getPositionRobotWrtWorld() const;
    virtual int setPositionRobotWrtWorld(const Eigen::Vector3d& position);

public:
    virtual Eigen::Vector3d getLinearSpeedRobotWrtWorld() const;
    virtual int setLinearSpeedRobotWrtWorld(const Eigen::Vector3d& linear_speed);

public:
    virtual Eigen::Vector3d getLinearAccelerationRobotWrtWorld() const;
    virtual int setLinearAccelerationRobotWrtWorld(const Eigen::Vector3d& linear_acceleration);

public:
    virtual Eigen::Vector4d getAttitudeRobotWrtWorld() const;
    virtual int setAttitudeRobotWrtWorld(const Eigen::Vector4d& attitude);

public:
    virtual Eigen::Vector3d getAngularVelocityRobotWrtWorld() const;
    virtual int setAngularVelocityRobotWrtWorld(const Eigen::Vector3d& angular_velocity);

public:
    virtual Eigen::Vector3d getAngularAccelerationRobotWrtWorld() const;
    virtual int setAngularAccelerationRobotWrtWorld(const Eigen::Vector3d& angular_acceleration);



protected:
    // TODO predictState()





};



#endif
