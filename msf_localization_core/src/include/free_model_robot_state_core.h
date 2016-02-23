
#ifndef _FREE_MODEL_ROBOT_STATE_CORE_H
#define _FREE_MODEL_ROBOT_STATE_CORE_H



#include <Eigen/Dense>


#include "robot_state_core.h"





class FreeModelRobotStateCore : public RobotStateCore
{
public:
    FreeModelRobotStateCore();
    ~FreeModelRobotStateCore();



protected:
public:
    Eigen::Vector3d position;
public:
    Eigen::Vector3d getPosition() const;
    int setPosition(Eigen::Vector3d position);


protected:
public:
    Eigen::Vector3d linear_speed;
public:
    Eigen::Vector3d getLinearSpeed() const;
    int setLinearSpeed(Eigen::Vector3d linear_speed);


protected:
public:
    Eigen::Vector3d linear_acceleration;
public:
    Eigen::Vector3d getLinearAcceleration() const;
    int setLinearAcceleration(Eigen::Vector3d linear_acceleration);


protected:
public:
    Eigen::Vector4d attitude;
public:
    Eigen::Vector4d getAttitude() const;
    int setAttitude(Eigen::Vector4d attitude);


protected:
public:
    Eigen::Vector3d angular_velocity;
public:
    Eigen::Vector3d getAngularVelocity() const;
    int setAngularVelocity(Eigen::Vector3d angular_velocity);



};





#endif
