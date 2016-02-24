
#include "free_model_robot_state_core.h"


FreeModelRobotStateCore::FreeModelRobotStateCore()
{
    //this->errorStateJacobian.resize();

    return;
}

FreeModelRobotStateCore::~FreeModelRobotStateCore()
{
    return;
}

Eigen::Vector3d FreeModelRobotStateCore::getPosition() const
{
    return this->position;
}

int FreeModelRobotStateCore::setPosition(Eigen::Vector3d position)
{
    this->position=position;
    return 0;
}

Eigen::Vector3d FreeModelRobotStateCore::getLinearSpeed() const
{
    return this->linear_speed;
}

int FreeModelRobotStateCore::setLinearSpeed(Eigen::Vector3d linear_speed)
{
    this->linear_speed=linear_speed;
    return 0;
}

Eigen::Vector3d FreeModelRobotStateCore::getLinearAcceleration() const
{
    return this->linear_acceleration;
}

int FreeModelRobotStateCore::setLinearAcceleration(Eigen::Vector3d linear_acceleration)
{
    this->linear_acceleration=linear_acceleration;
    return 0;
}

Eigen::Vector4d FreeModelRobotStateCore::getAttitude() const
{
    return this->attitude;
}

int FreeModelRobotStateCore::setAttitude(Eigen::Vector4d attitude)
{
    this->attitude=attitude;
    return 0;
}

Eigen::Vector3d FreeModelRobotStateCore::getAngularVelocity() const
{
    return this->angular_velocity;
}

int FreeModelRobotStateCore::setAngularVelocity(Eigen::Vector3d angular_velocity)
{
    this->angular_velocity=angular_velocity;
    return 0;
}
