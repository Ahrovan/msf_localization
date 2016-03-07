
#include "msf_localization_core/free_model_robot_state_core.h"


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

Eigen::Vector3d FreeModelRobotStateCore::getAngularAcceleration() const
{
    return this->angular_acceleration;
}

int FreeModelRobotStateCore::setAngularAcceleration(Eigen::Vector3d angular_acceleration)
{
    this->angular_acceleration=angular_acceleration;
    return 0;
}


int FreeModelRobotStateCore::updateStateFromIncrementErrorState(Eigen::VectorXd increment_error_state)
{

    position+=increment_error_state.block<3,1>(0,0);
    linear_speed+=increment_error_state.block<3,1>(3,0);
    linear_acceleration+=increment_error_state.block<3,1>(6,0);


    Eigen::Vector4d DeltaQuat;
    DeltaQuat[0]=1;
    DeltaQuat.block<3,1>(1,0)=0.5*increment_error_state.block<3,1>(9,0);
    DeltaQuat=DeltaQuat/DeltaQuat.norm();

    attitude=Quaternion::cross(attitude, DeltaQuat);
    angular_velocity+=increment_error_state.block<3,1>(12,0);
    angular_acceleration+=increment_error_state.block<3,1>(15,0);


    return 0;
}
