
#include "msf_localization_core/free_model_robot_state_core.h"

#include "msf_localization_core/free_model_robot_core.h"


FreeModelRobotStateCore::FreeModelRobotStateCore() :
    RobotStateCore()
{
    init();

    return;
}

FreeModelRobotStateCore::FreeModelRobotStateCore(std::weak_ptr<MsfElementCore> msf_element_core_ptr) :
    RobotStateCore(msf_element_core_ptr)
{
    init();

    return;
}

FreeModelRobotStateCore::~FreeModelRobotStateCore()
{
    return;
}

int FreeModelRobotStateCore::init()
{
    this->setRobotStateType(RobotStateCoreTypes::free_model);

    return 0;
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



int FreeModelRobotStateCore::updateStateFromIncrementErrorState(const Eigen::VectorXd &increment_error_state)
{

    position+=increment_error_state.block<3,1>(0,0);
    linear_speed+=increment_error_state.block<3,1>(3,0);
    linear_acceleration+=increment_error_state.block<3,1>(6,0);


    Eigen::Vector4d DeltaQuat, DeltaQuatAux;
    double normDeltaQuatAux;
    DeltaQuatAux[0]=1;
    DeltaQuatAux.block<3,1>(1,0)=0.5*increment_error_state.block<3,1>(9,0);
    normDeltaQuatAux=DeltaQuatAux.norm();
    DeltaQuat=DeltaQuatAux/normDeltaQuatAux;

    Eigen::Vector4d attitudeAux=Quaternion::cross(attitude, DeltaQuat);


    attitude=attitudeAux;


    angular_velocity+=increment_error_state.block<3,1>(12,0);
    angular_acceleration+=increment_error_state.block<3,1>(15,0);


    return 0;
}
