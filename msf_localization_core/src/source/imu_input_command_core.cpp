
#include "msf_localization_core/imu_input_command_core.h"


ImuInputCommandCore::ImuInputCommandCore() :
    InputCommandCore()
{
    init();

    return;
}

ImuInputCommandCore::ImuInputCommandCore(std::weak_ptr<InputCore> input_core_ptr) :
    InputCommandCore(input_core_ptr)
{
    init();

    return;
}

ImuInputCommandCore::~ImuInputCommandCore()
{

    return;
}

int ImuInputCommandCore::init()
{
    // Variables
    linear_acceleration_.setZero();
    orientation_.setZero();
    angular_velocity_.setZero();

    // Type
    setInputCommandType(InputCommandTypes::imu);

    return 0;
}

int ImuInputCommandCore::setOrientation(const Eigen::Vector4d orientation)
{
    this->orientation_=orientation;
    return 0;
}

Eigen::Vector4d ImuInputCommandCore::getOrientation() const
{
    return this->orientation_;
}

int ImuInputCommandCore::setAngularVelocity(Eigen::Vector3d angular_velocity)
{
    this->angular_velocity_=angular_velocity;
    return 0;
}

Eigen::Vector3d ImuInputCommandCore::getAngularVelocity() const
{
    return this->angular_velocity_;
}

int ImuInputCommandCore::setLinearAcceleration(Eigen::Vector3d linear_acceleration)
{
    this->linear_acceleration_=linear_acceleration;
    return 0;
}

Eigen::Vector3d ImuInputCommandCore::getLinearAcceleration() const
{
    return this->linear_acceleration_;
}
