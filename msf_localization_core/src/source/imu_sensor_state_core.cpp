
#include "msf_localization_core/imu_sensor_state_core.h"


ImuSensorStateCore::ImuSensorStateCore()
{
    // Init to zero
    errorStateJacobian.positionSensorWrtRobot.setZero();
    errorStateJacobian.attitudeSensorWrtRobot.setZero();
    errorStateJacobian.biasesLinearAcceleration.setZero();
    errorStateJacobian.biasesAngularVelocity.setZero();

    return;
}

ImuSensorStateCore::~ImuSensorStateCore()
{
    return;
}


Eigen::Vector3d ImuSensorStateCore::getBiasesAngularVelocity() const
{
    return this->biasesAngularVelocity;
}

int ImuSensorStateCore::setBiasesAngularVelocity(Eigen::Vector3d biasesAngularVelocity)
{
    this->biasesAngularVelocity=biasesAngularVelocity;
    return 0;
}

Eigen::Vector3d ImuSensorStateCore::getScaleAngularVelocity() const
{
    return this->scaleAngularVelocity;
}

int ImuSensorStateCore::setScaleAngularVelocity(Eigen::Vector3d scaleAngularVelocity)
{
    this->scaleAngularVelocity=scaleAngularVelocity;
    return 0;
}


Eigen::Vector3d ImuSensorStateCore::getBiasesLinearAcceleration() const
{
    return this->biasesLinearAcceleration;
}

int ImuSensorStateCore::setBiasesLinearAcceleration(Eigen::Vector3d biasesLinearAcceleration)
{
    this->biasesLinearAcceleration=biasesLinearAcceleration;
    return 0;
}

Eigen::Vector3d ImuSensorStateCore::getScaleLinearAcceleration() const
{
    return this->scaleLinearAcceleration;
}

int ImuSensorStateCore::setScaleLinearAcceleration(Eigen::Vector3d scaleLinearAcceleration)
{
    this->scaleLinearAcceleration=scaleLinearAcceleration;
    return 0;
}
