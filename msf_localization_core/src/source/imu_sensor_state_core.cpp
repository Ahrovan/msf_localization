
#include "imu_sensor_state_core.h"


ImuSensorStateCore::ImuSensorStateCore()
{
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


Eigen::Vector3d ImuSensorStateCore::getBiasesLinearAcceleration() const
{
    return this->biasesLinearAcceleration;
}

int ImuSensorStateCore::setBiasesLinearAcceleration(Eigen::Vector3d biasesLinearAcceleration)
{
    this->biasesLinearAcceleration=biasesLinearAcceleration;
    return 0;
}
