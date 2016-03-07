
#include "msf_localization_core/imu_sensor_state_core.h"


#include "msf_localization_core/imu_sensor_core.h"


ImuSensorStateCore::ImuSensorStateCore()
{
    // Error State Jacobian: Init to zero
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

int ImuSensorStateCore::updateStateFromIncrementErrorState(Eigen::VectorXd increment_error_state)
{
    unsigned int dimension=0;

    std::shared_ptr<const ImuSensorCore> TheImuSensorCore=std::dynamic_pointer_cast<const ImuSensorCore>(this->getTheSensorCore());

    if(TheImuSensorCore->isEstimationPositionSensorWrtRobotEnabled())
    {
        this->positionSensorWrtRobot+=increment_error_state.block<3,1>(dimension, 0);
        dimension+=3;
    }
    if(TheImuSensorCore->isEstimationAttitudeSensorWrtRobotEnabled())
    {
        Eigen::Vector4d DeltaQuat;
        DeltaQuat[0]=1;
        DeltaQuat.block<3,1>(1,0)=0.5*increment_error_state.block<3,1>(dimension,0);
        DeltaQuat=DeltaQuat/DeltaQuat.norm();

        this->attitudeSensorWrtRobot=Quaternion::cross(this->attitudeSensorWrtRobot, DeltaQuat);
        dimension+=3;
    }
    if(TheImuSensorCore->isEstimationBiasLinearAccelerationEnabled())
    {
        this->biasesLinearAcceleration+=increment_error_state.block<3,1>(dimension, 0);
        dimension+=3;
    }
    if(TheImuSensorCore->isEstimationScaleLinearAccelerationEnabled())
    {
        this->scaleLinearAcceleration+=increment_error_state.block<3,1>(dimension, 0);
        dimension+=3;
    }
    if(TheImuSensorCore->isEstimationBiasAngularVelocityEnabled())
    {
        this->biasesAngularVelocity+=increment_error_state.block<3,1>(dimension, 0);
        dimension+=3;
    }
    if(TheImuSensorCore->isEstimationScaleAngularVelocityEnabled())
    {
        this->scaleAngularVelocity+=increment_error_state.block<3,1>(dimension, 0);
        dimension+=3;
    }


    return 0;
}
