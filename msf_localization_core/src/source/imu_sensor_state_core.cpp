
#include "msf_localization_core/imu_sensor_state_core.h"


#include "msf_localization_core/imu_sensor_core.h"


ImuSensorStateCore::ImuSensorStateCore() :
    SensorStateCore()
{
    init();

    return;
}

ImuSensorStateCore::ImuSensorStateCore(const std::weak_ptr<MsfElementCore> msf_element_core_ptr) :
    SensorStateCore(msf_element_core_ptr)
{
    init();

    return;
}

ImuSensorStateCore::~ImuSensorStateCore()
{
    return;
}

int ImuSensorStateCore::init()
{
    this->setSensorStateCoreType(SensorStateCoreTypes::imu);

    return 0;
}


Eigen::Vector3d ImuSensorStateCore::getBiasesAngularVelocity() const
{
    return this->biasesAngularVelocity;
}

int ImuSensorStateCore::setBiasesAngularVelocity(const Eigen::Vector3d &biasesAngularVelocity)
{
    this->biasesAngularVelocity=biasesAngularVelocity;
    return 0;
}

Eigen::Vector3d ImuSensorStateCore::getScaleAngularVelocity() const
{
    return this->scaleAngularVelocity;
}

int ImuSensorStateCore::setScaleAngularVelocity(const Eigen::Vector3d& scaleAngularVelocity)
{
    this->scaleAngularVelocity=scaleAngularVelocity;
    return 0;
}


Eigen::Vector3d ImuSensorStateCore::getBiasesLinearAcceleration() const
{
    return this->biasesLinearAcceleration;
}

int ImuSensorStateCore::setBiasesLinearAcceleration(const Eigen::Vector3d &biasesLinearAcceleration)
{
    this->biasesLinearAcceleration=biasesLinearAcceleration;
    return 0;
}

Eigen::Vector3d ImuSensorStateCore::getScaleLinearAcceleration() const
{
    return this->scaleLinearAcceleration;
}

int ImuSensorStateCore::setScaleLinearAcceleration(const Eigen::Vector3d& scaleLinearAcceleration)
{
    this->scaleLinearAcceleration=scaleLinearAcceleration;
    return 0;
}

int ImuSensorStateCore::updateStateFromIncrementErrorState(const Eigen::VectorXd &increment_error_state)
{
    unsigned int dimension=0;

    std::shared_ptr<ImuSensorCore> TheImuSensorCore=std::dynamic_pointer_cast<ImuSensorCore>(this->getMsfElementCoreSharedPtr());

    if(TheImuSensorCore->isEstimationPositionSensorWrtRobotEnabled())
    {
        this->positionSensorWrtRobot+=increment_error_state.block<3,1>(dimension, 0);
        dimension+=3;
    }
    if(TheImuSensorCore->isEstimationAttitudeSensorWrtRobotEnabled())
    {
        Eigen::Vector4d DeltaQuat, DeltaQuatAux;
        double normDeltaQuatAux;
        DeltaQuatAux[0]=1;
        DeltaQuatAux.block<3,1>(1,0)=0.5*increment_error_state.block<3,1>(dimension,0);
        normDeltaQuatAux=DeltaQuatAux.norm();
        DeltaQuat=DeltaQuatAux/normDeltaQuatAux;

        Eigen::Vector4d attitudeSensorWrtRobot=Quaternion::cross(this->attitudeSensorWrtRobot, DeltaQuat);

        this->attitudeSensorWrtRobot=attitudeSensorWrtRobot;

        dimension+=3;
    }
    if(TheImuSensorCore->isEstimationBiasLinearAccelerationEnabled())
    {
        this->biasesLinearAcceleration+=increment_error_state.block<3,1>(dimension, 0);

        //std::cout<<"biasesLinearAcceleration="<<this->biasesLinearAcceleration.transpose()<<std::endl;

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

        //std::cout<<"biasesAngularVelocity="<<this->biasesAngularVelocity.transpose()<<std::endl;

        dimension+=3;
    }
    if(TheImuSensorCore->isEstimationScaleAngularVelocityEnabled())
    {
        this->scaleAngularVelocity+=increment_error_state.block<3,1>(dimension, 0);
        dimension+=3;
    }


    return 0;
}
