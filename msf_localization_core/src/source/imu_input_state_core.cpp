
#include "msf_localization_core/imu_input_state_core.h"


#include "msf_localization_core/imu_input_core.h"


ImuInputStateCore::ImuInputStateCore() :
    InputStateCore()
{
    init();

    return;
}

ImuInputStateCore::ImuInputStateCore(std::weak_ptr<MsfElementCore> msf_element_core_ptr) :
    InputStateCore(msf_element_core_ptr)
{
    init();

    return;
}

ImuInputStateCore::~ImuInputStateCore()
{
    return;
}

int ImuInputStateCore::init()
{
    // Input Type
    this->setInputStateType(InputStateCoreTypes::imu);

    // Values
    position_input_wrt_robot_.setZero();
    attitude_input_wrt_robot_.setZero();
    biases_linear_acceleration_.setZero();
    biases_angular_velocity_.setZero();
    sensitivity_linear_acceleration_.setZero();
    sensitivity_angular_velocity_.setZero();

    return 0;
}

Eigen::Vector3d ImuInputStateCore::getPositionInputWrtRobot() const
{
    return this->position_input_wrt_robot_;
}

int ImuInputStateCore::setPositionInputWrtRobot(Eigen::Vector3d position_input_wrt_robot)
{
    this->position_input_wrt_robot_=position_input_wrt_robot;
    return 0;
}

Eigen::Vector4d ImuInputStateCore::getAttitudeInputWrtRobot() const
{
    return this->attitude_input_wrt_robot_;
}

int ImuInputStateCore::setAttitudeInputWrtRobot(Eigen::Vector4d attitude_input_wrt_robot)
{
    this->attitude_input_wrt_robot_=attitude_input_wrt_robot;
    return 0;
}

Eigen::Vector3d ImuInputStateCore::getBiasesLinearAcceleration() const
{
    return this->biases_linear_acceleration_;
}

int ImuInputStateCore::setBiasesLinearAcceleration(Eigen::Vector3d biases_linear_acceleration)
{
    this->biases_linear_acceleration_=biases_linear_acceleration;
    return 0;
}

Eigen::Vector3d ImuInputStateCore::getBiasesAngularVelocity() const
{
    return this->biases_angular_velocity_;
}

int ImuInputStateCore::setBiasesAngularVelocity(Eigen::Vector3d biases_angular_velocity)
{
    this->biases_angular_velocity_=biases_angular_velocity;
    return 0;
}

Eigen::Matrix3d ImuInputStateCore::getSensitivityLinearAcceleration() const
{
    return this->sensitivity_linear_acceleration_;
}

int ImuInputStateCore::setSensitivityLinearAcceleration(Eigen::Matrix3d sensitivity_linear_acceleration)
{
    this->sensitivity_linear_acceleration_=sensitivity_linear_acceleration;
    return 0;
}

Eigen::Matrix3d ImuInputStateCore::getSensitivityAngularVelocity() const
{
    return this->sensitivity_angular_velocity_;
}

int ImuInputStateCore::setSensitivityAngularVelocity(Eigen::Matrix3d sensitivity_angular_velocity)
{
    this->sensitivity_angular_velocity_=sensitivity_angular_velocity;
    return 0;
}


int ImuInputStateCore::updateStateFromIncrementErrorState(const Eigen::VectorXd &increment_error_state)
{

    unsigned int dimension=0;

    std::shared_ptr<ImuInputCore> imu_input_core=std::dynamic_pointer_cast<ImuInputCore>(this->getMsfElementCoreSharedPtr());

    if(imu_input_core->isEstimationPositionInputWrtRobotEnabled())
    {
        this->position_input_wrt_robot_+=increment_error_state.block<3,1>(dimension, 0);
        dimension+=3;
    }
    if(imu_input_core->isEstimationAttitudeInputWrtRobotEnabled())
    {
        Eigen::Vector4d DeltaQuat, DeltaQuatAux;
        double normDeltaQuatAux;
        DeltaQuatAux[0]=1;
        DeltaQuatAux.block<3,1>(1,0)=0.5*increment_error_state.block<3,1>(dimension,0);
        normDeltaQuatAux=DeltaQuatAux.norm();
        DeltaQuat=DeltaQuatAux/normDeltaQuatAux;

        Eigen::Vector4d attitude_input_wrt_robot=Quaternion::cross(this->attitude_input_wrt_robot_, DeltaQuat);

        this->attitude_input_wrt_robot_=attitude_input_wrt_robot;

        dimension+=3;
    }
    if(imu_input_core->isEstimationBiasLinearAccelerationEnabled())
    {
        this->biases_linear_acceleration_+=increment_error_state.block<3,1>(dimension, 0);
        dimension+=3;
    }
    if(imu_input_core->isEstimationBiasAngularVelocityEnabled())
    {
        this->biases_angular_velocity_+=increment_error_state.block<3,1>(dimension, 0);
        dimension+=3;
    }
    if(imu_input_core->isEstimationSensitivityLinearAccelerationEnabled())
    {
        // TODO
    }
    if(imu_input_core->isEstimationSensitivityAngularVelocityEnabled())
    {
        // TODO
    }


    return 0;
}
