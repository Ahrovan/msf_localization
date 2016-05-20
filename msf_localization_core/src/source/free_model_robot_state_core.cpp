
#include "msf_localization_core/free_model_robot_state_core.h"

#include "msf_localization_core/free_model_robot_core.h"


FreeModelRobotStateCore::FreeModelRobotStateCore() :
    RobotStateCore()
{
    init();

    return;
}

FreeModelRobotStateCore::FreeModelRobotStateCore(const std::weak_ptr<MsfElementCore> msf_element_core_ptr) :
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


Eigen::Vector3d FreeModelRobotStateCore::getPositionRobotWrtWorld() const
{
    return this->position_robot_wrt_world_;
}

int FreeModelRobotStateCore::setPositionRobotWrtWorld(const Eigen::Vector3d &position)
{
    this->position_robot_wrt_world_=position;
    return 0;
}

Eigen::Vector3d FreeModelRobotStateCore::getLinearSpeedRobotWrtWorld() const
{
    return this->linear_speed_robot_wrt_world_;
}

int FreeModelRobotStateCore::setLinearSpeedRobotWrtWorld(const Eigen::Vector3d& linear_speed)
{
    this->linear_speed_robot_wrt_world_=linear_speed;
    return 0;
}

Eigen::Vector3d FreeModelRobotStateCore::getLinearAccelerationRobotWrtWorld() const
{
    return this->linear_acceleration_robot_wrt_world_;
}

int FreeModelRobotStateCore::setLinearAccelerationRobotWrtWorld(const Eigen::Vector3d &linear_acceleration)
{
    this->linear_acceleration_robot_wrt_world_=linear_acceleration;
    return 0;
}

Eigen::Vector4d FreeModelRobotStateCore::getAttitudeRobotWrtWorld() const
{
    return this->attitude_robot_wrt_world_;
}

int FreeModelRobotStateCore::setAttitudeRobotWrtWorld(const Eigen::Vector4d& attitude)
{
    this->attitude_robot_wrt_world_=attitude;
    return 0;
}

Eigen::Vector3d FreeModelRobotStateCore::getAngularVelocityRobotWrtWorld() const
{
    return this->angular_velocity_robot_wrt_world_;
}

int FreeModelRobotStateCore::setAngularVelocityRobotWrtWorld(const Eigen::Vector3d &angular_velocity)
{
    this->angular_velocity_robot_wrt_world_=angular_velocity;
    return 0;
}

Eigen::Vector3d FreeModelRobotStateCore::getAngularAccelerationRobotWrtWorld() const
{
    return this->angular_acceleration_robot_wrt_world_;
}

int FreeModelRobotStateCore::setAngularAccelerationRobotWrtWorld(const Eigen::Vector3d& angular_acceleration)
{
    this->angular_acceleration_robot_wrt_world_=angular_acceleration;
    return 0;
}



int FreeModelRobotStateCore::updateStateFromIncrementErrorState(const Eigen::VectorXd &increment_error_state)
{

    position_robot_wrt_world_+=increment_error_state.block<3,1>(0,0);
    linear_speed_robot_wrt_world_+=increment_error_state.block<3,1>(3,0);
    linear_acceleration_robot_wrt_world_+=increment_error_state.block<3,1>(6,0);


    Eigen::Vector4d DeltaQuat, DeltaQuatAux;
    double normDeltaQuatAux;
    DeltaQuatAux[0]=1;
    DeltaQuatAux.block<3,1>(1,0)=0.5*increment_error_state.block<3,1>(9,0);
    normDeltaQuatAux=DeltaQuatAux.norm();
    DeltaQuat=DeltaQuatAux/normDeltaQuatAux;

    Eigen::Vector4d attitudeAux=Quaternion::cross(attitude_robot_wrt_world_, DeltaQuat);


    attitude_robot_wrt_world_=attitudeAux;


    angular_velocity_robot_wrt_world_+=increment_error_state.block<3,1>(12,0);
    angular_acceleration_robot_wrt_world_+=increment_error_state.block<3,1>(15,0);


    return 0;
}
