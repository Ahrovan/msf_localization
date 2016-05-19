
#include "msf_localization_core/imu_driven_robot_state_core.h"


#include "msf_localization_core/imu_driven_robot_core.h"


ImuDrivenRobotStateCore::ImuDrivenRobotStateCore() :
    RobotStateCore()
{
    init();

    return;
}

ImuDrivenRobotStateCore::ImuDrivenRobotStateCore(std::weak_ptr<MsfElementCore> msf_element_core_ptr) :
    RobotStateCore(msf_element_core_ptr)
{
    init();

    return;
}

ImuDrivenRobotStateCore::~ImuDrivenRobotStateCore()
{
    return;
}

int ImuDrivenRobotStateCore::init()
{
    this->setRobotStateType(RobotStateCoreTypes::imu_driven);

    return 0;
}


Eigen::Vector3d ImuDrivenRobotStateCore::getPositionRobotWrtWorld() const
{
    return this->position_robot_wrt_world_;
}

int ImuDrivenRobotStateCore::setPositionRobotWrtWorld(const Eigen::Vector3d &position_robot_wrt_world)
{
    this->position_robot_wrt_world_=position_robot_wrt_world;
    return 0;
}

Eigen::Vector3d ImuDrivenRobotStateCore::getLinearSpeedRobotWrtWorld() const
{
    return this->linear_speed_robot_wrt_world_;
}

int ImuDrivenRobotStateCore::setLinearSpeedRobotWrtWorld(const Eigen::Vector3d& linear_speed_robot_wrt_world)
{
    this->linear_speed_robot_wrt_world_=linear_speed_robot_wrt_world;
    return 0;
}

Eigen::Vector3d ImuDrivenRobotStateCore::getLinearAccelerationRobotWrtWorld() const
{
    return this->linear_acceleration_robot_wrt_world_;
}

int ImuDrivenRobotStateCore::setLinearAccelerationRobotWrtWorld(const Eigen::Vector3d& linear_acceleration_robot_wrt_world)
{
    this->linear_acceleration_robot_wrt_world_=linear_acceleration_robot_wrt_world;
    return 0;
}

Eigen::Vector4d ImuDrivenRobotStateCore::getAttitudeRobotWrtWorld() const
{
    return this->attitude_robot_wrt_world_;
}

int ImuDrivenRobotStateCore::setAttitudeRobotWrtWorld(const Eigen::Vector4d& attitude_robot_wrt_world)
{
    this->attitude_robot_wrt_world_=attitude_robot_wrt_world;
    return 0;
}

Eigen::Vector3d ImuDrivenRobotStateCore::getAngularVelocityRobotWrtWorld() const
{
    return this->angular_velocity_robot_wrt_world_;
}

int ImuDrivenRobotStateCore::setAngularVelocityRobotWrtWorld(const Eigen::Vector3d& angular_velocity_robot_wrt_world)
{
    this->angular_velocity_robot_wrt_world_=angular_velocity_robot_wrt_world;
    return 0;
}

int ImuDrivenRobotStateCore::updateStateFromIncrementErrorState(const Eigen::VectorXd &increment_error_state)
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

    // End
    return 0;
}
