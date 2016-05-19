
#include "msf_localization_core/robot_state_core.h"


RobotStateCore::RobotStateCore() :
    StateCore()
{
    init();

    return;
}

RobotStateCore::RobotStateCore(std::weak_ptr<MsfElementCore> msf_element_core_ptr) :
    StateCore(msf_element_core_ptr)
{
    init();

    return;
}

RobotStateCore::~RobotStateCore()
{
    return;
}

int RobotStateCore::init()
{
    this->setStateCoreType(StateCoreTypes::robot);
    this->robot_state_core_type_=RobotStateCoreTypes::undefined;

    return 0;
}

int RobotStateCore::setRobotStateType(RobotStateCoreTypes robot_state_core_type)
{
    this->robot_state_core_type_=robot_state_core_type;
    return 0;
}

RobotStateCoreTypes RobotStateCore::getRobotStateType()
{
    return robot_state_core_type_;
}

Eigen::Vector3d RobotStateCore::getPositionRobotWrtWorld() const
{
    std::cout<<"Error in RobotStateCore::getXXRobotWrtWorld()"<<std::endl;
    return Eigen::Vector3d();
}

int RobotStateCore::setPositionRobotWrtWorld(const Eigen::Vector3d& position)
{
    std::cout<<"Error in RobotStateCore::setXXRobotWrtWorld()"<<std::endl;
    return -1;
}

Eigen::Vector3d RobotStateCore::getLinearSpeedRobotWrtWorld() const
{
    std::cout<<"Error in RobotStateCore::getXXRobotWrtWorld()"<<std::endl;
    return Eigen::Vector3d();
}

int RobotStateCore::setLinearSpeedRobotWrtWorld(const Eigen::Vector3d& linear_speed)
{
    std::cout<<"Error in RobotStateCore::setXXRobotWrtWorld()"<<std::endl;
    return -1;
}

Eigen::Vector3d RobotStateCore::getLinearAccelerationRobotWrtWorld() const
{
    std::cout<<"Error in RobotStateCore::getXXRobotWrtWorld()"<<std::endl;
    return Eigen::Vector3d();
}

int RobotStateCore::setLinearAccelerationRobotWrtWorld(const Eigen::Vector3d& linear_acceleration)
{
    std::cout<<"Error in RobotStateCore::setXXRobotWrtWorld()"<<std::endl;
    return -1;
}

Eigen::Vector4d RobotStateCore::getAttitudeRobotWrtWorld() const
{
    std::cout<<"Error in RobotStateCore::getXXRobotWrtWorld()"<<std::endl;
    return Eigen::Vector4d();
}

int RobotStateCore::setAttitudeRobotWrtWorld(const Eigen::Vector4d& attitude)
{
    std::cout<<"Error in RobotStateCore::setXXRobotWrtWorld()"<<std::endl;
    return -1;
}

Eigen::Vector3d RobotStateCore::getAngularVelocityRobotWrtWorld() const
{
    std::cout<<"Error in RobotStateCore::getXXRobotWrtWorld()"<<std::endl;
    return Eigen::Vector3d();
}

int RobotStateCore::setAngularVelocityRobotWrtWorld(const Eigen::Vector3d& angular_velocity)
{
    std::cout<<"Error in RobotStateCore::setXXRobotWrtWorld()"<<std::endl;
    return -1;
}

Eigen::Vector3d RobotStateCore::getAngularAccelerationRobotWrtWorld() const
{
    std::cout<<"Error in RobotStateCore::getXXRobotWrtWorld()"<<std::endl;
    return Eigen::Vector3d();
}

int RobotStateCore::setAngularAccelerationRobotWrtWorld(const Eigen::Vector3d& angular_acceleration)
{
    std::cout<<"Error in RobotStateCore::setXXRobotWrtWorld()"<<std::endl;
    return -1;
}
