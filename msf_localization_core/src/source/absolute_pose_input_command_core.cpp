
#include "msf_localization_core/absolute_pose_input_command_core.h"


AbsolutePoseInputCommandCore::AbsolutePoseInputCommandCore()
{

}

AbsolutePoseInputCommandCore::AbsolutePoseInputCommandCore(std::weak_ptr<InputCore> input_core_ptr)
{

}

AbsolutePoseInputCommandCore::~AbsolutePoseInputCommandCore()
{

}

int AbsolutePoseInputCommandCore::init()
{

}

int AbsolutePoseInputCommandCore::setPositionRobotWrtWorld(const Eigen::Vector3d& position_robot_wrt_world)
{

}

Eigen::Vector3d AbsolutePoseInputCommandCore::getPositionRobotWrtWorld() const
{

}

int AbsolutePoseInputCommandCore::setAttitudeRobotWrtWorld(const Eigen::Vector4d& attitude_robot_wrt_world)
{

}

Eigen::Vector4d AbsolutePoseInputCommandCore::getAttitudeRobotWrtWorld() const
{

}

Eigen::SparseMatrix<double> AbsolutePoseInputCommandCore::getCovarianceInputs(const TimeStamp& deltaTimeStamp)
{
    return this->getInputCoreSharedPtr()->getCovarianceInputs(deltaTimeStamp);
}
