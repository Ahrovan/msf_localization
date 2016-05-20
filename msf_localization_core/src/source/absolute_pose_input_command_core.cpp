
#include "msf_localization_core/absolute_pose_input_command_core.h"

#include "msf_localization_core/absolute_pose_input_core.h"


AbsolutePoseInputCommandCore::AbsolutePoseInputCommandCore() :
    InputCommandCore()
{
    init();

    return;
}
AbsolutePoseInputCommandCore::AbsolutePoseInputCommandCore(const std::weak_ptr<InputCore> input_core_ptr) :
    InputCommandCore(input_core_ptr)
{
    init();

    return;
}

AbsolutePoseInputCommandCore::~AbsolutePoseInputCommandCore()
{

    return;
}

int AbsolutePoseInputCommandCore::init()
{
    // Variables
    position_input_wrt_input_world_.setZero();
    attitude_input_wrt_input_world_.setZero();

    // Noise
    noise_input_command_pose_input_wrt_input_world_.setZero();

    // Type
    setInputCommandType(InputCommandTypes::absolute_pose);

    return 0;
}

void AbsolutePoseInputCommandCore::setPositionInputWrtInputWorld(const Eigen::Vector3d& position_input_wrt_input_world)
{
    this->position_input_wrt_input_world_=position_input_wrt_input_world;
    return;
}

Eigen::Vector3d AbsolutePoseInputCommandCore::getPositionInputWrtInputWorld() const
{
    return this->position_input_wrt_input_world_;
}

void AbsolutePoseInputCommandCore::setAttitudeInputWrtInputWorld(const Eigen::Vector4d& attitude_input_wrt_input_world)
{
    this->attitude_input_wrt_input_world_=attitude_input_wrt_input_world;
    return;
}

Eigen::Vector4d AbsolutePoseInputCommandCore::getAttitudeInputWrtInputWorld() const
{
    return this->attitude_input_wrt_input_world_;
}

void AbsolutePoseInputCommandCore::setNoiseInputCommandPoseInputWrtInputWorld(const Eigen::MatrixXd& noise_input_command_pose_input_wrt_input_world)
{
    this->noise_input_command_pose_input_wrt_input_world_=noise_input_command_pose_input_wrt_input_world;
    return;
}

Eigen::MatrixXd  AbsolutePoseInputCommandCore::getNoiseInputCommandPoseInputWrtInputWorld() const
{
    return this->noise_input_command_pose_input_wrt_input_world_;
}

Eigen::SparseMatrix<double> AbsolutePoseInputCommandCore::getCovarianceInputs(const TimeStamp& deltaTimeStamp)
{

    if(std::dynamic_pointer_cast<AbsolutePoseInputCore>(this->getInputCoreSharedPtr())->hasInputCommandPoseInputWrtInputWorldCovariance())
    {
        return this->noise_input_command_pose_input_wrt_input_world_.sparseView();
    }
    else
    {
        return this->getInputCoreSharedPtr()->getCovarianceInputs(deltaTimeStamp);
    }
}
