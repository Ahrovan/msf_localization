
#include "msf_localization_core/absolute_pose_input_state_core.h"

#include "msf_localization_core/absolute_pose_input_core.h"


AbsolutePoseInputStateCore::AbsolutePoseInputStateCore() :
    InputStateCore()
{
    init();
    return;
}

AbsolutePoseInputStateCore::AbsolutePoseInputStateCore(const std::weak_ptr<MsfElementCore> msf_element_core_ptr) :
    InputStateCore(msf_element_core_ptr)
{
    init();
    return;
}

AbsolutePoseInputStateCore::~AbsolutePoseInputStateCore()
{
    return;
}

int AbsolutePoseInputStateCore::init()
{
    // Input Type
    this->setInputStateType(InputStateCoreTypes::absolute_pose);

    // Values
    position_input_wrt_robot_.setZero();
    attitude_input_wrt_robot_.setZero();

    // End
    return 0;
}

Eigen::Vector3d AbsolutePoseInputStateCore::getPositionInputWrtRobot() const
{
    return this->position_input_wrt_robot_;
}

int AbsolutePoseInputStateCore::setPositionInputWrtRobot(const Eigen::Vector3d &position_input_wrt_robot)
{
    this->position_input_wrt_robot_=position_input_wrt_robot;
    return 0;
}

Eigen::Vector4d AbsolutePoseInputStateCore::getAttitudeInputWrtRobot() const
{
    return this->attitude_input_wrt_robot_;
}

int AbsolutePoseInputStateCore::setAttitudeInputWrtRobot(const Eigen::Vector4d &attitude_input_wrt_robot)
{
    this->attitude_input_wrt_robot_=attitude_input_wrt_robot;
    return 0;
}

int AbsolutePoseInputStateCore::updateStateFromIncrementErrorState(const Eigen::VectorXd& increment_error_state)
{
    unsigned int dimension=0;

    std::shared_ptr<AbsolutePoseInputCore> input_core=std::dynamic_pointer_cast<AbsolutePoseInputCore>(this->getMsfElementCoreSharedPtr());

    if(input_core->isEstimationPositionInputWrtRobotEnabled())
    {
        this->position_input_wrt_robot_+=increment_error_state.block<3,1>(dimension, 0);
        dimension+=3;
    }
    if(input_core->isEstimationAttitudeInputWrtRobotEnabled())
    {
        Eigen::Vector4d DeltaQuat;
        DeltaQuat[0]=1;
        DeltaQuat.block<3,1>(1,0)=0.5*increment_error_state.block<3,1>(dimension,0);
        DeltaQuat=DeltaQuat/DeltaQuat.norm();

        this->attitude_input_wrt_robot_=Quaternion::cross(this->attitude_input_wrt_robot_, DeltaQuat);
        dimension+=3;
    }


    return 0;
}
