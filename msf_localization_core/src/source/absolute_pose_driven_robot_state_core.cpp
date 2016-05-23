
#include "msf_localization_core/absolute_pose_driven_robot_state_core.h"

#include "msf_localization_core/absolute_pose_driven_robot_core.h"


AbsolutePoseDrivenRobotStateCore::AbsolutePoseDrivenRobotStateCore() :
    RobotStateCore()
{
    init();

    return;
}

AbsolutePoseDrivenRobotStateCore::AbsolutePoseDrivenRobotStateCore(const std::weak_ptr<MsfElementCore> msf_element_core_ptr) :
    RobotStateCore(msf_element_core_ptr)
{
    init();

    return;
}

AbsolutePoseDrivenRobotStateCore::~AbsolutePoseDrivenRobotStateCore()
{
    return;
}

int AbsolutePoseDrivenRobotStateCore::init()
{
    this->setRobotStateType(RobotStateCoreTypes::absolute_pose_driven);

    return 0;
}


Eigen::Vector3d AbsolutePoseDrivenRobotStateCore::getPositionRobotWrtWorld() const
{
    return this->position_robot_wrt_world_;
}

int AbsolutePoseDrivenRobotStateCore::setPositionRobotWrtWorld(const Eigen::Vector3d &position)
{
    this->position_robot_wrt_world_=position;
    return 0;
}

Eigen::Vector4d AbsolutePoseDrivenRobotStateCore::getAttitudeRobotWrtWorld() const
{
    return this->attitude_robot_wrt_world_;
}

int AbsolutePoseDrivenRobotStateCore::setAttitudeRobotWrtWorld(const Eigen::Vector4d& attitude)
{
    this->attitude_robot_wrt_world_=attitude;
    return 0;
}

int AbsolutePoseDrivenRobotStateCore::updateStateFromIncrementErrorState(const Eigen::VectorXd &increment_error_state)
{

    position_robot_wrt_world_+=increment_error_state.block<3,1>(0,0);


    Eigen::Vector4d DeltaQuat, DeltaQuatAux;
    double normDeltaQuatAux;
    DeltaQuatAux[0]=1;
    DeltaQuatAux.block<3,1>(1,0)=0.5*increment_error_state.block<3,1>(9,0);
    normDeltaQuatAux=DeltaQuatAux.norm();
    DeltaQuat=DeltaQuatAux/normDeltaQuatAux;

    Eigen::Vector4d attitudeAux=Quaternion::cross(attitude_robot_wrt_world_, DeltaQuat);


    attitude_robot_wrt_world_=attitudeAux;



    return 0;
}
