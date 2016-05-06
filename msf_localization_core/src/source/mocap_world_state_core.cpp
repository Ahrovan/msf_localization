
#include "msf_localization_core/mocap_world_state_core.h"


MocapWorldStateCore::MocapWorldStateCore() :
    MapElementStateCore()
{
    init();

    return;
}

MocapWorldStateCore::MocapWorldStateCore(std::weak_ptr<MsfElementCore> msf_element_core_ptr) :
    MapElementStateCore(msf_element_core_ptr)
{
    init();

    return;
}

int MocapWorldStateCore::init()
{

    return 0;
}

MocapWorldStateCore::~MocapWorldStateCore()
{
    return;
}

Eigen::Vector3d MocapWorldStateCore::getPositionMocapWorldWrtWorld() const
{
    return this->position_mocap_world_wrt_world_;
}

int MocapWorldStateCore::setPositionMocapWorldWrtWorld(Eigen::Vector3d position_mocap_world_wrt_world)
{
    this->position_mocap_world_wrt_world_=position_mocap_world_wrt_world;
    return 0;
}

Eigen::Vector4d MocapWorldStateCore::getAttitudeMocapWorldWrtWorld() const
{
    return this->attitude_mocap_world_wrt_world_;
}

int MocapWorldStateCore::setAttitudeMocapWorldWrtWorld(Eigen::Vector4d attitude_mocap_world_wrt_world)
{
    this->attitude_mocap_world_wrt_world_=attitude_mocap_world_wrt_world;
    return 0;
}

Eigen::MatrixXd MocapWorldStateCore::getJacobianMappingRobotErrorState()
{
    return this->jacobian_mapping_error_state_.jacobian_mapping_robot_error_state_;
}

Eigen::MatrixXd MocapWorldStateCore::getJacobianMappingGlobalParametersErrorState()
{
    return this->jacobian_mapping_error_state_.jacobian_mapping_global_parameters_error_state_;
}

Eigen::MatrixXd MocapWorldStateCore::getJacobianMappingSensorErrorState()
{
    return this->jacobian_mapping_error_state_.jacobian_mapping_sensor_error_state_;
}

Eigen::MatrixXd MocapWorldStateCore::getJacobianMappingErrorStateNoise()
{
    return this->jacobian_mapping_error_state_noise_;
}

int MocapWorldStateCore::updateStateFromIncrementErrorState(Eigen::VectorXd increment_error_state)
{
    // Position
    position_mocap_world_wrt_world_+=increment_error_state.block<3,1>(0,0);

    // Attitude
    Eigen::Vector4d DeltaQuat, DeltaQuatAux;
    double NormDeltaQuatAux;
    DeltaQuatAux[0]=1;
    DeltaQuatAux.block<3,1>(1,0)=0.5*increment_error_state.block<3,1>(3,0);
    NormDeltaQuatAux=DeltaQuatAux.norm();
    DeltaQuat=DeltaQuatAux/NormDeltaQuatAux;

    Eigen::Vector4d attitude_aux=Quaternion::cross(attitude_mocap_world_wrt_world_, DeltaQuat);

    attitude_mocap_world_wrt_world_=attitude_aux;

    return 0;
}
