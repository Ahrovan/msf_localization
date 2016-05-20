
#include "msf_localization_core/world_reference_frame_state_core.h"


#include "msf_localization_core/world_reference_frame_core.h"


WorldReferenceFrameStateCore::WorldReferenceFrameStateCore() :
    MapElementStateCore()
{
    init();

    return;
}

WorldReferenceFrameStateCore::WorldReferenceFrameStateCore(const std::weak_ptr<MsfElementCore> msf_element_core_ptr) :
    MapElementStateCore(msf_element_core_ptr)
{
    init();

    return;
}

int WorldReferenceFrameStateCore::init()
{

    return 0;
}

WorldReferenceFrameStateCore::~WorldReferenceFrameStateCore()
{
    return;
}

Eigen::Vector3d WorldReferenceFrameStateCore::getPositionMocapWorldWrtWorld() const
{
    return this->position_mocap_world_wrt_world_;
}

int WorldReferenceFrameStateCore::setPositionMocapWorldWrtWorld(const Eigen::Vector3d &position_mocap_world_wrt_world)
{
    this->position_mocap_world_wrt_world_=position_mocap_world_wrt_world;
    return 0;
}

Eigen::Vector4d WorldReferenceFrameStateCore::getAttitudeMocapWorldWrtWorld() const
{
    return this->attitude_mocap_world_wrt_world_;
}

int WorldReferenceFrameStateCore::setAttitudeMocapWorldWrtWorld(const Eigen::Vector4d& attitude_mocap_world_wrt_world)
{
    this->attitude_mocap_world_wrt_world_=attitude_mocap_world_wrt_world;
    return 0;
}

int WorldReferenceFrameStateCore::updateStateFromIncrementErrorState(const Eigen::VectorXd &increment_error_state)
{
    std::shared_ptr<WorldReferenceFrameCore> map_element_core=std::dynamic_pointer_cast<WorldReferenceFrameCore>(this->getMsfElementCoreSharedPtr());

    int dimension_error_state_i=0;

    // Position
    if(map_element_core->isEstimationPositionMocapWorldWrtWorldEnabled())
    {
        position_mocap_world_wrt_world_+=increment_error_state.block<3,1>(dimension_error_state_i,0);
        dimension_error_state_i+=3;
    }

    // Attitude
    if(map_element_core->isEstimationAttitudeMocapWorldWrtWorldEnabled())
    {
        Eigen::Vector4d DeltaQuat, DeltaQuatAux;
        double NormDeltaQuatAux;
        DeltaQuatAux[0]=1;
        DeltaQuatAux.block<3,1>(1,0)=0.5*increment_error_state.block<3,1>(dimension_error_state_i,0);
        NormDeltaQuatAux=DeltaQuatAux.norm();
        DeltaQuat=DeltaQuatAux/NormDeltaQuatAux;

        Eigen::Vector4d attitude_aux=Quaternion::cross(attitude_mocap_world_wrt_world_, DeltaQuat);

        attitude_mocap_world_wrt_world_=attitude_aux;

        dimension_error_state_i+=3;
    }


    return 0;
}
