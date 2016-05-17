
#include "msf_localization_core/mocap_world_state_core.h"


#include "msf_localization_core/mocap_world_core.h"


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

int MocapWorldStateCore::updateStateFromIncrementErrorState(const Eigen::VectorXd &increment_error_state)
{
    std::shared_ptr<MocapWorldCore> map_element_core=std::dynamic_pointer_cast<MocapWorldCore>(this->getMsfElementCoreSharedPtr());

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
