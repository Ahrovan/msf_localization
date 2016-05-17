
#include "msf_localization_core/coded_visual_marker_landmark_core.h"


CodedVisualMarkerLandmarkStateCore::CodedVisualMarkerLandmarkStateCore() :
    MapElementStateCore()
{
    init();

    return;
}

CodedVisualMarkerLandmarkStateCore::CodedVisualMarkerLandmarkStateCore(std::weak_ptr<MsfElementCore> msf_element_core_ptr) :
    MapElementStateCore(msf_element_core_ptr)
{
    init();

    return;
}

int CodedVisualMarkerLandmarkStateCore::init()
{

    return 0;
}

CodedVisualMarkerLandmarkStateCore::~CodedVisualMarkerLandmarkStateCore()
{
    return;
}

Eigen::Vector3d CodedVisualMarkerLandmarkStateCore::getPosition() const
{
    return this->position_;
}

int CodedVisualMarkerLandmarkStateCore::setPosition(const Eigen::Vector3d &position)
{
    this->position_=position;
    return 0;
}

Eigen::Vector4d CodedVisualMarkerLandmarkStateCore::getAttitude() const
{
    return this->attitude_;
}

int CodedVisualMarkerLandmarkStateCore::setAttitude(const Eigen::Vector4d &attitude)
{
    this->attitude_=attitude;
    return 0;
}

int CodedVisualMarkerLandmarkStateCore::updateStateFromIncrementErrorState(const Eigen::VectorXd &increment_error_state)
{
    std::shared_ptr<CodedVisualMarkerLandmarkCore> map_element_core=std::dynamic_pointer_cast<CodedVisualMarkerLandmarkCore>(this->getMsfElementCoreSharedPtr());

    int dimension_error_state_i=0;

    if(map_element_core->isEstimationPositionVisualMarkerWrtWorldEnabled())
    {
        position_+=increment_error_state.block<3,1>(dimension_error_state_i,0);
        dimension_error_state_i+=3;
    }

    if(map_element_core->isEstimationAttitudeVisualMarkerWrtWorldEnabled())
    {
        Eigen::Vector4d DeltaQuat, DeltaQuatAux;
        double NormDeltaQuatAux;
        DeltaQuatAux[0]=1;
        DeltaQuatAux.block<3,1>(1,0)=0.5*increment_error_state.block<3,1>(dimension_error_state_i,0);
        NormDeltaQuatAux=DeltaQuatAux.norm();
        DeltaQuat=DeltaQuatAux/NormDeltaQuatAux;

        Eigen::Vector4d attitude_aux=Quaternion::cross(attitude_, DeltaQuat);

        attitude_=attitude_aux;

        dimension_error_state_i+=3;
    }


    return 0;
}
