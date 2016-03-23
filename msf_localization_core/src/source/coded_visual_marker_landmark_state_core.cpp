
#include "msf_localization_core/coded_visual_marker_landmark_core.h"


CodedVisualMarkerLandmarkStateCore::CodedVisualMarkerLandmarkStateCore() :
    MapElementStateCore()
{
    init();

    return;
}

CodedVisualMarkerLandmarkStateCore::CodedVisualMarkerLandmarkStateCore(std::weak_ptr<const MapElementCore> the_map_element_core_pt) :
    MapElementStateCore(the_map_element_core_pt)
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

int CodedVisualMarkerLandmarkStateCore::setPosition(Eigen::Vector3d position)
{
    this->position_=position;
    return 0;
}

Eigen::Vector4d CodedVisualMarkerLandmarkStateCore::getAttitude() const
{
    return this->attitude_;
}

int CodedVisualMarkerLandmarkStateCore::setAttitude(Eigen::Vector4d attitude)
{
    this->attitude_=attitude;
    return 0;
}

int CodedVisualMarkerLandmarkStateCore::updateStateFromIncrementErrorState(Eigen::VectorXd increment_error_state)
{

    position_+=increment_error_state.block<3,1>(0,0);


    Eigen::Vector4d DeltaQuat;
    DeltaQuat[0]=1;
    DeltaQuat.block<3,1>(1,0)=0.5*increment_error_state.block<3,1>(3,0);
    DeltaQuat=DeltaQuat/DeltaQuat.norm();

    attitude_=Quaternion::cross(attitude_, DeltaQuat);


    return 0;
}
