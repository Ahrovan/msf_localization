
#ifndef _CODED_VISUAL_MARKER_LANDMARK_CORE_H
#define _CODED_VISUAL_MARKER_LANDMARK_CORE_H


// Math
#include "cmath"

// Time Stamp
#include "msf_localization_core/time_stamp.h"

// Quaternion algebra
#include "msf_localization_core/quaternion_algebra.h"

// Map element Core
#include "msf_localization_core/map_element_core.h"

// State
#include "msf_localization_core/coded_visual_marker_landmark_state_core.h"


class CodedVisualMarkerLandmarkCore : public MapElementCore
{

public:
    CodedVisualMarkerLandmarkCore();
    ~CodedVisualMarkerLandmarkCore();



    ///// Prediction Noises





    ////// Init error state variances -> Temporal, only for the initial configuration

public:
    int setInitErrorStateVariancePosition(Eigen::Vector3d initVariance);
    int setInitErrorStateVarianceAttitude(Eigen::Vector3d initVariance);





    ///// Predict functions

    // State: xL=[pos, attit]'

    // Prediction state function
public:
    int predictState(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, const std::shared_ptr<CodedVisualMarkerLandmarkStateCore> pastState, std::shared_ptr<CodedVisualMarkerLandmarkStateCore>& predictedState);

    // Jacobian
public:
    int predictStateErrorStateJacobians(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, std::shared_ptr<CodedVisualMarkerLandmarkStateCore> pastState, std::shared_ptr<CodedVisualMarkerLandmarkStateCore>& predictedState);




};





#endif
