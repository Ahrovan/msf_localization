


#ifndef _CODED_VISUAL_MARKER_LANDMARK_STATE_CORE_H
#define _CODED_VISUAL_MARKER_LANDMARK_STATE_CORE_H



#include <Eigen/Dense>
#include <Eigen/Sparse>


#include "msf_localization_core/map_element_state_core.h"

#include "msf_localization_core/map_element_core.h"


#include "quaternion_algebra/quaternion_algebra.h"


class CodedVisualMarkerLandmarkStateCore : public MapElementStateCore
{
public:
    CodedVisualMarkerLandmarkStateCore();
    CodedVisualMarkerLandmarkStateCore(const std::weak_ptr<MsfElementCore> msf_element_core_ptr);
    ~CodedVisualMarkerLandmarkStateCore();

protected:
    int init();


    // State: xL=[pos (3/3), attit (4/3)]'

protected:
public:
    Eigen::Vector3d position_;
public:
    Eigen::Vector3d getPosition() const;
    int setPosition(const Eigen::Vector3d& position);



protected:
public:
    Eigen::Vector4d attitude_;
public:
    Eigen::Vector4d getAttitude() const;
    int setAttitude(const Eigen::Vector4d& attitude);





public:
    int updateStateFromIncrementErrorState(const Eigen::VectorXd& increment_error_state);

};





#endif
