


#ifndef _CODED_VISUAL_MARKER_LANDMARK_STATE_CORE_H
#define _CODED_VISUAL_MARKER_LANDMARK_STATE_CORE_H



#include <Eigen/Dense>
#include <Eigen/Sparse>


#include "msf_localization_core/map_element_state_core.h"

#include "msf_localization_core/map_element_core.h"


#include "msf_localization_core/quaternion_algebra.h"


class CodedVisualMarkerLandmarkStateCore : public MapElementStateCore
{
public:
    CodedVisualMarkerLandmarkStateCore();
    CodedVisualMarkerLandmarkStateCore(std::weak_ptr<const MapElementCore> the_map_element_core_pt);
    ~CodedVisualMarkerLandmarkStateCore();

protected:
    int init();


    // State: xL=[pos (3/3), attit (4/3)]'

protected:
public:
    Eigen::Vector3d position_;
public:
    Eigen::Vector3d getPosition() const;
    int setPosition(Eigen::Vector3d position);



protected:
public:
    Eigen::Vector4d attitude_;
public:
    Eigen::Vector4d getAttitude() const;
    int setAttitude(Eigen::Vector4d attitude);



    // Error State Jacobians (6 x 6)
public:
    struct
    {
        // Fx_robot linear (3 x 3)
        Eigen::MatrixXd linear;

        // Fx_robot angular (3 x 3)
        Eigen::MatrixXd angular;
    } error_state_jacobian_;





public:
    int updateStateFromIncrementErrorState(Eigen::VectorXd increment_error_state);

};





#endif
