

#ifndef _CODED_VISUAL_MARKER_EYE_STATE_CORE_H
#define _CODED_VISUAL_MARKER_EYE_STATE_CORE_H


#include <Eigen/Dense>


#include "msf_localization_core/sensor_state_core.h"

#include "quaternion_algebra/quaternion_algebra.h"



class CodedVisualMarkerEyeStateCore : public SensorStateCore
{
public:
    CodedVisualMarkerEyeStateCore();
    CodedVisualMarkerEyeStateCore(const std::weak_ptr<MsfElementCore> msf_element_core_ptr);
    ~CodedVisualMarkerEyeStateCore();

protected:
    int init();


    ///// State if enabled (or Parameters if disabled)

    // if enabled
    // State: x_sen=[t_sens_wrt_robot, q_sens_wrt_robot]'





public:
    int updateStateFromIncrementErrorState(const Eigen::VectorXd& increment_error_state);




};








#endif
