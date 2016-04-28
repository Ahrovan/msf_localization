

#ifndef _CODED_VISUAL_MARKER_EYE_STATE_CORE_H
#define _CODED_VISUAL_MARKER_EYE_STATE_CORE_H


#include <Eigen/Dense>


#include "msf_localization_core/sensor_state_core.h"

#include "msf_localization_core/quaternion_algebra.h"



class CodedVisualMarkerEyeStateCore : public SensorStateCore
{
public:
    CodedVisualMarkerEyeStateCore();
    CodedVisualMarkerEyeStateCore(std::weak_ptr<MsfElementCore> msf_element_core_ptr);
    ~CodedVisualMarkerEyeStateCore();

protected:
    int init();


    ///// State if enabled (or Parameters if disabled)

    // if enabled
    // State: x_sen=[t_sens_wrt_robot, q_sens_wrt_robot]'



    ////// Jacobians

    /// Jacobians Error State

public:
    struct
    {
        Eigen::Matrix3d position_sensor_wrt_robot_;
        Eigen::Matrix3d attitude_sensor_wrt_robot_;
    } error_state_jacobian_;


//public:
//    Eigen::MatrixXd getJacobianErrorState();



public:
    int updateStateFromIncrementErrorState(Eigen::VectorXd increment_error_state);




};








#endif
