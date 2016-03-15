

#ifndef _VISUAL_MARKER_EYE_STATE_CORE_H
#define _VISUAL_MARKER_EYE_STATE_CORE_H


#include <Eigen/Dense>


#include "msf_localization_core/sensor_state_core.h"

#include "msf_localization_core/quaternion_algebra.h"



class VisualMarkerEyeStateCore : public SensorStateCore
{
public:
    VisualMarkerEyeStateCore();
    ~VisualMarkerEyeStateCore();


    ///// State if enabled (or Parameters if disabled)

    // if enabled
    // State: xs=[posi_sensor_wrt_robot, att_sensor_wrt_robot]'



    ////// Jacobians

    /// Jacobians Error State

public:
    struct
    {
        Eigen::Matrix3d positionSensorWrtRobot;
        Eigen::Matrix3d attitudeSensorWrtRobot;
    } errorStateJacobian;



public:
    int updateStateFromIncrementErrorState(Eigen::VectorXd increment_error_state);




};








#endif
