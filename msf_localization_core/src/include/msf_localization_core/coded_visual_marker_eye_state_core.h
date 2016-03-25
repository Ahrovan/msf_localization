

#ifndef _CODED_VISUAL_MARKER_EYE_STATE_CORE_H
#define _CODED_VISUAL_MARKER_EYE_STATE_CORE_H


#include <Eigen/Dense>


#include "msf_localization_core/sensor_state_core.h"

#include "msf_localization_core/quaternion_algebra.h"



class CodedVisualMarkerEyeStateCore : public SensorStateCore
{
public:
    CodedVisualMarkerEyeStateCore();
    CodedVisualMarkerEyeStateCore(std::weak_ptr<const SensorCore> the_sensor_core_ptr);
    ~CodedVisualMarkerEyeStateCore();

protected:
    int init();


    ///// State if enabled (or Parameters if disabled)

    // if enabled
    // State: xs=[posi_sensor_wrt_robot, att_sensor_wrt_robot]'



    ////// Jacobians

    /// Jacobians Error State

public:
    struct
    {
        Eigen::Matrix3d position_sensor_wrt_robot_;
        Eigen::Matrix3d attitude_sensor_wrt_robot_;
    } error_state_jacobian_;

    Eigen::SparseMatrix<double> jacobian_error_state_;



public:
    Eigen::SparseMatrix<double> jacobian_error_state_noise_;

public:
    Eigen::MatrixXd getJacobianErrorState();
    Eigen::SparseMatrix<double> getJacobianErrorStateNoise(); //TODO


public:
    int updateStateFromIncrementErrorState(Eigen::VectorXd increment_error_state);




};








#endif
