
#ifndef _IMU_INPUT_STATE_CORE_H
#define _IMU_INPUT_STATE_CORE_H


#include <Eigen/Dense>
#include <Eigen/Sparse>


#include "msf_localization_core/input_state_core.h"


#include "msf_localization_core/quaternion_algebra.h"


class ImuInputStateCore : public InputStateCore
{
public:
    ImuInputStateCore();
    ImuInputStateCore(std::weak_ptr<MsfElementCore> msf_element_core_ptr);
    ~ImuInputStateCore();

protected:
    int init();


    // State:
    // TODO



    // Error State Jacobians
public:
    Eigen::SparseMatrix<double> errorStateJacobian;


    // Jacobian Error State Noise
public:
    Eigen::SparseMatrix<double> errorStateNoiseJacobian;



public:
    Eigen::SparseMatrix<double> getJacobianErrorState();
    Eigen::SparseMatrix<double> getJacobianErrorStateNoise();


public:
    int updateStateFromIncrementErrorState(Eigen::VectorXd increment_error_state);

};







#endif
