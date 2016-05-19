
#ifndef _ABSOLUTE_POSE_INPUT_STATE_CORE_H
#define _ABSOLUTE_POSE_INPUT_STATE_CORE_H


#include <Eigen/Dense>
#include <Eigen/Sparse>


#include "msf_localization_core/input_state_core.h"


#include "msf_localization_core/quaternion_algebra.h"


class AbsolutePoseInputStateCore : public InputStateCore
{
public:
    AbsolutePoseInputStateCore();
    AbsolutePoseInputStateCore(std::weak_ptr<MsfElementCore> msf_element_core_ptr);
    ~AbsolutePoseInputStateCore();

protected:
    int init();



    ///// State and parameters
    // none



public:
    int updateStateFromIncrementErrorState(const Eigen::VectorXd& increment_error_state);

};


#endif
