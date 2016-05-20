
#ifndef _GLOBAL_PARAMETERS_STATE_CORE_H
#define _GLOBAL_PARAMETERS_STATE_CORE_H


#include "msf_localization_core/state_core.h"
#include "msf_localization_core/global_parameters_core.h"


#include <Eigen/Dense>
#include <Eigen/Sparse>


class GlobalParametersStateCore : public StateCore
{

public:
    GlobalParametersStateCore();
    GlobalParametersStateCore(const std::weak_ptr<MsfElementCore> msf_element_core_ptr);
    ~GlobalParametersStateCore();

protected:
    int init();



    // Gravity
protected:
public:
    Eigen::Vector3d gravity;
public:
    Eigen::Vector3d getGravity() const;
    int setGravity(const Eigen::Vector3d& gravity);



public:
    int updateStateFromIncrementErrorState(const Eigen::VectorXd& increment_error_state);


};



#endif
