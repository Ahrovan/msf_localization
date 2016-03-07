
#ifndef _GLOBAL_PARAMETERS_STATE_CORE_H
#define _GLOBAL_PARAMETERS_STATE_CORE_H


#include "msf_localization_core/global_parameters_core.h"


class GlobalParametersStateCore
{

public:
    GlobalParametersStateCore();
    GlobalParametersStateCore(std::weak_ptr<const GlobalParametersCore> TheGlobalParametersCorePtr);
    ~GlobalParametersStateCore();



    // Ptr to the core
protected:
    // It is not the owner of this Pointer. it doesn't modify the pointer
    std::weak_ptr<const GlobalParametersCore> TheGlobalParametersCorePtr;
public:
    int setTheGlobalParametersCore(std::weak_ptr<const GlobalParametersCore> TheGlobalParametersCorePtr);
    std::shared_ptr<const GlobalParametersCore> getTheGlobalParametersCore() const;



    // Gravity
protected:
public:
    Eigen::Vector3d gravity;
public:
    Eigen::Vector3d getGravity() const;
    int setGravity(Eigen::Vector3d gravity);



public:
    int updateStateFromIncrementErrorState(Eigen::VectorXd increment_error_state);


};



#endif
