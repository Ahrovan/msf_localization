
#ifndef _STATE_CORE_H
#define _STATE_CORE_H


#include <Eigen/Dense>

#include "msf_localization_core/msf_element_core.h"



enum class StateCoreTypes
{
    undefined=0,
    input,
    sensor,
    robot,
    map,
    world
};



class StateCore
{
public:
    StateCore();
    StateCore(std::weak_ptr<MsfElementCore> msf_element_core_ptr);
    ~StateCore();


protected:
    int init();



    // Ptr to MsfElementCore
protected:
    // It is not the owner of this Pointer. it doesn't modify the pointer
    std::weak_ptr<MsfElementCore> msf_element_core_ptr_;
public:
    int setMsfElementCorePtr(std::weak_ptr<MsfElementCore> msf_element_core_ptr);
    std::shared_ptr<MsfElementCore> getMsfElementCoreSharedPtr() const;
    std::weak_ptr<MsfElementCore> getMsfElementCoreWeakPtr() const;


    // Type
protected:
    StateCoreTypes state_core_type_;
public:
    int setStateCoreType(StateCoreTypes state_core_type);
    StateCoreTypes getStateCoreType() const;



/*

public:
    virtual Eigen::MatrixXd getJacobianErrorState()=0;
    virtual Eigen::SparseMatrix<double> getJacobianErrorStateNoise()=0;


public:
    virtual int updateStateFromIncrementErrorState(Eigen::VectorXd increment_error_state)=0;
*/

};





#endif
