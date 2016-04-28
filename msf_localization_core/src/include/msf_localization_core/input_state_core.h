
#ifndef _INPUT_STATE_CORE_H
#define _INPUT_STATE_CORE_H



#include "msf_localization_core/state_core.h"


#include <Eigen/Dense>
#include <Eigen/Sparse>



enum class InputStateCoreTypes
{
    undefined=0,
    imu=1
};




class InputStateCore : public StateCore
{
public:
    InputStateCore();
    InputStateCore(std::weak_ptr<MsfElementCore> msf_element_core_ptr);
    ~InputStateCore();


protected:
    int init();


protected:
    InputStateCoreTypes input_state_core_type_;
public:
    int setInputStateType(InputStateCoreTypes input_state_core_type);
    InputStateCoreTypes getInputStateType();




protected:
    // TODO predictState()


public:
    virtual Eigen::SparseMatrix<double> getJacobianErrorState()=0;
    virtual Eigen::SparseMatrix<double> getJacobianErrorStateNoise()=0;


public:
    virtual int updateStateFromIncrementErrorState(Eigen::VectorXd increment_error_state)=0;


};





#endif
