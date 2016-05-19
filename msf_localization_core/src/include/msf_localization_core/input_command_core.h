
#ifndef _INPUT_COMMAND_CORE_H
#define _INPUT_COMMAND_CORE_H


#include <memory>


#include <Eigen/Dense>


#include "msf_localization_core/input_core.h"


enum class InputCommandTypes
{
    undefined=0,
    imu=1,
};


class InputCommandCore
{
public:
    InputCommandCore();
    InputCommandCore(std::weak_ptr<InputCore> input_core_ptr);
    ~InputCommandCore();

protected:
    int init();

protected:
    // It is not the owner of this Pointer. it doesn't modify the pointer
    std::weak_ptr<InputCore> input_core_ptr_;
public:
    int setInputCorePtr(std::weak_ptr<InputCore> input_core_ptr);
    std::weak_ptr<InputCore> getInputCoreWeakPtr() const;
    std::shared_ptr<InputCore> getInputCoreSharedPtr() const;


protected:
    InputCommandTypes input_command_type_;
public:
    int setInputCommandType(InputCommandTypes input_command_type);
    InputCommandTypes getInputCommandType() const;





    ///// Covariances Getters

    // Covariance Error Inputs: Qu
public:
    virtual Eigen::SparseMatrix<double> getCovarianceInputs(const TimeStamp& deltaTimeStamp)=0;




};



#endif
