
#ifndef _ROBOT_STATE_CORE_H
#define _ROBOT_STATE_CORE_H


#include "msf_localization_core/robot_core.h"

#include <Eigen/Dense>
#include <Eigen/Sparse>


class RobotStateCore
{
public:
    RobotStateCore();
    ~RobotStateCore();



    // Ptr to the robot core
protected:
    // It is not the owner of this Pointer. it doesn't modify the pointer
    std::weak_ptr<const RobotCore> TheRobotCorePtr;
public:
    int setTheRobotCore(std::weak_ptr<const RobotCore> TheRobotCorePtr);
    std::shared_ptr<const RobotCore> getTheRobotCore() const;




protected:
    // TODO RobotState
    // TODO RobotErrorState



protected:
    // TODO predictState()


public:
    virtual Eigen::SparseMatrix<double> getJacobianErrorState()=0;
    virtual Eigen::SparseMatrix<double> getJacobianErrorStateNoise()=0;


public:
    virtual int updateStateFromIncrementErrorState(Eigen::VectorXd increment_error_state)=0;


};



#endif
