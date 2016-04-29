
#ifndef _ROBOT_STATE_CORE_H
#define _ROBOT_STATE_CORE_H


#include "msf_localization_core/state_core.h"


#include <Eigen/Dense>
#include <Eigen/Sparse>



enum class RobotStateCoreTypes
{
    undefined=0,
    free_model=1,
    imu_driven
};




class RobotStateCore : public StateCore
{
public:
    RobotStateCore();
    RobotStateCore(std::weak_ptr<MsfElementCore> msf_element_core_ptr);
    ~RobotStateCore();


protected:
    int init();


protected:
    RobotStateCoreTypes robot_state_core_type_;
public:
    int setRobotStateType(RobotStateCoreTypes robot_state_core_type);
    RobotStateCoreTypes getRobotStateType();




protected:
    // TODO predictState()





};



#endif
