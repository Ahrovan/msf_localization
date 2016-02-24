
#ifndef _FREE_MODEL_ROBOT_CORE_H
#define _FREE_MODEL_ROBOT_CORE_H


// Time Stamp
#include "time_stamp.h"

// Quaternion algebra
#include "quaternion_algebra.h"

// Robot Core
#include "robot_core.h"

// State
#include "free_model_robot_state_core.h"


class FreeModelRobotCore : public RobotCore
{

public:
    FreeModelRobotCore();
    ~FreeModelRobotCore();




    ///// Predict functions

    // State: xR=[pos, lin_speed, lin_accel, attit, ang_vel]'

    // Prediction state function
public:
    int predictState(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, const std::shared_ptr<FreeModelRobotStateCore> pastState, std::shared_ptr<FreeModelRobotStateCore>& predictedState);

    // Jacobian
public:
    int predictStateErrorStateJacobians(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, std::shared_ptr<FreeModelRobotStateCore> pastState, std::shared_ptr<FreeModelRobotStateCore>& predictedState);




};





#endif
