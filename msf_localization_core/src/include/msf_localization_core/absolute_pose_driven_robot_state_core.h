
#ifndef _ABSOLUTE_POSE_DRIVEN_ROBOT_STATE_CORE_H
#define _ABSOLUTE_POSE_DRIVEN_ROBOT_STATE_CORE_H


#include <Eigen/Dense>
#include <Eigen/Sparse>


#include "msf_localization_core/robot_state_core.h"


#include "msf_localization_core/quaternion_algebra.h"


class AbsolutePoseDrivenRobotStateCore : public RobotStateCore
{
public:
    AbsolutePoseDrivenRobotStateCore();
    AbsolutePoseDrivenRobotStateCore(const std::weak_ptr<MsfElementCore> msf_element_core_ptr);
    ~AbsolutePoseDrivenRobotStateCore();

protected:
    int init();


    // State: xR=[pos (3/3), attit (4/3)]'

protected:
public:
    Eigen::Vector3d position_robot_wrt_world_;
public:
    Eigen::Vector3d getPositionRobotWrtWorld() const;
    int setPositionRobotWrtWorld(const Eigen::Vector3d& position);



protected:
public:
    Eigen::Vector4d attitude_robot_wrt_world_;
public:
    Eigen::Vector4d getAttitudeRobotWrtWorld() const;
    int setAttitudeRobotWrtWorld(const Eigen::Vector4d &attitude);





    // Jacobian Error State (6 x 6)
    // Fx_robot linear (3 x 3)
    // Fx_robot angular (3 x 3)


    // Jacobian Error State Noise (6 x 6)



public:
    int updateStateFromIncrementErrorState(const Eigen::VectorXd& increment_error_state);

};



#endif
