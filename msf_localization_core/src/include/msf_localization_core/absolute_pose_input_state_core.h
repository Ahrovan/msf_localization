
#ifndef _ABSOLUTE_POSE_INPUT_STATE_CORE_H
#define _ABSOLUTE_POSE_INPUT_STATE_CORE_H


#include <Eigen/Dense>
#include <Eigen/Sparse>


#include "msf_localization_core/input_state_core.h"


#include "quaternion_algebra/quaternion_algebra.h"


class AbsolutePoseInputStateCore : public InputStateCore
{
public:
    AbsolutePoseInputStateCore();
    AbsolutePoseInputStateCore(const std::weak_ptr<MsfElementCore> msf_element_core_ptr);
    ~AbsolutePoseInputStateCore();

protected:
    int init();



    ///// State and parameters
    // xI=[posi_Input_wrt_robot, atti_Input_wrt_robot]


    // Input World



    // Pose of the input wrt robot
    // position of the input wrt robot
protected:
public:
    Eigen::Vector3d position_input_wrt_robot_;
public:
    Eigen::Vector3d getPositionInputWrtRobot() const;
    int setPositionInputWrtRobot(const Eigen::Vector3d& position_input_wrt_robot);


    // attitude of the input wrt robot
protected:
public:
    Eigen::Vector4d attitude_input_wrt_robot_;
public:
    Eigen::Vector4d getAttitudeInputWrtRobot() const;
    int setAttitudeInputWrtRobot(const Eigen::Vector4d& attitude_input_wrt_robot);




public:
    int updateStateFromIncrementErrorState(const Eigen::VectorXd& increment_error_state);

};


#endif
