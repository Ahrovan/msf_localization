
#ifndef _ABSOLUTE_POSE_INPUT_COMMAND_CORE_H
#define _ABSOLUTE_POSE_INPUT_COMMAND_CORE_H


#include <Eigen/Dense>


#include "msf_localization_core/input_command_core.h"

#include "msf_localization_core/quaternion_algebra.h"



class AbsolutePoseInputCommandCore : public InputCommandCore
{
public:
    AbsolutePoseInputCommandCore();
    AbsolutePoseInputCommandCore(std::weak_ptr<InputCore> input_core_ptr);
    ~AbsolutePoseInputCommandCore();

protected:
    int init();


    ///// Input Commands

    // Position Robot wrt World
protected:
    Eigen::Vector3d position_robot_wrt_world_;
public:
    int setPositionRobotWrtWorld(const Eigen::Vector3d& position_robot_wrt_world);
    Eigen::Vector3d getPositionRobotWrtWorld() const;


    // Attitude Robot wrt World
protected:
    Eigen::Vector4d attitude_robot_wrt_world_;
public:
    int setAttitudeRobotWrtWorld(const Eigen::Vector4d& attitude_robot_wrt_world);
    Eigen::Vector4d getAttitudeRobotWrtWorld() const;




    ///// Covariances Getters

    // Covariance Error Inputs: Qu
public:
    Eigen::SparseMatrix<double> getCovarianceInputs(const TimeStamp& deltaTimeStamp);



};


#endif
