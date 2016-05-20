
#ifndef _ABSOLUTE_POSE_INPUT_COMMAND_CORE_H
#define _ABSOLUTE_POSE_INPUT_COMMAND_CORE_H


#include <Eigen/Dense>


#include "msf_localization_core/input_command_core.h"

#include "msf_localization_core/quaternion_algebra.h"



class AbsolutePoseInputCommandCore : public InputCommandCore
{
public:
    AbsolutePoseInputCommandCore();
    AbsolutePoseInputCommandCore(const std::weak_ptr<InputCore> input_core_ptr);
    ~AbsolutePoseInputCommandCore();

protected:
    int init();


    ///// Input Commands

    // Position Input wrt Input World
protected:
    Eigen::Vector3d position_input_wrt_input_world_;
public:
    void setPositionInputWrtInputWorld(const Eigen::Vector3d& position_input_wrt_input_world);
    Eigen::Vector3d getPositionInputWrtInputWorld() const;


    // Attitude Input wrt Input World
protected:
    Eigen::Vector4d attitude_input_wrt_input_world_;
public:
    void setAttitudeInputWrtInputWorld(const Eigen::Vector4d& attitude_input_wrt_input_world);
    Eigen::Vector4d getAttitudeInputWrtInputWorld() const;


    // Noise Input Command
protected:
    Eigen::MatrixXd noise_input_command_pose_input_wrt_input_world_;
public:
    void setNoiseInputCommandPoseInputWrtInputWorld(const Eigen::MatrixXd& noise_input_command_pose_input_wrt_input_world);
    Eigen::MatrixXd  getNoiseInputCommandPoseInputWrtInputWorld() const;


    ///// Covariances Getters

    // Covariance Error Inputs: Qu
public:
    Eigen::SparseMatrix<double> getCovarianceInputs(const TimeStamp& deltaTimeStamp);



};


#endif
