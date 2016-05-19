
#ifndef _ABSOLUTE_POSE_INPUT_CORE_H
#define _ABSOLUTE_POSE_INPUT_CORE_H


#include "msf_localization_core/input_core.h"


#include "msf_localization_core/absolute_pose_input_command_core.h"

#include "msf_localization_core/absolute_pose_input_state_core.h"


#include "pugixml/pugixml.hpp"


class AbsolutePoseInputCore : public InputCore
{
public:
    AbsolutePoseInputCore();
    AbsolutePoseInputCore(std::weak_ptr<MsfStorageCore> the_msf_storage_core);
    ~AbsolutePoseInputCore();

protected:
    int init();

public:
    int readConfig(pugi::xml_node input, std::shared_ptr<AbsolutePoseInputStateCore>& init_state_core);




    ///// Inputs Command

    // u=[u_posi, u_attit]'



    /// Position Robot Wrt World Input Command
protected:
    bool flag_input_command_position_robot_wrt_world_;
public:
    bool isInputCommandPositionRobotWrtWorldEnabled() const;
    int enableInputCommandPositionRobotWrtWorld();

    // position Input Command covariance
protected:
    Eigen::Matrix3d noise_input_command_position_robot_wrt_world_;
public:
    Eigen::Matrix3d getNoiseInputCommandPositionRobotWrtWorld() const;
    int setNoiseInputCommandPositionRobotWrtWorld(const Eigen::Matrix3d& noise_input_command_position_robot_wrt_world);


    /// Attitude Robot Wrt World Input Command
protected:
    bool flag_input_command_attitude_robot_wrt_world_;
public:
    bool isInputCommandAttitudeRobotWrtWorldEnabled() const;
    int enableInputCommandAttitudeRobotWrtWorld();

    // attitude Input Command covariance
protected:
    Eigen::Matrix3d noise_input_command_attitude_robot_wrt_world_;
public:
    Eigen::Matrix3d getNoiseInputCommandAttitudeRobotWrtWorld() const;
    int setNoiseInputCommandAttitudeRobotWrtWorld(Eigen::Matrix3d noise_input_command_attitude_robot_wrt_world);



    /// Store Input Command
public:
    int setInputCommand(const TimeStamp time_stamp, std::shared_ptr<AbsolutePoseInputCommandCore> input_command_core);




    ///// State and parameters
    // None






    ////// Init error state variances -> Temporal, only for the initial configuration
public:
    int prepareCovarianceInitErrorStateSpecific();



    ///// Covariances Getters

    // Covariance Error Inputs: Qu
public:
    Eigen::SparseMatrix<double> getCovarianceInputs(const TimeStamp deltaTimeStamp);

    // Covariance Error Parameters: Rp = Qp
public:
    Eigen::SparseMatrix<double> getCovarianceParameters();

    // Covariance Noise Estimation: Qn
public:
    Eigen::SparseMatrix<double> getCovarianceNoise(const TimeStamp deltaTimeStamp);






    //// Predict Step Functions


    // Prediction state function: f()
public:
    // TODO
    int predictState(//Time
                     const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp,
                     // Previous State
                     const std::shared_ptr<StateEstimationCore> pastState,
                     // Inputs
                     const std::shared_ptr<InputCommandComponent> inputCommand,
                     // Predicted State
                     std::shared_ptr<StateCore>& predictedState);

protected:
//    int predictStateSpecific(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp,
//                             const std::shared_ptr<AbsolutePoseInputStateCore> pastState,
//                             std::shared_ptr<AbsolutePoseInputStateCore>& predictedState);

    // int predictStateSpecificCore();



    // Jacobian: F
public:
    // TODO
    int predictErrorStateJacobian(//Time
                                 const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp,
                                 // Previous State
                                 const std::shared_ptr<StateEstimationCore> pastState,
                                  // Inputs
                                  const std::shared_ptr<InputCommandComponent> inputCommand,
                                 // Predicted State
                                 std::shared_ptr<StateCore>& predictedState);

protected:
//    int predictErrorStateJacobianSpecific(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp,
//                                          const std::shared_ptr<AbsolutePoseInputStateCore> pastState,
//                                          std::shared_ptr<AbsolutePoseInputStateCore>& predictedState);

    // int predictErrorStateJacobianSpecificCore();




    //// Update Step Functions

    // NONE


public:
    // TODO
    int resetErrorStateJacobian(// Time
                                const TimeStamp& current_time_stamp,
                                // Increment Error State
                                const Eigen::VectorXd& increment_error_state,
                                // Current State
                                std::shared_ptr<StateCore>& current_state
                                );


};



#endif
