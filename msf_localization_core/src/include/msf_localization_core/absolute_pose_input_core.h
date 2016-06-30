
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
    AbsolutePoseInputCore(MsfLocalizationCore* msf_localization_core_ptr);
    ~AbsolutePoseInputCore();

protected:
    int init();

public:
    int readConfig(const pugi::xml_node& input, std::shared_ptr<AbsolutePoseInputStateCore>& init_state_core);




    ///// Inputs Command

    // u=[u_posi, u_attit]'


    /// World Reference Frame id
protected:
    int world_reference_frame_id_;
public:
    int setWorldReferenceFrameId(int world_reference_frame_id);
    int getWorldReferenceFrameId() const;


    /// Position Input Wrt Input World Command
protected:
    bool flag_input_command_position_input_wrt_input_world_;
public:
    bool isInputCommandPositionInputWrtInputWorldEnabled() const;
    int enableInputCommandPositionInputWrtInputWorld();

protected:
    Eigen::Matrix3d noise_input_command_position_input_wrt_input_world_;
public:
    Eigen::Matrix3d getNoiseInputCommandPositionInputWrtInputWorld() const;
    void setNoiseInputCommandPositionInputWrtInputWorld(const Eigen::Matrix3d& noise_input_command_position_input_wrt_input_world);


    /// Attitude Input Wrt Input World Command
protected:
    bool flag_input_command_attitude_input_wrt_input_world_;
public:
    bool isInputCommandAttitudeInputWrtInputWorldEnabled() const;
    int enableInputCommandAttitudeInputWrtInputWorld();

protected:
    Eigen::Matrix3d noise_input_command_attitude_input_wrt_input_world_;
public:
    Eigen::Matrix3d getNoiseInputCommandAttitudeInputWrtInputWorld() const;
    void setNoiseInputCommandAttitudeInputWrtInputWorld(const Eigen::Matrix3d& noise_input_command_attitude_input_wrt_input_world);


    /// Noise Input Command Pose Input Wrt Input World
    // The noise of the input command is not set externally, but it comes with the input command
protected:
    bool flag_input_command_pose_input_wrt_input_world_has_covariance_;
public:
    bool hasInputCommandPoseInputWrtInputWorldCovariance() const;
    void setInputCommandPoseInputWrtInputWorldHasCovariance(bool flag_input_command_pose_input_wrt_input_world_has_covariance);



    /// Store Input Command
public:
    int setInputCommand(const TimeStamp& time_stamp, const std::shared_ptr<AbsolutePoseInputCommandCore> input_command_core);




    ///// State and parameters

    // xI=[posi_Input_wrt_robot, atti_Input_wrt_robot]


    /// Position input wrt robot

    // Position sensor wrt robot: t_sensor_wrt_robot (3x1)
protected:
    bool flag_estimation_position_input_wrt_robot_;
public:
    bool isEstimationPositionInputWrtRobotEnabled() const;
    int enableEstimationPositionInputWrtRobot();
    int enableParameterPositionInputWrtRobot();

    // Position sensor wrt robot: covariance (if enabled estimation -> P; if no enabled estimation -> Rp = Qp)
protected:
    Eigen::Matrix3d noise_position_input_wrt_robot_;
public:
    Eigen::Matrix3d getNoisePositionInputWrtRobot() const;
    void setNoisePositionInputWrtRobot(const Eigen::Matrix3d& noise_position_input_wrt_robot);



    /// Attitude Input Wrt Robot

    // Attitude sensor wrt robot: q_sensor_wrt_robot (4x1) [Theta_sensor_wrt_robot (3x1)]
protected:
    bool flag_estimation_attitude_input_wrt_robot_;
public:
    bool isEstimationAttitudeInputWrtRobotEnabled() const;
    int enableEstimationAttitudeInputWrtRobot();
    int enableParameterAttitudeInputWrtRobot();

    // Attitude sensor wrt robot: covariance (if enabled estimation -> P; if no enabled estimation -> Rp = Qp)
protected:
    Eigen::Matrix3d noise_attitude_input_wrt_robot_;
public:
    Eigen::Matrix3d getNoiseAttitudeInputWrtRobot() const;
    void setNoiseAttitudeInputWrtRobot(const Eigen::Matrix3d& noise_attitude_input_wrt_robot);






    ////// Init error state variances -> Temporal, only for the initial configuration
public:
    int prepareCovarianceInitErrorStateSpecific();



    ///// Covariances Getters

    // Covariance Error Inputs: Qu
public:
    Eigen::SparseMatrix<double> getCovarianceInputs(const TimeStamp deltaTimeStamp);

    // Covariance Error Parameters: Qp = Rp
public:
    Eigen::SparseMatrix<double> getCovarianceParameters();

    // Covariance Noise Estimation: Qn
public:
    Eigen::SparseMatrix<double> getCovarianceNoise(const TimeStamp deltaTimeStamp);






    //// Predict Step Functions


    // Prediction state function: f()
public:
    int predictState(//Time
                     const TimeStamp& previousTimeStamp, const TimeStamp& currentTimeStamp,
                     // Previous State
                     const std::shared_ptr<StateComponent>& pastState,
                     // Inputs
                     const std::shared_ptr<InputCommandComponent>& inputCommand,
                     // Predicted State
                     std::shared_ptr<StateCore>& predictedState);

protected:
    int predictStateSpecific(const TimeStamp& previousTimeStamp, const TimeStamp& currentTimeStamp,
                             const std::shared_ptr<AbsolutePoseInputStateCore> pastState,
                             std::shared_ptr<AbsolutePoseInputStateCore>& predictedState);

protected:
    int predictStateCore(// State k: Input
                         const Eigen::Vector3d& position_input_wrt_robot, const Eigen::Vector4d& attitude_input_wrt_robot,
                         // State k+1: Input
                         Eigen::Vector3d& pred_position_input_wrt_robot, Eigen::Vector4d& pred_attitude_input_wrt_robot);




    // Jacobian error state: F
public:
    int predictErrorStateJacobian(//Time
                                 const TimeStamp& previousTimeStamp, const TimeStamp& currentTimeStamp,
                                 // Previous State
                                 const std::shared_ptr<StateComponent>& past_state,
                                 // Inputs
                                 const std::shared_ptr<InputCommandComponent>& input_command,
                                 // Predicted State
                                 std::shared_ptr<StateCore>& predicted_state);

protected:
    int predictErrorStateJacobiansSpecific(const TimeStamp& previousTimeStamp, const TimeStamp& currentTimeStamp,
                                           const std::shared_ptr<AbsolutePoseInputStateCore> pastState,
                                           std::shared_ptr<AbsolutePoseInputStateCore>& predictedState,
                                           // Jacobians Error State: Fx, Fp
                                           // Sensor
                                           Eigen::SparseMatrix<double>& jacobian_error_state_wrt_input_error_state,
                                           Eigen::SparseMatrix<double>& jacobian_error_state_wrt_input_error_parameters
                                           // Jacobians Noise: Hn
                                           // Nothing
                                           );
protected:
    // TODO Fix!!
    int predictErrorStateJacobiansCore(// State k: Sensor
                                       const Eigen::Vector3d& position_sensor_wrt_robot, const Eigen::Vector4d& attitude_sensor_wrt_robot,
                                       // State k+1: Sensor
                                       const Eigen::Vector3d& pred_position_sensor_wrt_robot, const Eigen::Vector4d& pred_attitude_sensor_wrt_robot,
                                       // Jacobian: State
                                       Eigen::Matrix3d& jacobian_error_sens_pos_wrt_error_state_sens_pos,  Eigen::Matrix3d& jacobian_error_sens_att_wrt_error_state_sens_att);





    //// Update Step Functions

    // NONE


public:
    int resetErrorStateJacobian(// Time
                                const TimeStamp& current_time_stamp,
                                // Increment Error State
                                const Eigen::VectorXd& increment_error_state,
                                // Current State
                                std::shared_ptr<StateCore>& current_state
                                );


    /// Auxiliar functions
protected:
    int findState(const std::list< std::shared_ptr<StateCore> >& list_state,
                  std::shared_ptr<AbsolutePoseInputStateCore>& found_state);

};



#endif
