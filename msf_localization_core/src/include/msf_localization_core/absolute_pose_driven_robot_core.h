
#ifndef _ABSOLUTE_POSE_DRIVEN_ROBOT_CORE_H
#define _ABSOLUTE_POSE_DRIVEN_ROBOT_CORE_H


// Math
#include "cmath"

// Time Stamp
#include "msf_localization_core/time_stamp.h"

// Quaternion algebra
#include "quaternion_algebra/quaternion_algebra.h"

// Robot Core
#include "msf_localization_core/robot_core.h"

// Robot State
#include "msf_localization_core/absolute_pose_driven_robot_state_core.h"

// Input State
#include "msf_localization_core/absolute_pose_input_state_core.h"

// Map Element STate
#include "msf_localization_core/world_reference_frame_state_core.h"

// Input Command
#include "msf_localization_core/absolute_pose_input_command_core.h"


#include "pugixml/pugixml.hpp"



class AbsolutePoseDrivenRobotCore : public RobotCore
{

public:
    AbsolutePoseDrivenRobotCore();
    AbsolutePoseDrivenRobotCore(MsfLocalizationCore* msf_localization_core_ptr);
    ~AbsolutePoseDrivenRobotCore();


protected:
    int init();


public:
    int readConfig(const pugi::xml_node& robot, std::shared_ptr<AbsolutePoseDrivenRobotStateCore>& RobotInitStateCore);



    ///// Prediction Noises


    // Noise Position
protected:
public:
    Eigen::Matrix3d noisePosition;
public:
    Eigen::Matrix3d getNoisePosition() const;
    int setNoisePosition(const Eigen::Matrix3d& noisePosition);

    // Noise Attitude
protected:
public:
    Eigen::Matrix3d noiseAttitude;
public:
    Eigen::Matrix3d getNoiseAttitude() const;
    int setNoiseAttitude(const Eigen::Matrix3d& noiseAttitude);



    ///// Covariances Getters

    // Covariance Error Parameters: Rp = Qp
public:
    Eigen::SparseMatrix<double> getCovarianceParameters();

    // Covariance Noise Estimation: Qn
public:
    Eigen::SparseMatrix<double> getCovarianceNoise(const TimeStamp deltaTimeStamp);



    ////// Init error state variances -> Temporal, only for the initial configuration

public:
    int prepareCovarianceInitErrorStateSpecific();

public:
    int setInitErrorStateVariancePosition(const Eigen::Vector3d& initVariance);
    int setInitErrorStateVarianceAttitude(const Eigen::Vector3d &initVariance);







    ///// Predict Step functions

    // State: xR=[pos, attit]'

    // Prediction state function: f()
public:
    int predictState(//Time
                     const TimeStamp& previousTimeStamp, const TimeStamp& currentTimeStamp,
                     // Previous State
                     const std::shared_ptr<StateComponent>& past_state,
                     // Inputs
                     const std::shared_ptr<InputCommandComponent>& input_command,
                     // Predicted State
                     std::shared_ptr<StateCore>& predicted_state);

protected:
    int predictStateSpecific(const TimeStamp& previousTimeStamp, const TimeStamp& currentTimeStamp,
                             const AbsolutePoseDrivenRobotStateCore* past_robot_state,
                             const AbsolutePoseInputStateCore* past_input_state,
                             const WorldReferenceFrameStateCore* past_map_element_state,
                             const AbsolutePoseInputCommandCore* past_input_command,
                             AbsolutePoseDrivenRobotStateCore*& predicted_robot_state);

    // int predictStateSpecificCore();


    // Jacobian: F
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
    int predictErrorStateJacobianSpecific(const TimeStamp& previousTimeStamp, const TimeStamp& currentTimeStamp,
                                          const AbsolutePoseDrivenRobotStateCore* past_robot_state,
                                          const AbsolutePoseInputStateCore* past_input_state,
                                          const WorldReferenceFrameStateCore* past_map_element_state,
                                          const AbsolutePoseInputCommandCore* past_input_command,
                                          AbsolutePoseDrivenRobotStateCore*& predictedState,
                                          // Jacobians Error State: Fx, Fp
                                          // Robot State
                                          Eigen::SparseMatrix<double>& jacobian_error_state_wrt_robot_error_state,
                                          Eigen::SparseMatrix<double>& jacobian_error_state_wrt_robot_error_parameters,
                                          // Input State
                                          Eigen::SparseMatrix<double>& jacobian_error_state_wrt_input_error_state,
                                          Eigen::SparseMatrix<double>& jacobian_error_state_wrt_input_error_parameters,
                                          // Map Element State
                                          Eigen::SparseMatrix<double>& jacobian_error_state_wrt_map_element_error_state,
                                          Eigen::SparseMatrix<double>& jacobian_error_state_wrt_map_element_error_parameters,
                                          // Jacobian Input: Fu
                                          // Input Command
                                          Eigen::SparseMatrix<double>& jacobian_error_state_wrt_input_command,
                                          // Jacobians Noise: Hn
                                          Eigen::SparseMatrix<double>& jacobian_error_state_wrt_noise
                                          );

    // TODO FINISH!!
    int predictErrorStateJacobianSpecificCore(// State k
                                              // Robot
                                              const Eigen::Vector3d& position_robot_wrt_world,
                                              const Eigen::Vector4d& attitude_robot_wrt_world,
                                              // Input State
                                              const Eigen::Vector3d& position_input_wrt_robot,
                                              const Eigen::Vector4d& attitude_input_wrt_robot,
                                              // Map Element State
                                              const Eigen::Vector3d& position_input_world_wrt_world,
                                              const Eigen::Vector4d& attitude_input_world_wrt_world,
                                              // Input Command
                                              const Eigen::Vector3d& position_input_wrt_input_world,
                                              const Eigen::Vector4d& attitude_input_wrt_input_world,
                                              // State k+1
                                              const Eigen::Vector3d& estim_position_robot_wrt_world,
                                              const Eigen::Vector4d& estim_attitude_robot_wrt_world,
                                              // Jacobians Error State: Fx, Fp
                                              // Input
                                              Eigen::Matrix3d &jacobian_error_state_robot_pos_wrt_input_error_state_pos,
                                              Eigen::Matrix3d &jacobian_error_state_robot_pos_wrt_input_error_state_att,
                                              Eigen::Matrix3d &jacobian_error_state_robot_att_wrt_input_error_state_att,
                                              // Map Element
                                              Eigen::Matrix3d &jacobian_error_state_robot_pos_wrt_map_element_error_state_pos,
                                              Eigen::Matrix3d &jacobian_error_state_robot_pos_wrt_map_element_error_state_att,
                                              Eigen::Matrix3d &jacobian_error_state_robot_att_wrt_map_element_error_state_att,
                                              // Jacobian Input: Fu
                                              Eigen::Matrix3d &jacobian_error_state_robot_pos_wrt_input_command_pos,
                                              Eigen::Matrix3d &jacobian_error_state_robot_pos_wrt_input_command_att,
                                              Eigen::Matrix3d &jacobian_error_state_robot_att_wrt_input_command_att,
                                              // Jacobians Noise: Fn
                                              Eigen::Matrix3d& jacobian_error_state_pos_wrt_noise_pos,
                                              Eigen::Matrix3d& jacobian_error_state_att_wrt_noise_att
                                              );


    //// Update Step Functions

    // None

public:
    int resetErrorStateJacobian(// Time
                                const TimeStamp& current_time_stamp,
                                // Increment Error State
                                const Eigen::VectorXd& increment_error_state,
                                // Current State
                                std::shared_ptr<StateCore>& current_state
                                );


};


#endif
