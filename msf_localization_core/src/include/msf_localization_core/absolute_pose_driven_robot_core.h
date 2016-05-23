
#ifndef _ABSOLUTE_POSE_DRIVEN_ROBOT_CORE_H
#define _ABSOLUTE_POSE_DRIVEN_ROBOT_CORE_H


// Math
#include "cmath"

// Time Stamp
#include "msf_localization_core/time_stamp.h"

// Quaternion algebra
#include "msf_localization_core/quaternion_algebra.h"

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
    AbsolutePoseDrivenRobotCore(const std::weak_ptr<MsfStorageCore> msf_storage_core_ptr);
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
                     const std::shared_ptr<StateEstimationCore>& pastState,
                     // Inputs
                     const std::shared_ptr<InputCommandComponent>& inputCommand,
                     // Predicted State
                     std::shared_ptr<StateCore>& predictedState);

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
                                 const std::shared_ptr<StateEstimationCore>& past_state,
                                  // Inputs
                                  const std::shared_ptr<InputCommandComponent>& input_command,
                                 // Predicted State
                                 std::shared_ptr<StateCore>& predicted_state);

protected:
    int predictErrorStateJacobianSpecific(const TimeStamp& previousTimeStamp, const TimeStamp& currentTimeStamp,
                                          const AbsolutePoseDrivenRobotStateCore* pastState,
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

    // int predictErrorStateJacobianSpecificCore();


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
