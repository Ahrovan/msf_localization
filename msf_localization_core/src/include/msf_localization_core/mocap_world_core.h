
#ifndef _MOCAP_WORLD_CORE_H
#define _MOCAP_WORLD_CORE_H



// Math
#include "cmath"

// Time Stamp
#include "msf_localization_core/time_stamp.h"

// Quaternion algebra
#include "msf_localization_core/quaternion_algebra.h"

// Map element Core
#include "msf_localization_core/map_element_core.h"

// State
#include "msf_localization_core/mocap_world_state_core.h"


#include "pugixml/pugixml.hpp"


class MocapWorldCore : public MapElementCore
{

public:
    MocapWorldCore();
    MocapWorldCore(std::weak_ptr<MsfStorageCore> TheMsfStorageCore);
    ~MocapWorldCore();


protected:
    int init();




    /// Configs
public:
    int readConfig(pugi::xml_node map_element, std::shared_ptr<MocapWorldStateCore>& MapElementInitStateCore);



    ///// State if enabled (or Parameters if disabled)

    // State: x_map=[t_mocap_world_wrt_world, q_mocap_world_wrt_world]'


    // Position Mocap World Wrt World 3x1
protected:
    bool flag_estimation_position_mocap_world_wrt_world_;
public:
    bool isEstimationPositionMocapWorldWrtWorldEnabled();
    int enableEstimationPositionMocapWorldWrtWorld();
    int enableParameterPositionMocapWorldWrtWorld();


    // Covariance (if enabled estimation -> P; if no enabled estimation -> Sigma_mu)
protected:
    Eigen::Matrix3d covariance_position_mocap_world_wrt_world_;
public:
    Eigen::Matrix3d getCovariancePositionMocapWorldWrtWorld() const;
    int setCovariancePositionMocapWorldWrtWorld(Eigen::Matrix3d covariance_position_mocap_world_wrt_world);



    // Attitude Mocap World Wrt World 4x1 [3x1]
protected:
    bool flag_estimation_attitude_mocap_world_wrt_world_;
public:
    bool isEstimationAttitudeMocapWorldWrtWorldEnabled();
    int enableEstimationAttitudeMocapWorldWrtWorld();
    int enableParameterAttitudeMocapWorldWrtWorld();


    // Covariance (if enabled estimation -> P; if no enabled estimation -> Sigma_mu)
protected:
    Eigen::Matrix3d covariance_attitude_mocap_world_wrt_world_;
public:
    Eigen::Matrix3d getCovarianceAttitudeMocapWorldWrtWorld() const;
    int setCovarianceAttitudeMocapWorldWrtWorld(Eigen::Matrix3d covariance_attitude_mocap_world_wrt_world);




    ////// Init error state covariances -> Temporal, only for the initial configuration
public:
    int prepareCovarianceInitErrorStateSpecific();




    ///// Covariances Getters

    // Covariance Error Parameters: Rp = Qp
public:
    Eigen::SparseMatrix<double> getCovarianceParameters();

    // Covariance Noise Estimation: Qn
public:
    Eigen::SparseMatrix<double> getCovarianceNoise(const TimeStamp deltaTimeStamp);





    ///// Predict step functions

    // State: xL=[pos, attit]'

    // Prediction state function: f()
public:
    int predictState(//Time
                     const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp,
                     // Previous State
                     const std::shared_ptr<StateEstimationCore> pastState,
                     // Inputs
                     const std::shared_ptr<InputCommandComponent> inputCommand,
                     // Predicted State
                     std::shared_ptr<StateCore>& predictedState);

protected:
    int predictStateSpecific(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp,
                             const std::shared_ptr<MocapWorldStateCore> pastState,
                             std::shared_ptr<MocapWorldStateCore>& predictedState);


    // Jacobian: F
public:
    int predictErrorStateJacobian(//Time
                                 const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp,
                                 // Previous State
                                 const std::shared_ptr<StateEstimationCore> pastState,
                                 // Inputs
                                 const std::shared_ptr<InputCommandComponent> inputCommand,
                                 // Predicted State
                                 std::shared_ptr<StateCore>& predictedState);

protected:
    int predictErrorStateJacobiansSpecific(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp,
                                   const std::shared_ptr<MocapWorldStateCore> pastState,
                                   std::shared_ptr<MocapWorldStateCore>& predictedState);



    //// Update step functions

    // NONE



    /// Reset Error State

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