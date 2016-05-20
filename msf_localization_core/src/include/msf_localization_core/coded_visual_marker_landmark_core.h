
#ifndef _CODED_VISUAL_MARKER_LANDMARK_CORE_H
#define _CODED_VISUAL_MARKER_LANDMARK_CORE_H


// Math
#include "cmath"

// Time Stamp
#include "msf_localization_core/time_stamp.h"

// Quaternion algebra
#include "msf_localization_core/quaternion_algebra.h"

// Map element Core
#include "msf_localization_core/map_element_core.h"

// State
#include "msf_localization_core/coded_visual_marker_landmark_state_core.h"


#include "pugixml/pugixml.hpp"


class CodedVisualMarkerLandmarkCore : public MapElementCore
{

public:
    CodedVisualMarkerLandmarkCore();
    CodedVisualMarkerLandmarkCore(const std::weak_ptr<MsfStorageCore> TheMsfStorageCore);
    ~CodedVisualMarkerLandmarkCore();


protected:
    int init();



    ///// Id
protected:
    int id_;
public:
    int getId() const;
    int setId(int id);


    /// Configs
public:
    int readConfig(const pugi::xml_node& map_element, std::shared_ptr<CodedVisualMarkerLandmarkStateCore>& MapElementInitStateCore);



    ///// State if enabled (or Parameters if disabled)

    // State: x_map=[t_land_wrt_world, q_land_wrt_world]'


    // Position Visual Marker Wrt World 3x1
protected:
    bool flag_estimation_position_visual_marker_wrt_world;
public:
    bool isEstimationPositionVisualMarkerWrtWorldEnabled();
    int enableEstimationPositionVisualMarkerWrtWorld();
    int enableParameterPositionVisualMarkerWrtWorld();


    // Covariance (if enabled estimation -> P; if no enabled estimation -> Sigma_mu)
protected:
    Eigen::Matrix3d covariancePositionVisualMarkerWrtWorld;
public:
    Eigen::Matrix3d getCovariancePositionVisualMarkerWrtWorld() const;
    int setCovariancePositionVisualMarkerWrtWorld(const Eigen::Matrix3d& covariancePositionVisualMarkerWrtWorld);



    // Attitude Visual Marker Wrt World 4x1 [3x1]
protected:
    bool flag_estimation_attitude_visual_marker_wrt_world;
public:
    bool isEstimationAttitudeVisualMarkerWrtWorldEnabled();
    int enableEstimationAttitudeVisualMarkerWrtWorld();
    int enableParameterAttitudeVisualMarkerWrtWorld();


    // Covariance (if enabled estimation -> P; if no enabled estimation -> Sigma_mu)
protected:
    Eigen::Matrix3d covarianceAttitudeVisualMarkerWrtWorld;
public:
    Eigen::Matrix3d getCovarianceAttitudeVisualMarkerWrtWorld() const;
    int setCovarianceAttitudeVisualMarkerWrtWorld(const Eigen::Matrix3d& covarianceAttitudeVisualMarkerWrtWorld);




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
                     const TimeStamp& previousTimeStamp, const TimeStamp& currentTimeStamp,
                     // Previous State
                     const std::shared_ptr<StateEstimationCore>& pastState,
                     // Inputs
                     const std::shared_ptr<InputCommandComponent>& inputCommand,
                     // Predicted State
                     std::shared_ptr<StateCore>& predictedState);

protected:
    int predictStateSpecific(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp,
                             const std::shared_ptr<CodedVisualMarkerLandmarkStateCore> pastState,
                             std::shared_ptr<CodedVisualMarkerLandmarkStateCore>& predictedState);


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
    int predictErrorStateJacobiansSpecific(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp,
                                           const std::shared_ptr<CodedVisualMarkerLandmarkStateCore> pastState,
                                           std::shared_ptr<CodedVisualMarkerLandmarkStateCore>& predictedState,
                                           // Jacobians Error State: Fx, Fp
                                           // Map
                                           Eigen::SparseMatrix<double>& jacobian_error_state_wrt_map_error_state,
                                           Eigen::SparseMatrix<double>& jacobian_error_state_wrt_map_error_parameters
                                           );



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
