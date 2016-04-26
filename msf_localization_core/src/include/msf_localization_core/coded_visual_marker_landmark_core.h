
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


class CodedVisualMarkerLandmarkCore : public MapElementCore
{

public:
    CodedVisualMarkerLandmarkCore();
    CodedVisualMarkerLandmarkCore(std::weak_ptr<MapElementCore> the_map_element_core_ptr, std::weak_ptr<MsfStorageCore> TheMsfStorageCore);
    ~CodedVisualMarkerLandmarkCore();


protected:
    int init();



    ///// Id
protected:
    int id_;
public:
    int getId() const;
    int setId(int id);


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
    int setCovariancePositionVisualMarkerWrtWorld(Eigen::Matrix3d covariancePositionVisualMarkerWrtWorld);



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
    int setCovarianceAttitudeVisualMarkerWrtWorld(Eigen::Matrix3d covarianceAttitudeVisualMarkerWrtWorld);




    ////// Init error state covariances -> Temporal, only for the initial configuration
public:
    Eigen::MatrixXd getInitCovarianceErrorState();


    ///// Get Covariances as a Eigen::MatrixXd
public:
    // Covariance Map Element Error Measurements
    //Eigen::SparseMatrix<double> getCovarianceMeasurement();
    // Covariance Map Element Error Parameters
    Eigen::SparseMatrix<double> getCovarianceParameters();

public:
    // Covariance Map Element Error Noise
    Eigen::SparseMatrix<double> getCovarianceNoise(const TimeStamp deltaTimeStamp) const;






    ///// Predict functions

    // State: xL=[pos, attit]'

    // Prediction state function
public:
    int predictState(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, const std::shared_ptr<MapElementStateCore> pastState, std::shared_ptr<MapElementStateCore>& predictedState);

    // Jacobian
public:
    int predictStateErrorStateJacobians(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, std::shared_ptr<MapElementStateCore> pastState, std::shared_ptr<MapElementStateCore>& predictedState);


    /*
    // Prediction measurements
public:
    int predictMeasurement(const TimeStamp theTimeStamp, std::shared_ptr<GlobalParametersStateCore> currentGlobalParametersState, const std::shared_ptr<RobotStateCore> currentRobotState, const std::shared_ptr<CodedVisualMarkerEyeStateCore> currentSensorState, std::shared_ptr<CodedVisualMarkerMeasurementCore>& predictedMeasurement);


    // Jacobian of the measurements
public:
    int jacobiansMeasurements(const TimeStamp theTimeStamp, std::shared_ptr<GlobalParametersStateCore> currentGlobalParametersState, std::shared_ptr<RobotStateCore> currentRobotState, std::shared_ptr<CodedVisualMarkerEyeStateCore> currentSensorState, std::shared_ptr<CodedVisualMarkerMeasurementCore>& predictedMeasurement);

    */

};





#endif
