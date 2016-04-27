
#ifndef _CODED_VISUAL_MARKER_EYE_CORE_H
#define _CODED_VISUAL_MARKER_EYE_CORE_H



#include <list>


// Time Stamp
#include "msf_localization_core/time_stamp.h"


// Sensor core
#include "msf_localization_core/sensor_core.h"

// VM Measurement
#include "msf_localization_core/coded_visual_marker_measurement_core.h"

// VM State
#include "msf_localization_core/coded_visual_marker_eye_state_core.h"


// Robot state
#include "msf_localization_core/robot_state_core.h"
#include "msf_localization_core/free_model_robot_state_core.h"


// Global parameters
#include "msf_localization_core/global_parameters_core.h"
#include "msf_localization_core/global_parameters_state_core.h"

// Map
#include "msf_localization_core/coded_visual_marker_landmark_core.h"
#include "msf_localization_core/coded_visual_marker_landmark_state_core.h"




class CodedVisualMarkerEyeCore : public SensorCore
{
public:
    CodedVisualMarkerEyeCore();
    CodedVisualMarkerEyeCore(std::weak_ptr<MsfStorageCore> the_msf_storage_core);
public:
    ~CodedVisualMarkerEyeCore();

public:
    int init();


    ///// Measurements

    // z=[{id_vmi, z_posi_vmi__wrt_aruco_eye, z_attit_vmi_wrt_aruco_eye}]'



    // Position Measurement
protected:
    bool flag_measurement_position_;
public:
    bool isMeasurementPositionEnabled() const;
    int enableMeasurementPosition();

    // position measurement covariance
protected:
    Eigen::Matrix3d noise_measurement_position_;
public:
    Eigen::Matrix3d getNoiseMeasurementPosition() const;
    int setNoiseMeasurementPosition(Eigen::Matrix3d noise_measurement_position);



    // Attitude Measurement
protected:
    bool flag_measurement_attitude_;
public:
    bool isMeasurementAttitudeEnabled() const;
    int enableMeasurementAttitude();

    // attitude measurement covariance
protected:
    Eigen::Matrix3d noise_measurement_attitude_;
public:
    Eigen::Matrix3d getNoiseMeasurementAttitude() const;
    int setNoiseMeasurementAttitude(Eigen::Matrix3d noise_measurement_attitude);




    // Store Measurement
public:
    //int setMeasurement(const TimeStamp the_time_stamp, std::shared_ptr<CodedVisualMarkerMeasurementCore> the_visual_marker_measurement);
    int setMeasurementList(const TimeStamp the_time_stamp, std::list< std::shared_ptr<SensorMeasurementCore> > the_visual_marker_measurement_list);







    ///// State estimation and parameters

    // Noise added to the landmarks




    ///// Get Covariances as a Eigen::MatrixXd
public:
    // Covariance Sensor Error Measurements
    Eigen::SparseMatrix<double> getCovarianceMeasurement();
    // Covariance Sensor Error Parameters
    Eigen::SparseMatrix<double> getCovarianceParameters();



    ////// Init error state variances -> Temporal, only for the initial configuration
public:
    int prepareInitErrorStateVariance();


public:
    Eigen::SparseMatrix<double> getCovarianceNoise(const TimeStamp deltaTimeStamp) const;


    ///// Predict functions

    // State: xs=[posi_sensor_wrt_robot, att_sensor_wrt_robot]'

    // Prediction state function
public:
    int predictState(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, const std::shared_ptr<SensorStateCore> pastState, std::shared_ptr<SensorStateCore>& predictedState);

    // Jacobian of the error state
public:
    int predictStateErrorStateJacobians(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, std::shared_ptr<SensorStateCore> pastState, std::shared_ptr<SensorStateCore>& predictedState);




    // Prediction measurements
public:
    int predictMeasurement(const TimeStamp theTimeStamp, const std::shared_ptr<GlobalParametersStateCore> TheGlobalParametersStateCore, const std::shared_ptr<RobotStateCore> currentRobotState, const std::shared_ptr<SensorStateCore> currentSensorState, const std::shared_ptr<MapElementStateCore> currentMapElementState, std::shared_ptr<CodedVisualMarkerMeasurementCore>& predictedMeasurement);


    // Jacobian of the measurements
public:
    int jacobiansMeasurements(const TimeStamp theTimeStamp, const std::shared_ptr<GlobalParametersStateCore> TheGlobalParametersStateCore, const std::shared_ptr<RobotStateCore> currentRobotState, const std::shared_ptr<SensorStateCore> currentSensorState, const std::shared_ptr<MapElementStateCore> currentMapElementState, std::shared_ptr<SensorMeasurementCore> matchedMeasurement, std::shared_ptr<CodedVisualMarkerMeasurementCore>& predictedMeasurement);




    // Mapping
public:
    int mapMeasurement(const TimeStamp theTimeStamp, const std::shared_ptr<GlobalParametersStateCore> TheGlobalParametersStateCore, const std::shared_ptr<RobotStateCore> currentRobotState, const std::shared_ptr<SensorStateCore> currentSensorState, const std::shared_ptr<SensorMeasurementCore> matchedMeasurement, std::shared_ptr<MapElementCore>& newMapElementCore, std::shared_ptr<MapElementStateCore>& newMapElementState);


public:
    int jacobiansMapMeasurement(const TimeStamp theTimeStamp, const std::shared_ptr<GlobalParametersStateCore> TheGlobalParametersStateCore, const std::shared_ptr<RobotStateCore> currentRobotState, const std::shared_ptr<SensorStateCore> currentSensorState, const std::shared_ptr<SensorMeasurementCore> matchedMeasurement, std::shared_ptr<MapElementStateCore>& newMapElementState);


};




#endif
