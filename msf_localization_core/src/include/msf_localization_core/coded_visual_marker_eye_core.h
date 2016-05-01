
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
#include "msf_localization_core/imu_driven_robot_state_core.h"


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


public:
    int readConfig(pugi::xml_node sensor, unsigned int sensorId, std::shared_ptr<CodedVisualMarkerEyeStateCore>& SensorInitStateCore);




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




    ///// Covariances getters

    // Covariance Sensor Error Measurements: Rn
public:
    Eigen::SparseMatrix<double> getCovarianceMeasurement();

    // Covariance Sensor Error Parameters: Qp = Rp
public:
    Eigen::SparseMatrix<double> getCovarianceParameters();

    // Covariance Noise Estimation: Qn
public:
    Eigen::SparseMatrix<double> getCovarianceNoise(const TimeStamp deltaTimeStamp);




    ////// Init error state variances -> Temporal, only for the initial configuration
public:
    int prepareCovarianceInitErrorStateSpecific();





    ///// Predict functions

    // State: xs=[posi_sensor_wrt_robot, att_sensor_wrt_robot]'

    // Prediction state function: f()
public:
    int predictState(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, const std::shared_ptr<SensorStateCore> pastState, std::shared_ptr<SensorStateCore>& predictedState);
protected:
    int predictStateCore(// State k: Sensor
                         Eigen::Vector3d position_sensor_wrt_robot, Eigen::Vector4d attitude_sensor_wrt_robot,
                         // State k+1: Sensor
                         Eigen::Vector3d& pred_position_sensor_wrt_robot, Eigen::Vector4d& pred_attitude_sensor_wrt_robot);


    // Jacobian of the error state: F
public:
    int predictErrorStateJacobians(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, std::shared_ptr<SensorStateCore> pastState, std::shared_ptr<SensorStateCore>& predictedState);
protected:
    // TODO Fix!!
    int predictErrorStateJacobiansCore(// State k: Sensor
                                       Eigen::Vector3d position_sensor_wrt_robot, Eigen::Vector4d attitude_sensor_wrt_robot,
                                       // State k+1: Sensor
                                       Eigen::Vector3d pred_position_sensor_wrt_robot, Eigen::Vector4d pred_attitude_sensor_wrt_robot,
                                       // Jacobian: State
                                       Eigen::Matrix3d& jacobian_error_sens_pos_wrt_error_state_sens_pos,  Eigen::Matrix3d& jacobian_error_sens_att_wrt_error_state_sens_att);




    ///// Update functions


    /// State Correction

    // Prediction measurements: h()
public:
    int predictMeasurement(const TimeStamp theTimeStamp, const std::shared_ptr<GlobalParametersStateCore> TheGlobalParametersStateCore, const std::shared_ptr<RobotStateCore> currentRobotState, const std::shared_ptr<SensorStateCore> currentSensorState, const std::shared_ptr<MapElementStateCore> currentMapElementState, std::shared_ptr<CodedVisualMarkerMeasurementCore>& predictedMeasurement);
protected:
    int predictMeasurementCore(// State: Robot
                               Eigen::Vector3d position_robot_wrt_world, Eigen::Vector4d attitude_robot_wrt_world,
                               // State: Sensor
                               Eigen::Vector3d position_sensor_wrt_robot, Eigen::Vector4d attitude_sensor_wrt_robot,
                               // State: Map
                               Eigen::Vector3d position_map_element_wrt_world, Eigen::Vector4d attitude_map_element_wrt_world,
                               // Predicted Measurement
                               Eigen::Vector3d& position_map_element_wrt_sensor, Eigen::Vector4d& attitude_map_element_wrt_sensor);


    // Jacobian of the measurements: H
public:
    int jacobiansErrorMeasurements(const TimeStamp theTimeStamp, const std::shared_ptr<GlobalParametersStateCore> TheGlobalParametersStateCore, const std::shared_ptr<RobotStateCore> currentRobotState, const std::shared_ptr<SensorStateCore> currentSensorState, const std::shared_ptr<MapElementStateCore> currentMapElementState, std::shared_ptr<SensorMeasurementCore> matchedMeasurement, std::shared_ptr<CodedVisualMarkerMeasurementCore>& predictedMeasurement);
protected:
    int jacobiansErrorMeasurementsCore(// State: Robot
                                       Eigen::Vector3d position_robot_wrt_world, Eigen::Vector4d attitude_robot_wrt_world,
                                       // State: Sensor
                                       Eigen::Vector3d position_sensor_wrt_robot, Eigen::Vector4d attitude_sensor_wrt_robot,
                                       // State: Map
                                       Eigen::Vector3d position_map_element_wrt_world, Eigen::Vector4d attitude_map_element_wrt_world,
                                       // Measurement
                                       Eigen::Vector3d meas_position_map_element_wrt_sensor, Eigen::Vector4d meas_attitude_map_element_wrt_sensor,
                                       // Predicted Measurement
                                       Eigen::Vector3d position_map_element_wrt_sensor, Eigen::Vector4d attitude_map_element_wrt_sensor,
                                       // Jacobians: State and Params
                                       Eigen::Matrix3d& jacobian_error_meas_pos_wrt_error_state_robot_pos, Eigen::Matrix3d& jacobian_error_meas_pos_wrt_error_state_robot_att, Eigen::Matrix3d& jacobian_error_meas_att_wrt_error_state_robot_att,
                                       Eigen::Matrix3d& jacobian_error_meas_pos_wrt_error_state_sens_pos, Eigen::Matrix3d& jacobian_error_meas_pos_wrt_error_state_sens_att, Eigen::Matrix3d& jacobian_error_meas_att_wrt_error_state_sens_att,
                                       Eigen::Matrix3d& jacobian_error_meas_pos_wrt_error_state_map_elem_pos, Eigen::Matrix3d& jacobian_error_meas_att_wrt_error_state_map_elem_att,
                                       // Jacobians: Noise
                                       Eigen::Matrix3d& jacobian_error_meas_pos_wrt_error_meas_pos, Eigen::Matrix3d& jacobian_error_meas_att_wrt_error_meas_att);


    /// Mapping

    // Mapping: g()
public:
    int mapMeasurement(const TimeStamp theTimeStamp, const std::shared_ptr<GlobalParametersStateCore> TheGlobalParametersStateCore, const std::shared_ptr<RobotStateCore> currentRobotState, const std::shared_ptr<SensorStateCore> currentSensorState, const std::shared_ptr<SensorMeasurementCore> matchedMeasurement, std::shared_ptr<MapElementCore>& newMapElementCore, std::shared_ptr<MapElementStateCore>& newMapElementState);
protected:
    int mapMeasurementCore(// robot wrt world (state)
                           Eigen::Vector3d position_robot_wrt_world, Eigen::Vector4d attitude_robot_wrt_world,
                           // sensor wrt world (state)
                           Eigen::Vector3d position_sensor_wrt_robot, Eigen::Vector4d attitude_sensor_wrt_robot,
                           // Map element wrt sensor (measurement)
                           Eigen::Vector3d meas_position_map_element_wrt_sensor, Eigen::Vector4d meas_attitude_map_element_wrt_sensor,
                           // Map element wrt world (state new)
                           Eigen::Vector3d& position_map_element_wrt_world, Eigen::Vector4d& attitude_map_element_wrt_world);


    // Jacobian Mapping: G
public:
    int jacobiansMapMeasurement(const TimeStamp theTimeStamp, const std::shared_ptr<GlobalParametersStateCore> TheGlobalParametersStateCore, const std::shared_ptr<RobotStateCore> currentRobotState, const std::shared_ptr<SensorStateCore> currentSensorState, const std::shared_ptr<SensorMeasurementCore> matchedMeasurement, std::shared_ptr<MapElementStateCore>& newMapElementState);
protected:
    int jacobiansMapMeasurementCore(// robot wrt world (state)
                                    Eigen::Vector3d position_robot_wrt_world, Eigen::Vector4d attitude_robot_wrt_world,
                                    // sensor wrt world (state)
                                    Eigen::Vector3d position_sensor_wrt_robot, Eigen::Vector4d attitude_sensor_wrt_robot,
                                    // Map element wrt world (state new)
                                    Eigen::Vector3d position_map_element_wrt_world, Eigen::Vector4d attitude_map_element_wrt_world,
                                    // Map element wrt sensor (measurement)
                                    Eigen::Vector3d meas_position_map_element_wrt_sensor, Eigen::Vector4d meas_attitude_map_element_wrt_sensor,
                                    // Jacobians
                                    // State and Params
                                    Eigen::Matrix3d& jacobian_error_map_pos_wrt_error_state_robot_pos, Eigen::Matrix3d& jacobian_error_map_pos_wrt_error_state_robot_att, Eigen::Matrix3d& jacobian_error_map_att_wrt_error_state_robot_att,
                                    Eigen::Matrix3d& jacobian_error_map_pos_wrt_error_state_sens_pos, Eigen::Matrix3d& jacobian_error_map_pos_wrt_error_state_sens_att, Eigen::Matrix3d& jacobian_error_map_att_wrt_error_state_sens_att,
                                    // Measurement
                                    Eigen::Matrix3d& jacobian_error_map_pos_wrt_error_meas_pos, Eigen::Matrix3d& jacobian_error_map_att_wrt_error_meas_att);




};




#endif
