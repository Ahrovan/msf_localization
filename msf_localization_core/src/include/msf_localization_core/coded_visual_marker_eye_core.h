
#ifndef _CODED_VISUAL_MARKER_EYE_CORE_H
#define _CODED_VISUAL_MARKER_EYE_CORE_H



#include <list>

#include "msf_localization_core/block_matrix.h"


// Time Stamp
#include "msf_localization_core/time_stamp.h"


// Sensor core
#include "msf_localization_core/sensor_core.h"

// VM Measurement
#include "msf_localization_core/coded_visual_marker_measurement_core.h"

// VM State
#include "msf_localization_core/coded_visual_marker_eye_state_core.h"

// World
#include "msf_localization_core/global_parameters_state_core.h"

// Robot core
#include "msf_localization_core/robot_core.h"

// Robot state
#include "msf_localization_core/robot_state_core.h"


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
    int readConfig(const pugi::xml_node& sensor, const unsigned int sensorId, std::shared_ptr<CodedVisualMarkerEyeStateCore>& SensorInitStateCore);




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
    int setNoiseMeasurementPosition(const Eigen::Matrix3d& noise_measurement_position);



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
    int setNoiseMeasurementAttitude(const Eigen::Matrix3d& noise_measurement_attitude);




    // Store Measurement
public:
    //int setMeasurement(const TimeStamp the_time_stamp, std::shared_ptr<CodedVisualMarkerMeasurementCore> the_visual_marker_measurement);
    int setMeasurementList(const TimeStamp& the_time_stamp, const std::list< std::shared_ptr<SensorMeasurementCore> >& the_visual_marker_measurement_list);







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
    int predictState(//Time
                     const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp,
                     // Previous State
                     const std::shared_ptr<StateEstimationCore> pastState,
                     // Inputs
                     const std::shared_ptr<InputCommandComponent> inputCommand,
                     // Predicted State
                     std::shared_ptr<StateCore> &predictedState);

protected:
    int predictStateSpecific(const TimeStamp& previousTimeStamp, const TimeStamp& currentTimeStamp,
                             const std::shared_ptr<CodedVisualMarkerEyeStateCore> pastState,
                             std::shared_ptr<CodedVisualMarkerEyeStateCore>& predictedState);
protected:
    int predictStateCore(// State k: Sensor
                         const Eigen::Vector3d& position_sensor_wrt_robot, const Eigen::Vector4d& attitude_sensor_wrt_robot,
                         // State k+1: Sensor
                         Eigen::Vector3d& pred_position_sensor_wrt_robot, Eigen::Vector4d& pred_attitude_sensor_wrt_robot);


    // Jacobian of the error state: F
public:
    int predictErrorStateJacobian(//Time
                                 const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp,
                                 // Previous State
                                 const std::shared_ptr<StateEstimationCore> past_state,
                                 // Inputs
                                 const std::shared_ptr<InputCommandComponent> input_command,
                                 // Predicted State
                                 std::shared_ptr<StateCore>& predicted_state);

protected:
    int predictErrorStateJacobiansSpecific(const TimeStamp& previousTimeStamp, const TimeStamp& currentTimeStamp,
                                           const std::shared_ptr<CodedVisualMarkerEyeStateCore> pastState,
                                           std::shared_ptr<CodedVisualMarkerEyeStateCore>& predictedState,
                                           // Jacobians Error State: Fx, Fp
                                           // Sensor
                                           Eigen::SparseMatrix<double>& jacobian_error_state_wrt_sensor_error_state,
                                           Eigen::SparseMatrix<double>& jacobian_error_state_wrt_sensor_error_parameters
                                           );
protected:
    // TODO Fix!!
    int predictErrorStateJacobiansCore(// State k: Sensor
                                       const Eigen::Vector3d& position_sensor_wrt_robot, const Eigen::Vector4d& attitude_sensor_wrt_robot,
                                       // State k+1: Sensor
                                       const Eigen::Vector3d& pred_position_sensor_wrt_robot, const Eigen::Vector4d& pred_attitude_sensor_wrt_robot,
                                       // Jacobian: State
                                       Eigen::Matrix3d& jacobian_error_sens_pos_wrt_error_state_sens_pos,  Eigen::Matrix3d& jacobian_error_sens_att_wrt_error_state_sens_att);




    ///// Update functions


    /// State Correction

    // Prediction measurements: h()
public:
    int predictMeasurement(// Time
                           const TimeStamp current_time_stamp,
                           // Current State
                           const std::shared_ptr<StateEstimationCore> current_state,
                           // Measurement to match
                           const std::shared_ptr<SensorMeasurementCore> measurement,
                           // Predicted Measurements
                           std::shared_ptr<SensorMeasurementCore> &predicted_measurement);

protected:
    int predictMeasurementSpecific(const TimeStamp& theTimeStamp,
                                   const std::shared_ptr<GlobalParametersStateCore> TheGlobalParametersStateCore,
                                   const std::shared_ptr<RobotStateCore> currentRobotState,
                                   const std::shared_ptr<CodedVisualMarkerEyeStateCore> currentSensorState,
                                   const std::shared_ptr<CodedVisualMarkerLandmarkStateCore> currentMapElementState,
                                   std::shared_ptr<CodedVisualMarkerMeasurementCore>& predictedMeasurement);
protected:
    int predictMeasurementCore(// State: Robot
                               const Eigen::Vector3d& position_robot_wrt_world, const Eigen::Vector4d& attitude_robot_wrt_world,
                               // State: Sensor
                               const Eigen::Vector3d& position_sensor_wrt_robot, const Eigen::Vector4d& attitude_sensor_wrt_robot,
                               // State: Map
                               const Eigen::Vector3d& position_map_element_wrt_world, const Eigen::Vector4d& attitude_map_element_wrt_world,
                               // Predicted Measurement
                               Eigen::Vector3d& position_map_element_wrt_sensor, Eigen::Vector4d& attitude_map_element_wrt_sensor);


    // Jacobian of the measurements: H
public:
    int predictErrorMeasurementJacobian(// Time
                                        const TimeStamp current_time_stamp,
                                        // Current State
                                        const std::shared_ptr<StateEstimationCore> current_state,
                                        // Measurement to match
                                        const std::shared_ptr<SensorMeasurementCore> measurement,
                                        // Predicted Measurements
                                        std::shared_ptr<SensorMeasurementCore> &predicted_measurement);

protected:
    int predictErrorMeasurementJacobianSpecific(const TimeStamp& theTimeStamp,
                                               const std::shared_ptr<RobotStateCore> currentRobotState,
                                               const std::shared_ptr<CodedVisualMarkerEyeStateCore> currentSensorState,
                                               const std::shared_ptr<CodedVisualMarkerLandmarkStateCore> currentMapElementState,
                                               const std::shared_ptr<CodedVisualMarkerMeasurementCore> matchedMeasurement,
                                               std::shared_ptr<CodedVisualMarkerMeasurementCore>& predictedMeasurement,
                                                // Jacobians Error State: Hx, Hp
                                                // Robot
                                                Eigen::SparseMatrix<double>& jacobian_error_measurement_wrt_robot_error_state,
                                                Eigen::SparseMatrix<double>& jacobian_error_measurement_wrt_robot_error_parameters,
                                                // Sensor
                                                Eigen::SparseMatrix<double>& jacobian_error_measurement_wrt_sensor_error_state,
                                                Eigen::SparseMatrix<double>& jacobian_error_measurement_wrt_sensor_error_parameters,
                                                // Map Element
                                                Eigen::SparseMatrix<double>& jacobian_error_measurement_wrt_map_element_error_state,
                                                Eigen::SparseMatrix<double>& jacobian_error_measurement_wrt_map_element_error_parameters,
                                                // Jacobians Noise: Hn
                                                Eigen::SparseMatrix<double>& jacobian_error_measurement_wrt_error_measurement
                                                );
protected:
    int jacobiansErrorMeasurementsCore(// State: Robot
                                       const Eigen::Vector3d& position_robot_wrt_world, const Eigen::Vector4d& attitude_robot_wrt_world,
                                       // State: Sensor
                                       const Eigen::Vector3d& position_sensor_wrt_robot, const Eigen::Vector4d& attitude_sensor_wrt_robot,
                                       // State: Map
                                       const Eigen::Vector3d& position_map_element_wrt_world, const Eigen::Vector4d& attitude_map_element_wrt_world,
                                       // Measurement
                                       const Eigen::Vector3d& meas_position_map_element_wrt_sensor, const Eigen::Vector4d& meas_attitude_map_element_wrt_sensor,
                                       // Predicted Measurement
                                       const Eigen::Vector3d& position_map_element_wrt_sensor, const Eigen::Vector4d& attitude_map_element_wrt_sensor,
                                       // Jacobians: State and Params
                                       Eigen::Matrix3d& jacobian_error_meas_pos_wrt_error_state_robot_pos, Eigen::Matrix3d& jacobian_error_meas_pos_wrt_error_state_robot_att, Eigen::Matrix3d& jacobian_error_meas_att_wrt_error_state_robot_att,
                                       Eigen::Matrix3d& jacobian_error_meas_pos_wrt_error_state_sens_pos, Eigen::Matrix3d& jacobian_error_meas_pos_wrt_error_state_sens_att, Eigen::Matrix3d& jacobian_error_meas_att_wrt_error_state_sens_att,
                                       Eigen::Matrix3d& jacobian_error_meas_pos_wrt_error_state_map_elem_pos, Eigen::Matrix3d& jacobian_error_meas_att_wrt_error_state_map_elem_att,
                                       // Jacobians: Noise
                                       Eigen::Matrix3d& jacobian_error_meas_pos_wrt_error_meas_pos, Eigen::Matrix3d& jacobian_error_meas_att_wrt_error_meas_att);


    /// Reset Error State

public:
    int resetErrorStateJacobian(// Time
                                const TimeStamp& current_time_stamp,
                                // Increment Error State
                                const Eigen::VectorXd& increment_error_state,
                                // Current State
                                std::shared_ptr<StateCore>& current_state
                                );



    /// Mapping

    // Mapping: g()
public:
    int mapMeasurement(// Time
                       const TimeStamp current_time_stamp,
                       // Current State
                       const std::shared_ptr<StateEstimationCore> current_state,
                       // Current Measurement
                       const std::shared_ptr<SensorMeasurementCore> current_measurement,
                       // List Map Element Core -> New will be added if not available
                       std::list< std::shared_ptr<MapElementCore> >& list_map_element_core,
                       // New Map Element State Core
                       std::shared_ptr<StateCore> &new_map_element_state);

protected:
    int mapMeasurementSpecific(const TimeStamp& theTimeStamp,
                               const std::shared_ptr<RobotStateCore> currentRobotState,
                               const std::shared_ptr<CodedVisualMarkerEyeStateCore>&currentSensorState,
                               const std::shared_ptr<CodedVisualMarkerMeasurementCore> matchedMeasurement,
                               std::shared_ptr<CodedVisualMarkerLandmarkCore>& newMapElementCore,
                               std::shared_ptr<CodedVisualMarkerLandmarkStateCore>& newMapElementState);
protected:
    int mapMeasurementCore(// robot wrt world (state)
                           const Eigen::Vector3d& position_robot_wrt_world, const Eigen::Vector4d& attitude_robot_wrt_world,
                           // sensor wrt world (state)
                           const Eigen::Vector3d& position_sensor_wrt_robot, const Eigen::Vector4d& attitude_sensor_wrt_robot,
                           // Map element wrt sensor (measurement)
                           const Eigen::Vector3d& meas_position_map_element_wrt_sensor, const Eigen::Vector4d& meas_attitude_map_element_wrt_sensor,
                           // Map element wrt world (state new)
                           Eigen::Vector3d& position_map_element_wrt_world, Eigen::Vector4d& attitude_map_element_wrt_world);


    // Jacobian Mapping: G
public:
    int jacobiansMapMeasurement(// Time
                       const TimeStamp current_time_stamp,
                       // Current State
                       const std::shared_ptr<StateEstimationCore> current_state,
                       // Current Measurement
                       const std::shared_ptr<SensorMeasurementCore> current_measurement,
                       // New Map Element State Core
                       std::shared_ptr<StateCore> &new_map_element_state);

protected:
    int jacobiansMapMeasurementSpecific(const TimeStamp& theTimeStamp,
                                        const std::shared_ptr<RobotStateCore> currentRobotState,
                                        const std::shared_ptr<CodedVisualMarkerEyeStateCore> currentSensorState,
                                        const std::shared_ptr<CodedVisualMarkerMeasurementCore> matchedMeasurement,
                                        std::shared_ptr<CodedVisualMarkerLandmarkStateCore>& newMapElementState);
protected:
    int jacobiansMapMeasurementCore(// robot wrt world (state)
                                    const Eigen::Vector3d& position_robot_wrt_world, const Eigen::Vector4d& attitude_robot_wrt_world,
                                    // sensor wrt world (state)
                                    const Eigen::Vector3d& position_sensor_wrt_robot, const Eigen::Vector4d& attitude_sensor_wrt_robot,
                                    // Map element wrt world (state new)
                                    const Eigen::Vector3d& position_map_element_wrt_world, const Eigen::Vector4d& attitude_map_element_wrt_world,
                                    // Map element wrt sensor (measurement)
                                    const Eigen::Vector3d& meas_position_map_element_wrt_sensor, const Eigen::Vector4d& meas_attitude_map_element_wrt_sensor,
                                    // Jacobians
                                    // State and Params
                                    Eigen::Matrix3d& jacobian_error_map_pos_wrt_error_state_robot_pos, Eigen::Matrix3d& jacobian_error_map_pos_wrt_error_state_robot_att, Eigen::Matrix3d& jacobian_error_map_att_wrt_error_state_robot_att,
                                    Eigen::Matrix3d& jacobian_error_map_pos_wrt_error_state_sens_pos, Eigen::Matrix3d& jacobian_error_map_pos_wrt_error_state_sens_att, Eigen::Matrix3d& jacobian_error_map_att_wrt_error_state_sens_att,
                                    // Measurement
                                    Eigen::Matrix3d& jacobian_error_map_pos_wrt_error_meas_pos, Eigen::Matrix3d& jacobian_error_map_att_wrt_error_meas_att);



    /// Auxiliar functions
protected:
    int findSensorState(const std::list< std::shared_ptr<StateCore> >& list_sensors_state,
                        std::shared_ptr<CodedVisualMarkerEyeStateCore>& sensor_state);

    int findMapElementCore(const std::list<std::shared_ptr<MapElementCore>>& list_map_elements_core,
                           const std::shared_ptr<CodedVisualMarkerMeasurementCore> sensor_measurement,
                           std::shared_ptr<CodedVisualMarkerLandmarkCore>& map_element_core);

    int findMapElementState(const std::list<std::shared_ptr<StateCore>>& list_map_elements_state,
                            const std::shared_ptr<CodedVisualMarkerMeasurementCore> sensor_measurement,
                            std::shared_ptr<CodedVisualMarkerLandmarkStateCore>& map_element_state);

};




#endif
