
#ifndef _MOCAP_SENSOR_CORE_H
#define _MOCAP_SENSOR_CORE_H



#include <list>

#include "pugixml/pugixml.hpp"


// Time Stamp
#include "msf_localization_core/time_stamp.h"


// Sensor core
#include "msf_localization_core/sensor_core.h"

// Sensor Measurement
#include "msf_localization_core/mocap_sensor_measurement_core.h"
// Sensor State
#include "msf_localization_core/mocap_sensor_state_core.h"


// Robot state
#include "msf_localization_core/robot_state_core.h"
#include "msf_localization_core/free_model_robot_state_core.h"
#include "msf_localization_core/imu_driven_robot_state_core.h"


// Map
#include "msf_localization_core/mocap_world_core.h"
#include "msf_localization_core/mocap_world_state_core.h"



class MocapSensorCore : public SensorCore
{
public:
    MocapSensorCore();
    MocapSensorCore(std::weak_ptr<MsfStorageCore> the_msf_storage_core);
public:
    ~MocapSensorCore();

public:
    int init();


public:
    int readConfig(pugi::xml_node sensor, unsigned int sensorId, std::shared_ptr<MocapSensorStateCore>& SensorInitStateCore);




    ///// Measurements

    // z=[{z_posi_vmi__wrt_aruco_eye, z_attit_vmi_wrt_aruco_eye}]'



    // Position Measurement
protected:
    bool flag_measurement_position_mocap_sensor_wrt_mocap_world_;
public:
    bool isMeasurementPositionMocapSensorWrtMocapWorldEnabled() const;
    int enableMeasurementPositionMocapSensorWrtMocapWorld();

    // position measurement covariance
protected:
    Eigen::Matrix3d noise_measurement_position_mocap_sensor_wrt_mocap_world_;
public:
    Eigen::Matrix3d getNoiseMeasurementPositionMocapSensorWrtMocapWorld() const;
    int setNoiseMeasurementPositionMocapSensorWrtMocapWorld(Eigen::Matrix3d noise_measurement_position_mocap_sensor_wrt_mocap_world);



    // Attitude Measurement
protected:
    bool flag_measurement_attitude_mocap_sensor_wrt_mocap_world_;
public:
    bool isMeasurementAttitudeMocapSensorWrtMocapWorldEnabled() const;
    int enableMeasurementAttitudeMocapSensorWrtMocapWorld();

    // attitude measurement covariance
protected:
    Eigen::Matrix3d noise_measurement_attitude_mocap_sensor_wrt_mocap_world_;
public:
    Eigen::Matrix3d getNoiseMeasurementAttitudeMocapSensorWrtMocapWorld() const;
    int setNoiseMeasurementAttitudeMocapSensorWrtMocapWorld(Eigen::Matrix3d noise_measurement_attitude_mocap_sensor_wrt_mocap_world);




    // Store Measurement
public:
    int setMeasurement(const TimeStamp the_time_stamp, std::shared_ptr<MocapSensorMeasurementCore> sensor_measurement);
    //int setMeasurementList(const TimeStamp the_time_stamp, std::list< std::shared_ptr<SensorMeasurementCore> > sensor_measurement_list);







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
    int predictStateSpecific(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp,
                             const std::shared_ptr<MocapSensorStateCore> pastState,
                             std::shared_ptr<MocapSensorStateCore>& predictedState);

protected:
    int predictStateCore(// State k: Sensor
                         Eigen::Vector3d position_sensor_wrt_robot, Eigen::Vector4d attitude_sensor_wrt_robot,
                         // State k+1: Sensor
                         Eigen::Vector3d& pred_position_sensor_wrt_robot, Eigen::Vector4d& pred_attitude_sensor_wrt_robot);


    // Jacobian of the error state: F
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
                                   const std::shared_ptr<MocapSensorStateCore> pastState,
                                   std::shared_ptr<MocapSensorStateCore>& predictedState);
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
    int predictMeasurement(// Time
                           const TimeStamp current_time_stamp,
                           // Current State
                           const std::shared_ptr<StateEstimationCore> current_state,
                           // Measurement to match
                           const std::shared_ptr<SensorMeasurementCore> measurement,
                           // Predicted Measurements
                           std::shared_ptr<SensorMeasurementCore> &predicted_measurement);

protected:
    int predictMeasurementSpecific(const TimeStamp theTimeStamp,
                           const std::shared_ptr<RobotStateCore> currentRobotState,
                           const std::shared_ptr<MocapSensorStateCore> currentSensorState,
                           const std::shared_ptr<MocapWorldStateCore> currentMapElementState,
                           std::shared_ptr<MocapSensorMeasurementCore>& predictedMeasurement);

protected:
    int predictMeasurementCore(// State: Robot
                               Eigen::Vector3d position_robot_wrt_world, Eigen::Vector4d attitude_robot_wrt_world,
                               // State: Sensor
                               Eigen::Vector3d position_sensor_wrt_robot, Eigen::Vector4d attitude_sensor_wrt_robot,
                               // State: Map
                               Eigen::Vector3d position_map_element_wrt_world, Eigen::Vector4d attitude_map_element_wrt_world,
                               // Predicted Measurement
                               Eigen::Vector3d& position_sensor_wrt_map_element, Eigen::Vector4d& attitude_sensor_wrt_map_element);


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
    int predictErrorMeasurementJacobianSpecific(const TimeStamp theTimeStamp,
                                                const std::shared_ptr<RobotStateCore> currentRobotState,
                                                const std::shared_ptr<MocapSensorStateCore> currentSensorState,
                                                const std::shared_ptr<MocapWorldStateCore> currentMapElementState,
                                                const std::shared_ptr<MocapSensorMeasurementCore> matchedMeasurement,
                                                std::shared_ptr<MocapSensorMeasurementCore>& predictedMeasurement,
                                                // Jacobians State
                                                // Robot
                                                Eigen::SparseMatrix<double>& jacobian_error_measurement_wrt_robot_error_state,
                                                Eigen::SparseMatrix<double>& jacobian_error_measurement_wrt_robot_error_parameters,
                                                // Sensor
                                                Eigen::SparseMatrix<double>& jacobian_error_measurement_wrt_sensor_error_state,
                                                Eigen::SparseMatrix<double>& jacobian_error_measurement_wrt_sensor_error_parameters,
                                                // Map Element
                                                Eigen::SparseMatrix<double>& jacobian_error_measurement_wrt_map_element_error_state,
                                                Eigen::SparseMatrix<double>& jacobian_error_measurement_wrt_map_element_error_parameters,
                                                // Jacobians Noise
                                                Eigen::SparseMatrix<double>& jacobian_error_measurement_wrt_error_measurement
                                                );

protected:
    int jacobiansErrorMeasurementsCore(// State: Robot
                                       Eigen::Vector3d position_robot_wrt_world, Eigen::Vector4d attitude_robot_wrt_world,
                                       // State: Sensor
                                       Eigen::Vector3d position_sensor_wrt_robot, Eigen::Vector4d attitude_sensor_wrt_robot,
                                       // State: Map
                                       Eigen::Vector3d position_map_element_wrt_world, Eigen::Vector4d attitude_map_element_wrt_world,
                                       // Measurement
                                       Eigen::Vector3d meas_position_sensor_wrt_map_element, Eigen::Vector4d meas_attitude_sensor_wrt_map_element,
                                       // Predicted Measurement
                                       Eigen::Vector3d position_msensor_wrt_map_element, Eigen::Vector4d attitude_sensor_wrt_map_element,
                                       // Jacobians: State and Params
                                       Eigen::Matrix3d& jacobian_error_meas_pos_wrt_error_state_robot_pos, Eigen::Matrix3d& jacobian_error_meas_pos_wrt_error_state_robot_att, Eigen::Matrix3d& jacobian_error_meas_att_wrt_error_state_robot_att,
                                       Eigen::Matrix3d& jacobian_error_meas_pos_wrt_error_state_sens_pos, Eigen::Matrix3d& jacobian_error_meas_att_wrt_error_state_sens_att,
                                       Eigen::Matrix3d& jacobian_error_meas_pos_wrt_error_state_map_elem_pos, Eigen::Matrix3d& jacobian_error_meas_pos_wrt_error_state_map_elem_att, Eigen::Matrix3d& jacobian_error_meas_att_wrt_error_state_map_elem_att,
                                       // Jacobians: Noise
                                       Eigen::Matrix3d& jacobian_error_meas_pos_wrt_error_meas_pos, Eigen::Matrix3d& jacobian_error_meas_att_wrt_error_meas_att);


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
                       std::shared_ptr<MapElementStateCore> &new_map_element_state);

protected:
    int mapMeasurementSpecific(const TimeStamp theTimeStamp,
                               const std::shared_ptr<RobotStateCore> currentRobotState,
                               const std::shared_ptr<MocapSensorStateCore> currentSensorState,
                               const std::shared_ptr<MocapSensorMeasurementCore> matchedMeasurement,
                               std::shared_ptr<MocapWorldCore>& newMapElementCore,
                               std::shared_ptr<MocapWorldStateCore>& newMapElementState);

protected:
    int mapMeasurementCore(// robot wrt world (state)
                           Eigen::Vector3d position_robot_wrt_world, Eigen::Vector4d attitude_robot_wrt_world,
                           // sensor wrt world (state)
                           Eigen::Vector3d position_sensor_wrt_robot, Eigen::Vector4d attitude_sensor_wrt_robot,
                           // Sensor Wrt Map element (measurement)
                           Eigen::Vector3d meas_position_sensor_wrt_map_element, Eigen::Vector4d meas_attitude_sensor_wrt_map_element,
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
                       std::shared_ptr<MapElementStateCore> &new_map_element_state);

protected:
    int jacobiansMapMeasurementSpecific(const TimeStamp theTimeStamp,
                                        const std::shared_ptr<RobotStateCore> currentRobotState,
                                        const std::shared_ptr<MocapSensorStateCore> currentSensorState,
                                        const std::shared_ptr<MocapSensorMeasurementCore> matchedMeasurement,
                                        std::shared_ptr<MocapWorldStateCore>& newMapElementState);

protected:
    int jacobiansMapMeasurementCore(// robot wrt world (state)
                                    Eigen::Vector3d position_robot_wrt_world, Eigen::Vector4d attitude_robot_wrt_world,
                                    // sensor wrt world (state)
                                    Eigen::Vector3d position_sensor_wrt_robot, Eigen::Vector4d attitude_sensor_wrt_robot,
                                    // Map element wrt world (state new)
                                    Eigen::Vector3d position_map_element_wrt_world, Eigen::Vector4d attitude_map_element_wrt_world,
                                    // Sensor wrt map element (measurement)
                                    Eigen::Vector3d meas_position_sensor_wrt_map_element, Eigen::Vector4d meas_attitude_sensor_wrt_map_element,
                                    // Jacobians
                                    // State and Params
                                    Eigen::Matrix3d& jacobian_error_map_pos_wrt_error_state_robot_pos, Eigen::Matrix3d& jacobian_error_map_pos_wrt_error_state_robot_att, Eigen::Matrix3d& jacobian_error_map_att_wrt_error_state_robot_att,
                                    Eigen::Matrix3d& jacobian_error_map_pos_wrt_error_state_sens_pos, Eigen::Matrix3d& jacobian_error_map_pos_wrt_error_state_sens_att, Eigen::Matrix3d& jacobian_error_map_att_wrt_error_state_sens_att,
                                    // Measurement
                                    Eigen::Matrix3d& jacobian_error_map_pos_wrt_error_meas_pos, Eigen::Matrix3d& jacobian_error_map_att_wrt_error_meas_att);



    /// Auxiliar functions
protected:
    int findSensorState(const std::list< std::shared_ptr<SensorStateCore> > list_sensors_state,
                        std::shared_ptr<MocapSensorStateCore>& sensor_state);

    int findMapElementCore(const std::list<std::shared_ptr<MapElementCore>> list_map_elements_core,
                           const std::shared_ptr<MocapSensorMeasurementCore> sensor_measurement,
                           std::shared_ptr<MocapWorldCore>& map_element_core);

    int findMapElementState(const std::list<std::shared_ptr<MapElementStateCore>> list_map_elements_state,
                            const std::shared_ptr<MocapSensorMeasurementCore> sensor_measurement,
                            std::shared_ptr<MocapWorldStateCore>& map_element_state);


};


#endif
