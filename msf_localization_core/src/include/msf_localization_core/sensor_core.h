
#ifndef _SENSOR_CORE_H
#define _SENSOR_CORE_H


//I/O stream
//std::cout
#include <iostream>

#include <memory>


//File Stream
//std::ofstream, std::ifstream
#include <fstream>


#include <string>

#include <mutex>


#include <Eigen/Dense>
#include <Eigen/Sparse>


#include "msf_localization_core/sensor_basics.h"


// For TimeStamp
#include "time_stamp/time_stamp.h"


#include "msf_localization_core/msf_element_core.h"



#define _DEBUG_SENSOR_CORE 0


//class SensorStateCore;
class MapElementCore;


class SensorCore : public SensorBasics, public MsfElementCore
{
public:
    SensorCore();
    SensorCore(MsfLocalizationCore* msf_localization_core_ptr);
    ~SensorCore();


protected:
    int init();


    // Sensor core
public:
    std::weak_ptr<SensorCore> getSensorCoreWeakPtr() const;
    std::shared_ptr<SensorCore> getSensorCoreSharedPtr() const;




    // Dimension of the measurement
protected:
    int dimension_measurement_;
public:
    int getDimensionMeasurement() const;


    // Dimension of the error measurement
protected:
    int dimension_error_measurement_;
public:
    int getDimensionErrorMeasurement() const;



    //// Measurement
public:
    int setMeasurement(const TimeStamp& time_stamp, const std::shared_ptr<SensorMeasurementCore> sensor_measurement);
    int setMeasurementList(const TimeStamp& time_stamp, const std::list< std::shared_ptr<SensorMeasurementCore> >& sensor_measurement_list);


    //// Pose of the sensor wrt robot


    /// Attitude sensor wrt robot

    // Attitude sensor wrt robot: q_sensor_wrt_robot (4x1) [Theta_sensor_wrt_robot (3x1)]
protected:
    bool flagEstimationAttitudeSensorWrtRobot;
public:
    bool isEstimationAttitudeSensorWrtRobotEnabled() const;
    int enableEstimationAttitudeSensorWrtRobot();
    int enableParameterAttitudeSensorWrtRobot();

    // Attitude sensor wrt robot: covariance (if enabled estimation -> P; if no enabled estimation -> Sigma_mu)
protected:
    Eigen::Matrix3d noiseAttitudeSensorWrtRobot;
public:
    Eigen::Matrix3d getNoiseAttitudeSensorWrtRobot() const;
    int setNoiseAttitudeSensorWrtRobot(const Eigen::Matrix3d& noiseAttitudeSensorWrtRobot);



    /// Position sensor wrt robot

    // Position sensor wrt robot: t_sensor_wrt_robot (3x1)
protected:
    bool flagEstimationPositionSensorWrtRobot;
public:
    bool isEstimationPositionSensorWrtRobotEnabled() const;
    int enableEstimationPositionSensorWrtRobot();
    int enableParameterPositionSensorWrtRobot();

    // Position sensor wrt robot: covariance (if enabled estimation -> P; if no enabled estimation -> Sigma_mu)
protected:
    Eigen::Matrix3d noisePositionSensorWrtRobot;
public:
    Eigen::Matrix3d getNoisePositionSensorWrtRobot() const;
    int setNoisePositionSensorWrtRobot(const Eigen::Matrix3d& noisePositionSensorWrtRobot);





    ///// Covariances Getters

    // Covariance Sensor Error Measurements: Rn
public:
    virtual Eigen::SparseMatrix<double> getCovarianceMeasurement();






    ///// Predict Step Functions

    // Nothing new



    ///// Update Step Functions


    /// Correct State

    // TODO

public:
    virtual int predictMeasurement(// Time
                                   const TimeStamp& current_time_stamp,
                                   // Current State
                                   const std::shared_ptr<StateComponent>& current_state,
                                   // Measurements
                                   const std::shared_ptr<SensorMeasurementCore>& measurement,
                                   // Predicted Measurements
                                   std::shared_ptr<SensorMeasurementCore> &predicted_measurement)
    {
        std::cout<<"SensorCore::predictMeasurement()"<<std::endl;
        return 1;
    }


public:
    virtual int predictErrorMeasurementJacobian(// Time
                                                const TimeStamp& current_time_stamp,
                                               // Current State
                                               const std::shared_ptr<StateComponent>& current_state,
                                               // Measurements
                                               const std::shared_ptr<SensorMeasurementCore>& measurement,
                                               // Predicted Measurements
                                               std::shared_ptr<SensorMeasurementCore> &predicted_measurement)
    {
        std::cout<<"SensorCore::predictErrorMeasurementJacobian()"<<std::endl;
        return 1;
    }

protected:
    int predictErrorMeasurementJacobianInit(// Current State
                                            const std::shared_ptr<StateComponent>& current_state,
                                            // Sensor Measurements
                                            const std::shared_ptr<SensorMeasurementCore> &sensor_measurement,
                                            // Predicted Measurements
                                            std::shared_ptr<SensorMeasurementCore> &predicted_measurement);



    /// Mapping

    // TODO
public:
    virtual int mapMeasurement(// Time
                       const TimeStamp& current_time_stamp,
                       // Current State
                       const std::shared_ptr<StateComponent>& current_state,
                       // Current Measurement
                       const std::shared_ptr<SensorMeasurementCore>& current_measurement,
                       // List Map Element Core -> New will be added if not available
                       std::list< std::shared_ptr<MapElementCore> >& list_map_element_core,
                       // New Map Element State Core
                       std::shared_ptr<StateCore> &new_map_element_state)
    {
        std::cout<<"SensorCore::mapMeasurement()"<<std::endl;
        return 1;
    }


public:
    virtual int jacobiansMapMeasurement(// Time
                       const TimeStamp& current_time_stamp,
                       // Current State
                       const std::shared_ptr<StateComponent>& current_state,
                       // Current Measurement
                       const std::shared_ptr<SensorMeasurementCore>& current_measurement,
                       // New Map Element State Core
                       std::shared_ptr<StateCore> &new_map_element_state)
    {
        std::cout<<"SensorCore::jacobiansMapMeasurement()"<<std::endl;
        return 1;
    }




};







#endif
