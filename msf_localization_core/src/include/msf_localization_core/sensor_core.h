
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
#include "msf_localization_core/time_stamp.h"


#include "msf_localization_core/msf_element_core.h"


#define _DEBUG_SENSOR_CORE 0


class SensorStateCore;


class SensorCore : public SensorBasics, public MsfElementCore
{
public:
    SensorCore();
    SensorCore(std::weak_ptr<MsfStorageCore> msf_storage_core_ptr);
    ~SensorCore();


protected:
    int init();


/*
    // Pointer to itself
protected:
    std::weak_ptr<SensorCore> TheSensorCorePtr;
public:
    int setTheSensorCore(std::weak_ptr<SensorCore> TheSensorCorePtr);
    std::shared_ptr<SensorCore> getTheSensorCore() const;
    std::shared_ptr<SensorCore> getTheSensorCoreShared() const;
    std::weak_ptr<SensorCore> getTheSensorCoreWeak() const;


    // Pointer to the MSF Storage Core
protected:
    std::weak_ptr<MsfStorageCore> TheMsfStorageCore;
public:
    int setTheMsfStorageCore(std::weak_ptr<MsfStorageCore> TheMsfStorageCore);
    std::shared_ptr<MsfStorageCore> getTheMsfStorageCore() const;
*/


    // Dimension state
protected:
    unsigned int dimensionState;
public:
    unsigned int getDimensionState() const;
    //int setDimensionState(unsigned int dimensionState);


    // Dimension error state
protected:
    unsigned int dimensionErrorState;
public:
    unsigned int getDimensionErrorState() const;
    //int setDimensionErrorState(unsigned int dimensionErrorState);


    // Dimension parameters
protected:
    unsigned int dimensionParameters;
public:
    unsigned int getDimensionParameters() const;

    // Dimension error parameters
protected:
    unsigned int dimensionErrorParameters;
public:
    unsigned int getDimensionErrorParameters() const;



    // Dimension of the measurement
protected:
    unsigned int dimensionMeasurement;
public:
    unsigned int getDimensionMeasurement() const;


    // Dimension of the error measurement
protected:
    unsigned int dimensionErrorMeasurement;
public:
    unsigned int getDimensionErrorMeasurement() const;


    // Dimension of the sensor noise
protected:
    unsigned int dimensionNoise;
public:
    unsigned int getDimensionNoise() const;



    //// Name
protected:
    std::string sensor_name_;
public:
    int setSensorName(std::string sensor_name);
    std::string getSensorName() const;



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
    int setNoiseAttitudeSensorWrtRobot(Eigen::Matrix3d noiseAttitudeSensorWrtRobot);



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
    int setNoisePositionSensorWrtRobot(Eigen::Matrix3d noisePositionSensorWrtRobot);




    ////// Init error state variances -> Temporal, only for the initial configuration
protected:
    Eigen::MatrixXd InitErrorStateVariance;
public:
    Eigen::MatrixXd getInitErrorStateVariance() const;

public:
    virtual int prepareInitErrorStateVariance();



    ///// Get Covariances as a Eigen::MatrixXd
public:
    virtual Eigen::SparseMatrix<double> getCovarianceMeasurement()=0;
    virtual Eigen::SparseMatrix<double> getCovarianceParameters()=0;



public:
    virtual Eigen::SparseMatrix<double> getCovarianceNoise(const TimeStamp deltaTimeStamp) const =0;



    // Prediction state function
public:
    virtual int predictState(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, const std::shared_ptr<SensorStateCore> pastState, std::shared_ptr<SensorStateCore>& predictedState) =0;

    // Jacobian of the error state
public:
    virtual int predictStateErrorStateJacobians(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, std::shared_ptr<SensorStateCore> pastState, std::shared_ptr<SensorStateCore>& predictedState) =0;



};







#endif
