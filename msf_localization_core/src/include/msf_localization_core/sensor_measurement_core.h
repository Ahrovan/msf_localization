

#ifndef _SENSOR_MEASUREMENT_CORE_H
#define _SENSOR_MEASUREMENT_CORE_H


#include <memory>


#include <Eigen/Dense>


#include "msf_localization_core/sensor_core.h"


enum class MeasurementTypes
{
    undefined=0,
    imu=1,
    coded_visual_marker=2
};


class SensorMeasurementCore
{
public:
    SensorMeasurementCore();
    SensorMeasurementCore(std::weak_ptr<SensorCore> sensor_core_ptr);
    ~SensorMeasurementCore();

protected:
    int init();

protected:
    // It is not the owner of this Pointer. it doesn't modify the pointer
    std::weak_ptr<SensorCore> sensor_core_ptr_;
public:
    int setSensorCorePtr(std::weak_ptr<SensorCore> sensor_core_ptr);
    std::weak_ptr<SensorCore> getSensorCoreWeakPtr() const;
    std::shared_ptr<SensorCore> getSensorCoreSharedPtr() const;


protected:
    MeasurementTypes measurementType;
public:
    int setMeasurementType(MeasurementTypes measurementType);
    MeasurementTypes getMeasurementType() const;


public:
    virtual bool isCorrect();


    /// Jacobians Measurement

public:
    // Jacobian Error Measurement - Error State
    struct
    {
        Eigen::MatrixXd jacobianMeasurementRobotErrorState;
        Eigen::MatrixXd jacobianMeasurementGlobalParametersErrorState;
        Eigen::MatrixXd jacobianMeasurementSensorErrorState;
        Eigen::MatrixXd jacobianMeasurementMapElementErrorState; // TODO
    } jacobianMeasurementErrorState;


    // Jacobian Error Measurement - Error Parameters
    struct
    {
        // TODO Robot Parameters
        Eigen::MatrixXd jacobianMeasurementGlobalParameters;
        Eigen::MatrixXd jacobianMeasurementSensorParameters;
        Eigen::MatrixXd jacobianMeasurementMapElementParameters;
    } jacobianMeasurementErrorParameters;

    /*
    struct
    {
        Eigen::MatrixXd jacobianMeasurementSensorParameters;

    } jacobianMeasurementSensorParameters;

    struct
    {
        Eigen::MatrixXd jacobianMeasurementGlobalParameters;

    } jacobianMeasurementGlobalParameters;

    struct
    {
        Eigen::MatrixXd jacobianMeasurementMapElementParameters;
    } jacobianMeasurementMapElementParameters;
    */

    // Jacobian Error Measurement - Sensor Noise
    struct
    {
        Eigen::MatrixXd jacobianMeasurementSensorNoise;

    } jacobianMeasurementSensorNoise;




    //// Get the innovation vector as an Eigen::VectorXd
public:
    virtual Eigen::VectorXd getInnovation(std::shared_ptr<SensorMeasurementCore> theMatchedMeasurement, std::shared_ptr<SensorMeasurementCore> thePredictedMeasurement)=0;


    //// Get the full measurement as an Eigen::VectorXd
public:
    virtual Eigen::VectorXd getMeasurement()=0;




//    //// Debug log
//protected:
//    std::string logPath;
//    std::ofstream logFile;
//    // mutex to protect the log file
//protected:
//    std::mutex TheLogFileMutex;
//public:
//    int log(std::string logString);

};






#endif
