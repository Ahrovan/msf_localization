

#ifndef _SENSOR_MEASUREMENT_CORE_H
#define _SENSOR_MEASUREMENT_CORE_H


#include <memory>


#include <Eigen/Dense>


#include "msf_localization_core/sensor_core.h"



class SensorMeasurementCore
{
public:
    SensorMeasurementCore();
    SensorMeasurementCore(std::weak_ptr<SensorCore> TheSensorCorePtr);
    ~SensorMeasurementCore();


protected:
    // It is not the owner of this Pointer. it doesn't modify the pointer
    std::weak_ptr<SensorCore> TheSensorCorePtr;
public:
    int setTheSensorCore(std::weak_ptr<SensorCore> TheSensorCorePtr);
    std::shared_ptr<SensorCore> getTheSensorCore() const;




    /// Jacobians Measurement

public:
    struct
    {
        Eigen::MatrixXd jacobianMeasurementRobotErrorState;
        Eigen::MatrixXd jacobianMeasurementGlobalParametersErrorState;
        Eigen::MatrixXd jacobianMeasurementSensorErrorState;

    } jacobianMeasurementErrorState;

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
        Eigen::MatrixXd jacobianMeasurementSensorNoise;

    } jacobianMeasurementSensorNoise;




    //// Get the innovation vector as an Eigen::VectorXd
public:
    virtual Eigen::VectorXd getInnovation(std::shared_ptr<SensorMeasurementCore> theMatchedMeasurement, std::shared_ptr<SensorMeasurementCore> thePredictedMeasurement)=0;


    //// Get the full measurement as an Eigen::VectorXd
public:
    virtual Eigen::VectorXd getMeasurement()=0;

};






#endif
