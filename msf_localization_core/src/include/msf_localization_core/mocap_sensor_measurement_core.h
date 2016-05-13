
#ifndef _MOCAP_SENSOR_MEASUREMENT_CORE_H
#define _MOCAP_SENSOR_MEASUREMENT_CORE_H


#include "msf_localization_core/sensor_measurement_core.h"


class MocapSensorMeasurementCore : public SensorMeasurementCore
{
public:
    MocapSensorMeasurementCore();
    MocapSensorMeasurementCore(std::weak_ptr<SensorCore> the_sensor_core);
public:
    ~MocapSensorMeasurementCore();

protected:
    int init();



    // Measurement
protected:
    // Position of the mocap sensor wrt mocap world
    Eigen::Vector3d position_mocap_sensor_wrt_mocap_world_;
    // Attitude of the mocap sensor wrt mocap world
    Eigen::Vector4d attitude_mocap_sensor_wrt_mocap_world_;


public:
    int setPositionMocapSensorWrtMocapWorld(const Eigen::Vector3d& position_mocap_sensor_wrt_mocap_world);
    int setAttitudeMocapSensorWrtMocapWorld(const Eigen::Vector4d& attitude_mocap_sensor_wrt_mocap_world);
public:
    Eigen::Vector3d getPositionMocapSensorWrtMocapWorld() const;
    Eigen::Vector4d getAttitudeMocapSensorWrtMocapWorld() const;




    //// Get the innovation vector as an Eigen::VectorXd
public:
    Eigen::VectorXd getInnovation(std::shared_ptr<SensorMeasurementCore> theMatchedMeasurement, std::shared_ptr<SensorMeasurementCore> thePredictedMeasurement);


    //// Get the full measurement as an Eigen::VectorXd
public:
    Eigen::VectorXd getMeasurement();
};


#endif
