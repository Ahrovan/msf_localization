
#ifndef _ABSOLUTE_POSE_SENSOR_MEASUREMENT_CORE_H
#define _ABSOLUTE_POSE_SENSOR_MEASUREMENT_CORE_H


#include "msf_localization_core/sensor_measurement_core.h"


class AbsolutePoseSensorMeasurementCore : public SensorMeasurementCore
{
public:
    AbsolutePoseSensorMeasurementCore();
    AbsolutePoseSensorMeasurementCore(const std::weak_ptr<SensorCore> the_sensor_core);
public:
    ~AbsolutePoseSensorMeasurementCore();

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


    // Noise Sensor Measurement
protected:
    Eigen::MatrixXd noise_sensor_measurement_pose_sensor_wrt_sensor_world_;
public:
    void setNoiseSensorMeasurementPoseSensorWrtSensorWorld(const Eigen::MatrixXd& noise_sensor_measurement_pose_sensor_wrt_sensor_world);
    Eigen::MatrixXd  getNoiseSensorMeasurementPoseSensorWrtSensorWorld() const;




    //// Get the innovation vector as an Eigen::VectorXd
public:
    Eigen::VectorXd getInnovation(const std::shared_ptr<SensorMeasurementCore>& theMatchedMeasurement,
                                  const std::shared_ptr<SensorMeasurementCore>& thePredictedMeasurement);


    //// Get the full measurement as an Eigen::VectorXd
public:
    Eigen::VectorXd getMeasurement();



    ///// Covariances getters

    // Covariance Sensor Error Measurements: Rn
public:
    Eigen::SparseMatrix<double> getCovarianceMeasurement();

};


#endif
