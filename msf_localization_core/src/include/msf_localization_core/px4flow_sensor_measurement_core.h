
#ifndef _PX4FLOW_SENSOR_MEASUREMENT_CORE_H
#define _PX4FLOW_SENSOR_MEASUREMENT_CORE_H



#include "msf_localization_core/sensor_measurement_core.h"


//////////////////////////////
/// \brief The Px4FlowSensorMeasurementCore class
/////////////////////////////
class Px4FlowSensorMeasurementCore : public SensorMeasurementCore
{
public:
    Px4FlowSensorMeasurementCore();
    Px4FlowSensorMeasurementCore(const std::weak_ptr<SensorCore> sensor_core_ptr);
    ~Px4FlowSensorMeasurementCore();

protected:
    int init();

public:
    bool isMeasurementSet() const;

    // Dimension of the measurement
public:
    int getDimensionMeasurement() const;
    int getDimensionErrorMeasurement() const;



    ///// Measurement


    // Meas=[v, dg]


    // Velocity: v=[vx, vy]
protected:
    bool flag_velocity_set_;
public:
    bool isVelocitySet() const;
protected:
    Eigen::Vector2d velocity_;
public:
    void setVelocity(const Eigen::Vector2d& velocity);
    Eigen::Vector2d getVelocity() const;


    // Ground Distance: dg
protected:
    bool flag_ground_distance_set_;
public:
    bool isGroundDistanceSet() const;
protected:
    double ground_distance_;
public:
    void setGroundDistance(double ground_distance);
    double getGroundDistance() const;





    //// Get the innovation vector as an Eigen::VectorXd
public:
    Eigen::VectorXd getInnovation(const std::shared_ptr<SensorMeasurementCore>& theMatchedMeasurement,
                                  const std::shared_ptr<SensorMeasurementCore>& thePredictedMeasurement);


    //// Get the full measurement as a Eigen::VectorXd
public:
    Eigen::VectorXd getMeasurement();


    ///// Covariances getters

    // Covariance Sensor Error Measurements: Rn
public:
    Eigen::SparseMatrix<double> getCovarianceMeasurement();



};



#endif
