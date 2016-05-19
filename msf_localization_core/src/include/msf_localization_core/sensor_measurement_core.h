

#ifndef _SENSOR_MEASUREMENT_CORE_H
#define _SENSOR_MEASUREMENT_CORE_H


#include <memory>


#include <Eigen/Dense>


#include "msf_localization_core/sensor_core.h"


enum class MeasurementTypes
{
    undefined=0,
    imu=1,
    coded_visual_marker=2,
    mocap
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



    //// Jacobians Error Measurement

    /// Jacobian Error Measurement wrt Error State
    /// Hx
protected:
public:
    struct
    {
        Eigen::SparseMatrix<double> world;
        Eigen::SparseMatrix<double> robot;
        std::vector<Eigen::SparseMatrix<double> > inputs;
        std::vector<Eigen::SparseMatrix<double> > sensors;
        std::vector<Eigen::SparseMatrix<double> > map_elements;
    } jacobian_error_measurement_wrt_error_state_;


    /// Jacobian Error Measurement wrt Error Parameters
    /// Hp
protected:
public:
    struct
    {
        Eigen::SparseMatrix<double> world;
        Eigen::SparseMatrix<double> robot;
        std::vector<Eigen::SparseMatrix<double> > inputs;
        std::vector<Eigen::SparseMatrix<double> > sensors;
        std::vector<Eigen::SparseMatrix<double> > map_elements;
    } jacobian_error_measurement_wrt_error_parameters_;




    /// Jacobian Error Measurement wrt Noise
    /// Hn
protected:
public:
    struct
    {
        Eigen::SparseMatrix<double> measurement;
    } jacobian_error_measurement_wrt_error_measurement_;





    //// Get the innovation vector as an Eigen::VectorXd
public:
    virtual Eigen::VectorXd getInnovation(std::shared_ptr<SensorMeasurementCore> theMatchedMeasurement, std::shared_ptr<SensorMeasurementCore> thePredictedMeasurement)=0;


    //// Get the full measurement as an Eigen::VectorXd
public:
    virtual Eigen::VectorXd getMeasurement()=0;




    ///// Covariances getters

    // Covariance Sensor Error Measurements: Rn
public:
    virtual Eigen::SparseMatrix<double> getCovarianceMeasurement();




};






#endif
