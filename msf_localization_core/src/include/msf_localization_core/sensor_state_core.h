
#ifndef _SENSOR_STATE_CORE_H
#define _SENSOR_STATE_CORE_H



#include <Eigen/Dense>

#include "msf_localization_core/sensor_core.h"



class SensorStateCore
{
public:
    SensorStateCore();
    SensorStateCore(std::weak_ptr<const SensorCore> TheSensorCorePtr);
    ~SensorStateCore();


protected:
    // TODO SensorState
    // TODO SensorErrorState
    // TODO SensorMeasurements



protected:
    // TODO predictState()
    // TODO measurementsPrediction()



    // Ptr to the sensor core
protected:
    // It is not the owner of this Pointer. it doesn't modify the pointer
    std::weak_ptr<const SensorCore> TheSensorCorePtr;
public:
    int setTheSensorCore(std::weak_ptr<const SensorCore> TheSensorCorePtr);
    std::shared_ptr<const SensorCore> getTheSensorCore() const;
    std::shared_ptr<const SensorCore> getTheSensorCoreShared() const;
    std::weak_ptr<const SensorCore> getTheSensorCoreWeak() const;




    // Common State

    // Pose of the sensor wrt robot
    // position
protected:
public:
    Eigen::Vector3d positionSensorWrtRobot;
public:
    Eigen::Vector3d getPositionSensorWrtRobot() const;
    int setPositionSensorWrtRobot(Eigen::Vector3d positionSensorWrtRobot);


    // attitude
protected:
public:
    Eigen::Vector4d attitudeSensorWrtRobot;
public:
    Eigen::Vector4d getAttitudeSensorWrtRobot() const;
    int setAttitudeSensorWrtRobot(Eigen::Vector4d attitudeSensorWrtRobot);




public:
    virtual Eigen::MatrixXd getJacobianErrorState()=0;
    virtual Eigen::SparseMatrix<double> getJacobianErrorStateNoise()=0;


public:
    virtual int updateStateFromIncrementErrorState(Eigen::VectorXd increment_error_state)=0;


};



#endif
