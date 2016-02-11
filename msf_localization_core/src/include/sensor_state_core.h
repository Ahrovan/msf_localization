
#ifndef _SENSOR_STATE_CORE_H
#define _SENSOR_STATE_CORE_H



#include <Eigen/Dense>

#include "sensor_core.h"



class SensorStateCore
{
public:
    SensorStateCore();
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




    // Common State

    // Pose of the sensor wrt robot
protected:
    // Reference
    Eigen::Vector3d tranPoseSensorWrtRobot_ref;
    Eigen::Vector4d quatPoseSensorWrtRobot_ref;
    // Error
    // Not needed!



};



#endif
