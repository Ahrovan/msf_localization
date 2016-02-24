
#ifndef _SENSOR_CORE_H
#define _SENSOR_CORE_H


//I/O stream
//std::cout
#include <iostream>

#include <memory>



#include "sensor_basics.h"


// For TimeStamp






class MsfStorageCore;


class SensorCore : public SensorBasics
{
public:
    SensorCore();
    virtual ~SensorCore();


    // Pointer to itself
protected:
//public:
    std::weak_ptr<const SensorCore> TheSensorCorePtr;
public:
    int setTheSensorCore(std::weak_ptr<const SensorCore> TheSensorCorePtr);
    std::shared_ptr<const SensorCore> getTheSensorCore() const;


    // Pointer to the MSF Storage Core
protected:
    std::weak_ptr<MsfStorageCore> TheMsfStorageCore;
public:
    int setTheMsfStorageCore(std::weak_ptr<MsfStorageCore> TheMsfStorageCore);




    // Pose of the sensor wrt robot
protected:
    bool flagEstimationAttitudeSensorWrtRobot;
public:
    bool isEstimationAttitudeSensorWrtRobotEnabled() const;
    int enableEstimationAttitudeSensorWrtRobot();

    // Covariance


protected:
    bool flagEstimationPositionSensorWrtRobot;
public:
    bool isEstimationPositionSensorWrtRobotEnabled() const;
    int enableEstimationPositionSensorWrtRobot();

    // Covariance








};







#endif
