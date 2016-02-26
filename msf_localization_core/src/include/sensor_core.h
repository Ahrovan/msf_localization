
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
    std::weak_ptr<const SensorCore> TheSensorCorePtr;
public:
    int setTheSensorCore(std::weak_ptr<const SensorCore> TheSensorCorePtr);
    std::shared_ptr<const SensorCore> getTheSensorCore() const;


    // Pointer to the MSF Storage Core
protected:
    std::weak_ptr<MsfStorageCore> TheMsfStorageCore;
public:
    int setTheMsfStorageCore(std::weak_ptr<MsfStorageCore> TheMsfStorageCore);


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



    // Pose of the sensor wrt robot

    // Attitude
protected:
    bool flagEstimationAttitudeSensorWrtRobot;
public:
    bool isEstimationAttitudeSensorWrtRobotEnabled() const;
    int enableEstimationAttitudeSensorWrtRobot();


    // Position
protected:
    bool flagEstimationPositionSensorWrtRobot;
public:
    bool isEstimationPositionSensorWrtRobotEnabled() const;
    int enableEstimationPositionSensorWrtRobot();







};







#endif
