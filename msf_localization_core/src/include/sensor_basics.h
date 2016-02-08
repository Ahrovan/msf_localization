
#ifndef _SENSOR_BASICS_H
#define _SENSOR_BASICS_H


//I/O stream
//std::cout
#include <iostream>




enum SensorTypes
{
    undefined=0,
    imu=1
};



class SensorBasics
{
public:
    SensorBasics();
    virtual ~SensorBasics();


protected:
    SensorTypes sensorType;
public:
    int setSensorType(SensorTypes sensorType);
    SensorTypes getSensorType() const;

protected:
    int sensorId;
public:
    int setSensorId(int sensorId);
    int getSensorId() const;


};







#endif
