
#ifndef _SENSOR_CORE_H
#define _SENSOR_CORE_H


//I/O stream
//std::cout
#include <iostream>




enum SensorTypes
{
    undefined=0,
    imu=1
};



class SensorCore
{
public:
    SensorCore();
    virtual ~SensorCore();

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
