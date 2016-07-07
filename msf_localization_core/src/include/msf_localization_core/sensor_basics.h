
#ifndef _SENSOR_BASICS_H
#define _SENSOR_BASICS_H


//I/O stream
//std::cout
#include <iostream>




enum class SensorTypes
{
    undefined=0,
    imu=1,
    coded_visual_marker_eye=2,
    absolute_pose=3,
    px4flow
};



class SensorBasics
{
public:
    SensorBasics();
    ~SensorBasics();


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



protected:
    bool flagSensorEnabled;
public:
    bool isSensorEnabled() const;
    int setSensorEnabled(bool flagSensorEnabled);


    //// Name
protected:
    std::string sensor_name_;
public:
    int setSensorName(std::string sensor_name);
    std::string getSensorName() const;

};







#endif
