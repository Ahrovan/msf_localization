
#ifndef _SENSOR_STATE_CORE_H
#define _SENSOR_STATE_CORE_H



#include <Eigen/Dense>

#include "msf_localization_core/state_core.h"

//#include "msf_localization_core/sensor_core.h"



enum class SensorStateCoreTypes
{
    undefined=0,
    imu=1,
    coded_visual_maker_eye
};



class SensorStateCore : public StateCore
{
public:
    SensorStateCore();
    SensorStateCore(std::weak_ptr<MsfElementCore> msf_element_core_ptr);
    ~SensorStateCore();


protected:
    int init();



protected:
    SensorStateCoreTypes sensor_state_core_type_;
public:
    int setSensorStateCoreType(SensorStateCoreTypes sensor_state_core_type);
    SensorStateCoreTypes getSensorStateCoreType();





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







};



#endif
