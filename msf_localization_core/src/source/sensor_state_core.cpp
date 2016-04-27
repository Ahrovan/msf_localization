
#include "msf_localization_core/sensor_state_core.h"



SensorStateCore::SensorStateCore() :
    StateCore()
{
    return;
}

SensorStateCore::SensorStateCore(std::weak_ptr<MsfElementCore> msf_element_core_ptr) :
    StateCore(msf_element_core_ptr)
{


    return;
}

SensorStateCore::~SensorStateCore()
{
    return;
}

int SensorStateCore::init()
{
    this->setStateCoreType(StateCoreTypes::sensor);
    this->sensor_state_core_type_=SensorStateCoreTypes::undefined;


    return 0;
}


int SensorStateCore::setSensorStateCoreType(SensorStateCoreTypes sensor_state_core_type)
{
    this->sensor_state_core_type_=sensor_state_core_type;
    return 0;
}

SensorStateCoreTypes SensorStateCore::getSensorStateCoreType()
{
    return sensor_state_core_type_;
}



Eigen::Vector3d SensorStateCore::getPositionSensorWrtRobot() const
{
    return this->positionSensorWrtRobot;
}

int SensorStateCore::setPositionSensorWrtRobot(Eigen::Vector3d positionSensorWrtRobot)
{
    this->positionSensorWrtRobot=positionSensorWrtRobot;
    return 0;
}

Eigen::Vector4d SensorStateCore::getAttitudeSensorWrtRobot() const
{
    return this->attitudeSensorWrtRobot;
}

int SensorStateCore::setAttitudeSensorWrtRobot(Eigen::Vector4d attitudeSensorWrtRobot)
{
    this->attitudeSensorWrtRobot=attitudeSensorWrtRobot;
    return 0;
}
