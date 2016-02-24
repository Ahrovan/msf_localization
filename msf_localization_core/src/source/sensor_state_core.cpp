
#include "sensor_state_core.h"



SensorStateCore::SensorStateCore()
{
    return;
}

SensorStateCore::~SensorStateCore()
{
    return;
}

int SensorStateCore::setTheSensorCore(std::weak_ptr<const SensorCore> TheSensorCorePtr)
{
    this->TheSensorCorePtr=TheSensorCorePtr;
    return 0;
}
std::shared_ptr<const SensorCore> SensorStateCore::getTheSensorCore() const
{
    std::shared_ptr<const SensorCore> TheSensorCoreSharedPtr=this->TheSensorCorePtr.lock();
    return TheSensorCoreSharedPtr;
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
