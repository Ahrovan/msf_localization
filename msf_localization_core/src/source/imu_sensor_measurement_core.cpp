
#include "imu_sensor_measurement_core.h"

#include "imu_sensor_core.h"


ImuSensorMeasurementCore::ImuSensorMeasurementCore()
{
    // SensorType
    //std::shared_ptr<ImuSensorCore> ImuSensorCorePtrAux=std::dynamic_pointer_cast<ImuSensorCore>(this->TheSensorCorePtr.lock());
//    if(!ImuSensorCorePtrAux.lock())
//        std::cout<<"error"<<std::endl;
    //ImuSensorCorePtrAux->setSensorType(SensorTypes::imu);

    // Flags Enabled
//    flagOrientationEnabled=true;
//    flagAngularVelocityEnabled=true;
//    flagLinearAccelerationEnabled=true;

    // Flags Set
    flagOrientationSet=false;
    flagAngularVelocitySet=false;
    flagLinearAccelerationSet=false;

    return;
}

ImuSensorMeasurementCore::~ImuSensorMeasurementCore()
{
    return;
}


bool ImuSensorMeasurementCore::isOrientationSet() const
{
    return this->flagOrientationSet;
}

int ImuSensorMeasurementCore::setOrientation(const Eigen::Vector4d Orientation)
{
    std::shared_ptr<const ImuSensorCore> TheSensorCorePtrAux=std::dynamic_pointer_cast<const ImuSensorCore>(this->TheSensorCorePtr.lock());
    if(TheSensorCorePtrAux->isOrientationEnabled())
    {
        this->Orientation=Orientation;
        this->flagOrientationSet=true;
    }
    else
        return 1;
    return 0;
}

Eigen::Vector4d ImuSensorMeasurementCore::getOrientation() const
{
    return this->Orientation;
}


bool ImuSensorMeasurementCore::isAngularVelocitySet() const
{
    return this->flagAngularVelocitySet;
}

int ImuSensorMeasurementCore::setAngularVelocity(Eigen::Vector3d AngularVelocity)
{
    std::shared_ptr<const ImuSensorCore> TheSensorCorePtrAux=std::dynamic_pointer_cast<const ImuSensorCore>(this->TheSensorCorePtr.lock());
    if(TheSensorCorePtrAux->isAngularVelocityEnabled())
    {
        this->AngularVelocity=AngularVelocity;
        this->flagAngularVelocitySet=true;
    }
    else
        return 1;
    return 0;
}

Eigen::Vector3d ImuSensorMeasurementCore::getAngularVelocity() const
{
    return this->AngularVelocity;
}

bool ImuSensorMeasurementCore::isLinearAccelerationSet() const
{
    return this->flagLinearAccelerationSet;
}

int ImuSensorMeasurementCore::setLinearAcceleration(Eigen::Vector3d LinearAcceleration)
{
    std::shared_ptr<const ImuSensorCore> TheSensorCorePtrAux=std::dynamic_pointer_cast<const ImuSensorCore>(this->TheSensorCorePtr.lock());
    if(TheSensorCorePtrAux->isLinearAccelerationEnabled())
    {
        this->LinearAcceleration=LinearAcceleration;
        this->flagLinearAccelerationSet=true;
    }
    else
        return 1;
    return 0;
}

Eigen::Vector3d ImuSensorMeasurementCore::getLinearAcceleration() const
{
    return this->LinearAcceleration;
}
