
#include "msf_localization_core/imu_sensor_measurement_core.h"

#include "msf_localization_core/imu_sensor_core.h"


ImuSensorMeasurementCore::ImuSensorMeasurementCore() :
    SensorMeasurementCore()
{
    init();

    return;
}

ImuSensorMeasurementCore::ImuSensorMeasurementCore(std::weak_ptr<SensorCore> TheSensorCorePtr) :
    SensorMeasurementCore(TheSensorCorePtr)
{
    init();

    return;
}

ImuSensorMeasurementCore::~ImuSensorMeasurementCore()
{
    return;
}

int ImuSensorMeasurementCore::init()
{
    // Flags Set
    flagOrientationSet=false;
    flagAngularVelocitySet=false;
    flagLinearAccelerationSet=false;

    // Measurement type
    measurementType=MeasurementTypes::imu;

    return 0;
}

bool ImuSensorMeasurementCore::isOrientationSet() const
{
    return this->flagOrientationSet;
}

int ImuSensorMeasurementCore::setOrientation(const Eigen::Vector4d &Orientation)
{
    std::shared_ptr<const ImuSensorCore> TheSensorCorePtrAux=std::dynamic_pointer_cast<const ImuSensorCore>(this->getSensorCoreSharedPtr());
    if(TheSensorCorePtrAux->isMeasurementOrientationEnabled())
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

int ImuSensorMeasurementCore::setAngularVelocity(const Eigen::Vector3d &AngularVelocity)
{
    std::shared_ptr<const ImuSensorCore> TheSensorCorePtrAux=std::dynamic_pointer_cast<const ImuSensorCore>(this->getSensorCoreSharedPtr());
    if(TheSensorCorePtrAux->isMeasurementAngularVelocityEnabled())
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

int ImuSensorMeasurementCore::setLinearAcceleration(const Eigen::Vector3d &LinearAcceleration)
{
    std::shared_ptr<const ImuSensorCore> TheSensorCorePtrAux=std::dynamic_pointer_cast<const ImuSensorCore>(this->getSensorCoreSharedPtr());
    if(TheSensorCorePtrAux->isMeasurementLinearAccelerationEnabled())
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

Eigen::VectorXd ImuSensorMeasurementCore::getInnovation(std::shared_ptr<SensorMeasurementCore> theMatchedMeasurementI, std::shared_ptr<SensorMeasurementCore> thePredictedMeasurementI)
{
    // Create the Measurement
    Eigen::VectorXd TheInnovation;
    TheInnovation.resize(this->getSensorCoreSharedPtr()->getDimensionErrorMeasurement(), 1);
    TheInnovation.setZero();

    // Check
    if(theMatchedMeasurementI->getSensorCoreSharedPtr() != thePredictedMeasurementI->getSensorCoreSharedPtr())
    {
        std::cout<<"CodedVisualMarkerMeasurementCore::getInnovation() error"<<std::endl;
    }


    // TODO Improve!
    TheInnovation=theMatchedMeasurementI->getMeasurement()-thePredictedMeasurementI->getMeasurement();

    //std::cout<<"Innov IMU="<<TheInnovation.transpose()<<std::endl;


    return TheInnovation;
}

Eigen::VectorXd ImuSensorMeasurementCore::getMeasurement()
{
    // Create the Measurement
    Eigen::VectorXd TheMeasurement;
    TheMeasurement.resize(this->getSensorCoreSharedPtr()->getDimensionMeasurement(), 1);
    TheMeasurement.setZero();

    // Sensor Core
    std::shared_ptr<ImuSensorCore> TheImuSensorCore=std::dynamic_pointer_cast<ImuSensorCore>(this->getSensorCoreSharedPtr());

    // Fill
    unsigned int dimension=0;
    if(TheImuSensorCore->isMeasurementLinearAccelerationEnabled())
    {
        TheMeasurement.block<3,1>(dimension,0)=getLinearAcceleration();
        dimension+=3;
    }
    if(TheImuSensorCore->isMeasurementOrientationEnabled())
    {
        TheMeasurement.block<4,1>(dimension,0)=getOrientation();
        dimension+=4;
    }
    if(TheImuSensorCore->isMeasurementAngularVelocityEnabled())
    {
        TheMeasurement.block<3,1>(dimension,0)=getAngularVelocity();
        dimension+=3;
    }


    return TheMeasurement;
}
