
#include "imu_sensor_core.h"

// Circular Dependency
#include "msf_storage_core.h"


ImuSensorCore::ImuSensorCore()
{
    // Sensor Type
    setSensorType(SensorTypes::imu);

    // Flags measurement
    flagMeasurementOrientation=false;
    flagMeasurementAngularVelocity=false;
    flagMeasurementLinearAcceleration=false;

    return;
}

ImuSensorCore::~ImuSensorCore()
{
    return;
}

bool ImuSensorCore::isOrientationEnabled() const
{
    return flagMeasurementOrientation;
}

int ImuSensorCore::enableOrientation()
{
    this->flagMeasurementOrientation=true;
    return 0;
}

bool ImuSensorCore::isAngularVelocityEnabled() const
{
    return flagMeasurementAngularVelocity;
}

int ImuSensorCore::enableAngularVelocity()
{
    this->flagMeasurementAngularVelocity=true;
    return 0;
}

bool ImuSensorCore::isLinearAccelerationEnabled() const
{
    return flagMeasurementLinearAcceleration;
}

int ImuSensorCore::enableLinearAcceleration()
{
    this->flagMeasurementLinearAcceleration=true;
    return 0;
}

int ImuSensorCore::setMeasurement(const TimeStamp TheTimeStamp, std::shared_ptr<ImuSensorMeasurementCore> TheImuSensorMeasurement)
{
    std::cout<<"Imu Measurement Set"<<std::endl;

    std::shared_ptr<MsfStorageCore> TheMsfStorageCoreAux=this->TheMsfStorageCore.lock();
//    if(!TheMsfStorageCoreAux)
//        std::cout<<"Unable to lock TheMsfStorageCore"<<std::endl;

    TheMsfStorageCoreAux->setMeasurement(TheTimeStamp, TheImuSensorMeasurement);

    return 0;
}



int ImuSensorCore::predictState(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, const std::shared_ptr<ImuSensorStateCore> pastState, std::shared_ptr<ImuSensorStateCore>& predictedState)
{
    //std::cout<<"ImuSensorCore::predictState()"<<std::endl;

    // Create the predicted state if it doen't exists
    if(!predictedState)
    {
        predictedState=std::make_shared<ImuSensorStateCore>();
    }

    // Equations
    *predictedState=*pastState;

    // Set The core
    predictedState->setTheSensorCore(pastState->getTheSensorCore());

    //std::cout<<"ImuSensorCore::predictState() end"<<std::endl;

    return 0;
}

int ImuSensorCore::predictStateJacobians(TimeStamp theTimeStamp, std::shared_ptr<ImuSensorStateCore> currentState)
{

    return 0;
}

int ImuSensorCore::predictMeasurement(TimeStamp theTimeStamp, std::shared_ptr<ImuSensorStateCore> currentState, std::shared_ptr<ImuSensorMeasurementCore> predictedMeasurement)
{

    return 0;
}
