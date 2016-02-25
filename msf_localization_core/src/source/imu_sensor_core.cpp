
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

    // Flags estimation
    flagEstimationAttitudeSensorWrtRobot=false;
    flagEstimationPositionSensorWrtRobot=false;
    flagEstimationBiasAngularVelocity=false;
    flagEstimationBiasLinearAcceleration=false;


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
    //std::cout<<"Imu Measurement Set"<<std::endl;

    if(!isSensorEnabled())
        return 0;

    std::shared_ptr<MsfStorageCore> TheMsfStorageCoreAux=this->TheMsfStorageCore.lock();
//    if(!TheMsfStorageCoreAux)
//        std::cout<<"Unable to lock TheMsfStorageCore"<<std::endl;

    TheMsfStorageCoreAux->setMeasurement(TheTimeStamp, TheImuSensorMeasurement);

    return 0;
}

bool ImuSensorCore::isEstimationBiasAngularVelocityEnabled() const
{
    return this->flagEstimationBiasAngularVelocity;
}

int ImuSensorCore::enableEstimationBiasAngularVelocity()
{
    if(!this->flagEstimationBiasAngularVelocity)
    {
        // Enable
        this->flagEstimationBiasAngularVelocity=true;
        // Update State Dimension
        this->dimensionState+=3;
        // Update Error State Dimension
        this->dimensionErrorState+=3;
    }
    return 0;
}


bool ImuSensorCore::isEstimationBiasLinearAccelerationEnabled() const
{
    return this->flagEstimationBiasLinearAcceleration;
}

int ImuSensorCore::enableEstimationBiasLinearAcceleration()
{
    if(!this->flagEstimationBiasLinearAcceleration)
    {
        // Enable
        this->flagEstimationBiasLinearAcceleration=true;
        // Update State Dimension
        this->dimensionState+=3;
        // Update Error State Dimension
        this->dimensionErrorState+=3;
    }
    return 0;
}



int ImuSensorCore::predictState(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, const std::shared_ptr<ImuSensorStateCore> pastState, std::shared_ptr<ImuSensorStateCore>& predictedState)
{
    //std::cout<<"ImuSensorCore::predictState()"<<std::endl;

    // Create the predicted state if it doesn't exists
    if(!predictedState)
    {
        predictedState=std::make_shared<ImuSensorStateCore>();
    }

    // Set The sensor core if it doesn't exist
    if(!predictedState->getTheSensorCore())
    {
        predictedState->setTheSensorCore(pastState->getTheSensorCore());
    }



    // Equations
    //*predictedState=*pastState;


    // Pose of the sensor wrt Robot

    // Position of sensor wrt Robot
    predictedState->positionSensorWrtRobot=pastState->positionSensorWrtRobot;

    // Attitude of sensor wrt Robot
    predictedState->attitudeSensorWrtRobot=pastState->attitudeSensorWrtRobot;


    // Bias Angular Velocity
    predictedState->biasesAngularVelocity=pastState->biasesAngularVelocity;

    // Bias Linear Acceleration
    predictedState->biasesLinearAcceleration=pastState->biasesLinearAcceleration;



    //std::cout<<"ImuSensorCore::predictState() end"<<std::endl;

    return 0;
}

int ImuSensorCore::predictStateErrorStateJacobians(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, std::shared_ptr<ImuSensorStateCore> pastState, std::shared_ptr<ImuSensorStateCore>& predictedState)
{
    //std::cout<<"ImuSensorCore::predictStateErrorStateJacobians"<<std::endl;

    // Create the predicted state if it doesn't exist
    if(!predictedState)
    {
        return 1;
    }


    // Jacobians
    // posi / posi
    if(flagEstimationPositionSensorWrtRobot)
        predictedState->errorStateJacobian.positionSensorWrtRobot=Eigen::Matrix3d::Identity();

    // att / att
    if(flagEstimationAttitudeSensorWrtRobot)
        predictedState->errorStateJacobian.attitudeSensorWrtRobot=Eigen::Matrix3d::Identity();

    // ba / ba
    if(flagEstimationBiasLinearAcceleration)
        predictedState->errorStateJacobian.biasesLinearAcceleration=Eigen::Matrix3d::Identity();

    // bw / bw
    if(flagEstimationBiasAngularVelocity)
        predictedState->errorStateJacobian.biasesAngularVelocity=Eigen::Matrix3d::Identity();





    return 0;
}

int ImuSensorCore::predictMeasurement(TimeStamp theTimeStamp, std::shared_ptr<ImuSensorStateCore> currentState, std::shared_ptr<ImuSensorMeasurementCore> predictedMeasurement)
{

    return 0;
}
