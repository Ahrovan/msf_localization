
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



//unsigned int ImuSensorCore::getDimensionState() const
//{
//    unsigned int dimensionState=0;

//    if(isEstimationAttitudeSensorWrtRobotEnabled())
//        dimensionState+=4;

//    if(isEstimationPositionSensorWrtRobotEnabled())
//        dimensionState+=3;

//    if(isEstimationBiasAngularVelocityEnabled())
//        dimensionState+=3;

//    if(isEstimationBiasLinearAccelerationEnabled())
//        dimensionState+=3;

//    return dimensionState;
//}

//unsigned int ImuSensorCore::getDimensionErrorState() const
//{
//    unsigned int dimensionErrorState=0;

//    if(isEstimationAttitudeSensorWrtRobotEnabled())
//        dimensionState+=3;

//    if(isEstimationPositionSensorWrtRobotEnabled())
//        dimensionState+=3;

//    if(isEstimationBiasAngularVelocityEnabled())
//        dimensionState+=3;

//    if(isEstimationBiasLinearAccelerationEnabled())
//        dimensionState+=3;

//    return dimensionErrorState;
//}


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

Eigen::Matrix3d ImuSensorCore::getNoiseBiasAngularVelocity() const
{
    return this->noiseBiasAngularVelocity;
}

int ImuSensorCore::setNoiseBiasAngularVelocity(Eigen::Matrix3d noiseBiasAngularVelocity)
{
    this->noiseBiasAngularVelocity=noiseBiasAngularVelocity;
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

Eigen::Matrix3d ImuSensorCore::getNoiseBiasLinearAcceleration() const
{
    return this->noiseBiasLinearAcceleration;
}

int ImuSensorCore::setNoiseBiasLinearAcceleration(Eigen::Matrix3d noiseBiasLinearAcceleration)
{
    this->noiseBiasLinearAcceleration=noiseBiasLinearAcceleration;
    return 0;
}



int ImuSensorCore::predictState(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, const std::shared_ptr<ImuSensorStateCore> pastState, std::shared_ptr<ImuSensorStateCore>& predictedState)
{
    //std::cout<<"ImuSensorCore::predictState()"<<std::endl;

    // Checks in the past state
    if(!pastState->getTheSensorCore())
    {
        return -5;
        std::cout<<"ImuSensorCore::predictState() error !pastState->getTheSensorCore()"<<std::endl;
    }


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

int ImuSensorCore::predictMeasurement(const TimeStamp theTimeStamp, const std::shared_ptr<RobotStateCore> currentRobotState, std::shared_ptr<ImuSensorStateCore> currentImuState, std::shared_ptr<ImuSensorMeasurementCore>& predictedMeasurement)
{
    logFile<<"ImuSensorCore::predictMeasurement()"<<std::endl;

    // Check
    if(!this->getTheSensorCore())
    {
        std::cout<<"ImuSensorCore::predictMeasurement() error 50"<<std::endl;
        return 50;
    }

    // Check imu
    if(!currentImuState)
    {
        std::cout<<"ImuSensorCore::predictMeasurement() error 1"<<std::endl;
        return 1;
    }

    // Robot check
    if(!currentRobotState)
    {
        std::cout<<"ImuSensorCore::predictMeasurement() error 2"<<std::endl;
        return 2;
    }

    // Robot core check
    if(!currentRobotState->getTheRobotCore())
    {
        std::cout<<"ImuSensorCore::predictMeasurement() error 3"<<std::endl;
        return 3;
    }


    // Create pointer
    // TODO check if it must be done here
    if(!predictedMeasurement)
    {
        predictedMeasurement=std::make_shared<ImuSensorMeasurementCore>();
        logFile<<"ImuSensorCore::predictMeasurement() pointer created"<<std::endl;
    }


    // Set the sensor core -> Needed
    if(predictedMeasurement)
    {
        predictedMeasurement->setTheSensorCore(this->getTheSensorCore());
    }


    // Prediction
    // Orientation
    if(this->isOrientationEnabled())
    {
        logFile<<"ImuSensorCore::predictMeasurement() orientation"<<std::endl;

        // TODO

    }

    // Angular velocity
    if(isAngularVelocityEnabled())
    {
        logFile<<"ImuSensorCore::predictMeasurement() angular velocity"<<std::endl;


        Eigen::Vector3d ThePredictedAngularVelocity;


        // Switch depending on robot used
        switch(currentRobotState->getTheRobotCore()->getRobotType())
        {
            case RobotTypes::free_model:
            {
                // Cast
                std::shared_ptr<FreeModelRobotStateCore> currentFreeModelRobotState=std::static_pointer_cast<FreeModelRobotStateCore>(currentRobotState);

                // Model
                // TODO improve!!!
                ThePredictedAngularVelocity=currentFreeModelRobotState->getAngularVelocity()+currentImuState->getBiasesAngularVelocity();

                logFile<<"ImuSensorCore::predictMeasurement() predicted w="<<ThePredictedAngularVelocity.transpose()<<std::endl;

                // End
                break;
            }

        }


        // Set
        predictedMeasurement->setAngularVelocity(ThePredictedAngularVelocity);
    }

    // Linear acceleration
    if(isLinearAccelerationEnabled())
    {
        logFile<<"ImuSensorCore::predictMeasurement() linear acceleration"<<std::endl;


        Eigen::Vector3d ThePredictedLinearAcceleration;


        // Switch depending on robot used
        switch(currentRobotState->getTheRobotCore()->getRobotType())
        {
            case RobotTypes::free_model:
            {
                // Cast
                std::shared_ptr<FreeModelRobotStateCore> currentFreeModelRobotState=std::static_pointer_cast<FreeModelRobotStateCore>(currentRobotState);

                // Model
                // TODO improve!!!
                ThePredictedLinearAcceleration=currentFreeModelRobotState->getLinearAcceleration()+currentImuState->getBiasesLinearAcceleration();

                logFile<<"ImuSensorCore::predictMeasurement() predicted a="<<ThePredictedLinearAcceleration.transpose()<<std::endl;

                // End
                break;
            }

        }


        // Set
        predictedMeasurement->setLinearAcceleration(ThePredictedLinearAcceleration);
    }





    logFile<<"ImuSensorCore::predictMeasurement()"<<std::endl;

    // End
    return 0;
}
