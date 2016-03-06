
#include "msf_localization_core/imu_sensor_core.h"

// Circular Dependency
#include "msf_localization_core/msf_storage_core.h"


ImuSensorCore::ImuSensorCore() :
    SensorCore()
{
    // Sensor Type
    setSensorType(SensorTypes::imu);

    // Flags measurement
    flagMeasurementOrientation=false;
    flagMeasurementAngularVelocity=false;
    flagMeasurementLinearAcceleration=false;

    // Flags estimation -> By default are considered parameters
    flagEstimationBiasAngularVelocity=false;
    flagEstimationScaleAngularVelocity=false;
    flagEstimationBiasLinearAcceleration=false;
    flagEstimationScaleLinearAcceleration=false;

    // State -> Again just in case
    dimensionErrorState=0;
    dimensionState=0;

    // Parameters
    dimensionParameters+=12;
    dimensionErrorParameters+=12;

    // Noises measurements
    noiseMeasurementAngularVelocity.setZero();
    noiseMeasurementLinearAcceleration.setZero();

    // Noises parameters
    noiseBiasAngularVelocity.setZero();
    noiseScaleAngularVelocity.setZero();
    noiseBiasLinearAcceleration.setZero();
    noiseScaleLinearAcceleration.setZero();

    // Noises estimation
    noiseEstimationBiasAngularVelocity.setZero();
    noiseEstimationBiasLinearAcceleration.setZero();

    return;
}

ImuSensorCore::~ImuSensorCore()
{
    return;
}

bool ImuSensorCore::isMeasurementOrientationEnabled() const
{
    return flagMeasurementOrientation;
}

int ImuSensorCore::enableMeasurementOrientation()
{
    if(!this->flagMeasurementOrientation)
    {
        this->flagMeasurementOrientation=true;
        this->dimensionMeasurement+=3;
    }
    return 0;
}

bool ImuSensorCore::isMeasurementAngularVelocityEnabled() const
{
    return flagMeasurementAngularVelocity;
}

int ImuSensorCore::enableMeasurementAngularVelocity()
{
    if(!this->flagMeasurementAngularVelocity)
    {
        this->flagMeasurementAngularVelocity=true;
        this->dimensionMeasurement+=3;
    }
    return 0;
}

Eigen::Matrix3d ImuSensorCore::getNoiseMeasurementAngularVelocity() const
{
    return this->noiseMeasurementAngularVelocity;
}

int ImuSensorCore::setNoiseMeasurementAngularVelocity(Eigen::Matrix3d noiseMeasurementAngularVelocity)
{
    this->noiseMeasurementAngularVelocity=noiseMeasurementAngularVelocity;
    return 0;
}


bool ImuSensorCore::isMeasurementLinearAccelerationEnabled() const
{
    return flagMeasurementLinearAcceleration;
}

int ImuSensorCore::enableMeasurementLinearAcceleration()
{
    if(!this->flagMeasurementLinearAcceleration)
    {
        this->flagMeasurementLinearAcceleration=true;
        this->dimensionMeasurement+=3;
    }
    return 0;
}

Eigen::Matrix3d ImuSensorCore::getNoiseMeasurementLinearAcceleration() const
{
    return this->noiseMeasurementLinearAcceleration;
}

int ImuSensorCore::setNoiseMeasurementLinearAcceleration(Eigen::Matrix3d noiseMeasurementLinearAcceleration)
{
    this->noiseMeasurementLinearAcceleration=noiseMeasurementLinearAcceleration;
    return 0;
}


int ImuSensorCore::setMeasurement(const TimeStamp TheTimeStamp, std::shared_ptr<ImuSensorMeasurementCore> TheImuSensorMeasurement)
{
    //std::cout<<"Imu Measurement Set"<<std::endl;

    if(!isSensorEnabled())
        return 0;

    //std::shared_ptr<MsfStorageCore> TheMsfStorageCoreAux=this->TheMsfStorageCore.lock();
//    if(!TheMsfStorageCoreAux)
//        std::cout<<"Unable to lock TheMsfStorageCore"<<std::endl;

    this->getTheMsfStorageCore()->setMeasurement(TheTimeStamp, TheImuSensorMeasurement);

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
        //
        this->dimensionParameters-=3;
        //
        this->dimensionErrorParameters-=3;
    }
    return 0;
}

int ImuSensorCore::enableParameterBiasAngularVelocity()
{
    if(this->flagEstimationBiasAngularVelocity)
    {
        // Enable
        this->flagEstimationBiasAngularVelocity=false;
        // Update State Dimension
        this->dimensionState-=3;
        // Update Error State Dimension
        this->dimensionErrorState-=3;
        //
        this->dimensionParameters+=3;
        //
        this->dimensionErrorParameters+=3;
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


// Angular Velocity biases: Estimation Covariance (if enabled estimation)
Eigen::Matrix3d ImuSensorCore::getNoiseEstimationBiasAngularVelocity() const
{
    return this->noiseEstimationBiasAngularVelocity;
}

int ImuSensorCore::setNoiseEstimationBiasAngularVelocity(Eigen::Matrix3d noiseEstimationBiasAngularVelocity)
{
    this->noiseEstimationBiasAngularVelocity=noiseEstimationBiasAngularVelocity;
    return 0;
}


bool ImuSensorCore::isEstimationScaleAngularVelocityEnabled() const
{
    return this->flagEstimationScaleAngularVelocity;
}

int ImuSensorCore::enableEstimationScaleAngularVelocity()
{
    if(!this->flagEstimationScaleAngularVelocity)
    {
        // Enable
        this->flagEstimationScaleAngularVelocity=true;
        // Update State Dimension
        this->dimensionState+=3;
        // Update Error State Dimension
        this->dimensionErrorState+=3;
        //
        this->dimensionParameters-=3;
        //
        this->dimensionErrorParameters-=3;
    }
    return 0;
}

int ImuSensorCore::enableParameterScaleAngularVelocity()
{
    if(this->flagEstimationScaleAngularVelocity)
    {
        // Enable
        this->flagEstimationScaleAngularVelocity=false;
        // Update State Dimension
        this->dimensionState-=3;
        // Update Error State Dimension
        this->dimensionErrorState-=3;
        //
        this->dimensionParameters+=3;
        //
        this->dimensionErrorParameters+=3;
    }
    return 0;
}

Eigen::Matrix3d ImuSensorCore::getNoiseScaleAngularVelocity() const
{
    return this->noiseScaleAngularVelocity;
}

int ImuSensorCore::setNoiseScaleAngularVelocity(Eigen::Matrix3d noiseScaleAngularVelocity)
{
    this->noiseScaleAngularVelocity=noiseScaleAngularVelocity;
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
        //
        this->dimensionParameters-=3;
        //
        this->dimensionErrorParameters-=3;
    }
    return 0;
}

int ImuSensorCore::enableParameterBiasLinearAcceleration()
{
    if(this->flagEstimationBiasLinearAcceleration)
    {
        // Enable
        this->flagEstimationBiasLinearAcceleration=false;
        // Update State Dimension
        this->dimensionState-=3;
        // Update Error State Dimension
        this->dimensionErrorState-=3;
        //
        this->dimensionParameters+=3;
        //
        this->dimensionErrorParameters+=3;
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

Eigen::Matrix3d ImuSensorCore::getNoiseEstimationBiasLinearAcceleration() const
{
    return this->noiseEstimationBiasLinearAcceleration;
}

int ImuSensorCore::setNoiseEstimationBiasLinearAcceleration(Eigen::Matrix3d noiseEstimationBiasLinearAcceleration)
{
    this->noiseEstimationBiasLinearAcceleration=noiseEstimationBiasLinearAcceleration;
    return 0;
}


bool ImuSensorCore::isEstimationScaleLinearAccelerationEnabled() const
{
    return this->flagEstimationScaleLinearAcceleration;
}

int ImuSensorCore::enableEstimationScaleLinearAcceleration()
{
    if(!this->flagEstimationScaleLinearAcceleration)
    {
        // Enable
        this->flagEstimationScaleLinearAcceleration=true;
        // Update State Dimension
        this->dimensionState+=3;
        // Update Error State Dimension
        this->dimensionErrorState+=3;
        //
        this->dimensionParameters-=3;
        //
        this->dimensionErrorParameters-=3;
    }
    return 0;
}

int ImuSensorCore::enableParameterScaleLinearAcceleration()
{
    if(this->flagEstimationScaleLinearAcceleration)
    {
        // Enable
        this->flagEstimationScaleLinearAcceleration=false;
        // Update State Dimension
        this->dimensionState-=3;
        // Update Error State Dimension
        this->dimensionErrorState-=3;
        //
        this->dimensionParameters+=3;
        //
        this->dimensionErrorParameters+=3;
    }
    return 0;
}

Eigen::Matrix3d ImuSensorCore::getNoiseScaleLinearAcceleration() const
{
    return this->noiseScaleLinearAcceleration;
}

int ImuSensorCore::setNoiseScaleLinearAcceleration(Eigen::Matrix3d noiseScaleLinearAcceleration)
{
    this->noiseScaleLinearAcceleration=noiseScaleLinearAcceleration;
    return 0;
}


/*
int ImuSensorCore::setInitErrorStateVariancePositionSensorWrtRobot(Eigen::Vector3d initVariance)
{
    int point=0;
    if(this->isEstimationPositionSensorWrtRobotEnabled())
    {
        this->InitErrorStateVariance.block<3,3>(point,point)=initVariance.asDiagonal();
        return 0;
    }
    return -1;
}

int ImuSensorCore::setInitErrorStateVarianceAttitudeSensorWrtRobot(Eigen::Vector3d initVariance)
{
    int point=0;
    if(this->isEstimationPositionSensorWrtRobotEnabled())
        point+=3;
    if(this->isEstimationAttitudeSensorWrtRobotEnabled())
    {
        this->InitErrorStateVariance.block<3,3>(point,point)=initVariance.asDiagonal();
        return 0;
    }
    return -1;
}

int ImuSensorCore::setInitErrorStateVarianceBiasLinearAcceleration(Eigen::Vector3d initVariance)
{
    int point=0;
    if(this->isEstimationPositionSensorWrtRobotEnabled())
        point+=3;
    if(this->isEstimationAttitudeSensorWrtRobotEnabled())
    {
        point+=3;
    }
    if(this->isEstimationBiasLinearAccelerationEnabled())
    {
        this->InitErrorStateVariance.block<3,3>(point,point)=initVariance.asDiagonal();
        return 0;
    }
    return -1;
}

int ImuSensorCore::setInitErrorStateVarianceBiasAngularVelocity(Eigen::Vector3d initVariance)
{
    int point=0;
    if(this->isEstimationPositionSensorWrtRobotEnabled())
        point+=3;
    if(this->isEstimationAttitudeSensorWrtRobotEnabled())
    {
        point+=3;
    }
    if(this->isEstimationBiasLinearAccelerationEnabled())
    {
        point+=3;
    }
    if(this->isEstimationBiasAngularVelocityEnabled())
    {
        this->InitErrorStateVariance.block<3,3>(point,point)=initVariance.asDiagonal();
        return 0;
    }
    return -1;
}
*/


int ImuSensorCore::prepareInitErrorStateVariance()
{
    int error=SensorCore::prepareInitErrorStateVariance();

    if(error)
        return error;


    int point=0;
    if(this->isEstimationPositionSensorWrtRobotEnabled())
    {
        this->InitErrorStateVariance.block<3,3>(point,point)=noisePositionSensorWrtRobot;
        point+=3;
    }
    if(this->isEstimationAttitudeSensorWrtRobotEnabled())
    {
        this->InitErrorStateVariance.block<3,3>(point,point)=noiseAttitudeSensorWrtRobot;
        point+=3;
    }
    if(this->isEstimationBiasLinearAccelerationEnabled())
    {
        this->InitErrorStateVariance.block<3,3>(point,point)=noiseBiasLinearAcceleration;
        point+=3;
    }
    if(this->isEstimationBiasAngularVelocityEnabled())
    {
        this->InitErrorStateVariance.block<3,3>(point,point)=noiseBiasAngularVelocity;
    }



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


    // Scale Angular Velocity
    predictedState->scaleAngularVelocity=pastState->scaleAngularVelocity;


    // Bias Linear Acceleration
    predictedState->biasesLinearAcceleration=pastState->biasesLinearAcceleration;

    // Scale Linear Velocity
    predictedState->scaleLinearAcceleration=pastState->scaleLinearAcceleration;


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

    // ka / ka
    if(flagEstimationScaleLinearAcceleration)
        predictedState->errorStateJacobian.scaleLinearAcceleration=Eigen::Matrix3d::Identity();

    // bw / bw
    if(flagEstimationBiasAngularVelocity)
        predictedState->errorStateJacobian.biasesAngularVelocity=Eigen::Matrix3d::Identity();

    // kw / kw
    if(flagEstimationScaleAngularVelocity)
        predictedState->errorStateJacobian.scaleAngularVelocity=Eigen::Matrix3d::Identity();




    return 0;
}

int ImuSensorCore::predictMeasurement(const TimeStamp theTimeStamp, std::shared_ptr<GlobalParametersStateCore> TheGlobalParametersStateCore, const std::shared_ptr<RobotStateCore> currentRobotState, std::shared_ptr<ImuSensorStateCore> currentImuState, std::shared_ptr<ImuSensorMeasurementCore>& predictedMeasurement)
{
#if _DEBUG_SENSOR_CORE
    logFile<<"ImuSensorCore::predictMeasurement() TS: sec="<<theTimeStamp.sec<<" s; nsec="<<theTimeStamp.nsec<<" ns"<<std::endl;
#endif

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


    // Checks
    // TODO


    // Create pointer
    // TODO check if it must be done here
    if(!predictedMeasurement)
    {
        predictedMeasurement=std::make_shared<ImuSensorMeasurementCore>();
#if _DEBUG_SENSOR_CORE
        logFile<<"ImuSensorCore::predictMeasurement() pointer created"<<std::endl;
#endif
    }


    // Set the sensor core -> Needed
    if(predictedMeasurement)
    {
        predictedMeasurement->setTheSensorCore(this->getTheSensorCore());
    }


    // Prediction
    // Orientation
    if(this->isMeasurementOrientationEnabled())
    {
#if _DEBUG_SENSOR_CORE
        logFile<<"ImuSensorCore::predictMeasurement() orientation"<<std::endl;
#endif

        // TODO

    }

    // Angular velocity
    if(isMeasurementAngularVelocityEnabled())
    {
#if _DEBUG_SENSOR_CORE
        logFile<<"ImuSensorCore::predictMeasurement() angular velocity"<<std::endl;
#endif


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

#if _DEBUG_SENSOR_CORE
                logFile<<"ImuSensorCore::predictMeasurement() predicted w="<<ThePredictedAngularVelocity.transpose()<<std::endl;
#endif

                // End
                break;
            }
            default:
            {
                return -1000;
                break;
            }
        }


        // Set
        predictedMeasurement->setAngularVelocity(ThePredictedAngularVelocity);
    }

    // Linear acceleration
    if(isMeasurementLinearAccelerationEnabled())
    {
#if _DEBUG_SENSOR_CORE
        logFile<<"ImuSensorCore::predictMeasurement() linear acceleration"<<std::endl;
#endif


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

#if _DEBUG_SENSOR_CORE
                logFile<<"ImuSensorCore::predictMeasurement() predicted a="<<ThePredictedLinearAcceleration.transpose()<<std::endl;
#endif

                // End
                break;
            }
            default:
            {
                return -1000;
                break;
            }
        }


        // Set
        predictedMeasurement->setLinearAcceleration(ThePredictedLinearAcceleration);
    }




#if _DEBUG_SENSOR_CORE
    logFile<<"ImuSensorCore::predictMeasurement() ended TS: sec="<<theTimeStamp.sec<<" s; nsec="<<theTimeStamp.nsec<<" ns"<<std::endl;
#endif

    // End
    return 0;
}




int ImuSensorCore::jacobiansMeasurements(const TimeStamp theTimeStamp, std::shared_ptr<GlobalParametersStateCore> TheGlobalParametersStateCore, std::shared_ptr<RobotStateCore> TheRobotStateCore, std::shared_ptr<ImuSensorStateCore>& TheImuStateCore)
{

    // Checks
    // TODO



    /// Common Variables

    // dimension of the measurement
    int dimensionMeasurement=TheImuStateCore->getTheSensorCore()->getDimensionMeasurement();




    /// Jacobians measurement - state

    // Jacobian measurement - robot state
    switch(TheRobotStateCore->getTheRobotCore()->getRobotType())
    {
        case RobotTypes::free_model:
        {
            // Robot
            std::shared_ptr<FreeModelRobotStateCore> TheRobotStateCoreAux=std::static_pointer_cast<FreeModelRobotStateCore>(TheRobotStateCore);
            std::shared_ptr<const FreeModelRobotCore> TheRobotCoreAux=std::static_pointer_cast<const FreeModelRobotCore>(TheRobotStateCoreAux->getTheRobotCore());

            // dimension of robot state
            int dimensionRobotErrorState=TheRobotCoreAux->getDimensionErrorState();

            // Resize and init Jacobian
            TheImuStateCore->jacobianMeasurementErrorState.jacobianMeasurementRobotErrorState.resize(dimensionMeasurement, dimensionRobotErrorState);
            TheImuStateCore->jacobianMeasurementErrorState.jacobianMeasurementRobotErrorState.setZero();

            // Fill Jacobian
            // TODO

            // end
            break;
        }
        default:
        {
            return -2;
        }

    }


    // Jacobian measurement - sensor state

    // dimension of the sensor state
    int dimensionSensorErrorState=TheImuStateCore->getTheSensorCore()->getDimensionErrorState();

    // Resize and init Jacobian
    TheImuStateCore->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState.resize(dimensionMeasurement, dimensionSensorErrorState);
    TheImuStateCore->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState.setZero();

    // Fill Jacobian
    // TODO



    /// Jacobians measurement - sensor parameters

    // dimension of the sensor parameters
    int dimensionSensorParameters=TheImuStateCore->getTheSensorCore()->getDimensionErrorParameters();

    // Resize and init Jacobian
    TheImuStateCore->jacobianMeasurementSensorParameters.jacobianMeasurementSensorParameters.resize(dimensionMeasurement, dimensionSensorParameters);
    TheImuStateCore->jacobianMeasurementSensorParameters.jacobianMeasurementSensorParameters.setZero();

    // Fill Jacobian
    // TODO



    /// Jacobians measurement - global parameters

    // dimension of the global parameters
    int dimensionGlobalParameters=TheGlobalParametersStateCore->getTheGlobalParametersCore()->getDimensionErrorParameters();

    // Resize and init Jacobian
    TheImuStateCore->jacobianMeasurementGlobalParameters.jacobianMeasurementGlobalParameters.resize(dimensionMeasurement, dimensionGlobalParameters);
    TheImuStateCore->jacobianMeasurementGlobalParameters.jacobianMeasurementGlobalParameters.setZero();

    // Fill Jacobian
    // TODO



    /// Jacobians measurement - sensor noise of the measurement

    // Resize and init Jacobian
    TheImuStateCore->jacobianMeasurementSensorNoise.jacobianMeasurementSensorNoise.resize(dimensionMeasurement, dimensionMeasurement);
    TheImuStateCore->jacobianMeasurementSensorNoise.jacobianMeasurementSensorNoise.setZero();

    // Fill Jacobian
    // TODO



    // End
    return 0;
}
