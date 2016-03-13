
#include "msf_localization_core/imu_sensor_core.h"

// Circular Dependency
#include "msf_localization_core/msf_storage_core.h"


ImuSensorCore::ImuSensorCore() :
    SensorCore()
{
    // Sensor Type
    setSensorType(SensorTypes::imu);


    // Sensor name -> Default
    sensor_name_="imu_sensor";

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

Eigen::MatrixXd ImuSensorCore::getCovarianceMeasurement()
{
    Eigen::MatrixXd CovariancesMatrix;
    CovariancesMatrix.resize(this->getDimensionMeasurement(), this->getDimensionMeasurement());
    CovariancesMatrix.setZero();

    unsigned int dimension=0;
    if(this->isMeasurementLinearAccelerationEnabled())
    {
        CovariancesMatrix.block<3,3>(dimension, dimension)=this->getNoiseMeasurementLinearAcceleration();
        dimension+=3;
    }
    if(this->isMeasurementOrientationEnabled())
    {
        // TODO
        dimension+=3;
    }
    if(this->isMeasurementAngularVelocityEnabled())
    {
        CovariancesMatrix.block<3,3>(dimension, dimension)=this->getNoiseMeasurementAngularVelocity();
        dimension+=3;
    }

    return CovariancesMatrix;
}

Eigen::MatrixXd ImuSensorCore::getCovarianceParameters()
{
    Eigen::MatrixXd CovariancesMatrix;
    CovariancesMatrix.resize(this->getDimensionErrorParameters(), this->getDimensionErrorParameters());
    CovariancesMatrix.setZero();

    unsigned int dimension=0;
    if(!this->isEstimationPositionSensorWrtRobotEnabled())
    {
        CovariancesMatrix.block<3,3>(dimension, dimension)=this->getNoisePositionSensorWrtRobot();
        dimension+=3;
    }
    if(!this->isEstimationAttitudeSensorWrtRobotEnabled())
    {
        CovariancesMatrix.block<3,3>(dimension, dimension)=this->getNoiseAttitudeSensorWrtRobot();
        dimension+=3;
    }
    if(!this->isEstimationBiasLinearAccelerationEnabled())
    {
        CovariancesMatrix.block<3,3>(dimension, dimension)=this->getNoiseBiasLinearAcceleration();
        dimension+=3;
    }
    if(!this->isEstimationScaleLinearAccelerationEnabled())
    {
        CovariancesMatrix.block<3,3>(dimension, dimension)=this->getNoiseScaleLinearAcceleration();
        dimension+=3;
    }
    if(!this->isEstimationBiasAngularVelocityEnabled())
    {
        CovariancesMatrix.block<3,3>(dimension, dimension)=this->getNoiseBiasAngularVelocity();
        dimension+=3;
    }
    if(!this->isEstimationScaleAngularVelocityEnabled())
    {
        CovariancesMatrix.block<3,3>(dimension, dimension)=this->getNoiseScaleAngularVelocity();
        dimension+=3;
    }


    return CovariancesMatrix;
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
    // TODO
    // Add ka
    if(this->isEstimationBiasAngularVelocityEnabled())
    {
        this->InitErrorStateVariance.block<3,3>(point,point)=noiseBiasAngularVelocity;
    }
    //TODO
    // Add kw



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
                Eigen::Vector4d quat_predicted_angular_velocity_in_robot;
                quat_predicted_angular_velocity_in_robot[0]=0;
                quat_predicted_angular_velocity_in_robot.block<3,1>(1,0)=currentFreeModelRobotState->getAngularVelocity();
                Eigen::Vector4d quat_predicted_angular_velocity_in_imu;

                quat_predicted_angular_velocity_in_imu=Quaternion::cross(Quaternion::inv(currentImuState->getAttitudeSensorWrtRobot()), Quaternion::inv(currentFreeModelRobotState->getAttitude()), quat_predicted_angular_velocity_in_robot, currentFreeModelRobotState->getAttitude(), currentImuState->getAttitudeSensorWrtRobot());

                Eigen::Vector3d predicted_angular_velocity_in_imu;
                predicted_angular_velocity_in_imu=quat_predicted_angular_velocity_in_imu.block<3,1>(1,0);

                ThePredictedAngularVelocity=currentImuState->getScaleAngularVelocity().asDiagonal()*predicted_angular_velocity_in_imu+currentImuState->getBiasesAngularVelocity();

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


                // Angular vel
                Eigen::Vector3d angular_vel_robot_wrt_world_in_robot=Quaternion::cross_sandwich(Quaternion::inv(currentFreeModelRobotState->getAttitude()), currentFreeModelRobotState->getAngularVelocity() ,currentFreeModelRobotState->getAttitude());
                // Angular acceler
                Eigen::Vector3d angular_acc_robot_wrt_world_in_robot=Quaternion::cross_sandwich(Quaternion::inv(currentFreeModelRobotState->getAttitude()), currentFreeModelRobotState->getAngularAcceleration() ,currentFreeModelRobotState->getAttitude());

                // Ficticious Acc: Normal
                Eigen::Vector3d normal_acceleration=angular_vel_robot_wrt_world_in_robot.cross(angular_vel_robot_wrt_world_in_robot.cross(currentImuState->getPositionSensorWrtRobot()));
                // Ficticious Acc: Tang
                Eigen::Vector3d tangencial_acceleration=angular_acc_robot_wrt_world_in_robot.cross(currentImuState->getPositionSensorWrtRobot());

                // Ficticious Acc total
                Eigen::Vector3d ficticious_acceleration=normal_acceleration+tangencial_acceleration;

                // Attitude sensor wrt world
                Eigen::Vector4d attitude_sensor_wrt_world=Quaternion::cross(currentFreeModelRobotState->getAttitude(), currentImuState->getAttitudeSensorWrtRobot());

                // Acceleracion in sensor
                Eigen::Vector3d accel_sensor_wrt_sensor=Quaternion::cross_sandwich(Quaternion::inv(attitude_sensor_wrt_world), currentFreeModelRobotState->getLinearAcceleration(), attitude_sensor_wrt_world) + Quaternion::cross_sandwich(Quaternion::inv(currentImuState->getAttitudeSensorWrtRobot()), ficticious_acceleration, currentImuState->getAttitudeSensorWrtRobot());

                // Gravity in sensor
                Eigen::Vector3d gravity_sensor=Quaternion::cross_sandwich(Quaternion::inv(attitude_sensor_wrt_world), TheGlobalParametersStateCore->getGravity(), attitude_sensor_wrt_world);


                // IMU Model
                ThePredictedLinearAcceleration=currentImuState->getScaleLinearAcceleration().asDiagonal()*(accel_sensor_wrt_sensor+gravity_sensor)+currentImuState->getBiasesLinearAcceleration();

#if _DEBUG_SENSOR_CORE


                //logFile<<"ImuSensorCore::predictMeasurement() predicted quat_angular_speed_r_w_w="<<quat_angular_speed_r_w_w.transpose()<<std::endl;
                //logFile<<"ImuSensorCore::predictMeasurement() predicted quat_robot_w="<<currentFreeModelRobotState->getAttitude().transpose()<<std::endl;
                //logFile<<"ImuSensorCore::predictMeasurement() predicted quat_angular_speed_r_w_r="<<quat_angular_speed_r_w_r.transpose()<<std::endl;
                //logFile<<"ImuSensorCore::predictMeasurement() predicted p_s_r="<<currentImuState->getPositionSensorWrtRobot().transpose()<<std::endl;
                //logFile<<"ImuSensorCore::predictMeasurement() predicted quaternion_tangencial_acceleration="<<quaternion_tangencial_acceleration.transpose()<<std::endl;
                //logFile<<"ImuSensorCore::predictMeasurement() predicted quaternion_normal_acceleration="<<quaternion_normal_acceleration.transpose()<<std::endl;
                //logFile<<"ImuSensorCore::predictMeasurement() predicted ficticious_acceleration="<<ficticious_acceleration.transpose()<<std::endl;

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




int ImuSensorCore::jacobiansMeasurements(const TimeStamp theTimeStamp, std::shared_ptr<GlobalParametersStateCore> TheGlobalParametersStateCore, std::shared_ptr<RobotStateCore> TheRobotStateCore, std::shared_ptr<ImuSensorStateCore> TheImuStateCore, std::shared_ptr<ImuSensorMeasurementCore>& predictedMeasurement)
{

    // Checks
    // TODO

    //predictedMeasurement



    /// Common Variables

    // Imu sensor core
    std::shared_ptr<const ImuSensorCore> the_imu_sensor_core=std::dynamic_pointer_cast<const ImuSensorCore>(TheImuStateCore->getTheSensorCore());


    // dimension of the measurement
    int dimensionMeasurement=TheImuStateCore->getTheSensorCore()->getDimensionMeasurement();


    // Robot
    std::shared_ptr<FreeModelRobotStateCore> TheRobotStateCoreAux=std::static_pointer_cast<FreeModelRobotStateCore>(TheRobotStateCore);
    std::shared_ptr<const FreeModelRobotCore> TheRobotCoreAux=std::static_pointer_cast<const FreeModelRobotCore>(TheRobotStateCoreAux->getTheRobotCore());


    //
    Quaternion::Quaternion quat_attitude_sensor_wrt_world=Quaternion::cross(TheRobotStateCoreAux->getAttitude(), TheImuStateCore->getAttitudeSensorWrtRobot());

    Quaternion::PureQuaternion angular_velocity_imu_wrt_world_in_imu = Quaternion::cross_sandwich( Quaternion::inv(quat_attitude_sensor_wrt_world), TheRobotStateCoreAux->getAngularVelocity() , quat_attitude_sensor_wrt_world );

    //
    Eigen::Matrix4d mat_q_plus_attitude_robot_wrt_world=Quaternion::quatMatPlus(TheRobotStateCoreAux->getAttitude());
    Eigen::Matrix4d mat_q_minus_attitude_sensor_wrt_robot=Quaternion::quatMatMinus(TheImuStateCore->getAttitudeSensorWrtRobot());
    Eigen::Matrix4d mat_q_plus_attitude_world_wrt_sensor=Quaternion::quatMatPlus(Quaternion::inv(quat_attitude_sensor_wrt_world));
    Eigen::Matrix4d mat_q_minus_attitude_sensor_wrt_world=Quaternion::quatMatMinus(quat_attitude_sensor_wrt_world);
    Eigen::Matrix4d mat_q_plus_attitude_sensor_wrt_robot=Quaternion::quatMatPlus(TheImuStateCore->getAttitudeSensorWrtRobot());
    Eigen::Matrix4d mat_q_plus_attitude_robot_wrt_sensor=Quaternion::quatMatPlus(Quaternion::inv(TheImuStateCore->getAttitudeSensorWrtRobot()));
    Eigen::Matrix4d mat_q_plus_attitude_world_wrt_robot=Quaternion::quatMatPlus(Quaternion::inv(TheRobotStateCoreAux->getAttitude()));

    //
    Eigen::Matrix4d mat_q_plus_angular_velocity_robot_wrt_world_in_world = Quaternion::quatMatPlus(TheRobotStateCoreAux->getAngularVelocity());
    Eigen::Matrix4d mat_q_minus_cross_angular_vel_robot_wrt_world_in_world_and_atti_imu_wrt_world = Quaternion::quatMatMinus(Quaternion::cross_pure_gen(TheRobotStateCoreAux->getAngularVelocity(), quat_attitude_sensor_wrt_world));

    Eigen::Matrix4d mat_q_minus_cross_gravity_wrt_wolrd_and_attitude_robot_wrt_world=Quaternion::quatMatMinus(Quaternion::cross_pure_gen(TheGlobalParametersStateCore->getGravity(), TheRobotStateCoreAux->getAttitude()));
    Eigen::Matrix4d mat_q_plus_gravity_wrt_world=Quaternion::quatMatPlus(TheGlobalParametersStateCore->getGravity());

    Eigen::Matrix4d mat_q_minus_cross_acceleration_robot_wrt_world_and_attitude_robot_wrt_world=Quaternion::quatMatMinus(Quaternion::cross_pure_gen(TheRobotStateCoreAux->getLinearAcceleration(), TheRobotStateCoreAux->getAttitude()));
    Eigen::Matrix4d mat_q_plus_linear_acceleration_robot_wrt_world=Quaternion::quatMatPlus(TheRobotStateCoreAux->getLinearAcceleration());

    //
    Eigen::Matrix4d mat_diff_quat_inv_wrt_quat;
    mat_diff_quat_inv_wrt_quat<<1, 0, 0, 0,
                                0, -1, 0, 0,
                                0, 0, -1, 0,
                                0, 0, 0, -1;

    Eigen::MatrixXd mat_diff_error_quat_wrt_error_theta(4,3);
    mat_diff_error_quat_wrt_error_theta<<0, 0, 0,
                                        1, 0, 0,
                                        0, 1, 0,
                                        0, 0, 1;

    Eigen::MatrixXd mat_diff_w_amp_wrt_w(3,4);
    mat_diff_w_amp_wrt_w<<  0, 1, 0, 0,
                            0, 0, 1, 0,
                            0, 0, 0, 1;



TheImuStateCore->getScaleLinearAcceleration().asDiagonal()*mat_diff_w_amp_wrt_w*(
        // d(ga_I)/d(q_r_w)
        ( mat_q_plus_attitude_robot_wrt_sensor*mat_q_minus_attitude_sensor_wrt_world*(mat_q_minus_cross_gravity_wrt_wolrd_and_attitude_robot_wrt_world*mat_diff_quat_inv_wrt_quat + mat_q_plus_attitude_world_wrt_robot*mat_q_plus_gravity_wrt_world) ) +
        // d(a_real)/d(q_r_w)
        (mat_q_plus_attitude_robot_wrt_sensor*mat_q_minus_attitude_sensor_wrt_world*(mat_q_minus_cross_acceleration_robot_wrt_world_and_attitude_robot_wrt_world* mat_diff_quat_inv_wrt_quat + mat_q_plus_attitude_world_wrt_robot* mat_q_plus_linear_acceleration_robot_wrt_world) ) +
        // d(a_ficticious)/d(q_r_w)
        ( mat_q_plus_attitude_robot_wrt_sensor*mat_q_minus_attitude_sensor_wrt_world*( Eigen::Matrix4d::Zero(4,4) + Eigen::Matrix4d::Zero(4,4)) )
        // others
        )*mat_q_plus_attitude_robot_wrt_world*0.5*mat_diff_error_quat_wrt_error_theta;


//    Eigen::MatrixXd J2;
//    J2=TheImuStateCore->getScaleAngularVelocity().asDiagonal();

//    logFile<<"J2="<<std::endl;
//    logFile<<J2<<std::endl;


    /// Jacobians measurement - state

    /// Jacobian measurement - robot state
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
            predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementRobotErrorState.resize(dimensionMeasurement, dimensionRobotErrorState);
            predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementRobotErrorState.setZero();

            // Fill Jacobian
            // TODO
            {
                unsigned int dimension_measurement_i=0;

                // z_lin_acc
                if(this->isMeasurementLinearAccelerationEnabled())
                {
                    // z_lin_acc / posi
                    // Zeros

                    // z_lin_acc / lin_speed
                    // Zeros

                    // z_lin_acc / lin_acc
                    predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementRobotErrorState.block<3,3>(dimension_measurement_i, 6)=TheImuStateCore->getScaleLinearAcceleration().asDiagonal()*mat_diff_w_amp_wrt_w*mat_q_plus_attitude_world_wrt_sensor* mat_q_minus_attitude_sensor_wrt_robot*mat_diff_w_amp_wrt_w.transpose();


                    // z_lin_acc / attit
                    // TODO
                    predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementRobotErrorState.block<3,3>(dimension_measurement_i, 9)=TheImuStateCore->getScaleLinearAcceleration().asDiagonal()*mat_diff_w_amp_wrt_w*(
                        // d(ga_I)/d(q_r_w)
                        ( mat_q_plus_attitude_robot_wrt_sensor*mat_q_minus_attitude_sensor_wrt_robot*(mat_q_minus_cross_gravity_wrt_wolrd_and_attitude_robot_wrt_world*mat_diff_quat_inv_wrt_quat + mat_q_plus_attitude_world_wrt_robot*mat_q_plus_gravity_wrt_world) ) +
                        // d(a_real)/d(q_r_w)
                        ( mat_q_plus_attitude_robot_wrt_sensor*mat_q_minus_attitude_sensor_wrt_robot*(mat_q_minus_cross_acceleration_robot_wrt_world_and_attitude_robot_wrt_world* mat_diff_quat_inv_wrt_quat + mat_q_plus_attitude_world_wrt_robot* mat_q_plus_linear_acceleration_robot_wrt_world) ) +
                        // d(a_ficticious)/d(q_r_w)
                        ( mat_q_plus_attitude_robot_wrt_sensor*mat_q_minus_attitude_sensor_wrt_robot*( Eigen::Matrix4d::Zero(4,4) + Eigen::Matrix4d::Zero(4,4)) )
                        // others
                        )*mat_q_plus_attitude_robot_wrt_world*0.5*mat_diff_error_quat_wrt_error_theta;


                    // z_lin_acc / ang_vel
                    // TODO
                    //predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementRobotErrorState.block<3,3>(dimension_measurement_i, 12);

                    // z_lin_acc / ang_acc
                    // TODO
                    //predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementRobotErrorState.block<3,3>(dimension_measurement_i, 15);


                    dimension_measurement_i+=3;
                }

                // z_atti
                if(this->isMeasurementOrientationEnabled())
                {
                    // TODO
                    //predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementRobotErrorState;




                    dimension_measurement_i+=3;
                }

                // z_ang_vel
                if(this->isMeasurementAngularVelocityEnabled())
                {
                    // z_ang_vel / posi
                    // Zeros

                    // z_ang_vel / lin_speed
                    // Zeros

                    // z_ang_vel / lin_acc
                    // Zeros


                    // z_ang_vel / attit
                    predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementRobotErrorState.block<3,3>(dimension_measurement_i, 9)=TheImuStateCore->getScaleAngularVelocity().asDiagonal()*mat_diff_w_amp_wrt_w*( mat_q_minus_cross_angular_vel_robot_wrt_world_in_world_and_atti_imu_wrt_world*mat_diff_quat_inv_wrt_quat + mat_q_plus_attitude_world_wrt_sensor*mat_q_plus_angular_velocity_robot_wrt_world_in_world )*mat_q_minus_attitude_sensor_wrt_robot*mat_q_plus_attitude_robot_wrt_world*0.5*mat_diff_error_quat_wrt_error_theta;


                    // z_ang_vel / ang_vel
                    predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementRobotErrorState.block<3,3>(dimension_measurement_i, 12)=TheImuStateCore->getScaleAngularVelocity().asDiagonal()*mat_diff_w_amp_wrt_w* mat_q_plus_attitude_world_wrt_sensor*mat_q_minus_attitude_sensor_wrt_world *mat_diff_w_amp_wrt_w.transpose();


                    // z_ang_vel / ang_acc
                    // Zeros


                    dimension_measurement_i+=3;
                }

            }


            // end
            break;
        }
        default:
        {
            return -2;
        }

    }



    /// Jacobian measurement - sensor state & Jacobians measurement - sensor parameters

    // dimension of the sensor state
    int dimensionSensorErrorState=TheImuStateCore->getTheSensorCore()->getDimensionErrorState();

    // Resize and init Jacobian
    predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState.resize(dimensionMeasurement, dimensionSensorErrorState);
    predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState.setZero();


    // dimension of the sensor parameters
    int dimensionSensorParameters=TheImuStateCore->getTheSensorCore()->getDimensionErrorParameters();

    // Resize and init Jacobian
    predictedMeasurement->jacobianMeasurementSensorParameters.jacobianMeasurementSensorParameters.resize(dimensionMeasurement, dimensionSensorParameters);
    predictedMeasurement->jacobianMeasurementSensorParameters.jacobianMeasurementSensorParameters.setZero();



    // Fill Jacobian
    // TODO
    {
        unsigned int dimension_measurement_i=0;

        // z_lin_acc
        if(this->isMeasurementLinearAccelerationEnabled())
        {
            unsigned int dimension_error_state_sensor_i=0;
            unsigned int dimension_error_parameter_sensor_i=0;


            // z_lin_acc / posi_sen_wrt_robot
            if(the_imu_sensor_core->isEstimationPositionSensorWrtRobotEnabled())
            {
                // TODO
                //predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState.block<3,3>(dimension_measurement_i, dimension_error_state_sensor_i);
                dimension_error_state_sensor_i+=3;
            }
            else
            {
                // TODO
                //predictedMeasurement->jacobianMeasurementSensorParameters.jacobianMeasurementSensorParameters.block<3,3>(dimension_measurement_i, dimension_error_parameter_sensor_i);
                dimension_error_parameter_sensor_i+=3;
            }


            // z_lin_acc / atti_sen_wrt_robot
            if(the_imu_sensor_core->isEstimationAttitudeSensorWrtRobotEnabled())
            {
                // TODO
                //predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState.block<3,3>(dimension_measurement_i, dimension_error_state_sensor_i);
                dimension_error_state_sensor_i+=3;
            }
            else
            {
                // TODO
                //predictedMeasurement->jacobianMeasurementSensorParameters.jacobianMeasurementSensorParameters.block<3,3>(dimension_measurement_i, dimension_error_parameter_sensor_i);
                dimension_error_parameter_sensor_i+=3;
            }


            // z_lin_acc / ba
            if(the_imu_sensor_core->isEstimationBiasLinearAccelerationEnabled())
            {
                predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState.block<3,3>(dimension_measurement_i, dimension_error_state_sensor_i)=Eigen::Matrix3d::Identity(3,3);
                dimension_error_state_sensor_i+=3;
            }
            else
            {
                predictedMeasurement->jacobianMeasurementSensorParameters.jacobianMeasurementSensorParameters.block<3,3>(dimension_measurement_i, dimension_error_parameter_sensor_i)=Eigen::Matrix3d::Identity(3,3);
                dimension_error_parameter_sensor_i+=3;
            }


            // z_lin_acc / ka
            if(the_imu_sensor_core->isEstimationScaleLinearAccelerationEnabled())
            {
                // TODO
                //predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState.block<3,3>(dimension_measurement_i, dimension_error_state_sensor_i);
                dimension_error_state_sensor_i+=3;
            }
            else
            {
                // TODO
                //predictedMeasurement->jacobianMeasurementSensorParameters.jacobianMeasurementSensorParameters.block<3,3>(dimension_measurement_i, dimension_error_parameter_sensor_i);
                dimension_error_parameter_sensor_i+=3;
            }


            // z_lin_acc / bw
            if(the_imu_sensor_core->isEstimationBiasAngularVelocityEnabled())
            {
                // Zeros
                dimension_error_state_sensor_i+=3;
            }
            else
            {
                // Zeros
                dimension_error_parameter_sensor_i+=3;
            }


            // z_lin_acc / kw
            if(the_imu_sensor_core->isEstimationScaleAngularVelocityEnabled())
            {
                // Zeros
                dimension_error_state_sensor_i+=3;
            }
            else
            {
                // Zeros
                dimension_error_parameter_sensor_i+=3;
            }


            dimension_measurement_i+=3;
        }

        // z_atti
        if(this->isMeasurementOrientationEnabled())
        {

            unsigned int dimension_error_state_sensor_i=0;
            unsigned int dimension_error_parameter_sensor_i=0;


            // z_atti / posi_sen_wrt_robot
            if(the_imu_sensor_core->isEstimationPositionSensorWrtRobotEnabled())
            {
                // TODO
                //predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState.block<3,3>(dimension_measurement_i, dimension_error_state_sensor_i);
                dimension_error_state_sensor_i+=3;
            }
            else
            {
                // TODO
                //predictedMeasurement->jacobianMeasurementSensorParameters.jacobianMeasurementSensorParameters.block<3,3>(dimension_measurement_i, dimension_error_parameter_sensor_i);
                dimension_error_parameter_sensor_i+=3;
            }


            // z_atti / atti_sen_wrt_robot
            if(the_imu_sensor_core->isEstimationAttitudeSensorWrtRobotEnabled())
            {
                // TODO
                //predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState.block<3,3>(dimension_measurement_i, dimension_error_state_sensor_i);
                dimension_error_state_sensor_i+=3;
            }
            else
            {
                // TODO
                //predictedMeasurement->jacobianMeasurementSensorParameters.jacobianMeasurementSensorParameters.block<3,3>(dimension_measurement_i, dimension_error_parameter_sensor_i);
                dimension_error_parameter_sensor_i+=3;
            }


            // z_atti / ba
            if(the_imu_sensor_core->isEstimationBiasLinearAccelerationEnabled())
            {
                // Zeros
                dimension_error_state_sensor_i+=3;
            }
            else
            {
                // Zeros
                dimension_error_parameter_sensor_i+=3;
            }


            // z_atti / ka
            if(the_imu_sensor_core->isEstimationScaleLinearAccelerationEnabled())
            {
                // Zeros
                dimension_error_state_sensor_i+=3;
            }
            else
            {
                // Zeros
                dimension_error_parameter_sensor_i+=3;
            }


            // z_atti / bw
            if(the_imu_sensor_core->isEstimationBiasAngularVelocityEnabled())
            {
                // Zeros
                dimension_error_state_sensor_i+=3;
            }
            else
            {
                // Zeros
                dimension_error_parameter_sensor_i+=3;
            }


            // z_atti / kw
            if(the_imu_sensor_core->isEstimationScaleAngularVelocityEnabled())
            {
                // Zeros
                dimension_error_state_sensor_i+=3;
            }
            else
            {
                // Zeros
                dimension_error_parameter_sensor_i+=3;
            }


            dimension_measurement_i+=3;
        }

        // z_ang_vel
        if(this->isMeasurementAngularVelocityEnabled())
        {
            unsigned int dimension_error_state_sensor_i=0;
            unsigned int dimension_error_parameter_sensor_i=0;


            // z_ang_vel / posi_sen_wrt_robot
            if(the_imu_sensor_core->isEstimationPositionSensorWrtRobotEnabled())
            {
                // Zeros
                dimension_error_state_sensor_i+=3;
            }
            else
            {
                // Zeros
                dimension_error_parameter_sensor_i+=3;
            }


            // z_ang_vel / atti_sen_wrt_robot
            if(the_imu_sensor_core->isEstimationAttitudeSensorWrtRobotEnabled())
            {
                predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState.block<3,3>(dimension_measurement_i, dimension_error_state_sensor_i)=TheImuStateCore->getScaleAngularVelocity().asDiagonal()*mat_diff_w_amp_wrt_w* ( mat_q_minus_cross_angular_vel_robot_wrt_world_in_world_and_atti_imu_wrt_world*mat_diff_quat_inv_wrt_quat + mat_q_plus_attitude_world_wrt_sensor*mat_q_plus_angular_velocity_robot_wrt_world_in_world ) *mat_q_plus_attitude_robot_wrt_world*mat_q_plus_attitude_sensor_wrt_robot*0.5*mat_diff_error_quat_wrt_error_theta;
                dimension_error_state_sensor_i+=3;
            }
            else
            {
                predictedMeasurement->jacobianMeasurementSensorParameters.jacobianMeasurementSensorParameters.block<3,3>(dimension_measurement_i, dimension_error_parameter_sensor_i)=TheImuStateCore->getScaleAngularVelocity().asDiagonal()*mat_diff_w_amp_wrt_w* ( mat_q_minus_cross_angular_vel_robot_wrt_world_in_world_and_atti_imu_wrt_world*mat_diff_quat_inv_wrt_quat + mat_q_plus_attitude_world_wrt_sensor*mat_q_plus_angular_velocity_robot_wrt_world_in_world ) *mat_q_plus_attitude_robot_wrt_world*mat_q_plus_attitude_sensor_wrt_robot*0.5*mat_diff_error_quat_wrt_error_theta;
                dimension_error_parameter_sensor_i+=3;
            }


            // z_ang_vel / ba
            if(the_imu_sensor_core->isEstimationBiasLinearAccelerationEnabled())
            {
                // Zeros
                dimension_error_state_sensor_i+=3;
            }
            else
            {
                // Zeros
                dimension_error_parameter_sensor_i+=3;
            }


            // z_ang_vel / ka
            if(the_imu_sensor_core->isEstimationScaleLinearAccelerationEnabled())
            {
                // Zeros
                dimension_error_state_sensor_i+=3;
            }
            else
            {
                // Zeros
                dimension_error_parameter_sensor_i+=3;
            }


            // z_ang_vel / bw
            if(the_imu_sensor_core->isEstimationBiasAngularVelocityEnabled())
            {
                predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState.block<3,3>(dimension_measurement_i, dimension_error_state_sensor_i)=Eigen::Matrix3d::Identity(3,3);
                dimension_error_state_sensor_i+=3;
            }
            else
            {
                predictedMeasurement->jacobianMeasurementSensorParameters.jacobianMeasurementSensorParameters.block<3,3>(dimension_measurement_i, dimension_error_parameter_sensor_i)=Eigen::Matrix3d::Identity(3,3);
                dimension_error_parameter_sensor_i+=3;
            }


            // z_ang_vel / kw
            if(the_imu_sensor_core->isEstimationScaleAngularVelocityEnabled())
            {
                predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState.block<3,3>(dimension_measurement_i, dimension_error_state_sensor_i)=angular_velocity_imu_wrt_world_in_imu.asDiagonal();
                dimension_error_state_sensor_i+=3;
            }
            else
            {
                predictedMeasurement->jacobianMeasurementSensorParameters.jacobianMeasurementSensorParameters.block<3,3>(dimension_measurement_i, dimension_error_parameter_sensor_i)=angular_velocity_imu_wrt_world_in_imu.asDiagonal();
                dimension_error_parameter_sensor_i+=3;
            }


            dimension_measurement_i+=3;
        }

    }




    /// Jacobians measurement - global parameters

    // dimension of the global parameters
    int dimensionGlobalParameters=TheGlobalParametersStateCore->getTheGlobalParametersCore()->getDimensionErrorParameters();

    // Resize and init Jacobian
    predictedMeasurement->jacobianMeasurementGlobalParameters.jacobianMeasurementGlobalParameters.resize(dimensionMeasurement, dimensionGlobalParameters);
    predictedMeasurement->jacobianMeasurementGlobalParameters.jacobianMeasurementGlobalParameters.setZero();

    // Fill Jacobian
    {
        unsigned int dimension_measurement_i=0;

        // z_lin_acc
        if(this->isMeasurementLinearAccelerationEnabled())
        {
            // z_lin_acc / grav
            predictedMeasurement->jacobianMeasurementGlobalParameters.jacobianMeasurementGlobalParameters.block<3,3>(dimension_measurement_i,0)=TheImuStateCore->getScaleLinearAcceleration().asDiagonal()*mat_diff_w_amp_wrt_w*mat_q_plus_attitude_world_wrt_sensor* mat_q_minus_attitude_sensor_wrt_robot*mat_diff_w_amp_wrt_w.transpose();

            dimension_measurement_i+=3;
        }

        // z_atti
        if(this->isMeasurementOrientationEnabled())
        {
            // z_atti / grav
            // Zeros

            dimension_measurement_i+=3;
        }

        // z_ang_vel
        if(this->isMeasurementAngularVelocityEnabled())
        {
            // z_ang_vel / grav
            // Zeros

            dimension_measurement_i+=3;
        }


    }


    /// Jacobians measurement - sensor noise of the measurement

    // Resize and init Jacobian
    predictedMeasurement->jacobianMeasurementSensorNoise.jacobianMeasurementSensorNoise.resize(dimensionMeasurement, dimensionMeasurement);
    predictedMeasurement->jacobianMeasurementSensorNoise.jacobianMeasurementSensorNoise.setZero();

    // Fill Jacobian
    predictedMeasurement->jacobianMeasurementSensorNoise.jacobianMeasurementSensorNoise=Eigen::MatrixXd::Identity(dimensionMeasurement, dimensionMeasurement);



    // LOG
#if _DEBUG_SENSOR_CORE
    {
        logFile<<"ImuSensorCore::jacobiansMeasurements() TS: sec="<<theTimeStamp.sec<<" s; nsec="<<theTimeStamp.nsec<<" ns"<<std::endl;

        logFile<<"predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementRobotErrorState="<<std::endl;
        logFile<<predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementRobotErrorState<<std::endl;

        logFile<<"predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState="<<std::endl;
        logFile<<predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState<<std::endl;

        logFile<<"predictedMeasurement->jacobianMeasurementSensorParameters.jacobianMeasurementSensorParameters="<<std::endl;
        logFile<<predictedMeasurement->jacobianMeasurementSensorParameters.jacobianMeasurementSensorParameters<<std::endl;

        logFile<<"predictedMeasurement->jacobianMeasurementGlobalParameters.jacobianMeasurementGlobalParameters="<<std::endl;
        logFile<<predictedMeasurement->jacobianMeasurementGlobalParameters.jacobianMeasurementGlobalParameters<<std::endl;

        logFile<<"predictedMeasurement->jacobianMeasurementSensorNoise.jacobianMeasurementSensorNoise="<<std::endl;
        logFile<<predictedMeasurement->jacobianMeasurementSensorNoise.jacobianMeasurementSensorNoise<<std::endl;
    }
#endif



    // End
    return 0;
}
