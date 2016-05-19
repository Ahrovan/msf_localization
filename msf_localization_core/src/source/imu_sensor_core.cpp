
#include "msf_localization_core/imu_sensor_core.h"

// Circular Dependency
#include "msf_localization_core/msf_storage_core.h"


ImuSensorCore::ImuSensorCore() :
    SensorCore()
{
    init();

    return;
}

ImuSensorCore::ImuSensorCore(std::weak_ptr<MsfStorageCore> the_msf_storage_core) :
    SensorCore(the_msf_storage_core)
{
    //std::cout<<"ImuSensorCore::ImuSensorCore(std::weak_ptr<MsfStorageCore> the_msf_storage_core)"<<std::endl;

    init();

    return;
}

ImuSensorCore::~ImuSensorCore()
{
    return;
}

int ImuSensorCore::init()
{
    // Sensor Type
    setSensorType(SensorTypes::imu);


    // Sensor name -> Default
    //sensor_name_="imu_sensor";

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
    dimension_error_state_=0;
    dimension_state_=0;

    // Parameters
    dimension_parameters_+=3+3; // TODO Sensitivities
    dimension_error_parameters_+=3+3; // TODO Sensitivities

    // Dimension measurements -> Again just in case
    dimension_measurement_=0;
    dimension_error_measurement_=0;

    // Dimension noise -> Again just in case
    dimension_noise_=0;

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

    return 0;
}

int ImuSensorCore::readConfig(const pugi::xml_node& sensor, const unsigned int sensorId, std::shared_ptr<ImuSensorStateCore>& SensorInitStateCore)
{

    // Create a class for the SensorStateCore
    if(!SensorInitStateCore)
        SensorInitStateCore=std::make_shared<ImuSensorStateCore>(this->getMsfElementCoreWeakPtr());


    // Set Id
    this->setSensorId(sensorId);


    // Auxiliar reading value
    std::string readingValue;


    //// Sensor configurations


    /// Name
    std::string sensor_name=sensor.child_value("name");
    this->setSensorName(sensor_name);


    /// Pose of the sensor wrt robot
    pugi::xml_node pose_in_robot=sensor.child("pose_in_robot");

    // Position of the sensor wrt robot
    readingValue=pose_in_robot.child("position").child_value("enabled");
    if(std::stoi(readingValue))
        this->enableEstimationPositionSensorWrtRobot();

    // Attitude of the sensor wrt robot
    readingValue=pose_in_robot.child("attitude").child_value("enabled");
    if(std::stoi(readingValue))
        this->enableEstimationAttitudeSensorWrtRobot();


    /// Other Parameters
    pugi::xml_node parameters = sensor.child("parameters");

    // Angular Velocity
    pugi::xml_node param_angular_velocity = parameters.child("angular_velocity");

    // Angular Velocity Biases
    readingValue=param_angular_velocity.child("biases").child_value("enabled");
    if(std::stoi(readingValue))
    {
        this->enableEstimationBiasAngularVelocity();
    }

    // Angular Velocity Scale
    readingValue=param_angular_velocity.child("scale").child_value("enabled");
    if(std::stoi(readingValue))
    {
        this->enableEstimationScaleAngularVelocity();
    }

    // Linear Acceleration
    pugi::xml_node param_linear_acceleration = parameters.child("linear_acceleration");

    // Linear Acceleration Biases
    readingValue=param_linear_acceleration.child("biases").child_value("enabled");
    if(std::stoi(readingValue))
        this->enableEstimationBiasLinearAcceleration();

    // Linear Acceleration Scale
    readingValue=param_linear_acceleration.child("scale").child_value("enabled");
    if(std::stoi(readingValue))
        this->enableEstimationScaleLinearAcceleration();



    //// Measurements
    pugi::xml_node measurements = sensor.child("measurements");


    /// Linear Acceleration
    pugi::xml_node meas_linear_acceleration = measurements.child("linear_acceleration");

    readingValue=meas_linear_acceleration.child_value("enabled");
    if(std::stoi(readingValue))
        this->enableMeasurementLinearAcceleration();

    readingValue=meas_linear_acceleration.child_value("var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoiseMeasurementLinearAcceleration(variance.asDiagonal());
    }


    /// Orientation
    pugi::xml_node orientation = measurements.child("orientation");

    readingValue=orientation.child_value("enabled");
    // TODO

    readingValue=orientation.child_value("var");
    // TODO


    /// Angular Velocity
    pugi::xml_node meas_angular_velocity = measurements.child("angular_velocity");

    readingValue=meas_angular_velocity.child_value("enabled");
    if(std::stoi(readingValue))
        this->enableMeasurementAngularVelocity();

    readingValue=meas_angular_velocity.child_value("var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoiseMeasurementAngularVelocity(variance.asDiagonal());
    }


    //// Init State

    /// Pose of the sensor wrt robot

    // Position of the sensor wrt robot
    readingValue=pose_in_robot.child("position").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d init_estimation;
        stm>>init_estimation[0]>>init_estimation[1]>>init_estimation[2];
        SensorInitStateCore->setPositionSensorWrtRobot(init_estimation);
    }

    // Attitude of the sensor wrt robot
    readingValue=pose_in_robot.child("attitude").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector4d init_estimation;
        stm>>init_estimation[0]>>init_estimation[1]>>init_estimation[2]>>init_estimation[3];
        SensorInitStateCore->setAttitudeSensorWrtRobot(init_estimation);
    }


    /// Parameters

    // Bias Angular Velocity
    readingValue=param_angular_velocity.child("biases").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d init_estimation;
        stm>>init_estimation[0]>>init_estimation[1]>>init_estimation[2];
        SensorInitStateCore->setBiasesAngularVelocity(init_estimation);
    }

    // Scale Angular Velocity
    readingValue=param_angular_velocity.child("scale").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d init_estimation;
        stm>>init_estimation[0]>>init_estimation[1]>>init_estimation[2];
        SensorInitStateCore->setScaleAngularVelocity(init_estimation);
    }

    // Bias Linear Acceleration
    readingValue=param_linear_acceleration.child("biases").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d init_estimation;
        stm>>init_estimation[0]>>init_estimation[1]>>init_estimation[2];
        SensorInitStateCore->setBiasesLinearAcceleration(init_estimation);
    }

    // Scale Linear Acceleration
    readingValue=param_linear_acceleration.child("scale").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d init_estimation;
        stm>>init_estimation[0]>>init_estimation[1]>>init_estimation[2];
        SensorInitStateCore->setScaleLinearAcceleration(init_estimation);
    }


    //// Init Variances

    /// Pose of the sensor wrt robot

    // Position of the sensor wrt robot
    readingValue=pose_in_robot.child("position").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoisePositionSensorWrtRobot(variance.asDiagonal());
    }

    // Attitude of the sensor wrt robot
    readingValue=pose_in_robot.child("attitude").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoiseAttitudeSensorWrtRobot(variance.asDiagonal());
    }


    /// Other Parameters

    // Bias Linear Acceleration
    readingValue=param_linear_acceleration.child("biases").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoiseBiasLinearAcceleration(variance.asDiagonal());
    }

    // Scale Linear Acceleration
    readingValue=param_linear_acceleration.child("scale").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoiseScaleLinearAcceleration(variance.asDiagonal());
    }

    // Bias Angular Velocity
    readingValue=param_angular_velocity.child("biases").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoiseBiasAngularVelocity(variance.asDiagonal());
    }

    // Scale Angular Velocity
    readingValue=param_angular_velocity.child("scale").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoiseScaleAngularVelocity(variance.asDiagonal());
    }


    // Noises in the estimation (if enabled)

    // Bias Linear Acceleration
    if(this->isEstimationBiasLinearAccelerationEnabled())
    {
        readingValue=param_linear_acceleration.child("biases").child_value("noise");
        {
            std::istringstream stm(readingValue);
            Eigen::Vector3d variance;
            stm>>variance[0]>>variance[1]>>variance[2];
            this->setNoiseEstimationBiasLinearAcceleration(variance.asDiagonal());
        }
    }

    // Sensitivity
    // TODO

    // Bias Angular Velocity
    if(this->isEstimationBiasAngularVelocityEnabled())
    {
        readingValue=param_angular_velocity.child("biases").child_value("noise");
        {
            std::istringstream stm(readingValue);
            Eigen::Vector3d variance;
            stm>>variance[0]>>variance[1]>>variance[2];
            this->setNoiseEstimationBiasAngularVelocity(variance.asDiagonal());
        }
    }

    // Sensitivity
    // TODO


    // Prepare covariance matrix
    this->prepareCovarianceInitErrorState();


    /// Finish

    // End
    return 0;
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
        this->dimension_measurement_+=4;
        dimension_error_measurement_+=3;
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
        this->dimension_measurement_+=3;
        dimension_error_measurement_+=3;
    }
    return 0;
}

Eigen::Matrix3d ImuSensorCore::getNoiseMeasurementAngularVelocity() const
{
    return this->noiseMeasurementAngularVelocity;
}

int ImuSensorCore::setNoiseMeasurementAngularVelocity(const Eigen::Matrix3d &noiseMeasurementAngularVelocity)
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
        this->dimension_measurement_+=3;
        dimension_error_measurement_+=3;
    }
    return 0;
}

Eigen::Matrix3d ImuSensorCore::getNoiseMeasurementLinearAcceleration() const
{
    return this->noiseMeasurementLinearAcceleration;
}

int ImuSensorCore::setNoiseMeasurementLinearAcceleration(const Eigen::Matrix3d &noiseMeasurementLinearAcceleration)
{
    this->noiseMeasurementLinearAcceleration=noiseMeasurementLinearAcceleration;

    return 0;
}


int ImuSensorCore::setMeasurement(const TimeStamp& TheTimeStamp, const std::shared_ptr<ImuSensorMeasurementCore> TheImuSensorMeasurement)
{
    //std::cout<<"Imu Measurement Set"<<std::endl;

    if(!isSensorEnabled())
        return 0;

    if(!this->isCorrect())
        std::cout<<"ERROR"<<std::endl;

    //std::shared_ptr<MsfStorageCore> TheMsfStorageCoreAux=this->TheMsfStorageCore.lock();
//    if(!TheMsfStorageCoreAux)
//        std::cout<<"Unable to lock TheMsfStorageCore"<<std::endl;

    this->getMsfStorageCoreSharedPtr()->setMeasurement(TheTimeStamp, TheImuSensorMeasurement);

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
        this->dimension_state_+=3;
        // Update Error State Dimension
        this->dimension_error_state_+=3;
        //
        this->dimension_parameters_-=3;
        //
        this->dimension_error_parameters_-=3;
        //
        this->dimension_noise_+=3;
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
        this->dimension_state_-=3;
        // Update Error State Dimension
        this->dimension_error_state_-=3;
        //
        this->dimension_parameters_+=3;
        //
        this->dimension_error_parameters_+=3;
        //
        this->dimension_noise_-=3;
    }
    return 0;
}

Eigen::Matrix3d ImuSensorCore::getNoiseBiasAngularVelocity() const
{
    return this->noiseBiasAngularVelocity;
}

int ImuSensorCore::setNoiseBiasAngularVelocity(const Eigen::Matrix3d &noiseBiasAngularVelocity)
{
    this->noiseBiasAngularVelocity=noiseBiasAngularVelocity;
    return 0;
}


// Angular Velocity biases: Estimation Covariance (if enabled estimation)
Eigen::Matrix3d ImuSensorCore::getNoiseEstimationBiasAngularVelocity() const
{
    return this->noiseEstimationBiasAngularVelocity;
}

int ImuSensorCore::setNoiseEstimationBiasAngularVelocity(const Eigen::Matrix3d& noiseEstimationBiasAngularVelocity)
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
        this->dimension_state_+=3;
        // Update Error State Dimension
        this->dimension_error_state_+=3;
        //
        this->dimension_parameters_-=3;
        //
        this->dimension_error_parameters_-=3;
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
        this->dimension_state_-=3;
        // Update Error State Dimension
        this->dimension_error_state_-=3;
        //
        this->dimension_parameters_+=3;
        //
        this->dimension_error_parameters_+=3;
    }
    return 0;
}

Eigen::Matrix3d ImuSensorCore::getNoiseScaleAngularVelocity() const
{
    return this->noiseScaleAngularVelocity;
}

int ImuSensorCore::setNoiseScaleAngularVelocity(const Eigen::Matrix3d &noiseScaleAngularVelocity)
{
    this->noiseScaleAngularVelocity=noiseScaleAngularVelocity;
    return 0;
}


bool ImuSensorCore::isEstimationSensitivityAngularVelocityEnabled() const
{
    return this->flagEstimationSensitivityAngularVelocity;
}

int ImuSensorCore::enableEstimationSensitivityAngularVelocity()
{
    if(!this->flagEstimationSensitivityAngularVelocity)
    {
        // Enable
        this->flagEstimationSensitivityAngularVelocity=true;
        // Update State Dimension
        this->dimension_state_+=9;
        // Update Error State Dimension
        this->dimension_error_state_+=9;
        //
        this->dimension_parameters_-=9;
        //
        this->dimension_error_parameters_-=9;
    }
    return 0;
}

int ImuSensorCore::enableParameterSensitivityAngularVelocity()
{
    if(this->flagEstimationSensitivityAngularVelocity)
    {
        // Enable
        this->flagEstimationSensitivityAngularVelocity=false;
        // Update State Dimension
        this->dimension_state_-=9;
        // Update Error State Dimension
        this->dimension_error_state_-=9;
        //
        this->dimension_parameters_+=9;
        //
        this->dimension_error_parameters_+=9;
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
        this->dimension_state_+=3;
        // Update Error State Dimension
        this->dimension_error_state_+=3;
        //
        this->dimension_parameters_-=3;
        //
        this->dimension_error_parameters_-=3;
        //
        this->dimension_noise_+=3;
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
        this->dimension_state_-=3;
        // Update Error State Dimension
        this->dimension_error_state_-=3;
        //
        this->dimension_parameters_+=3;
        //
        this->dimension_error_parameters_+=3;
        //
        this->dimension_noise_-=3;
    }
    return 0;
}

Eigen::Matrix3d ImuSensorCore::getNoiseBiasLinearAcceleration() const
{
    return this->noiseBiasLinearAcceleration;
}

int ImuSensorCore::setNoiseBiasLinearAcceleration(const Eigen::Matrix3d &noiseBiasLinearAcceleration)
{
    this->noiseBiasLinearAcceleration=noiseBiasLinearAcceleration;
    return 0;
}

Eigen::Matrix3d ImuSensorCore::getNoiseEstimationBiasLinearAcceleration() const
{
    return this->noiseEstimationBiasLinearAcceleration;
}

int ImuSensorCore::setNoiseEstimationBiasLinearAcceleration(const Eigen::Matrix3d &noiseEstimationBiasLinearAcceleration)
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
        this->dimension_state_+=3;
        // Update Error State Dimension
        this->dimension_error_state_+=3;
        //
        this->dimension_parameters_-=3;
        //
        this->dimension_error_parameters_-=3;
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
        this->dimension_state_-=3;
        // Update Error State Dimension
        this->dimension_error_state_-=3;
        //
        this->dimension_parameters_+=3;
        //
        this->dimension_error_parameters_+=3;
    }
    return 0;
}

Eigen::Matrix3d ImuSensorCore::getNoiseScaleLinearAcceleration() const
{
    return this->noiseScaleLinearAcceleration;
}

int ImuSensorCore::setNoiseScaleLinearAcceleration(const Eigen::Matrix3d &noiseScaleLinearAcceleration)
{
    this->noiseScaleLinearAcceleration=noiseScaleLinearAcceleration;
    return 0;
}

Eigen::SparseMatrix<double> ImuSensorCore::getCovarianceMeasurement()
{
    Eigen::SparseMatrix<double> CovariancesMatrix;

    CovariancesMatrix.resize(this->getDimensionErrorMeasurement(), this->getDimensionErrorMeasurement());
    //CovariancesMatrix.setZero();
    CovariancesMatrix.reserve(this->getDimensionErrorMeasurement());

    std::vector<Eigen::Triplet<double> > tripletCovarianceMeasurement;

    unsigned int dimension=0;
    if(this->isMeasurementLinearAccelerationEnabled())
    {
        //CovariancesMatrix.block<3,3>(dimension, dimension)=this->getNoiseMeasurementLinearAcceleration();

        for(int i=0; i<3; i++)
            tripletCovarianceMeasurement.push_back(Eigen::Triplet<double>(dimension+i,dimension+i,noiseMeasurementLinearAcceleration(i,i)));

        dimension+=3;
    }
    if(this->isMeasurementOrientationEnabled())
    {
        // TODO
        dimension+=3;
    }
    if(this->isMeasurementAngularVelocityEnabled())
    {
        //CovariancesMatrix.block<3,3>(dimension, dimension)=this->getNoiseMeasurementAngularVelocity();

        for(int i=0; i<3; i++)
            tripletCovarianceMeasurement.push_back(Eigen::Triplet<double>(dimension+i,dimension+i,noiseMeasurementAngularVelocity(i,i)));

        dimension+=3;
    }

    CovariancesMatrix.setFromTriplets(tripletCovarianceMeasurement.begin(), tripletCovarianceMeasurement.end());


//    std::cout<<"CovariancesMatrix="<<std::endl;
//    std::cout<<Eigen::MatrixXd(CovariancesMatrix)<<std::endl;


    return CovariancesMatrix;
}

Eigen::SparseMatrix<double> ImuSensorCore::getCovarianceParameters()
{
    Eigen::SparseMatrix<double> CovariancesMatrix;

    CovariancesMatrix.resize(this->getDimensionErrorParameters(), this->getDimensionErrorParameters());
    //CovariancesMatrix.setZero();
    CovariancesMatrix.reserve(this->getDimensionErrorParameters());

    std::vector<Eigen::Triplet<double> > tripletCovarianceParameters;


    unsigned int dimension=0;
    if(!this->isEstimationPositionSensorWrtRobotEnabled())
    {
        //CovariancesMatrix.block<3,3>(dimension, dimension)=this->getNoisePositionSensorWrtRobot();

        for(int i=0; i<3; i++)
            tripletCovarianceParameters.push_back(Eigen::Triplet<double>(dimension+i,dimension+i,noisePositionSensorWrtRobot(i,i)));


        dimension+=3;
    }
    if(!this->isEstimationAttitudeSensorWrtRobotEnabled())
    {
        //CovariancesMatrix.block<3,3>(dimension, dimension)=this->getNoiseAttitudeSensorWrtRobot();

        for(int i=0; i<3; i++)
            tripletCovarianceParameters.push_back(Eigen::Triplet<double>(dimension+i,dimension+i,noiseAttitudeSensorWrtRobot(i,i)));

        dimension+=3;
    }
    if(!this->isEstimationBiasLinearAccelerationEnabled())
    {
        //CovariancesMatrix.block<3,3>(dimension, dimension)=this->getNoiseBiasLinearAcceleration();

        for(int i=0; i<3; i++)
            tripletCovarianceParameters.push_back(Eigen::Triplet<double>(dimension+i,dimension+i,noiseBiasLinearAcceleration(i,i)));

        dimension+=3;
    }
    /*
    // TODO
    if(!this->isEstimationScaleLinearAccelerationEnabled())
    {
        //CovariancesMatrix.block<3,3>(dimension, dimension)=this->getNoiseScaleLinearAcceleration();

        for(int i=0; i<3; i++)
            tripletCovarianceParameters.push_back(Eigen::Triplet<double>(dimension+i,dimension+i,noiseScaleLinearAcceleration(i,i)));

        dimension+=3;
    }
    */
    if(!this->isEstimationBiasAngularVelocityEnabled())
    {
        //CovariancesMatrix.block<3,3>(dimension, dimension)=this->getNoiseBiasAngularVelocity();

        for(int i=0; i<3; i++)
            tripletCovarianceParameters.push_back(Eigen::Triplet<double>(dimension+i,dimension+i,noiseBiasAngularVelocity(i,i)));

        dimension+=3;
    }
    /*
    // TODO
    if(!this->isEstimationScaleAngularVelocityEnabled())
    {
        //CovariancesMatrix.block<3,3>(dimension, dimension)=this->getNoiseScaleAngularVelocity();

        for(int i=0; i<3; i++)
            tripletCovarianceParameters.push_back(Eigen::Triplet<double>(dimension+i,dimension+i,noiseScaleAngularVelocity(i,i)));

        dimension+=3;
    }
    */

    CovariancesMatrix.setFromTriplets(tripletCovarianceParameters.begin(), tripletCovarianceParameters.end());


    return CovariancesMatrix;
}

int ImuSensorCore::prepareCovarianceInitErrorStateSpecific()
{
    int point=0;
    if(this->isEstimationPositionSensorWrtRobotEnabled())
    {
        this->covariance_init_error_state_.block<3,3>(point,point)=noisePositionSensorWrtRobot;
        point+=3;
    }
    if(this->isEstimationAttitudeSensorWrtRobotEnabled())
    {
        this->covariance_init_error_state_.block<3,3>(point,point)=noiseAttitudeSensorWrtRobot;
        point+=3;
    }
    if(this->isEstimationBiasLinearAccelerationEnabled())
    {
        this->covariance_init_error_state_.block<3,3>(point,point)=noiseBiasLinearAcceleration;
        point+=3;
    }
    // TODO
    // Add ka
    if(this->isEstimationBiasAngularVelocityEnabled())
    {
        this->covariance_init_error_state_.block<3,3>(point,point)=noiseBiasAngularVelocity;
    }
    // TODO
    // Add kw


    return 0;
}


Eigen::SparseMatrix<double> ImuSensorCore::getCovarianceNoise(const TimeStamp deltaTimeStamp)
{
    Eigen::SparseMatrix<double> covariance_noise;

    // Dimension noise
    int dimension_noise=getDimensionNoise();


    // Resize
    covariance_noise.resize(dimension_noise, dimension_noise);
    //covariance_noise.setZero();
    covariance_noise.reserve(dimension_noise);

    std::vector<Eigen::Triplet<double> > tripletCovarianceNoise;

    // dt
    double dt=deltaTimeStamp.get_double();

    // Fill
    int dimension_noise_i=0;
    if(isEstimationBiasLinearAccelerationEnabled())
    {
        //covariance_noise.block<3,3>(dimension_noise_i,dimension_noise_i)=this->noiseEstimationBiasLinearAcceleration*deltaTimeStamp.get_double();

        for(int i=0; i<3; i++)
            tripletCovarianceNoise.push_back(Eigen::Triplet<double>(dimension_noise_i+i,dimension_noise_i+i,noiseEstimationBiasLinearAcceleration(i,i)*dt));


        dimension_noise_i+=3;
    }

    if(isEstimationBiasAngularVelocityEnabled())
    {
        //covariance_noise.block<3,3>(dimension_noise_i,dimension_noise_i)=this->noiseEstimationBiasAngularVelocity*deltaTimeStamp.get_double();

        for(int i=0; i<3; i++)
            tripletCovarianceNoise.push_back(Eigen::Triplet<double>(dimension_noise_i+i,dimension_noise_i+i,noiseEstimationBiasAngularVelocity(i,i)*dt));


        dimension_noise_i+=3;
    }


    covariance_noise.setFromTriplets(tripletCovarianceNoise.begin(), tripletCovarianceNoise.end());


//    std::cout<<"covariance_noise"<<std::endl;
//    std::cout<<Eigen::MatrixXd(covariance_noise)<<std::endl;


    // End
    return covariance_noise;
}

int ImuSensorCore::predictState(//Time
                 const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp,
                 // Previous State
                 const std::shared_ptr<StateEstimationCore> pastState,
                 // Inputs
                 const std::shared_ptr<InputCommandComponent> inputCommand,
                 // Predicted State
                 std::shared_ptr<StateCore> &predictedState)
{
    // Checks

    // Past State
    if(!pastState)
        return -1;

    // TODO

    // Search for the past sensor State Core
    std::shared_ptr<ImuSensorStateCore> past_sensor_state;

    for(std::list< std::shared_ptr<StateCore> >::iterator it_sensor_state=pastState->TheListSensorStateCore.begin();
        it_sensor_state!=pastState->TheListSensorStateCore.end();
        ++it_sensor_state)
    {
        if((*it_sensor_state)->getMsfElementCoreSharedPtr() == this->getMsfElementCoreSharedPtr())
        {
            past_sensor_state=std::dynamic_pointer_cast<ImuSensorStateCore>(*it_sensor_state);
            break;
        }
    }
    if(!past_sensor_state)
        return -10;


    // Predicted State
    std::shared_ptr<ImuSensorStateCore> predicted_sensor_state;
    // Create the prediction if it does not exist
    if(!predicted_sensor_state)
        predicted_sensor_state=std::make_shared<ImuSensorStateCore>(past_sensor_state->getMsfElementCoreWeakPtr());
    else
        predicted_sensor_state=std::dynamic_pointer_cast<ImuSensorStateCore>(predictedState);



    // Predict State
    int error_predict_state=predictStateSpecific(previousTimeStamp, currentTimeStamp,
                                         past_sensor_state,
                                         predicted_sensor_state);

    // Check error
    if(error_predict_state)
        return error_predict_state;


    // Set predicted state
    predictedState=predicted_sensor_state;


    // End
    return 0;
}



int ImuSensorCore::predictStateSpecific(const TimeStamp &previousTimeStamp, const TimeStamp &currentTimeStamp,
                                        const std::shared_ptr<ImuSensorStateCore> pastState,
                                        std::shared_ptr<ImuSensorStateCore>& predictedState)
{
    //std::cout<<"ImuSensorCore::predictState()"<<std::endl;


    // Poly
    //std::shared_ptr<ImuSensorStateCore> pastState=std::static_pointer_cast<ImuSensorStateCore>(pastStateI);
    //std::shared_ptr<ImuSensorStateCore> predictedState=std::static_pointer_cast<ImuSensorStateCore>(predictedStateI);


    // Checks in the past state
    if(!pastState->isCorrect())
    {
        return -5;
        std::cout<<"ImuSensorCore::predictState() error !pastState->getTheSensorCore()"<<std::endl;
    }


    // Create the predicted state if it doesn't exists
    if(!predictedState)
    {
        predictedState=std::make_shared<ImuSensorStateCore>(pastState->getMsfElementCoreSharedPtr());
    }

//    // Set The sensor core if it doesn't exist
//    if(!predictedState->getTheSensorCore())
//    {
//        predictedState->setTheSensorCore(pastState->getTheSensorCore());
//    }



    // Equations
    //*predictedState=*pastState;


    // Pose of the sensor wrt Robot

    // Position of sensor wrt Robot
    predictedState->positionSensorWrtRobot=pastState->positionSensorWrtRobot;

    // Attitude of sensor wrt Robot
//    if(pastState->attitudeSensorWrtRobot[0]<0)
//    {
//        predictedState->attitudeSensorWrtRobot=-pastState->attitudeSensorWrtRobot;
//        std::cout<<"ImuSensorCore::predictState() quaternion!"<<std::endl;
//    }
//    else
        predictedState->attitudeSensorWrtRobot=pastState->attitudeSensorWrtRobot;


    // Bias Angular Velocity
    predictedState->biasesAngularVelocity=pastState->biasesAngularVelocity;


    // Scale Angular Velocity
    predictedState->scaleAngularVelocity=pastState->scaleAngularVelocity;


    // Bias Linear Acceleration
    predictedState->biasesLinearAcceleration=pastState->biasesLinearAcceleration;

    // Scale Linear Velocity
    predictedState->scaleLinearAcceleration=pastState->scaleLinearAcceleration;



    //predictedStateI=predictedState;


    //std::cout<<"ImuSensorCore::predictState() end"<<std::endl;

    return 0;
}

int ImuSensorCore::predictErrorStateJacobian(//Time
                             const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp,
                             // Previous State
                             const std::shared_ptr<StateEstimationCore> past_state,
                            // Inputs
                            const std::shared_ptr<InputCommandComponent> input_command,
                             // Predicted State
                             std::shared_ptr<StateCore> &predicted_state)
{
    // Checks

    // Past State
    if(!past_state)
        return -1;

    // Predicted State
    if(!predicted_state)
        return -10;

    // TODO


    // Search for the past sensor State Core
    std::shared_ptr<ImuSensorStateCore> past_sensor_state;

    for(std::list< std::shared_ptr<StateCore> >::iterator it_sensor_state=past_state->TheListSensorStateCore.begin();
        it_sensor_state!=past_state->TheListSensorStateCore.end();
        ++it_sensor_state)
    {
        if((*it_sensor_state)->getMsfElementCoreSharedPtr() == this->getMsfElementCoreSharedPtr())
        {
            past_sensor_state=std::dynamic_pointer_cast<ImuSensorStateCore>(*it_sensor_state);
            break;
        }
    }
    if(!past_sensor_state)
        return -10;


    //// Init Jacobians
    int error_init_jacobians=predictErrorStateJacobianInit(// Past State
                                                           past_state,
                                                           // Input commands
                                                           input_command,
                                                           // Predicted State
                                                           predicted_state);

    if(error_init_jacobians)
        return error_init_jacobians;



    /// Predicted State Cast
    std::shared_ptr<ImuSensorStateCore> predicted_sensor_state;
    predicted_sensor_state=std::dynamic_pointer_cast<ImuSensorStateCore>(predicted_state);


    /// Get iterators to fill jacobians

    // Fx & Fp
    // Sensor
    std::vector<Eigen::SparseMatrix<double> >::iterator it_jacobian_error_state_wrt_sensor_error_state;
    it_jacobian_error_state_wrt_sensor_error_state=predicted_sensor_state->jacobian_error_state_.sensors.begin();

    std::vector<Eigen::SparseMatrix<double> >::iterator it_jacobian_error_state_wrt_sensor_error_parameters;
    it_jacobian_error_state_wrt_sensor_error_parameters=predicted_sensor_state->jacobian_error_parameters_.sensors.begin();

    for(std::list< std::shared_ptr<StateCore> >::iterator itSensorStateCore=past_state->TheListSensorStateCore.begin();
        itSensorStateCore!=past_state->TheListSensorStateCore.end();
        ++itSensorStateCore, ++it_jacobian_error_state_wrt_sensor_error_state, ++it_jacobian_error_state_wrt_sensor_error_parameters
        )
    {
        if( std::dynamic_pointer_cast<ImuSensorStateCore>((*itSensorStateCore)) == past_sensor_state )
            break;
    }


    // Fu
    // Nothing


    // Fn
    // Nothing






    // Predict State
    int error_predict_state=predictErrorStateJacobiansSpecific(previousTimeStamp, currentTimeStamp,
                                                               past_sensor_state,
                                                               predicted_sensor_state,
                                                               // Jacobians Error State: Fx, Fp
                                                               // Sensor
                                                               (*it_jacobian_error_state_wrt_sensor_error_state),
                                                               (*it_jacobian_error_state_wrt_sensor_error_parameters),
                                                               // Jacobians Noise: Fn
                                                               predicted_sensor_state->jacobian_error_state_noise_
                                                               );

    // Check error
    if(error_predict_state)
        return error_predict_state;


    // Set predicted state
    predicted_state=predicted_sensor_state;


    // End
    return 0;
}

int ImuSensorCore::predictErrorStateJacobiansSpecific(const TimeStamp &previousTimeStamp, const TimeStamp &currentTimeStamp,
                                                      const std::shared_ptr<ImuSensorStateCore> pastState,
                                                      std::shared_ptr<ImuSensorStateCore>& predictedState,
                                                      // Jacobians Error State: Fx, Fp
                                                      // Sensor
                                                      Eigen::SparseMatrix<double>& jacobian_error_state_wrt_sensor_error_state,
                                                      Eigen::SparseMatrix<double>& jacobian_error_state_wrt_sensor_error_parameters,
                                                      // Jacobians Noise: Fn
                                                      Eigen::SparseMatrix<double>& jacobian_error_state_wrt_noise
                                                      )
{

    /// Checks
    if(!predictedState)
    {
        return -1;
    }

    /// Variables
    // State k: Sensor
    Eigen::Vector3d position_sensor_wrt_robot;
    Eigen::Vector4d attitude_sensor_wrt_robot;
    // State k+1: Sensor
    Eigen::Vector3d pred_position_sensor_wrt_robot;
    Eigen::Vector4d pred_attitude_sensor_wrt_robot;

    // Jacobian: State
    Eigen::Matrix3d jacobian_error_sens_pos_wrt_error_state_sens_pos;
    Eigen::Matrix3d jacobian_error_sens_att_wrt_error_state_sens_att;
    Eigen::Matrix3d jacobian_error_bias_lin_acc_wrt_error_bias_lin_acc;
    Eigen::Matrix3d jacobian_error_bias_ang_vel_wrt_error_bias_ang_vel;



    /// Fill variables
    // State k: Sensor
    position_sensor_wrt_robot=pastState->getPositionSensorWrtRobot();
    attitude_sensor_wrt_robot=pastState->getAttitudeSensorWrtRobot();

    // State k+1: Sensor
    pred_position_sensor_wrt_robot=predictedState->getPositionSensorWrtRobot();
    pred_attitude_sensor_wrt_robot=predictedState->getAttitudeSensorWrtRobot();


    //// Core

    int error_predict_error_state_jacobians_core=predictErrorStateJacobiansCore(// State k: Sensor
                                                                                position_sensor_wrt_robot,
                                                                                attitude_sensor_wrt_robot,
                                                                                // State k+1: Sensor
                                                                                pred_position_sensor_wrt_robot,
                                                                                pred_attitude_sensor_wrt_robot,
                                                                                // Jacobians
                                                                                jacobian_error_sens_pos_wrt_error_state_sens_pos,
                                                                                jacobian_error_sens_att_wrt_error_state_sens_att,
                                                                                jacobian_error_bias_lin_acc_wrt_error_bias_lin_acc,
                                                                                jacobian_error_bias_ang_vel_wrt_error_bias_ang_vel);

    if(error_predict_error_state_jacobians_core)
        return error_predict_error_state_jacobians_core;


    //// Jacobians Error State - Error State: Fx & Jacobians Error State - Error Parameters: Fp
    // TODO FIX!
    /// World
    {
        // Nothing to do
    }

    /// Robot
    {
        // Nothing to do
    }

    /// Inputs
    {
        // Nothing to do
    }

    /// Sensors
    {
        // Resize and init
        jacobian_error_state_wrt_sensor_error_state.resize(dimension_error_state_, dimension_error_state_);
        jacobian_error_state_wrt_sensor_error_parameters.resize(dimension_error_state_, dimension_error_parameters_);

        std::vector<Eigen::Triplet<double>> triplet_list_jacobian_error_state_wrt_error_state;
        std::vector<Eigen::Triplet<double>> triplet_list_jacobian_error_state_wrt_error_parameters;


        // Fill
        int dimension_error_state_i=0;
        int dimension_error_parameters_i=0;


        // Position sensor wrt robot
        if(this->isEstimationPositionSensorWrtRobotEnabled())
        {
            // Add to the triplets
            BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_state_wrt_error_state, jacobian_error_sens_pos_wrt_error_state_sens_pos, dimension_error_state_i, dimension_error_state_i);

            // Update dimension for next
            dimension_error_state_i+=3;
        }


        // Attitude sensor wrt robot
        if(this->isEstimationAttitudeSensorWrtRobotEnabled())
        {
            // Add to the triplets
            BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_state_wrt_error_state, jacobian_error_sens_att_wrt_error_state_sens_att, dimension_error_state_i, dimension_error_state_i);

            // Update dimension for next
            dimension_error_state_i+=3;
        }


        // bias linear acceleration
        if(this->isEstimationBiasLinearAccelerationEnabled())
        {
            // Add to the triplets
            BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_state_wrt_error_state, jacobian_error_bias_lin_acc_wrt_error_bias_lin_acc, dimension_error_state_i, dimension_error_state_i);

            // Update dimension for next
            dimension_error_state_i+=3;
        }


        // Ka
        // TODO

        // bias angular velocity
        if(this->isEstimationBiasAngularVelocityEnabled())
        {
            // Add to the triplets
            BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_state_wrt_error_state, jacobian_error_bias_ang_vel_wrt_error_bias_ang_vel, dimension_error_state_i, dimension_error_state_i);

            // Update dimension for next
            dimension_error_state_i+=3;
        }


        // Kw
        // TODO




        // Set From Triplets
        jacobian_error_state_wrt_sensor_error_state.setFromTriplets(triplet_list_jacobian_error_state_wrt_error_state.begin(), triplet_list_jacobian_error_state_wrt_error_state.end());
        jacobian_error_state_wrt_sensor_error_parameters.setFromTriplets(triplet_list_jacobian_error_state_wrt_error_parameters.begin(), triplet_list_jacobian_error_state_wrt_error_parameters.end());

    }

    /// Map Elements
    {
        // Nothing to do
    }



    //// Jacobian Error State - Error Noise

    {
        // Resize and init
        jacobian_error_state_wrt_noise.resize(getDimensionErrorState(), getDimensionNoise());



        // Fill
        int dimension_noise_i=0;

        std::vector<Eigen::Triplet<double> > tripletJacobianErrorStateNoise;


        // bias linear acceleration
        if(isEstimationBiasLinearAccelerationEnabled())
        {
            int dimension_error_state_i=0;

            if(isEstimationPositionSensorWrtRobotEnabled())
                dimension_error_state_i+=3;

            if(isEstimationAttitudeSensorWrtRobotEnabled())
                dimension_error_state_i+=3;

            // Update jacobian
            //jacobian_error_state_noise.block<3,3>(dimension_error_state_i, dimension_noise_i)=Eigen::MatrixXd::Identity(3,3);
            for(int i=0; i<3; i++)
                tripletJacobianErrorStateNoise.push_back(Eigen::Triplet<double>(dimension_error_state_i+i,dimension_noise_i+i,1));


            // Update dimension for next
            dimension_noise_i+=3;
        }

        // bias angular velocity
        if(isEstimationBiasAngularVelocityEnabled())
        {
            int dimension_error_state_i=0;

            if(isEstimationPositionSensorWrtRobotEnabled())
                dimension_error_state_i+=3;

            if(isEstimationAttitudeSensorWrtRobotEnabled())
                dimension_error_state_i+=3;

            if(isEstimationBiasLinearAccelerationEnabled())
                dimension_error_state_i+=3;

            if(isEstimationScaleLinearAccelerationEnabled())
                dimension_error_state_i+=3;

            // Update jacobian
            //jacobian_error_state_noise.block<3,3>(dimension_error_state_i, dimension_noise_i)=Eigen::MatrixXd::Identity(3,3);
            for(int i=0; i<3; i++)
                tripletJacobianErrorStateNoise.push_back(Eigen::Triplet<double>(dimension_error_state_i+i,dimension_noise_i+i,1));

            // Update dimension for next
            dimension_noise_i+=3;
        }


        // Set From Triplets
        jacobian_error_state_wrt_noise.setFromTriplets(tripletJacobianErrorStateNoise.begin(), tripletJacobianErrorStateNoise.end());

    }


    // End
    return 0;
}


int ImuSensorCore::predictErrorStateJacobiansCore(// State k: Sensor
                                                  const Eigen::Vector3d& position_sensor_wrt_robot, const Eigen::Vector4d& attitude_sensor_wrt_robot,
                                                  // TODO Add others
                                                  // State k+1: Sensor
                                                  const Eigen::Vector3d& pred_position_sensor_wrt_robot, const Eigen::Vector4d& pred_attitude_sensor_wrt_robot,
                                                  // TODO add others
                                                  // Jacobian: State Fx & Fp
                                                  Eigen::Matrix3d& jacobian_error_sens_pos_wrt_error_state_sens_pos,  Eigen::Matrix3d& jacobian_error_sens_att_wrt_error_state_sens_att,
                                                  Eigen::Matrix3d& jacobian_error_bias_lin_acc_wrt_error_bias_lin_acc,
                                                  Eigen::Matrix3d& jacobian_error_bias_ang_vel_wrt_error_bias_ang_vel
                                                  )
{


    // posi sensor / posi sensor
    jacobian_error_sens_pos_wrt_error_state_sens_pos=Eigen::Matrix3d::Identity();

    // att sensor / att sensor
    // TODO FIX
    jacobian_error_sens_att_wrt_error_state_sens_att=Eigen::Matrix3d::Identity();


    // ba / ba
    jacobian_error_bias_lin_acc_wrt_error_bias_lin_acc=Eigen::Matrix3d::Identity();;

    // ka / ka
    // TODO

    // bw / bw
    jacobian_error_bias_ang_vel_wrt_error_bias_ang_vel=Eigen::Matrix3d::Identity();

    // kw / kw
    // TODO

    // End
    return 0;
}


int ImuSensorCore::predictMeasurement(// Time
                                       const TimeStamp current_time_stamp,
                                       // Current State
                                       const std::shared_ptr<StateEstimationCore> current_state,
                                      // Measurements
                                      const std::shared_ptr<SensorMeasurementCore> measurement,
                                       // Predicted Measurements
                                       std::shared_ptr<SensorMeasurementCore> &predicted_measurement)
{
    // State
    if(!current_state)
        return -1;

    // TODO

    // Measurement -> Matching
    if(!measurement)
        return -2;

    if(measurement->getSensorCoreSharedPtr() != std::dynamic_pointer_cast<SensorCore>(this->getMsfElementCoreSharedPtr()))
        return -10;

    // Nothing else needed


    // Predicted Measurement
    std::shared_ptr<ImuSensorMeasurementCore> predicted_sensor_measurement;
    if(!predicted_measurement)
        predicted_sensor_measurement=std::make_shared<ImuSensorMeasurementCore>(std::dynamic_pointer_cast<SensorCore>(this->getMsfElementCoreSharedPtr()));
    else
    {
        if(measurement->getSensorCoreSharedPtr() != predicted_measurement->getSensorCoreSharedPtr())
            return -3;
        predicted_sensor_measurement=std::dynamic_pointer_cast<ImuSensorMeasurementCore>(predicted_measurement);
    }



    // Search for the current sensor State Core
    std::shared_ptr<ImuSensorStateCore> current_sensor_state;

    for(std::list< std::shared_ptr<StateCore> >::iterator it_sensor_state=current_state->TheListSensorStateCore.begin();
        it_sensor_state!=current_state->TheListSensorStateCore.end();
        ++it_sensor_state)
    {
        if((*it_sensor_state)->getMsfElementCoreSharedPtr() == this->getMsfElementCoreSharedPtr())
        {
            current_sensor_state=std::dynamic_pointer_cast<ImuSensorStateCore>(*it_sensor_state);
            break;
        }
    }
    if(!current_sensor_state)
        return -10;


    // Predict State
    int error_predict_measurement=predictMeasurementSpecific(current_time_stamp,
                                                             std::dynamic_pointer_cast<GlobalParametersStateCore>(current_state->TheGlobalParametersStateCore),
                                                             std::dynamic_pointer_cast<RobotStateCore>(current_state->TheRobotStateCore),
                                                             current_sensor_state,
                                                             predicted_sensor_measurement);

    // Check error
    if(error_predict_measurement)
        return error_predict_measurement;


    // Set prediction
    predicted_measurement=predicted_sensor_measurement;


    // End
    return 0;
}

int ImuSensorCore::predictMeasurementSpecific(const TimeStamp &theTimeStamp,
                                      const std::shared_ptr<GlobalParametersStateCore> TheGlobalParametersStateCore,
                                      const std::shared_ptr<RobotStateCore> currentRobotState,
                                      const std::shared_ptr<ImuSensorStateCore> currentImuState,
                                      std::shared_ptr<ImuSensorMeasurementCore>& predictedMeasurement)
{
#if _DEBUG_SENSOR_CORE
    logFile<<"ImuSensorCore::predictMeasurementSpecific() TS: sec="<<theTimeStamp.sec<<" s; nsec="<<theTimeStamp.nsec<<" ns"<<std::endl;
#endif

    // Check
    if(!isCorrect())
    {
        std::cout<<"ImuSensorCore::predictMeasurementSpecific() error 50"<<std::endl;
        return -50;
    }

    // Check imu
    if(!currentImuState)
    {
        std::cout<<"ImuSensorCore::predictMeasurementSpecific() error 1"<<std::endl;
        return -1;
    }

    // Robot check
    if(!currentRobotState)
    {
        std::cout<<"ImuSensorCore::predictMeasurementSpecific() error 2"<<std::endl;
        return -2;
    }

    // Robot core check
    if(!currentRobotState->isCorrect())
    {
        std::cout<<"ImuSensorCore::predictMeasurementSpecific() error 3"<<std::endl;
        return -3;
    }


    // Checks
    // TODO


    // Create pointer
    // TODO check if it must be done here
    if(!predictedMeasurement)
    {
        std::weak_ptr<ImuSensorCore> TheImuSensorCore=std::dynamic_pointer_cast<ImuSensorCore>(this->getMsfElementCoreSharedPtr());
        predictedMeasurement=std::make_shared<ImuSensorMeasurementCore>(TheImuSensorCore);

        //std::weak_ptr<ImuSensorCore> TheImuSensorCore=std::dynamic_pointer_cast<ImuSensorCore>(this->getMsfElementCoreSharedPtr());
        //predictedMeasurement=std::make_shared<ImuSensorMeasurementCore>(std::shared_ptr<ImuSensorCore>(this));

#if _DEBUG_SENSOR_CORE
        logFile<<"ImuSensorCore::predictMeasurementSpecific() pointer created"<<std::endl;
#endif
    }


//    // Set the sensor core -> Needed
//    if(predictedMeasurement)
//    {
//        predictedMeasurement->setTheSensorCore(this->getMsfElementCoreWeakPtr());
//    }



    // Variables
    // State: World
    Eigen::Vector3d gravity_wrt_world;
    // State: Robot
    Eigen::Vector3d position_robot_wrt_world;
    Eigen::Vector4d attitude_robot_wrt_world;
    Eigen::Vector3d lin_speed_robot_wrt_world;
    Eigen::Vector3d ang_velocity_robot_wrt_world;
    Eigen::Vector3d lin_accel_robot_wrt_world;
    Eigen::Vector3d ang_accel_robot_wrt_world;
    // State: Sensor
    Eigen::Vector3d position_sensor_wrt_robot;
    Eigen::Vector4d attitude_sensor_wrt_robot;
    // Parameters: Sensor
    Eigen::Vector3d biases_meas_linear_acceleration;
    Eigen::Matrix3d sensitivity_meas_linear_acceleration;
    Eigen::Vector3d biases_meas_angular_velocity;
    Eigen::Matrix3d sensitivity_meas_angular_velocity;
//    // Measurement
//    Eigen::Vector3d meas_lin_accel_sensor_wrt_sensor;
//    Eigen::Vector3d meas_attitude_sensor_wrt_sensor;
//    Eigen::Vector3d meas_ang_velocity_sensor_wrt_sensor;
//    // Predicted Measurement
//    Eigen::Vector3d lin_accel_sensor_wrt_sensor;
//    Eigen::Vector3d attitude_sensor_wrt_sensor;
//    Eigen::Vector3d ang_velocity_sensor_wrt_sensor;


    /// Fill Variables


    // State: World
    gravity_wrt_world=TheGlobalParametersStateCore->getGravity();

    // State: Robot
    position_robot_wrt_world; // Not needed
    attitude_robot_wrt_world=currentRobotState->getAttitudeRobotWrtWorld();
    lin_speed_robot_wrt_world; // Not needed
    ang_velocity_robot_wrt_world=currentRobotState->getAngularVelocityRobotWrtWorld();
    lin_accel_robot_wrt_world=currentRobotState->getLinearAccelerationRobotWrtWorld();
    ang_accel_robot_wrt_world=currentRobotState->getAngularAccelerationRobotWrtWorld();

    // State: Sensor
    position_sensor_wrt_robot=currentImuState->getPositionSensorWrtRobot();
    attitude_sensor_wrt_robot=currentImuState->getAttitudeSensorWrtRobot();

    // Parameters: Sensor
    biases_meas_linear_acceleration=currentImuState->getBiasesLinearAcceleration();
    sensitivity_meas_linear_acceleration=currentImuState->getScaleLinearAcceleration().asDiagonal();
    biases_meas_angular_velocity=currentImuState->getBiasesAngularVelocity();
    sensitivity_meas_angular_velocity=currentImuState->getScaleAngularVelocity().asDiagonal();

//    // Measurement
//    meas_lin_accel_sensor_wrt_sensor;
//    meas_attitude_sensor_wrt_sensor;
//    meas_ang_velocity_sensor_wrt_sensor;

//    // Predicted Measurement
//    lin_accel_sensor_wrt_sensor;
//    attitude_sensor_wrt_sensor;
//    ang_velocity_sensor_wrt_sensor;


    /// Measurements Prediction



    // Linear acceleration
    if(isMeasurementLinearAccelerationEnabled())
    {
#if _DEBUG_SENSOR_CORE
        logFile<<"ImuSensorCore::predictMeasurementSpecific() linear acceleration"<<std::endl;
#endif


        Eigen::Vector3d ThePredictedLinearAcceleration;


        // Angular vel
        Eigen::Vector3d angular_vel_robot_wrt_world_in_robot=Quaternion::cross_sandwich(Quaternion::inv(attitude_robot_wrt_world), ang_velocity_robot_wrt_world ,attitude_robot_wrt_world);
        // Angular acceler
        Eigen::Vector3d angular_acc_robot_wrt_world_in_robot=Quaternion::cross_sandwich(Quaternion::inv(attitude_robot_wrt_world), ang_accel_robot_wrt_world ,attitude_robot_wrt_world);

        // Ficticious Acc: Normal
        Eigen::Vector3d normal_acceleration=angular_vel_robot_wrt_world_in_robot.cross(angular_vel_robot_wrt_world_in_robot.cross(position_sensor_wrt_robot));
        // Ficticious Acc: Tang
        Eigen::Vector3d tangencial_acceleration=angular_acc_robot_wrt_world_in_robot.cross(position_sensor_wrt_robot);

        // Ficticious Acc total
        Eigen::Vector3d ficticious_acceleration=normal_acceleration+tangencial_acceleration;
        //std::cout<<"ficticious_accel= "<<ficticious_acceleration.transpose()<<std::endl;


//                Eigen::Vector3d ficticious_acceleration;
//                ficticious_acceleration.setZero();


        // Attitude sensor wrt world
        Eigen::Vector4d attitude_sensor_wrt_world=Quaternion::cross(attitude_robot_wrt_world, attitude_sensor_wrt_robot);

        // Acceleracion in sensor
        Eigen::Vector3d accel_sensor_wrt_sensor=Quaternion::cross_sandwich(Quaternion::inv(attitude_sensor_wrt_world), lin_accel_robot_wrt_world, attitude_sensor_wrt_world) + Quaternion::cross_sandwich(Quaternion::inv(attitude_sensor_wrt_robot), ficticious_acceleration, attitude_sensor_wrt_robot);

        // Gravity in sensor
        Eigen::Vector3d gravity_sensor=Quaternion::cross_sandwich(Quaternion::inv(attitude_sensor_wrt_world), gravity_wrt_world, attitude_sensor_wrt_world);


        // IMU Model
        ThePredictedLinearAcceleration=sensitivity_meas_linear_acceleration*(accel_sensor_wrt_sensor-gravity_sensor)+biases_meas_linear_acceleration;

#if _DEBUG_SENSOR_CORE

        logFile<<"ImuSensorCore::predictMeasurementSpecific()"<<std::endl;

        //logFile<<"ImuSensorCore::predictMeasurementSpecific() predicted quat_angular_speed_r_w_w="<<quat_angular_speed_r_w_w.transpose()<<std::endl;
        //logFile<<"ImuSensorCore::predictMeasurementSpecific() predicted quat_robot_w="<<currentRobotState->getAttitude().transpose()<<std::endl;
        //logFile<<"ImuSensorCore::predictMeasurementSpecific() predicted quat_angular_speed_r_w_r="<<quat_angular_speed_r_w_r.transpose()<<std::endl;
        //logFile<<"ImuSensorCore::predictMeasurementSpecific() predicted p_s_r="<<position_sensor_wrt_robot.transpose()<<std::endl;
        //logFile<<"ImuSensorCore::predictMeasurementSpecific() predicted quaternion_tangencial_acceleration="<<quaternion_tangencial_acceleration.transpose()<<std::endl;
        //logFile<<"ImuSensorCore::predictMeasurementSpecific() predicted quaternion_normal_acceleration="<<quaternion_normal_acceleration.transpose()<<std::endl;
        //logFile<<"ImuSensorCore::predictMeasurementSpecific() predicted ficticious_acceleration="<<ficticious_acceleration.transpose()<<std::endl;


        logFile<<"ImuSensorCore::predictMeasurementSpecific() currentRobotState->getLinearAcceleration()"<<std::endl;
        logFile<<currentRobotState->getLinearAcceleration().transpose()<<std::endl;

        logFile<<"ImuSensorCore::predictMeasurementSpecific() currentRobotState->getAttitude()"<<std::endl;
        logFile<<currentRobotState->getAttitude().transpose()<<std::endl;

        logFile<<"ImuSensorCore::predictMeasurementSpecific() gravity_sensor"<<std::endl;
        logFile<<gravity_sensor.transpose()<<std::endl;

        logFile<<"ImuSensorCore::predictMeasurementSpecific() ficticious_acceleration"<<std::endl;
        logFile<<ficticious_acceleration.transpose()<<std::endl;

        logFile<<"ImuSensorCore::predictMeasurementSpecific() accel_sensor_wrt_sensor"<<std::endl;
        logFile<<accel_sensor_wrt_sensor.transpose()<<std::endl;

        logFile<<"ImuSensorCore::predictMeasurementSpecific() biases_meas_linear_acceleration"<<std::endl;
        logFile<<biases_meas_linear_acceleration.transpose()<<std::endl;

        logFile<<"ImuSensorCore::predictMeasurementSpecific() predicted a="<<std::endl;
        logFile<<ThePredictedLinearAcceleration.transpose()<<std::endl;
#endif


        // Set
        predictedMeasurement->setLinearAcceleration(ThePredictedLinearAcceleration);
    }



    // Orientation
    if(this->isMeasurementOrientationEnabled())
    {
#if _DEBUG_SENSOR_CORE
        logFile<<"ImuSensorCore::predictMeasurementSpecific() orientation"<<std::endl;
#endif

        // TODO

    }

    // Angular velocity
    if(isMeasurementAngularVelocityEnabled())
    {
#if _DEBUG_SENSOR_CORE
        logFile<<"ImuSensorCore::predictMeasurementSpecific() angular velocity"<<std::endl;
#endif


        Eigen::Vector3d ThePredictedAngularVelocity;


        // Model
        Eigen::Vector4d quat_predicted_angular_velocity_in_robot;
        quat_predicted_angular_velocity_in_robot[0]=0;
        quat_predicted_angular_velocity_in_robot.block<3,1>(1,0)=ang_velocity_robot_wrt_world;
        Eigen::Vector4d quat_predicted_angular_velocity_in_imu;

        quat_predicted_angular_velocity_in_imu=Quaternion::cross(Quaternion::inv(attitude_sensor_wrt_robot), Quaternion::inv(attitude_robot_wrt_world), quat_predicted_angular_velocity_in_robot, attitude_robot_wrt_world, attitude_sensor_wrt_robot);

        Eigen::Vector3d predicted_angular_velocity_in_imu;
        predicted_angular_velocity_in_imu=quat_predicted_angular_velocity_in_imu.block<3,1>(1,0);

        ThePredictedAngularVelocity=sensitivity_meas_angular_velocity*predicted_angular_velocity_in_imu+biases_meas_angular_velocity;

#if _DEBUG_SENSOR_CORE
        logFile<<"ImuSensorCore::predictMeasurementSpecific() predicted w="<<ThePredictedAngularVelocity.transpose()<<std::endl;
#endif


        // Set
        predictedMeasurement->setAngularVelocity(ThePredictedAngularVelocity);
    }






#if _DEBUG_SENSOR_CORE
    logFile<<"ImuSensorCore::predictMeasurementSpecific() ended TS: sec="<<theTimeStamp.sec<<" s; nsec="<<theTimeStamp.nsec<<" ns"<<std::endl;
#endif

    // End
    return 0;
}


int ImuSensorCore::predictErrorMeasurementJacobian(// Time
                                                const TimeStamp current_time_stamp,
                                                // Current State
                                                const std::shared_ptr<StateEstimationCore> current_state,
                                                // Measurements
                                                const std::shared_ptr<SensorMeasurementCore> measurement,
                                                // Predicted Measurement
                                                std::shared_ptr<SensorMeasurementCore> &predicted_measurement)
{
    // State
    if(!current_state)
        return -1;

    // TODO


    // Measurement -> Matching
    if(!measurement)
        return -2;

    if(measurement->getSensorCoreSharedPtr() != std::dynamic_pointer_cast<SensorCore>(this->getMsfElementCoreSharedPtr()))
        return -10;

    // Nothing else needed


    // Predicted Measurement
    if(!predicted_measurement)
        return -1;



    // Search for the current sensor State Core
    std::shared_ptr<ImuSensorStateCore> current_sensor_state;

    for(std::list< std::shared_ptr<StateCore> >::iterator it_sensor_state=current_state->TheListSensorStateCore.begin();
        it_sensor_state!=current_state->TheListSensorStateCore.end();
        ++it_sensor_state)
    {
        if((*it_sensor_state)->getMsfElementCoreSharedPtr() == this->getMsfElementCoreSharedPtr())
        {
            current_sensor_state=std::dynamic_pointer_cast<ImuSensorStateCore>(*it_sensor_state);
            break;
        }
    }
    if(!current_sensor_state)
        return -10;


    //// Init Jacobians
    int error_init_jacobians=predictErrorMeasurementJacobianInit(// Current State
                                                                current_state,
                                                                // Predicted Measurements
                                                                predicted_measurement);

    if(error_init_jacobians)
        return error_init_jacobians;


    // Predicted Measurement Cast
    std::shared_ptr<ImuSensorMeasurementCore> predicted_sensor_measurement;
    if(measurement->getSensorCoreSharedPtr() != predicted_measurement->getSensorCoreSharedPtr())
        return -3;
    predicted_sensor_measurement=std::dynamic_pointer_cast<ImuSensorMeasurementCore>(predicted_measurement);



    /// Get iterators to fill jacobians

    // World
    // Nothing to do

    // Robot
    // Nothing to do

    // Sensor
    std::vector<Eigen::SparseMatrix<double> >::iterator it_jacobian_error_measurement_wrt_sensor_error_state;
    it_jacobian_error_measurement_wrt_sensor_error_state=predicted_sensor_measurement->jacobian_error_measurement_wrt_error_state_.sensors.begin();

    std::vector<Eigen::SparseMatrix<double> >::iterator it_jacobian_error_measurement_wrt_sensor_error_parameters;
    it_jacobian_error_measurement_wrt_sensor_error_parameters=predicted_sensor_measurement->jacobian_error_measurement_wrt_error_parameters_.sensors.begin();

    for(std::list< std::shared_ptr<StateCore> >::iterator itSensorStateCore=current_state->TheListSensorStateCore.begin();
        itSensorStateCore!=current_state->TheListSensorStateCore.end();
        ++itSensorStateCore, ++it_jacobian_error_measurement_wrt_sensor_error_state, ++it_jacobian_error_measurement_wrt_sensor_error_parameters
        )
    {
        if( std::dynamic_pointer_cast<ImuSensorStateCore>((*itSensorStateCore)) == current_sensor_state )
            break;
    }


    /// Predict Error Measurement Jacobians
    int error_predict_measurement=predictErrorMeasurementJacobianSpecific(current_time_stamp,
                                                                          std::dynamic_pointer_cast<GlobalParametersStateCore>(current_state->TheGlobalParametersStateCore),
                                                                          std::dynamic_pointer_cast<RobotStateCore>(current_state->TheRobotStateCore),
                                                                          current_sensor_state,
                                                                          predicted_sensor_measurement,
                                                                          // Jacobians State / Parameters
                                                                          // World
                                                                          predicted_sensor_measurement->jacobian_error_measurement_wrt_error_state_.world,
                                                                          predicted_sensor_measurement->jacobian_error_measurement_wrt_error_parameters_.world,
                                                                          // Robot
                                                                          predicted_sensor_measurement->jacobian_error_measurement_wrt_error_state_.robot,
                                                                          predicted_sensor_measurement->jacobian_error_measurement_wrt_error_parameters_.robot,
                                                                          // Sensor
                                                                          (*it_jacobian_error_measurement_wrt_sensor_error_state),
                                                                          (*it_jacobian_error_measurement_wrt_sensor_error_parameters),
                                                                          // Jacobians Measurement
                                                                          predicted_sensor_measurement->jacobian_error_measurement_wrt_error_measurement_.measurement
                                                                          );

    // Check error
    if(error_predict_measurement)
        return error_predict_measurement;


    // Set predicted state
    predicted_measurement=predicted_sensor_measurement;


    // End
    return 0;
}


int ImuSensorCore::predictErrorMeasurementJacobianSpecific(const TimeStamp& theTimeStamp,
                                                           const std::shared_ptr<GlobalParametersStateCore> TheGlobalParametersStateCore,
                                                           const std::shared_ptr<RobotStateCore> TheRobotStateCore,
                                                           const std::shared_ptr<ImuSensorStateCore> TheImuStateCore,
                                                           std::shared_ptr<ImuSensorMeasurementCore>& predictedMeasurement,
                                                           // Jacobians State / Parameters
                                                           // World
                                                           Eigen::SparseMatrix<double>& jacobian_error_measurement_wrt_world_error_state,
                                                           Eigen::SparseMatrix<double>& jacobian_error_measurement_wrt_world_error_parameters,
                                                           // Robot
                                                           Eigen::SparseMatrix<double>& jacobian_error_measurement_wrt_robot_error_state,
                                                           Eigen::SparseMatrix<double>& jacobian_error_measurement_wrt_robot_error_parameters,
                                                           // Sensor
                                                           Eigen::SparseMatrix<double>& jacobian_error_measurement_wrt_sensor_error_state,
                                                           Eigen::SparseMatrix<double>& jacobian_error_measurement_wrt_sensor_error_parameters,
                                                           // Jacobians Measurement
                                                           Eigen::SparseMatrix<double>& jacobian_error_measurement_wrt_error_measurement
                                                           )
{

    // Checks
    // TODO

    //predictedMeasurement



    /// Common Variables

    // Imu sensor core
    std::shared_ptr<const ImuSensorCore> the_imu_sensor_core=std::dynamic_pointer_cast<const ImuSensorCore>(TheImuStateCore->getMsfElementCoreSharedPtr());


    // dimension of the measurement
    int dimension_error_measurement=this->getDimensionErrorMeasurement();




    /// Variables

    // State: World
    Eigen::Vector3d gravity_wrt_world;
    // State: Robot
    Eigen::Vector3d position_robot_wrt_world;
    Eigen::Vector4d attitude_robot_wrt_world;
    Eigen::Vector3d lin_speed_robot_wrt_world;
    Eigen::Vector3d ang_velocity_robot_wrt_world;
    Eigen::Vector3d lin_accel_robot_wrt_world;
    Eigen::Vector3d ang_accel_robot_wrt_world;
    // State: Sensor
    Eigen::Vector3d position_sensor_wrt_robot;
    Eigen::Vector4d attitude_sensor_wrt_robot;
    // Parameters: Sensor
    Eigen::Matrix3d sensitivity_meas_linear_acceleration;
    Eigen::Matrix3d sensitivity_meas_angular_velocity;
    // Measurement
    Eigen::Vector3d meas_lin_accel_sensor_wrt_sensor;
    Eigen::Vector3d meas_attitude_sensor_wrt_sensor;
    Eigen::Vector3d meas_ang_velocity_sensor_wrt_sensor;
    // Predicted Measurement
    Eigen::Vector3d lin_accel_sensor_wrt_sensor;
    Eigen::Vector3d attitude_sensor_wrt_sensor;
    Eigen::Vector3d ang_velocity_sensor_wrt_sensor;


    /// Fill Variables


    // State: World
    gravity_wrt_world=TheGlobalParametersStateCore->getGravity();

    // State: Robot
    position_robot_wrt_world; // Not needed
    attitude_robot_wrt_world=TheRobotStateCore->getAttitudeRobotWrtWorld();
    lin_speed_robot_wrt_world; // Not needed
    ang_velocity_robot_wrt_world=TheRobotStateCore->getAngularVelocityRobotWrtWorld();
    lin_accel_robot_wrt_world=TheRobotStateCore->getLinearAccelerationRobotWrtWorld();
    ang_accel_robot_wrt_world=TheRobotStateCore->getAngularAccelerationRobotWrtWorld();

    // State: Sensor
    position_sensor_wrt_robot=TheImuStateCore->getPositionSensorWrtRobot();
    attitude_sensor_wrt_robot=TheImuStateCore->getAttitudeSensorWrtRobot();

    // Parameters: Sensor
    sensitivity_meas_linear_acceleration=TheImuStateCore->getScaleLinearAcceleration().asDiagonal();
    sensitivity_meas_angular_velocity=TheImuStateCore->getScaleAngularVelocity().asDiagonal();


    // Measurement
    meas_lin_accel_sensor_wrt_sensor;
    meas_attitude_sensor_wrt_sensor;
    meas_ang_velocity_sensor_wrt_sensor;

    // Predicted Measurement
    lin_accel_sensor_wrt_sensor;
    attitude_sensor_wrt_sensor;
    ang_velocity_sensor_wrt_sensor;




    /// Jacobians Variables

    // Jacobian State

    // World
    Eigen::Matrix3d jacobian_error_meas_lin_acc_wrt_error_gravity;

    // Robot
    Eigen::Matrix3d jacobian_error_meas_lin_acc_wrt_error_state_robot_lin_acc;
    Eigen::Matrix3d jacobian_error_meas_lin_acc_wrt_error_state_robot_att;
    Eigen::Matrix3d jacobian_error_meas_lin_acc_wrt_error_state_robot_ang_vel;
    Eigen::Matrix3d jacobian_error_meas_lin_acc_wrt_error_state_robot_ang_acc;

    // TODO

    Eigen::Matrix3d jacobian_error_meas_ang_vel_wrt_error_state_robot_att;
    Eigen::Matrix3d jacobian_error_meas_ang_vel_wrt_error_state_robot_ang_vel;


    // Sensor
    Eigen::Matrix3d jacobian_error_meas_lin_acc_wrt_error_state_sensor_pos;
    Eigen::Matrix3d jacobian_error_meas_lin_acc_wrt_error_state_sensor_att;
    Eigen::Matrix3d jacobian_error_meas_lin_acc_wrt_error_state_sensor_bias_lin_acc;

    Eigen::Matrix3d jacobian_error_meas_ang_vel_wrt_error_state_sensor_att;
    Eigen::Matrix3d jacobian_error_meas_ang_vel_wrt_error_state_sensor_bias_ang_vel;


    // Jacobians: Noise
    Eigen::Matrix3d jacobian_error_meas_lin_acc_wrt_error_meas_lin_acc;
    Eigen::Matrix3d jacobian_error_meas_att_wrt_error_meas_att;
    Eigen::Matrix3d jacobian_error_meas_ang_vel_wrt_error_meas_ang_vel;




    /// Call core

    int error=predictErrorMeasurementJacobianCore(// State: World
                                                  gravity_wrt_world,
                                                  // State: Robot
                                                  position_robot_wrt_world, attitude_robot_wrt_world,
                                                  lin_speed_robot_wrt_world, ang_velocity_robot_wrt_world,
                                                  lin_accel_robot_wrt_world, ang_accel_robot_wrt_world,
                                                  // State: Sensor
                                                  position_sensor_wrt_robot, attitude_sensor_wrt_robot,
                                                  // Parameters: Sensor
                                                  sensitivity_meas_linear_acceleration, sensitivity_meas_angular_velocity,
                                                  // Measurement
                                                  meas_lin_accel_sensor_wrt_sensor, meas_attitude_sensor_wrt_sensor, meas_ang_velocity_sensor_wrt_sensor,
                                                  // Predicted Measurement
                                                  lin_accel_sensor_wrt_sensor, attitude_sensor_wrt_sensor, ang_velocity_sensor_wrt_sensor,
                                                  // Jacobians: State and Params
                                                  jacobian_error_meas_lin_acc_wrt_error_gravity,
                                                  jacobian_error_meas_lin_acc_wrt_error_state_robot_lin_acc, jacobian_error_meas_lin_acc_wrt_error_state_robot_att, jacobian_error_meas_lin_acc_wrt_error_state_robot_ang_vel, jacobian_error_meas_lin_acc_wrt_error_state_robot_ang_acc,
                                                  jacobian_error_meas_ang_vel_wrt_error_state_robot_att, jacobian_error_meas_ang_vel_wrt_error_state_robot_ang_vel,
                                                  jacobian_error_meas_lin_acc_wrt_error_state_sensor_pos, jacobian_error_meas_lin_acc_wrt_error_state_sensor_att, jacobian_error_meas_lin_acc_wrt_error_state_sensor_bias_lin_acc,
                                                  jacobian_error_meas_ang_vel_wrt_error_state_sensor_att, jacobian_error_meas_ang_vel_wrt_error_state_sensor_bias_ang_vel,
                                                  // Jacobians: Noise
                                                  jacobian_error_meas_lin_acc_wrt_error_meas_lin_acc, jacobian_error_meas_att_wrt_error_meas_att, jacobian_error_meas_ang_vel_wrt_error_meas_ang_vel
                                                  );

    if(error)
        return error;


    /// Dimensions

    // dimension of the global parameters
    int dimension_world_error_state=TheGlobalParametersStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorState();
    int dimension_world_error_parameters=TheGlobalParametersStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorParameters();

    // dimension of robot
    int dimension_robot_error_state=TheRobotStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorState();
    int dimension_robot_error_parameters=TheRobotStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorParameters();

    // dimension of the sensor
    int dimension_sensor_error_state=TheImuStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorState();
    int dimension_sensor_error_parameters=TheImuStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorParameters();




    ///// Jacobians error measurement - error state / error parameters


    /// Jacobians error measurement - world error state / error parameters

    {

        // Resize and init Jacobian
        jacobian_error_measurement_wrt_world_error_state.resize(dimension_error_measurement_, dimension_world_error_state);
        jacobian_error_measurement_wrt_world_error_parameters.resize(dimension_error_measurement_, dimension_world_error_parameters);

        std::vector<Eigen::Triplet<double>> triplet_list_jacobian_error_measurement_wrt_error_state;
        std::vector<Eigen::Triplet<double>> triplet_list_jacobian_error_measurement_wrt_error_parameters;


        // Fill Jacobian
        unsigned int dimension_error_measurement_i=0;

        // z_lin_acc
        if(this->isMeasurementLinearAccelerationEnabled())
        {
            // z_lin_acc / grav
            if(std::dynamic_pointer_cast<GlobalParametersCore>(TheGlobalParametersStateCore->getMsfElementCoreSharedPtr())->isEstimationGravityEnabled())
            {
//              predictedMeasurement->jacobianMeasurementErrorParameters.jacobianMeasurementGlobalParameters.block<3,3>(dimension_error_measurement_i,0)=
//                      jacobian_error_meas_lin_acc_wrt_error_gravity;
                BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_state, jacobian_error_meas_lin_acc_wrt_error_gravity, dimension_error_measurement_i, 0);
            }
            else
            {
//              predictedMeasurement->jacobianMeasurementErrorParameters.jacobianMeasurementGlobalParameters.block<3,3>(dimension_error_measurement_i,0)=
//                      jacobian_error_meas_lin_acc_wrt_error_gravity;
                BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_parameters, jacobian_error_meas_lin_acc_wrt_error_gravity, dimension_error_measurement_i, 0);
            }

            dimension_error_measurement_i+=3;
        }

        // z_atti
        if(this->isMeasurementOrientationEnabled())
        {
            // z_atti / grav
            // Zeros

            dimension_error_measurement_i+=3;
        }

        // z_ang_vel
        if(this->isMeasurementAngularVelocityEnabled())
        {
            // z_ang_vel / grav
            // Zeros

            dimension_error_measurement_i+=3;
        }

        // Set From Triplets
        jacobian_error_measurement_wrt_world_error_state.setFromTriplets(triplet_list_jacobian_error_measurement_wrt_error_state.begin(), triplet_list_jacobian_error_measurement_wrt_error_state.end());
        jacobian_error_measurement_wrt_world_error_parameters.setFromTriplets(triplet_list_jacobian_error_measurement_wrt_error_parameters.begin(), triplet_list_jacobian_error_measurement_wrt_error_parameters.end());

    }


    /// Jacobian error measurement - robot error state / error parameters

    {

        // Resize and init
        jacobian_error_measurement_wrt_robot_error_state.resize(dimension_error_measurement_, dimension_robot_error_state);
        jacobian_error_measurement_wrt_robot_error_parameters.resize(dimension_error_measurement_, dimension_robot_error_parameters);

        std::vector<Eigen::Triplet<double>> triplet_list_jacobian_error_measurement_wrt_error_state;
        std::vector<Eigen::Triplet<double>> triplet_list_jacobian_error_measurement_wrt_error_parameters;



        // Fill Jacobian
        unsigned int dimension_error_measurement_i=0;

        // z_lin_acc
        if(this->isMeasurementLinearAccelerationEnabled())
        {
            // Switch depending on robot used
            switch(std::dynamic_pointer_cast<RobotCore>(TheRobotStateCore->getMsfElementCoreSharedPtr())->getRobotCoreType())
            {
                case RobotCoreTypes::free_model:
                {
                    // z_lin_acc / posi
                    // Zeros

                    // z_lin_acc / lin_speed
                    // Zeros

                    // z_lin_acc / lin_acc
//                    predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementRobotErrorState.block<3,3>(dimension_error_measurement_i, 6)=
//                            jacobian_error_meas_lin_acc_wrt_error_state_robot_lin_acc;
                    BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_state, jacobian_error_meas_lin_acc_wrt_error_state_robot_lin_acc, dimension_error_measurement_i, 6);


                    // z_lin_acc / attit
//                    predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementRobotErrorState.block<3,3>(dimension_error_measurement_i, 9)=
//                            jacobian_error_meas_lin_acc_wrt_error_state_robot_att;
                    BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_state, jacobian_error_meas_lin_acc_wrt_error_state_robot_att, dimension_error_measurement_i, 9);

                    // z_lin_acc / ang_vel
//                    predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementRobotErrorState.block<3,3>(dimension_error_measurement_i, 12)=
//                            jacobian_error_meas_lin_acc_wrt_error_state_robot_ang_vel;
                    BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_state, jacobian_error_meas_lin_acc_wrt_error_state_robot_ang_vel, dimension_error_measurement_i, 12);

                    // z_lin_acc / ang_acc
//                    predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementRobotErrorState.block<3,3>(dimension_error_measurement_i, 15)=
//                            jacobian_error_meas_lin_acc_wrt_error_state_robot_ang_acc;
                    BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_state, jacobian_error_meas_lin_acc_wrt_error_state_robot_ang_acc, dimension_error_measurement_i, 15);


                    // end
                    break;
                }
                default:
                {
                    return -2;
                }

            }

            dimension_error_measurement_i+=3;
        }


        // z_atti
        if(this->isMeasurementOrientationEnabled())
        {
            // Switch depending on robot used
            switch(std::dynamic_pointer_cast<RobotCore>(TheRobotStateCore->getMsfElementCoreSharedPtr())->getRobotCoreType())
            {
                case RobotCoreTypes::free_model:
                {
                    // TODO
                    //predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementRobotErrorState;


                    // end
                    break;
                }
                default:
                {
                    return -2;
                }

            }

            dimension_error_measurement_i+=3;
        }


        // z_ang_vel
        if(this->isMeasurementAngularVelocityEnabled())
        {
            // Switch depending on robot used
            switch(std::dynamic_pointer_cast<RobotCore>(TheRobotStateCore->getMsfElementCoreSharedPtr())->getRobotCoreType())
            {
                case RobotCoreTypes::free_model:
                {
                    // z_ang_vel / posi
                    // Zeros

                    // z_ang_vel / lin_speed
                    // Zeros

                    // z_ang_vel / lin_acc
                    // Zeros


                    // z_ang_vel / attit
//                    predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementRobotErrorState.block<3,3>(dimension_error_measurement_i, 9)=
//                            jacobian_error_meas_ang_vel_wrt_error_state_robot_att;
                    BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_state, jacobian_error_meas_ang_vel_wrt_error_state_robot_att, dimension_error_measurement_i, 9);

                    // z_ang_vel / ang_vel
//                    predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementRobotErrorState.block<3,3>(dimension_error_measurement_i, 12)=
//                            jacobian_error_meas_ang_vel_wrt_error_state_robot_ang_vel;
                    BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_state, jacobian_error_meas_ang_vel_wrt_error_state_robot_ang_vel, dimension_error_measurement_i, 12);

                    // z_ang_vel / ang_acc
                    // Zeros

                    // end
                    break;
                }
                default:
                {
                    return -2;
                }

            }

            dimension_error_measurement_i+=3;
        }


        // Set From Triplets
        jacobian_error_measurement_wrt_robot_error_state.setFromTriplets(triplet_list_jacobian_error_measurement_wrt_error_state.begin(), triplet_list_jacobian_error_measurement_wrt_error_state.end());
        jacobian_error_measurement_wrt_robot_error_parameters.setFromTriplets(triplet_list_jacobian_error_measurement_wrt_error_parameters.begin(), triplet_list_jacobian_error_measurement_wrt_error_parameters.end());

    }


    /// Jacobian error measurement - inputs error state / error parameters

    {
        // Resize and init
        // Nothing to do

        // Fill
        // No dependency on global parameters -> Everything is set to zero
    }


    /// Jacobian error measurement - sensor error / error parameters

    {
        // Resize and init
        jacobian_error_measurement_wrt_sensor_error_state.resize(dimension_error_measurement_, dimension_sensor_error_state);
        jacobian_error_measurement_wrt_sensor_error_parameters.resize(dimension_error_measurement_, dimension_sensor_error_parameters);


        std::vector<Eigen::Triplet<double>> triplet_list_jacobian_error_measurement_wrt_error_state;
        std::vector<Eigen::Triplet<double>> triplet_list_jacobian_error_measurement_wrt_error_parameters;




        // Fill Jacobian
        unsigned int dimension_error_measurement_i=0;

        // z_lin_acc
        if(this->isMeasurementLinearAccelerationEnabled())
        {
            int dimension_sensor_error_state_i=0;
            int dimension_sensor_error_parameters_i=0;


            // z_lin_acc / posi_sen_wrt_robot
            if(the_imu_sensor_core->isEstimationPositionSensorWrtRobotEnabled())
            {
//                predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState.block<3,3>(dimension_error_measurement_i, dimension_sensor_error_state_i)=
//                        jacobian_error_meas_lin_acc_wrt_error_state_sensor_pos;
                BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_state, jacobian_error_meas_lin_acc_wrt_error_state_sensor_pos, dimension_error_measurement_i, dimension_sensor_error_state_i);

                dimension_sensor_error_state_i+=3;
            }
            else
            {
//                predictedMeasurement->jacobianMeasurementErrorParameters.jacobianMeasurementSensorParameters.block<3,3>(dimension_error_measurement_i, dimension_sensor_error_parameters_i)=
//                        jacobian_error_meas_lin_acc_wrt_error_state_sensor_pos;
                BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_parameters, jacobian_error_meas_lin_acc_wrt_error_state_sensor_pos, dimension_error_measurement_i, dimension_sensor_error_parameters_i);

                dimension_sensor_error_parameters_i+=3;
            }


            // z_lin_acc / atti_sen_wrt_robot
            if(the_imu_sensor_core->isEstimationAttitudeSensorWrtRobotEnabled())
            {
//                predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState.block<3,3>(dimension_error_measurement_i, dimension_sensor_error_state_i)=
//                        jacobian_error_meas_lin_acc_wrt_error_state_sensor_att;
                BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_state, jacobian_error_meas_lin_acc_wrt_error_state_sensor_att, dimension_error_measurement_i, dimension_sensor_error_state_i);

                dimension_sensor_error_state_i+=3;
            }
            else
            {
//                predictedMeasurement->jacobianMeasurementErrorParameters.jacobianMeasurementSensorParameters.block<3,3>(dimension_error_measurement_i, dimension_sensor_error_parameters_i)=
//                        jacobian_error_meas_lin_acc_wrt_error_state_sensor_att;
                BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_parameters, jacobian_error_meas_lin_acc_wrt_error_state_sensor_att, dimension_error_measurement_i, dimension_sensor_error_parameters_i);

                dimension_sensor_error_parameters_i+=3;
            }


            // z_lin_acc / ba
            if(the_imu_sensor_core->isEstimationBiasLinearAccelerationEnabled())
            {
//                predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState.block<3,3>(dimension_error_measurement_i, dimension_sensor_error_state_i)=
//                        jacobian_error_meas_lin_acc_wrt_error_state_sensor_bias_lin_acc;
                BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_state, jacobian_error_meas_lin_acc_wrt_error_state_sensor_bias_lin_acc, dimension_error_measurement_i, dimension_sensor_error_state_i);

                dimension_sensor_error_state_i+=3;
            }
            else
            {
//                predictedMeasurement->jacobianMeasurementErrorParameters.jacobianMeasurementSensorParameters.block<3,3>(dimension_error_measurement_i, dimension_sensor_error_parameters_i)=
//                        jacobian_error_meas_lin_acc_wrt_error_state_sensor_bias_lin_acc;
                BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_parameters, jacobian_error_meas_lin_acc_wrt_error_state_sensor_bias_lin_acc, dimension_error_measurement_i, dimension_sensor_error_parameters_i);

                dimension_sensor_error_parameters_i+=3;
            }


            // z_lin_acc / ka
            /*
            if(the_imu_sensor_core->isEstimationScaleLinearAccelerationEnabled())
            {
//                predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState.block<3,3>(dimension_error_measurement_i, dimension_sensor_error_state_i)=
//                        (accel_sensor_wrt_sensor+gravity_sensor).asDiagonal();
                dimension_sensor_error_state_i+=3;
            }
            else
            {
//                predictedMeasurement->jacobianMeasurementErrorParameters.jacobianMeasurementSensorParameters.block<3,3>(dimension_error_measurement_i, dimension_sensor_error_parameters_i)=
//                        (accel_sensor_wrt_sensor+gravity_sensor).asDiagonal();
                dimension_sensor_error_parameters_i+=3;
            }
            */


            // z_lin_acc / bw
            if(the_imu_sensor_core->isEstimationBiasAngularVelocityEnabled())
            {
                // Zeros
                dimension_sensor_error_state_i+=3;
            }
            else
            {
                // Zeros
                dimension_sensor_error_parameters_i+=3;
            }


            // z_lin_acc / kw
            /*
            if(the_imu_sensor_core->isEstimationScaleAngularVelocityEnabled())
            {
                // Zeros
                dimension_sensor_error_state_i+=3;
            }
            else
            {
                // Zeros
                dimension_sensor_error_parameters_i+=3;
            }
            */


            dimension_error_measurement_i+=3;
        }

        // z_atti
        if(this->isMeasurementOrientationEnabled())
        {

            int dimension_sensor_error_state_i=0;
            int dimension_sensor_error_parameters_i=0;


            // z_atti / posi_sen_wrt_robot
            if(the_imu_sensor_core->isEstimationPositionSensorWrtRobotEnabled())
            {
                // TODO
                //predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState.block<3,3>(dimension_error_measurement_i, dimension_sensor_error_state_i);
                dimension_sensor_error_state_i+=3;
            }
            else
            {
                // TODO
                //predictedMeasurement->jacobianMeasurementSensorParameters.jacobianMeasurementSensorParameters.block<3,3>(dimension_error_measurement_i, dimension_sensor_error_parameters_i);
                dimension_sensor_error_parameters_i+=3;
            }


            // z_atti / atti_sen_wrt_robot
            if(the_imu_sensor_core->isEstimationAttitudeSensorWrtRobotEnabled())
            {
                // TODO
                //predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState.block<3,3>(dimension_error_measurement_i, dimension_sensor_error_state_i);
                dimension_sensor_error_state_i+=3;
            }
            else
            {
                // TODO
                //predictedMeasurement->jacobianMeasurementSensorParameters.jacobianMeasurementSensorParameters.block<3,3>(dimension_error_measurement_i, dimension_sensor_error_parameters_i);
                dimension_sensor_error_parameters_i+=3;
            }


            // z_atti / ba
            if(the_imu_sensor_core->isEstimationBiasLinearAccelerationEnabled())
            {
                // Zeros
                dimension_sensor_error_state_i+=3;
            }
            else
            {
                // Zeros
                dimension_sensor_error_parameters_i+=3;
            }


            // z_atti / ka
            /*
            if(the_imu_sensor_core->isEstimationScaleLinearAccelerationEnabled())
            {
                // Zeros
                dimension_sensor_error_state_i+=3;
            }
            else
            {
                // Zeros
                dimension_sensor_error_parameters_i+=3;
            }
            */


            // z_atti / bw
            if(the_imu_sensor_core->isEstimationBiasAngularVelocityEnabled())
            {
                // Zeros
                dimension_sensor_error_state_i+=3;
            }
            else
            {
                // Zeros
                dimension_sensor_error_parameters_i+=3;
            }


            // z_atti / kw
            /*
            if(the_imu_sensor_core->isEstimationScaleAngularVelocityEnabled())
            {
                // Zeros
                dimension_sensor_error_state_i+=3;
            }
            else
            {
                // Zeros
                dimension_sensor_error_parameters_i+=3;
            }
            */

            dimension_error_measurement_i+=3;
        }

        // z_ang_vel
        if(this->isMeasurementAngularVelocityEnabled())
        {
            int dimension_sensor_error_state_i=0;
            int dimension_sensor_error_parameters_i=0;


            // z_ang_vel / posi_sen_wrt_robot
            if(the_imu_sensor_core->isEstimationPositionSensorWrtRobotEnabled())
            {
                // Zeros
                dimension_sensor_error_state_i+=3;
            }
            else
            {
                // Zeros
                dimension_sensor_error_parameters_i+=3;
            }


            // z_ang_vel / atti_sen_wrt_robot
            if(the_imu_sensor_core->isEstimationAttitudeSensorWrtRobotEnabled())
            {
//                predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState.block<3,3>(dimension_error_measurement_i, dimension_sensor_error_state_i)=
//                        jacobian_error_meas_ang_vel_wrt_error_state_sensor_att;
                BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_state, jacobian_error_meas_ang_vel_wrt_error_state_sensor_att, dimension_error_measurement_i, dimension_sensor_error_state_i);

                dimension_sensor_error_state_i+=3;
            }
            else
            {
//                predictedMeasurement->jacobianMeasurementErrorParameters.jacobianMeasurementSensorParameters.block<3,3>(dimension_error_measurement_i, dimension_sensor_error_parameters_i)=
//                        jacobian_error_meas_ang_vel_wrt_error_state_sensor_att;
                BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_parameters, jacobian_error_meas_ang_vel_wrt_error_state_sensor_att, dimension_error_measurement_i, dimension_sensor_error_parameters_i);

                dimension_sensor_error_parameters_i+=3;
            }


            // z_ang_vel / ba
            if(the_imu_sensor_core->isEstimationBiasLinearAccelerationEnabled())
            {
                // Zeros
                dimension_sensor_error_state_i+=3;
            }
            else
            {
                // Zeros
                dimension_sensor_error_parameters_i+=3;
            }


            // z_ang_vel / ka
            /*
            if(the_imu_sensor_core->isEstimationScaleLinearAccelerationEnabled())
            {
                // Zeros
                dimension_sensor_error_state_i+=3;
            }
            else
            {
                // Zeros
                dimension_sensor_error_parameters_i+=3;
            }
            */


            // z_ang_vel / bw
            if(the_imu_sensor_core->isEstimationBiasAngularVelocityEnabled())
            {
//                predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState.block<3,3>(dimension_error_measurement_i, dimension_sensor_error_state_i)=
//                        jacobian_error_meas_ang_vel_wrt_error_state_sensor_bias_ang_vel;
                BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_state, jacobian_error_meas_ang_vel_wrt_error_state_sensor_bias_ang_vel, dimension_error_measurement_i, dimension_sensor_error_state_i);

                dimension_sensor_error_state_i+=3;
            }
            else
            {
//                predictedMeasurement->jacobianMeasurementErrorParameters.jacobianMeasurementSensorParameters.block<3,3>(dimension_error_measurement_i, dimension_sensor_error_parameters_i)=
//                        jacobian_error_meas_ang_vel_wrt_error_state_sensor_bias_ang_vel;
                BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_parameters, jacobian_error_meas_ang_vel_wrt_error_state_sensor_bias_ang_vel, dimension_error_measurement_i, dimension_sensor_error_parameters_i);

                dimension_sensor_error_parameters_i+=3;
            }


            // z_ang_vel / kw
            /*
            if(the_imu_sensor_core->isEstimationScaleAngularVelocityEnabled())
            {
//                predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState.block<3,3>(dimension_error_measurement_i, dimension_sensor_error_state_i)=
//                        angular_velocity_imu_wrt_world_in_imu.asDiagonal();
                dimension_sensor_error_state_i+=3;
            }
            else
            {
//                predictedMeasurement->jacobianMeasurementErrorParameters.jacobianMeasurementSensorParameters.block<3,3>(dimension_error_measurement_i, dimension_sensor_error_parameters_i)=
//                        angular_velocity_imu_wrt_world_in_imu.asDiagonal();
                dimension_sensor_error_parameters_i+=3;
            }
            */


            dimension_error_measurement_i+=3;
        }


        // Set From Triplets
        jacobian_error_measurement_wrt_sensor_error_state.setFromTriplets(triplet_list_jacobian_error_measurement_wrt_error_state.begin(), triplet_list_jacobian_error_measurement_wrt_error_state.end());
        jacobian_error_measurement_wrt_sensor_error_parameters.setFromTriplets(triplet_list_jacobian_error_measurement_wrt_error_parameters.begin(), triplet_list_jacobian_error_measurement_wrt_error_parameters.end());

    }


    /// Jacobian measurement - map element error state / error parameters

    {
        // Nothing to do
    }





    ///// Jacobians error measurement - sensor noise of the measurement

    {
        // Resize and init Jacobian
        jacobian_error_measurement_wrt_error_measurement.resize(dimension_error_measurement_, dimension_error_measurement_);

        std::vector<Eigen::Triplet<double>> triplet_list_jacobian_error_measurement_wrt_error_measurement;


        // Fill Jacobian
        int dimension_error_measurement_i=0;

        // z_lin_acc
        if(this->isMeasurementLinearAccelerationEnabled())
        {
            // z_lin_acc
//            predictedMeasurement->jacobianMeasurementSensorNoise.jacobianMeasurementSensorNoise.block<3,3>(dimension_error_measurement_i, dimension_error_measurement_i)=
//                    jacobian_error_meas_lin_acc_wrt_error_meas_lin_acc;
            BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_measurement, jacobian_error_meas_lin_acc_wrt_error_meas_lin_acc, dimension_error_measurement_i, dimension_error_measurement_i);

            dimension_error_measurement_i+=3;
        }

        // z_atti
        if(this->isMeasurementOrientationEnabled())
        {
            // z_atti
//            predictedMeasurement->jacobianMeasurementSensorNoise.jacobianMeasurementSensorNoise.block<3,3>(dimension_error_measurement_i, dimension_error_measurement_i)=
//                    jacobian_error_meas_att_wrt_error_meas_att;
            BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_measurement, jacobian_error_meas_att_wrt_error_meas_att, dimension_error_measurement_i, dimension_error_measurement_i);

            dimension_error_measurement_i+=3;
        }

        // z_ang_vel
        if(this->isMeasurementAngularVelocityEnabled())
        {
            // z_ang_vel
//            predictedMeasurement->jacobianMeasurementSensorNoise.jacobianMeasurementSensorNoise.block<3,3>(dimension_error_measurement_i, dimension_error_measurement_i)=
//                    jacobian_error_meas_ang_vel_wrt_error_meas_ang_vel;
            BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_measurement, jacobian_error_meas_ang_vel_wrt_error_meas_ang_vel, dimension_error_measurement_i, dimension_error_measurement_i);

            dimension_error_measurement_i+=3;
        }


        // Set From Triplets
        jacobian_error_measurement_wrt_error_measurement.setFromTriplets(triplet_list_jacobian_error_measurement_wrt_error_measurement.begin(), triplet_list_jacobian_error_measurement_wrt_error_measurement.end());

    }


    // End
    return 0;
}

int ImuSensorCore::predictErrorMeasurementJacobianCore(// State: World
                                                       const Eigen::Vector3d& gravity_wrt_world,
                                                       // State: Robot
                                                       const Eigen::Vector3d& position_robot_wrt_world, const Eigen::Vector4d& attitude_robot_wrt_world,
                                                       const Eigen::Vector3d& lin_speed_robot_wrt_world, const Eigen::Vector3d& ang_velocity_robot_wrt_world,
                                                       const Eigen::Vector3d& lin_accel_robot_wrt_world, const Eigen::Vector3d& ang_accel_robot_wrt_world,
                                                       // State: Sensor
                                                       const Eigen::Vector3d& position_sensor_wrt_robot, const Eigen::Vector4d& attitude_sensor_wrt_robot,
                                                       // Parameters: Sensor
                                                       const Eigen::Matrix3d& sensitivity_meas_linear_acceleration, const Eigen::Matrix3d& sensitivity_meas_angular_velocity,
                                                       // Measurement
                                                       const Eigen::Vector3d& meas_lin_accel_sensor_wrt_sensor, const Eigen::Vector3d& meas_attitude_sensor_wrt_sensor, const Eigen::Vector3d& meas_ang_velocity_sensor_wrt_sensor,
                                                       // Predicted Measurement
                                                       const Eigen::Vector3d& lin_accel_sensor_wrt_sensor, const Eigen::Vector3d& attitude_sensor_wrt_sensor, const Eigen::Vector3d& ang_velocity_sensor_wrt_sensor,
                                                       // Jacobians: State and Params
                                                       Eigen::Matrix3d& jacobian_error_meas_lin_acc_wrt_error_gravity,
                                                       Eigen::Matrix3d& jacobian_error_meas_lin_acc_wrt_error_state_robot_lin_acc, Eigen::Matrix3d& jacobian_error_meas_lin_acc_wrt_error_state_robot_att, Eigen::Matrix3d& jacobian_error_meas_lin_acc_wrt_error_state_robot_ang_vel, Eigen::Matrix3d& jacobian_error_meas_lin_acc_wrt_error_state_robot_ang_acc,
                                                       Eigen::Matrix3d& jacobian_error_meas_ang_vel_wrt_error_state_robot_att, Eigen::Matrix3d& jacobian_error_meas_ang_vel_wrt_error_state_robot_ang_vel,
                                                       Eigen::Matrix3d& jacobian_error_meas_lin_acc_wrt_error_state_sensor_pos, Eigen::Matrix3d& jacobian_error_meas_lin_acc_wrt_error_state_sensor_att, Eigen::Matrix3d& jacobian_error_meas_lin_acc_wrt_error_state_sensor_bias_lin_acc,
                                                       Eigen::Matrix3d& jacobian_error_meas_ang_vel_wrt_error_state_sensor_att, Eigen::Matrix3d& jacobian_error_meas_ang_vel_wrt_error_state_sensor_bias_ang_vel,
                                                       // Jacobians: Noise
                                                       Eigen::Matrix3d& jacobian_error_meas_lin_acc_wrt_error_meas_lin_acc, Eigen::Matrix3d& jacobian_error_meas_att_wrt_error_meas_att, Eigen::Matrix3d& jacobian_error_meas_ang_vel_wrt_error_meas_ang_vel
                                                       )
{

    // Aux vars
    //
    Quaternion::Quaternion quat_attitude_sensor_wrt_world=Quaternion::cross(attitude_robot_wrt_world, attitude_sensor_wrt_robot);

    Quaternion::PureQuaternion angular_velocity_imu_wrt_world_in_imu = Quaternion::cross_sandwich( Quaternion::inv(quat_attitude_sensor_wrt_world), ang_velocity_robot_wrt_world , quat_attitude_sensor_wrt_world );

    Quaternion::PureQuaternion angular_velocity_robot_wrt_world_in_robot = Quaternion::cross_sandwich(Quaternion::inv(attitude_robot_wrt_world), ang_velocity_robot_wrt_world , attitude_robot_wrt_world);

    Quaternion::PureQuaternion angular_acceleration_robot_wrt_world_in_robot = Quaternion::cross_sandwich(Quaternion::inv(attitude_robot_wrt_world), ang_accel_robot_wrt_world , attitude_robot_wrt_world);

    //
    Eigen::Matrix4d mat_q_plus_attitude_robot_wrt_world=Quaternion::quatMatPlus(attitude_robot_wrt_world);
    Eigen::Matrix4d mat_q_minus_attitude_sensor_wrt_robot=Quaternion::quatMatMinus(attitude_sensor_wrt_robot);
    Eigen::Matrix4d mat_q_plus_attitude_world_wrt_sensor=Quaternion::quatMatPlus(Quaternion::inv(quat_attitude_sensor_wrt_world));
    Eigen::Matrix4d mat_q_minus_attitude_sensor_wrt_world=Quaternion::quatMatMinus(quat_attitude_sensor_wrt_world);
    Eigen::Matrix4d mat_q_plus_attitude_sensor_wrt_robot=Quaternion::quatMatPlus(attitude_sensor_wrt_robot);
    Eigen::Matrix4d mat_q_plus_attitude_robot_wrt_sensor=Quaternion::quatMatPlus(Quaternion::inv(attitude_sensor_wrt_robot));
    Eigen::Matrix4d mat_q_plus_attitude_world_wrt_robot=Quaternion::quatMatPlus(Quaternion::inv(attitude_robot_wrt_world));
    Eigen::Matrix4d mat_q_minus_attitude_robot_wrt_world=Quaternion::quatMatMinus(attitude_robot_wrt_world);

    //
    Eigen::Matrix4d mat_q_plus_angular_velocity_robot_wrt_world_in_world = Quaternion::quatMatPlus(ang_velocity_robot_wrt_world);
    Eigen::Matrix4d mat_q_minus_cross_angular_vel_robot_wrt_world_in_world_and_atti_imu_wrt_world = Quaternion::quatMatMinus(Quaternion::cross_pure_gen(ang_velocity_robot_wrt_world, quat_attitude_sensor_wrt_world));

    Eigen::Matrix4d mat_q_minus_cross_gravity_wrt_wolrd_and_attitude_robot_wrt_world=Quaternion::quatMatMinus(Quaternion::cross_pure_gen(gravity_wrt_world, attitude_robot_wrt_world));
    Eigen::Matrix4d mat_q_plus_gravity_wrt_world=Quaternion::quatMatPlus(gravity_wrt_world);

    Eigen::Matrix4d mat_q_minus_cross_acceleration_robot_wrt_world_and_attitude_robot_wrt_world=Quaternion::quatMatMinus(Quaternion::cross_pure_gen(lin_accel_robot_wrt_world, attitude_robot_wrt_world));
    Eigen::Matrix4d mat_q_plus_linear_acceleration_robot_wrt_world=Quaternion::quatMatPlus(lin_accel_robot_wrt_world);

    Eigen::Matrix4d mat_q_minus_cross_angular_vel_robot_wrt_world_in_world_and_atti_robot_wrt_world=Quaternion::quatMatMinus(Quaternion::cross_pure_gen(ang_velocity_robot_wrt_world, attitude_robot_wrt_world));

    Eigen::Matrix4d mat_q_minus_cross_angular_acceleration_robot_wrt_world_in_world_and_atti_robot_wrt_world=Quaternion::quatMatMinus(Quaternion::cross_pure_gen(ang_accel_robot_wrt_world, attitude_robot_wrt_world));

    Eigen::Matrix4d mat_q_plus_angular_acceleration_robot_wrt_world=Quaternion::quatMatPlus(ang_accel_robot_wrt_world);

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




    // More Aux vars
    Eigen::MatrixXd jacobianMeas_g_I_wrt_q_r_w=mat_q_plus_attitude_robot_wrt_sensor*mat_q_minus_attitude_sensor_wrt_robot*(mat_q_minus_cross_gravity_wrt_wolrd_and_attitude_robot_wrt_world*mat_diff_quat_inv_wrt_quat + mat_q_plus_attitude_world_wrt_robot*mat_q_plus_gravity_wrt_world);
    Eigen::MatrixXd jacobianMeas_a_real_wrt_q_r_w=mat_q_plus_attitude_robot_wrt_sensor*mat_q_minus_attitude_sensor_wrt_robot*(mat_q_minus_cross_acceleration_robot_wrt_world_and_attitude_robot_wrt_world* mat_diff_quat_inv_wrt_quat + mat_q_plus_attitude_world_wrt_robot* mat_q_plus_linear_acceleration_robot_wrt_world);
    Eigen::MatrixXd jacobianMeas_an_wrt_q_r_w=-2*Quaternion::skewSymMat(angular_velocity_robot_wrt_world_in_robot.cross(position_sensor_wrt_robot))*mat_diff_w_amp_wrt_w*(mat_q_minus_cross_angular_vel_robot_wrt_world_in_world_and_atti_robot_wrt_world*mat_diff_quat_inv_wrt_quat+mat_q_plus_attitude_world_wrt_robot*mat_q_plus_angular_velocity_robot_wrt_world_in_world);
    Eigen::MatrixXd jacobianMeas_at_wrt_q_r_w=-1*Quaternion::skewSymMat(position_sensor_wrt_robot)*mat_diff_w_amp_wrt_w*(mat_q_minus_cross_angular_acceleration_robot_wrt_world_in_world_and_atti_robot_wrt_world*mat_diff_quat_inv_wrt_quat+mat_q_plus_attitude_world_wrt_robot*mat_q_plus_angular_acceleration_robot_wrt_world);



    // Even more Auxiliar vars

    // Angular vel
    Eigen::Vector3d angular_vel_robot_wrt_world_in_robot=Quaternion::cross_sandwich(Quaternion::inv(attitude_robot_wrt_world), ang_velocity_robot_wrt_world ,attitude_robot_wrt_world);
    // Angular acceler
    Eigen::Vector3d angular_acc_robot_wrt_world_in_robot=Quaternion::cross_sandwich(Quaternion::inv(attitude_robot_wrt_world), ang_accel_robot_wrt_world ,attitude_robot_wrt_world);

    // Ficticious Acc: Normal wrt robot
    Eigen::Vector3d normal_acceleration=angular_vel_robot_wrt_world_in_robot.cross(angular_vel_robot_wrt_world_in_robot.cross(position_sensor_wrt_robot));
    // Ficticious Acc: Tang wrt robot
    Eigen::Vector3d tangencial_acceleration=angular_acc_robot_wrt_world_in_robot.cross(position_sensor_wrt_robot);

    // Ficticious Acc total wrt robot
    Eigen::Vector3d ficticious_acceleration=normal_acceleration+tangencial_acceleration;

    // Attitude sensor wrt world
    Eigen::Vector4d attitude_sensor_wrt_world=Quaternion::cross(attitude_robot_wrt_world, attitude_sensor_wrt_robot);

    // Acceleracion in sensor
    Eigen::Vector3d accel_sensor_wrt_sensor=Quaternion::cross_sandwich(Quaternion::inv(attitude_sensor_wrt_world), lin_accel_robot_wrt_world, attitude_sensor_wrt_world) + Quaternion::cross_sandwich(Quaternion::inv(attitude_sensor_wrt_robot), ficticious_acceleration, attitude_sensor_wrt_robot);

    // Gravity in sensor
    Eigen::Vector3d gravity_sensor=Quaternion::cross_sandwich(Quaternion::inv(attitude_sensor_wrt_world), gravity_wrt_world, attitude_sensor_wrt_world);
    // Gravity in robot
    Eigen::Vector3d gravity_robot=Quaternion::cross_sandwich(Quaternion::inv(attitude_robot_wrt_world), gravity_wrt_world, attitude_robot_wrt_world);

    // Linear acceleration in sensor wrt robot
    Eigen::Vector3d accel_robot_wrt_robot=Quaternion::cross_sandwich(Quaternion::inv(attitude_robot_wrt_world), lin_accel_robot_wrt_world, attitude_robot_wrt_world);

    // Acceleration total imu wrt robot
    Eigen::Vector3d acceleration_total_imu_wrt_robot=gravity_robot+accel_robot_wrt_robot+ficticious_acceleration;






    /// Jacobian World Gravity

    jacobian_error_meas_lin_acc_wrt_error_gravity=
            -1*sensitivity_meas_linear_acceleration*mat_diff_w_amp_wrt_w*mat_q_plus_attitude_world_wrt_sensor* mat_q_minus_attitude_sensor_wrt_robot*mat_diff_w_amp_wrt_w.transpose();




    /// Jacobian Robot

    jacobian_error_meas_lin_acc_wrt_error_state_robot_lin_acc=
            sensitivity_meas_linear_acceleration*mat_diff_w_amp_wrt_w*mat_q_plus_attitude_world_wrt_sensor* mat_q_minus_attitude_sensor_wrt_world*mat_diff_w_amp_wrt_w.transpose();

    jacobian_error_meas_lin_acc_wrt_error_state_robot_att=
            // common 1
            sensitivity_meas_linear_acceleration*mat_diff_w_amp_wrt_w*(
            // d(ga_I)/d(q_r_w)
            -jacobianMeas_g_I_wrt_q_r_w  +
            // d(a_real)/d(q_r_w)
            jacobianMeas_a_real_wrt_q_r_w +
            // d(a_ficticious)/d(q_r_w)
            ( mat_q_plus_attitude_robot_wrt_sensor*mat_q_minus_attitude_sensor_wrt_robot*mat_diff_w_amp_wrt_w.transpose()*
                (
                // an
                jacobianMeas_an_wrt_q_r_w
                +
                // at
                jacobianMeas_at_wrt_q_r_w
                )
            )
            // common 2
            )*mat_q_plus_attitude_robot_wrt_world*mat_diff_error_quat_wrt_error_theta*0.5;

    jacobian_error_meas_lin_acc_wrt_error_state_robot_ang_vel=
            sensitivity_meas_linear_acceleration*mat_diff_w_amp_wrt_w*mat_q_plus_attitude_robot_wrt_sensor*mat_q_minus_attitude_sensor_wrt_robot*mat_diff_w_amp_wrt_w.transpose()* (-2*Quaternion::skewSymMat(angular_velocity_robot_wrt_world_in_robot.cross(position_sensor_wrt_robot))) *mat_diff_w_amp_wrt_w*mat_q_plus_attitude_world_wrt_robot* mat_q_minus_attitude_robot_wrt_world*mat_diff_w_amp_wrt_w.transpose();

    jacobian_error_meas_lin_acc_wrt_error_state_robot_ang_acc=
            sensitivity_meas_linear_acceleration*mat_diff_w_amp_wrt_w*mat_q_plus_attitude_robot_wrt_sensor*mat_q_minus_attitude_sensor_wrt_robot*mat_diff_w_amp_wrt_w.transpose()*(-1*Quaternion::skewSymMat(position_sensor_wrt_robot))*mat_diff_w_amp_wrt_w*mat_q_plus_attitude_world_wrt_robot*mat_q_minus_attitude_robot_wrt_world*mat_diff_w_amp_wrt_w.transpose();


    jacobian_error_meas_ang_vel_wrt_error_state_robot_att=
            sensitivity_meas_angular_velocity*mat_diff_w_amp_wrt_w*( mat_q_minus_cross_angular_vel_robot_wrt_world_in_world_and_atti_imu_wrt_world*mat_diff_quat_inv_wrt_quat + mat_q_plus_attitude_world_wrt_sensor*mat_q_plus_angular_velocity_robot_wrt_world_in_world )*mat_q_minus_attitude_sensor_wrt_robot*mat_q_plus_attitude_robot_wrt_world*mat_diff_error_quat_wrt_error_theta*0.5;

    jacobian_error_meas_ang_vel_wrt_error_state_robot_ang_vel=
            sensitivity_meas_angular_velocity*mat_diff_w_amp_wrt_w* mat_q_plus_attitude_world_wrt_sensor*mat_q_minus_attitude_sensor_wrt_world *mat_diff_w_amp_wrt_w.transpose();



    /// Jacobian Sensor
    jacobian_error_meas_lin_acc_wrt_error_state_sensor_pos=
            sensitivity_meas_linear_acceleration*mat_diff_w_amp_wrt_w*mat_q_plus_attitude_robot_wrt_sensor*mat_q_plus_attitude_sensor_wrt_robot*mat_diff_w_amp_wrt_w.transpose()*(Quaternion::skewSymMat( angular_velocity_robot_wrt_world_in_robot.cross(angular_velocity_robot_wrt_world_in_robot) )+Quaternion::skewSymMat( angular_acceleration_robot_wrt_world_in_robot ));

    // TODO FIX!
    jacobian_error_meas_lin_acc_wrt_error_state_sensor_att=
            sensitivity_meas_linear_acceleration*mat_diff_w_amp_wrt_w*( Quaternion::quatMatMinus(Quaternion::cross_pure_gen(acceleration_total_imu_wrt_robot, attitude_sensor_wrt_robot))*mat_diff_quat_inv_wrt_quat + mat_q_plus_attitude_robot_wrt_sensor*Quaternion::quatMatPlus(acceleration_total_imu_wrt_robot) )*mat_q_plus_attitude_sensor_wrt_robot*mat_diff_error_quat_wrt_error_theta*0.5;

    jacobian_error_meas_lin_acc_wrt_error_state_sensor_bias_lin_acc=
            Eigen::Matrix3d::Identity(3,3);



    jacobian_error_meas_ang_vel_wrt_error_state_sensor_att=
            sensitivity_meas_angular_velocity*mat_diff_w_amp_wrt_w* ( mat_q_minus_cross_angular_vel_robot_wrt_world_in_world_and_atti_imu_wrt_world*mat_diff_quat_inv_wrt_quat + mat_q_plus_attitude_world_wrt_sensor*mat_q_plus_angular_velocity_robot_wrt_world_in_world ) *mat_q_plus_attitude_robot_wrt_world*mat_q_plus_attitude_sensor_wrt_robot*mat_diff_error_quat_wrt_error_theta*0.5;

    jacobian_error_meas_ang_vel_wrt_error_state_sensor_bias_ang_vel=
            Eigen::Matrix3d::Identity(3,3);


    /// Jacobians: Noise

    jacobian_error_meas_lin_acc_wrt_error_meas_lin_acc=
            Eigen::MatrixXd::Identity(3, 3);

    // TODO
    jacobian_error_meas_att_wrt_error_meas_att=
            Eigen::MatrixXd::Identity(3, 3);

    jacobian_error_meas_ang_vel_wrt_error_meas_ang_vel=
            Eigen::MatrixXd::Identity(3, 3);


    /// End
    return 0;
}

int ImuSensorCore::resetErrorStateJacobian(// Time
                                            const TimeStamp& current_time_stamp,
                                            // Increment Error State
                                            const Eigen::VectorXd& increment_error_state,
                                            // Current State
                                            std::shared_ptr<StateCore>& current_state
                                            )
{
    // Checks
    if(!current_state)
        return -1;


    // Resize Jacobian
    current_state->jacobian_error_state_reset_.resize(this->dimension_error_state_, this->dimension_error_state_);

    // Fill
    std::vector< Eigen::Triplet<double> > triplets_jacobian_error_reset;

    int dimension_error_state_i=0;

    // Position Sensor Wrt Robot
    if(this->isEstimationPositionSensorWrtRobotEnabled())
    {
        for(int i=0; i<3; i++)
            triplets_jacobian_error_reset.push_back(Eigen::Triplet<double>(dimension_error_state_i+i, dimension_error_state_i+i, 1.0));

        dimension_error_state_i+=3;
    }

    // Attitude Sensor Wrt Robot
    if(this->isEstimationAttitudeSensorWrtRobotEnabled())
    {
        // Error Reset Matrixes
        Eigen::Matrix3d G_update_theta_robot=Eigen::Matrix3d::Identity(3,3);


        // Ojo, signo cambiado por la definicion de incrementError!
        G_update_theta_robot-=Quaternion::skewSymMat(0.5*increment_error_state.block<3,1>(dimension_error_state_i,0));

        // Triplets
        BlockMatrix::insertVectorEigenTripletFromEigenDense(triplets_jacobian_error_reset, G_update_theta_robot, dimension_error_state_i, dimension_error_state_i);

        dimension_error_state_i+=3;
    }

    // Bias Linear Acceleration
    if(this->isEstimationBiasLinearAccelerationEnabled())
    {
        for(int i=0; i<3; i++)
            triplets_jacobian_error_reset.push_back(Eigen::Triplet<double>(dimension_error_state_i+i, dimension_error_state_i+i, 1.0));

        dimension_error_state_i+=3;
    }

    // Bias Angular Acceleration
    if(this->isEstimationBiasAngularVelocityEnabled())
    {
        for(int i=0; i<3; i++)
            triplets_jacobian_error_reset.push_back(Eigen::Triplet<double>(dimension_error_state_i+i, dimension_error_state_i+i, 1.0));

        dimension_error_state_i+=3;
    }


    current_state->jacobian_error_state_reset_.setFromTriplets(triplets_jacobian_error_reset.begin(), triplets_jacobian_error_reset.end());

    // End
    return 0;
}
