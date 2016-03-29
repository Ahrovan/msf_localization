
#include "msf_localization_core/coded_visual_marker_eye_core.h"

// Circular Dependency
#include "msf_localization_core/msf_storage_core.h"


CodedVisualMarkerEyeCore::CodedVisualMarkerEyeCore() :
    SensorCore()
{
    //
    init();

    // End
    return;
}

CodedVisualMarkerEyeCore::CodedVisualMarkerEyeCore(std::weak_ptr<SensorCore> the_sensor_core, std::weak_ptr<MsfStorageCore> the_msf_storage_core) :
    SensorCore(the_sensor_core, the_msf_storage_core)
{
    //
    init();

    return;
}

CodedVisualMarkerEyeCore::~CodedVisualMarkerEyeCore()
{

    return;
}

int CodedVisualMarkerEyeCore::init()
{
    // Sensor Type
    setSensorType(SensorTypes::coded_visual_marker_eye);


    // Sensor name -> Default
    sensor_name_="coded_visual_marker_eye";

    // Flags measurement
    flag_measurement_position_=false;
    flag_measurement_attitude_=false;

    // Flags estimation -> By default are considered parameters
    // none

    // State -> Again just in case
    dimensionErrorState=0;
    dimensionState=0;

    // Parameters
    dimensionParameters+=0;
    dimensionErrorParameters+=0;

    // Dimension measurements -> Again just in case
    dimensionMeasurement=0;
    dimensionErrorMeasurement=0;

    // Dimension noise -> Again just in case
    dimensionNoise=0;

    // Noises measurements
    noise_measurement_position_.setZero();
    noise_measurement_attitude_.setZero();

    // Noises parameters
    // None

    // Noises estimation
    // None

    return 0;
}

bool CodedVisualMarkerEyeCore::isMeasurementPositionEnabled() const
{
    return this->flag_measurement_position_;
}

int CodedVisualMarkerEyeCore::enableMeasurementPosition()
{
    if(!this->flag_measurement_position_)
    {
        this->flag_measurement_position_=true;
        this->dimensionMeasurement+=3;
        dimensionErrorMeasurement+=3;
    }
    return 0;
}

Eigen::Matrix3d CodedVisualMarkerEyeCore::getNoiseMeasurementPosition() const
{
    return this->noise_measurement_position_;
}

int CodedVisualMarkerEyeCore::setNoiseMeasurementPosition(Eigen::Matrix3d noise_measurement_position)
{
    this->noise_measurement_position_=noise_measurement_position;
    return 0;
}

bool CodedVisualMarkerEyeCore::isMeasurementAttitudeEnabled() const
{
    return this->flag_measurement_attitude_;
}

int CodedVisualMarkerEyeCore::enableMeasurementAttitude()
{
    if(!this->flag_measurement_attitude_)
    {
        this->flag_measurement_attitude_=true;
        this->dimensionMeasurement+=4;
        dimensionErrorMeasurement+=3;
    }
    return 0;
}

Eigen::Matrix3d CodedVisualMarkerEyeCore::getNoiseMeasurementAttitude() const
{
    return this->noise_measurement_attitude_;
}

int CodedVisualMarkerEyeCore::setNoiseMeasurementAttitude(Eigen::Matrix3d noise_measurement_attitude)
{
    this->noise_measurement_attitude_=noise_measurement_attitude;
    return 0;
}

int CodedVisualMarkerEyeCore::setMeasurement(const TimeStamp the_time_stamp, std::shared_ptr<CodedVisualMarkerMeasurementCore> the_visual_marker_measurement)
{
    if(!isSensorEnabled())
        return 0;

    if(this->getTheMsfStorageCore()->setMeasurement(the_time_stamp, the_visual_marker_measurement))
    {
        std::cout<<"CodedVisualMarkerEyeCore::setMeasurement() error"<<std::endl;
        return 1;
    }

    return 0;
}

Eigen::SparseMatrix<double> CodedVisualMarkerEyeCore::getCovarianceMeasurement()
{
    Eigen::SparseMatrix<double> covariances_matrix;

    covariances_matrix.resize(this->getDimensionErrorMeasurement(), this->getDimensionErrorMeasurement());
    //covariances_matrix.setZero();
    covariances_matrix.reserve(this->getDimensionErrorMeasurement());

    std::vector<Eigen::Triplet<double> > tripletCovarianceMeasurement;

    unsigned int dimension=0;
    if(this->isMeasurementPositionEnabled())
    {
        //covariances_matrix.block<3,3>(dimension, dimension)=this->getNoiseMeasurementPosition();

        for(int i=0; i<3; i++)
            tripletCovarianceMeasurement.push_back(Eigen::Triplet<double>(dimension+i,dimension+i,noise_measurement_position_(i,i)));


        dimension+=3;
    }
    if(this->isMeasurementAttitudeEnabled())
    {
        //covariances_matrix.block<3,3>(dimension, dimension)=this->getNoiseMeasurementAttitude();

        for(int i=0; i<3; i++)
            tripletCovarianceMeasurement.push_back(Eigen::Triplet<double>(dimension+i,dimension+i,noise_measurement_attitude_(i,i)));


        dimension+=3;
    }

    covariances_matrix.setFromTriplets(tripletCovarianceMeasurement.begin(), tripletCovarianceMeasurement.end());

    return covariances_matrix;
}

Eigen::SparseMatrix<double> CodedVisualMarkerEyeCore::getCovarianceParameters()
{
    Eigen::SparseMatrix<double> covariances_matrix;
    covariances_matrix.resize(this->getDimensionErrorParameters(), this->getDimensionErrorParameters());
    //covariances_matrix.setZero();
    covariances_matrix.reserve(this->getDimensionErrorParameters());

    std::vector<Eigen::Triplet<double> > tripletCovarianceParameters;

    unsigned int dimension=0;
    if(!this->isEstimationPositionSensorWrtRobotEnabled())
    {
        //covariances_matrix.block<3,3>(dimension, dimension)=this->getNoisePositionSensorWrtRobot();

        for(int i=0; i<3; i++)
            tripletCovarianceParameters.push_back(Eigen::Triplet<double>(dimension+i,dimension+i,noisePositionSensorWrtRobot(i,i)));

        dimension+=3;
    }
    if(!this->isEstimationAttitudeSensorWrtRobotEnabled())
    {
        //covariances_matrix.block<3,3>(dimension, dimension)=this->getNoiseAttitudeSensorWrtRobot();

        for(int i=0; i<3; i++)
            tripletCovarianceParameters.push_back(Eigen::Triplet<double>(dimension+i,dimension+i,noiseAttitudeSensorWrtRobot(i,i)));

        dimension+=3;
    }

    covariances_matrix.setFromTriplets(tripletCovarianceParameters.begin(), tripletCovarianceParameters.end());


    return covariances_matrix;
}

int CodedVisualMarkerEyeCore::prepareInitErrorStateVariance()
{
    int error=SensorCore::prepareInitErrorStateVariance();

    if(error)
        return error;


    int point=0;
    if(this->isEstimationPositionSensorWrtRobotEnabled())
    {
        this->InitErrorStateVariance.block<3,3>(point,point)=this->getNoisePositionSensorWrtRobot();
        point+=3;
    }
    if(this->isEstimationAttitudeSensorWrtRobotEnabled())
    {
        this->InitErrorStateVariance.block<3,3>(point,point)=this->getNoiseAttitudeSensorWrtRobot();
        point+=3;
    }



    return 0;
}

Eigen::SparseMatrix<double> CodedVisualMarkerEyeCore::getCovarianceNoise(const TimeStamp deltaTimeStamp) const
{
    Eigen::SparseMatrix<double> covariance_noise;

    // Dimension noise
    int dimension_noise=getDimensionNoise();


    // Resize
    covariance_noise.resize(dimension_noise, dimension_noise);
    //covariance_noise.setZero();

    // Fill
    int dimension_noise_i=0;

    // Nothing


    // End
    return covariance_noise;
}

int CodedVisualMarkerEyeCore::predictState(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, const std::shared_ptr<SensorStateCore> pastStateI, std::shared_ptr<SensorStateCore>& predictedStateI)
{

    // Poly
    std::shared_ptr<CodedVisualMarkerEyeStateCore> pastState=std::static_pointer_cast<CodedVisualMarkerEyeStateCore>(pastStateI);
    std::shared_ptr<CodedVisualMarkerEyeStateCore> predictedState=std::static_pointer_cast<CodedVisualMarkerEyeStateCore>(predictedStateI);



    // Checks in the past state
    if(pastState->getTheSensorCoreWeak().expired())
    {
        return -5;
        std::cout<<"FreeModelRobotCore::predictState() error !pastState->getTheRobotCore()"<<std::endl;
    }


    // Create the predicted state if it doesn't exist
    if(!predictedState)
    {
        predictedState=std::make_shared<CodedVisualMarkerEyeStateCore>(pastState->getTheSensorCoreWeak());
    }

    // Set The robot core if it doesn't exist
    if(predictedState->getTheSensorCoreWeak().expired())
    {
        predictedState->setTheSensorCore(pastState->getTheSensorCoreWeak());
    }


    // Equations


    //Delta Time
    TimeStamp DeltaTime=currentTimeStamp-previousTimeStamp;
    double dt=DeltaTime.get_double();


    /// Position
    if(this->isEstimationPositionSensorWrtRobotEnabled())
    {
        // Estimation
        predictedState->positionSensorWrtRobot=pastState->positionSensorWrtRobot;
    }
    else
    {
        // Parameter
        predictedState->positionSensorWrtRobot=pastState->positionSensorWrtRobot;
    }


    /// Attitude
    if(this->isEstimationAttitudeSensorWrtRobotEnabled())
    {
        predictedState->attitudeSensorWrtRobot=pastState->attitudeSensorWrtRobot;
    }
    else
    {
        predictedState->attitudeSensorWrtRobot=pastState->attitudeSensorWrtRobot;
    }


    predictedStateI=predictedState;

    return 0;
}

// Jacobian
int CodedVisualMarkerEyeCore::predictStateErrorStateJacobians(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, std::shared_ptr<SensorStateCore> pastStateI, std::shared_ptr<SensorStateCore>& predictedStateI)
{

    // Poly
    std::shared_ptr<CodedVisualMarkerEyeStateCore> pastState=std::static_pointer_cast<CodedVisualMarkerEyeStateCore>(pastStateI);
    std::shared_ptr<CodedVisualMarkerEyeStateCore> predictedState=std::static_pointer_cast<CodedVisualMarkerEyeStateCore>(predictedStateI);


    // Create the predicted state if it doesn't exist
    if(!predictedState)
    {
        return 1;
    }


    //Delta Time
    TimeStamp DeltaTime=currentTimeStamp-previousTimeStamp;
    // delta time
    double dt=DeltaTime.get_double();


    ///// Jacobian Error State

    /// Jacobian of the error: Linear Part

    // posi / posi
    if(this->isEstimationPositionSensorWrtRobotEnabled())
    {
        predictedState->error_state_jacobian_.position_sensor_wrt_robot_.resize(3, 3);
        predictedState->error_state_jacobian_.position_sensor_wrt_robot_.setZero();

        predictedState->error_state_jacobian_.position_sensor_wrt_robot_.block<3,3>(0,0)=Eigen::MatrixXd::Identity(3,3);
    }


    /// Jacobian of the error -> Angular Part

    // att / att
    if(this->isEstimationAttitudeSensorWrtRobotEnabled())
    {
        predictedState->error_state_jacobian_.attitude_sensor_wrt_robot_.resize(3, 3);
        predictedState->error_state_jacobian_.attitude_sensor_wrt_robot_.setZero();

        predictedState->error_state_jacobian_.attitude_sensor_wrt_robot_.block<3,3>(0,0)=Eigen::MatrixXd::Identity(3,3);
    }


    //// Jacobian Error State Noise

    // Resize the jacobian
    predictedState->jacobian_error_state_noise_.resize(getDimensionErrorState(), getDimensionNoise());

    // Fill

    // Nothing to do



    //// Finish
    predictedStateI=predictedState;

    // End
    return 0;
}


int CodedVisualMarkerEyeCore::predictMeasurement(const TimeStamp theTimeStamp, const std::shared_ptr<RobotStateCore> currentRobotState, const std::shared_ptr<CodedVisualMarkerEyeStateCore> currentSensorState, const std::shared_ptr<CodedVisualMarkerLandmarkStateCore> currentMapElementState, std::shared_ptr<CodedVisualMarkerMeasurementCore>& predictedMeasurement)
{
    // TODO

    return 0;
}



int CodedVisualMarkerEyeCore::jacobiansMeasurements(const TimeStamp theTimeStamp, const std::shared_ptr<RobotStateCore> currentRobotState, const std::shared_ptr<CodedVisualMarkerEyeStateCore> currentSensorState, const std::shared_ptr<CodedVisualMarkerLandmarkStateCore> currentMapElementState, std::shared_ptr<CodedVisualMarkerMeasurementCore>& predictedMeasurement)
{
    // TODO


    // sensor core
    std::shared_ptr<const CodedVisualMarkerEyeCore> the_sensor_core=std::dynamic_pointer_cast<const CodedVisualMarkerEyeCore>(currentSensorState->getTheSensorCoreShared());


    // dimension of the measurement
    int dimension_error_measurement=the_sensor_core->getTheSensorCore()->getDimensionErrorMeasurement();


    // Robot
    std::shared_ptr<FreeModelRobotStateCore> TheRobotStateCoreAux=std::static_pointer_cast<FreeModelRobotStateCore>(currentRobotState);
    std::shared_ptr<const FreeModelRobotCore> TheRobotCoreAux=std::static_pointer_cast<const FreeModelRobotCore>(TheRobotStateCoreAux->getTheRobotCore());


    // TODO



    /// Jacobians measurement - sensor noise of the measurement

    // Resize and init Jacobian
    predictedMeasurement->jacobianMeasurementSensorNoise.jacobianMeasurementSensorNoise.resize(dimensionErrorMeasurement, dimensionErrorMeasurement);
    predictedMeasurement->jacobianMeasurementSensorNoise.jacobianMeasurementSensorNoise.setZero();

    // Fill Jacobian
    predictedMeasurement->jacobianMeasurementSensorNoise.jacobianMeasurementSensorNoise=Eigen::MatrixXd::Identity(dimensionErrorMeasurement, dimensionErrorMeasurement);




    return 0;
}

