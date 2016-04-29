
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

CodedVisualMarkerEyeCore::CodedVisualMarkerEyeCore(std::weak_ptr<MsfStorageCore> the_msf_storage_core) :
    SensorCore(the_msf_storage_core)
{
    //std::cout<<"CodedVisualMarkerEyeCore::CodedVisualMarkerEyeCore(std::weak_ptr<MsfStorageCore> the_msf_storage_core)"<<std::endl;

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


    // Flags measurement
    flag_measurement_position_=false;
    flag_measurement_attitude_=false;

    // Flags estimation -> By default are considered parameters
    // none

    // State -> Again just in case
    dimension_error_state_=0;
    dimension_state_=0;

    // Parameters
    dimension_parameters_+=0;
    dimension_error_parameters_+=0;

    // Dimension measurements -> Again just in case
    dimension_measurement_=0;
    dimension_error_measurement_=0;

    // Dimension noise -> Again just in case
    dimension_noise_=0;

    // Noises measurements
    noise_measurement_position_.setZero();
    noise_measurement_attitude_.setZero();

    // Noises parameters
    // None

    // Noises estimation
    // None

    return 0;
}

int CodedVisualMarkerEyeCore::readConfig(pugi::xml_node sensor, unsigned int sensorId, std::shared_ptr<CodedVisualMarkerEyeStateCore>& SensorInitStateCore)
{
    // Create a class for the SensorStateCore
    if(!SensorInitStateCore)
        SensorInitStateCore=std::make_shared<CodedVisualMarkerEyeStateCore>(this->getMsfElementCoreWeakPtr());

    // Set Id
    this->setSensorId(sensorId);


    // Auxiliar reading value
    std::string readingValue;



    //// Sensor configurations


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

    // None



    //// Measurements
    pugi::xml_node measurements = sensor.child("measurements");

    /// Orientation
    pugi::xml_node meas_orientation = measurements.child("orientation");

    readingValue=meas_orientation.child_value("enabled");
    if(std::stoi(readingValue))
        this->enableMeasurementAttitude();

    readingValue=meas_orientation.child_value("var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoiseMeasurementAttitude(variance.asDiagonal());
    }


    /// Position
    pugi::xml_node meas_position = measurements.child("position");

    readingValue=meas_position.child_value("enabled");
    if(std::stoi(readingValue))
        this->enableMeasurementPosition();

    readingValue=meas_position.child_value("var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoiseMeasurementPosition(variance.asDiagonal());
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

    // None



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

    // None


    // Noises in the estimation (if enabled)

    // None


    // Prepare covariance matrix
    this->prepareCovarianceInitErrorState();


    /// Finish


    // End
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
        this->dimension_measurement_+=3;
        dimension_error_measurement_+=3;
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
        this->dimension_measurement_+=4;
        dimension_error_measurement_+=3;
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

/*
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
*/

int CodedVisualMarkerEyeCore::setMeasurementList(const TimeStamp the_time_stamp, std::list< std::shared_ptr<SensorMeasurementCore> > the_visual_marker_measurement_list)
{
    if(!isSensorEnabled())
        return 0;

    if(this->getMsfStorageCoreSharedPtr()->setMeasurementList(the_time_stamp, the_visual_marker_measurement_list))
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

int CodedVisualMarkerEyeCore::prepareCovarianceInitErrorState()
{
    int error=MsfElementCore::prepareCovarianceInitErrorState();

    if(error)
        return error;


    int point=0;
    if(this->isEstimationPositionSensorWrtRobotEnabled())
    {
        this->covariance_init_error_state_.block<3,3>(point,point)=this->getNoisePositionSensorWrtRobot();
        point+=3;
    }
    if(this->isEstimationAttitudeSensorWrtRobotEnabled())
    {
        this->covariance_init_error_state_.block<3,3>(point,point)=this->getNoiseAttitudeSensorWrtRobot();
        point+=3;
    }


    return 0;
}

Eigen::SparseMatrix<double> CodedVisualMarkerEyeCore::getCovarianceNoise(const TimeStamp deltaTimeStamp)
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
    if(!pastState->isCorrect())
    {
        std::cout<<"FreeModelRobotCore::predictState() error !pastState->getTheRobotCore()"<<std::endl;
        return -5;
    }


    // Create the predicted state if it doesn't exist
    if(!predictedState)
    {
        predictedState=std::make_shared<CodedVisualMarkerEyeStateCore>(pastState->getMsfElementCoreSharedPtr());
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
int CodedVisualMarkerEyeCore::predictErrorStateJacobians(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, std::shared_ptr<SensorStateCore> pastStateI, std::shared_ptr<SensorStateCore>& predictedStateI)
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
    // TODO Fix!!
    if(this->isEstimationAttitudeSensorWrtRobotEnabled())
    {
        predictedState->error_state_jacobian_.attitude_sensor_wrt_robot_.resize(3, 3);
        predictedState->error_state_jacobian_.attitude_sensor_wrt_robot_.setZero();

        predictedState->error_state_jacobian_.attitude_sensor_wrt_robot_.block<3,3>(0,0)=Eigen::MatrixXd::Identity(3,3);
    }


    /// Convert to Eigen::Sparse<double> and store in jacobian_error_state_
    {
        predictedState->jacobian_error_state_.resize(dimension_error_state_, dimension_error_state_);
        predictedState->jacobian_error_state_.reserve(3*dimension_error_state_); //worst case -> Optimize

        std::vector<Eigen::Triplet<double> > tripletListErrorJacobian;

        int dimension_error_state_i=0;

        // Position sensor wrt robot
        if(this->isEstimationPositionSensorWrtRobotEnabled())
        {
            // Add to triplet list
            for(int i=0; i<3; i++)
                for(int j=0; j<3; j++)
                    tripletListErrorJacobian.push_back(Eigen::Triplet<double>(dimension_error_state_i+i,dimension_error_state_i+j, predictedState->error_state_jacobian_.position_sensor_wrt_robot_(i,j)));

            // Update dimension for next
            dimension_error_state_i+=3;
        }

        // Attitude sensor wrt robot
        if(this->isEstimationAttitudeSensorWrtRobotEnabled())
        {
            // Add to triplet list
            for(int i=0; i<3; i++)
                for(int j=0; j<3; j++)
                    tripletListErrorJacobian.push_back(Eigen::Triplet<double>(dimension_error_state_i+i,dimension_error_state_i+j, predictedState->error_state_jacobian_.attitude_sensor_wrt_robot_(i,j)));

            // Update dimension for next
            dimension_error_state_i+=3;
        }


        // Set from triplets
        predictedState->jacobian_error_state_.setFromTriplets(tripletListErrorJacobian.begin(), tripletListErrorJacobian.end());

    }



    //// Jacobian Error State Noise

    // Resize the jacobian
    predictedState->jacobian_error_state_noise_.resize(getDimensionErrorState(), getDimensionNoise());
    predictedState->jacobian_error_state_noise_.setZero();

    // Fill

    // Nothing to do



    //// Finish
    predictedStateI=predictedState;

    // End
    return 0;
}


int CodedVisualMarkerEyeCore::predictMeasurement(const TimeStamp theTimeStamp, const std::shared_ptr<GlobalParametersStateCore> TheGlobalParametersStateCore, const std::shared_ptr<RobotStateCore> currentRobotState, const std::shared_ptr<SensorStateCore> currentSensorStateI, const std::shared_ptr<MapElementStateCore> currentMapElementStateI, std::shared_ptr<CodedVisualMarkerMeasurementCore>& predictedMeasurement)
{
#if _DEBUG_SENSOR_CORE
    logFile<<"CodedVisualMarkerEyeCore::predictMeasurement() TS: sec="<<theTimeStamp.sec<<" s; nsec="<<theTimeStamp.nsec<<" ns"<<std::endl;
#endif

    /// Check
    if(!this->isCorrect())
    {
        std::cout<<"CodedVisualMarkerEyeCore::predictMeasurement() error 50"<<std::endl;
        return 50;
    }

    /// Check global parameters
    if(!TheGlobalParametersStateCore)
        return 1;

    if(!TheGlobalParametersStateCore->isCorrect())
        return 1;

    /// check Robot
    if(!currentRobotState)
    {
        std::cout<<"CodedVisualMarkerEyeCore::predictMeasurement() error 2"<<std::endl;
        return 2;
    }

    if(!currentRobotState->isCorrect())
    {
        std::cout<<"CodedVisualMarkerEyeCore::predictMeasurement() error 3"<<std::endl;
        return 3;
    }

    /// Check sensor
    if(!currentSensorStateI)
    {
        std::cout<<"CodedVisualMarkerEyeCore::predictMeasurement() error 1"<<std::endl;
        return 1;
    }

    if(!currentSensorStateI->isCorrect())
    {
        std::cout<<"CodedVisualMarkerEyeCore::predictMeasurement() error 1"<<std::endl;
        return 1;
    }

    // Cast
    std::shared_ptr<CodedVisualMarkerEyeStateCore> currentSensorState=std::dynamic_pointer_cast<CodedVisualMarkerEyeStateCore>(currentSensorStateI);


    /// Check Map element
    if(!currentMapElementStateI)
    {
        std::cout<<"CodedVisualMarkerEyeCore::predictMeasurement() error 2"<<std::endl;
        return 4;
    }

    if(!currentMapElementStateI->isCorrect())
    {
        std::cout<<"CodedVisualMarkerEyeCore::predictMeasurement() error 2"<<std::endl;
        return 5;
    }

    // Cast
    std::shared_ptr<CodedVisualMarkerLandmarkStateCore> currentMapElementState=std::dynamic_pointer_cast<CodedVisualMarkerLandmarkStateCore>(currentMapElementStateI);

    // Checks
    // TODO


    /// Create pointer
    if(!predictedMeasurement)
    {
        std::shared_ptr<CodedVisualMarkerEyeCore> TheCodedVisualMarkerEyeCore=std::dynamic_pointer_cast<CodedVisualMarkerEyeCore>(this->getMsfElementCoreSharedPtr());
        predictedMeasurement=std::make_shared<CodedVisualMarkerMeasurementCore>(TheCodedVisualMarkerEyeCore);
#if _DEBUG_SENSOR_CORE
        logFile<<"CodedVisualMarkerEyeCore::predictMeasurement() pointer created"<<std::endl;
#endif
    }



    /// id -> Needed
    std::shared_ptr<CodedVisualMarkerLandmarkCore> TheCodedVisualMarkerLandmarkCore=std::dynamic_pointer_cast<CodedVisualMarkerLandmarkCore>(currentMapElementState->getMsfElementCoreSharedPtr());
    predictedMeasurement->setVisualMarkerId(TheCodedVisualMarkerLandmarkCore->getId());




    /// Variables needed
    // Robot
    Eigen::Vector3d position_robot_wrt_world;
    Eigen::Vector4d attitude_robot_wrt_world;
    // Sensor
    Eigen::Vector3d position_sensor_wrt_robot;
    Eigen::Vector4d attitude_sensor_wrt_robot;
    // Map Element
    Eigen::Vector3d position_map_element_wrt_world;
    Eigen::Vector4d attitude_map_element_wrt_world;


    // Switch depending on robot used
    switch(std::dynamic_pointer_cast<RobotCore>(currentRobotState->getMsfElementCoreSharedPtr())->getRobotCoreType())
    {
        // Free model robot
        case RobotCoreTypes::free_model:
        {
            // Cast
            std::shared_ptr<FreeModelRobotStateCore> currentFreeModelRobotState=std::dynamic_pointer_cast<FreeModelRobotStateCore>(currentRobotState);

            // Set Values of variables
            position_robot_wrt_world=currentFreeModelRobotState->getPosition();
            attitude_robot_wrt_world=currentFreeModelRobotState->getAttitude();

            break;
        }

        // Imu Driven robot
        case RobotCoreTypes::imu_driven:
        {
            // Cast
            std::shared_ptr<ImuDrivenRobotStateCore> currentFreeModelRobotState=std::dynamic_pointer_cast<ImuDrivenRobotStateCore>(currentRobotState);

            // Robot
            position_robot_wrt_world=currentFreeModelRobotState->getPositionRobotWrtWorld();
            attitude_robot_wrt_world=currentFreeModelRobotState->getAttitudeRobotWrtWorld();

            break;
        }

        // Default
        default:
            return -1000;
    }


    // Sensor
    position_sensor_wrt_robot=currentSensorState->getPositionSensorWrtRobot();
    attitude_sensor_wrt_robot=currentSensorState->getAttitudeSensorWrtRobot();


    // Map Element
    position_map_element_wrt_world=currentMapElementState->getPosition();
    attitude_map_element_wrt_world=currentMapElementState->getAttitude();



    /// Prediction Core
    int error_predict_measurement_core=this->predictMeasurementCore(position_robot_wrt_world, attitude_robot_wrt_world, position_sensor_wrt_robot, attitude_sensor_wrt_robot, position_map_element_wrt_world, attitude_map_element_wrt_world, predictedMeasurement);

    if(error_predict_measurement_core)
        return error_predict_measurement_core;



#if _DEBUG_SENSOR_CORE
    logFile<<"CodedVisualMarkerEyeCore::predictMeasurement() ended TS: sec="<<theTimeStamp.sec<<" s; nsec="<<theTimeStamp.nsec<<" ns"<<std::endl;
#endif


    // End
    return 0;
}

int CodedVisualMarkerEyeCore::predictMeasurementCore(Eigen::Vector3d position_robot_wrt_world, Eigen::Vector4d attitude_robot_wrt_world, Eigen::Vector3d position_sensor_wrt_robot, Eigen::Vector4d attitude_sensor_wrt_robot, Eigen::Vector3d position_map_element_wrt_world, Eigen::Vector4d attitude_map_element_wrt_world, std::shared_ptr<CodedVisualMarkerMeasurementCore>& predictedMeasurement)
{
    // Aux vars
    Eigen::Vector4d attitude_visual_marker_eye_wrt_world=
            Quaternion::cross(attitude_robot_wrt_world, attitude_sensor_wrt_robot);


    // Position
    if(this->isMeasurementPositionEnabled())
    {
        // Aux Variable
        Eigen::Vector3d position_visual_marker_wrt_visual_marker_eye;
        position_visual_marker_wrt_visual_marker_eye.setZero();


        Eigen::Vector3d position_visual_marker_eye_wrt_world=
                Quaternion::cross_sandwich(attitude_robot_wrt_world, position_sensor_wrt_robot, Quaternion::inv(attitude_robot_wrt_world));


        // Equation
        position_visual_marker_wrt_visual_marker_eye=
                Quaternion::cross_sandwich(Quaternion::inv(attitude_visual_marker_eye_wrt_world), position_map_element_wrt_world-position_visual_marker_eye_wrt_world-position_robot_wrt_world, attitude_visual_marker_eye_wrt_world);

        // Set
        predictedMeasurement->setVisualMarkerPosition(position_visual_marker_wrt_visual_marker_eye);
    }



    // Attitude
    if(this->isMeasurementAttitudeEnabled())
    {
        // Aux Variable
        Eigen::Vector4d attitude_visual_marker_wrt_visual_marker_eye;
        attitude_visual_marker_wrt_visual_marker_eye.setZero();

        // Equation
        attitude_visual_marker_wrt_visual_marker_eye=
                Quaternion::cross(Quaternion::inv(attitude_visual_marker_eye_wrt_world), attitude_map_element_wrt_world);


        // Set
        predictedMeasurement->setVisualMarkerAttitude(attitude_visual_marker_wrt_visual_marker_eye);
    }

    return 0;
}


int CodedVisualMarkerEyeCore::jacobiansErrorMeasurements(const TimeStamp theTimeStamp, const std::shared_ptr<GlobalParametersStateCore> currentGlobalParametersStateCore, const std::shared_ptr<RobotStateCore> currentRobotState, const std::shared_ptr<SensorStateCore> currentSensorState, const std::shared_ptr<MapElementStateCore> currentMapElementStateI, std::shared_ptr<SensorMeasurementCore> matchedMeasurement, std::shared_ptr<CodedVisualMarkerMeasurementCore>& predictedMeasurement)
{
    /// Variables needed
    // State
    // Robot
    Eigen::Vector3d position_robot_wrt_world;
    Eigen::Vector4d attitude_robot_wrt_world;
    // Sensor
    Eigen::Vector3d position_sensor_wrt_robot;
    Eigen::Vector4d attitude_sensor_wrt_robot;
    // Map Element
    Eigen::Vector3d position_map_element_wrt_world;
    Eigen::Vector4d attitude_map_element_wrt_world;


    // Switch depending on robot used
    switch(std::dynamic_pointer_cast<RobotCore>(currentRobotState->getMsfElementCoreSharedPtr())->getRobotCoreType())
    {
        // Free model robot
        case RobotCoreTypes::free_model:
        {
            // Cast
            std::shared_ptr<FreeModelRobotStateCore> currentFreeModelRobotState=std::dynamic_pointer_cast<FreeModelRobotStateCore>(currentRobotState);

            // Set Values of variables
            position_robot_wrt_world=currentFreeModelRobotState->getPosition();
            attitude_robot_wrt_world=currentFreeModelRobotState->getAttitude();

            break;
        }

        // Imu Driven robot
        case RobotCoreTypes::imu_driven:
        {
            // Cast
            std::shared_ptr<ImuDrivenRobotStateCore> currentFreeModelRobotState=std::dynamic_pointer_cast<ImuDrivenRobotStateCore>(currentRobotState);

            // Robot
            position_robot_wrt_world=currentFreeModelRobotState->getPositionRobotWrtWorld();
            attitude_robot_wrt_world=currentFreeModelRobotState->getAttitudeRobotWrtWorld();

            break;
        }

        // Default
        default:
            return -1000;
    }


    // Sensor
    // Cast
    std::shared_ptr<CodedVisualMarkerEyeStateCore> the_sensor_state_core=std::dynamic_pointer_cast<CodedVisualMarkerEyeStateCore>(currentSensorState);
    std::shared_ptr<const CodedVisualMarkerEyeCore> the_sensor_core=std::dynamic_pointer_cast<const CodedVisualMarkerEyeCore>(the_sensor_state_core->getMsfElementCoreSharedPtr());

    position_sensor_wrt_robot=currentSensorState->getPositionSensorWrtRobot();
    attitude_sensor_wrt_robot=currentSensorState->getAttitudeSensorWrtRobot();


    // Map Element
    // Cast
    std::shared_ptr<CodedVisualMarkerLandmarkStateCore> currentMapElementState=std::dynamic_pointer_cast<CodedVisualMarkerLandmarkStateCore>(currentMapElementStateI);
    std::shared_ptr<CodedVisualMarkerLandmarkCore> TheMapElementCore=std::dynamic_pointer_cast<CodedVisualMarkerLandmarkCore>(currentMapElementState->getMsfElementCoreSharedPtr());

    position_map_element_wrt_world=currentMapElementState->getPosition();
    attitude_map_element_wrt_world=currentMapElementState->getAttitude();


    // Matched measurement
    std::shared_ptr<CodedVisualMarkerMeasurementCore> TheMatchedMeasurementCore=std::dynamic_pointer_cast<CodedVisualMarkerMeasurementCore>(matchedMeasurement);



    // dimensions

    // Dimension
    int dimension_global_parameters_error_state=currentGlobalParametersStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorState();
    int dimension_global_parameters_error_parameters=currentGlobalParametersStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorParameters();

    // Dimension robot error state
    int dimension_robot_error_state=currentRobotState->getMsfElementCoreSharedPtr()->getDimensionErrorState();

    // Dimension
    int dimension_sensor_error_state=the_sensor_core->getDimensionErrorState();
    int dimension_sensor_error_parameters=the_sensor_core->getDimensionErrorParameters();

    // Dimension
    int dimension_map_element_error_state=currentMapElementStateI->getMsfElementCoreSharedPtr()->getDimensionErrorState();
    int dimension_map_element_error_parameters=currentMapElementStateI->getMsfElementCoreSharedPtr()->getDimensionErrorParameters();



    /// Core

    // State and Params
    Eigen::Matrix3d jacobian_error_meas_pos_wrt_error_state_robot_pos;
    Eigen::Matrix3d jacobian_error_meas_pos_wrt_error_state_robot_att;
    Eigen::Matrix3d jacobian_error_meas_att_wrt_error_state_robot_att;
    Eigen::Matrix3d jacobian_error_meas_pos_wrt_error_state_sens_pos;
    Eigen::Matrix3d jacobian_error_meas_pos_wrt_error_state_sens_att;
    Eigen::Matrix3d jacobian_error_meas_att_wrt_error_state_sens_att;
    Eigen::Matrix3d jacobian_error_meas_pos_wrt_error_state_map_elem_pos;
    Eigen::Matrix3d jacobian_error_meas_att_wrt_error_state_map_elem_att;
    // Noise
    Eigen::Matrix3d jacobian_error_meas_pos_wrt_error_meas_pos;
    Eigen::Matrix3d jacobian_error_meas_att_wrt_error_meas_att;


    int error_jacobians_error_measurements_core=jacobiansErrorMeasurementsCore(position_robot_wrt_world, attitude_robot_wrt_world, position_sensor_wrt_robot, attitude_sensor_wrt_robot, position_map_element_wrt_world, attitude_map_element_wrt_world, TheMatchedMeasurementCore, predictedMeasurement,
                                           // State and Params
                                           jacobian_error_meas_pos_wrt_error_state_robot_pos, jacobian_error_meas_pos_wrt_error_state_robot_att, jacobian_error_meas_att_wrt_error_state_robot_att,
                                           jacobian_error_meas_pos_wrt_error_state_sens_pos, jacobian_error_meas_pos_wrt_error_state_sens_att, jacobian_error_meas_att_wrt_error_state_sens_att,
                                           jacobian_error_meas_pos_wrt_error_state_map_elem_pos, jacobian_error_meas_att_wrt_error_state_map_elem_att,
                                           // Noise
                                           jacobian_error_meas_pos_wrt_error_meas_pos, jacobian_error_meas_att_wrt_error_meas_att);


    if(error_jacobians_error_measurements_core)
        return error_jacobians_error_measurements_core;


    /// All jacobians


    //// Jacobian Measurement Error - Error State && Jacobian Measurement Error - Error Parameters

    /// Jacobian Measurement Error - Robot Error State


    // Resize and init
    predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementRobotErrorState.resize(dimension_error_measurement_, dimension_robot_error_state);
    predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementRobotErrorState.setZero();

    // Fill
    {
        int dimension_error_measurement_i=0;
        if(this->isMeasurementPositionEnabled())
        {
            // Switch depending on robot used
            switch(std::dynamic_pointer_cast<RobotCore>(currentRobotState->getMsfElementCoreSharedPtr())->getRobotCoreType())
            {
                // Free model robot
                case RobotCoreTypes::free_model:
                {
                    // pos
                    predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementRobotErrorState.block<3,3>(dimension_error_measurement_i,0)=
                            jacobian_error_meas_pos_wrt_error_state_robot_pos;

                    // lin_vel
                    // zeros

                    // lin_acc
                    // zeros

                    // attit
                    predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementRobotErrorState.block<3,3>(dimension_error_measurement_i,9)=
                            jacobian_error_meas_pos_wrt_error_state_robot_att;

                    // ang_vel
                    // zeros

                    // ang_acc
                    // zeros

                    break;
                }

                // Imu Driven robot
                case RobotCoreTypes::imu_driven:
                {
                    // pos
                    predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementRobotErrorState.block<3,3>(dimension_error_measurement_i,0)=
                            jacobian_error_meas_pos_wrt_error_state_robot_pos;

                    // lin_vel
                    // zeros

                    // lin_acc
                    // zeros

                    // attit
                    predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementRobotErrorState.block<3,3>(dimension_error_measurement_i,9)=
                            jacobian_error_meas_pos_wrt_error_state_robot_att;

                    // ang_vel
                    // zeros

                    break;
                }

                // Default
                default:
                    return -1000;

            }

            dimension_error_measurement_i+=3;
        }

        if(this->isMeasurementAttitudeEnabled())
        {
            // Switch depending on robot used
            switch(std::dynamic_pointer_cast<RobotCore>(currentRobotState->getMsfElementCoreSharedPtr())->getRobotCoreType())
            {
                // Free model robot
                case RobotCoreTypes::free_model:
                {
                    // pos
                    // zeros

                    // lin_vel
                    // zeros

                    // lin_acc
                    // zeros

                    // attit
                    predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementRobotErrorState.block<3,3>(dimension_error_measurement_i,9)=
                            jacobian_error_meas_att_wrt_error_state_robot_att;

                    // ang_vel
                    // zeros

                    // ang_acc
                    // zeros

                    break;
                }
                // Imu Driven robot
                case RobotCoreTypes::imu_driven:
                {
                    // pos
                    // zeros

                    // lin_vel
                    // zeros

                    // lin_acc
                    // zeros

                    // attit
                    predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementRobotErrorState.block<3,3>(dimension_error_measurement_i,9)=
                            jacobian_error_meas_att_wrt_error_state_robot_att;

                    // ang_vel
                    // zeros

                    break;
                }
                // Default
                default:
                    return -1000;

            }


            dimension_error_measurement_i+=3;
        }
    }



    /// Jacobian Measurement Error - Global Parameters Error State & Error Parameters


    // Resize and init
    predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementGlobalParametersErrorState.resize(dimension_error_measurement_, dimension_global_parameters_error_state);
    predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementGlobalParametersErrorState.setZero();

    predictedMeasurement->jacobianMeasurementErrorParameters.jacobianMeasurementGlobalParameters.resize(dimension_error_measurement_, dimension_global_parameters_error_parameters);
    predictedMeasurement->jacobianMeasurementErrorParameters.jacobianMeasurementGlobalParameters.setZero();

    // Fill
    // No dependency on global parameters -> Everything is set to zero



    /// Jacobian Measurement Error - Sensor Error State & Error Parameters

    // Resize and init
    predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState.resize(dimension_error_measurement_, dimension_sensor_error_state);
    predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState.setZero();

    predictedMeasurement->jacobianMeasurementErrorParameters.jacobianMeasurementSensorParameters.resize(dimension_error_measurement_, dimension_sensor_error_parameters);
    predictedMeasurement->jacobianMeasurementErrorParameters.jacobianMeasurementSensorParameters.setZero();

    // Fill
    {
        int dimension_error_measurement_i=0;

        // pos
        if(this->isMeasurementPositionEnabled())
        {
            int dimension_sensor_error_parameters_i=0;
            int dimension_sensor_error_state_i=0;

            // pos
            if(the_sensor_core->isEstimationAttitudeSensorWrtRobotEnabled())
            {
                predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState.block<3, 3>(dimension_error_measurement_i, dimension_sensor_error_state_i)=
                        jacobian_error_meas_pos_wrt_error_state_sens_pos;
                dimension_sensor_error_state_i+=3;
            }
            else
            {
                predictedMeasurement->jacobianMeasurementErrorParameters.jacobianMeasurementSensorParameters.block<3,3>(dimension_error_measurement_i, dimension_sensor_error_parameters_i)=
                        jacobian_error_meas_pos_wrt_error_state_sens_pos;
                dimension_sensor_error_parameters_i+=3;
            }

            // att
            if(the_sensor_core->isEstimationAttitudeSensorWrtRobotEnabled())
            {
                predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState.block<3, 3>(dimension_error_measurement_i, dimension_sensor_error_state_i)=
                        jacobian_error_meas_pos_wrt_error_state_sens_att;
                dimension_sensor_error_state_i+=3;
            }
            else
            {
                predictedMeasurement->jacobianMeasurementErrorParameters.jacobianMeasurementSensorParameters.block<3,3>(dimension_error_measurement_i, dimension_sensor_error_parameters_i)=
                        jacobian_error_meas_pos_wrt_error_state_sens_att;
                dimension_sensor_error_parameters_i+=3;
            }

            dimension_error_measurement_i+=3;
        }

        // att
        if(this->isMeasurementAttitudeEnabled())
        {
            int dimension_sensor_error_parameters_i=0;
            int dimension_sensor_error_state_i=0;

            // pos
            if(the_sensor_core->isEstimationAttitudeSensorWrtRobotEnabled())
            {
                // Zeros
                dimension_sensor_error_state_i+=3;
            }
            else
            {
                // Zeros
                dimension_sensor_error_parameters_i+=3;
            }

            // att
            if(the_sensor_core->isEstimationAttitudeSensorWrtRobotEnabled())
            {
                predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState.block<3,3>(dimension_error_measurement_i, dimension_sensor_error_state_i)=
                        jacobian_error_meas_att_wrt_error_state_sens_att;
                dimension_sensor_error_state_i+=3;
            }
            else
            {
                predictedMeasurement->jacobianMeasurementErrorParameters.jacobianMeasurementSensorParameters.block<3,3>(dimension_error_measurement_i, dimension_sensor_error_parameters_i)=
                        jacobian_error_meas_att_wrt_error_state_sens_att;
                dimension_sensor_error_parameters_i+=3;
            }

            dimension_error_measurement_i+=3;
        }
    }



    /// Jacobian Measurement Error - Map Element Error State & Error Parameters

    // Resize and init
    predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementMapElementErrorState.resize(dimension_error_measurement_, dimension_map_element_error_state);
    predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementMapElementErrorState.setZero();

    predictedMeasurement->jacobianMeasurementErrorParameters.jacobianMeasurementMapElementParameters.resize(dimension_error_measurement_, dimension_map_element_error_parameters);
    predictedMeasurement->jacobianMeasurementErrorParameters.jacobianMeasurementMapElementParameters.setZero();

    // Fill
    {
        int dimension_error_measurement_i=0;

        // pos
        if(this->isMeasurementPositionEnabled())
        {
            int dimension_map_element_error_parameters_i=0;
            int dimension_map_element_error_state_i=0;

            // pos
            if(TheMapElementCore->isEstimationPositionVisualMarkerWrtWorldEnabled())
            {
                predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementMapElementErrorState.block<3,3>(dimension_error_measurement_i, dimension_map_element_error_state_i)=
                        jacobian_error_meas_pos_wrt_error_state_map_elem_pos;
                dimension_map_element_error_state_i+=3;
            }
            else
            {
                predictedMeasurement->jacobianMeasurementErrorParameters.jacobianMeasurementMapElementParameters.block<3,3>(dimension_error_measurement_i, dimension_map_element_error_parameters_i)=
                        jacobian_error_meas_pos_wrt_error_state_map_elem_pos;
                dimension_map_element_error_parameters_i+=3;
            }

            // att
            if(TheMapElementCore->isEstimationAttitudeVisualMarkerWrtWorldEnabled())
            {
                // Zeros
                dimension_map_element_error_state_i+=3;
            }
            else
            {
                // Zeros
                dimension_map_element_error_parameters_i+=3;
            }

            dimension_error_measurement_i+=3;
        }

        // att
        if(this->isMeasurementAttitudeEnabled())
        {
            int dimension_map_element_error_parameters_i=0;
            int dimension_map_element_error_state_i=0;

            // pos
            if(TheMapElementCore->isEstimationPositionVisualMarkerWrtWorldEnabled())
            {
                // Zeros
                dimension_map_element_error_state_i+=3;
            }
            else
            {
                // Zeros
                dimension_map_element_error_parameters_i+=3;
            }

            // att
            if(TheMapElementCore->isEstimationAttitudeVisualMarkerWrtWorldEnabled())
            {
                predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementMapElementErrorState.block<3,3>(dimension_error_measurement_i, dimension_map_element_error_state_i)=
                        jacobian_error_meas_att_wrt_error_state_map_elem_att;
                dimension_map_element_error_state_i+=3;
            }
            else
            {
                predictedMeasurement->jacobianMeasurementErrorParameters.jacobianMeasurementMapElementParameters.block<3,3>(dimension_error_measurement_i, dimension_map_element_error_parameters_i)=
                        jacobian_error_meas_att_wrt_error_state_map_elem_att;
                dimension_map_element_error_parameters_i+=3;
            }

            dimension_error_measurement_i+=3;
        }
    }




    /// Jacobians error measurement - sensor noise of the measurement

    // Resize and init Jacobian
    predictedMeasurement->jacobianMeasurementSensorNoise.jacobianMeasurementSensorNoise.resize(dimension_error_measurement_, dimension_error_measurement_);
    predictedMeasurement->jacobianMeasurementSensorNoise.jacobianMeasurementSensorNoise.setZero();

    // Fill
    {
        int dimension_error_measurement_i=0;

        // pos
        if(this->isMeasurementPositionEnabled())
        {
            predictedMeasurement->jacobianMeasurementSensorNoise.jacobianMeasurementSensorNoise.block<3,3>(dimension_error_measurement_i, dimension_error_measurement_i)=
                    jacobian_error_meas_pos_wrt_error_meas_pos;

            dimension_error_measurement_i+=3;
        }


        // att
        if(this->isMeasurementAttitudeEnabled())
        {
            predictedMeasurement->jacobianMeasurementSensorNoise.jacobianMeasurementSensorNoise.block<3,3>(dimension_error_measurement_i, dimension_error_measurement_i)=
                    jacobian_error_meas_att_wrt_error_meas_att;

            dimension_error_measurement_i+=3;
        }

    }


    /// End
    return 0;
}

int CodedVisualMarkerEyeCore::jacobiansErrorMeasurementsCore(Eigen::Vector3d position_robot_wrt_world, Eigen::Vector4d attitude_robot_wrt_world, Eigen::Vector3d position_sensor_wrt_robot, Eigen::Vector4d attitude_sensor_wrt_robot, Eigen::Vector3d position_map_element_wrt_world, Eigen::Vector4d attitude_map_element_wrt_world, std::shared_ptr<CodedVisualMarkerMeasurementCore> matchedMeasurement, std::shared_ptr<CodedVisualMarkerMeasurementCore> predictedMeasurement,
                                                             // State and Params
                                                             Eigen::Matrix3d& jacobian_error_meas_pos_wrt_error_state_robot_pos, Eigen::Matrix3d& jacobian_error_meas_pos_wrt_error_state_robot_att, Eigen::Matrix3d& jacobian_error_meas_att_wrt_error_state_robot_att,
                                                             Eigen::Matrix3d& jacobian_error_meas_pos_wrt_error_state_sens_pos, Eigen::Matrix3d& jacobian_error_meas_pos_wrt_error_state_sens_att, Eigen::Matrix3d& jacobian_error_meas_att_wrt_error_state_sens_att,
                                                             Eigen::Matrix3d& jacobian_error_meas_pos_wrt_error_state_map_elem_pos, Eigen::Matrix3d& jacobian_error_meas_att_wrt_error_state_map_elem_att,
                                                             // Noise
                                                             Eigen::Matrix3d& jacobian_error_meas_pos_wrt_error_meas_pos, Eigen::Matrix3d& jacobian_error_meas_att_wrt_error_meas_att)
{

    // Auxiliar variables
    Eigen::Vector3d tran_inc_wrt_world=position_map_element_wrt_world-position_robot_wrt_world-Quaternion::cross_sandwich(attitude_robot_wrt_world, position_sensor_wrt_robot, Quaternion::inv(attitude_robot_wrt_world));
    Eigen::Vector3d tran_inc2_wrt_world=position_map_element_wrt_world-position_robot_wrt_world;

    //Eigen::Vector4d att_pred_visual_marker_wrt_visual_marker_eye=TheMatchedMeasurementCore->getVisualMarkerAttitude();
    Eigen::Vector4d att_pred_visual_marker_wrt_visual_marker_eye=predictedMeasurement->getVisualMarkerAttitude();

    Eigen::Vector4d att_meas_visual_marker_wrt_visual_marker_eye=matchedMeasurement->getVisualMarkerAttitude();

    Eigen::Vector4d att_visual_marker_eye_wrt_world=Quaternion::cross(attitude_robot_wrt_world, attitude_sensor_wrt_robot);
    Eigen::Vector4d att_world_wrt_visual_marker_eye=Quaternion::inv(att_visual_marker_eye_wrt_world);

    Eigen::Matrix4d mat_q_plus_att_world_wrt_visual_marker_eye=Quaternion::quatMatPlus(att_world_wrt_visual_marker_eye);

    Eigen::Matrix4d mat_q_minus_att_visual_marker_eye_wrt_world=Quaternion::quatMatMinus(att_visual_marker_eye_wrt_world);

    Eigen::Matrix4d mat_q_plus_att_pred_visual_marker_wrt_visual_marker_eye=Quaternion::quatMatPlus(att_pred_visual_marker_wrt_visual_marker_eye);
    Eigen::Matrix4d inv_mat_q_plus_att_pred_visual_marker_wrt_visual_marker_eye=mat_q_plus_att_pred_visual_marker_wrt_visual_marker_eye.inverse();

    Eigen::Matrix4d mat_q_plus_att_meas_visual_marker_wrt_visual_marker_eye=Quaternion::quatMatPlus(att_meas_visual_marker_wrt_visual_marker_eye);
    Eigen::Matrix4d inv_mat_q_plus_att_meas_visual_marker_wrt_visual_marker_eye=mat_q_plus_att_meas_visual_marker_wrt_visual_marker_eye.inverse();

    Eigen::Matrix4d mat_q_minus_att_visual_marker_wrt_world=Quaternion::quatMatMinus(attitude_map_element_wrt_world);
    Eigen::Matrix4d mat_q_plus_att_visual_marker_wrt_world=Quaternion::quatMatPlus(attitude_map_element_wrt_world);

    Eigen::Matrix4d mat_q_plus_att_robot_wrt_world=Quaternion::quatMatPlus(attitude_robot_wrt_world);

    Eigen::Matrix4d mat_q_minus_att_visual_marker_eye_wrt_robot=Quaternion::quatMatMinus(attitude_sensor_wrt_robot);
    Eigen::Matrix4d mat_q_plus_att_visual_marker_eye_wrt_robot=Quaternion::quatMatPlus(attitude_sensor_wrt_robot);

    Eigen::Matrix4d mat_q_plus_tran_inc2_wrt_world=Quaternion::quatMatPlus(tran_inc2_wrt_world);

    Eigen::Matrix4d mat_q_minus_tinc2aux_wrt_world=Quaternion::quatMatMinus(Quaternion::cross_pure_gen(tran_inc2_wrt_world, att_visual_marker_eye_wrt_world));

    Eigen::Matrix4d mat_q_minus_aux1=Quaternion::quatMatMinus(Quaternion::cross_pure_gen(position_sensor_wrt_robot,attitude_sensor_wrt_robot));

    Eigen::Matrix4d mat_q_plus_tra_visual_marker_eye_wrt_robot=Quaternion::quatMatPlus(attitude_sensor_wrt_robot);

    Eigen::Matrix4d mat_q_plus_tran_inc_wrt_world=Quaternion::quatMatPlus(tran_inc_wrt_world);

    Eigen::Matrix4d mat_q_mins_aux2=Quaternion::quatMatMinus(Quaternion::cross_pure_gen(tran_inc_wrt_world, att_visual_marker_eye_wrt_world));

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

    Eigen::MatrixXd mat_diff_vector_wrt_vector_amp(3,4);
    mat_diff_vector_wrt_vector_amp<<0, 1, 0, 0,
                                    0, 0, 1, 0,
                                    0, 0, 0, 1;




    // State and Params
    jacobian_error_meas_pos_wrt_error_state_robot_pos=
            -mat_diff_vector_wrt_vector_amp*mat_q_plus_att_world_wrt_visual_marker_eye*mat_q_minus_att_visual_marker_eye_wrt_world*mat_diff_vector_wrt_vector_amp.transpose();

    jacobian_error_meas_pos_wrt_error_state_robot_att=
            mat_diff_vector_wrt_vector_amp*( mat_q_minus_tinc2aux_wrt_world*mat_diff_quat_inv_wrt_quat + mat_q_plus_att_world_wrt_visual_marker_eye*mat_q_plus_tran_inc2_wrt_world )*mat_q_minus_att_visual_marker_eye_wrt_robot*mat_q_plus_att_robot_wrt_world*0.5*mat_diff_error_quat_wrt_error_theta;

    jacobian_error_meas_att_wrt_error_state_robot_att=
            mat_diff_error_quat_wrt_error_theta.transpose()*inv_mat_q_plus_att_pred_visual_marker_wrt_visual_marker_eye*mat_q_minus_att_visual_marker_wrt_world*mat_diff_quat_inv_wrt_quat*mat_q_minus_att_visual_marker_eye_wrt_robot*mat_q_plus_att_robot_wrt_world*mat_diff_error_quat_wrt_error_theta;

    jacobian_error_meas_pos_wrt_error_state_sens_pos=
            -mat_diff_vector_wrt_vector_amp*(mat_q_minus_aux1*mat_diff_quat_inv_wrt_quat + mat_q_plus_att_visual_marker_eye_wrt_robot*mat_q_plus_tra_visual_marker_eye_wrt_robot )*mat_diff_vector_wrt_vector_amp.transpose();

    jacobian_error_meas_pos_wrt_error_state_sens_att=
            mat_diff_vector_wrt_vector_amp*( mat_q_mins_aux2*mat_diff_quat_inv_wrt_quat + mat_q_plus_att_world_wrt_visual_marker_eye*mat_q_plus_tran_inc_wrt_world )*mat_q_plus_att_robot_wrt_world*mat_q_plus_att_visual_marker_eye_wrt_robot*0.5*mat_diff_error_quat_wrt_error_theta;

    jacobian_error_meas_att_wrt_error_state_sens_att=
            mat_diff_error_quat_wrt_error_theta.transpose()*inv_mat_q_plus_att_pred_visual_marker_wrt_visual_marker_eye*mat_q_minus_att_visual_marker_wrt_world*mat_diff_quat_inv_wrt_quat*mat_q_plus_att_robot_wrt_world*mat_q_plus_att_visual_marker_eye_wrt_robot*mat_diff_error_quat_wrt_error_theta;

    jacobian_error_meas_pos_wrt_error_state_map_elem_pos=
            mat_diff_vector_wrt_vector_amp*mat_q_plus_att_world_wrt_visual_marker_eye*mat_q_minus_att_visual_marker_eye_wrt_world  *mat_diff_vector_wrt_vector_amp.transpose();

    jacobian_error_meas_att_wrt_error_state_map_elem_att=
            mat_diff_error_quat_wrt_error_theta.transpose()*inv_mat_q_plus_att_pred_visual_marker_wrt_visual_marker_eye* mat_q_plus_att_world_wrt_visual_marker_eye *mat_q_plus_att_visual_marker_wrt_world*mat_diff_error_quat_wrt_error_theta;


    // Noise
    jacobian_error_meas_pos_wrt_error_meas_pos=
            Eigen::MatrixXd::Identity(3, 3);

    jacobian_error_meas_att_wrt_error_meas_att=
            mat_diff_error_quat_wrt_error_theta.transpose() *  inv_mat_q_plus_att_meas_visual_marker_wrt_visual_marker_eye*  mat_q_plus_att_pred_visual_marker_wrt_visual_marker_eye  *mat_diff_error_quat_wrt_error_theta;



    return 0;
}


int CodedVisualMarkerEyeCore::mapMeasurement(const TimeStamp theTimeStamp, const std::shared_ptr<GlobalParametersStateCore> TheGlobalParametersStateCore, const std::shared_ptr<RobotStateCore> currentRobotState, const std::shared_ptr<SensorStateCore> currentSensorState, const std::shared_ptr<SensorMeasurementCore> matchedMeasurement, std::shared_ptr<MapElementCore>& newMapElementCore, std::shared_ptr<MapElementStateCore>& newMapElementState)
{
    /// Checks

    // Global Parameters State
    if(!TheGlobalParametersStateCore)
        return -10;
    if(!TheGlobalParametersStateCore->isCorrect())
        return -11;

    // Current Robot State
    if(!currentRobotState)
        return -20;
    if(!currentRobotState->isCorrect())
        return -21;

    // Sensor State
    if(!currentSensorState)
        return 30;
    if(!currentSensorState->isCorrect())
        return 31;

    // Matched Measurement
    if(!matchedMeasurement)
        return -1;
    if(!matchedMeasurement->isCorrect())
        return -1;


    /// Variables needed
    // Robot
    Eigen::Vector3d position_robot_wrt_world;
    Eigen::Vector4d attitude_robot_wrt_world;
    // Sensor
    Eigen::Vector3d position_sensor_wrt_robot;
    Eigen::Vector4d attitude_sensor_wrt_robot;
    // Measurement: Map Element
    Eigen::Vector3d position_map_element_wrt_sensor;
    Eigen::Vector4d attitude_map_element_wrt_sensor;



    // Robot

    // Switch depending on robot used
    switch(std::dynamic_pointer_cast<RobotCore>(currentRobotState->getMsfElementCoreSharedPtr())->getRobotCoreType())
    {
        // Free model robot
        case RobotCoreTypes::free_model:
        {
            // Cast
            std::shared_ptr<FreeModelRobotStateCore> currentFreeModelRobotState=std::dynamic_pointer_cast<FreeModelRobotStateCore>(currentRobotState);

            // Set Values of variables
            position_robot_wrt_world=currentFreeModelRobotState->getPosition();
            attitude_robot_wrt_world=currentFreeModelRobotState->getAttitude();

            break;
        }

        // Imu Driven robot
        case RobotCoreTypes::imu_driven:
        {
            // Cast
            std::shared_ptr<ImuDrivenRobotStateCore> currentFreeModelRobotState=std::dynamic_pointer_cast<ImuDrivenRobotStateCore>(currentRobotState);

            // Robot
            position_robot_wrt_world=currentFreeModelRobotState->getPositionRobotWrtWorld();
            attitude_robot_wrt_world=currentFreeModelRobotState->getAttitudeRobotWrtWorld();

            break;
        }

        // Default
        default:
            return -1000;
    }


    // Sensor
    // Cast
    std::shared_ptr<CodedVisualMarkerEyeCore> TheSensorCore=std::dynamic_pointer_cast<CodedVisualMarkerEyeCore>(currentSensorState->getMsfElementCoreSharedPtr());
    std::shared_ptr<CodedVisualMarkerEyeStateCore> TheSensorState=std::dynamic_pointer_cast<CodedVisualMarkerEyeStateCore>(currentSensorState);

    position_sensor_wrt_robot=TheSensorState->getPositionSensorWrtRobot();
    attitude_sensor_wrt_robot=TheSensorState->getAttitudeSensorWrtRobot();


    // Measurement: Map Element
    // Cast
    std::shared_ptr<CodedVisualMarkerMeasurementCore> TheCodedVisualMarkerMeasurement=std::dynamic_pointer_cast<CodedVisualMarkerMeasurementCore>(matchedMeasurement);

    position_map_element_wrt_sensor=TheCodedVisualMarkerMeasurement->getVisualMarkerPosition();
    attitude_map_element_wrt_sensor=TheCodedVisualMarkerMeasurement->getVisualMarkerAttitude();





    /// Map Element Core

    // Create Map Element Core if it is empty
    std::shared_ptr<CodedVisualMarkerLandmarkCore> TheCodeCodedVisualMarkerLandmarkCore=std::dynamic_pointer_cast<CodedVisualMarkerLandmarkCore>(newMapElementCore);
    if(!TheCodeCodedVisualMarkerLandmarkCore)
    {
        // Map element core
        TheCodeCodedVisualMarkerLandmarkCore=std::make_shared<CodedVisualMarkerLandmarkCore>(this->getMsfStorageCoreWeakPtr());



        // Configurations of the map element core. Only needed if was not previously set. Can be done anycase.

        // Set id in the map element core
        if(TheCodeCodedVisualMarkerLandmarkCore->setId(TheCodedVisualMarkerMeasurement->getVisualMarkerId()))
            return 2;

        // Set name
        if(TheCodeCodedVisualMarkerLandmarkCore->setMapElementName("visual_marker_"+std::to_string(TheCodeCodedVisualMarkerLandmarkCore->getId())))
            return 3;


        // Set Default configurations
        // Enable Estimation Position if measurement pos is enabled
        // TODO Check!
        if(TheSensorCore->isMeasurementPositionEnabled())
            TheCodeCodedVisualMarkerLandmarkCore->enableEstimationPositionVisualMarkerWrtWorld();
        else
            TheCodeCodedVisualMarkerLandmarkCore->enableParameterPositionVisualMarkerWrtWorld();

        // Enable Estimation Attitude if measurement att is enabled
        // TODO Check!
        if(TheSensorCore->isMeasurementAttitudeEnabled())
            TheCodeCodedVisualMarkerLandmarkCore->enableEstimationAttitudeVisualMarkerWrtWorld();
        else
            TheCodeCodedVisualMarkerLandmarkCore->enableParameterAttitudeVisualMarkerWrtWorld();

        // Covariances not needed because are included in P.
        // TODO check!


    }


    // Create Map Element State Core With Map Element Core
    std::shared_ptr<CodedVisualMarkerLandmarkStateCore> TheCodedVisualMarkerLandmarkStateCore=std::dynamic_pointer_cast<CodedVisualMarkerLandmarkStateCore>(newMapElementState);
    if(!TheCodedVisualMarkerLandmarkStateCore)
        TheCodedVisualMarkerLandmarkStateCore=std::make_shared<CodedVisualMarkerLandmarkStateCore>(TheCodeCodedVisualMarkerLandmarkCore);



    //// State Estimation

    /// Core

    int error_map_measurement_core=this->mapMeasurementCore(position_robot_wrt_world, attitude_robot_wrt_world, position_sensor_wrt_robot, attitude_sensor_wrt_robot, position_map_element_wrt_sensor, attitude_map_element_wrt_sensor, TheCodedVisualMarkerLandmarkStateCore);

    if(error_map_measurement_core)
        return error_map_measurement_core;


    /// Polymorph
    newMapElementCore=TheCodeCodedVisualMarkerLandmarkCore;
    newMapElementState=TheCodedVisualMarkerLandmarkStateCore;

    // End
    return 0;
}

int CodedVisualMarkerEyeCore::mapMeasurementCore(Eigen::Vector3d position_robot_wrt_world, Eigen::Vector4d attitude_robot_wrt_world, Eigen::Vector3d position_sensor_wrt_robot, Eigen::Vector4d attitude_sensor_wrt_robot, Eigen::Vector3d position_map_element_wrt_sensor, Eigen::Vector4d attitude_map_element_wrt_sensor, std::shared_ptr<CodedVisualMarkerLandmarkStateCore>& newMapElementState)
{

    // Position
    if(this->isMeasurementPositionEnabled())
    {
        Eigen::Vector3d tran_visual_marker_wrt_robot=Quaternion::cross_sandwich(attitude_sensor_wrt_robot, position_map_element_wrt_sensor, Quaternion::inv(attitude_sensor_wrt_robot)) + position_sensor_wrt_robot;
        newMapElementState->position_=Quaternion::cross_sandwich(attitude_robot_wrt_world, tran_visual_marker_wrt_robot, Quaternion::inv(attitude_robot_wrt_world)) + position_robot_wrt_world;
    }


    // Attitude
    if(this->isMeasurementAttitudeEnabled())
    {
        newMapElementState->attitude_=Quaternion::cross(attitude_robot_wrt_world, attitude_sensor_wrt_robot, attitude_map_element_wrt_sensor);
    }


    return 0;
}

int CodedVisualMarkerEyeCore::jacobiansMapMeasurement(const TimeStamp theTimeStamp, const std::shared_ptr<GlobalParametersStateCore> TheGlobalParametersStateCore, const std::shared_ptr<RobotStateCore> currentRobotState, const std::shared_ptr<SensorStateCore> currentSensorState, const std::shared_ptr<SensorMeasurementCore> matchedMeasurement, std::shared_ptr<MapElementStateCore>& newMapElementState)
{
    /// Checks

    // Global Parameters State
    if(!TheGlobalParametersStateCore)
        return -10;
    if(!TheGlobalParametersStateCore->isCorrect())
        return -11;

    // Current Robot State
    if(!currentRobotState)
        return -20;
    if(!currentRobotState->isCorrect())
        return -21;

    // Sensor State
    if(!currentSensorState)
        return 30;
    if(!currentSensorState->isCorrect())
        return 31;

    // Matched Measurement
    if(!matchedMeasurement)
        return -1;
    if(!matchedMeasurement->isCorrect())
        return -1;

    // Map Element State Core
    if(!newMapElementState)
        return -50;
    if(!newMapElementState->isCorrect())
        return -50;


    /// Variables needed
    // Robot
    Eigen::Vector3d position_robot_wrt_world;
    Eigen::Vector4d attitude_robot_wrt_world;
    // Sensor
    Eigen::Vector3d position_sensor_wrt_robot;
    Eigen::Vector4d attitude_sensor_wrt_robot;
    // Measurement: Map Element
    Eigen::Vector3d position_map_element_wrt_sensor;
    Eigen::Vector4d attitude_map_element_wrt_sensor;



    // Robot

    // Switch depending on robot used
    switch(std::dynamic_pointer_cast<RobotCore>(currentRobotState->getMsfElementCoreSharedPtr())->getRobotCoreType())
    {
        // Free model robot
        case RobotCoreTypes::free_model:
        {
            // Cast
            std::shared_ptr<FreeModelRobotStateCore> currentFreeModelRobotState=std::dynamic_pointer_cast<FreeModelRobotStateCore>(currentRobotState);

            // Set Values of variables
            position_robot_wrt_world=currentFreeModelRobotState->getPosition();
            attitude_robot_wrt_world=currentFreeModelRobotState->getAttitude();

            break;
        }

        // Imu Driven robot
        case RobotCoreTypes::imu_driven:
        {
            // Cast
            std::shared_ptr<ImuDrivenRobotStateCore> currentFreeModelRobotState=std::dynamic_pointer_cast<ImuDrivenRobotStateCore>(currentRobotState);

            // Robot
            position_robot_wrt_world=currentFreeModelRobotState->getPositionRobotWrtWorld();
            attitude_robot_wrt_world=currentFreeModelRobotState->getAttitudeRobotWrtWorld();

            break;
        }

        // Default
        default:
            return -1000;
    }


    // Sensor
    // Cast
    std::shared_ptr<CodedVisualMarkerEyeCore> TheSensorCore=std::dynamic_pointer_cast<CodedVisualMarkerEyeCore>(currentSensorState->getMsfElementCoreSharedPtr());
    std::shared_ptr<CodedVisualMarkerEyeStateCore> TheSensorState=std::dynamic_pointer_cast<CodedVisualMarkerEyeStateCore>(currentSensorState);

    position_sensor_wrt_robot=TheSensorState->getPositionSensorWrtRobot();
    attitude_sensor_wrt_robot=TheSensorState->getAttitudeSensorWrtRobot();


    // Measurement: Map Element
    // Cast
    std::shared_ptr<CodedVisualMarkerMeasurementCore> TheCodedVisualMarkerMeasurement=std::dynamic_pointer_cast<CodedVisualMarkerMeasurementCore>(matchedMeasurement);

    position_map_element_wrt_sensor=TheCodedVisualMarkerMeasurement->getVisualMarkerPosition();
    attitude_map_element_wrt_sensor=TheCodedVisualMarkerMeasurement->getVisualMarkerAttitude();



    // Map Element State Core
    // Cast
    std::shared_ptr<CodedVisualMarkerLandmarkStateCore> TheCodedVisualMarkerLandmarkStateCore=std::dynamic_pointer_cast<CodedVisualMarkerLandmarkStateCore>(newMapElementState);
    std::shared_ptr<CodedVisualMarkerLandmarkCore> TheCodeCodedVisualMarkerLandmarkCore=std::dynamic_pointer_cast<CodedVisualMarkerLandmarkCore>(newMapElementState->getMsfElementCoreSharedPtr());


    //// Core


    // State and Params
    Eigen::Matrix3d jacobian_error_map_pos_wrt_error_state_robot_pos;
    Eigen::Matrix3d jacobian_error_map_pos_wrt_error_state_robot_att;
    Eigen::Matrix3d jacobian_error_map_att_wrt_error_state_robot_att;
    Eigen::Matrix3d jacobian_error_map_pos_wrt_error_state_sens_pos;
    Eigen::Matrix3d jacobian_error_map_pos_wrt_error_state_sens_att;
    Eigen::Matrix3d jacobian_error_map_att_wrt_error_state_sens_att;
    // Measurement
    Eigen::Matrix3d jacobian_error_map_pos_wrt_error_meas_pos;
    Eigen::Matrix3d jacobian_error_map_att_wrt_error_meas_att;


    int error_jacobians_map_measurement_core=this->jacobiansMapMeasurementCore(position_robot_wrt_world, attitude_robot_wrt_world, position_sensor_wrt_robot, attitude_sensor_wrt_robot, position_map_element_wrt_sensor, attitude_map_element_wrt_sensor, TheCodedVisualMarkerLandmarkStateCore,
                                                              // State and Params
                                                              jacobian_error_map_pos_wrt_error_state_robot_pos, jacobian_error_map_pos_wrt_error_state_robot_att, jacobian_error_map_att_wrt_error_state_robot_att,
                                                              jacobian_error_map_pos_wrt_error_state_sens_pos, jacobian_error_map_pos_wrt_error_state_sens_att, jacobian_error_map_att_wrt_error_state_sens_att,
                                                              // Measurement
                                                              jacobian_error_map_pos_wrt_error_meas_pos, jacobian_error_map_att_wrt_error_meas_att);

    if(error_jacobians_map_measurement_core)
        return error_jacobians_map_measurement_core;


    //// Dimensions

    // Global Parameters
    int dimension_global_parameters_error_state_total=TheGlobalParametersStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorState();

    // Robot
    int dimension_robot_error_state_total=currentRobotState->getMsfElementCoreSharedPtr()->getDimensionErrorState();

    // Sensor
    int dimension_sensor_error_state_total=TheSensorState->getMsfElementCoreSharedPtr()->getDimensionErrorState();

    // Measurement
    int dimension_map_new_element_measurement=TheCodedVisualMarkerMeasurement->getSensorCoreSharedPtr()->getDimensionErrorMeasurement();

    // New Map element
    int dimension_map_new_element_error_state_total=TheCodeCodedVisualMarkerLandmarkCore->getDimensionErrorState();




    //// Jacobian



    //// Jacobian Mapping Error-State


    /// Robot

    TheCodedVisualMarkerLandmarkStateCore->jacobian_mapping_error_state_.jacobian_mapping_robot_error_state_.resize(dimension_map_new_element_error_state_total, dimension_robot_error_state_total);
    TheCodedVisualMarkerLandmarkStateCore->jacobian_mapping_error_state_.jacobian_mapping_robot_error_state_.setZero();


    // tran landmark -> Enabled by default -> TODO Check
    {
        // Switch depending on robot used
        switch(std::dynamic_pointer_cast<RobotCore>(currentRobotState->getMsfElementCoreSharedPtr())->getRobotCoreType())
        {
            // Free model robot
            case RobotCoreTypes::free_model:
            {
                // tran robot
                TheCodedVisualMarkerLandmarkStateCore->jacobian_mapping_error_state_.jacobian_mapping_robot_error_state_.block<3,3>(0,0)=
                        jacobian_error_map_pos_wrt_error_state_robot_pos;

                // lin vel robot
                // Zeros

                // lin acc robot
                // Zeros

                // attitude robot
                TheCodedVisualMarkerLandmarkStateCore->jacobian_mapping_error_state_.jacobian_mapping_robot_error_state_.block<3,3>(0,9)=
                        jacobian_error_map_pos_wrt_error_state_robot_att;

                // ang vel robot
                // Zeros

                // ang acc robot
                // Zeros

                break;
            }
            // Imu Driven robot
            case RobotCoreTypes::imu_driven:
            {
                // tran robot
                TheCodedVisualMarkerLandmarkStateCore->jacobian_mapping_error_state_.jacobian_mapping_robot_error_state_.block<3,3>(0,0)=
                        jacobian_error_map_pos_wrt_error_state_robot_pos;

                // lin vel robot
                // Zeros

                // lin acc robot
                // Zeros

                // attitude robot
                TheCodedVisualMarkerLandmarkStateCore->jacobian_mapping_error_state_.jacobian_mapping_robot_error_state_.block<3,3>(0,9)=
                        jacobian_error_map_pos_wrt_error_state_robot_att;

                // ang vel robot
                // Zeros

                break;
            }
            // Default
            default:
                return -1000;
        }
    }

    // atti landmark -> Enabled by default -> TODO Check
    {
        // Switch depending on robot used
        switch(std::dynamic_pointer_cast<RobotCore>(currentRobotState->getMsfElementCoreSharedPtr())->getRobotCoreType())
        {
            // Free model robot
            case RobotCoreTypes::free_model:
            {
                // tran robot
                // Zeros

                // lin vel robot
                // Zeros

                // lin acc robot
                // Zeros

                // attitude robot
                TheCodedVisualMarkerLandmarkStateCore->jacobian_mapping_error_state_.jacobian_mapping_robot_error_state_.block<3,3>(3,9)=
                    jacobian_error_map_att_wrt_error_state_robot_att;

                // ang vel robot
                // Zeros

                // ang acc robot
                // Zeros

                break;
            }
            // Imu Driven robot
            case RobotCoreTypes::imu_driven:
            {
                // tran robot
                // Zeros

                // lin vel robot
                // Zeros

                // lin acc robot
                // Zeros

                // attitude robot
                TheCodedVisualMarkerLandmarkStateCore->jacobian_mapping_error_state_.jacobian_mapping_robot_error_state_.block<3,3>(3,9)=
                    jacobian_error_map_att_wrt_error_state_robot_att;

                // ang vel robot
                // Zeros

                break;
            }
            // Default
            default:
                return -1000;
        }

    }



    /// Global Parameters

    TheCodedVisualMarkerLandmarkStateCore->jacobian_mapping_error_state_.jacobian_mapping_global_parameters_error_state_.resize(dimension_map_new_element_error_state_total, dimension_global_parameters_error_state_total);
    TheCodedVisualMarkerLandmarkStateCore->jacobian_mapping_error_state_.jacobian_mapping_global_parameters_error_state_.setZero();


    // Zeros -> Do nothing



    /// Sensor

    TheCodedVisualMarkerLandmarkStateCore->jacobian_mapping_error_state_.jacobian_mapping_sensor_error_state_.resize(dimension_map_new_element_error_state_total, dimension_sensor_error_state_total);
    TheCodedVisualMarkerLandmarkStateCore->jacobian_mapping_error_state_.jacobian_mapping_sensor_error_state_.setZero();


    {
        int dimension_map_new_element_error_state_i=0;

        // tran landmark -> Enabled by default -> TODO Check
        {
            int dimension_error_state_i=0;

            // Posi sensor wrt robot
            if(TheSensorCore->isEstimationPositionSensorWrtRobotEnabled())
            {
                TheCodedVisualMarkerLandmarkStateCore->jacobian_mapping_error_state_.jacobian_mapping_sensor_error_state_.block<3,3>(dimension_map_new_element_error_state_i, dimension_error_state_i)=
                    jacobian_error_map_pos_wrt_error_state_sens_pos;
                dimension_error_state_i+=3;
            }

            // Atti sensor wrt robot
            if(TheSensorCore->isEstimationAttitudeSensorWrtRobotEnabled())
            {
                TheCodedVisualMarkerLandmarkStateCore->jacobian_mapping_error_state_.jacobian_mapping_sensor_error_state_.block<3,3>(dimension_map_new_element_error_state_i, dimension_error_state_i)=
                    jacobian_error_map_pos_wrt_error_state_sens_att;
                dimension_error_state_i+=3;
            }
            dimension_map_new_element_error_state_i+=3;
        }


        // atti landmark -> Enabled by default -> TODO Check
        {
            int dimension_error_state_i=0;

            // Posi sensor wrt robot
            if(TheSensorCore->isEstimationPositionSensorWrtRobotEnabled())
            {
                // Zeros
                dimension_error_state_i+=3;
            }

            // Atti sensor wrt robot
            if(TheSensorCore->isEstimationAttitudeSensorWrtRobotEnabled())
            {
                TheCodedVisualMarkerLandmarkStateCore->jacobian_mapping_error_state_.jacobian_mapping_sensor_error_state_.block<3,3>(dimension_map_new_element_error_state_i, dimension_error_state_i)=
                    jacobian_error_map_att_wrt_error_state_sens_att;
                dimension_error_state_i+=3;
            }
            dimension_map_new_element_error_state_i+=3;
        }
    }




    //// Jacobian Mapping Error-State Noise

    TheCodedVisualMarkerLandmarkStateCore->jacobian_mapping_error_state_noise_.resize(dimension_map_new_element_error_state_total, dimension_map_new_element_measurement);
    TheCodedVisualMarkerLandmarkStateCore->jacobian_mapping_error_state_noise_.setZero();


    {
        int dimension_map_new_element_error_state_i=0;

        // tran landmark -> Enabled by default -> TODO Check
        {
            int dimension_measurement_i=0;
            // tran meas
            if(TheSensorCore->isMeasurementPositionEnabled())
            {
                TheCodedVisualMarkerLandmarkStateCore->jacobian_mapping_error_state_noise_.block<3,3>(dimension_map_new_element_error_state_i, dimension_measurement_i)=
                     jacobian_error_map_pos_wrt_error_meas_pos;
                dimension_measurement_i+=3;
            }

            // Atti meas
            if(TheSensorCore->isMeasurementAttitudeEnabled())
            {
                // Zeros
                dimension_measurement_i+=3;
            }

            dimension_map_new_element_error_state_i+=3;
        }


        // atti landmark -> Enabled by default -> TODO Check
        {
            int dimension_measurement_i=0;

            // tran meas
            if(TheSensorCore->isMeasurementPositionEnabled())
            {
                // Zeros
                dimension_measurement_i+=3;
            }

            // Atti meas
            if(TheSensorCore->isMeasurementAttitudeEnabled())
            {
                TheCodedVisualMarkerLandmarkStateCore->jacobian_mapping_error_state_noise_.block<3,3>(dimension_map_new_element_error_state_i, dimension_measurement_i)=
                    jacobian_error_map_att_wrt_error_meas_att;
                dimension_measurement_i+=3;
            }

            dimension_map_new_element_error_state_i+=3;
        }
    }




    /// End
    newMapElementState=TheCodedVisualMarkerLandmarkStateCore;

    return 0;
}

int CodedVisualMarkerEyeCore::jacobiansMapMeasurementCore(Eigen::Vector3d position_robot_wrt_world, Eigen::Vector4d attitude_robot_wrt_world, Eigen::Vector3d position_sensor_wrt_robot, Eigen::Vector4d attitude_sensor_wrt_robot, Eigen::Vector3d position_map_element_wrt_sensor, Eigen::Vector4d attitude_map_element_wrt_sensor, std::shared_ptr<CodedVisualMarkerLandmarkStateCore> newMapElementState,
                                                          // State and Params
                                                          Eigen::Matrix3d& jacobian_error_map_pos_wrt_error_state_robot_pos, Eigen::Matrix3d& jacobian_error_map_pos_wrt_error_state_robot_att, Eigen::Matrix3d& jacobian_error_map_att_wrt_error_state_robot_att,
                                                          Eigen::Matrix3d& jacobian_error_map_pos_wrt_error_state_sens_pos, Eigen::Matrix3d& jacobian_error_map_pos_wrt_error_state_sens_att, Eigen::Matrix3d& jacobian_error_map_att_wrt_error_state_sens_att,
                                                          // Measurement
                                                          Eigen::Matrix3d& jacobian_error_map_pos_wrt_error_meas_pos, Eigen::Matrix3d& jacobian_error_map_att_wrt_error_meas_att)
{

    // Aux vars
    Eigen::Vector3d tran_visual_marker_wrt_robot=Quaternion::cross_sandwich(attitude_sensor_wrt_robot, position_map_element_wrt_sensor, Quaternion::inv(attitude_sensor_wrt_robot))+position_sensor_wrt_robot;

    Eigen::Vector4d att_visual_marker_eye_wrt_world=Quaternion::cross(attitude_robot_wrt_world, attitude_sensor_wrt_robot);


    Eigen::Matrix4d mat_quat_plus_att_visual_marker_wrt_world=Quaternion::quatMatPlus(newMapElementState->getAttitude());
    Eigen::Matrix4d mat_inv_quat_plus_att_visual_marker_wrt_world=mat_quat_plus_att_visual_marker_wrt_world.inverse();

    Eigen::Matrix4d mat_quat_minus_att_visual_marker_wrt_visual_marker_eye=Quaternion::quatMatMinus(attitude_map_element_wrt_sensor);
    Eigen::Matrix4d mat_quat_plus_att_visual_marker_wrt_visual_marker_eye=Quaternion::quatMatPlus(attitude_map_element_wrt_sensor);

    Eigen::Matrix4d mat_quat_minus_att_visual_marker_eye_wrt_robot=Quaternion::quatMatMinus(attitude_sensor_wrt_robot);
    Eigen::Matrix4d mat_quat_plus_att_visual_marker_eye_wrt_robot=Quaternion::quatMatPlus(attitude_sensor_wrt_robot);

    Eigen::Matrix4d mat_quat_minus_att_world_wrt_visual_marker_eye=Quaternion::quatMatMinus(Quaternion::inv(att_visual_marker_eye_wrt_world));

    Eigen::Matrix4d mat_quat_plus_att_robot_wrt_world=Quaternion::quatMatPlus(attitude_robot_wrt_world);

    Eigen::Matrix4d mat_quat_minus_att_world_wrt_robot=Quaternion::quatMatMinus(Quaternion::inv(attitude_robot_wrt_world));

    Eigen::Matrix4d mat_quat_plus_att_visual_marker_eye_wrt_world=Quaternion::quatMatPlus(att_visual_marker_eye_wrt_world);

    Eigen::Matrix4d mat_quat_plus_tran_visual_marker_wrt_robot=Quaternion::quatMatPlus(tran_visual_marker_wrt_robot);

    Eigen::Matrix4d mat_quat_plus_tran_visual_marker_wrt_visual_marker_eye=Quaternion::quatMatPlus(position_map_element_wrt_sensor);

    Eigen::Matrix4d mat_quat_minus_aux1=Quaternion::quatMatMinus(Quaternion::cross_pure_gen(tran_visual_marker_wrt_robot, attitude_robot_wrt_world));
    Eigen::Matrix4d mat_quat_minus_aux2=Quaternion::quatMatMinus(Quaternion::cross_pure_gen(position_map_element_wrt_sensor, att_visual_marker_eye_wrt_world));


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

    Eigen::MatrixXd mat_diff_vector_wrt_vector_amp(3,4);
    mat_diff_vector_wrt_vector_amp<<0, 1, 0, 0,
                                    0, 0, 1, 0,
                                    0, 0, 0, 1;



    // State and Params
    jacobian_error_map_pos_wrt_error_state_robot_pos=
            Eigen::Matrix3d::Identity(3,3);
    jacobian_error_map_pos_wrt_error_state_robot_att=
            mat_diff_vector_wrt_vector_amp*(mat_quat_minus_aux1*mat_diff_quat_inv_wrt_quat + mat_quat_plus_att_robot_wrt_world*mat_quat_plus_tran_visual_marker_wrt_robot  )*mat_quat_plus_att_robot_wrt_world*0.5*mat_diff_error_quat_wrt_error_theta;
    jacobian_error_map_att_wrt_error_state_robot_att=
            mat_diff_error_quat_wrt_error_theta.transpose()*mat_inv_quat_plus_att_visual_marker_wrt_world*mat_quat_minus_att_visual_marker_wrt_visual_marker_eye*mat_quat_minus_att_visual_marker_eye_wrt_robot* mat_quat_plus_att_robot_wrt_world *mat_diff_error_quat_wrt_error_theta;
    jacobian_error_map_pos_wrt_error_state_sens_pos=
            mat_diff_vector_wrt_vector_amp*mat_quat_plus_att_robot_wrt_world* mat_quat_minus_att_world_wrt_robot *mat_diff_vector_wrt_vector_amp.transpose();
    jacobian_error_map_pos_wrt_error_state_sens_att=
            mat_diff_vector_wrt_vector_amp*( mat_quat_minus_aux2 *mat_diff_quat_inv_wrt_quat + mat_quat_plus_att_visual_marker_eye_wrt_world* mat_quat_plus_tran_visual_marker_wrt_visual_marker_eye )*mat_quat_plus_att_robot_wrt_world*mat_quat_plus_att_visual_marker_eye_wrt_robot*0.5*mat_diff_error_quat_wrt_error_theta;
    jacobian_error_map_att_wrt_error_state_sens_att=
            mat_diff_error_quat_wrt_error_theta.transpose()*mat_inv_quat_plus_att_visual_marker_wrt_world*mat_quat_plus_att_robot_wrt_world*mat_quat_minus_att_visual_marker_wrt_visual_marker_eye* mat_quat_plus_att_visual_marker_eye_wrt_robot *mat_diff_error_quat_wrt_error_theta;

    // Measurement
    jacobian_error_map_pos_wrt_error_meas_pos=
            mat_diff_vector_wrt_vector_amp* mat_quat_plus_att_visual_marker_eye_wrt_world* mat_quat_minus_att_world_wrt_visual_marker_eye *mat_diff_vector_wrt_vector_amp.transpose();
    jacobian_error_map_att_wrt_error_meas_att=
            mat_diff_error_quat_wrt_error_theta.transpose()*mat_inv_quat_plus_att_visual_marker_wrt_world*mat_quat_plus_att_visual_marker_eye_wrt_world*mat_quat_plus_att_visual_marker_wrt_visual_marker_eye  *mat_diff_error_quat_wrt_error_theta;


    // end
    return 0;
}

