
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

CodedVisualMarkerEyeCore::CodedVisualMarkerEyeCore(const std::weak_ptr<MsfStorageCore> the_msf_storage_core) :
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

int CodedVisualMarkerEyeCore::readConfig(const pugi::xml_node& sensor, unsigned int sensorId, std::shared_ptr<CodedVisualMarkerEyeStateCore>& SensorInitStateCore)
{
    // Create a class for the SensorStateCore
    if(!SensorInitStateCore)
        SensorInitStateCore=std::make_shared<CodedVisualMarkerEyeStateCore>(this->getMsfElementCoreWeakPtr());

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

int CodedVisualMarkerEyeCore::setNoiseMeasurementPosition(const Eigen::Matrix3d &noise_measurement_position)
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

int CodedVisualMarkerEyeCore::setNoiseMeasurementAttitude(const Eigen::Matrix3d& noise_measurement_attitude)
{
    this->noise_measurement_attitude_=noise_measurement_attitude;
    return 0;
}

/*
int CodedVisualMarkerEyeCore::setMeasurement(const TimeStamp the_time_stamp, std::shared_ptr<CodedVisualMarkerMeasurementCore> the_visual_marker_measurement)
{
    if(!isSensorEnabled())
        return 0;

    if(this->getMsfStorageCoreSharedPtr()->setMeasurement(the_time_stamp, the_visual_marker_measurement))
    {
        std::cout<<"CodedVisualMarkerEyeCore::setMeasurement() error"<<std::endl;
        return 1;
    }

    return 0;
}
*/

int CodedVisualMarkerEyeCore::setMeasurementList(const TimeStamp& the_time_stamp, const std::list< std::shared_ptr<SensorMeasurementCore> >& the_visual_marker_measurement_list)
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

    std::vector<Eigen::Triplet<double> > tripletCovarianceParameters;

    unsigned int dimension=0;
    if(!this->isEstimationPositionSensorWrtRobotEnabled())
    {
        for(int i=0; i<3; i++)
            tripletCovarianceParameters.push_back(Eigen::Triplet<double>(dimension+i,dimension+i,noisePositionSensorWrtRobot(i,i)));

        dimension+=3;
    }
    if(!this->isEstimationAttitudeSensorWrtRobotEnabled())
    {
        for(int i=0; i<3; i++)
            tripletCovarianceParameters.push_back(Eigen::Triplet<double>(dimension+i,dimension+i,noiseAttitudeSensorWrtRobot(i,i)));

        dimension+=3;
    }

    covariances_matrix.setFromTriplets(tripletCovarianceParameters.begin(), tripletCovarianceParameters.end());


    return covariances_matrix;
}

int CodedVisualMarkerEyeCore::prepareCovarianceInitErrorStateSpecific()
{
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

    // Fill
    {
        int dimension_noise_i=0;

        // Nothing
    }


    // End
    return covariance_noise;
}

int CodedVisualMarkerEyeCore::predictState(//Time
                                         const TimeStamp& previousTimeStamp, const TimeStamp& currentTimeStamp,
                                         // Previous State
                                         const std::shared_ptr<StateEstimationCore>& pastState,
                                         // Inputs
                                         const std::shared_ptr<InputCommandComponent>& inputCommand,
                                         // Predicted State
                                         std::shared_ptr<StateCore>& predictedState)
{
    // Checks

    // Past State
    if(!pastState)
        return -1;

    // TODO

    // Search for the past sensor State Core
    CodedVisualMarkerEyeStateCore* past_sensor_state(nullptr);
    if(findSensorState(pastState->TheListSensorStateCore, past_sensor_state))
        return -2;
    if(!past_sensor_state)
        return -10;


    // Predicted State
    CodedVisualMarkerEyeStateCore* predicted_sensor_state(nullptr);
    if(!predictedState)
    {
        predicted_sensor_state=new CodedVisualMarkerEyeStateCore;
        predicted_sensor_state->setMsfElementCorePtr(past_sensor_state->getMsfElementCoreWeakPtr());
        predictedState=std::shared_ptr<CodedVisualMarkerEyeStateCore>(predicted_sensor_state);
    }
    else
        predicted_sensor_state=dynamic_cast<CodedVisualMarkerEyeStateCore*>(predictedState.get());




    // Predict State
    int error_predict_state=predictStateSpecific(previousTimeStamp, currentTimeStamp,
                                         past_sensor_state,
                                         predicted_sensor_state);

    // Check error
    if(error_predict_state)
        return error_predict_state;



    // End
    return 0;
}

int CodedVisualMarkerEyeCore::predictStateSpecific(const TimeStamp &previousTimeStamp, const TimeStamp &currentTimeStamp,
                                                   const CodedVisualMarkerEyeStateCore *pastState,
                                                   CodedVisualMarkerEyeStateCore *&predictedState)
{
    // Checks in the past state
    if(!pastState->isCorrect())
    {
        std::cout<<"CodedVisualMarkerEyeCore::predictStateSpecific() error"<<std::endl;
        return -5;
    }


    // Create the predicted state if it doesn't exist
    if(!predictedState)
    {
        predictedState=new CodedVisualMarkerEyeStateCore;
        predictedState->setMsfElementCorePtr(pastState->getMsfElementCoreSharedPtr());
    }


    /// Aux values
    Eigen::Vector3d position_sensor_wrt_robot;
    Eigen::Vector4d attitude_sensor_wrt_robot;
    // State k+1: Sensor
    Eigen::Vector3d pred_position_sensor_wrt_robot;
    Eigen::Vector4d pred_attitude_sensor_wrt_robot;


    /// Fill
    position_sensor_wrt_robot=pastState->positionSensorWrtRobot;
    attitude_sensor_wrt_robot=pastState->attitudeSensorWrtRobot;;


    //Delta Time
    TimeStamp DeltaTime=currentTimeStamp-previousTimeStamp;
    double dt=DeltaTime.get_double();


    //// Core

    int error_predict_state_core=this->predictStateCore(position_sensor_wrt_robot, attitude_sensor_wrt_robot,
                                                        pred_position_sensor_wrt_robot, pred_attitude_sensor_wrt_robot);

    if(error_predict_state_core)
        return error_predict_state_core;



    /// Fill


    /// Position
    if(this->isEstimationPositionSensorWrtRobotEnabled())
    {
        // Estimation
        predictedState->positionSensorWrtRobot=pred_position_sensor_wrt_robot;
    }
    else
    {
        // Parameter
        predictedState->positionSensorWrtRobot=pred_position_sensor_wrt_robot;
    }


    /// Attitude
    if(this->isEstimationAttitudeSensorWrtRobotEnabled())
    {
        predictedState->attitudeSensorWrtRobot=pred_attitude_sensor_wrt_robot;
    }
    else
    {
        predictedState->attitudeSensorWrtRobot=pred_attitude_sensor_wrt_robot;
    }



    return 0;
}

int CodedVisualMarkerEyeCore::predictStateCore(// State k: Sensor
                                                 const Eigen::Vector3d& position_sensor_wrt_robot, const Eigen::Vector4d& attitude_sensor_wrt_robot,
                                                 // State k+1: Sensor
                                                 Eigen::Vector3d& pred_position_sensor_wrt_robot, Eigen::Vector4d& pred_attitude_sensor_wrt_robot)
{
    // Position
    pred_position_sensor_wrt_robot=position_sensor_wrt_robot;

    // Attitude
    pred_attitude_sensor_wrt_robot=attitude_sensor_wrt_robot;

    return 0;
}


// Jacobian

int CodedVisualMarkerEyeCore::predictErrorStateJacobian(//Time
                             const TimeStamp& previousTimeStamp, const TimeStamp& currentTimeStamp,
                             // Previous State
                             const std::shared_ptr<StateEstimationCore>& past_state,
                            // Inputs
                            const std::shared_ptr<InputCommandComponent>& input_command,
                             // Predicted State
                             std::shared_ptr<StateCore> &predicted_state)
{
    // Checks

    // Past State
    if(!past_state)
        return -1;

    // Predicted State
    if(!predicted_state)
        return -1;

    // TODO


    // Search for the past sensor State Core
    CodedVisualMarkerEyeStateCore* past_sensor_state(nullptr);
    if(findSensorState(past_state->TheListSensorStateCore, past_sensor_state))
        return -2;
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
    CodedVisualMarkerEyeStateCore* predicted_sensor_state=dynamic_cast<CodedVisualMarkerEyeStateCore*>(predicted_state.get());




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
        if( dynamic_cast<CodedVisualMarkerEyeStateCore*>((*itSensorStateCore).get()) == past_sensor_state )
            break;
    }


    // Fu
    // Nothing


    // Fn
    // Nothing



    /// Predict State Jacobians
    int error_predict_state_jacobians=predictErrorStateJacobiansSpecific(previousTimeStamp, currentTimeStamp,
                                                                         past_sensor_state,
                                                                         predicted_sensor_state,
                                                                         // Jacobians Error State: Fx, Fp
                                                                         // Sensor
                                                                         (*it_jacobian_error_state_wrt_sensor_error_state),
                                                                         (*it_jacobian_error_state_wrt_sensor_error_parameters)
                                                                         );

    // Check error
    if(error_predict_state_jacobians)
        return error_predict_state_jacobians;



    // End
    return 0;
}

int CodedVisualMarkerEyeCore::predictErrorStateJacobiansSpecific(const TimeStamp &previousTimeStamp, const TimeStamp &currentTimeStamp,
                                                                 const CodedVisualMarkerEyeStateCore *pastState,
                                                                 const CodedVisualMarkerEyeStateCore *predictedState,
                                                                 // Jacobians Error State: Fx, Fp
                                                                 // Sensor
                                                                 Eigen::SparseMatrix<double>& jacobian_error_state_wrt_sensor_error_state,
                                                                 Eigen::SparseMatrix<double>& jacobian_error_state_wrt_sensor_error_parameters
                                                                 )
{

    // Checks
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
                                                                                jacobian_error_sens_att_wrt_error_state_sens_att);

    if(error_predict_error_state_jacobians_core)
        return error_predict_error_state_jacobians_core;



    ///// Jacobian Error State - Error State: Fx & Jacobian Error State - Error Parameters: Fp

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
            // Add to triplet list
            BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_state_wrt_error_state, jacobian_error_sens_att_wrt_error_state_sens_att, dimension_error_state_i, dimension_error_state_i);

            // Update dimension for next
            dimension_error_state_i+=3;
        }


        // Set From Triplets
        jacobian_error_state_wrt_sensor_error_state.setFromTriplets(triplet_list_jacobian_error_state_wrt_error_state.begin(), triplet_list_jacobian_error_state_wrt_error_state.end());
        jacobian_error_state_wrt_sensor_error_parameters.setFromTriplets(triplet_list_jacobian_error_state_wrt_error_parameters.begin(), triplet_list_jacobian_error_state_wrt_error_parameters.end());

    }

    /// Map elements
    {
        // Nothing to do
    }



    //// Jacobian Error State - Error Input

    {
        // Nothing to do
    }


    //// Jacobian Error State - Noise Estimation: Fn

    {
        // Fill
        // Nothing to do
    }



    // End
    return 0;
}

int CodedVisualMarkerEyeCore::predictErrorStateJacobiansCore(// State k: Sensor
                                                           const Eigen::Vector3d& position_sensor_wrt_robot, const Eigen::Vector4d& attitude_sensor_wrt_robot,
                                                           // State k+1: Sensor
                                                           const Eigen::Vector3d& pred_position_sensor_wrt_robot, const Eigen::Vector4d& pred_attitude_sensor_wrt_robot,
                                                           // Jacobian: State
                                                           Eigen::Matrix3d& jacobian_error_sens_pos_wrt_error_state_sens_pos,  Eigen::Matrix3d& jacobian_error_sens_att_wrt_error_state_sens_att)
{

    /// Position
    jacobian_error_sens_pos_wrt_error_state_sens_pos=Eigen::MatrixXd::Identity(3,3);


    /// Attitude
    // TODO FIX!
    jacobian_error_sens_att_wrt_error_state_sens_att=Eigen::MatrixXd::Identity(3,3);


    // End
    return 0;
}

int CodedVisualMarkerEyeCore::predictMeasurement(// Time
                                               const TimeStamp& current_time_stamp,
                                               // Current State
                                               const std::shared_ptr<StateEstimationCore>& current_state,
                                                 // Measurements
                                                 const std::shared_ptr<SensorMeasurementCore>& measurement,
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

    CodedVisualMarkerMeasurementCore* sensor_measurement=dynamic_cast<CodedVisualMarkerMeasurementCore*>(measurement.get());

    // Nothing else needed


    // Predicted Measurement
    CodedVisualMarkerMeasurementCore* predicted_sensor_measurement(nullptr);
    if(!predicted_measurement)
    {
        predicted_sensor_measurement=new CodedVisualMarkerMeasurementCore;
        predicted_sensor_measurement->setSensorCorePtr(std::dynamic_pointer_cast<SensorCore>(this->getMsfElementCoreSharedPtr()));
        predicted_measurement=std::shared_ptr<CodedVisualMarkerMeasurementCore>(predicted_sensor_measurement);
    }
    else
    {
        if(measurement->getSensorCoreSharedPtr() != predicted_measurement->getSensorCoreSharedPtr())
            return -3;
        predicted_sensor_measurement=dynamic_cast<CodedVisualMarkerMeasurementCore*>(predicted_measurement.get());
    }


    // Search for the past sensor State Core
    CodedVisualMarkerEyeStateCore* current_sensor_state(nullptr);
    if(findSensorState(current_state->TheListSensorStateCore, current_sensor_state))
        return -2;
    if(!current_sensor_state)
        return -10;


    // Search for the associated map element -> If does not exist, need to be mapped!
    CodedVisualMarkerLandmarkStateCore* current_map_element_state(nullptr);
    if(findMapElementState(current_state->TheListMapElementStateCore, sensor_measurement, current_map_element_state))
        return 1;
    if(!current_map_element_state)
        return 1;


    // Predict State
    int error_predict_measurement=predictMeasurementSpecific(current_time_stamp,
                                                             dynamic_cast<RobotStateCore*>(current_state->TheRobotStateCore.get()),
                                                             current_sensor_state,
                                                             current_map_element_state,
                                                             predicted_sensor_measurement);

    // Check error
    if(error_predict_measurement)
        return error_predict_measurement;


    // End
    return 0;
}

int CodedVisualMarkerEyeCore::predictMeasurementSpecific(const TimeStamp &theTimeStamp,
                                                 const RobotStateCore *currentRobotState,
                                                 const CodedVisualMarkerEyeStateCore *currentSensorState,
                                                 const CodedVisualMarkerLandmarkStateCore *currentMapElementState,
                                                 CodedVisualMarkerMeasurementCore *&predictedMeasurement)
{
#if _DEBUG_SENSOR_CORE
    logFile<<"CodedVisualMarkerEyeCore::predictMeasurement() TS: sec="<<theTimeStamp.sec<<" s; nsec="<<theTimeStamp.nsec<<" ns"<<std::endl;
#endif

    /// Check
    if(!this->isCorrect())
    {
        std::cout<<"CodedVisualMarkerEyeCore::predictMeasurement() error 50"<<std::endl;
        return -50;
    }

    /// check Robot
    if(!currentRobotState)
    {
        std::cout<<"CodedVisualMarkerEyeCore::predictMeasurement() error 2"<<std::endl;
        return -2;
    }

    if(!currentRobotState->isCorrect())
    {
        std::cout<<"CodedVisualMarkerEyeCore::predictMeasurement() error 3"<<std::endl;
        return -3;
    }

    /// Check sensor
    if(!currentSensorState)
    {
        std::cout<<"CodedVisualMarkerEyeCore::predictMeasurement() error 1"<<std::endl;
        return -1;
    }

    if(!currentSensorState->isCorrect())
    {
        std::cout<<"CodedVisualMarkerEyeCore::predictMeasurement() error 1"<<std::endl;
        return -1;
    }

    // Cast
    //std::shared_ptr<CodedVisualMarkerEyeStateCore> currentSensorState=std::dynamic_pointer_cast<CodedVisualMarkerEyeStateCore>(currentSensorStateI);


    /// Check Map element
    if(!currentMapElementState)
    {
        std::cout<<"CodedVisualMarkerEyeCore::predictMeasurement() error 2"<<std::endl;
        return -4;
    }

    if(!currentMapElementState->isCorrect())
    {
        std::cout<<"CodedVisualMarkerEyeCore::predictMeasurement() error 2"<<std::endl;
        return -5;
    }

    // Cast
    //std::shared_ptr<CodedVisualMarkerLandmarkStateCore> currentMapElementState=std::dynamic_pointer_cast<CodedVisualMarkerLandmarkStateCore>(currentMapElementStateI);

    // Checks
    // TODO


    /// Create pointer
    if(!predictedMeasurement)
    {
        predictedMeasurement=new CodedVisualMarkerMeasurementCore;
        predictedMeasurement->setSensorCorePtr(std::dynamic_pointer_cast<SensorCore>(this->getMsfElementCoreSharedPtr()));

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

    // Predicted Measurement
    Eigen::Vector3d position_map_element_wrt_sensor;
    Eigen::Vector4d attitude_map_element_wrt_sensor;


    // Robot
    position_robot_wrt_world=currentRobotState->getPositionRobotWrtWorld();
    attitude_robot_wrt_world=currentRobotState->getAttitudeRobotWrtWorld();


    // Sensor
    position_sensor_wrt_robot=currentSensorState->getPositionSensorWrtRobot();
    attitude_sensor_wrt_robot=currentSensorState->getAttitudeSensorWrtRobot();


    // Map Element
    position_map_element_wrt_world=currentMapElementState->getPosition();
    attitude_map_element_wrt_world=currentMapElementState->getAttitude();



    /// Prediction Core
    int error_predict_measurement_core=this->predictMeasurementCore(position_robot_wrt_world, attitude_robot_wrt_world,
                                                                    position_sensor_wrt_robot, attitude_sensor_wrt_robot,
                                                                    position_map_element_wrt_world, attitude_map_element_wrt_world,
                                                                    position_map_element_wrt_sensor, attitude_map_element_wrt_sensor);

    if(error_predict_measurement_core)
        return error_predict_measurement_core;


    /// Set
    // Position
    if(this->isMeasurementPositionEnabled())
    {
        predictedMeasurement->setVisualMarkerPosition(position_map_element_wrt_sensor);
    }
    // Attitude
    if(this->isMeasurementAttitudeEnabled())
    {
        predictedMeasurement->setVisualMarkerAttitude(attitude_map_element_wrt_sensor);
    }



#if _DEBUG_SENSOR_CORE
    logFile<<"CodedVisualMarkerEyeCore::predictMeasurement() ended TS: sec="<<theTimeStamp.sec<<" s; nsec="<<theTimeStamp.nsec<<" ns"<<std::endl;
#endif


    // End
    return 0;
}

int CodedVisualMarkerEyeCore::predictMeasurementCore(// State: Robot
                                                     const Eigen::Vector3d& position_robot_wrt_world, const Eigen::Vector4d& attitude_robot_wrt_world,
                                                     // State: Sensor
                                                     const Eigen::Vector3d& position_sensor_wrt_robot, const Eigen::Vector4d& attitude_sensor_wrt_robot,
                                                     // State: Map
                                                     const Eigen::Vector3d& position_map_element_wrt_world, const Eigen::Vector4d& attitude_map_element_wrt_world,
                                                     // Predicted Measurement
                                                     Eigen::Vector3d& position_map_element_wrt_sensor, Eigen::Vector4d& attitude_map_element_wrt_sensor)
{
    // Aux vars
    Eigen::Vector4d attitude_visual_marker_eye_wrt_world=
            Quaternion::cross(attitude_robot_wrt_world, attitude_sensor_wrt_robot);


    // Position
    if(this->isMeasurementPositionEnabled())
    {
        // Aux Variable
        //Eigen::Vector3d position_visual_marker_wrt_visual_marker_eye;
        //position_visual_marker_wrt_visual_marker_eye.setZero();


        Eigen::Vector3d position_visual_marker_eye_wrt_world=
                Quaternion::cross_sandwich(attitude_robot_wrt_world, position_sensor_wrt_robot, Quaternion::inv(attitude_robot_wrt_world));


        // Equation
        //position_visual_marker_wrt_visual_marker_eye=
        position_map_element_wrt_sensor=
                Quaternion::cross_sandwich(Quaternion::inv(attitude_visual_marker_eye_wrt_world), position_map_element_wrt_world-position_visual_marker_eye_wrt_world-position_robot_wrt_world, attitude_visual_marker_eye_wrt_world);

        // Set
        //predictedMeasurement->setVisualMarkerPosition(position_visual_marker_wrt_visual_marker_eye);
    }



    // Attitude
    if(this->isMeasurementAttitudeEnabled())
    {
        // Aux Variable
        //Eigen::Vector4d attitude_visual_marker_wrt_visual_marker_eye;
        //attitude_visual_marker_wrt_visual_marker_eye.setZero();

        // Equation
        //attitude_visual_marker_wrt_visual_marker_eye=
        attitude_map_element_wrt_sensor=
                Quaternion::cross(Quaternion::inv(attitude_visual_marker_eye_wrt_world), attitude_map_element_wrt_world);


        // Set
        //predictedMeasurement->setVisualMarkerAttitude(attitude_visual_marker_wrt_visual_marker_eye);

    }

    return 0;
}


int CodedVisualMarkerEyeCore::predictErrorMeasurementJacobian(// Time
                                                              const TimeStamp &current_time_stamp,
                                                              // Current State
                                                              const std::shared_ptr<StateEstimationCore> &current_state,
                                                              // Measurements
                                                              const std::shared_ptr<SensorMeasurementCore> &measurement,
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

    CodedVisualMarkerMeasurementCore* sensor_measurement=dynamic_cast<CodedVisualMarkerMeasurementCore*>(measurement.get());

    // Nothing else needed


    // Predicted Measurement
    if(!predicted_measurement)
        return -3;



    // Search for the sensor State Core
    CodedVisualMarkerEyeStateCore* current_sensor_state(nullptr);
    if(findSensorState(current_state->TheListSensorStateCore, current_sensor_state))
        return -2;
    if(!current_sensor_state)
        return -10;


    // Search for the associated map element
    CodedVisualMarkerLandmarkStateCore* current_map_element_state(nullptr);
    if(findMapElementState(current_state->TheListMapElementStateCore, sensor_measurement, current_map_element_state))
        return 1;
    if(!current_map_element_state)
        return 1;


    //// Init Jacobians
    int error_init_jacobians=predictErrorMeasurementJacobianInit(// Current State
                                                                current_state,
                                                                // Predicted Measurements
                                                                predicted_measurement);

    if(error_init_jacobians)
        return error_init_jacobians;



    /// Predicted measurement cast
    CodedVisualMarkerMeasurementCore* predicted_sensor_measurement(nullptr);
    if(measurement->getSensorCoreSharedPtr() != predicted_measurement->getSensorCoreSharedPtr())
        return -3;
    predicted_sensor_measurement=dynamic_cast<CodedVisualMarkerMeasurementCore*>(predicted_measurement.get());




    /// Get iterators to fill jacobians

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
        if( dynamic_cast<CodedVisualMarkerEyeStateCore*>((*itSensorStateCore).get()) == current_sensor_state )
            break;
    }

    // Map Element
    std::vector<Eigen::SparseMatrix<double> >::iterator it_jacobian_error_measurement_wrt_map_element_error_state;
    it_jacobian_error_measurement_wrt_map_element_error_state=predicted_sensor_measurement->jacobian_error_measurement_wrt_error_state_.map_elements.begin();

    std::vector<Eigen::SparseMatrix<double> >::iterator it_jacobian_error_measurement_wrt_map_element_error_parameters;
    it_jacobian_error_measurement_wrt_map_element_error_parameters=predicted_sensor_measurement->jacobian_error_measurement_wrt_error_parameters_.map_elements.begin();

    for(std::list< std::shared_ptr<StateCore> >::iterator itMapElementStateCore=current_state->TheListMapElementStateCore.begin();
        itMapElementStateCore!=current_state->TheListMapElementStateCore.end();
        ++itMapElementStateCore, ++it_jacobian_error_measurement_wrt_map_element_error_state, ++it_jacobian_error_measurement_wrt_map_element_error_parameters
        )
    {
        if( dynamic_cast<CodedVisualMarkerLandmarkStateCore*>((*itMapElementStateCore).get()) == current_map_element_state )
            break;
    }


    /// Fill Jacobians
    int error_predict_measurement=predictErrorMeasurementJacobianSpecific(current_time_stamp,
                                                                          dynamic_cast<RobotStateCore*>(current_state->TheRobotStateCore.get()),
                                                                          current_sensor_state,
                                                                          current_map_element_state,
                                                                          sensor_measurement,
                                                                          predicted_sensor_measurement,
                                                                          // Jacobians State
                                                                          // Robot
                                                                          predicted_sensor_measurement->jacobian_error_measurement_wrt_error_state_.robot,
                                                                          predicted_sensor_measurement->jacobian_error_measurement_wrt_error_parameters_.robot,
                                                                          // Sensor
                                                                          (*it_jacobian_error_measurement_wrt_sensor_error_state),
                                                                          (*it_jacobian_error_measurement_wrt_sensor_error_parameters),
                                                                          // Map Element
                                                                          (*it_jacobian_error_measurement_wrt_map_element_error_state),
                                                                          (*it_jacobian_error_measurement_wrt_map_element_error_parameters),
                                                                          // Jacobian Measurement
                                                                          predicted_sensor_measurement->jacobian_error_measurement_wrt_error_measurement_.measurement
                                                                          );

    // Check error
    if(error_predict_measurement)
        return error_predict_measurement;



    // End
    return 0;
}


int CodedVisualMarkerEyeCore::predictErrorMeasurementJacobianSpecific(const TimeStamp& theTimeStamp,
                                                                    const RobotStateCore *currentRobotState,
                                                                    const CodedVisualMarkerEyeStateCore *currentSensorState,
                                                                    const CodedVisualMarkerLandmarkStateCore *currentMapElementState,
                                                                    const CodedVisualMarkerMeasurementCore *matchedMeasurement,
                                                                    CodedVisualMarkerMeasurementCore *&predictedMeasurement,
                                                                    // Jacobians State
                                                                    // Robot
                                                                    Eigen::SparseMatrix<double>& jacobian_error_measurement_wrt_robot_error_state,
                                                                    Eigen::SparseMatrix<double>& jacobian_error_measurement_wrt_robot_error_parameters,
                                                                    // Sensor
                                                                    Eigen::SparseMatrix<double>& jacobian_error_measurement_wrt_sensor_error_state,
                                                                    Eigen::SparseMatrix<double>& jacobian_error_measurement_wrt_sensor_error_parameters,
                                                                    // Map Element
                                                                    Eigen::SparseMatrix<double>& jacobian_error_measurement_wrt_map_element_error_state,
                                                                    Eigen::SparseMatrix<double>& jacobian_error_measurement_wrt_map_element_error_parameters,
                                                                    // Jacobians Noise
                                                                    Eigen::SparseMatrix<double>& jacobian_error_measurement_wrt_error_measurement
                                                          )
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

    // Measurement
    Eigen::Vector3d meas_position_map_element_wrt_sensor;
    Eigen::Vector4d meas_attitude_map_element_wrt_sensor;
    // Predicted Measurement
    Eigen::Vector3d position_map_element_wrt_sensor;
    Eigen::Vector4d attitude_map_element_wrt_sensor;


    // Robot
    position_robot_wrt_world=currentRobotState->getPositionRobotWrtWorld();
    attitude_robot_wrt_world=currentRobotState->getAttitudeRobotWrtWorld();


    // Sensor
    // Cast
    std::shared_ptr<const CodedVisualMarkerEyeCore> the_sensor_core=std::dynamic_pointer_cast<const CodedVisualMarkerEyeCore>(currentSensorState->getMsfElementCoreSharedPtr());

    position_sensor_wrt_robot=currentSensorState->getPositionSensorWrtRobot();
    attitude_sensor_wrt_robot=currentSensorState->getAttitudeSensorWrtRobot();


    // Map Element
    // Cast
    std::shared_ptr<CodedVisualMarkerLandmarkCore> TheMapElementCore=std::dynamic_pointer_cast<CodedVisualMarkerLandmarkCore>(currentMapElementState->getMsfElementCoreSharedPtr());

    position_map_element_wrt_world=currentMapElementState->getPosition();
    attitude_map_element_wrt_world=currentMapElementState->getAttitude();


    // Matched measurement
    meas_position_map_element_wrt_sensor=matchedMeasurement->getVisualMarkerPosition();
    meas_attitude_map_element_wrt_sensor=matchedMeasurement->getVisualMarkerAttitude();


    // Predicted Measurement
    position_map_element_wrt_sensor=predictedMeasurement->getVisualMarkerPosition();
    attitude_map_element_wrt_sensor=predictedMeasurement->getVisualMarkerAttitude();



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


    int error_jacobians_error_measurements_core=jacobiansErrorMeasurementsCore(position_robot_wrt_world, attitude_robot_wrt_world,
                                                                               position_sensor_wrt_robot, attitude_sensor_wrt_robot,
                                                                               position_map_element_wrt_world, attitude_map_element_wrt_world,
                                                                               meas_position_map_element_wrt_sensor, meas_attitude_map_element_wrt_sensor,
                                                                               position_map_element_wrt_sensor, attitude_map_element_wrt_sensor,
                                           // Jacobians: State and Params
                                           jacobian_error_meas_pos_wrt_error_state_robot_pos, jacobian_error_meas_pos_wrt_error_state_robot_att, jacobian_error_meas_att_wrt_error_state_robot_att,
                                           jacobian_error_meas_pos_wrt_error_state_sens_pos, jacobian_error_meas_pos_wrt_error_state_sens_att, jacobian_error_meas_att_wrt_error_state_sens_att,
                                           jacobian_error_meas_pos_wrt_error_state_map_elem_pos, jacobian_error_meas_att_wrt_error_state_map_elem_att,
                                           // Jacobians: Noise
                                           jacobian_error_meas_pos_wrt_error_meas_pos, jacobian_error_meas_att_wrt_error_meas_att);


    if(error_jacobians_error_measurements_core)
        return error_jacobians_error_measurements_core;




    /// dimensions

    // Dimension robot error state
    int dimension_robot_error_state=currentRobotState->getMsfElementCoreSharedPtr()->getDimensionErrorState();
    int dimension_robot_error_parameters=currentRobotState->getMsfElementCoreSharedPtr()->getDimensionErrorParameters();

    // Dimension sensor
    int dimension_sensor_error_state=the_sensor_core->getDimensionErrorState();
    int dimension_sensor_error_parameters=the_sensor_core->getDimensionErrorParameters();

    // Dimension map element
    int dimension_map_element_error_state=currentMapElementState->getMsfElementCoreSharedPtr()->getDimensionErrorState();
    int dimension_map_element_error_parameters=currentMapElementState->getMsfElementCoreSharedPtr()->getDimensionErrorParameters();




    /// All jacobians


    //// Jacobian Measurement Error - Error State / Error Parameters


    /// Jacobian Measurement Error - World Error State / Error Parameters

    {
        // Fill
        // No dependency on global parameters -> Everything is set to zero
    }


    /// Jacobian Measurement Error - Robot Error State / Error Parameters

    {
        // Resize and init
        jacobian_error_measurement_wrt_robot_error_state.resize(dimension_error_measurement_, dimension_robot_error_state);
        jacobian_error_measurement_wrt_robot_error_parameters.resize(dimension_error_measurement_, dimension_robot_error_parameters);

        std::vector<Eigen::Triplet<double>> triplet_list_jacobian_error_measurement_wrt_error_state;
        std::vector<Eigen::Triplet<double>> triplet_list_jacobian_error_measurement_wrt_error_parameters;

        // Fill
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
//                    predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementRobotErrorState.block<3,3>(dimension_error_measurement_i,0)=
//                            jacobian_error_meas_pos_wrt_error_state_robot_pos;
                    BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_state, jacobian_error_meas_pos_wrt_error_state_robot_pos, dimension_error_measurement_i, 0);

                    // lin_vel
                    // zeros

                    // lin_acc
                    // zeros

                    // attit
//                    predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementRobotErrorState.block<3,3>(dimension_error_measurement_i,9)=
//                            jacobian_error_meas_pos_wrt_error_state_robot_att;
                    BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_state, jacobian_error_meas_pos_wrt_error_state_robot_att, dimension_error_measurement_i, 9);

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
//                    predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementRobotErrorState.block<3,3>(dimension_error_measurement_i,0)=
//                            jacobian_error_meas_pos_wrt_error_state_robot_pos;
                    BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_state, jacobian_error_meas_pos_wrt_error_state_robot_pos, dimension_error_measurement_i, 0);

                    // lin_vel
                    // zeros

                    // lin_acc
                    // zeros

                    // attit
//                    predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementRobotErrorState.block<3,3>(dimension_error_measurement_i,9)=
//                            jacobian_error_meas_pos_wrt_error_state_robot_att;
                    BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_state, jacobian_error_meas_pos_wrt_error_state_robot_att, dimension_error_measurement_i, 9);

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
//                    predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementRobotErrorState.block<3,3>(dimension_error_measurement_i,9)=
//                            jacobian_error_meas_att_wrt_error_state_robot_att;
                    BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_state, jacobian_error_meas_att_wrt_error_state_robot_att, dimension_error_measurement_i, 9);

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
//                    predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementRobotErrorState.block<3,3>(dimension_error_measurement_i,9)=
//                            jacobian_error_meas_att_wrt_error_state_robot_att;
                    BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_state, jacobian_error_meas_att_wrt_error_state_robot_att, dimension_error_measurement_i, 9);

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

        // Set From Triplets
        jacobian_error_measurement_wrt_robot_error_state.setFromTriplets(triplet_list_jacobian_error_measurement_wrt_error_state.begin(), triplet_list_jacobian_error_measurement_wrt_error_state.end());
        jacobian_error_measurement_wrt_robot_error_parameters.setFromTriplets(triplet_list_jacobian_error_measurement_wrt_error_parameters.begin(), triplet_list_jacobian_error_measurement_wrt_error_parameters.end());

    }


    /// Jacobian Measurement Error - Inputs Error State / Error Parameters

    {
        // Resize and init
        // Nothing to do

        // Fill
        // No dependency on global parameters -> Everything is set to zero
    }


    /// Jacobian Measurement Error - Sensor Error State & Error Parameters

    {

        // Resize and init
        jacobian_error_measurement_wrt_sensor_error_state.resize(dimension_error_measurement_, dimension_sensor_error_state);
        jacobian_error_measurement_wrt_sensor_error_parameters.resize(dimension_error_measurement_, dimension_sensor_error_parameters);


        std::vector<Eigen::Triplet<double>> triplet_list_jacobian_error_measurement_wrt_error_state;
        std::vector<Eigen::Triplet<double>> triplet_list_jacobian_error_measurement_wrt_error_parameters;



        // Fill
        int dimension_error_measurement_i=0;

        // pos
        if(this->isMeasurementPositionEnabled())
        {
            int dimension_sensor_error_parameters_i=0;
            int dimension_sensor_error_state_i=0;

            // pos
            if(the_sensor_core->isEstimationPositionSensorWrtRobotEnabled())
            {
//                predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState.block<3, 3>(dimension_error_measurement_i, dimension_sensor_error_state_i)=
//                        jacobian_error_meas_pos_wrt_error_state_sens_pos;
                BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_state, jacobian_error_meas_pos_wrt_error_state_sens_pos, dimension_error_measurement_i, dimension_sensor_error_state_i);

                dimension_sensor_error_state_i+=3;
            }
            else
            {
//                predictedMeasurement->jacobianMeasurementErrorParameters.jacobianMeasurementSensorParameters.block<3,3>(dimension_error_measurement_i, dimension_sensor_error_parameters_i)=
//                        jacobian_error_meas_pos_wrt_error_state_sens_pos;
                BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_parameters, jacobian_error_meas_pos_wrt_error_state_sens_pos, dimension_error_measurement_i, dimension_sensor_error_parameters_i);

                dimension_sensor_error_parameters_i+=3;
            }

            // att
            if(the_sensor_core->isEstimationAttitudeSensorWrtRobotEnabled())
            {
//                predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState.block<3, 3>(dimension_error_measurement_i, dimension_sensor_error_state_i)=
//                        jacobian_error_meas_pos_wrt_error_state_sens_att;
                BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_state, jacobian_error_meas_pos_wrt_error_state_sens_att, dimension_error_measurement_i, dimension_sensor_error_state_i);

                dimension_sensor_error_state_i+=3;
            }
            else
            {
//                predictedMeasurement->jacobianMeasurementErrorParameters.jacobianMeasurementSensorParameters.block<3,3>(dimension_error_measurement_i, dimension_sensor_error_parameters_i)=
//                        jacobian_error_meas_pos_wrt_error_state_sens_att;
                BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_parameters, jacobian_error_meas_pos_wrt_error_state_sens_att, dimension_error_measurement_i, dimension_sensor_error_parameters_i);

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
            if(the_sensor_core->isEstimationPositionSensorWrtRobotEnabled())
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
//                predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState.block<3,3>(dimension_error_measurement_i, dimension_sensor_error_state_i)=
//                        jacobian_error_meas_att_wrt_error_state_sens_att;
                BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_state, jacobian_error_meas_att_wrt_error_state_sens_att, dimension_error_measurement_i, dimension_sensor_error_state_i);

                dimension_sensor_error_state_i+=3;
            }
            else
            {
//                predictedMeasurement->jacobianMeasurementErrorParameters.jacobianMeasurementSensorParameters.block<3,3>(dimension_error_measurement_i, dimension_sensor_error_parameters_i)=
//                        jacobian_error_meas_att_wrt_error_state_sens_att;
                BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_parameters, jacobian_error_meas_att_wrt_error_state_sens_att, dimension_error_measurement_i, dimension_sensor_error_parameters_i);

                dimension_sensor_error_parameters_i+=3;
            }

            dimension_error_measurement_i+=3;
        }


        // Set From Triplets
        jacobian_error_measurement_wrt_sensor_error_state.setFromTriplets(triplet_list_jacobian_error_measurement_wrt_error_state.begin(), triplet_list_jacobian_error_measurement_wrt_error_state.end());
        jacobian_error_measurement_wrt_sensor_error_parameters.setFromTriplets(triplet_list_jacobian_error_measurement_wrt_error_parameters.begin(), triplet_list_jacobian_error_measurement_wrt_error_parameters.end());

    }


    /// Jacobian Measurement Error - Map Element Error State & Error Parameters

    {
        // Resize and init
        jacobian_error_measurement_wrt_map_element_error_state.resize(dimension_error_measurement_, dimension_map_element_error_state);
        jacobian_error_measurement_wrt_map_element_error_parameters.resize(dimension_error_measurement_, dimension_map_element_error_parameters);


        std::vector<Eigen::Triplet<double>> triplet_list_jacobian_error_measurement_wrt_error_state;
        std::vector<Eigen::Triplet<double>> triplet_list_jacobian_error_measurement_wrt_error_parameters;



        // Fill
        int dimension_error_measurement_i=0;

        // pos
        if(this->isMeasurementPositionEnabled())
        {
            int dimension_map_element_error_parameters_i=0;
            int dimension_map_element_error_state_i=0;

            // pos
            if(TheMapElementCore->isEstimationPositionVisualMarkerWrtWorldEnabled())
            {
//                predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementMapElementErrorState.block<3,3>(dimension_error_measurement_i, dimension_map_element_error_state_i)=
//                        jacobian_error_meas_pos_wrt_error_state_map_elem_pos;
                BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_state, jacobian_error_meas_pos_wrt_error_state_map_elem_pos, dimension_error_measurement_i, dimension_map_element_error_state_i);

                dimension_map_element_error_state_i+=3;
            }
            else
            {
//                predictedMeasurement->jacobianMeasurementErrorParameters.jacobianMeasurementMapElementParameters.block<3,3>(dimension_error_measurement_i, dimension_map_element_error_parameters_i)=
//                        jacobian_error_meas_pos_wrt_error_state_map_elem_pos;
                BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_parameters, jacobian_error_meas_pos_wrt_error_state_map_elem_pos, dimension_error_measurement_i, dimension_map_element_error_parameters_i);

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
//                predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementMapElementErrorState.block<3,3>(dimension_error_measurement_i, dimension_map_element_error_state_i)=
//                        jacobian_error_meas_att_wrt_error_state_map_elem_att;
                BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_state, jacobian_error_meas_att_wrt_error_state_map_elem_att, dimension_error_measurement_i, dimension_map_element_error_state_i);

                dimension_map_element_error_state_i+=3;
            }
            else
            {
//                predictedMeasurement->jacobianMeasurementErrorParameters.jacobianMeasurementMapElementParameters.block<3,3>(dimension_error_measurement_i, dimension_map_element_error_parameters_i)=
//                        jacobian_error_meas_att_wrt_error_state_map_elem_att;
                BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_parameters, jacobian_error_meas_att_wrt_error_state_map_elem_att, dimension_error_measurement_i, dimension_map_element_error_parameters_i);

                dimension_map_element_error_parameters_i+=3;
            }

            dimension_error_measurement_i+=3;
        }


        // Set From Triplets
        jacobian_error_measurement_wrt_map_element_error_state.setFromTriplets(triplet_list_jacobian_error_measurement_wrt_error_state.begin(), triplet_list_jacobian_error_measurement_wrt_error_state.end());
        jacobian_error_measurement_wrt_map_element_error_parameters.setFromTriplets(triplet_list_jacobian_error_measurement_wrt_error_parameters.begin(), triplet_list_jacobian_error_measurement_wrt_error_parameters.end());

    }




    /// Jacobians error measurement - sensor noise of the measurement

    {
        // Resize and init Jacobian
        jacobian_error_measurement_wrt_error_measurement.resize(dimension_error_measurement_, dimension_error_measurement_);

        std::vector<Eigen::Triplet<double>> triplet_list_jacobian_error_measurement_wrt_error_measurement;


        // Fill
        int dimension_error_measurement_i=0;

        // pos
        if(this->isMeasurementPositionEnabled())
        {
//            predictedMeasurement->jacobianMeasurementSensorNoise.jacobianMeasurementSensorNoise.block<3,3>(dimension_error_measurement_i, dimension_error_measurement_i)=
//                    jacobian_error_meas_pos_wrt_error_meas_pos;
            BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_measurement, jacobian_error_meas_pos_wrt_error_meas_pos, dimension_error_measurement_i, dimension_error_measurement_i);

            dimension_error_measurement_i+=3;
        }


        // att
        if(this->isMeasurementAttitudeEnabled())
        {
//            predictedMeasurement->jacobianMeasurementSensorNoise.jacobianMeasurementSensorNoise.block<3,3>(dimension_error_measurement_i, dimension_error_measurement_i)=
//                    jacobian_error_meas_att_wrt_error_meas_att;
            BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_measurement, jacobian_error_meas_att_wrt_error_meas_att, dimension_error_measurement_i, dimension_error_measurement_i);

            dimension_error_measurement_i+=3;
        }


        // Set From Triplets
        jacobian_error_measurement_wrt_error_measurement.setFromTriplets(triplet_list_jacobian_error_measurement_wrt_error_measurement.begin(), triplet_list_jacobian_error_measurement_wrt_error_measurement.end());

    }


    /// End
    return 0;
}

int CodedVisualMarkerEyeCore::jacobiansErrorMeasurementsCore(// State: Robot
                                                             const Eigen::Vector3d& position_robot_wrt_world, const Eigen::Vector4d& attitude_robot_wrt_world,
                                                             // State: Sensor
                                                             const Eigen::Vector3d& position_sensor_wrt_robot, const Eigen::Vector4d& attitude_sensor_wrt_robot,
                                                             // State: Map
                                                             const Eigen::Vector3d& position_map_element_wrt_world, const Eigen::Vector4d& attitude_map_element_wrt_world,
                                                             // Measurement
                                                             const Eigen::Vector3d& meas_position_map_element_wrt_sensor, const Eigen::Vector4d& meas_attitude_map_element_wrt_sensor,
                                                             // Predicted Measurement
                                                             const Eigen::Vector3d& position_map_element_wrt_sensor, const Eigen::Vector4d& attitude_map_element_wrt_sensor,
                                                             // Jacobians: State and Params
                                                             Eigen::Matrix3d& jacobian_error_meas_pos_wrt_error_state_robot_pos, Eigen::Matrix3d& jacobian_error_meas_pos_wrt_error_state_robot_att, Eigen::Matrix3d& jacobian_error_meas_att_wrt_error_state_robot_att,
                                                             Eigen::Matrix3d& jacobian_error_meas_pos_wrt_error_state_sens_pos, Eigen::Matrix3d& jacobian_error_meas_pos_wrt_error_state_sens_att, Eigen::Matrix3d& jacobian_error_meas_att_wrt_error_state_sens_att,
                                                             Eigen::Matrix3d& jacobian_error_meas_pos_wrt_error_state_map_elem_pos, Eigen::Matrix3d& jacobian_error_meas_att_wrt_error_state_map_elem_att,
                                                             // Jacobians: Noise
                                                             Eigen::Matrix3d& jacobian_error_meas_pos_wrt_error_meas_pos, Eigen::Matrix3d& jacobian_error_meas_att_wrt_error_meas_att)
{

    // Auxiliar variables
    Eigen::Vector3d tran_inc_wrt_world=position_map_element_wrt_world-position_robot_wrt_world-Quaternion::cross_sandwich(attitude_robot_wrt_world, position_sensor_wrt_robot, Quaternion::inv(attitude_robot_wrt_world));
    Eigen::Vector3d tran_inc2_wrt_world=position_map_element_wrt_world-position_robot_wrt_world;

    //Eigen::Vector4d att_pred_visual_marker_wrt_visual_marker_eye=TheMatchedMeasurementCore->getVisualMarkerAttitude();
    Eigen::Vector4d att_pred_visual_marker_wrt_visual_marker_eye=attitude_map_element_wrt_sensor;

    Eigen::Vector4d att_meas_visual_marker_wrt_visual_marker_eye=meas_attitude_map_element_wrt_sensor;

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

int CodedVisualMarkerEyeCore::resetErrorStateJacobian(// Time
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


    current_state->jacobian_error_state_reset_.setFromTriplets(triplets_jacobian_error_reset.begin(), triplets_jacobian_error_reset.end());

    // End
    return 0;
}

int CodedVisualMarkerEyeCore::mapMeasurement(// Time
                   const TimeStamp &current_time_stamp,
                   // Current State
                   const std::shared_ptr<StateEstimationCore> &current_state,
                   // Current Measurement
                   const std::shared_ptr<SensorMeasurementCore> &current_measurement,
                   // List Map Element Core -> New will be added if not available
                   std::list< std::shared_ptr<MapElementCore> >& list_map_element_core,
                   // New Map Element State Core
                   std::shared_ptr<StateCore> &new_map_element_state)
{
    // Checks
    if(!current_state)
        return -1;

    // Robot State
    RobotStateCore* current_robot_state=dynamic_cast<RobotStateCore*>(current_state->TheRobotStateCore.get());

    // Sensor State
    CodedVisualMarkerEyeStateCore* current_sensor_state(nullptr);
    if(findSensorState(current_state->TheListSensorStateCore, current_sensor_state))
        return -2;
    if(!current_sensor_state)
        return -10;

    // Current Measurement
    if(!current_measurement)
        return -1;
    if(current_measurement->getSensorCoreSharedPtr() != std::dynamic_pointer_cast<SensorCore>(this->getMsfElementCoreSharedPtr()))
        return -10;
    CodedVisualMarkerMeasurementCore* current_sensor_measurement=dynamic_cast<CodedVisualMarkerMeasurementCore*>(current_measurement.get());

    // Find the map element core
    bool flag_new_map_element_core_not_found;
    CodedVisualMarkerLandmarkCore* new_map_element_core(nullptr);
    if(findMapElementCore(list_map_element_core,
                          current_sensor_measurement,
                          new_map_element_core))
    {
        // Need to be mapped
        flag_new_map_element_core_not_found=true;
    }
    if(!new_map_element_core)
        flag_new_map_element_core_not_found=true;


    // Create the map element
    if(flag_new_map_element_core_not_found)
    {
        // Map element core
        new_map_element_core=new CodedVisualMarkerLandmarkCore;

        // Set the storage core
        new_map_element_core->setMsfStorageCorePtr(this->getMsfStorageCoreWeakPtr());

        // Set the map core shared_ptr
        std::shared_ptr<MapElementCore> new_map_element_core_shared_ptr=std::shared_ptr<MapElementCore>(new_map_element_core);
        new_map_element_core->setMsfElementCorePtr(new_map_element_core_shared_ptr);

        // Push into the list
        list_map_element_core.push_back(new_map_element_core_shared_ptr);


        // Configurations of the map element core. Only needed if was not previously set. Can be done anycase.

        // Sensor Core Cast
        std::shared_ptr<CodedVisualMarkerEyeCore> TheSensorCore=std::dynamic_pointer_cast<CodedVisualMarkerEyeCore>(current_sensor_state->getMsfElementCoreSharedPtr());


        // Set id in the map element core
        if(new_map_element_core->setId(current_sensor_measurement->getVisualMarkerId()))
            return 2;

        // Set name
        if(new_map_element_core->setMapElementName("visual_marker_"+std::to_string(new_map_element_core->getId())))
            return 3;


        // Set Default configurations
        // Enable Estimation Position if measurement pos is enabled
        // TODO Check!
        if(TheSensorCore->isMeasurementPositionEnabled())
            new_map_element_core->enableEstimationPositionVisualMarkerWrtWorld();
        else
            new_map_element_core->enableParameterPositionVisualMarkerWrtWorld();

        // Enable Estimation Attitude if measurement att is enabled
        // TODO Check!
        if(TheSensorCore->isMeasurementAttitudeEnabled())
            new_map_element_core->enableEstimationAttitudeVisualMarkerWrtWorld();
        else
            new_map_element_core->enableParameterAttitudeVisualMarkerWrtWorld();

        // Covariances not needed because are included in P.
        // TODO check!



    }


    // New map element state
    CodedVisualMarkerLandmarkStateCore* new_landmark_state(nullptr);
    if(!new_map_element_state)
    {
        new_landmark_state=new CodedVisualMarkerLandmarkStateCore;
        new_landmark_state->setMsfElementCorePtr(new_map_element_core->getMsfElementCoreWeakPtr());
        new_map_element_state=std::shared_ptr<StateCore>(new_landmark_state);
    }
    else
    {
        new_landmark_state=dynamic_cast<CodedVisualMarkerLandmarkStateCore*>(new_map_element_state.get());
    }


    // Call the specific
    if(mapMeasurementSpecific(current_time_stamp,
                              current_robot_state,
                              current_sensor_state,
                              current_sensor_measurement,
                              new_landmark_state))
        return -30;




    // End
    return 0;
}

int CodedVisualMarkerEyeCore::mapMeasurementSpecific(const TimeStamp &theTimeStamp,
                                             const RobotStateCore *currentRobotState,
                                             const CodedVisualMarkerEyeStateCore* currentSensorState,
                                             const CodedVisualMarkerMeasurementCore* matchedMeasurement,
                                             CodedVisualMarkerLandmarkStateCore *&newMapElementState)
{
    /// Checks

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

    // New state
    Eigen::Vector3d position_map_element_wrt_world;
    Eigen::Vector4d attitude_map_element_wrt_world;


    // Robot
    position_robot_wrt_world=currentRobotState->getPositionRobotWrtWorld();
    attitude_robot_wrt_world=currentRobotState->getAttitudeRobotWrtWorld();

    // Sensor
    position_sensor_wrt_robot=currentSensorState->getPositionSensorWrtRobot();
    attitude_sensor_wrt_robot=currentSensorState->getAttitudeSensorWrtRobot();


    // Measurement: Map Element
    position_map_element_wrt_sensor=matchedMeasurement->getVisualMarkerPosition();
    attitude_map_element_wrt_sensor=matchedMeasurement->getVisualMarkerAttitude();




    //// State Estimation

    /// Core

    int error_map_measurement_core=this->mapMeasurementCore(position_robot_wrt_world, attitude_robot_wrt_world, position_sensor_wrt_robot, attitude_sensor_wrt_robot, position_map_element_wrt_sensor, attitude_map_element_wrt_sensor, position_map_element_wrt_world, attitude_map_element_wrt_world);

    if(error_map_measurement_core)
        return error_map_measurement_core;


    /// Set values

    newMapElementState->position_=position_map_element_wrt_world;
    newMapElementState->attitude_=attitude_map_element_wrt_world;



    // End
    return 0;
}

int CodedVisualMarkerEyeCore::mapMeasurementCore(// robot wrt world (state)
                                                 const Eigen::Vector3d& position_robot_wrt_world, const Eigen::Vector4d& attitude_robot_wrt_world,
                                                 // sensor wrt world (state)
                                                 const Eigen::Vector3d& position_sensor_wrt_robot, const Eigen::Vector4d& attitude_sensor_wrt_robot,
                                                 // Map element wrt sensor (measurement)
                                                 const Eigen::Vector3d& meas_position_map_element_wrt_sensor, const Eigen::Vector4d& meas_attitude_map_element_wrt_sensor,
                                                 // Map element wrt world (state new)
                                                 Eigen::Vector3d& position_map_element_wrt_world, Eigen::Vector4d& attitude_map_element_wrt_world)
{

    // Position
    if(this->isMeasurementPositionEnabled())
    {
        Eigen::Vector3d tran_visual_marker_wrt_robot=Quaternion::cross_sandwich(attitude_sensor_wrt_robot, meas_position_map_element_wrt_sensor, Quaternion::inv(attitude_sensor_wrt_robot)) + position_sensor_wrt_robot;
        position_map_element_wrt_world=Quaternion::cross_sandwich(attitude_robot_wrt_world, tran_visual_marker_wrt_robot, Quaternion::inv(attitude_robot_wrt_world)) + position_robot_wrt_world;
    }


    // Attitude
    if(this->isMeasurementAttitudeEnabled())
    {
        attitude_map_element_wrt_world=Quaternion::cross(attitude_robot_wrt_world, attitude_sensor_wrt_robot, meas_attitude_map_element_wrt_sensor);
    }


    return 0;
}

int CodedVisualMarkerEyeCore::jacobiansMapMeasurement(// Time
                   const TimeStamp& current_time_stamp,
                   // Current State
                   const std::shared_ptr<StateEstimationCore>& current_state,
                   // Current Measurement
                   const std::shared_ptr<SensorMeasurementCore>& current_measurement,
                   // New Map Element State Core
                   std::shared_ptr<StateCore> &new_map_element_state)
{
    // Checks
    if(!current_state)
        return -1;

    // Robot State
    RobotStateCore* current_robot_state=dynamic_cast<RobotStateCore*>(current_state->TheRobotStateCore.get());

    // Sensor State
    CodedVisualMarkerEyeStateCore* current_sensor_state(nullptr);
    if(findSensorState(current_state->TheListSensorStateCore, current_sensor_state))
        return -2;
    if(!current_sensor_state)
        return -10;

    // Current Measurement
    if(!current_measurement)
        return -1;
    if(current_measurement->getSensorCoreSharedPtr() != std::dynamic_pointer_cast<SensorCore>(this->getMsfElementCoreSharedPtr()))
        return -10;
    CodedVisualMarkerMeasurementCore* current_sensor_measurement=dynamic_cast<CodedVisualMarkerMeasurementCore*>(current_measurement.get());

    // New map element state
    if(!new_map_element_state)
        return -5;
    if(!new_map_element_state->getMsfElementCoreSharedPtr())
        return -6;
    CodedVisualMarkerLandmarkStateCore* new_landmark_state=dynamic_cast<CodedVisualMarkerLandmarkStateCore*>(new_map_element_state.get());


    // Call the specific
    if(jacobiansMapMeasurementSpecific(current_time_stamp, current_robot_state, current_sensor_state, current_sensor_measurement, new_landmark_state))
        return -30;


    return 0;
}

int CodedVisualMarkerEyeCore::jacobiansMapMeasurementSpecific(const TimeStamp &theTimeStamp,
                                                              const RobotStateCore *currentRobotState,
                                                              const CodedVisualMarkerEyeStateCore *currentSensorState,
                                                              const CodedVisualMarkerMeasurementCore *matchedMeasurement,
                                                              CodedVisualMarkerLandmarkStateCore *&newMapElementState)
{
    /// Checks

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
    // state: Robot
    Eigen::Vector3d position_robot_wrt_world;
    Eigen::Vector4d attitude_robot_wrt_world;
    // state: Sensor
    Eigen::Vector3d position_sensor_wrt_robot;
    Eigen::Vector4d attitude_sensor_wrt_robot;
    // state: Map element wrt world
    Eigen::Vector3d position_map_element_wrt_world;
    Eigen::Vector4d attitude_map_element_wrt_world;
    // Measurement: Map Element
    Eigen::Vector3d position_map_element_wrt_sensor;
    Eigen::Vector4d attitude_map_element_wrt_sensor;



    // Robot
    position_robot_wrt_world=currentRobotState->getPositionRobotWrtWorld();
    attitude_robot_wrt_world=currentRobotState->getAttitudeRobotWrtWorld();


    // Sensor
    // Cast
    std::shared_ptr<CodedVisualMarkerEyeCore> TheSensorCore=std::dynamic_pointer_cast<CodedVisualMarkerEyeCore>(currentSensorState->getMsfElementCoreSharedPtr());
    position_sensor_wrt_robot=currentSensorState->getPositionSensorWrtRobot();
    attitude_sensor_wrt_robot=currentSensorState->getAttitudeSensorWrtRobot();


    // Measurement: Map Element
    position_map_element_wrt_sensor=matchedMeasurement->getVisualMarkerPosition();
    attitude_map_element_wrt_sensor=matchedMeasurement->getVisualMarkerAttitude();



    // Map Element State Core
    // Cast
    std::shared_ptr<CodedVisualMarkerLandmarkCore> TheCodeCodedVisualMarkerLandmarkCore=std::dynamic_pointer_cast<CodedVisualMarkerLandmarkCore>(newMapElementState->getMsfElementCoreSharedPtr());

    position_map_element_wrt_world=newMapElementState->getPosition();
    attitude_map_element_wrt_world=newMapElementState->getAttitude();


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


    int error_jacobians_map_measurement_core=this->jacobiansMapMeasurementCore(position_robot_wrt_world, attitude_robot_wrt_world,
                                                                               position_sensor_wrt_robot, attitude_sensor_wrt_robot,
                                                                               position_map_element_wrt_world, attitude_map_element_wrt_world,
                                                                               position_map_element_wrt_sensor, attitude_map_element_wrt_sensor,
                                                                              // Jacobians State and Params
                                                                              jacobian_error_map_pos_wrt_error_state_robot_pos, jacobian_error_map_pos_wrt_error_state_robot_att, jacobian_error_map_att_wrt_error_state_robot_att,
                                                                              jacobian_error_map_pos_wrt_error_state_sens_pos, jacobian_error_map_pos_wrt_error_state_sens_att, jacobian_error_map_att_wrt_error_state_sens_att,
                                                                              // Jacobians Measurement
                                                                              jacobian_error_map_pos_wrt_error_meas_pos, jacobian_error_map_att_wrt_error_meas_att);

    if(error_jacobians_map_measurement_core)
        return error_jacobians_map_measurement_core;


    //// Dimensions


    // Robot
    int dimension_robot_error_state_total=currentRobotState->getMsfElementCoreSharedPtr()->getDimensionErrorState();

    // Sensor
    int dimension_sensor_error_state_total=currentSensorState->getMsfElementCoreSharedPtr()->getDimensionErrorState();

    // Measurement
    int dimension_map_new_element_measurement=matchedMeasurement->getSensorCoreSharedPtr()->getDimensionErrorMeasurement();

    // New Map element
    int dimension_map_new_element_error_state_total=TheCodeCodedVisualMarkerLandmarkCore->getDimensionErrorState();




    //// Jacobian



    //// Jacobian Mapping Error-State


    /// Robot

    {
        newMapElementState->jacobian_mapping_error_state_.jacobian_mapping_robot_error_state_.resize(dimension_map_new_element_error_state_total, dimension_robot_error_state_total);
        newMapElementState->jacobian_mapping_error_state_.jacobian_mapping_robot_error_state_.setZero();


        // tran landmark -> Enabled by default -> TODO Check

        // Switch depending on robot used
        switch(std::dynamic_pointer_cast<RobotCore>(currentRobotState->getMsfElementCoreSharedPtr())->getRobotCoreType())
        {
            // Free model robot
            case RobotCoreTypes::free_model:
            {
                // tran robot
                newMapElementState->jacobian_mapping_error_state_.jacobian_mapping_robot_error_state_.block<3,3>(0,0)=
                        jacobian_error_map_pos_wrt_error_state_robot_pos;

                // lin vel robot
                // Zeros

                // lin acc robot
                // Zeros

                // attitude robot
                newMapElementState->jacobian_mapping_error_state_.jacobian_mapping_robot_error_state_.block<3,3>(0,9)=
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
                newMapElementState->jacobian_mapping_error_state_.jacobian_mapping_robot_error_state_.block<3,3>(0,0)=
                        jacobian_error_map_pos_wrt_error_state_robot_pos;

                // lin vel robot
                // Zeros

                // lin acc robot
                // Zeros

                // attitude robot
                newMapElementState->jacobian_mapping_error_state_.jacobian_mapping_robot_error_state_.block<3,3>(0,9)=
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
                newMapElementState->jacobian_mapping_error_state_.jacobian_mapping_robot_error_state_.block<3,3>(3,9)=
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
                newMapElementState->jacobian_mapping_error_state_.jacobian_mapping_robot_error_state_.block<3,3>(3,9)=
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




    /// Sensor

    {
        newMapElementState->jacobian_mapping_error_state_.jacobian_mapping_sensor_error_state_.resize(dimension_map_new_element_error_state_total, dimension_sensor_error_state_total);
        newMapElementState->jacobian_mapping_error_state_.jacobian_mapping_sensor_error_state_.setZero();

        int dimension_map_new_element_error_state_i=0;

        // tran landmark -> Enabled by default -> TODO Check
        {
            int dimension_error_state_i=0;

            // Posi sensor wrt robot
            if(TheSensorCore->isEstimationPositionSensorWrtRobotEnabled())
            {
                newMapElementState->jacobian_mapping_error_state_.jacobian_mapping_sensor_error_state_.block<3,3>(dimension_map_new_element_error_state_i, dimension_error_state_i)=
                    jacobian_error_map_pos_wrt_error_state_sens_pos;
                dimension_error_state_i+=3;
            }

            // Atti sensor wrt robot
            if(TheSensorCore->isEstimationAttitudeSensorWrtRobotEnabled())
            {
                newMapElementState->jacobian_mapping_error_state_.jacobian_mapping_sensor_error_state_.block<3,3>(dimension_map_new_element_error_state_i, dimension_error_state_i)=
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
                newMapElementState->jacobian_mapping_error_state_.jacobian_mapping_sensor_error_state_.block<3,3>(dimension_map_new_element_error_state_i, dimension_error_state_i)=
                    jacobian_error_map_att_wrt_error_state_sens_att;
                dimension_error_state_i+=3;
            }
            dimension_map_new_element_error_state_i+=3;
        }
    }




    //// Jacobian Mapping Error-State Noise

    {
        // Resize
        newMapElementState->jacobian_mapping_error_state_noise_.resize(dimension_map_new_element_error_state_total, dimension_map_new_element_measurement);

        std::vector<Eigen::Triplet<double>> triplet_list_jacobian_mapping_error_state_wrt_error_measurement;


        // Fill
        int dimension_map_new_element_error_state_i=0;

        // tran landmark -> Enabled by default -> TODO Check
        {
            int dimension_measurement_i=0;
            // tran meas
            if(TheSensorCore->isMeasurementPositionEnabled())
            {
//                TheCodedVisualMarkerLandmarkStateCore->jacobian_mapping_error_state_noise_.block<3,3>(dimension_map_new_element_error_state_i, dimension_measurement_i)=
//                     jacobian_error_map_pos_wrt_error_meas_pos;
                BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_mapping_error_state_wrt_error_measurement, jacobian_error_map_pos_wrt_error_meas_pos, dimension_map_new_element_error_state_i, dimension_measurement_i);

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
//                TheCodedVisualMarkerLandmarkStateCore->jacobian_mapping_error_state_noise_.block<3,3>(dimension_map_new_element_error_state_i, dimension_measurement_i)=
//                    jacobian_error_map_att_wrt_error_meas_att;
                BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_mapping_error_state_wrt_error_measurement, jacobian_error_map_att_wrt_error_meas_att, dimension_map_new_element_error_state_i, dimension_measurement_i);

                dimension_measurement_i+=3;
            }

            dimension_map_new_element_error_state_i+=3;
        }


        // Set From Triplets
        newMapElementState->jacobian_mapping_error_state_noise_.setFromTriplets(triplet_list_jacobian_mapping_error_state_wrt_error_measurement.begin(), triplet_list_jacobian_mapping_error_state_wrt_error_measurement.end());
    }



    return 0;
}

int CodedVisualMarkerEyeCore::jacobiansMapMeasurementCore(// robot wrt world (state)
                                                          const Eigen::Vector3d& position_robot_wrt_world, const Eigen::Vector4d& attitude_robot_wrt_world,
                                                          // sensor wrt world (state)
                                                          const Eigen::Vector3d& position_sensor_wrt_robot, const Eigen::Vector4d& attitude_sensor_wrt_robot,
                                                          // Map element wrt world (state new)
                                                          const Eigen::Vector3d& position_map_element_wrt_world, const Eigen::Vector4d& attitude_map_element_wrt_world,
                                                          // Map element wrt sensor (measurement)
                                                          const Eigen::Vector3d& meas_position_map_element_wrt_sensor, const Eigen::Vector4d& meas_attitude_map_element_wrt_sensor,
                                                          // Jacobians
                                                          // State and Params
                                                          Eigen::Matrix3d& jacobian_error_map_pos_wrt_error_state_robot_pos, Eigen::Matrix3d& jacobian_error_map_pos_wrt_error_state_robot_att, Eigen::Matrix3d& jacobian_error_map_att_wrt_error_state_robot_att,
                                                          Eigen::Matrix3d& jacobian_error_map_pos_wrt_error_state_sens_pos, Eigen::Matrix3d& jacobian_error_map_pos_wrt_error_state_sens_att, Eigen::Matrix3d& jacobian_error_map_att_wrt_error_state_sens_att,
                                                          // Measurement
                                                          Eigen::Matrix3d& jacobian_error_map_pos_wrt_error_meas_pos, Eigen::Matrix3d& jacobian_error_map_att_wrt_error_meas_att)
{

    // Aux vars
    Eigen::Vector3d tran_visual_marker_wrt_robot=Quaternion::cross_sandwich(attitude_sensor_wrt_robot, meas_position_map_element_wrt_sensor, Quaternion::inv(attitude_sensor_wrt_robot))+position_sensor_wrt_robot;

    Eigen::Vector4d att_visual_marker_eye_wrt_world=Quaternion::cross(attitude_robot_wrt_world, attitude_sensor_wrt_robot);


    Eigen::Matrix4d mat_quat_plus_att_visual_marker_wrt_world=Quaternion::quatMatPlus(attitude_map_element_wrt_world);
    Eigen::Matrix4d mat_inv_quat_plus_att_visual_marker_wrt_world=mat_quat_plus_att_visual_marker_wrt_world.inverse();

    Eigen::Matrix4d mat_quat_minus_att_visual_marker_wrt_visual_marker_eye=Quaternion::quatMatMinus(meas_attitude_map_element_wrt_sensor);
    Eigen::Matrix4d mat_quat_plus_att_visual_marker_wrt_visual_marker_eye=Quaternion::quatMatPlus(meas_attitude_map_element_wrt_sensor);

    Eigen::Matrix4d mat_quat_minus_att_visual_marker_eye_wrt_robot=Quaternion::quatMatMinus(attitude_sensor_wrt_robot);
    Eigen::Matrix4d mat_quat_plus_att_visual_marker_eye_wrt_robot=Quaternion::quatMatPlus(attitude_sensor_wrt_robot);

    Eigen::Matrix4d mat_quat_minus_att_world_wrt_visual_marker_eye=Quaternion::quatMatMinus(Quaternion::inv(att_visual_marker_eye_wrt_world));

    Eigen::Matrix4d mat_quat_plus_att_robot_wrt_world=Quaternion::quatMatPlus(attitude_robot_wrt_world);

    Eigen::Matrix4d mat_quat_minus_att_world_wrt_robot=Quaternion::quatMatMinus(Quaternion::inv(attitude_robot_wrt_world));

    Eigen::Matrix4d mat_quat_plus_att_visual_marker_eye_wrt_world=Quaternion::quatMatPlus(att_visual_marker_eye_wrt_world);

    Eigen::Matrix4d mat_quat_plus_tran_visual_marker_wrt_robot=Quaternion::quatMatPlus(tran_visual_marker_wrt_robot);

    Eigen::Matrix4d mat_quat_plus_tran_visual_marker_wrt_visual_marker_eye=Quaternion::quatMatPlus(meas_position_map_element_wrt_sensor);

    Eigen::Matrix4d mat_quat_minus_aux1=Quaternion::quatMatMinus(Quaternion::cross_pure_gen(tran_visual_marker_wrt_robot, attitude_robot_wrt_world));
    Eigen::Matrix4d mat_quat_minus_aux2=Quaternion::quatMatMinus(Quaternion::cross_pure_gen(meas_position_map_element_wrt_sensor, att_visual_marker_eye_wrt_world));


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

int CodedVisualMarkerEyeCore::findSensorState(const std::list<std::shared_ptr<StateCore> > &list_sensors_state, CodedVisualMarkerEyeStateCore *&sensor_state)
{
    for(std::list< std::shared_ptr<StateCore> >::const_iterator it_sensor_state=list_sensors_state.begin();
        it_sensor_state!=list_sensors_state.end();
        ++it_sensor_state)
    {
        if((*it_sensor_state)->getMsfElementCoreSharedPtr() == this->getMsfElementCoreSharedPtr())
        {
            sensor_state=dynamic_cast<CodedVisualMarkerEyeStateCore*>((*it_sensor_state).get());
            return 0;
        }
    }
    return -1;
}

int CodedVisualMarkerEyeCore::findMapElementCore(const std::list<std::shared_ptr<MapElementCore> > &list_map_elements_core,
                       const CodedVisualMarkerMeasurementCore *sensor_measurement,
                       CodedVisualMarkerLandmarkCore *&map_element_core)
{
    // Match with the map element
    for(std::list<std::shared_ptr<MapElementCore>>::const_iterator itVisualMarkerLandmark=list_map_elements_core.begin();
        itVisualMarkerLandmark!=list_map_elements_core.end();
        ++itVisualMarkerLandmark)
    {
        switch(std::dynamic_pointer_cast<MapElementCore>((*itVisualMarkerLandmark)->getMsfElementCoreSharedPtr())->getMapElementType())
        {
            // Coded visual markers
            case MapElementTypes::coded_visual_marker:
            {
                // Cast
                std::shared_ptr<CodedVisualMarkerLandmarkCore> the_coded_visual_marker_landmark_core=std::dynamic_pointer_cast<CodedVisualMarkerLandmarkCore>((*itVisualMarkerLandmark)->getMsfElementCoreSharedPtr());

                // Check ids
                if(the_coded_visual_marker_landmark_core->getId() == sensor_measurement->getVisualMarkerId())
                {
                    map_element_core=dynamic_cast<CodedVisualMarkerLandmarkCore*>((*itVisualMarkerLandmark).get());
                    return 0;
                }

                // End
                break;
            }
            // Default
            case MapElementTypes::undefined:
            default:
                break;
        }
    }
    return -1;
}

int CodedVisualMarkerEyeCore::findMapElementState(const std::list<std::shared_ptr<StateCore> > &list_map_elements_state,
                                                  const CodedVisualMarkerMeasurementCore *sensor_measurement,
                                                  CodedVisualMarkerLandmarkStateCore *&map_element_state)
{
    // Match with the map element
    for(std::list<std::shared_ptr<StateCore>>::const_iterator itVisualMarkerLandmark=list_map_elements_state.begin();
        itVisualMarkerLandmark!=list_map_elements_state.end();
        ++itVisualMarkerLandmark)
    {
        switch(std::dynamic_pointer_cast<MapElementCore>((*itVisualMarkerLandmark)->getMsfElementCoreSharedPtr())->getMapElementType())
        {
            // Coded visual markers
            case MapElementTypes::coded_visual_marker:
            {
                // Cast
                std::shared_ptr<CodedVisualMarkerLandmarkCore> the_coded_visual_marker_landmark_core=std::dynamic_pointer_cast<CodedVisualMarkerLandmarkCore>((*itVisualMarkerLandmark)->getMsfElementCoreSharedPtr());

                // Check ids
                if(the_coded_visual_marker_landmark_core->getId() == sensor_measurement->getVisualMarkerId())
                {
                    map_element_state=dynamic_cast<CodedVisualMarkerLandmarkStateCore*>((*itVisualMarkerLandmark).get());
                    return 0;
                }

                // End
                break;
            }
            // Default
            case MapElementTypes::undefined:
            default:
                break;
        }
    }
    return -1;
}
