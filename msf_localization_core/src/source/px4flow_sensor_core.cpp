#include "msf_localization_core/px4flow_sensor_core.h"

// Circular Dependency
#include "msf_localization_core/msfLocalization.h"


Px4FlowSensorCore::Px4FlowSensorCore() :
    SensorCore()
{
    init();

    return;
}

Px4FlowSensorCore::Px4FlowSensorCore(MsfLocalizationCore *msf_localization_core_ptr) :
    SensorCore(msf_localization_core_ptr)
{

    init();

    return;
}

Px4FlowSensorCore::~Px4FlowSensorCore()
{
    return;
}

int Px4FlowSensorCore::init()
{
    // Sensor Type
    setSensorType(SensorTypes::px4flow);


    // Flags measurement
    flag_measurement_velocity_=false;
    flag_measurement_ground_distance_=false;

    // Flags estimation -> By default are considered parameters

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
    noise_measurement_velocity_.setZero();
    noise_measurement_ground_distance_=0;


    // Noises parameters
    // Nothing

    // Noises estimation
    // Nothing


    // Fixed parameters
    // Sensitivity of the measurement of velocity
    {
        sensitivity_meas_lin_vel_.resize(2, 3);
        std::vector<Eigen::Triplet<double> > triplet_list;

        triplet_list.push_back(Eigen::Triplet<double>(0, 0, 1.0));
        triplet_list.push_back(Eigen::Triplet<double>(1, 1, 1.0));

        sensitivity_meas_lin_vel_.setFromTriplets(triplet_list.begin(), triplet_list.end());
    }


    return 0;
}

int Px4FlowSensorCore::readConfig(const pugi::xml_node& sensor, const unsigned int sensor_id, std::shared_ptr<Px4FlowSensorStateCore>& sensor_init_state)
{

    // Create a class for the SensorStateCore
    if(!sensor_init_state)
        sensor_init_state=std::make_shared<Px4FlowSensorStateCore>(this->getMsfElementCoreWeakPtr());


    // Set Id
    this->setSensorId(sensor_id);


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





    //// Measurements
    pugi::xml_node measurements = sensor.child("measurements");


    /// Velocity
    pugi::xml_node meas_velocity = measurements.child("velocity");

    readingValue=meas_velocity.child_value("enabled");
    if(std::stoi(readingValue))
        this->enableMeasurementVelocity();

    readingValue=meas_velocity.child_value("var");
    if(!readingValue.empty())
    {
        std::istringstream stm(readingValue);
        Eigen::Vector2d variance;
        stm>>variance[0]>>variance[1];
        this->setNoiseMeasurementVelocity(variance.asDiagonal());
    }

    /// Ground Distance
    pugi::xml_node meas_ground_distance = measurements.child("ground_distance");

    readingValue=meas_ground_distance.child_value("enabled");
    if(std::stoi(readingValue))
        this->enableMeasurementGroundDistance();

    readingValue=meas_ground_distance.child_value("var");
    if(!readingValue.empty())
    {
        std::istringstream stm(readingValue);
        double variance;
        stm>>variance;
        this->setNoiseMeasurementGroundDistance(variance);
    }



    //// Init State

    /// Pose of the sensor wrt robot

    // Position of the sensor wrt robot
    readingValue=pose_in_robot.child("position").child_value("init_estimation");
    if(!readingValue.empty())
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d init_estimation;
        stm>>init_estimation[0]>>init_estimation[1]>>init_estimation[2];
        sensor_init_state->setPositionSensorWrtRobot(init_estimation);
    }

    // Attitude of the sensor wrt robot
    readingValue=pose_in_robot.child("attitude").child_value("init_estimation");
    if(!readingValue.empty())
    {
        std::istringstream stm(readingValue);
        Eigen::Vector4d init_estimation;
        stm>>init_estimation[0]>>init_estimation[1]>>init_estimation[2]>>init_estimation[3];
        sensor_init_state->setAttitudeSensorWrtRobot(init_estimation);
    }


    /// Parameters





    //// Init Variances

    /// Pose of the sensor wrt robot

    // Position of the sensor wrt robot
    readingValue=pose_in_robot.child("position").child_value("init_var");
    if(!readingValue.empty())
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoisePositionSensorWrtRobot(variance.asDiagonal());
    }

    // Attitude of the sensor wrt robot
    readingValue=pose_in_robot.child("attitude").child_value("init_var");
    if(!readingValue.empty())
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoiseAttitudeSensorWrtRobot(variance.asDiagonal());
    }


    /// Other Parameters






    // Noises in the estimation (if enabled)




    // Prepare covariance matrix
    this->prepareCovarianceInitErrorState();


    /// Finish

    // End
    return 0;
}

bool Px4FlowSensorCore::isMeasurementVelocityEnabled() const
{
    return this->flag_measurement_velocity_;
}

void Px4FlowSensorCore::enableMeasurementVelocity()
{
    if(this->flag_measurement_velocity_==false)
    {
        this->flag_measurement_velocity_=true;
        dimension_error_measurement_+=2;
        dimension_measurement_+=2;
    }
    return;
}

Eigen::Matrix2d Px4FlowSensorCore::getNoiseMeasurementVelocity() const
{
    return this->noise_measurement_velocity_;
}

void Px4FlowSensorCore::setNoiseMeasurementVelocity(const Eigen::Matrix2d& noise_measurement_velocity)
{
    this->noise_measurement_velocity_=noise_measurement_velocity;
    return;
}

bool Px4FlowSensorCore::isMeasurementGroundDistanceEnabled() const
{
    return this->flag_measurement_ground_distance_;
}

void Px4FlowSensorCore::enableMeasurementGroundDistance()
{
    if(this->flag_measurement_ground_distance_==false)
    {
        flag_measurement_ground_distance_=true;
        dimension_error_measurement_+=1;
        dimension_measurement_+=1;
    }
    return;
}

double Px4FlowSensorCore::getNoiseMeasurementGroundDistance() const
{
    return this->noise_measurement_ground_distance_;
}

void Px4FlowSensorCore::setNoiseMeasurementGroundDistance(double noise_measurement_ground_distance)
{
    this->noise_measurement_ground_distance_=noise_measurement_ground_distance;
    return;
}

Eigen::SparseMatrix<double> Px4FlowSensorCore::getCovarianceParameters()
{
    Eigen::SparseMatrix<double> covariances_matrix;

    covariances_matrix.resize(this->getDimensionErrorParameters(), this->getDimensionErrorParameters());

    std::vector<Eigen::Triplet<double> > triplets_covariance;


    unsigned int dimension=0;
    if(!this->isEstimationPositionSensorWrtRobotEnabled())
    {
        for(int i=0; i<3; i++)
            triplets_covariance.push_back(Eigen::Triplet<double>(dimension+i,dimension+i,noisePositionSensorWrtRobot(i,i)));

        dimension+=3;
    }
    if(!this->isEstimationAttitudeSensorWrtRobotEnabled())
    {
        for(int i=0; i<3; i++)
            triplets_covariance.push_back(Eigen::Triplet<double>(dimension+i,dimension+i,noiseAttitudeSensorWrtRobot(i,i)));

        dimension+=3;
    }


    // Set
    covariances_matrix.setFromTriplets(triplets_covariance.begin(), triplets_covariance.end());

    // End
    return covariances_matrix;
}

int Px4FlowSensorCore::prepareCovarianceInitErrorStateSpecific()
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


    return 0;
}


Eigen::SparseMatrix<double> Px4FlowSensorCore::getCovarianceNoise(const TimeStamp deltaTimeStamp)
{
    Eigen::SparseMatrix<double> covariances_matrix;

    // Dimension noise
    int dimension_noise=getDimensionNoise();


    // Resize
    covariances_matrix.resize(dimension_noise, dimension_noise);
    //covariance_noise.setZero();
    covariances_matrix.reserve(dimension_noise);

    std::vector<Eigen::Triplet<double> > triplets_covariance;

    // dt
    double dt=deltaTimeStamp.getDouble();

    // Fill
    int dimension_noise_i=0;

    // Nothing


    // Set
    covariances_matrix.setFromTriplets(triplets_covariance.begin(), triplets_covariance.end());


    // End
    return covariances_matrix;
}

int Px4FlowSensorCore::predictState(//Time
                                     const TimeStamp& previousTimeStamp, const TimeStamp& currentTimeStamp,
                                     // Previous State
                                     const std::shared_ptr<StateComponent>& pastState,
                                     // Inputs
                                     const std::shared_ptr<InputCommandComponent>& inputCommand,
                                     // Predicted State
                                     std::shared_ptr<StateCore> &predictedState)
{
    // Checks

    // Past State
    if(!pastState)
        return -1;

    // TODO

    // Search for the past sensor State Core
    Px4FlowSensorStateCore* past_sensor_state(nullptr);

    for(std::list< std::shared_ptr<StateCore> >::iterator it_sensor_state=pastState->TheListSensorStateCore.begin();
        it_sensor_state!=pastState->TheListSensorStateCore.end();
        ++it_sensor_state)
    {
        if((*it_sensor_state)->getMsfElementCoreSharedPtr() == this->getMsfElementCoreSharedPtr())
        {
            past_sensor_state=dynamic_cast<Px4FlowSensorStateCore*>((*it_sensor_state).get());
            break;
        }
    }
    if(!past_sensor_state)
        return -10;


    // Predicted State
    Px4FlowSensorStateCore* predicted_sensor_state(nullptr);
    // Create the prediction if it does not exist
    if(!predicted_sensor_state)
    {
        predicted_sensor_state=new Px4FlowSensorStateCore;
        predicted_sensor_state->setMsfElementCorePtr(past_sensor_state->getMsfElementCoreWeakPtr());
        predictedState=std::shared_ptr<StateCore>(predicted_sensor_state);
    }
    else
        predicted_sensor_state=dynamic_cast<Px4FlowSensorStateCore*>(predictedState.get());



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



int Px4FlowSensorCore::predictStateSpecific(const TimeStamp &previousTimeStamp, const TimeStamp &currentTimeStamp,
                                            const Px4FlowSensorStateCore *pastState,
                                            Px4FlowSensorStateCore *&predictedState)
{

    // Checks in the past state
    if(!pastState->isCorrect())
    {
        std::cout<<"Px4FlowSensorCore::predictState() error !pastState->isCorrect()"<<std::endl;
        return -5;
    }


    // Create the predicted state if it doesn't exists
    if(!predictedState)
    {
        predictedState=new Px4FlowSensorStateCore;
        predictedState->setMsfElementCorePtr(pastState->getMsfElementCoreWeakPtr());
    }



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




    // End
    return 0;
}

int Px4FlowSensorCore::predictErrorStateJacobian(//Time
                                                 const TimeStamp& previousTimeStamp, const TimeStamp& currentTimeStamp,
                                                 // Previous State
                                                 const std::shared_ptr<StateComponent>& past_state,
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
        return -10;

    // TODO


    // Search for the past sensor State Core
    Px4FlowSensorStateCore* past_sensor_state(nullptr);

    for(std::list< std::shared_ptr<StateCore> >::iterator it_sensor_state=past_state->TheListSensorStateCore.begin();
        it_sensor_state!=past_state->TheListSensorStateCore.end();
        ++it_sensor_state)
    {
        if((*it_sensor_state)->getMsfElementCoreSharedPtr() == this->getMsfElementCoreSharedPtr())
        {
            past_sensor_state=dynamic_cast<Px4FlowSensorStateCore*>((*it_sensor_state).get());
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
    Px4FlowSensorStateCore* predicted_sensor_state=dynamic_cast<Px4FlowSensorStateCore*>(predicted_state.get());


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
        if( dynamic_cast<Px4FlowSensorStateCore*>((*itSensorStateCore).get()) == past_sensor_state )
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



    // End
    return 0;
}

int Px4FlowSensorCore::predictErrorStateJacobiansSpecific(const TimeStamp &previousTimeStamp, const TimeStamp &currentTimeStamp,
                                                      const Px4FlowSensorStateCore *pastState,
                                                      const Px4FlowSensorStateCore *predictedState,
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


        // Nothing to do


        // Set From Triplets
        jacobian_error_state_wrt_noise.setFromTriplets(tripletJacobianErrorStateNoise.begin(), tripletJacobianErrorStateNoise.end());

    }


    // End
    return 0;
}

int Px4FlowSensorCore::predictErrorStateJacobiansCore(// State k: Sensor
                                                       const Eigen::Vector3d& position_sensor_wrt_robot, const Eigen::Vector4d& attitude_sensor_wrt_robot,
                                                       // State k+1: Sensor
                                                       const Eigen::Vector3d& pred_position_sensor_wrt_robot, const Eigen::Vector4d& pred_attitude_sensor_wrt_robot,
                                                       // Jacobian: State Fx & Fp
                                                       Eigen::Matrix3d& jacobian_error_sens_pos_wrt_error_state_sens_pos,  Eigen::Matrix3d& jacobian_error_sens_att_wrt_error_state_sens_att
                                                       )
{
    // posi sensor / posi sensor
    jacobian_error_sens_pos_wrt_error_state_sens_pos=Eigen::Matrix3d::Identity();

    // att sensor / att sensor
    // TODO FIX
    jacobian_error_sens_att_wrt_error_state_sens_att=Eigen::Matrix3d::Identity();


    // End
    return 0;
}

int Px4FlowSensorCore::predictMeasurement(// Time
                                       const TimeStamp& current_time_stamp,
                                       // Current State
                                       const std::shared_ptr<StateComponent>& current_state,
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

    // Nothing else needed



    // Predicted Measurement
    Px4FlowSensorMeasurementCore* predicted_sensor_measurement(nullptr);
    if(!predicted_measurement)
    {
        predicted_sensor_measurement=new Px4FlowSensorMeasurementCore;
        predicted_sensor_measurement->setSensorCorePtr(std::dynamic_pointer_cast<SensorCore>(this->getMsfElementCoreSharedPtr()));
        predicted_measurement=std::shared_ptr<Px4FlowSensorMeasurementCore>(predicted_sensor_measurement);
    }
    else
    {
        if(measurement->getSensorCoreSharedPtr() != predicted_measurement->getSensorCoreSharedPtr())
            return -3;
        predicted_sensor_measurement=dynamic_cast<Px4FlowSensorMeasurementCore*>(predicted_measurement.get());
    }



    // Search for the current sensor State Core
    Px4FlowSensorStateCore* current_sensor_state(nullptr);

    for(std::list< std::shared_ptr<StateCore> >::iterator it_sensor_state=current_state->TheListSensorStateCore.begin();
        it_sensor_state!=current_state->TheListSensorStateCore.end();
        ++it_sensor_state)
    {
        if((*it_sensor_state)->getMsfElementCoreSharedPtr() == this->getMsfElementCoreSharedPtr())
        {
            current_sensor_state=dynamic_cast<Px4FlowSensorStateCore*>((*it_sensor_state).get());
            break;
        }
    }
    if(!current_sensor_state)
        return -10;


    // Cast the measurement
    Px4FlowSensorMeasurementCore* sensor_measurement=dynamic_cast<Px4FlowSensorMeasurementCore*>(measurement.get());


    // Predict Measurement
    int error_predict_measurement=predictMeasurementSpecific(current_time_stamp,
                                                             dynamic_cast<RobotStateCore*>(current_state->TheRobotStateCore.get()),
                                                             current_sensor_state,
                                                             sensor_measurement,
                                                             predicted_sensor_measurement);

    // Check error
    if(error_predict_measurement)
        return error_predict_measurement;



    // End
    return 0;
}

int Px4FlowSensorCore::predictMeasurementSpecific(const TimeStamp &time_stamp,
                                                  const RobotStateCore *current_robot_state,
                                                  const Px4FlowSensorStateCore *current_sensor_state,
                                                  const Px4FlowSensorMeasurementCore* sensor_measurement,
                                                  Px4FlowSensorMeasurementCore *&predicted_measurement)
{


    // Check
    if(!isCorrect())
    {
        std::cout<<"Px4FlowSensorCore::predictMeasurementSpecific() error 50"<<std::endl;
        return -50;
    }

    // Check sensor state
    if(!current_sensor_state)
    {
        std::cout<<"Px4FlowSensorCore::predictMeasurementSpecific() error 1"<<std::endl;
        return -1;
    }

    // Robot check
    if(!current_robot_state)
    {
        std::cout<<"Px4FlowSensorCore::predictMeasurementSpecific() error 2"<<std::endl;
        return -2;
    }

    // Robot core check
    if(!current_robot_state->isCorrect())
    {
        std::cout<<"Px4FlowSensorCore::predictMeasurementSpecific() error 3"<<std::endl;
        return -3;
    }

    // Sensor Measurement
    // Not needed

    // Checks
    // TODO


    // Create pointer
    // TODO check if it must be done here
    if(!predicted_measurement)
    {
        predicted_measurement=new Px4FlowSensorMeasurementCore;
        predicted_measurement->setSensorCorePtr(std::dynamic_pointer_cast<SensorCore>(this->getMsfElementCoreSharedPtr()));
    }




    // Variables
    // State: Robot
    Eigen::Vector3d position_robot_wrt_world;
    Eigen::Vector4d attitude_robot_wrt_world;
    Eigen::Vector3d lin_speed_robot_wrt_world;
    Eigen::Vector3d ang_velocity_robot_wrt_world;
    // Eigen::Vector3d lin_accel_robot_wrt_world; // Not needed
    // Eigen::Vector3d ang_accel_robot_wrt_world; // Not needed
    // State: Sensor
    Eigen::Vector3d position_sensor_wrt_robot;
    Eigen::Vector4d attitude_sensor_wrt_robot;
    // Parameters: Sensor
    // Nothing
    // Measurement. Not needed
    // Eigen::Vector2d meas_lin_velocity_sensor_wrt_sensor;
    // double meas_ground_distance;
    // Predicted Measurement
    Eigen::Vector2d pred_meas_lin_velocity_sensor_wrt_sensor;
    double pred_meas_ground_distance;


    /// Fill Variables



    // State: Robot
    position_robot_wrt_world=current_robot_state->getPositionRobotWrtWorld();
    attitude_robot_wrt_world=current_robot_state->getAttitudeRobotWrtWorld();
    lin_speed_robot_wrt_world=current_robot_state->getLinearSpeedRobotWrtWorld();
    ang_velocity_robot_wrt_world=current_robot_state->getAngularVelocityRobotWrtWorld();
    //lin_accel_robot_wrt_world=currentRobotState->getLinearAccelerationRobotWrtWorld(); // Not needed
    //ang_accel_robot_wrt_world=currentRobotState->getAngularAccelerationRobotWrtWorld(); // Not needed

    // State: Sensor
    position_sensor_wrt_robot=current_sensor_state->getPositionSensorWrtRobot();
    attitude_sensor_wrt_robot=current_sensor_state->getAttitudeSensorWrtRobot();

    // Parameters: Sensor
    // Nothing


    // Measurement. Not Needed
    // meas_lin_velocity_sensor_wrt_sensor;
    // meas_ground_distance;


    // Predicted Measurement. Will be filled before
    // pred_meas_lin_velocity_sensor_wrt_sensor;
    // pred_meas_ground_distance;



    /// Measurements Prediction


    int error_measurement_prediction_core=predictMeasurementCore(// State
                                                                 // Robot
                                                                 position_robot_wrt_world,
                                                                 lin_speed_robot_wrt_world,
                                                                 attitude_robot_wrt_world,
                                                                 ang_velocity_robot_wrt_world,
                                                                 // Sensor
                                                                 position_sensor_wrt_robot,
                                                                 attitude_sensor_wrt_robot,
                                                                 // Predicted measurements
                                                                 sensor_measurement->isVelocitySet(), pred_meas_lin_velocity_sensor_wrt_sensor,
                                                                 sensor_measurement->isGroundDistanceSet(), pred_meas_ground_distance);

    // Check error
    if(error_measurement_prediction_core)
        return error_measurement_prediction_core;


    // Set Measurement Prediction

    // Velocity
    if(sensor_measurement->isVelocitySet())
    {
        predicted_measurement->setVelocity(pred_meas_lin_velocity_sensor_wrt_sensor);
    }

    // Ground Distance
    if(sensor_measurement->isGroundDistanceSet())
    {
        predicted_measurement->setGroundDistance(pred_meas_ground_distance);
    }




    // End
    return 0;
}

int Px4FlowSensorCore::predictMeasurementCore(// State
                                              // Robot
                                              const Eigen::Vector3d& position_robot_wrt_world,
                                              const Eigen::Vector3d& lin_speed_robot_wrt_world,
                                              const Eigen::Vector4d& attitude_robot_wrt_world,
                                              const Eigen::Vector3d& ang_velocity_robot_wrt_world,
                                              // Sensor
                                              const Eigen::Vector3d &position_sensor_wrt_robot,
                                              const Eigen::Vector4d &attitude_sensor_wrt_robot,
                                              // Predicted measurements
                                              bool flag_pred_meas_lin_velocity_sensor_wrt_sensor, Eigen::Vector2d& pred_meas_lin_velocity_sensor_wrt_sensor,
                                              bool flag_pred_meas_ground_distance, double& pred_meas_ground_distance)
{


    // Linear Velocity
    if(flag_pred_meas_lin_velocity_sensor_wrt_sensor)
    {
        // Vars
        Eigen::Vector4d quat_att_sensor_wrt_world=Quaternion::cross(attitude_robot_wrt_world, attitude_sensor_wrt_robot);

        Eigen::Vector3d ang_vel_robot_wrt_world_in_robot=Quaternion::cross_sandwich(Quaternion::inv(attitude_robot_wrt_world), ang_velocity_robot_wrt_world, attitude_robot_wrt_world);

        Eigen::Vector3d lin_vel_sensor_wrt_world_in_world=lin_speed_robot_wrt_world + Quaternion::cross_sandwich(attitude_robot_wrt_world, ang_vel_robot_wrt_world_in_robot.cross(position_sensor_wrt_robot), Quaternion::inv(attitude_robot_wrt_world));

        Eigen::Vector3d lin_vel_sensor_wrt_world_in_sensor=Quaternion::cross_sandwich(Quaternion::inv(quat_att_sensor_wrt_world), lin_vel_sensor_wrt_world_in_world, quat_att_sensor_wrt_world);

        // Pred measurement
        pred_meas_lin_velocity_sensor_wrt_sensor=lin_vel_sensor_wrt_world_in_sensor.block<2,1>(0,0);
    }


    // Ground Distance
    if(flag_pred_meas_ground_distance)
    {
        // TODO FIX
        // Provisional variables for ground plane:
        Eigen::Vector3d ground_plane_normal(0,0,1);
        Eigen::Vector3d ground_plane_point(0,0,0);

        // Distance sensor line
        Eigen::Vector4d att_sensor_wrt_world=Quaternion::cross(attitude_robot_wrt_world, attitude_sensor_wrt_robot);
        Eigen::Vector3d distance_sensor_tvect=Quaternion::cross_sandwich(att_sensor_wrt_world, Eigen::Vector3d(0,0,1), Quaternion::inv(att_sensor_wrt_world));

        Eigen::Vector3d position_sensor_wrt_world=Quaternion::cross_sandwich(attitude_robot_wrt_world, position_sensor_wrt_robot, Quaternion::inv(attitude_robot_wrt_world))+position_robot_wrt_world;

        // Pred measurement
        double lambda_num=ground_plane_normal.dot(ground_plane_point-position_sensor_wrt_world);
        double lambda_den=ground_plane_normal.dot(distance_sensor_tvect);
        double lamdba=lambda_num/lambda_den;
        pred_meas_ground_distance=std::abs(lamdba);

    }



    // End
    return 0;
}

int Px4FlowSensorCore::predictErrorMeasurementJacobian(// Time
                                                const TimeStamp& current_time_stamp,
                                                // Current State
                                                const std::shared_ptr<StateComponent>& current_state,
                                                // Measurements
                                                const std::shared_ptr<SensorMeasurementCore>& measurement,
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
    Px4FlowSensorStateCore* current_sensor_state(nullptr);

    for(std::list< std::shared_ptr<StateCore> >::iterator it_sensor_state=current_state->TheListSensorStateCore.begin();
        it_sensor_state!=current_state->TheListSensorStateCore.end();
        ++it_sensor_state)
    {
        if((*it_sensor_state)->getMsfElementCoreSharedPtr() == this->getMsfElementCoreSharedPtr())
        {
            current_sensor_state=dynamic_cast<Px4FlowSensorStateCore*>((*it_sensor_state).get());
            break;
        }
    }
    if(!current_sensor_state)
        return -10;


    // Cast the measurement
    Px4FlowSensorMeasurementCore* sensor_measurement=dynamic_cast<Px4FlowSensorMeasurementCore*>(measurement.get());


    //// Init Jacobians
    int error_init_jacobians=predictErrorMeasurementJacobianInit(// Current State
                                                                current_state,
                                                                 // Sensor Measurement
                                                                 measurement,
                                                                // Predicted Measurements
                                                                predicted_measurement);

    if(error_init_jacobians)
        return error_init_jacobians;


    // Predicted Measurement Cast
    Px4FlowSensorMeasurementCore* predicted_sensor_measurement(nullptr);
    if(measurement->getSensorCoreSharedPtr() != predicted_measurement->getSensorCoreSharedPtr())
        return -3;
    predicted_sensor_measurement=dynamic_cast<Px4FlowSensorMeasurementCore*>(predicted_measurement.get());



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
        if( dynamic_cast<Px4FlowSensorStateCore*>((*itSensorStateCore).get()) == current_sensor_state )
            break;
    }


    /// Predict Error Measurement Jacobians
    int error_predict_measurement=predictErrorMeasurementJacobianSpecific(current_time_stamp,
                                                                          dynamic_cast<RobotStateCore*>(current_state->TheRobotStateCore.get()),
                                                                          current_sensor_state,
                                                                          sensor_measurement,
                                                                          predicted_sensor_measurement,
                                                                          // Jacobians State / Parameters
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



    // End
    return 0;
}


int Px4FlowSensorCore::predictErrorMeasurementJacobianSpecific(const TimeStamp& time_stamp,
                                                               const RobotStateCore* robot_state,
                                                               const Px4FlowSensorStateCore* sensor_state,
                                                               const Px4FlowSensorMeasurementCore* sensor_measurement,
                                                               Px4FlowSensorMeasurementCore*& predicted_measurement,
                                                               // Jacobians State / Parameters
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




    /// Common Variables

    // Imu sensor core
    std::shared_ptr<const Px4FlowSensorCore> sensor_core=std::dynamic_pointer_cast<const Px4FlowSensorCore>(sensor_state->getMsfElementCoreSharedPtr());



    /// Variables

    // State: Robot
    Eigen::Vector3d position_robot_wrt_world;
    Eigen::Vector4d attitude_robot_wrt_world;
    Eigen::Vector3d lin_speed_robot_wrt_world;
    Eigen::Vector3d ang_velocity_robot_wrt_world;
    //Eigen::Vector3d lin_accel_robot_wrt_world;
    //Eigen::Vector3d ang_accel_robot_wrt_world;
    // State: Sensor
    Eigen::Vector3d position_sensor_wrt_robot;
    Eigen::Vector4d attitude_sensor_wrt_robot;
    // Parameters: Sensor
    // None
    // Measurement
    Eigen::Vector2d meas_lin_velocity_sensor_wrt_sensor;
    double meas_ground_distance;
    // Predicted Measurement
    Eigen::Vector2d pred_meas_lin_velocity_sensor_wrt_sensor;
    double pred_meas_ground_distance;



    /// Fill Variables



    // State: Robot
    position_robot_wrt_world=robot_state->getPositionRobotWrtWorld();
    attitude_robot_wrt_world=robot_state->getAttitudeRobotWrtWorld();
    lin_speed_robot_wrt_world=robot_state->getLinearSpeedRobotWrtWorld();
    ang_velocity_robot_wrt_world=robot_state->getAngularVelocityRobotWrtWorld();
    //lin_accel_robot_wrt_world=robot_state->getLinearAccelerationRobotWrtWorld();
    //ang_accel_robot_wrt_world=robot_state->getAngularAccelerationRobotWrtWorld();

    // State: Sensor
    position_sensor_wrt_robot=sensor_state->getPositionSensorWrtRobot();
    attitude_sensor_wrt_robot=sensor_state->getAttitudeSensorWrtRobot();

    // Parameters: Sensor
    // None


    // Measurement
    meas_lin_velocity_sensor_wrt_sensor=sensor_measurement->getVelocity();
    meas_ground_distance=sensor_measurement->getGroundDistance();


    // Predicted Measurement
    pred_meas_lin_velocity_sensor_wrt_sensor=predicted_measurement->getVelocity();
    pred_meas_ground_distance=predicted_measurement->getGroundDistance();





    /// Jacobians Variables

    // Jacobian State

    // Robot
    // meas_lin_vel
    Eigen::Matrix<double, 2, 3> jacobian_error_meas_lin_vel_wrt_error_state_robot_lin_vel;
    Eigen::Matrix<double, 2, 3> jacobian_error_meas_lin_vel_wrt_error_state_robot_att;
    Eigen::Matrix<double, 2, 3> jacobian_error_meas_lin_vel_wrt_error_state_robot_ang_vel;

    // meas_ground_distance
    Eigen::Matrix<double, 1, 3> jacobian_error_meas_ground_distance_wrt_error_state_robot_pos;
    Eigen::Matrix<double, 1, 3> jacobian_error_meas_ground_distance_wrt_error_state_robot_att;


    // Sensor
    // meas_lin_vel
    Eigen::Matrix<double, 2, 3> jacobian_error_meas_lin_vel_wrt_error_state_sensor_pos;
    Eigen::Matrix<double, 2, 3> jacobian_error_meas_lin_vel_wrt_error_state_sensor_att;

    // meas_ground_distance
    Eigen::Matrix<double, 1, 3> jacobian_error_meas_ground_distance_wrt_error_state_sensor_pos;
    Eigen::Matrix<double, 1, 3> jacobian_error_meas_ground_distance_wrt_error_state_sensor_att;


    // Map element
    // TODO: meas_ground_distance


    // Jacobians: Noise
    // meas_lin_vel
    Eigen::Matrix2d jacobian_error_meas_lin_vel_wrt_error_meas_lin_vel;
    // meas_ground_distance
    double jacobian_error_meas_ground_distance_wrt_error_meas_ground_distance;




    /// Call core

    int error=predictErrorMeasurementJacobianCore(// State: Robot
                                                  position_robot_wrt_world, attitude_robot_wrt_world,
                                                  lin_speed_robot_wrt_world, ang_velocity_robot_wrt_world,
                                                  // State: Sensor
                                                  position_sensor_wrt_robot, attitude_sensor_wrt_robot,
                                                  // Parameters: Sensor
                                                  // None
                                                  // Measurement
                                                  meas_lin_velocity_sensor_wrt_sensor, meas_ground_distance,
                                                  // Predicted Measurement
                                                  pred_meas_lin_velocity_sensor_wrt_sensor, pred_meas_ground_distance,
                                                  // FLags
                                                  sensor_measurement->isVelocitySet(), sensor_measurement->isGroundDistanceSet(),
                                                  // Jacobians: State and Params
                                                  jacobian_error_meas_lin_vel_wrt_error_state_robot_lin_vel, jacobian_error_meas_lin_vel_wrt_error_state_robot_att, jacobian_error_meas_lin_vel_wrt_error_state_robot_ang_vel,
                                                  jacobian_error_meas_ground_distance_wrt_error_state_robot_pos, jacobian_error_meas_ground_distance_wrt_error_state_robot_att,
                                                  jacobian_error_meas_lin_vel_wrt_error_state_sensor_pos, jacobian_error_meas_lin_vel_wrt_error_state_sensor_att,
                                                  jacobian_error_meas_ground_distance_wrt_error_state_sensor_pos, jacobian_error_meas_ground_distance_wrt_error_state_sensor_att,
                                                  // TODO: Jacobians ground_distance wrt map
                                                  // Jacobians: Noise
                                                  jacobian_error_meas_lin_vel_wrt_error_meas_lin_vel, jacobian_error_meas_ground_distance_wrt_error_meas_ground_distance
                                                  );

    if(error)
        return error;


    /// Dimensions


    // dimension of the measurement
    int dimension_error_measurement=sensor_measurement->getDimensionErrorMeasurement();

    // dimension of robot
    int dimension_robot_error_state=robot_state->getMsfElementCoreSharedPtr()->getDimensionErrorState();
    int dimension_robot_error_parameters=robot_state->getMsfElementCoreSharedPtr()->getDimensionErrorParameters();

    // dimension of the sensor
    int dimension_sensor_error_state=sensor_state->getMsfElementCoreSharedPtr()->getDimensionErrorState();
    int dimension_sensor_error_parameters=sensor_state->getMsfElementCoreSharedPtr()->getDimensionErrorParameters();




    ///// Jacobians error measurement - error state / error parameters


    /// Jacobians error measurement - world error state / error parameters

    {
        // Resize and init
        // Nothing to do

        // Fill
        // No dependency -> Everything is set to zero
    }


    /// Jacobian error measurement - robot error state / error parameters

    {

        // Resize and init
        jacobian_error_measurement_wrt_robot_error_state.resize(dimension_error_measurement, dimension_robot_error_state);
        jacobian_error_measurement_wrt_robot_error_parameters.resize(dimension_error_measurement, dimension_robot_error_parameters);

        std::vector<Eigen::Triplet<double>> triplet_list_jacobian_error_measurement_wrt_error_state;
        std::vector<Eigen::Triplet<double>> triplet_list_jacobian_error_measurement_wrt_error_parameters;



        // Fill Jacobian
        unsigned int dimension_error_measurement_i=0;

        // z_lin_vel
        if(sensor_measurement->isVelocitySet())
        {
            // Switch depending on robot used
            switch(std::dynamic_pointer_cast<RobotCore>(robot_state->getMsfElementCoreSharedPtr())->getRobotCoreType())
            {
                case RobotCoreTypes::free_model:
                {
                    // z_lin_acc / posi
                    // Zeros

                    // z_lin_acc / lin_speed
                    BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_state, jacobian_error_meas_lin_vel_wrt_error_state_robot_lin_vel, dimension_error_measurement_i, 3);

                    // z_lin_acc / lin_acc
                    // Zeros

                    // z_lin_acc / attit
                    BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_state, jacobian_error_meas_lin_vel_wrt_error_state_robot_att, dimension_error_measurement_i, 9);

                    // z_lin_acc / ang_vel
                    BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_state, jacobian_error_meas_lin_vel_wrt_error_state_robot_ang_vel, dimension_error_measurement_i, 12);

                    // z_lin_acc / ang_acc
                    // Zeros


                    // end
                    break;
                }
                default:
                {
                    return -2;
                }

            }

            dimension_error_measurement_i+=2;
        }


        // z_ground_distance
        if(sensor_measurement->isGroundDistanceSet())
        {
            // Switch depending on robot used
            switch(std::dynamic_pointer_cast<RobotCore>(robot_state->getMsfElementCoreSharedPtr())->getRobotCoreType())
            {
                case RobotCoreTypes::free_model:
                {
                    // z_ang_vel / posi
                    BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_state, jacobian_error_meas_ground_distance_wrt_error_state_robot_pos, dimension_error_measurement_i, 0);

                    // z_ang_vel / lin_speed
                    // Zeros

                    // z_ang_vel / lin_acc
                    // Zeros

                    // z_ang_vel / attit
                    BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_state, jacobian_error_meas_ground_distance_wrt_error_state_robot_att, dimension_error_measurement_i, 9);

                    // z_ang_vel / ang_vel
                    // Zeros

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

            dimension_error_measurement_i+=1;
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
        // No dependency -> Everything is set to zero
    }


    /// Jacobian error measurement - sensor error / error parameters

    {
        // Resize and init
        jacobian_error_measurement_wrt_sensor_error_state.resize(dimension_error_measurement, dimension_sensor_error_state);
        jacobian_error_measurement_wrt_sensor_error_parameters.resize(dimension_error_measurement, dimension_sensor_error_parameters);


        std::vector<Eigen::Triplet<double>> triplet_list_jacobian_error_measurement_wrt_error_state;
        std::vector<Eigen::Triplet<double>> triplet_list_jacobian_error_measurement_wrt_error_parameters;


        // Fill Jacobian
        unsigned int dimension_error_measurement_i=0;

        // z_lin_vel
        if(sensor_measurement->isVelocitySet())
        {
            int dimension_sensor_error_state_i=0;
            int dimension_sensor_error_parameters_i=0;


            // z_lin_vel / posi_sen_wrt_robot
            if(sensor_core->isEstimationPositionSensorWrtRobotEnabled())
            {
                BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_state, jacobian_error_meas_lin_vel_wrt_error_state_sensor_pos, dimension_error_measurement_i, dimension_sensor_error_state_i);

                dimension_sensor_error_state_i+=3;
            }
            else
            {
                BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_parameters, jacobian_error_meas_lin_vel_wrt_error_state_sensor_pos, dimension_error_measurement_i, dimension_sensor_error_parameters_i);

                dimension_sensor_error_parameters_i+=3;
            }


            // z_lin_vel / atti_sen_wrt_robot
            if(sensor_core->isEstimationAttitudeSensorWrtRobotEnabled())
            {
                BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_state, jacobian_error_meas_lin_vel_wrt_error_state_sensor_att, dimension_error_measurement_i, dimension_sensor_error_state_i);

                dimension_sensor_error_state_i+=3;
            }
            else
            {
                BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_parameters, jacobian_error_meas_lin_vel_wrt_error_state_sensor_att, dimension_error_measurement_i, dimension_sensor_error_parameters_i);

                dimension_sensor_error_parameters_i+=3;
            }

            dimension_error_measurement_i+=2;
        }


        // z_ground_distance
        if(sensor_measurement->isGroundDistanceSet())
        {
            int dimension_sensor_error_state_i=0;
            int dimension_sensor_error_parameters_i=0;


            // z_ground_distance / posi_sen_wrt_robot
            if(sensor_core->isEstimationPositionSensorWrtRobotEnabled())
            {
                BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_state, jacobian_error_meas_ground_distance_wrt_error_state_sensor_pos, dimension_error_measurement_i, dimension_sensor_error_state_i);

                dimension_sensor_error_state_i+=3;
            }
            else
            {
                BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_parameters, jacobian_error_meas_ground_distance_wrt_error_state_sensor_pos, dimension_error_measurement_i, dimension_sensor_error_parameters_i);

                dimension_sensor_error_parameters_i+=3;
            }


            // z_ground_distance / atti_sen_wrt_robot
            if(sensor_core->isEstimationAttitudeSensorWrtRobotEnabled())
            {
                BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_state, jacobian_error_meas_ground_distance_wrt_error_state_sensor_att, dimension_error_measurement_i, dimension_sensor_error_state_i);

                dimension_sensor_error_state_i+=3;
            }
            else
            {
                BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_parameters, jacobian_error_meas_ground_distance_wrt_error_state_sensor_att, dimension_error_measurement_i, dimension_sensor_error_parameters_i);

                dimension_sensor_error_parameters_i+=3;
            }

            dimension_error_measurement_i+=1;
        }


        // Set From Triplets
        jacobian_error_measurement_wrt_sensor_error_state.setFromTriplets(triplet_list_jacobian_error_measurement_wrt_error_state.begin(), triplet_list_jacobian_error_measurement_wrt_error_state.end());
        jacobian_error_measurement_wrt_sensor_error_parameters.setFromTriplets(triplet_list_jacobian_error_measurement_wrt_error_parameters.begin(), triplet_list_jacobian_error_measurement_wrt_error_parameters.end());

    }


    /// Jacobian measurement - map element error state / error parameters

    {
        // TODO FIX
    }




    ///// Jacobians error measurement - sensor noise of the measurement

    {
        // Resize and init Jacobian
        jacobian_error_measurement_wrt_error_measurement.resize(dimension_error_measurement, dimension_error_measurement);

        std::vector<Eigen::Triplet<double>> triplet_list_jacobian_error_measurement_wrt_error_measurement;


        // Fill Jacobian
        int dimension_error_measurement_i=0;

        // z_lin_vel
        if(sensor_measurement->isVelocitySet())
        {
            // z_lin_vel
            BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_measurement_wrt_error_measurement, jacobian_error_meas_lin_vel_wrt_error_meas_lin_vel, dimension_error_measurement_i, dimension_error_measurement_i);

            dimension_error_measurement_i+=2;
        }


        // z_ground_distance
        if(sensor_measurement->isGroundDistanceSet())
        {
            // z_ground_distance
            triplet_list_jacobian_error_measurement_wrt_error_measurement.push_back(Eigen::Triplet<double>(dimension_error_measurement_i, dimension_error_measurement_i, jacobian_error_meas_ground_distance_wrt_error_meas_ground_distance));
            dimension_error_measurement_i+=1;
        }


        // Set From Triplets
        jacobian_error_measurement_wrt_error_measurement.setFromTriplets(triplet_list_jacobian_error_measurement_wrt_error_measurement.begin(), triplet_list_jacobian_error_measurement_wrt_error_measurement.end());

    }




    // End
    return 0;
}

int Px4FlowSensorCore::predictErrorMeasurementJacobianCore(// State: Robot
                                                            const Eigen::Vector3d& position_robot_wrt_world, const Eigen::Vector4d& attitude_robot_wrt_world,
                                                            const Eigen::Vector3d& lin_speed_robot_wrt_world, const Eigen::Vector3d& ang_velocity_robot_wrt_world,
                                                            // State: Sensor
                                                            const Eigen::Vector3d& position_sensor_wrt_robot, const Eigen::Vector4d& attitude_sensor_wrt_robot,
                                                            // Parameters: Sensor
                                                            // None
                                                            // Measurement
                                                            const Eigen::Vector2d& meas_lin_velocity_sensor_wrt_sensor, const double meas_ground_distance,
                                                            // Predicted Measurement
                                                            const Eigen::Vector2d& pred_meas_lin_velocity_sensor_wrt_sensor, const double pred_meas_ground_distance,
                                                           // Flags
                                                           bool flag_measurement_velocity, bool flag_measurement_ground_distance,
                                                            // Jacobians: State and Params
                                                            Eigen::Matrix<double, 2, 3>& jacobian_error_meas_lin_vel_wrt_error_state_robot_lin_vel, Eigen::Matrix<double, 2, 3>& jacobian_error_meas_lin_vel_wrt_error_state_robot_att, Eigen::Matrix<double, 2, 3>& jacobian_error_meas_lin_vel_wrt_error_state_robot_ang_vel,
                                                           Eigen::Matrix<double, 1, 3>& jacobian_error_meas_ground_distance_wrt_error_state_robot_pos, Eigen::Matrix<double, 1, 3>& jacobian_error_meas_ground_distance_wrt_error_state_robot_att,
                                                            Eigen::Matrix<double, 2, 3> &jacobian_error_meas_lin_vel_wrt_error_state_sensor_pos, Eigen::Matrix<double, 2, 3>& jacobian_error_meas_lin_vel_wrt_error_state_sensor_att,
                                                           Eigen::Matrix<double, 1, 3>& jacobian_error_meas_ground_distance_wrt_error_state_sensor_pos, Eigen::Matrix<double, 1, 3>& jacobian_error_meas_ground_distance_wrt_error_state_sensor_att,
                                                           // TODO: Jacobians ground_distance wrt map
                                                            // Jacobians: Noise
                                                            Eigen::Matrix2d& jacobian_error_meas_lin_vel_wrt_error_meas_lin_vel, double jacobian_error_meas_ground_distance_wrt_error_meas_ground_distance
                                                            )
{

    // Measurement velocity
    if(flag_measurement_velocity)
    {
        // Common
        Eigen::Vector4d att_sensor_wrt_world=Quaternion::cross(attitude_robot_wrt_world, attitude_sensor_wrt_robot);
        Eigen::Vector4d att_world_wrt_sensor=Quaternion::inv(att_sensor_wrt_world);
        Eigen::Vector4d att_world_wrt_robot=Quaternion::inv(attitude_robot_wrt_world);
        Eigen::Vector4d att_robot_wrt_sensor=Quaternion::inv(attitude_sensor_wrt_robot);


        // Vars
        Eigen::Vector3d ang_vel_robot_wrt_world_in_robot=Quaternion::cross_sandwich(Quaternion::inv(attitude_robot_wrt_world), ang_velocity_robot_wrt_world, attitude_robot_wrt_world);

        Eigen::Vector3d lin_vel_sensor_wrt_world_in_world=lin_speed_robot_wrt_world + Quaternion::cross_sandwich(attitude_robot_wrt_world, ang_vel_robot_wrt_world_in_robot.cross(position_sensor_wrt_robot), Quaternion::inv(attitude_robot_wrt_world));

        Eigen::Vector3d lin_vel_sensor_wrt_world_in_robot=Quaternion::cross_sandwich(Quaternion::inv(attitude_robot_wrt_world), lin_vel_sensor_wrt_world_in_world, attitude_robot_wrt_world);


        //
        Eigen::Matrix4d mat_quat_plus_att_world_wrt_sensor=Quaternion::quatMatPlus(att_world_wrt_sensor);
        //
        Eigen::Matrix4d mat_quat_minus_att_sensor_wrt_world=Quaternion::quatMatMinus(att_sensor_wrt_world);
        //
        Eigen::Matrix4d mat_quat_plus_att_robot_wrt_world=Quaternion::quatMatPlus(attitude_robot_wrt_world);
        Eigen::Matrix4d mat_quat_minus_att_robot_wrt_world=Quaternion::quatMatMinus(attitude_robot_wrt_world);
        //
        Eigen::Matrix4d mat_quat_plus_att_world_wrt_robot=Quaternion::quatMatPlus(att_world_wrt_robot);
        Eigen::Matrix4d mat_quat_minus_att_world_wrt_robot=Quaternion::quatMatMinus(att_world_wrt_robot);
        //
        Eigen::Matrix4d mat_quat_minus_att_sensor_wrt_robot=Quaternion::quatMatMinus(attitude_sensor_wrt_robot);
        Eigen::Matrix4d mat_quat_plus_att_sensor_wrt_robot=Quaternion::quatMatPlus(attitude_sensor_wrt_robot);
        //
        Eigen::Matrix4d mat_quat_plus_att_robot_wrt_sensor=Quaternion::quatMatPlus(att_robot_wrt_sensor);

        //
        Eigen::Matrix4d mat_quat_plus_vel_robot_wrt_world_in_world=Quaternion::quatMatPlus(lin_speed_robot_wrt_world);
        //
        Eigen::Matrix4d mat_quat_plus_ang_vel_robot_wrt_world_in_world=Quaternion::quatMatPlus(ang_velocity_robot_wrt_world);

        //
        Eigen::Matrix3d j2a=-Quaternion::skewSymMat(position_sensor_wrt_robot);


        // Fill
        // Error Meas Lin Vel / Robot Error State (/ Parameters)
        jacobian_error_meas_lin_vel_wrt_error_state_robot_lin_vel=
                this->sensitivity_meas_lin_vel_*Quaternion::jacobians.mat_diff_vector_wrt_vector_amp_sparse*mat_quat_plus_att_world_wrt_sensor*mat_quat_minus_att_sensor_wrt_world*Quaternion::jacobians.mat_diff_vector_amp_wrt_vector_sparse;

        jacobian_error_meas_lin_vel_wrt_error_state_robot_att=
                this->sensitivity_meas_lin_vel_*Quaternion::jacobians.mat_diff_vector_wrt_vector_amp_sparse*(
                (Quaternion::quatMatMinus(Quaternion::cross_pure_gen(lin_speed_robot_wrt_world, att_sensor_wrt_world))*Quaternion::jacobians.mat_diff_quat_inv_wrt_quat_sparse+mat_quat_plus_att_world_wrt_sensor*mat_quat_plus_vel_robot_wrt_world_in_world)*mat_quat_minus_att_sensor_wrt_robot
                    +
                mat_quat_plus_att_robot_wrt_sensor*mat_quat_minus_att_sensor_wrt_robot*Quaternion::jacobians.mat_diff_vector_amp_wrt_vector_sparse*j2a*Quaternion::jacobians.mat_diff_vector_wrt_vector_amp_sparse*(Quaternion::quatMatMinus(Quaternion::cross_pure_gen(ang_velocity_robot_wrt_world, attitude_robot_wrt_world))*Quaternion::jacobians.mat_diff_quat_inv_wrt_quat_sparse+mat_quat_plus_att_world_wrt_robot*mat_quat_plus_ang_vel_robot_wrt_world_in_world)
                )*mat_quat_plus_att_robot_wrt_world*Quaternion::jacobians.mat_diff_error_quat_wrt_error_theta_sparse;

        jacobian_error_meas_lin_vel_wrt_error_state_robot_ang_vel=
                this->sensitivity_meas_lin_vel_*Quaternion::jacobians.mat_diff_vector_wrt_vector_amp_sparse*mat_quat_plus_att_world_wrt_sensor*mat_quat_minus_att_sensor_wrt_world*
                mat_quat_plus_att_robot_wrt_world*mat_quat_minus_att_world_wrt_robot*
                Quaternion::jacobians.mat_diff_vector_amp_wrt_vector_sparse*j2a*Quaternion::jacobians.mat_diff_vector_wrt_vector_amp_sparse*
                mat_quat_plus_att_world_wrt_robot*mat_quat_minus_att_robot_wrt_world*Quaternion::jacobians.mat_diff_vector_amp_wrt_vector_sparse;

        // Error Meas Lin Vel / Sensor Error State (/ Parameters)
        jacobian_error_meas_lin_vel_wrt_error_state_sensor_pos=
                this->sensitivity_meas_lin_vel_*Quaternion::jacobians.mat_diff_vector_wrt_vector_amp_sparse*mat_quat_plus_att_world_wrt_sensor*mat_quat_minus_att_sensor_wrt_world*mat_quat_plus_att_robot_wrt_world*mat_quat_minus_att_world_wrt_robot*Quaternion::jacobians.mat_diff_vector_amp_wrt_vector_sparse*Quaternion::skewSymMat(ang_vel_robot_wrt_world_in_robot);

        jacobian_error_meas_lin_vel_wrt_error_state_sensor_att=
                this->sensitivity_meas_lin_vel_*Quaternion::jacobians.mat_diff_vector_wrt_vector_amp_sparse*(Quaternion::quatMatMinus(Quaternion::cross_pure_gen(lin_vel_sensor_wrt_world_in_robot, attitude_sensor_wrt_robot))*Quaternion::jacobians.mat_diff_quat_inv_wrt_quat_sparse+mat_quat_plus_att_robot_wrt_sensor*Quaternion::quatMatPlus(lin_vel_sensor_wrt_world_in_robot))*mat_quat_plus_att_sensor_wrt_robot*Quaternion::jacobians.mat_diff_error_quat_wrt_error_theta_sparse;


        // Error Meas Lin Vel / Error Meas Lin Vel
        jacobian_error_meas_lin_vel_wrt_error_meas_lin_vel=Eigen::Matrix2d::Identity(2,2);

    }

    // Measurement Ground Distance
    if(flag_measurement_ground_distance)
    {
        // Common
        // TODO FIX
        // Provisional variables for ground plane:
        // n
        Eigen::Vector3d ground_plane_normal(0,0,1);
        // xp
        Eigen::Vector3d ground_plane_point(0,0,0);

        // Distance sensor line
        Eigen::Vector4d att_sensor_wrt_world=Quaternion::cross(attitude_robot_wrt_world, attitude_sensor_wrt_robot);
        // u
        Eigen::Vector3d distance_sensor_tvect=Quaternion::cross_sandwich(att_sensor_wrt_world, Eigen::Vector3d(0,0,1), Quaternion::inv(att_sensor_wrt_world));

        // xr
        Eigen::Vector3d position_sensor_wrt_world=Quaternion::cross_sandwich(attitude_robot_wrt_world, position_sensor_wrt_robot, Quaternion::inv(attitude_robot_wrt_world))+position_robot_wrt_world;

        // Pred measurement
        double lambda_num=ground_plane_normal.dot(ground_plane_point-position_sensor_wrt_world);
        double lambda_den=ground_plane_normal.dot(distance_sensor_tvect);
        //double lamdba=lambda_num/lambda_den;
        //pred_meas_ground_distance=std::abs(lamdba);

        //
        Eigen::Vector3d distance_sensor_tvec_robot=Quaternion::cross_sandwich(attitude_sensor_wrt_robot, Eigen::Vector3d(0,0,1), Quaternion::inv(attitude_sensor_wrt_robot));


        // Fill
        //
        // Robot
        jacobian_error_meas_ground_distance_wrt_error_state_robot_pos=
                -1/lambda_den*ground_plane_normal.transpose();

        jacobian_error_meas_ground_distance_wrt_error_state_robot_att=
                1/pow(lambda_den,2)*(
                ground_plane_normal.dot(position_sensor_wrt_world-ground_plane_point)*ground_plane_normal.transpose()*Quaternion::jacobians.mat_diff_vector_wrt_vector_amp_sparse*
                    (Quaternion::quatMatPlus(Quaternion::cross_gen_pure(attitude_robot_wrt_world, distance_sensor_tvec_robot))*Quaternion::jacobians.mat_diff_quat_inv_wrt_quat_sparse+Quaternion::quatMatMinus(Quaternion::inv(attitude_robot_wrt_world))*Quaternion::quatMatMinus(distance_sensor_tvec_robot))
                -
                ground_plane_normal.dot(distance_sensor_tvect)*ground_plane_normal.transpose()*Quaternion::jacobians.mat_diff_vector_wrt_vector_amp_sparse*
                    (Quaternion::quatMatPlus(Quaternion::cross_gen_pure(attitude_robot_wrt_world, position_sensor_wrt_robot))*Quaternion::jacobians.mat_diff_quat_inv_wrt_quat_sparse+Quaternion::quatMatMinus(Quaternion::inv(attitude_robot_wrt_world))*Quaternion::quatMatMinus(position_sensor_wrt_robot))
                )*Quaternion::quatMatPlus(attitude_robot_wrt_world)*Quaternion::jacobians.mat_diff_error_quat_wrt_error_theta_sparse;

        //
        // Sensor
        jacobian_error_meas_ground_distance_wrt_error_state_sensor_pos;

        jacobian_error_meas_ground_distance_wrt_error_state_sensor_att;

        //
        // Map
        // TODO


        //
        jacobian_error_meas_ground_distance_wrt_error_meas_ground_distance=1;
    }


    // End
    return 0;
}

int Px4FlowSensorCore::resetErrorStateJacobian(// Time
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
