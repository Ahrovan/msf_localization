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
    //noiseBiasAngularVelocity.setZero();

    // Noises estimation
    //noiseEstimationBiasAngularVelocity.setZero();


    return 0;
}

int Px4FlowSensorCore::readConfig(const pugi::xml_node& sensor, const unsigned int sensor_id, std::shared_ptr<Px4FlowSensorStateCore>& sensor_init_state)
{

    // Create a class for the SensorStateCore
    if(!sensor_init_state)
        sensor_init_state=std::make_shared<Px4FlowSensorStateCore>(this->getMsfElementCoreWeakPtr());


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

    /*
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
    */




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

    // Bias Angular Velocity
    /*
    readingValue=param_angular_velocity.child("biases").child_value("init_estimation");
    if(!readingValue.empty())
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d init_estimation;
        stm>>init_estimation[0]>>init_estimation[1]>>init_estimation[2];
        sensor_init_state->setBiasesAngularVelocity(init_estimation);
    }*/




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

    /*
    // Bias Linear Acceleration
    readingValue=param_linear_acceleration.child("biases").child_value("init_var");
    if(!readingValue.empty())
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoiseBiasLinearAcceleration(variance.asDiagonal());
    }
    */




    // Noises in the estimation (if enabled)

    /*
    // Bias Linear Acceleration
    if(this->isEstimationBiasLinearAccelerationEnabled())
    {
        readingValue=param_linear_acceleration.child("biases").child_value("noise");
        if(!readingValue.empty())
        {
            std::istringstream stm(readingValue);
            Eigen::Vector3d variance;
            stm>>variance[0]>>variance[1]>>variance[2];
            this->setNoiseEstimationBiasLinearAcceleration(variance.asDiagonal());
        }
    }
    */



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

Eigen::SparseMatrix<double> Px4FlowSensorCore::getCovarianceMeasurement()
{
    Eigen::SparseMatrix<double> covariances_matrix;

    covariances_matrix.resize(this->getDimensionErrorMeasurement(), this->getDimensionErrorMeasurement());
    //covariances_matrix.setZero();
    covariances_matrix.reserve(this->getDimensionErrorMeasurement());

    std::vector<Eigen::Triplet<double> > triplets_covariance_measurement;

    unsigned int dimension=0;

    if(isMeasurementVelocityEnabled())
    {
        for(int i=0; i<2; i++)
            triplets_covariance_measurement.push_back(Eigen::Triplet<double>(dimension+i, dimension+i, noise_measurement_velocity_(i,i)));

        dimension+=3;
    }

    if(isMeasurementGroundDistanceEnabled())
    {
        triplets_covariance_measurement.push_back(Eigen::Triplet<double>(dimension, dimension, noise_measurement_ground_distance_));

        dimension+=1;
    }


    covariances_matrix.setFromTriplets(triplets_covariance_measurement.begin(), triplets_covariance_measurement.end());


    return covariances_matrix;
}

Eigen::SparseMatrix<double> Px4FlowSensorCore::getCovarianceParameters()
{
    Eigen::SparseMatrix<double> CovariancesMatrix;

    CovariancesMatrix.resize(this->getDimensionErrorParameters(), this->getDimensionErrorParameters());

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
    /*
    if(!this->isEstimationBiasLinearAccelerationEnabled())
    {
        //CovariancesMatrix.block<3,3>(dimension, dimension)=this->getNoiseBiasLinearAcceleration();

        for(int i=0; i<3; i++)
            tripletCovarianceParameters.push_back(Eigen::Triplet<double>(dimension+i,dimension+i,noiseBiasLinearAcceleration(i,i)));

        dimension+=3;
    }
    */


    CovariancesMatrix.setFromTriplets(tripletCovarianceParameters.begin(), tripletCovarianceParameters.end());


    return CovariancesMatrix;
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
    /*
    if(this->isEstimationBiasLinearAccelerationEnabled())
    {
        this->covariance_init_error_state_.block<3,3>(point,point)=noiseBiasLinearAcceleration;
        point+=3;
    }
    */


    return 0;
}


Eigen::SparseMatrix<double> Px4FlowSensorCore::getCovarianceNoise(const TimeStamp deltaTimeStamp)
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
    /*
    if(isEstimationBiasLinearAccelerationEnabled())
    {
        //covariance_noise.block<3,3>(dimension_noise_i,dimension_noise_i)=this->noiseEstimationBiasLinearAcceleration*deltaTimeStamp.get_double();

        for(int i=0; i<3; i++)
            tripletCovarianceNoise.push_back(Eigen::Triplet<double>(dimension_noise_i+i,dimension_noise_i+i,noiseEstimationBiasLinearAcceleration(i,i)*dt));


        dimension_noise_i+=3;
    }
    */



    covariance_noise.setFromTriplets(tripletCovarianceNoise.begin(), tripletCovarianceNoise.end());



    // End
    return covariance_noise;
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


    // Bias Angular Velocity
    //predictedState->biasesAngularVelocity=pastState->biasesAngularVelocity;


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
    //Eigen::Matrix3d jacobian_error_bias_lin_acc_wrt_error_bias_lin_acc;




    /// Fill variables
    // State k: Sensor
    position_sensor_wrt_robot=pastState->getPositionSensorWrtRobot();
    attitude_sensor_wrt_robot=pastState->getAttitudeSensorWrtRobot();

    // State k+1: Sensor
    pred_position_sensor_wrt_robot=predictedState->getPositionSensorWrtRobot();
    pred_attitude_sensor_wrt_robot=predictedState->getAttitudeSensorWrtRobot();


    //// Core
    /*
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
    */

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

        /*
        // bias linear acceleration
        if(this->isEstimationBiasLinearAccelerationEnabled())
        {
            // Add to the triplets
            BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_state_wrt_error_state, jacobian_error_bias_lin_acc_wrt_error_bias_lin_acc, dimension_error_state_i, dimension_error_state_i);

            // Update dimension for next
            dimension_error_state_i+=3;
        }
        */





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


        /*
        // bias linear acceleration
        if(isEstimationBiasLinearAccelerationEnabled())
        {
            int dimension_error_state_i=0;

            if(isEstimationPositionSensorWrtRobotEnabled())
                dimension_error_state_i+=3;

            if(isEstimationAttitudeSensorWrtRobotEnabled())
                dimension_error_state_i+=3;

            // Update jacobian
            //jacobian_error_state_noise.block<3,3>(dimension_error_state_i, dimension_noise_i)=Eigen::Matrix3d::Identity(3,3);
            for(int i=0; i<3; i++)
                tripletJacobianErrorStateNoise.push_back(Eigen::Triplet<double>(dimension_error_state_i+i,dimension_noise_i+i,1));


            // Update dimension for next
            dimension_noise_i+=3;
        }
        */



        // Set From Triplets
        jacobian_error_state_wrt_noise.setFromTriplets(tripletJacobianErrorStateNoise.begin(), tripletJacobianErrorStateNoise.end());

    }


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


    // Predict State
    int error_predict_measurement=predictMeasurementSpecific(current_time_stamp,
                                                             dynamic_cast<RobotStateCore*>(current_state->TheRobotStateCore.get()),
                                                             current_sensor_state,
                                                             predicted_sensor_measurement);

    // Check error
    if(error_predict_measurement)
        return error_predict_measurement;



    // End
    return 0;
}

int Px4FlowSensorCore::predictMeasurementSpecific(const TimeStamp &theTimeStamp,
                                                  const RobotStateCore *currentRobotState,
                                                  const Px4FlowSensorStateCore *currentImuState,
                                                  Px4FlowSensorMeasurementCore *&predictedMeasurement)
{


    // Check
    if(!isCorrect())
    {
        std::cout<<"ImuSensorCore::predictMeasurementSpecific() error 50"<<std::endl;
        return -50;
    }

    // Check sensor state
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
        predictedMeasurement=new Px4FlowSensorMeasurementCore;
        predictedMeasurement->setSensorCorePtr(std::dynamic_pointer_cast<SensorCore>(this->getMsfElementCoreSharedPtr()));
    }




    // Variables
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
    //Eigen::Vector3d biases_meas_linear_acceleration;
//    // Measurement
//    Eigen::Vector3d meas_lin_accel_sensor_wrt_sensor;
//    // Predicted Measurement
//    Eigen::Vector3d lin_accel_sensor_wrt_sensor;


    /// Fill Variables



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
    //biases_meas_linear_acceleration=currentImuState->getBiasesLinearAcceleration();


//    // Measurement
//    meas_lin_accel_sensor_wrt_sensor;


//    // Predicted Measurement
//    lin_accel_sensor_wrt_sensor;



    /// Measurements Prediction


    /*
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
    */




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


    //// Init Jacobians
    int error_init_jacobians=predictErrorMeasurementJacobianInit(// Current State
                                                                current_state,
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


int Px4FlowSensorCore::predictErrorMeasurementJacobianSpecific(const TimeStamp& theTimeStamp,
                                                           const RobotStateCore* TheRobotStateCore,
                                                           const Px4FlowSensorStateCore* TheImuStateCore,
                                                           Px4FlowSensorMeasurementCore*& predictedMeasurement,
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

    //predictedMeasurement



    /// Common Variables

    // Imu sensor core
    std::shared_ptr<const Px4FlowSensorCore> the_imu_sensor_core=std::dynamic_pointer_cast<const Px4FlowSensorCore>(TheImuStateCore->getMsfElementCoreSharedPtr());


    // dimension of the measurement
    int dimension_error_measurement=this->getDimensionErrorMeasurement();




    /// Variables

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
    //Eigen::Matrix3d sensitivity_meas_linear_acceleration;
    // Measurement
    //Eigen::Vector3d meas_lin_accel_sensor_wrt_sensor;
    // Predicted Measurement
    //Eigen::Vector3d lin_accel_sensor_wrt_sensor;



    /// Fill Variables



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
    //sensitivity_meas_linear_acceleration=TheImuStateCore->getScaleLinearAcceleration().asDiagonal();


    // Measurement
    //meas_lin_accel_sensor_wrt_sensor;


    // Predicted Measurement
    //lin_accel_sensor_wrt_sensor;





    /// Jacobians Variables

    // Jacobian State
/*

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

*/


    /// Call core
/*
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
*/

    /// Dimensions


    // dimension of robot
    int dimension_robot_error_state=TheRobotStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorState();
    int dimension_robot_error_parameters=TheRobotStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorParameters();

    // dimension of the sensor
    int dimension_sensor_error_state=TheImuStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorState();
    int dimension_sensor_error_parameters=TheImuStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorParameters();




    ///// Jacobians error measurement - error state / error parameters

/*
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

    */


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

    /*
    // Bias Linear Acceleration
    if(this->isEstimationBiasLinearAccelerationEnabled())
    {
        for(int i=0; i<3; i++)
            triplets_jacobian_error_reset.push_back(Eigen::Triplet<double>(dimension_error_state_i+i, dimension_error_state_i+i, 1.0));

        dimension_error_state_i+=3;
    }
    */



    current_state->jacobian_error_state_reset_.setFromTriplets(triplets_jacobian_error_reset.begin(), triplets_jacobian_error_reset.end());

    // End
    return 0;
}
