
#include "msf_localization_core/absolute_pose_input_core.h"

// Circular dependency
#include "msf_localization_core/msf_storage_core.h"


AbsolutePoseInputCore::AbsolutePoseInputCore() :
    InputCore()
{
    init();
    return;
}

AbsolutePoseInputCore::AbsolutePoseInputCore(const std::weak_ptr<MsfStorageCore> the_msf_storage_core) :
    InputCore(the_msf_storage_core)
{
    init();
    return;
}

AbsolutePoseInputCore::~AbsolutePoseInputCore()
{
    return;
}

int AbsolutePoseInputCore::init()
{
    // type
    setInputType(InputTypes::absolute_pose);

    // name -> default
    setInputName("absolute_pose_input");

    // World reference
    world_reference_frame_id_=-1;

    // Flags Input Commands
    flag_input_command_position_input_wrt_input_world_=false;
    flag_input_command_attitude_input_wrt_input_world_=false;
    flag_input_command_pose_input_wrt_input_world_has_covariance_=false;

    // Flags Estimation
    flag_estimation_position_input_wrt_robot_=false;
    flag_estimation_attitude_input_wrt_robot_=false;

    // dimensions
    // Command
    dimension_input_command_+=0;
    dimension_error_input_command_+=0;
    // State and parameters
    dimension_state_=0;
    dimension_error_state_=0;
    dimension_parameters_=6;
    dimension_error_parameters_=6;
    // Noise
    dimension_noise_=0;


    // Noises Input commands
    noise_input_command_position_input_wrt_input_world_.setZero();
    noise_input_command_attitude_input_wrt_input_world_.setZero();

    // Noise State / Parameters
    noise_position_input_wrt_robot_.setZero();
    noise_attitude_input_wrt_robot_.setZero();


    // End
    return 0;
}

int AbsolutePoseInputCore::readConfig(const pugi::xml_node &input, std::shared_ptr<AbsolutePoseInputStateCore> &init_state_core)
{
    // Create a class for the SensorStateCore
    if(!init_state_core)
        init_state_core=std::make_shared<AbsolutePoseInputStateCore>(this->getMsfElementCoreWeakPtr());


    // Auxiliar reading value
    std::string readingValue;



    //// Input configurations


    /// World Id
    readingValue=input.child_value("world_ref_frame_id");
    if(!readingValue.empty())
    {
        this->setWorldReferenceFrameId(std::stoi(readingValue));
    }


    /// Input Name
    std::string sensor_name=input.child_value("name");
    this->setInputName(sensor_name);



    /// Pose of the sensor wrt robot
    pugi::xml_node pose_in_robot=input.child("pose_in_robot");

    // Position of the sensor wrt robot
    readingValue=pose_in_robot.child("position").child_value("enabled");
    if(std::stoi(readingValue))
        this->enableEstimationPositionInputWrtRobot();

    // Attitude of the sensor wrt robot
    readingValue=pose_in_robot.child("attitude").child_value("enabled");
    if(std::stoi(readingValue))
        this->enableEstimationAttitudeInputWrtRobot();


    /// Other Parameters
    pugi::xml_node parameters = input.child("parameters");

    // None



    //// Commands
    pugi::xml_node commands = input.child("commands");

    /// Orientation
    pugi::xml_node meas_orientation = commands.child("orientation");

    readingValue=meas_orientation.child_value("enabled");
    if(std::stoi(readingValue))
        this->enableInputCommandAttitudeInputWrtInputWorld();

    readingValue=meas_orientation.child_value("var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoiseInputCommandAttitudeInputWrtInputWorld(variance.asDiagonal());
    }


    /// Position
    pugi::xml_node meas_position = commands.child("position");

    readingValue=meas_position.child_value("enabled");
    if(std::stoi(readingValue))
        this->enableInputCommandPositionInputWrtInputWorld();

    readingValue=meas_position.child_value("var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoiseInputCommandPositionInputWrtInputWorld(variance.asDiagonal());
    }

    /// Subscribed covariance of the command
    readingValue=commands.child_value("use_subscribed_cov");
    if(std::stoi(readingValue))
        this->setInputCommandPoseInputWrtInputWorldHasCovariance(true);


    //// Init State

    /// Pose of the sensor wrt robot

    // Position of the sensor wrt robot
    readingValue=pose_in_robot.child("position").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d init_estimation;
        stm>>init_estimation[0]>>init_estimation[1]>>init_estimation[2];
        init_state_core->setPositionInputWrtRobot(init_estimation);
    }

    // Attitude of the sensor wrt robot
    readingValue=pose_in_robot.child("attitude").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector4d init_estimation;
        stm>>init_estimation[0]>>init_estimation[1]>>init_estimation[2]>>init_estimation[3];
        init_state_core->setAttitudeInputWrtRobot(init_estimation);
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
        this->setNoisePositionInputWrtRobot(variance.asDiagonal());
    }

    // Attitude of the sensor wrt robot
    readingValue=pose_in_robot.child("attitude").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoiseAttitudeInputWrtRobot(variance.asDiagonal());
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

int AbsolutePoseInputCore::setWorldReferenceFrameId(int world_reference_frame_id)
{
    this->world_reference_frame_id_=world_reference_frame_id;
    return 0;
}

int AbsolutePoseInputCore::getWorldReferenceFrameId() const
{
    return this->world_reference_frame_id_;
}

bool AbsolutePoseInputCore::isInputCommandPositionInputWrtInputWorldEnabled() const
{
    return flag_input_command_position_input_wrt_input_world_;
}

int AbsolutePoseInputCore::enableInputCommandPositionInputWrtInputWorld()
{
    if(!flag_input_command_position_input_wrt_input_world_)
    {
        flag_input_command_position_input_wrt_input_world_=true;
        dimension_input_command_+=3;
        dimension_error_input_command_+=3;
    }
    return 0;
}

Eigen::Matrix3d AbsolutePoseInputCore::getNoiseInputCommandPositionInputWrtInputWorld() const
{
    return this->noise_input_command_position_input_wrt_input_world_;
}

void AbsolutePoseInputCore::setNoiseInputCommandPositionInputWrtInputWorld(const Eigen::Matrix3d& noise_input_command_position_input_wrt_input_world)
{
    this->noise_input_command_position_input_wrt_input_world_=noise_input_command_position_input_wrt_input_world;
    return;
}

bool AbsolutePoseInputCore::isInputCommandAttitudeInputWrtInputWorldEnabled() const
{
    return this->flag_input_command_attitude_input_wrt_input_world_;
}

int AbsolutePoseInputCore::enableInputCommandAttitudeInputWrtInputWorld()
{
    if(!flag_input_command_attitude_input_wrt_input_world_)
    {
        flag_input_command_attitude_input_wrt_input_world_=true;
        dimension_input_command_+=4;
        dimension_error_input_command_+=3;
    }
    return 0;
}

Eigen::Matrix3d AbsolutePoseInputCore::getNoiseInputCommandAttitudeInputWrtInputWorld() const
{
    return this->noise_input_command_attitude_input_wrt_input_world_;
}

void AbsolutePoseInputCore::setNoiseInputCommandAttitudeInputWrtInputWorld(const Eigen::Matrix3d& noise_input_command_attitude_input_wrt_input_world)
{
    this->noise_input_command_attitude_input_wrt_input_world_=noise_input_command_attitude_input_wrt_input_world;
    return;
}

bool AbsolutePoseInputCore::hasInputCommandPoseInputWrtInputWorldCovariance() const
{
    return this->flag_input_command_pose_input_wrt_input_world_has_covariance_;
}

void AbsolutePoseInputCore::setInputCommandPoseInputWrtInputWorldHasCovariance(bool flag_input_command_pose_input_wrt_input_world_has_covariance)
{
    flag_input_command_pose_input_wrt_input_world_has_covariance_=flag_input_command_pose_input_wrt_input_world_has_covariance;
    return;
}

int AbsolutePoseInputCore::setInputCommand(const TimeStamp& time_stamp, const std::shared_ptr<AbsolutePoseInputCommandCore> input_command_core)
{
    if(!this->isInputEnabled())
        return 0;

    if(!this->isCorrect())
    {
        std::cout<<"ERROR"<<std::endl;
        return -1;
    }

    if(this->getMsfStorageCoreSharedPtr()->setInputCommand(time_stamp, input_command_core))
        return 1;

    return 0;
}

bool AbsolutePoseInputCore::isEstimationPositionInputWrtRobotEnabled() const
{
    return this->flag_estimation_position_input_wrt_robot_;
}

int AbsolutePoseInputCore::enableEstimationPositionInputWrtRobot()
{
    if(!this->flag_estimation_position_input_wrt_robot_)
    {
        this->flag_estimation_position_input_wrt_robot_=true;
        dimension_state_+=3;
        dimension_error_state_+=3;
        dimension_parameters_-=3;
        dimension_error_parameters_-=3;
    }
    return 0;
}

int AbsolutePoseInputCore::enableParameterPositionInputWrtRobot()
{
    if(this->flag_estimation_position_input_wrt_robot_)
    {
        this->flag_estimation_position_input_wrt_robot_=false;
        dimension_state_-=3;
        dimension_error_state_-=3;
        dimension_parameters_+=3;
        dimension_error_parameters_+=3;
    }
    return 0;
}

Eigen::Matrix3d AbsolutePoseInputCore::getNoisePositionInputWrtRobot() const
{
    return this->noise_position_input_wrt_robot_;
}

void AbsolutePoseInputCore::setNoisePositionInputWrtRobot(const Eigen::Matrix3d& noise_position_input_wrt_robot)
{
    this->noise_position_input_wrt_robot_=noise_position_input_wrt_robot;
    return;
}

bool AbsolutePoseInputCore::isEstimationAttitudeInputWrtRobotEnabled() const
{
    return this->flag_estimation_attitude_input_wrt_robot_;
}

int AbsolutePoseInputCore::enableEstimationAttitudeInputWrtRobot()
{
    if(!this->flag_estimation_attitude_input_wrt_robot_)
    {
        this->flag_estimation_attitude_input_wrt_robot_=true;
        dimension_state_+=4;
        dimension_error_state_+=3;
        dimension_parameters_-=4;
        dimension_error_parameters_-=3;
    }
    return 0;
}

int AbsolutePoseInputCore::enableParameterAttitudeInputWrtRobot()
{
    if(this->flag_estimation_attitude_input_wrt_robot_)
    {
        this->flag_estimation_attitude_input_wrt_robot_=false;
        dimension_state_-=4;
        dimension_error_state_-=3;
        dimension_parameters_+=4;
        dimension_error_parameters_+=3;
    }
    return 0;
}

Eigen::Matrix3d AbsolutePoseInputCore::getNoiseAttitudeInputWrtRobot() const
{
    return this->noise_attitude_input_wrt_robot_;
}

void AbsolutePoseInputCore::setNoiseAttitudeInputWrtRobot(const Eigen::Matrix3d& noise_attitude_input_wrt_robot)
{
    this->noise_attitude_input_wrt_robot_=noise_attitude_input_wrt_robot;
    return;
}

int AbsolutePoseInputCore::prepareCovarianceInitErrorStateSpecific()
{
    int point=0;
    if(this->isEstimationPositionInputWrtRobotEnabled())
    {
        this->covariance_init_error_state_.block<3,3>(point,point)=this->getNoisePositionInputWrtRobot();
        point+=3;
    }
    if(this->isEstimationAttitudeInputWrtRobotEnabled())
    {
        this->covariance_init_error_state_.block<3,3>(point,point)=this->getNoiseAttitudeInputWrtRobot();
        point+=3;
    }

    return 0;
}

Eigen::SparseMatrix<double> AbsolutePoseInputCore::getCovarianceInputs(const TimeStamp deltaTimeStamp)
{
    Eigen::SparseMatrix<double> covariances_matrix;
    covariances_matrix.resize(this->getDimensionErrorInputCommand(), this->getDimensionErrorInputCommand());

    std::vector<Eigen::Triplet<double> > tripletCovarianceMeasurement;

    double dt=deltaTimeStamp.get_double();
    double dt2=dt*dt;

    unsigned int dimension=0;
    if(this->isInputCommandPositionInputWrtInputWorldEnabled())
    {
        for(int i=0; i<3; i++)
            tripletCovarianceMeasurement.push_back(Eigen::Triplet<double>(dimension+i,dimension+i, dt2*noise_input_command_position_input_wrt_input_world_(i,i)));


        dimension+=3;
    }
    if(this->isInputCommandAttitudeInputWrtInputWorldEnabled())
    {
        for(int i=0; i<3; i++)
            tripletCovarianceMeasurement.push_back(Eigen::Triplet<double>(dimension+i,dimension+i, dt2*noise_input_command_attitude_input_wrt_input_world_(i,i)));


        dimension+=3;
    }

    covariances_matrix.setFromTriplets(tripletCovarianceMeasurement.begin(), tripletCovarianceMeasurement.end());

    return covariances_matrix;
}

Eigen::SparseMatrix<double> AbsolutePoseInputCore::getCovarianceParameters()
{
    Eigen::SparseMatrix<double> covariances_matrix;
    covariances_matrix.resize(this->getDimensionErrorParameters(), this->getDimensionErrorParameters());

    std::vector<Eigen::Triplet<double> > tripletCovarianceParameters;

    unsigned int dimension=0;
    if(!this->isEstimationPositionInputWrtRobotEnabled())
    {
        for(int i=0; i<3; i++)
            tripletCovarianceParameters.push_back(Eigen::Triplet<double>(dimension+i,dimension+i,noise_position_input_wrt_robot_(i,i)));

        dimension+=3;
    }
    if(!this->isEstimationAttitudeInputWrtRobotEnabled())
    {
        for(int i=0; i<3; i++)
            tripletCovarianceParameters.push_back(Eigen::Triplet<double>(dimension+i,dimension+i,noise_attitude_input_wrt_robot_(i,i)));

        dimension+=3;
    }

    covariances_matrix.setFromTriplets(tripletCovarianceParameters.begin(), tripletCovarianceParameters.end());


    return covariances_matrix;
}

Eigen::SparseMatrix<double> AbsolutePoseInputCore::getCovarianceNoise(const TimeStamp deltaTimeStamp)
{
    Eigen::SparseMatrix<double> covariance_noise;
    covariance_noise.resize(getDimensionNoise(), getDimensionNoise());

    // Fill
    int dimension_noise_i=0;

    // Nothing


    // End
    return covariance_noise;
}

int AbsolutePoseInputCore::predictState(//Time
                                         const TimeStamp &previousTimeStamp, const TimeStamp &currentTimeStamp,
                                         // Previous State
                                         const std::shared_ptr<StateEstimationCore> &pastState,
                                         // Inputs
                                         const std::shared_ptr<InputCommandComponent> &inputCommand,
                                         // Predicted State
                                         std::shared_ptr<StateCore>& predictedState)
{
    // Checks

    // Past State
    if(!pastState)
        return -1;

    // TODO

    // Search for the past input State Core
    std::shared_ptr<AbsolutePoseInputStateCore> past_input_state;
    if(findState(pastState->TheListInputStateCore, past_input_state))
        return -2;
    if(!past_input_state)
        return -10;


    // Predicted State
    std::shared_ptr<AbsolutePoseInputStateCore> predicted_input_state;
    if(!predictedState)
        predicted_input_state=std::make_shared<AbsolutePoseInputStateCore>(past_input_state->getMsfElementCoreWeakPtr());
    else
        predicted_input_state=std::dynamic_pointer_cast<AbsolutePoseInputStateCore>(predictedState);




    // Predict State
    int error_predict_state=predictStateSpecific(previousTimeStamp, currentTimeStamp,
                                         past_input_state,
                                         predicted_input_state);

    // Check error
    if(error_predict_state)
        return error_predict_state;


    // Set predicted state
    predictedState=predicted_input_state;


    // End
    return 0;
}

int AbsolutePoseInputCore::predictStateSpecific(const TimeStamp &previousTimeStamp, const TimeStamp &currentTimeStamp,
                                                   const std::shared_ptr<AbsolutePoseInputStateCore> pastState,
                                                   std::shared_ptr<AbsolutePoseInputStateCore>& predictedState)
{
    // Checks in the past state
    if(!pastState->isCorrect())
    {
        std::cout<<"AbsolutePoseInputCore::predictStateSpecific() error"<<std::endl;
        return -5;
    }


    // Create the predicted state if it doesn't exist
    if(!predictedState)
    {
        predictedState=std::make_shared<AbsolutePoseInputStateCore>(pastState->getMsfElementCoreSharedPtr());
    }


    /// Aux values
    Eigen::Vector3d position_input_wrt_robot;
    Eigen::Vector4d attitude_input_wrt_robot;
    // State k+1: input
    Eigen::Vector3d pred_position_input_wrt_robot;
    Eigen::Vector4d pred_attitude_input_wrt_robot;


    /// Fill
    position_input_wrt_robot=pastState->position_input_wrt_robot_;
    attitude_input_wrt_robot=pastState->attitude_input_wrt_robot_;;


    //Delta Time
    TimeStamp DeltaTime=currentTimeStamp-previousTimeStamp;
    double dt=DeltaTime.get_double();


    //// Core

    int error_predict_state_core=this->predictStateCore(position_input_wrt_robot, attitude_input_wrt_robot,
                                                        pred_position_input_wrt_robot, pred_attitude_input_wrt_robot);

    if(error_predict_state_core)
        return error_predict_state_core;



    /// Fill


    /// Position
    if(this->isEstimationPositionInputWrtRobotEnabled())
    {
        // Estimation
        predictedState->position_input_wrt_robot_=pred_position_input_wrt_robot;
    }
    else
    {
        // Parameter
        predictedState->position_input_wrt_robot_=pred_position_input_wrt_robot;
    }


    /// Attitude
    if(this->isEstimationAttitudeInputWrtRobotEnabled())
    {
        predictedState->attitude_input_wrt_robot_=pred_attitude_input_wrt_robot;
    }
    else
    {
        predictedState->attitude_input_wrt_robot_=pred_attitude_input_wrt_robot;
    }


    return 0;
}

int AbsolutePoseInputCore::predictStateCore(// State k: Input
                                             const Eigen::Vector3d& position_input_wrt_robot, const Eigen::Vector4d& attitude_input_wrt_robot,
                                             // State k+1: Input
                                             Eigen::Vector3d& pred_position_input_wrt_robot, Eigen::Vector4d& pred_attitude_input_wrt_robot)
{
    // Position
    pred_position_input_wrt_robot=position_input_wrt_robot;

    // Attitude
    pred_attitude_input_wrt_robot=attitude_input_wrt_robot;

    return 0;
}

int AbsolutePoseInputCore::predictErrorStateJacobian(//Time
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


    // Search for the past input State Core
    std::shared_ptr<AbsolutePoseInputStateCore> past_input_state;
    if(findState(past_state->TheListInputStateCore, past_input_state))
        return -2;
    if(!past_input_state)
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
    std::shared_ptr<AbsolutePoseInputStateCore> predicted_input_state;
    predicted_input_state=std::dynamic_pointer_cast<AbsolutePoseInputStateCore>(predicted_state);



    /// Get iterators to fill jacobians

    // Fx & Fp
    // input
    std::vector<Eigen::SparseMatrix<double> >::iterator it_jacobian_error_state_wrt_input_error_state;
    it_jacobian_error_state_wrt_input_error_state=predicted_input_state->jacobian_error_state_.inputs.begin();

    std::vector<Eigen::SparseMatrix<double> >::iterator it_jacobian_error_state_wrt_input_error_parameters;
    it_jacobian_error_state_wrt_input_error_parameters=predicted_input_state->jacobian_error_parameters_.inputs.begin();

    for(std::list< std::shared_ptr<StateCore> >::iterator itInputStateCore=past_state->TheListInputStateCore.begin();
        itInputStateCore!=past_state->TheListInputStateCore.end();
        ++itInputStateCore, ++it_jacobian_error_state_wrt_input_error_state, ++it_jacobian_error_state_wrt_input_error_parameters
        )
    {
        if( std::dynamic_pointer_cast<AbsolutePoseInputStateCore>((*itInputStateCore)) == past_input_state )
            break;
    }


    // Fu
    // Nothing


    // Fn
    // Nothing



    /// Predict State Jacobians
    int error_predict_state_jacobians=predictErrorStateJacobiansSpecific(previousTimeStamp, currentTimeStamp,
                                                                        past_input_state,
                                                                        predicted_input_state,
                                                                        // Jacobians Error State: Fx, Fp
                                                                        // input
                                                                        (*it_jacobian_error_state_wrt_input_error_state),
                                                                        (*it_jacobian_error_state_wrt_input_error_parameters)
                                                                        );

    // Check error
    if(error_predict_state_jacobians)
        return error_predict_state_jacobians;



    /// Set predicted state
    predicted_state=predicted_input_state;

    // End
    return 0;
}

int AbsolutePoseInputCore::predictErrorStateJacobiansSpecific(const TimeStamp &previousTimeStamp, const TimeStamp &currentTimeStamp,
                                                            const std::shared_ptr<AbsolutePoseInputStateCore> pastState,
                                                            std::shared_ptr<AbsolutePoseInputStateCore> &predictedState,
                                                            // Jacobians Error State: Fx, Fp
                                                            // input
                                                            Eigen::SparseMatrix<double>& jacobian_error_state_wrt_input_error_state,
                                                            Eigen::SparseMatrix<double>& jacobian_error_state_wrt_input_error_parameters
                                                            // Jacobians Noise: Hn
                                                            // Nothing
                                                            )
{
    // Create the predicted state if it doesn't exist
    if(!predictedState)
    {
        return 1;
    }

    // Variables
    // State k: input
    Eigen::Vector3d position_input_wrt_robot;
    Eigen::Vector4d attitude_input_wrt_robot;
    // State k+1: input
    Eigen::Vector3d pred_position_input_wrt_robot;
    Eigen::Vector4d pred_attitude_input_wrt_robot;

    // Jacobian: State
    Eigen::Matrix3d jacobian_error_sens_pos_wrt_error_state_sens_pos;
    Eigen::Matrix3d jacobian_error_sens_att_wrt_error_state_sens_att;


    //Delta Time
    TimeStamp DeltaTime=currentTimeStamp-previousTimeStamp;
    // delta time
    double dt=DeltaTime.get_double();


    /// Fill variables
    // State k: input
    position_input_wrt_robot=pastState->getPositionInputWrtRobot();
    attitude_input_wrt_robot=pastState->getAttitudeInputWrtRobot();

    // State k+1: input
    pred_position_input_wrt_robot=predictedState->getPositionInputWrtRobot();
    pred_attitude_input_wrt_robot=predictedState->getAttitudeInputWrtRobot();


    //// Core

    int error_predict_error_state_jacobians_core=predictErrorStateJacobiansCore(// State k: input
                                                                                position_input_wrt_robot,
                                                                                attitude_input_wrt_robot,
                                                                                // State k+1: input
                                                                                pred_position_input_wrt_robot,
                                                                                pred_attitude_input_wrt_robot,
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

    /// inputs
    {
        // Resize and init
        jacobian_error_state_wrt_input_error_state.resize(dimension_error_state_, dimension_error_state_);
        jacobian_error_state_wrt_input_error_parameters.resize(dimension_error_state_, dimension_error_parameters_);

        std::vector<Eigen::Triplet<double>> triplet_list_jacobian_error_state_wrt_error_state;
        std::vector<Eigen::Triplet<double>> triplet_list_jacobian_error_state_wrt_error_parameters;


        // Fill
        int dimension_error_state_i=0;
        int dimension_error_parameters_i=0;


        // Position input wrt robot
        if(this->isEstimationPositionInputWrtRobotEnabled())
        {
            // Add to the triplets
            BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_state_wrt_error_state, jacobian_error_sens_pos_wrt_error_state_sens_pos, dimension_error_state_i, dimension_error_state_i);

            // Update dimension for next
            dimension_error_state_i+=3;
        }

        // Attitude input wrt robot
        if(this->isEstimationAttitudeInputWrtRobotEnabled())
        {
            // Add to triplet list
            BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_state_wrt_error_state, jacobian_error_sens_att_wrt_error_state_sens_att, dimension_error_state_i, dimension_error_state_i);

            // Update dimension for next
            dimension_error_state_i+=3;
        }

        // Set From Triplets
        jacobian_error_state_wrt_input_error_state.setFromTriplets(triplet_list_jacobian_error_state_wrt_error_state.begin(), triplet_list_jacobian_error_state_wrt_error_state.end());
        jacobian_error_state_wrt_input_error_parameters.setFromTriplets(triplet_list_jacobian_error_state_wrt_error_parameters.begin(), triplet_list_jacobian_error_state_wrt_error_parameters.end());


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
        // Nothing to do
    }



    // End
    return 0;
}

int AbsolutePoseInputCore::predictErrorStateJacobiansCore(// State k: input
                                                           const Eigen::Vector3d& position_input_wrt_robot, const Eigen::Vector4d& attitude_input_wrt_robot,
                                                           // State k+1: input
                                                           const Eigen::Vector3d& pred_position_input_wrt_robot, const Eigen::Vector4d& pred_attitude_input_wrt_robot,
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

int AbsolutePoseInputCore::resetErrorStateJacobian(// Time
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
    current_state->jacobian_error_state_reset_.resize(this->getDimensionErrorState(), this->getDimensionErrorState());

    // Fill
    std::vector< Eigen::Triplet<double> > triplets_jacobian_error_reset;

    int dimension_error_state_i=0;

    // Position input Wrt Robot
    if(this->isEstimationPositionInputWrtRobotEnabled())
    {
        for(int i=0; i<3; i++)
            triplets_jacobian_error_reset.push_back(Eigen::Triplet<double>(dimension_error_state_i+i, dimension_error_state_i+i, 1.0));

        dimension_error_state_i+=3;
    }

    // Attitude input Wrt Robot
    if(this->isEstimationAttitudeInputWrtRobotEnabled())
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

int AbsolutePoseInputCore::findState(const std::list<std::shared_ptr<StateCore> > &list_state, std::shared_ptr<AbsolutePoseInputStateCore>& found_state)
{
    for(std::list< std::shared_ptr<StateCore> >::const_iterator it_state=list_state.begin();
        it_state!=list_state.end();
        ++it_state)
    {
        if((*it_state)->getMsfElementCoreSharedPtr() == this->getMsfElementCoreSharedPtr())
        {
            found_state=std::dynamic_pointer_cast<AbsolutePoseInputStateCore>(*it_state);
            return 0;
        }
    }
    return -1;
}
