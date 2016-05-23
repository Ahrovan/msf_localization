
#include "msf_localization_core/absolute_pose_driven_robot_core.h"

#include "msf_localization_core/absolute_pose_input_core.h"
#include "msf_localization_core/world_reference_frame_core.h"


AbsolutePoseDrivenRobotCore::AbsolutePoseDrivenRobotCore() :
    RobotCore()
{
    init();

    return;
}

AbsolutePoseDrivenRobotCore::AbsolutePoseDrivenRobotCore(const std::weak_ptr<MsfStorageCore> msf_storage_core_ptr) :
    RobotCore(msf_storage_core_ptr)
{
    init();

    return;
}

AbsolutePoseDrivenRobotCore::~AbsolutePoseDrivenRobotCore()
{
    return;
}

int AbsolutePoseDrivenRobotCore::init()
{
    dimension_state_=3+4;
    dimension_error_state_=3+3;

    dimension_parameters_=0;
    dimension_error_parameters_=0;

    dimension_noise_=3+3;

    noisePosition.setZero();
    noiseAttitude.setZero();


    // Type
    this->setRobotCoreType(RobotCoreTypes::absolute_pose_driven);


    return 0;
}

int AbsolutePoseDrivenRobotCore::readConfig(const pugi::xml_node &robot, std::shared_ptr<AbsolutePoseDrivenRobotStateCore>& RobotInitStateCore)
{

    // Create a class for the RobotStateCore
    if(!RobotInitStateCore)
        RobotInitStateCore=std::make_shared<AbsolutePoseDrivenRobotStateCore>(this->getMsfElementCoreWeakPtr());
    // Set pointer to the SensorCore
    //RobotInitStateCore->setTheRobotCore(TheRobotCoreCore);


    // Aux vars
    std::string readingValue;


//    // Name
//    readingValue=robot.child_value("name");
//    this->setRobotName(readingValue);


    /// Init State

    // Position
    readingValue=robot.child("position").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d position;
        stm>>position[0]>>position[1]>>position[2];
        RobotInitStateCore->setPositionRobotWrtWorld(position);
    }

    // Attitude
    readingValue=robot.child("attitude").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector4d attitude;
        stm>>attitude[0]>>attitude[1]>>attitude[2]>>attitude[3];
        RobotInitStateCore->setAttitudeRobotWrtWorld(attitude);
    }


    /// Init Variances

    // Prepare Covariance of init error state -> After preparing, the parameters are read!
    this->prepareCovarianceInitErrorState();

    // Position
    readingValue=robot.child("position").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setInitErrorStateVariancePosition(variance);
    }

    // Attitude
    readingValue=robot.child("attitude").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setInitErrorStateVarianceAttitude(variance);
    }



    // Noises Estimation

    // Linear acceleration
    readingValue=robot.child("position").child_value("noise");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoisePosition(variance.asDiagonal());
    }


    // Attitude
    readingValue=robot.child("attitude").child_value("noise");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoiseAttitude(variance.asDiagonal());
    }


    // End
    return 0;
}


Eigen::Matrix3d AbsolutePoseDrivenRobotCore::getNoisePosition() const
{
    return this->noisePosition;
}

int AbsolutePoseDrivenRobotCore::setNoisePosition(const Eigen::Matrix3d &noisePosition)
{
    this->noisePosition=noisePosition;
    return 0;
}

Eigen::Matrix3d AbsolutePoseDrivenRobotCore::getNoiseAttitude() const
{
    return this->noiseAttitude;
}

int AbsolutePoseDrivenRobotCore::setNoiseAttitude(const Eigen::Matrix3d &noiseAttitude)
{
    this->noiseAttitude=noiseAttitude;
    return 0;
}

Eigen::SparseMatrix<double> AbsolutePoseDrivenRobotCore::getCovarianceParameters()
{
    // No parameters

    return Eigen::SparseMatrix<double>();
}

Eigen::SparseMatrix<double> AbsolutePoseDrivenRobotCore::getCovarianceNoise(const TimeStamp deltaTimeStamp)
{
    Eigen::SparseMatrix<double> covariance_noise;

    // Resize
    covariance_noise.resize(dimension_noise_, dimension_noise_);
    //covariance_noise.setZero();
    covariance_noise.reserve(dimension_noise_);

    std::vector<Eigen::Triplet<double> > tripletList;

    //
    double dt=deltaTimeStamp.get_double();

    int dimension_noise_i=0;

    // Fill

    // Position
    for(int i=0; i<3; i++)
        tripletList.push_back(Eigen::Triplet<double>(dimension_noise_i+i,dimension_noise_i+i,this->noisePosition(i,i)*dt));
    dimension_noise_i+=3;


    // Attitude
    for(int i=0; i<3; i++)
        tripletList.push_back(Eigen::Triplet<double>(dimension_noise_i+i,dimension_noise_i+i,this->noiseAttitude(i,i)*dt));
    dimension_noise_i+=3;


    covariance_noise.setFromTriplets(tripletList.begin(), tripletList.end());


//    {
//        std::ostringstream logString;
//        logString<<"covariance_noise"<<std::endl;
//        logString<<covariance_noise<<std::endl;
//        this->log(logString.str());
//    }


    // End
    return covariance_noise;
}

int AbsolutePoseDrivenRobotCore::prepareCovarianceInitErrorStateSpecific()
{

    // Do nothing else. The rest is done when setting!

    return 0;
}

int AbsolutePoseDrivenRobotCore::setInitErrorStateVariancePosition(const Eigen::Vector3d &initVariance)
{
    this->covariance_init_error_state_.block<3,3>(0,0)=initVariance.asDiagonal();
    return 0;
}

int AbsolutePoseDrivenRobotCore::setInitErrorStateVarianceAttitude(const Eigen::Vector3d& initVariance)
{
    this->covariance_init_error_state_.block<3,3>(3,3)=initVariance.asDiagonal();
    return 0;
}

int AbsolutePoseDrivenRobotCore::predictState(//Time
                                             const TimeStamp &previousTimeStamp,
                                             const TimeStamp &currentTimeStamp,
                                             // Previous State
                                             const std::shared_ptr<StateEstimationCore> &pastState,
                                             // Inputs
                                             const std::shared_ptr<InputCommandComponent> &inputCommand,
                                             // Predicted State
                                             std::shared_ptr<StateCore> &predictedState)
{
    /// Checks

    // Past State
    if(!pastState)
        return -1;

    // TODO



    /// Robot Predicted State
    AbsolutePoseDrivenRobotStateCore* predicted_robot_state(nullptr);
    if(!predictedState)
    {
        predicted_robot_state=new AbsolutePoseDrivenRobotStateCore;
        predicted_robot_state->setMsfElementCorePtr(pastState->TheRobotStateCore->getMsfElementCoreWeakPtr());
        predictedState=std::shared_ptr<StateCore>(predicted_robot_state);
    }
    else
        predicted_robot_state=dynamic_cast<AbsolutePoseDrivenRobotStateCore*>(predictedState.get());


    /// Past State

    // Past Robot State
    // Nothing needed

    // Past Input State
    AbsolutePoseInputStateCore *past_input_state(nullptr);
    // Search
    for(std::list< std::shared_ptr<StateCore> >::iterator it_input_state=pastState->TheListInputStateCore.begin();
        it_input_state!=pastState->TheListInputStateCore.end();
        ++it_input_state)
    {
        // TODO
    }
    // Check if it was found
    if(!past_input_state)
        return -5;

    // Past Map Element State
    WorldReferenceFrameStateCore *past_map_element_state(nullptr);
    // Search
    for(std::list< std::shared_ptr<StateCore> >::iterator it_map_element_state=pastState->TheListMapElementStateCore.begin();
        it_map_element_state!=pastState->TheListMapElementStateCore.end();
        ++it_map_element_state)
    {
        if( std::dynamic_pointer_cast<AbsolutePoseInputCore>(past_input_state->getMsfElementCoreSharedPtr())->getWorldReferenceFrameId() == std::dynamic_pointer_cast<WorldReferenceFrameCore>((*it_map_element_state)->getMsfElementCoreSharedPtr())->getId() )
        {
            past_map_element_state=static_cast<WorldReferenceFrameStateCore*>((*it_map_element_state).get());
            break;
        }
    }
    // Check if it was found
    if(!past_map_element_state)
        return -3;


    /// Input Command
    AbsolutePoseInputCommandCore* past_input_command(nullptr);
    // Search
    for(std::list< std::shared_ptr<InputCommandCore> >::iterator it_input_command=inputCommand->TheListInputCommandCore.begin();
        it_input_command!=inputCommand->TheListInputCommandCore.end();
        ++it_input_command)
    {
        // TODO
    }
    // Check if it was found
    if(!past_input_command)
        return -4;




    /// Predict State
    int error_predict_state=predictStateSpecific(previousTimeStamp, currentTimeStamp,
                                                 dynamic_cast<AbsolutePoseDrivenRobotStateCore*>(pastState->TheRobotStateCore.get()),
                                                 past_input_state,
                                                 past_map_element_state,
                                                 past_input_command,
                                                 predicted_robot_state);

    // Check error
    if(error_predict_state)
        return error_predict_state;


    // End
    return 0;
}

int AbsolutePoseDrivenRobotCore::predictStateSpecific(const TimeStamp &previousTimeStamp, const TimeStamp &currentTimeStamp,
                                                      const AbsolutePoseDrivenRobotStateCore *past_robot_state,
                                                      const AbsolutePoseInputStateCore *past_input_state,
                                                      const WorldReferenceFrameStateCore *past_map_element_state,
                                                      const AbsolutePoseInputCommandCore* past_input_command,
                                                      AbsolutePoseDrivenRobotStateCore *&predicted_robot_state)
{

    // Checks in the past state
    if(!past_robot_state->isCorrect())
    {
        std::cout<<"AbsolutePoseDrivenRobotCore::predictStateSpecific() error"<<std::endl;
        return -5;
    }

    // TODO


    // Create the predicted state if it doesn't exist
    if(!predicted_robot_state)
    {
        return -4;
    }



    /// Variables

    // Delta Time
    TimeStamp DeltaTime=currentTimeStamp-previousTimeStamp;
    double dt=DeltaTime.get_double();

    // Past Robot State -> Not needed
    // Eigen::Vector3d position_robot_wrt_world=past_robot_state->getPositionRobotWrtWorld();
    // Eigen::Vector4d attitude_robot_wrt_world=past_robot_state->getAttitudeRobotWrtWorld();

    // Past Input State
    Eigen::Vector3d position_input_wrt_robot=past_input_state->getPositionInputWrtRobot();
    Eigen::Vector4d attitude_input_wrt_robot=past_input_state->getAttitudeInputWrtRobot();

    // Past Map Element State
    Eigen::Vector3d position_input_world_wrt_world=past_map_element_state->getPositionReferenceFrameWorldWrtWorld();
    Eigen::Vector4d attitude_input_world_wrt_world=past_map_element_state->getAttitudeReferenceFrameWorldWrtWorld();

    // Past Input Command
    Eigen::Vector3d position_input_wrt_input_world=past_input_command->getPositionInputWrtInputWorld();
    Eigen::Vector4d attitude_input_wrt_input_world=past_input_command->getAttitudeInputWrtInputWorld();


    /// Equations

    /// Position

    Eigen::Vector4d attitude_robot_wrt_world=Quaternion::cross(attitude_input_world_wrt_world, attitude_input_wrt_input_world, Quaternion::inv(attitude_input_wrt_robot));

    predicted_robot_state->position_robot_wrt_world_=position_input_world_wrt_world+
            Quaternion::cross_sandwich(attitude_input_world_wrt_world, position_input_wrt_input_world, Quaternion::inv(attitude_input_world_wrt_world))-
            Quaternion::cross_sandwich(attitude_robot_wrt_world, position_input_wrt_robot, Quaternion::inv(attitude_robot_wrt_world));


    /// Attitude
    predicted_robot_state->attitude_robot_wrt_world_=Quaternion::cross(attitude_input_world_wrt_world, attitude_input_wrt_input_world, Quaternion::inv(attitude_input_wrt_robot));



    // End
    return 0;
}

// Jacobian
int AbsolutePoseDrivenRobotCore::predictErrorStateJacobian(//Time
                                                 const TimeStamp& previousTimeStamp, const TimeStamp& currentTimeStamp,
                                                 // Previous State
                                                 const std::shared_ptr<StateEstimationCore>& past_state,
                                                  // Inputs
                                                  const std::shared_ptr<InputCommandComponent>& input_command,
                                                 // Predicted State
                                                 std::shared_ptr<StateCore>& predicted_state)
{
    // Checks

    // Past State
    if(!past_state)
        return -1;

    // TODO

    // Predicted State
    if(!predicted_state)
        return -1;


    //// Init Jacobians
    int error_init_jacobians=predictErrorStateJacobianInit(// Past State
                                                           past_state,
                                                           // Input commands
                                                           input_command,
                                                           // Predicted State
                                                           predicted_state);

    if(error_init_jacobians)
        return error_init_jacobians;



    /// Past State

    // Past Robot State
    // Nothing needed

    // Past Input State
    // TODO
    AbsolutePoseInputStateCore *past_input_state;

    // Past Map Element State
    // TODO
    WorldReferenceFrameStateCore *past_map_element_state;


    // Input Command
    // TODO
    AbsolutePoseInputCommandCore* past_input_command;


    /// Robot Predicted State
    AbsolutePoseDrivenRobotStateCore* predicted_robot_state=dynamic_cast<AbsolutePoseDrivenRobotStateCore*>(predicted_state.get());


    /// Get iterators to fill jacobians

    // Fx & Fp

    // Robot
    // Nothing to do

    // Input State
    // TODO
    Eigen::SparseMatrix<double> jacobian_error_state_wrt_input_error_state;
    Eigen::SparseMatrix<double> jacobian_error_state_wrt_input_error_parameters;

    // Map Element State
    // TODO
    Eigen::SparseMatrix<double> jacobian_error_state_wrt_map_element_error_state;
    Eigen::SparseMatrix<double> jacobian_error_state_wrt_map_element_error_parameters;


    // Fu
    // TODO
    Eigen::SparseMatrix<double> jacobian_error_state_wrt_input_command;

    // Fn
    // Nothing to do


    /// Predict State Jacobians
    int error_predict_state_jacobians=predictErrorStateJacobianSpecific(previousTimeStamp, currentTimeStamp,
                                                                        dynamic_cast<AbsolutePoseDrivenRobotStateCore*>(past_state->TheRobotStateCore.get()),
                                                                        past_input_state,
                                                                        past_map_element_state,
                                                                        past_input_command,
                                                                        predicted_robot_state,
                                                                        // Jacobians Error State: Fx, Fp
                                                                        // Robot
                                                                        predicted_robot_state->jacobian_error_state_.robot,
                                                                        predicted_robot_state->jacobian_error_parameters_.robot,
                                                                        // Input
                                                                        jacobian_error_state_wrt_input_error_state,
                                                                        jacobian_error_state_wrt_input_error_parameters,
                                                                        // Map Element
                                                                        jacobian_error_state_wrt_map_element_error_state,
                                                                        jacobian_error_state_wrt_map_element_error_parameters,
                                                                        // Fu
                                                                        jacobian_error_state_wrt_input_command,
                                                                        // Jacobian Error Noise: Fn
                                                                        predicted_robot_state->jacobian_error_state_noise_
                                                                        );

    // Check error
    if(error_predict_state_jacobians)
        return error_predict_state_jacobians;


    // End
    return 0;
}

int AbsolutePoseDrivenRobotCore::predictErrorStateJacobianSpecific(const TimeStamp& previousTimeStamp, const TimeStamp& currentTimeStamp,
                                                                   const AbsolutePoseDrivenRobotStateCore *pastState,
                                                                   const AbsolutePoseInputStateCore *past_input_state,
                                                                   const WorldReferenceFrameStateCore *past_map_element_state,
                                                                   const AbsolutePoseInputCommandCore *past_input_command,
                                                                   AbsolutePoseDrivenRobotStateCore *&predictedState,
                                                                   // Jacobians Error State: Fx, Fp
                                                                   // Robot
                                                                   Eigen::SparseMatrix<double>& jacobian_error_state_wrt_robot_error_state,
                                                                   Eigen::SparseMatrix<double>& jacobian_error_state_wrt_robot_error_parameters,
                                                                   // Input
                                                                   Eigen::SparseMatrix<double> &jacobian_error_state_wrt_input_error_state,
                                                                   Eigen::SparseMatrix<double> &jacobian_error_state_wrt_input_error_parameters,
                                                                   // Map Element
                                                                   Eigen::SparseMatrix<double> &jacobian_error_state_wrt_map_element_error_state,
                                                                   Eigen::SparseMatrix<double> &jacobian_error_state_wrt_map_element_error_parameters,
                                                                   // Fu
                                                                   Eigen::SparseMatrix<double> &jacobian_error_state_wrt_input_command,
                                                                   // Jacobians Noise: Fn
                                                                   Eigen::SparseMatrix<double>& jacobian_error_state_wrt_noise
                                                                   )
{

    // Create the predicted state if it doesn't exist
    if(!predictedState)
    {
        std::cout<<"AbsolutePoseDrivenRobotCore::predictErrorStateJacobians error en predictedState"<<std::endl;
        return 1;
    }



    /// Variables

    // Delta Time
    TimeStamp DeltaTime=currentTimeStamp-previousTimeStamp;
    // delta time
    double dt=DeltaTime.get_double();

    // Past Robot State -> Not needed
    // Eigen::Vector3d position_robot_wrt_world=past_robot_state->getPositionRobotWrtWorld();
    // Eigen::Vector4d attitude_robot_wrt_world=past_robot_state->getAttitudeRobotWrtWorld();

    // Past Input State
    Eigen::Vector3d position_input_wrt_robot=past_input_state->getPositionInputWrtRobot();
    Eigen::Vector4d attitude_input_wrt_robot=past_input_state->getAttitudeInputWrtRobot();

    // Past Map Element State
    Eigen::Vector3d position_input_world_wrt_world=past_map_element_state->getPositionReferenceFrameWorldWrtWorld();
    Eigen::Vector4d attitude_input_world_wrt_world=past_map_element_state->getAttitudeReferenceFrameWorldWrtWorld();

    // Past Input Command
    Eigen::Vector3d position_input_wrt_input_world=past_input_command->getPositionInputWrtInputWorld();
    Eigen::Vector4d attitude_input_wrt_input_world=past_input_command->getAttitudeInputWrtInputWorld();




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
        // TODO
    }

    /// Sensors
    {
        // Nothing to do
    }

    /// Map Elements
    {
        // TODO
    }



    //// Jacobian Error State - Error Input

    {
        // TODO
    }


    //// Jacobian Error State - Noise Estimation: Fn

    {
        // Resize and init
        jacobian_error_state_wrt_noise.resize(dimension_error_state_, dimension_noise_);


        std::vector<Eigen::Triplet<double> > tripletListNoiseJacobian;

        int dimension_state_i=0;
        int dimension_noise_i=0;


        // Fill

        // Position
        {
            for(int i=0; i<3; i++)
                tripletListNoiseJacobian.push_back(Eigen::Triplet<double>(dimension_state_i+i,dimension_noise_i+i,1));
            dimension_state_i+=3;
            dimension_noise_i+=3;
        }

        // Attitude
        {
            for(int i=0; i<3; i++)
                tripletListNoiseJacobian.push_back(Eigen::Triplet<double>(dimension_state_i+i,dimension_noise_i+i,1));
            dimension_state_i+=3;
            dimension_noise_i+=3;
        }


        // Set From Triplets
        jacobian_error_state_wrt_noise.setFromTriplets(tripletListNoiseJacobian.begin(), tripletListNoiseJacobian.end());

    }


    // End
    return 0;
}

int AbsolutePoseDrivenRobotCore::resetErrorStateJacobian(// Time
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

    // Position
    {
        for(int i=0; i<3; i++)
            triplets_jacobian_error_reset.push_back(Eigen::Triplet<double>(dimension_error_state_i+i, dimension_error_state_i+i, 1.0));

        dimension_error_state_i+=3;
    }

    // Attitude
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


