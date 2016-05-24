
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


    /// Inputs
    readingValue=robot.child_value("input_list");
    {
        std::istringstream stm(readingValue);
        std::list<int> input_ids;
        int read_id;
        while(stm>>read_id)
        {
            input_ids.push_back(read_id);
        }
        this->setInputIds(input_ids);
        // Check inputs
        if( this->getNumInputs() > 1 || this->getNumInputs() < 0 )
        {
            std::cout<<"ERROR!! Too many or few inputs defined!"<<std::endl;
            return -1;
        }
    }



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
                                             const std::shared_ptr<StateEstimationCore> &past_state,
                                             // Inputs
                                             const std::shared_ptr<InputCommandComponent> &input_command,
                                             // Predicted State
                                             std::shared_ptr<StateCore> &predicted_state)
{
    /// Checks

    // Past State
    if(!past_state)
        return -1;

    // TODO



    /// Robot Predicted State
    AbsolutePoseDrivenRobotStateCore* predicted_robot_state(nullptr);
    if(!predicted_state)
    {
        predicted_robot_state=new AbsolutePoseDrivenRobotStateCore;
        predicted_robot_state->setMsfElementCorePtr(past_state->TheRobotStateCore->getMsfElementCoreWeakPtr());
        predicted_state=std::shared_ptr<StateCore>(predicted_robot_state);
    }
    else
        predicted_robot_state=dynamic_cast<AbsolutePoseDrivenRobotStateCore*>(predicted_state.get());



    /// Past State

    // Past Robot State
    // Nothing needed

    // Past Input State
    AbsolutePoseInputStateCore *past_input_state(nullptr);
    // Search
    for(std::list< std::shared_ptr<StateCore> >::iterator it_input_state=past_state->TheListInputStateCore.begin();
        it_input_state!=past_state->TheListInputStateCore.end();
        ++it_input_state)
    {
        if( this->getInputIdI(0) == std::dynamic_pointer_cast<InputCore>((*it_input_state)->getMsfElementCoreSharedPtr())->getInputId() )
        {
            past_input_state=dynamic_cast<AbsolutePoseInputStateCore*>((*it_input_state).get());
            break;
        }
    }
    // Check if it was found
    if(!past_input_state)
        return -5;

    // Past Map Element State
    WorldReferenceFrameStateCore *past_map_element_state(nullptr);
    // Search
    for(std::list< std::shared_ptr<StateCore> >::iterator it_map_element_state=past_state->TheListMapElementStateCore.begin();
        it_map_element_state!=past_state->TheListMapElementStateCore.end();
        ++it_map_element_state)
    {
        if(std::dynamic_pointer_cast<MapElementCore>((*it_map_element_state)->getMsfElementCoreSharedPtr())->getMapElementType() == MapElementTypes::world_ref_frame )
        {
            if( std::dynamic_pointer_cast<AbsolutePoseInputCore>(past_input_state->getMsfElementCoreSharedPtr())->getWorldReferenceFrameId() == std::dynamic_pointer_cast<WorldReferenceFrameCore>((*it_map_element_state)->getMsfElementCoreSharedPtr())->getId() )
            {
                past_map_element_state=static_cast<WorldReferenceFrameStateCore*>((*it_map_element_state).get());
                break;
            }
        }
    }
    // Check if it was found
    if(!past_map_element_state)
        return -3;


    /// Input Command
    AbsolutePoseInputCommandCore* past_input_command(nullptr);
    // Search
    for(std::list< std::shared_ptr<InputCommandCore> >::iterator it_input_command=input_command->TheListInputCommandCore.begin();
        it_input_command!=input_command->TheListInputCommandCore.end();
        ++it_input_command)
        {
            if( this->getInputIdI(0) == std::dynamic_pointer_cast<InputCore>((*it_input_command)->getInputCoreSharedPtr())->getInputId() )
            {
                past_input_command=dynamic_cast<AbsolutePoseInputCommandCore*>((*it_input_command).get());
                break;
            }
        }
    // Check if it was found
    if(!past_input_command)
        return -4;




    /// Predict State
    int error_predict_state=predictStateSpecific(previousTimeStamp, currentTimeStamp,
                                                 dynamic_cast<AbsolutePoseDrivenRobotStateCore*>(past_state->TheRobotStateCore.get()),
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
    AbsolutePoseInputStateCore *past_input_state(nullptr);
    // Search
    for(std::list< std::shared_ptr<StateCore> >::iterator it_input_state=past_state->TheListInputStateCore.begin();
        it_input_state!=past_state->TheListInputStateCore.end();
        ++it_input_state)
    {
        if( this->getInputIdI(0) == std::dynamic_pointer_cast<InputCore>((*it_input_state)->getMsfElementCoreSharedPtr())->getInputId() )
        {
            past_input_state=dynamic_cast<AbsolutePoseInputStateCore*>((*it_input_state).get());
            break;
        }
    }
    // Check if it was found
    if(!past_input_state)
        return -5;

    // Past Map Element State
    WorldReferenceFrameStateCore *past_map_element_state(nullptr);
    // Search
    for(std::list< std::shared_ptr<StateCore> >::iterator it_map_element_state=past_state->TheListMapElementStateCore.begin();
        it_map_element_state!=past_state->TheListMapElementStateCore.end();
        ++it_map_element_state)
    {
        if( std::dynamic_pointer_cast<MapElementCore>((*it_map_element_state)->getMsfElementCoreSharedPtr())->getMapElementType() == MapElementTypes::world_ref_frame )
        {
            if( std::dynamic_pointer_cast<AbsolutePoseInputCore>(past_input_state->getMsfElementCoreSharedPtr())->getWorldReferenceFrameId() == std::dynamic_pointer_cast<WorldReferenceFrameCore>((*it_map_element_state)->getMsfElementCoreSharedPtr())->getId() )
            {
                past_map_element_state=static_cast<WorldReferenceFrameStateCore*>((*it_map_element_state).get());
                break;
            }
        }
    }
    // Check if it was found
    if(!past_map_element_state)
        return -3;


    /// Input Command
    AbsolutePoseInputCommandCore* past_input_command(nullptr);
    // Search
    for(std::list< std::shared_ptr<InputCommandCore> >::iterator it_input_command=input_command->TheListInputCommandCore.begin();
        it_input_command!=input_command->TheListInputCommandCore.end();
        ++it_input_command)
        {
            if( this->getInputIdI(0) == std::dynamic_pointer_cast<InputCore>((*it_input_command)->getInputCoreSharedPtr())->getInputId() )
            {
                past_input_command=dynamic_cast<AbsolutePoseInputCommandCore*>((*it_input_command).get());
                break;
            }
        }
    // Check if it was found
    if(!past_input_command)
        return -4;


    /// Robot Predicted State
    AbsolutePoseDrivenRobotStateCore* predicted_robot_state=dynamic_cast<AbsolutePoseDrivenRobotStateCore*>(predicted_state.get());


    /// Get iterators to fill jacobians

    // Fx & Fp

    // Robot
    // Nothing to do

    // Input State
    std::vector<Eigen::SparseMatrix<double> >::iterator it_jacobian_error_state_wrt_input_error_state;
    it_jacobian_error_state_wrt_input_error_state=predicted_robot_state->jacobian_error_state_.inputs.begin();
    std::vector<Eigen::SparseMatrix<double> >::iterator it_jacobian_error_state_wrt_input_error_parameters;
    it_jacobian_error_state_wrt_input_error_parameters=predicted_robot_state->jacobian_error_parameters_.inputs.begin();
    for(std::list< std::shared_ptr<StateCore> >::iterator itInputStateCore=past_state->TheListInputStateCore.begin();
        itInputStateCore!=past_state->TheListInputStateCore.end();
        ++itInputStateCore, ++it_jacobian_error_state_wrt_input_error_state, ++it_jacobian_error_state_wrt_input_error_parameters
        )
    {
        if( dynamic_cast<AbsolutePoseInputStateCore*>((*itInputStateCore).get()) == past_input_state )
            break;
    }

    // Map Element State
    std::vector<Eigen::SparseMatrix<double> >::iterator it_jacobian_error_state_wrt_map_element_error_state;
    it_jacobian_error_state_wrt_map_element_error_state=predicted_robot_state->jacobian_error_state_.map_elements.begin();
    std::vector<Eigen::SparseMatrix<double> >::iterator it_jacobian_error_state_wrt_map_element_error_parameters;
    it_jacobian_error_state_wrt_map_element_error_parameters=predicted_robot_state->jacobian_error_parameters_.map_elements.begin();
    for(std::list< std::shared_ptr<StateCore> >::iterator itMapElementStateCore=past_state->TheListMapElementStateCore.begin();
        itMapElementStateCore!=past_state->TheListMapElementStateCore.end();
        ++itMapElementStateCore, ++it_jacobian_error_state_wrt_map_element_error_state, ++it_jacobian_error_state_wrt_map_element_error_parameters
        )
    {
        if( dynamic_cast<WorldReferenceFrameStateCore*>((*itMapElementStateCore).get()) == past_map_element_state )
            break;
    }

    // Fu
    std::vector<Eigen::SparseMatrix<double> >::iterator it_jacobian_error_state_wrt_input_command;
    it_jacobian_error_state_wrt_input_command=predicted_robot_state->jacobian_error_input_commands_.input_commands.begin();
    for(std::list< std::shared_ptr<InputCommandCore> >::iterator itInputCommandCore=input_command->TheListInputCommandCore.begin();
        itInputCommandCore!=input_command->TheListInputCommandCore.end();
        ++itInputCommandCore, ++it_jacobian_error_state_wrt_input_command
        )
    {
        if( dynamic_cast<AbsolutePoseInputCommandCore*>((*itInputCommandCore).get()) == past_input_command )
            break;
    }

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
                                                                        (*it_jacobian_error_state_wrt_input_error_state),
                                                                        (*it_jacobian_error_state_wrt_input_error_parameters),
                                                                        // Map Element
                                                                        (*it_jacobian_error_state_wrt_map_element_error_state),
                                                                        (*it_jacobian_error_state_wrt_map_element_error_parameters),
                                                                        // Fu
                                                                        (*it_jacobian_error_state_wrt_input_command),
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
                                                                   const AbsolutePoseDrivenRobotStateCore *past_robot_state,
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
        return -1;
    }



    /// Variables

    // Delta Time
    TimeStamp DeltaTime=currentTimeStamp-previousTimeStamp;
    // delta time
    double dt=DeltaTime.get_double();

    // Past Robot State -> Not needed
    Eigen::Vector3d position_robot_wrt_world=past_robot_state->getPositionRobotWrtWorld();
    Eigen::Vector4d attitude_robot_wrt_world=past_robot_state->getAttitudeRobotWrtWorld();

    // Past Input State
    Eigen::Vector3d position_input_wrt_robot=past_input_state->getPositionInputWrtRobot();
    Eigen::Vector4d attitude_input_wrt_robot=past_input_state->getAttitudeInputWrtRobot();

    // Past Map Element State
    Eigen::Vector3d position_input_world_wrt_world=past_map_element_state->getPositionReferenceFrameWorldWrtWorld();
    Eigen::Vector4d attitude_input_world_wrt_world=past_map_element_state->getAttitudeReferenceFrameWorldWrtWorld();

    // Past Input Command
    Eigen::Vector3d position_input_wrt_input_world=past_input_command->getPositionInputWrtInputWorld();
    Eigen::Vector4d attitude_input_wrt_input_world=past_input_command->getAttitudeInputWrtInputWorld();


    // State k+1
    // Robot
    Eigen::Vector3d estim_position_robot_wrt_world=predictedState->getPositionRobotWrtWorld();
    Eigen::Vector4d estim_attitude_robot_wrt_world=predictedState->getAttitudeRobotWrtWorld();



    // Jacobians Error State: Fx, Fp
    // Input
    Eigen::MatrixXd jacobian_error_state_robot_pos_wrt_input_error_state_pos;
    Eigen::MatrixXd jacobian_error_state_robot_pos_wrt_input_error_state_att;
    Eigen::MatrixXd jacobian_error_state_robot_att_wrt_input_error_state_att;
    // Map Element
    Eigen::MatrixXd jacobian_error_state_robot_pos_wrt_map_element_error_state_pos;
    Eigen::MatrixXd jacobian_error_state_robot_pos_wrt_map_element_error_state_att;
    Eigen::MatrixXd jacobian_error_state_robot_att_wrt_map_element_error_state_att;
    // Jacobian Input: Fu
    Eigen::MatrixXd jacobian_error_state_robot_pos_wrt_input_command_pos;
    Eigen::MatrixXd jacobian_error_state_robot_pos_wrt_input_command_att;
    Eigen::MatrixXd jacobian_error_state_robot_att_wrt_input_command_att;
    // Jacobians Noise: Fn
    Eigen::MatrixXd jacobian_error_state_pos_wrt_noise_pos;
    Eigen::MatrixXd jacobian_error_state_att_wrt_noise_att;



    /// Core

    int error_predict_jacobian_error_state_core=predictErrorStateJacobianSpecificCore(
                                                                                    // State k
                                                                                    // Robot
                                                                                    position_robot_wrt_world,
                                                                                    attitude_robot_wrt_world,
                                                                                    // Input State
                                                                                    position_input_wrt_robot,
                                                                                    attitude_input_wrt_robot,
                                                                                    // Map Element State
                                                                                    position_input_world_wrt_world,
                                                                                    attitude_input_world_wrt_world,
                                                                                    // Input Command
                                                                                    position_input_wrt_input_world,
                                                                                    attitude_input_wrt_input_world,
                                                                                    // State k+1
                                                                                    estim_position_robot_wrt_world,
                                                                                    estim_attitude_robot_wrt_world,
                                                                                  // Jacobians Error State: Fx, Fp
                                                                                  // Input
                                                                                  jacobian_error_state_robot_pos_wrt_input_error_state_pos,
                                                                                  jacobian_error_state_robot_pos_wrt_input_error_state_att,
                                                                                  jacobian_error_state_robot_att_wrt_input_error_state_att,
                                                                                  // Map Element
                                                                                  jacobian_error_state_robot_pos_wrt_map_element_error_state_pos,
                                                                                  jacobian_error_state_robot_pos_wrt_map_element_error_state_att,
                                                                                  jacobian_error_state_robot_att_wrt_map_element_error_state_att,
                                                                                  // Jacobian Input: Fu
                                                                                  jacobian_error_state_robot_pos_wrt_input_command_pos,
                                                                                  jacobian_error_state_robot_pos_wrt_input_command_att,
                                                                                  jacobian_error_state_robot_att_wrt_input_command_att,
                                                                                  // Jacobians Noise: Fn
                                                                                  jacobian_error_state_pos_wrt_noise_pos,
                                                                                  jacobian_error_state_att_wrt_noise_att);

    if(error_predict_jacobian_error_state_core)
        return error_predict_jacobian_error_state_core;



    ///// Jacobian Error State - Error State: Fx & Jacobian Error State - Error Parameters: Fp

    /// World
    {
        // Nothing to do
    }

    /// Robot
    {
        // Nothing to do
        // jacobian_error_state_wrt_robot_error_state;
        // jacobian_error_state_wrt_robot_error_parameters;
    }

    /// Inputs
    {

        // Init and resize
        jacobian_error_state_wrt_input_error_state.resize(this->getDimensionErrorState(), past_input_state->getMsfElementCoreSharedPtr()->getDimensionErrorState());
        jacobian_error_state_wrt_input_error_parameters.resize(this->getDimensionErrorState(), past_input_state->getMsfElementCoreSharedPtr()->getDimensionErrorParameters());

        std::vector<Eigen::Triplet<double>> triplet_list_jacobian_error_state_wrt_error_state;
        std::vector<Eigen::Triplet<double>> triplet_list_jacobian_error_state_wrt_error_parameters;

        // Input Core
        std::shared_ptr<AbsolutePoseInputCore> input_core=std::dynamic_pointer_cast<AbsolutePoseInputCore>(past_input_state->getMsfElementCoreSharedPtr());


        // Fill
        int dimension_error_state_i=0;
        int dimension_error_parameters_i=0;

        // Robot Position
        {
            int dimension_error_state_j=0;
            int dimension_error_parameters_j=0;

            // Input Position
            if(input_core->isEstimationPositionInputWrtRobotEnabled())
            {
                BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_state_wrt_error_state, jacobian_error_state_robot_pos_wrt_input_error_state_pos, dimension_error_state_i, dimension_error_state_j);
                dimension_error_state_j+=3;
            }
            else
            {
                BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_state_wrt_error_parameters, jacobian_error_state_robot_pos_wrt_input_error_state_pos, dimension_error_state_i, dimension_error_parameters_j);
                dimension_error_parameters_j+=3;
            }

            // Input Attitude
            if(input_core->isEstimationAttitudeInputWrtRobotEnabled())
            {
                BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_state_wrt_error_state, jacobian_error_state_robot_pos_wrt_input_error_state_att, dimension_error_state_i, dimension_error_state_j);
                dimension_error_state_j+=3;
            }
            else
            {
                BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_state_wrt_error_parameters, jacobian_error_state_robot_pos_wrt_input_error_state_att, dimension_error_state_i, dimension_error_parameters_j);
                dimension_error_parameters_j+=3;
            }

            dimension_error_state_i+=3;
        }

        // Robot Attitude
        {
            int dimension_error_state_j=0;
            int dimension_error_parameters_j=0;

            // Input Position
            if(input_core->isEstimationPositionInputWrtRobotEnabled())
            {
                // Nothing to do
                dimension_error_state_j+=3;
            }
            else
            {
                // Nothing to do
                dimension_error_parameters_j+=3;
            }


            // Input Attitude
            if(input_core->isEstimationAttitudeInputWrtRobotEnabled())
            {
                BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_state_wrt_error_state, jacobian_error_state_robot_att_wrt_input_error_state_att, dimension_error_state_i, dimension_error_state_j);
                dimension_error_state_j+=3;
            }
            else
            {
                BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_state_wrt_error_parameters, jacobian_error_state_robot_att_wrt_input_error_state_att, dimension_error_state_i, dimension_error_parameters_j);
                dimension_error_parameters_j+=3;
            }

            dimension_error_state_i+=3;
        }


        // Set From Triplets
        jacobian_error_state_wrt_input_error_state.setFromTriplets(triplet_list_jacobian_error_state_wrt_error_state.begin(), triplet_list_jacobian_error_state_wrt_error_state.end());
        jacobian_error_state_wrt_input_error_parameters.setFromTriplets(triplet_list_jacobian_error_state_wrt_error_parameters.begin(), triplet_list_jacobian_error_state_wrt_error_parameters.end());

    }

    /// Sensors
    {
        // Nothing to do
    }

    /// Map Elements
    {
        // Init and resize
        jacobian_error_state_wrt_map_element_error_state.resize(this->getDimensionErrorState(), past_map_element_state->getMsfElementCoreSharedPtr()->getDimensionErrorState());
        jacobian_error_state_wrt_map_element_error_parameters.resize(this->getDimensionErrorState(), past_map_element_state->getMsfElementCoreSharedPtr()->getDimensionErrorParameters());

        std::vector<Eigen::Triplet<double>> triplet_list_jacobian_error_state_wrt_error_state;
        std::vector<Eigen::Triplet<double>> triplet_list_jacobian_error_state_wrt_error_parameters;


        // Map Element Core
        std::shared_ptr<WorldReferenceFrameCore> map_element_core=std::dynamic_pointer_cast<WorldReferenceFrameCore>(past_map_element_state->getMsfElementCoreSharedPtr());


        // Fill
        int dimension_error_state_i=0;
        int dimension_error_parameters_i=0;

        // Robot Position
        {
            int dimension_error_state_j=0;
            int dimension_error_parameters_j=0;

            // Map Element Position
            if(map_element_core->isEstimationPositionWorldReferenceFrameWrtWorldEnabled())
            {
                BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_state_wrt_error_state, jacobian_error_state_robot_pos_wrt_map_element_error_state_pos, dimension_error_state_i, dimension_error_state_j);
                dimension_error_state_j+=3;
            }
            else
            {
                BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_state_wrt_error_parameters, jacobian_error_state_robot_pos_wrt_map_element_error_state_pos, dimension_error_state_i, dimension_error_parameters_j);
                dimension_error_parameters_j+=3;
            }
            // Map Element Attitude
            if(map_element_core->isEstimationAttitudeWorldReferenceFrameWrtWorldEnabled())
            {
                BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_state_wrt_error_state, jacobian_error_state_robot_pos_wrt_map_element_error_state_att, dimension_error_state_i, dimension_error_state_j);
                dimension_error_state_j+=3;
            }
            else
            {
                BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_state_wrt_error_parameters, jacobian_error_state_robot_pos_wrt_map_element_error_state_att, dimension_error_state_i, dimension_error_parameters_j);
                dimension_error_parameters_j+=3;
            }

            dimension_error_state_i+=3;
        }

        // Robot Attitude
        {
            int dimension_error_state_j=0;
            int dimension_error_parameters_j=0;

            // Map Element Position
            if(map_element_core->isEstimationPositionWorldReferenceFrameWrtWorldEnabled())
            {
                // Nothing to do
                dimension_error_state_j+=3;
            }
            else
            {
                // Nothing to do
                dimension_error_parameters_j+=3;
            }


            // Map Element Attitude
            if(map_element_core->isEstimationAttitudeWorldReferenceFrameWrtWorldEnabled())
            {
                BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_state_wrt_error_state, jacobian_error_state_robot_att_wrt_map_element_error_state_att, dimension_error_state_i, dimension_error_state_j);
                dimension_error_state_j+=3;
            }
            else
            {
                BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_state_wrt_error_parameters, jacobian_error_state_robot_att_wrt_map_element_error_state_att, dimension_error_state_i, dimension_error_parameters_j);
                dimension_error_parameters_j+=3;
            }

            dimension_error_state_i+=3;
        }

        // Set From Triplets
        jacobian_error_state_wrt_map_element_error_state.setFromTriplets(triplet_list_jacobian_error_state_wrt_error_state.begin(), triplet_list_jacobian_error_state_wrt_error_state.end());
        jacobian_error_state_wrt_map_element_error_parameters.setFromTriplets(triplet_list_jacobian_error_state_wrt_error_parameters.begin(), triplet_list_jacobian_error_state_wrt_error_parameters.end());

    }


    //// Jacobian Error State - Error Input Command

    {
        // Init and resize
        jacobian_error_state_wrt_input_command.resize(this->getDimensionErrorState(), past_input_command->getInputCoreSharedPtr()->getDimensionErrorInputCommand());

        std::vector<Eigen::Triplet<double>> triplet_list_jacobian_error_state_wrt_error_input_command;

        // Input Core
        std::shared_ptr<AbsolutePoseInputCore> input_core=std::dynamic_pointer_cast<AbsolutePoseInputCore>(past_input_command->getInputCoreSharedPtr());


        // Fill
        int dimension_error_state_i=0;
        int dimension_error_parameters_i=0;

        // Robot Position
        {
            int dimension_error_input_command_j=0;

            // Input Command Position
            if(input_core->isInputCommandPositionInputWrtInputWorldEnabled())
            {
                BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_state_wrt_error_input_command, jacobian_error_state_robot_pos_wrt_input_command_pos, dimension_error_state_i, dimension_error_input_command_j);
                dimension_error_input_command_j+=3;
            }

            // Input Command Attitude
            if(input_core->isInputCommandAttitudeInputWrtInputWorldEnabled())
            {
                BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_state_wrt_error_input_command, jacobian_error_state_robot_pos_wrt_input_command_att, dimension_error_state_i, dimension_error_input_command_j);
                dimension_error_input_command_j+=3;
            }

            dimension_error_state_i+=3;
        }

        // Robot Attitude
        {
            int dimension_error_input_command_j=0;

            // Input Command Position
            if(input_core->isInputCommandPositionInputWrtInputWorldEnabled())
            {
                // Zeros
                dimension_error_input_command_j+=3;
            }


            // Input Command Attitude
            if(input_core->isInputCommandAttitudeInputWrtInputWorldEnabled())
            {
                BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_state_wrt_error_input_command, jacobian_error_state_robot_att_wrt_input_command_att, dimension_error_state_i, dimension_error_input_command_j);
                dimension_error_input_command_j+=3;
            }

            dimension_error_state_i+=3;
        }


        // Set From Triplets
        jacobian_error_state_wrt_input_command.setFromTriplets(triplet_list_jacobian_error_state_wrt_error_input_command.begin(), triplet_list_jacobian_error_state_wrt_error_input_command.end());

    }


    //// Jacobian Error State - Noise Estimation: Fn

    {
        // Resize and init
        jacobian_error_state_wrt_noise.resize(dimension_error_state_, dimension_noise_);

        std::vector<Eigen::Triplet<double> > triplet_list_jacobian_error_state_wrt_noise_error_state;


        // Fill
        int dimension_error_state_i=0;
        int dimension_error_noise_j=0;

        // Robot State Position
        {
            BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_state_wrt_noise_error_state, jacobian_error_state_pos_wrt_noise_pos, dimension_error_state_i, dimension_error_noise_j);
            dimension_error_state_i+=3;
            dimension_error_noise_j+=3;
        }

        // Robot State Attitude
        {
            BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_state_wrt_noise_error_state, jacobian_error_state_att_wrt_noise_att, dimension_error_state_i, dimension_error_noise_j);
            dimension_error_state_i+=3;
            dimension_error_noise_j+=3;
        }


        // Set From Triplets
        jacobian_error_state_wrt_noise.setFromTriplets(triplet_list_jacobian_error_state_wrt_noise_error_state.begin(), triplet_list_jacobian_error_state_wrt_noise_error_state.end());

    }


    // End
    return 0;
}

int AbsolutePoseDrivenRobotCore::predictErrorStateJacobianSpecificCore(// State k
                                                                      // Robot
                                                                      const Eigen::Vector3d& position_robot_wrt_world,
                                                                      const Eigen::Vector4d& attitude_robot_wrt_world,
                                                                      // Input State
                                                                      const Eigen::Vector3d& position_input_wrt_robot,
                                                                      const Eigen::Vector4d& attitude_input_wrt_robot,
                                                                      // Map Element State
                                                                      const Eigen::Vector3d& position_input_world_wrt_world,
                                                                      const Eigen::Vector4d& attitude_input_world_wrt_world,
                                                                      // Input Command
                                                                      const Eigen::Vector3d& position_input_wrt_input_world,
                                                                      const Eigen::Vector4d& attitude_input_wrt_input_world,
                                                                      // State k+1
                                                                       const Eigen::Vector3d& estim_position_robot_wrt_world,
                                                                       const Eigen::Vector4d& estim_attitude_robot_wrt_world,
                                                                      // Jacobians Error State: Fx, Fp
                                                                       // Input
                                                                       Eigen::MatrixXd &jacobian_error_state_robot_pos_wrt_input_error_state_pos,
                                                                       Eigen::MatrixXd &jacobian_error_state_robot_pos_wrt_input_error_state_att,
                                                                       Eigen::MatrixXd &jacobian_error_state_robot_att_wrt_input_error_state_att,
                                                                       // Map Element
                                                                       Eigen::MatrixXd &jacobian_error_state_robot_pos_wrt_map_element_error_state_pos,
                                                                       Eigen::MatrixXd &jacobian_error_state_robot_pos_wrt_map_element_error_state_att,
                                                                       Eigen::MatrixXd &jacobian_error_state_robot_att_wrt_map_element_error_state_att,
                                                                       // Jacobian Input: Fu
                                                                       Eigen::MatrixXd &jacobian_error_state_robot_pos_wrt_input_command_pos,
                                                                       Eigen::MatrixXd &jacobian_error_state_robot_pos_wrt_input_command_att,
                                                                       Eigen::MatrixXd &jacobian_error_state_robot_att_wrt_input_command_att,
                                                                       // Jacobians Noise: Fn
                                                                       Eigen::MatrixXd& jacobian_error_state_pos_wrt_noise_pos,
                                                                       Eigen::MatrixXd& jacobian_error_state_att_wrt_noise_att
                                                                      )
{
    /// Auxiliar Vars
    // TODO

    Eigen::Vector4d attitude_estim_world_wrt_robot=Quaternion::inv(estim_attitude_robot_wrt_world);
    Eigen::Vector4d attitude_robot_wrt_input=Quaternion::inv(attitude_input_wrt_robot);
    Eigen::Vector4d attitude_world_wrt_input_world=Quaternion::inv(attitude_input_world_wrt_world);
    Eigen::Vector4d attitude_input_wrt_world=Quaternion::cross(attitude_input_world_wrt_world, attitude_input_wrt_input_world);
    Eigen::Vector4d attitude_world_wrt_input=Quaternion::inv(attitude_input_wrt_world);

    Eigen::Vector3d position_robot_wrt_input=-Quaternion::cross_sandwich(Quaternion::inv(attitude_input_wrt_robot), position_input_wrt_robot, attitude_input_wrt_robot);

    Eigen::Vector4d vec_aux_cross_attitude_input_wrt_world_and_position_robot_wrt_input=Quaternion::cross_gen_pure(attitude_input_wrt_world, position_robot_wrt_input);
    Eigen::Vector4d vec_aux_cross_attitude_world_wrt_input_and_position_robot_wrt_input=Quaternion::cross_pure_gen(position_robot_wrt_input, attitude_world_wrt_input);

    // attitude_estim_world_wrt_robot
    Eigen::Matrix4d quat_mat_minus_attitude_estim_world_wrt_robot=Quaternion::quatMatMinus(attitude_estim_world_wrt_robot);

    // attitude_input_world_wrt_world
    Eigen::Matrix4d quat_mat_plus_attitude_input_world_wrt_world=Quaternion::quatMatPlus(attitude_input_world_wrt_world);

    // attitude_world_wrt_input_world
    Eigen::Matrix4d quat_mat_minus_attitude_world_wrt_input_world=Quaternion::quatMatMinus(attitude_world_wrt_input_world);

    // attitude_robot_wrt_input
    Eigen::Matrix4d quat_mat_minus_attitude_robot_wrt_input=Quaternion::quatMatMinus(attitude_robot_wrt_input);

    // attitude_input_wrt_input_world
    Eigen::Matrix4d quat_mat_plus_attitude_input_wrt_input_world=Quaternion::quatMatPlus(attitude_input_wrt_input_world);

    // Auxs
    Eigen::Matrix4d aux1=Quaternion::quatMatPlus(vec_aux_cross_attitude_input_wrt_world_and_position_robot_wrt_input);
    Eigen::Matrix4d aux2=Quaternion::quatMatMinus(vec_aux_cross_attitude_world_wrt_input_and_position_robot_wrt_input);


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


    /// Jacobians Error State: Fx, Fp

    // Input
    // TODO
    jacobian_error_state_robot_pos_wrt_input_error_state_pos;
    jacobian_error_state_robot_pos_wrt_input_error_state_att;
    jacobian_error_state_robot_att_wrt_input_error_state_att;

    // Map Element
    // TODO
    jacobian_error_state_robot_pos_wrt_map_element_error_state_pos;
    jacobian_error_state_robot_pos_wrt_map_element_error_state_att;
    jacobian_error_state_robot_att_wrt_map_element_error_state_att;


    /// Jacobian Input: Fu
    jacobian_error_state_robot_pos_wrt_input_command_pos=
            mat_diff_vector_wrt_vector_amp*quat_mat_plus_attitude_input_world_wrt_world*quat_mat_minus_attitude_world_wrt_input_world*mat_diff_vector_wrt_vector_amp.transpose();

    jacobian_error_state_robot_pos_wrt_input_command_att=
            mat_diff_vector_wrt_vector_amp*(aux1*mat_diff_quat_inv_wrt_quat+aux2)*quat_mat_plus_attitude_input_world_wrt_world*quat_mat_plus_attitude_input_wrt_input_world*0.5*mat_diff_error_quat_wrt_error_theta;

    jacobian_error_state_robot_att_wrt_input_command_att=
            mat_diff_error_quat_wrt_error_theta.transpose()*quat_mat_minus_attitude_estim_world_wrt_robot*quat_mat_plus_attitude_input_world_wrt_world*quat_mat_minus_attitude_robot_wrt_input*quat_mat_plus_attitude_input_wrt_input_world*mat_diff_error_quat_wrt_error_theta;


    /// Jacobians Noise: Fn
    // TODO Fix!
    jacobian_error_state_pos_wrt_noise_pos=Eigen::MatrixXd::Identity(3,3);
    jacobian_error_state_att_wrt_noise_att=Eigen::MatrixXd::Identity(3,3);

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


