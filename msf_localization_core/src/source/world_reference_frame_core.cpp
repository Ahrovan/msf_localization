
#include "msf_localization_core/world_reference_frame_core.h"

#include "msf_localization_core/map_element_state_core.h"
#include "msf_localization_core/world_reference_frame_state_core.h"


WorldReferenceFrameCore::WorldReferenceFrameCore():
    MapElementCore()
{
    init();

    return;
}

WorldReferenceFrameCore::WorldReferenceFrameCore(std::weak_ptr<MsfStorageCore> TheMsfStorageCore) :
    MapElementCore(TheMsfStorageCore)
{
    init();

    return;
}

WorldReferenceFrameCore::~WorldReferenceFrameCore()
{
    return;
}

int WorldReferenceFrameCore::init()
{
    // Dimensions
    dimension_state_=0;
    dimension_error_state_=0;

    dimension_parameters_+=3+4;
    dimension_error_parameters_+=3+3;

    // Init values
    id_=-1;

    // Flags
    flag_estimation_position_mocap_world_wrt_world_=false;
    flag_estimation_attitude_mocap_world_wrt_world_=false;

    // Noises
    covariance_position_mocap_world_wrt_world_.setZero();
    covariance_attitude_mocap_world_wrt_world_.setZero();


    // Map Type
    setMapElementType(MapElementTypes::world_ref_frame);


    return 0;
}

int WorldReferenceFrameCore::readConfig(const pugi::xml_node& map_element, std::shared_ptr<WorldReferenceFrameStateCore>& MapElementInitStateCore)
{
    // Create a class for the SensorStateCore
    if(!MapElementInitStateCore)
        MapElementInitStateCore=std::make_shared<WorldReferenceFrameStateCore>(this->getMsfElementCoreSharedPtr());


    // Auxiliar reading value
    std::string readingValue;



    /// Name
    std::string map_element_name=map_element.child_value("name");
    this->setMapElementName(map_element_name);


    /// Id
    readingValue=map_element.child_value("id");
    if(!readingValue.empty())
    {
        this->setId(std::stoi(readingValue));
    }


    //// Configs

    /// Pose of the map element wrt worl
    pugi::xml_node pose_in_world=map_element.child("pose_in_world");

    // Position of the sensor wrt robot
    readingValue=pose_in_world.child("position").child_value("enabled");
    if(std::stoi(readingValue))
        this->enableEstimationPositionMocapWorldWrtWorld();

    // Attitude of the sensor wrt robot
    readingValue=pose_in_world.child("attitude").child_value("enabled");
    if(std::stoi(readingValue))
        this->enableEstimationAttitudeMocapWorldWrtWorld();



    //// Init State

    /// Pose of the visual marker wrt world

    // Position of the visual marker wrt world
    readingValue=pose_in_world.child("position").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d init_estimation;
        stm>>init_estimation[0]>>init_estimation[1]>>init_estimation[2];
        MapElementInitStateCore->setPositionMocapWorldWrtWorld(init_estimation);
    }

    // Attitude of the visual marker wrt world
    readingValue=pose_in_world.child("attitude").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector4d init_estimation;
        stm>>init_estimation[0]>>init_estimation[1]>>init_estimation[2]>>init_estimation[3];
        MapElementInitStateCore->setAttitudeMocapWorldWrtWorld(init_estimation);
    }


    /// Parameters

    // None


    //// Init Variances


    /// Pose of the visual marker wrt world

    // Position of the visual marker wrt world
    readingValue=pose_in_world.child("position").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setCovariancePositionMocapWorldWrtWorld(variance.asDiagonal());
    }


    // Attitude of the visual marker wrt world
    readingValue=pose_in_world.child("attitude").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setCovarianceAttitudeMocapWorldWrtWorld(variance.asDiagonal());
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

int WorldReferenceFrameCore::setId(int id)
{
    this->id_=id;
    return 0;
}

int WorldReferenceFrameCore::getId() const
{
    return id_;
}

bool WorldReferenceFrameCore::isEstimationPositionMocapWorldWrtWorldEnabled()
{
    return flag_estimation_position_mocap_world_wrt_world_;
}

int WorldReferenceFrameCore::enableEstimationPositionMocapWorldWrtWorld()
{
    if(!flag_estimation_position_mocap_world_wrt_world_)
    {
        flag_estimation_position_mocap_world_wrt_world_=true;
        dimension_state_+=3;
        dimension_error_state_+=3;
        dimension_parameters_-=3;
        dimension_error_parameters_-=3;
    }
    return 0;
}

int WorldReferenceFrameCore::enableParameterPositionMocapWorldWrtWorld()
{
    if(flag_estimation_position_mocap_world_wrt_world_)
    {
        flag_estimation_position_mocap_world_wrt_world_=false;
        dimension_state_-=3;
        dimension_error_state_-=3;
        dimension_parameters_+=3;
        dimension_error_parameters_+=3;
    }
    return 0;
}

Eigen::Matrix3d WorldReferenceFrameCore::getCovariancePositionMocapWorldWrtWorld() const
{
    return this->covariance_position_mocap_world_wrt_world_;
}

int WorldReferenceFrameCore::setCovariancePositionMocapWorldWrtWorld(const Eigen::Matrix3d &covariance_position_mocap_world_wrt_world)
{
    this->covariance_position_mocap_world_wrt_world_=covariance_position_mocap_world_wrt_world;
    return 0;
}


bool WorldReferenceFrameCore::isEstimationAttitudeMocapWorldWrtWorldEnabled()
{
    return this->flag_estimation_attitude_mocap_world_wrt_world_;
}

int WorldReferenceFrameCore::enableEstimationAttitudeMocapWorldWrtWorld()
{
    if(!flag_estimation_attitude_mocap_world_wrt_world_)
    {
        flag_estimation_attitude_mocap_world_wrt_world_=true;
        dimension_state_+=4;
        dimension_error_state_+=3;
        dimension_parameters_-=4;
        dimension_error_parameters_-=3;
    }

    return 0;
}

int WorldReferenceFrameCore::enableParameterAttitudeMocapWorldWrtWorld()
{
    if(flag_estimation_attitude_mocap_world_wrt_world_)
    {
        flag_estimation_attitude_mocap_world_wrt_world_=false;
        dimension_state_-=4;
        dimension_error_state_-=3;
        dimension_parameters_+=4;
        dimension_error_parameters_+=3;
    }
    return 0;
}


Eigen::Matrix3d WorldReferenceFrameCore::getCovarianceAttitudeMocapWorldWrtWorld() const
{
    return this->covariance_attitude_mocap_world_wrt_world_;
}

int WorldReferenceFrameCore::setCovarianceAttitudeMocapWorldWrtWorld(const Eigen::Matrix3d &covariance_attitude_mocap_world_wrt_world)
{
    this->covariance_attitude_mocap_world_wrt_world_=covariance_attitude_mocap_world_wrt_world;
    return 0;
}




int WorldReferenceFrameCore::prepareCovarianceInitErrorStateSpecific()
{
    int dimension_i=0;

    if(isEstimationPositionMocapWorldWrtWorldEnabled())
    {
        this->covariance_init_error_state_.block<3,3>(dimension_i, dimension_i)=getCovariancePositionMocapWorldWrtWorld();
        dimension_i+=3;
    }

    if(isEstimationAttitudeMocapWorldWrtWorldEnabled())
    {
        this->covariance_init_error_state_.block<3,3>(dimension_i, dimension_i)=getCovarianceAttitudeMocapWorldWrtWorld();
        dimension_i+=3;
    }

    return 0;
}

Eigen::SparseMatrix<double> WorldReferenceFrameCore::getCovarianceParameters()
{
    Eigen::SparseMatrix<double> covariance;

    covariance.resize(this->getDimensionErrorParameters(), this->getDimensionErrorParameters());

    std::vector<Eigen::Triplet<double> > tripletCovarianceParameters;


    unsigned int dimension=0;
    if(!this->isEstimationPositionMocapWorldWrtWorldEnabled())
    {
        for(int i=0; i<3; i++)
            tripletCovarianceParameters.push_back(Eigen::Triplet<double>(dimension+i,dimension+i,covariance_position_mocap_world_wrt_world_(i,i)));

        dimension+=3;
    }
    if(!this->isEstimationAttitudeMocapWorldWrtWorldEnabled())
    {
        for(int i=0; i<3; i++)
            tripletCovarianceParameters.push_back(Eigen::Triplet<double>(dimension+i,dimension+i,covariance_attitude_mocap_world_wrt_world_(i,i)));

        dimension+=3;
    }


    covariance.setFromTriplets(tripletCovarianceParameters.begin(), tripletCovarianceParameters.end());



    return covariance;
}


Eigen::SparseMatrix<double> WorldReferenceFrameCore::getCovarianceNoise(const TimeStamp deltaTimeStamp)
{
    Eigen::SparseMatrix<double> covariance;

    covariance.resize(0,0);

    // Do nothing

    return covariance;
}

int WorldReferenceFrameCore::predictState(//Time
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

    // Search for the past map State Core
    std::shared_ptr<WorldReferenceFrameStateCore> past_map_state;

    for(std::list< std::shared_ptr<StateCore> >::iterator it_map_state=pastState->TheListMapElementStateCore.begin();
        it_map_state!=pastState->TheListMapElementStateCore.end();
        ++it_map_state)
    {
        if((*it_map_state)->getMsfElementCoreSharedPtr() == this->getMsfElementCoreSharedPtr())
        {
            past_map_state=std::dynamic_pointer_cast<WorldReferenceFrameStateCore>(*it_map_state);
            break;
        }
    }
    if(!past_map_state)
    {
        std::cout<<"WorldReferenceFrameCore::predictState() unable to find past_map_state"<<std::endl;
        return -10;
    }


    // Predicted State
    std::shared_ptr<WorldReferenceFrameStateCore> predicted_map_state;
    if(!predictedState)
        predicted_map_state=std::make_shared<WorldReferenceFrameStateCore>(past_map_state->getMsfElementCoreWeakPtr());
    else
        predicted_map_state=std::dynamic_pointer_cast<WorldReferenceFrameStateCore>(predictedState);





    // Predict State
    int error_predict_state=predictStateSpecific(previousTimeStamp, currentTimeStamp,
                                         past_map_state,
                                         predicted_map_state);

    // Check error
    if(error_predict_state)
    {
        std::cout<<"WorldReferenceFrameCore::predictState() error_predict_state"<<std::endl;
        return error_predict_state;
    }


    // Set predicted state
    predictedState=predicted_map_state;



    // End
    return 0;
}

int WorldReferenceFrameCore::predictStateSpecific(const TimeStamp &previousTimeStamp, const TimeStamp &currentTimeStamp,
                                                        const std::shared_ptr<WorldReferenceFrameStateCore> &pastState,
                                                        std::shared_ptr<WorldReferenceFrameStateCore>& predictedState)
{

    // Checks in the past state
    if(!pastState->isCorrect())
    {
        return -5;
        std::cout<<"WorldReferenceFrameCore::predictState() error"<<std::endl;
    }


    // Create the predicted state if it doesn't exist
    if(!predictedState)
    {
        predictedState=std::make_shared<WorldReferenceFrameStateCore>(pastState->getMsfElementCoreWeakPtr());
    }


    // Equations


    //Delta Time
    TimeStamp DeltaTime=currentTimeStamp-previousTimeStamp;
    double dt=DeltaTime.get_double();


    /// Position
    if(flag_estimation_position_mocap_world_wrt_world_)
        predictedState->position_mocap_world_wrt_world_=pastState->position_mocap_world_wrt_world_;
    else
        predictedState->position_mocap_world_wrt_world_=pastState->position_mocap_world_wrt_world_;


    /// Attitude
    if(flag_estimation_attitude_mocap_world_wrt_world_)
        predictedState->attitude_mocap_world_wrt_world_=pastState->attitude_mocap_world_wrt_world_;
    else
        predictedState->attitude_mocap_world_wrt_world_=pastState->attitude_mocap_world_wrt_world_;



    return 0;
}

// Jacobian
int WorldReferenceFrameCore::predictErrorStateJacobian(//Time
                             const TimeStamp &previousTimeStamp, const TimeStamp &currentTimeStamp,
                             // Previous State
                             const std::shared_ptr<StateEstimationCore> &past_state,
                            // Inputs
                            const std::shared_ptr<InputCommandComponent> &input_command,
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


    // Search for the past map State Core
    std::shared_ptr<WorldReferenceFrameStateCore> past_map_state;

    for(std::list< std::shared_ptr<StateCore> >::iterator it_map_state=past_state->TheListMapElementStateCore.begin();
        it_map_state!=past_state->TheListMapElementStateCore.end();
        ++it_map_state)
    {
        if((*it_map_state)->getMsfElementCoreSharedPtr() == this->getMsfElementCoreSharedPtr())
        {
            past_map_state=std::dynamic_pointer_cast<WorldReferenceFrameStateCore>(*it_map_state);
            break;
        }
    }
    if(!past_map_state)
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


    /// predicted map State
    std::shared_ptr<WorldReferenceFrameStateCore> predicted_map_state=std::dynamic_pointer_cast<WorldReferenceFrameStateCore>(predicted_state);


    /// Get iterators to fill jacobians

    // Fx & Fp
    // Map Element
    std::vector<Eigen::SparseMatrix<double> >::iterator it_jacobian_error_state_wrt_map_error_state;
    it_jacobian_error_state_wrt_map_error_state=predicted_map_state->jacobian_error_state_.map_elements.begin();

    std::vector<Eigen::SparseMatrix<double> >::iterator it_jacobian_error_state_wrt_map_error_parameters;
    it_jacobian_error_state_wrt_map_error_parameters=predicted_map_state->jacobian_error_parameters_.map_elements.begin();

    for(std::list< std::shared_ptr<StateCore> >::iterator itMapElementStateCore=past_state->TheListMapElementStateCore.begin();
        itMapElementStateCore!=past_state->TheListMapElementStateCore.end();
        ++itMapElementStateCore, ++it_jacobian_error_state_wrt_map_error_state, ++it_jacobian_error_state_wrt_map_error_parameters
        )
    {
        if( std::dynamic_pointer_cast<WorldReferenceFrameStateCore>((*itMapElementStateCore)) == past_map_state )
            break;
    }


    // Fu
    // Nothing


    // Fn
    // Nothing



    /// Predict State Jacobians
    int error_predict_state_jacobians=predictErrorStateJacobiansSpecific(previousTimeStamp, currentTimeStamp,
                                                                         past_map_state,
                                                                         predicted_map_state,
                                                                         // Jacobians Error State: Fx, Fp
                                                                         (*it_jacobian_error_state_wrt_map_error_state),
                                                                         (*it_jacobian_error_state_wrt_map_error_parameters)
                                                                         // Jacobian Error Noise
                                                                         // TODO
                                                                         );

    // Check error
    if(error_predict_state_jacobians)
        return error_predict_state_jacobians;


    /// Set predicted state
    predicted_state=predicted_map_state;

    // End
    return 0;
}

int WorldReferenceFrameCore::predictErrorStateJacobiansSpecific(const TimeStamp& previousTimeStamp, const TimeStamp& currentTimeStamp,
                                                       const std::shared_ptr<WorldReferenceFrameStateCore>& pastState,
                                                       std::shared_ptr<WorldReferenceFrameStateCore>& predictedState,
                                                       // Jacobians Error State: Fx, Fp
                                                       // Map
                                                       Eigen::SparseMatrix<double>& jacobian_error_state_wrt_map_error_state,
                                                       Eigen::SparseMatrix<double>& jacobian_error_state_wrt_map_error_parameters
                                                       // Jacobians Noise: Hn
                                                       // TODO
                                                       )
{
    // Create the predicted state if it doesn't exist
    if(!predictedState)
    {
        return -1;
    }


    //Delta Time
    TimeStamp DeltaTime=currentTimeStamp-previousTimeStamp;
    // delta time
    double dt=DeltaTime.get_double();



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
        // Nothing to do
    }

    /// Map Elements
    {
        // Resize and init
        jacobian_error_state_wrt_map_error_state.resize(dimension_error_state_, dimension_error_state_);
        jacobian_error_state_wrt_map_error_parameters.resize(dimension_error_state_, dimension_error_parameters_);

        std::vector<Eigen::Triplet<double>> triplet_list_jacobian_error_state_wrt_error_state;
        std::vector<Eigen::Triplet<double>> triplet_list_jacobian_error_state_wrt_error_parameters;


        // Fill
        int dimension_error_state_i=0;
        int dimension_error_parameters_i=0;



        // posi / posi
        if(isEstimationPositionMocapWorldWrtWorldEnabled())
        {
            //Eigen::MatrixXd::Identity(3,3);
            for(int i=0; i<3; i++)
                triplet_list_jacobian_error_state_wrt_error_state.push_back(Eigen::Triplet<double>(dimension_error_state_i+i,dimension_error_state_i+i,1));
            dimension_error_state_i+=3;
        }


        // att / att
        if(isEstimationAttitudeMocapWorldWrtWorldEnabled())
        {
            //Eigen::MatrixXd::Identity(3,3);
            for(int i=0; i<3; i++)
                triplet_list_jacobian_error_state_wrt_error_state.push_back(Eigen::Triplet<double>(dimension_error_state_i+i,dimension_error_state_i+i,1));
            dimension_error_state_i+=3;
        }


        // Set From Triplets
        jacobian_error_state_wrt_map_error_state.setFromTriplets(triplet_list_jacobian_error_state_wrt_error_state.begin(), triplet_list_jacobian_error_state_wrt_error_state.end());
        jacobian_error_state_wrt_map_error_parameters.setFromTriplets(triplet_list_jacobian_error_state_wrt_error_parameters.begin(), triplet_list_jacobian_error_state_wrt_error_parameters.end());

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

int WorldReferenceFrameCore::resetErrorStateJacobian(// Time
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

    // Position Sensor World wrt World
    if(this->isEstimationPositionMocapWorldWrtWorldEnabled())
    {
        for(int i=0; i<3; i++)
            triplets_jacobian_error_reset.push_back(Eigen::Triplet<double>(dimension_error_state_i+i, dimension_error_state_i+i, 1.0));

        dimension_error_state_i+=3;
    }

    // Attitude Sensor World wrt World
    if(this->isEstimationAttitudeMocapWorldWrtWorldEnabled())
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
