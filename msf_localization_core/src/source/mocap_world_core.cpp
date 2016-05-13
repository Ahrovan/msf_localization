
#include "msf_localization_core/mocap_world_core.h"

#include "msf_localization_core/map_element_state_core.h"
#include "msf_localization_core/mocap_world_state_core.h"


MocapWorldCore::MocapWorldCore():
    MapElementCore()
{
    init();

    return;
}

MocapWorldCore::MocapWorldCore(std::weak_ptr<MsfStorageCore> TheMsfStorageCore) :
    MapElementCore(TheMsfStorageCore)
{
    init();

    return;
}

MocapWorldCore::~MocapWorldCore()
{
    return;
}

int MocapWorldCore::init()
{
    // Dimensions
    dimension_state_=0;
    dimension_error_state_=0;

    dimension_parameters_+=3+4;
    dimension_error_parameters_+=3+3;

    // Flags
    flag_estimation_position_mocap_world_wrt_world_=false;
    flag_estimation_attitude_mocap_world_wrt_world_=false;

    // Noises
    covariance_position_mocap_world_wrt_world_.setZero();
    covariance_attitude_mocap_world_wrt_world_.setZero();


    // Map Type
    setMapElementType(MapElementTypes::mocap_world);


    return 0;
}

int MocapWorldCore::readConfig(pugi::xml_node map_element, std::shared_ptr<MocapWorldStateCore>& MapElementInitStateCore)
{
    // Create a class for the SensorStateCore
    if(!MapElementInitStateCore)
        MapElementInitStateCore=std::make_shared<MocapWorldStateCore>(this->getMsfElementCoreSharedPtr());


    // Visual landmark
    pugi::xml_node mocap_world=map_element.child("mocap_world");


    // Auxiliar reading value
    std::string readingValue;



    /// Name
    this->setMapElementName("mocap_world");


    //// Configs

    /// Pose of the map element wrt worl
    pugi::xml_node pose_in_world=mocap_world.child("pose_in_world");

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

bool MocapWorldCore::isEstimationPositionMocapWorldWrtWorldEnabled()
{
    return flag_estimation_position_mocap_world_wrt_world_;
}

int MocapWorldCore::enableEstimationPositionMocapWorldWrtWorld()
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

int MocapWorldCore::enableParameterPositionMocapWorldWrtWorld()
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

Eigen::Matrix3d MocapWorldCore::getCovariancePositionMocapWorldWrtWorld() const
{
    return this->covariance_position_mocap_world_wrt_world_;
}

int MocapWorldCore::setCovariancePositionMocapWorldWrtWorld(Eigen::Matrix3d covariance_position_mocap_world_wrt_world)
{
    this->covariance_position_mocap_world_wrt_world_=covariance_position_mocap_world_wrt_world;
    return 0;
}


bool MocapWorldCore::isEstimationAttitudeMocapWorldWrtWorldEnabled()
{
    return this->flag_estimation_attitude_mocap_world_wrt_world_;
}

int MocapWorldCore::enableEstimationAttitudeMocapWorldWrtWorld()
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

int MocapWorldCore::enableParameterAttitudeMocapWorldWrtWorld()
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


Eigen::Matrix3d MocapWorldCore::getCovarianceAttitudeMocapWorldWrtWorld() const
{
    return this->covariance_attitude_mocap_world_wrt_world_;
}

int MocapWorldCore::setCovarianceAttitudeMocapWorldWrtWorld(Eigen::Matrix3d covariance_attitude_mocap_world_wrt_world)
{
    this->covariance_attitude_mocap_world_wrt_world_=covariance_attitude_mocap_world_wrt_world;
    return 0;
}




int MocapWorldCore::prepareCovarianceInitErrorStateSpecific()
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

Eigen::SparseMatrix<double> MocapWorldCore::getCovarianceParameters()
{
    Eigen::SparseMatrix<double> covariance;

    covariance.resize(this->getDimensionErrorParameters(), this->getDimensionErrorParameters());
    covariance.reserve(this->getDimensionErrorParameters());

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


Eigen::SparseMatrix<double> MocapWorldCore::getCovarianceNoise(const TimeStamp deltaTimeStamp)
{
    Eigen::SparseMatrix<double> covariance;

    covariance.resize(0,0);

    // Do nothing

    return covariance;
}

int MocapWorldCore::predictState(//Time
                 const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp,
                 // Previous State
                 const std::shared_ptr<StateEstimationCore> pastState,
                 // Inputs
                 const std::shared_ptr<InputCommandComponent> inputCommand,
                 // Predicted State
                 std::shared_ptr<StateCore>& predictedState)
{

    // Checks

    // Past State
    if(!pastState)
        return -1;

    // TODO

    // Search for the past map State Core
    std::shared_ptr<MocapWorldStateCore> past_map_state;

    for(std::list< std::shared_ptr<MapElementStateCore> >::iterator it_map_state=pastState->TheListMapElementStateCore.begin();
        it_map_state!=pastState->TheListMapElementStateCore.end();
        ++it_map_state)
    {
        if((*it_map_state)->getMsfElementCoreSharedPtr() == this->getMsfElementCoreSharedPtr())
        {
            past_map_state=std::dynamic_pointer_cast<MocapWorldStateCore>(*it_map_state);
            break;
        }
    }
    if(!past_map_state)
    {
        std::cout<<"MocapWorldCore::predictState() unable to find past_map_state"<<std::endl;
        return -10;
    }


    // Predicted State
    std::shared_ptr<MocapWorldStateCore> predicted_map_state;
    if(!predictedState)
        predicted_map_state=std::make_shared<MocapWorldStateCore>(past_map_state->getMsfElementCoreWeakPtr());
    else
        predicted_map_state=std::dynamic_pointer_cast<MocapWorldStateCore>(predictedState);





    // Predict State
    int error_predict_state=predictStateSpecific(previousTimeStamp, currentTimeStamp,
                                         past_map_state,
                                         predicted_map_state);

    // Check error
    if(error_predict_state)
    {
        std::cout<<"MocapWorldCore::predictState() error_predict_state"<<std::endl;
        return error_predict_state;
    }


    // Set predicted state
    predictedState=predicted_map_state;



    // End
    return 0;
}

int MocapWorldCore::predictStateSpecific(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp,
                                                        const std::shared_ptr<MocapWorldStateCore> pastState,
                                                        std::shared_ptr<MocapWorldStateCore>& predictedState)
{

    // Checks in the past state
    if(!pastState->isCorrect())
    {
        return -5;
        std::cout<<"MocapWorldCore::predictState() error"<<std::endl;
    }


    // Create the predicted state if it doesn't exist
    if(!predictedState)
    {
        predictedState=std::make_shared<MocapWorldStateCore>(pastState->getMsfElementCoreWeakPtr());
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
int MocapWorldCore::predictErrorStateJacobian(//Time
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


    // Search for the past map State Core
    std::shared_ptr<MocapWorldStateCore> past_map_state;

    for(std::list< std::shared_ptr<MapElementStateCore> >::iterator it_map_state=pastState->TheListMapElementStateCore.begin();
        it_map_state!=pastState->TheListMapElementStateCore.end();
        ++it_map_state)
    {
        if((*it_map_state)->getMsfElementCoreSharedPtr() == this->getMsfElementCoreSharedPtr())
        {
            past_map_state=std::dynamic_pointer_cast<MocapWorldStateCore>(*it_map_state);
            break;
        }
    }
    if(!past_map_state)
        return -10;


    // Predicted State
    if(!predictedState)
        return -1;

    // Search for the predicted map Predicted State
    std::shared_ptr<MocapWorldStateCore> predicted_map_state=std::dynamic_pointer_cast<MocapWorldStateCore>(predictedState);



    // Predict State
    int error_predict_state=predictErrorStateJacobiansSpecific(previousTimeStamp, currentTimeStamp,
                                         past_map_state,
                                         predicted_map_state);

    // Check error
    if(error_predict_state)
        return error_predict_state;


    // Set predicted state
    predictedState=predicted_map_state;


    // End
    return 0;
}

int MocapWorldCore::predictErrorStateJacobiansSpecific(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp,
                                                                      const std::shared_ptr<MocapWorldStateCore> pastState,
                                                                      std::shared_ptr<MocapWorldStateCore>& predictedState)
{
    // Create the predicted state if it doesn't exist
    if(!predictedState)
    {
        return 1;
    }


    //Delta Time
    TimeStamp DeltaTime=currentTimeStamp-previousTimeStamp;
    // delta time
    double dt=DeltaTime.get_double();



    //// Jacobian Error State

    // Jacobian Size
    Eigen::SparseMatrix<double> jacobian_error_state;
    jacobian_error_state.resize(dimension_error_state_, dimension_error_state_);
    jacobian_error_state.reserve(dimension_error_state_);

    std::vector<Eigen::Triplet<double> > tripletJacobianErrorState;



    /// Jacobian of the error: Linear Part

    // posi / posi
    if(flag_estimation_position_mocap_world_wrt_world_)
    {
        //predictedState->error_state_jacobian_.linear.block<3,3>(0,0)=Eigen::MatrixXd::Identity(3,3);
        for(int i=0; i<3; i++)
            tripletJacobianErrorState.push_back(Eigen::Triplet<double>(i,i,1));
    }



    /// Jacobian of the error -> Angular Part

    // att / att
    if(flag_estimation_attitude_mocap_world_wrt_world_)
    {
        //predictedState->error_state_jacobian_.angular.block<3,3>(0,0)=Eigen::MatrixXd::Identity(3,3);
        for(int i=0; i<3; i++)
            tripletJacobianErrorState.push_back(Eigen::Triplet<double>(3+i,3+i,1));
    }


    /// Set
    jacobian_error_state.setFromTriplets(tripletJacobianErrorState.begin(), tripletJacobianErrorState.end());


    // TODO FIX!!
    predictedState->setJacobianErrorStateMapElement(jacobian_error_state, 0);



    ///// Jacobian Error State Noise

    predictedState->jacobian_error_state_noise_.resize(0, 0);
    predictedState->jacobian_error_state_noise_.reserve(0);

    std::vector<Eigen::Triplet<double> > tripletJacobianErrorStateNoise;

    // Nothing to do



    // Finish
    return 0;
}

int MocapWorldCore::resetErrorStateJacobian(// Time
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
