#include "msf_localization_core/global_parameters_core.h"

#include "msf_localization_core/msf_storage_core.h"

#include "msf_localization_core/global_parameters_state_core.h"


GlobalParametersCore::GlobalParametersCore() :
    MsfElementCore()
{
    init();

    // End
    return;
}

GlobalParametersCore::GlobalParametersCore(MsfLocalizationCore *msf_localization_core_ptr) :
    MsfElementCore(msf_localization_core_ptr)
{
    init();

    // End
    return;
}

GlobalParametersCore::~GlobalParametersCore()
{
    return;
}

int GlobalParametersCore::init()
{
    // Dimensions
    dimension_state_=0;
    dimension_error_state_=0;
    dimension_parameters_=3;
    dimension_error_parameters_=3;

    // Element Type
    this->setMsfElementCoreType(MsfElementCoreTypes::world);

    // Default name
    world_name_="world";

    // Flags
    flagEstimationGravity=false;

    // Noises
    noiseGravity.setZero();

    return 0;
}

std::string GlobalParametersCore::getWorldName() const
{
    return this->world_name_;
}

int GlobalParametersCore::setWorldName(const std::string& world_name)
{
    this->world_name_=world_name;
    return 0;
}

int GlobalParametersCore::readConfig(const pugi::xml_node& global_parameters, std::shared_ptr<GlobalParametersStateCore>& GlobalParametersInitStateCore)
{
    // Create a class for the TheGlobalParametersCore
    if(!GlobalParametersInitStateCore)
        GlobalParametersInitStateCore=std::make_shared<GlobalParametersStateCore>(this->getMsfElementCoreWeakPtr());

    // Aux vars
    std::string readingValue;


    /// World name
    readingValue=global_parameters.child_value("name");
    this->setWorldName(readingValue);


    /// Init State

    // Gravity
    readingValue=global_parameters.child("gravity").child_value("init_estimation");
    if(!readingValue.empty())
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d init_state;
        stm>>init_state[0]>>init_state[1]>>init_state[2];
        GlobalParametersInitStateCore->setGravity(init_state);
    }



    //// Init Variances


    // Gravity
    readingValue=global_parameters.child("gravity").child_value("init_var");
    if(!readingValue.empty())
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoiseGravity(variance.asDiagonal());
    }



    // Prepare covariance matrix
    this->prepareCovarianceInitErrorState();


    // End
    return 0;
}

bool GlobalParametersCore::isEstimationGravityEnabled() const
{
    return this->flagEstimationGravity;
}

int GlobalParametersCore::enableEstimationGravity()
{
    if(!this->flagEstimationGravity)
    {
        // Enable
        this->flagEstimationGravity=true;
        // Update State Dimension
        this->dimension_state_+=3;
        // Update Error State Dimension
        this->dimension_error_state_+=3;
        //
        this->dimension_parameters_-=3;
        //
        this->dimension_error_parameters_-=3;
    }
    return 0;
}

int GlobalParametersCore::enableParameterGravity()
{
    if(this->flagEstimationGravity)
    {
        // Enable
        this->flagEstimationGravity=false;
        // Update State Dimension
        this->dimension_state_-=3;
        // Update Error State Dimension
        this->dimension_error_state_-=3;
        //
        this->dimension_parameters_+=3;
        //
        this->dimension_error_parameters_+=3;
    }
    return 0;
}

Eigen::Matrix3d GlobalParametersCore::getNoiseGravity() const
{
    return this->noiseGravity;
}

int GlobalParametersCore::setNoiseGravity(const Eigen::Matrix3d& noiseGravity)
{
    this->noiseGravity=noiseGravity;
    return 0;
}


int GlobalParametersCore::prepareCovarianceInitErrorStateSpecific()
{
    int point=0;
    if(this->isEstimationGravityEnabled())
    {
        this->covariance_init_error_state_.block<3,3>(point,point)=noiseGravity;
        point+=3;
    }


    return 0;
}

Eigen::SparseMatrix<double> GlobalParametersCore::getCovarianceParameters()
{
    Eigen::SparseMatrix<double> covariances_matrix;
    covariances_matrix.resize(this->getDimensionErrorParameters(), this->getDimensionErrorParameters());

    std::vector<Eigen::Triplet<double> > tripletCovarianceParameters;

    unsigned int dimension=0;

    if(!this->isEstimationGravityEnabled())
    {
        //covariance_matrix.block<3,3>(dimension, dimension)=this->getNoiseGravity();

        for(int i=0; i<3; i++)
            tripletCovarianceParameters.push_back(Eigen::Triplet<double>(dimension+i,dimension+i,noiseGravity(i,i)));

        dimension+=3;
    }

    covariances_matrix.setFromTriplets(tripletCovarianceParameters.begin(), tripletCovarianceParameters.end());


    return covariances_matrix;
}

/*
Eigen::MatrixXd GlobalParametersCore::getCovarianceGlobalParameters()
{
    Eigen::MatrixXd covariance_matrix;
    covariance_matrix.resize(this->getDimensionErrorParameters(), this->getDimensionErrorParameters());
    covariance_matrix.setZero();

    unsigned int dimension=0;
    if(!this->isEstimationGravityEnabled())
    {
        covariance_matrix.block<3,3>(dimension, dimension)=this->getNoiseGravity();
        dimension+=3;
    }


    return covariance_matrix;
}
*/

Eigen::SparseMatrix<double> GlobalParametersCore::getCovarianceNoise(const TimeStamp deltaTimeStamp)
{
    Eigen::SparseMatrix<double> covariance_noise;


    return covariance_noise;
}

int GlobalParametersCore::predictState(//Time
                                     const TimeStamp& previousTimeStamp,
                                     const TimeStamp& currentTimeStamp,
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


    // World Predicted State
    GlobalParametersStateCore* predictedWorldState;
    if(!predictedState)
    {
        predictedWorldState=new GlobalParametersStateCore;
        predictedWorldState->setMsfElementCorePtr(pastState->TheGlobalParametersStateCore->getMsfElementCoreWeakPtr());
        predictedState=std::shared_ptr<GlobalParametersStateCore>(predictedWorldState);
    }
    else
        predictedWorldState=dynamic_cast<GlobalParametersStateCore*>(predictedState.get());


    // Predict State
    int error_predict_state=predictStateSpecific(previousTimeStamp, currentTimeStamp,
                                         dynamic_cast<GlobalParametersStateCore*>(pastState->TheGlobalParametersStateCore.get()),
                                         predictedWorldState);

    // Check error
    if(error_predict_state)
        return error_predict_state;


    // End
    return 0;
}

int GlobalParametersCore::predictStateSpecific(const TimeStamp &previousTimeStamp, const TimeStamp &currentTimeStamp,
                                               const GlobalParametersStateCore *pastState,
                                               GlobalParametersStateCore *&predictedState)
{

    // Checks in the past state
    if(!pastState->isCorrect())
    {
        std::cout<<"GlobalParametersCore::predictState() error !pastState->getTheRobotCore()"<<std::endl;
        return -5;
    }


    // Create the predicted state if it doesn't exist
    if(!predictedState)
    {
        predictedState= new GlobalParametersStateCore;
        predictedState->setMsfElementCorePtr(pastState->getMsfElementCoreWeakPtr());
    }





    // Equations


    //Delta Time
    TimeStamp DeltaTime=currentTimeStamp-previousTimeStamp;
    double dt=DeltaTime.getDouble();


    /// Gravity
    predictedState->setGravity(pastState->getGravity());



    // End
    return 0;
}

int GlobalParametersCore::predictErrorStateJacobian(//Time
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


    /// World Predicted State
    GlobalParametersStateCore* predicted_world_state=dynamic_cast<GlobalParametersStateCore*>(predicted_state.get());


    /// Get iterators to fill jacobians

    // Fx & Fp
    // World
    // Nothing to do


    // Fu
    // Nothing


    // Fn
    // TODO



    /// Predict State Jacobians
    int error_predict_state_jacobians=predictErrorStateJacobianSpecific(previousTimeStamp, currentTimeStamp,
                                                                        dynamic_cast<GlobalParametersStateCore*>(past_state->TheGlobalParametersStateCore.get()),
                                                                        predicted_world_state,
                                                                        // Jacobians Error State: Fx, Fp
                                                                        predicted_world_state->jacobian_error_state_.world,
                                                                        predicted_world_state->jacobian_error_parameters_.world
                                                                        // Jacobian Error Noise
                                                                        // TODO
                                                                        );

    // Check error
    if(error_predict_state_jacobians)
        return error_predict_state_jacobians;



    // End
    return 0;
}

int GlobalParametersCore::predictErrorStateJacobianSpecific(const TimeStamp& previousTimeStamp, const TimeStamp& currentTimeStamp,
                                                            const GlobalParametersStateCore* pastState,
                                                            const GlobalParametersStateCore *predictedState,
                                                            // Jacobians Error State: Fx, Fp
                                                            // World
                                                            Eigen::SparseMatrix<double>& jacobian_error_state_wrt_world_error_state,
                                                            Eigen::SparseMatrix<double>& jacobian_error_state_wrt_world_error_parameters
                                                            // Jacobians Noise: Hn
                                                            // TODO
                                                            )
{
    // Check
    if(!predictedState)
    {
        std::cout<<"GlobalParametersCore::predictErrorStateJacobians error en predictedState"<<std::endl;
        return 1;
    }


    //Delta Time
    TimeStamp DeltaTime=currentTimeStamp-previousTimeStamp;
    // delta time
    double dt=DeltaTime.getDouble();



    ///// Jacobian Error State - Error State: Fx & Jacobian Error State - Error Parameters: Fp

    /// World
    {
        // Resize and init
        jacobian_error_state_wrt_world_error_state.resize(dimension_error_state_, dimension_error_state_);
        jacobian_error_state_wrt_world_error_parameters.resize(dimension_error_state_, dimension_error_parameters_);

        std::vector<Eigen::Triplet<double>> triplet_list_jacobian_error_state_wrt_error_state;
        std::vector<Eigen::Triplet<double>> triplet_list_jacobian_error_state_wrt_error_parameters;


        // Fill

        int dimension_error_state_i=0;
        int dimension_error_parameters_i=0;


        // gravity
        if(this->isEstimationGravityEnabled())
        {
            // TODO
        }


        // Set From Triplets
        jacobian_error_state_wrt_world_error_state.setFromTriplets(triplet_list_jacobian_error_state_wrt_error_state.begin(), triplet_list_jacobian_error_state_wrt_error_state.end());
        jacobian_error_state_wrt_world_error_parameters.setFromTriplets(triplet_list_jacobian_error_state_wrt_error_parameters.begin(), triplet_list_jacobian_error_state_wrt_error_parameters.end());

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

int GlobalParametersCore::resetErrorStateJacobian(// Time
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

    if(this->isEstimationGravityEnabled())
    {
        for(int i=0; i<3; i++)
            triplets_jacobian_error_reset.push_back(Eigen::Triplet<double>(dimension_error_state_i+i, dimension_error_state_i+i, 1.0));

        dimension_error_state_i+=3;
    }


    current_state->jacobian_error_state_reset_.setFromTriplets(triplets_jacobian_error_reset.begin(), triplets_jacobian_error_reset.end());

    // End
    return 0;
}
