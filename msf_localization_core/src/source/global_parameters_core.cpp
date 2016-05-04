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

GlobalParametersCore::GlobalParametersCore(std::weak_ptr<MsfStorageCore> msf_storage_core_ptr) :
    MsfElementCore(msf_storage_core_ptr)
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

int GlobalParametersCore::setWorldName(std::string world_name)
{
    this->world_name_=world_name;
    return 0;
}

int GlobalParametersCore::readConfig(pugi::xml_node global_parameters, std::shared_ptr<GlobalParametersStateCore>& GlobalParametersInitStateCore)
{
    // Create a class for the TheGlobalParametersCore
    if(!GlobalParametersInitStateCore)
        GlobalParametersInitStateCore=std::make_shared<GlobalParametersStateCore>(this->getMsfElementCoreSharedPtr());

    // Aux vars
    std::string readingValue;


    /// World name
    readingValue=global_parameters.child_value("name");
    this->setWorldName(readingValue);


    /// Init State

    // Gravity
    readingValue=global_parameters.child("gravity").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d init_state;
        stm>>init_state[0]>>init_state[1]>>init_state[2];
        GlobalParametersInitStateCore->setGravity(init_state);
    }



    //// Init Variances


    // Gravity
    readingValue=global_parameters.child("gravity").child_value("init_var");
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

int GlobalParametersCore::setNoiseGravity(Eigen::Matrix3d noiseGravity)
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

int GlobalParametersCore::predictState(//Time
                                     const TimeStamp previousTimeStamp,
                                     const TimeStamp currentTimeStamp,
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


    // World Predicted State
    std::shared_ptr<GlobalParametersStateCore> predictedWorldState;
    if(!predictedState)
        predictedWorldState=std::make_shared<GlobalParametersStateCore>(pastState->TheGlobalParametersStateCore->getMsfElementCoreWeakPtr());
    else
        predictedWorldState=std::dynamic_pointer_cast<GlobalParametersStateCore>(predictedState);


    // Predict State
    int error_predict_state=predictStateSpecific(previousTimeStamp, currentTimeStamp,
                                         std::dynamic_pointer_cast<GlobalParametersStateCore>(pastState->TheGlobalParametersStateCore),
                                         predictedWorldState);

    // Check error
    if(error_predict_state)
        return error_predict_state;


    // Set predicted state
    predictedState=predictedWorldState;


    // End
    return 0;
}

int GlobalParametersCore::predictStateSpecific(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, const std::shared_ptr<GlobalParametersStateCore> pastState, std::shared_ptr<GlobalParametersStateCore>& predictedState)
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
        predictedState=std::make_shared<GlobalParametersStateCore>(pastState->getMsfElementCoreWeakPtr());
    }





    // Equations


    //Delta Time
    TimeStamp DeltaTime=currentTimeStamp-previousTimeStamp;
    double dt=DeltaTime.get_double();


    /// Gravity
    predictedState->setGravity(pastState->getGravity());



    // End
    return 0;
}

int GlobalParametersCore::predictErrorStateJacobian(//Time
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


    // Predicted State
    if(!predictedState)
        return -1;
    // World Predicted State
    std::shared_ptr<GlobalParametersStateCore> predictedWorldState=std::dynamic_pointer_cast<GlobalParametersStateCore>(predictedState);


    // Predict State
    int error_predict_state=predictErrorStateJacobianSpecific(previousTimeStamp, currentTimeStamp,
                                                            std::dynamic_pointer_cast<GlobalParametersStateCore>(pastState->TheGlobalParametersStateCore),
                                                            predictedWorldState);

    // Check error
    if(error_predict_state)
        return error_predict_state;


    // Set predicted state
    predictedState=predictedWorldState;


    // End
    return 0;
}

int GlobalParametersCore::predictErrorStateJacobianSpecific(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp,
                                                            std::shared_ptr<GlobalParametersStateCore> pastState,
                                                            std::shared_ptr<GlobalParametersStateCore>& predictedState)
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
    double dt=DeltaTime.get_double();


    ///// Jacobian Error State

    // TODO



    ///// Jacobian Error State Noise

    // TODO



    return 0;
}

