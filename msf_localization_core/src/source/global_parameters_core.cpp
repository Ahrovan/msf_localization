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
    dimensionState=0;
    dimensionErrorState=0;
    dimensionParameters=3;
    dimensionErrorParameters=3;

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
        this->dimensionState+=3;
        // Update Error State Dimension
        this->dimensionErrorState+=3;
        //
        this->dimensionParameters-=3;
        //
        this->dimensionErrorParameters-=3;
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
        this->dimensionState-=3;
        // Update Error State Dimension
        this->dimensionErrorState-=3;
        //
        this->dimensionParameters+=3;
        //
        this->dimensionErrorParameters+=3;
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

int GlobalParametersCore::readConfig(pugi::xml_node global_parameters, std::shared_ptr<GlobalParametersStateCore>& GlobalParametersInitStateCore)
{
    // Map Element Core Pointer
    //std::shared_ptr<GlobalParametersCore> TheGlobalParametersCoreAux(this);
    std::shared_ptr<GlobalParametersCore> TheGlobalParametersCoreAux=std::dynamic_pointer_cast<GlobalParametersCore>(this->getMsfElementCoreSharedPtr());

    // Set the access to the Storage core -> Not needed
    //this->setTheMsfStorageCore(TheMsfStorageCore);

    // Set pointer to the TheGlobalParametersCoreAux
    //this->setTheGlobalParametersCore(TheGlobalParametersCoreAux);


    // Create a class for the TheGlobalParametersCore
    if(!GlobalParametersInitStateCore)
        GlobalParametersInitStateCore=std::make_shared<GlobalParametersStateCore>(TheGlobalParametersCoreAux);

    // Set pointer to the Core -> Not needed?
    //GlobalParametersInitStateCore->setTheGlobalParametersCore(TheGlobalParametersCoreAux);


    //std::cout<<"count TheGlobalParametersCoreAux="<<TheGlobalParametersCoreAux.use_count()<<std::endl;


    //TheGlobalParametersCoreAux.reset();


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


    // End
    return 0;
}
