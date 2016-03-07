#include "msf_localization_core/global_parameters_core.h"

#include "msf_localization_core/msf_storage_core.h"


GlobalParametersCore::GlobalParametersCore() :
    dimensionState(0),
    dimensionErrorState(0),
    dimensionParameters(3),
    dimensionErrorParameters(3)
{
    // Flags
    flagEstimationGravity=false;

    // Noises
    noiseGravity.setZero();

    // End
    return;
}

GlobalParametersCore::GlobalParametersCore(std::weak_ptr<MsfStorageCore> TheMsfStorageCore) :
    GlobalParametersCore()
{
    this->setTheMsfStorageCore(TheMsfStorageCore);

    // End
    return;
}

GlobalParametersCore::~GlobalParametersCore()
{
    return;
}

unsigned int GlobalParametersCore::getDimensionState() const
{
    return this->dimensionState;
}

int GlobalParametersCore::setDimensionState(unsigned int dimensionState)
{
    this->dimensionState=dimensionState;
    return 0;
}

unsigned int GlobalParametersCore::getDimensionErrorState() const
{
    return this->dimensionErrorState;
}

int GlobalParametersCore::setDimensionErrorState(unsigned int dimensionErrorState)
{
    this->dimensionErrorState=dimensionErrorState;
    return 0;
}

unsigned int GlobalParametersCore::getDimensionParameters() const
{
    return this->dimensionParameters;
}

int GlobalParametersCore::setDimensionParameters(unsigned int dimensionParameters)
{
    this->dimensionParameters=dimensionParameters;
    return 0;
}

unsigned int GlobalParametersCore::getDimensionErrorParameters() const
{
    return this->dimensionErrorParameters;
}

int GlobalParametersCore::setDimensionErrorParameters(unsigned int dimensionErrorParameters)
{
    this->dimensionErrorParameters=dimensionErrorParameters;
    return 0;
}



int GlobalParametersCore::setTheGlobalParametersCore(std::weak_ptr<const GlobalParametersCore> TheGlobalParametersCorePtr)
{
    this->TheGlobalParametersCorePtr=TheGlobalParametersCorePtr;
    return 0;
}
std::shared_ptr<const GlobalParametersCore> GlobalParametersCore::getTheGlobalParametersCore() const
{
    std::shared_ptr<const GlobalParametersCore> TheGlobalParametersCoreSharedPtr=this->TheGlobalParametersCorePtr.lock();
    return TheGlobalParametersCoreSharedPtr;
}


int GlobalParametersCore::setTheMsfStorageCore(std::weak_ptr<MsfStorageCore> TheMsfStorageCore)
{
    this->TheMsfStorageCore=TheMsfStorageCore;
    return 0;
}

std::shared_ptr<MsfStorageCore> GlobalParametersCore::getTheMsfStorageCore() const
{
    std::shared_ptr<MsfStorageCore> TheMsfStorageCoreSharedPtr=this->TheMsfStorageCore.lock();
    return TheMsfStorageCoreSharedPtr;
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