#include "msf_localization_core/global_parameters_state_core.h"



GlobalParametersStateCore::GlobalParametersStateCore()
{
    // Init
    gravity.setZero();

    return;
}

GlobalParametersStateCore::GlobalParametersStateCore(std::weak_ptr<const GlobalParametersCore> TheGlobalParametersCorePtr) :
    GlobalParametersStateCore()
{
    this->setTheGlobalParametersCore(TheGlobalParametersCorePtr);

    return;
}

GlobalParametersStateCore::~GlobalParametersStateCore()
{
    return;
}


int GlobalParametersStateCore::setTheGlobalParametersCore(std::weak_ptr<const GlobalParametersCore> TheGlobalParametersCorePtr)
{
    this->TheGlobalParametersCorePtr=TheGlobalParametersCorePtr;
    return 0;
}

std::shared_ptr<const GlobalParametersCore> GlobalParametersStateCore::getTheGlobalParametersCore() const
{
    std::shared_ptr<const GlobalParametersCore> TheGlobalParametersCoreSharedPtr=this->TheGlobalParametersCorePtr.lock();
    return TheGlobalParametersCoreSharedPtr;
}

Eigen::Vector3d GlobalParametersStateCore::getGravity() const
{
    return this->gravity;
}

int GlobalParametersStateCore::setGravity(Eigen::Vector3d gravity)
{
    this->gravity=gravity;
    return 0;
}


int GlobalParametersStateCore::updateStateFromIncrementErrorState(Eigen::VectorXd increment_error_state)
{
    unsigned int dimension=0;
    if(this->getTheGlobalParametersCore()->isEstimationGravityEnabled())
    {
        this->gravity+=increment_error_state.block<3,1>(dimension, 0);
        dimension+=3;
    }


    return 0;
}
