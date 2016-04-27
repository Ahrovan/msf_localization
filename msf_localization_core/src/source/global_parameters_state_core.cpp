#include "msf_localization_core/global_parameters_state_core.h"



GlobalParametersStateCore::GlobalParametersStateCore() :
    StateCore()
{
    // Init
    init();

    return;
}

GlobalParametersStateCore::GlobalParametersStateCore(std::weak_ptr<MsfElementCore> msf_element_core_ptr) :
    StateCore(msf_element_core_ptr)
{
    init();

    return;
}

GlobalParametersStateCore::~GlobalParametersStateCore()
{
    return;
}

int GlobalParametersStateCore::init()
{
    this->setStateCoreType(StateCoreTypes::world);

    gravity.setZero();

    return 0;
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
    if(std::dynamic_pointer_cast<GlobalParametersCore>(this->getMsfElementCoreSharedPtr())->isEstimationGravityEnabled())
    {
        this->gravity+=increment_error_state.block<3,1>(dimension, 0);
        dimension+=3;
    }


    return 0;
}
