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

int GlobalParametersStateCore::setGravity(const Eigen::Vector3d& gravity)
{
    this->gravity=gravity;
    return 0;
}


int GlobalParametersStateCore::updateStateFromIncrementErrorState(const Eigen::VectorXd &increment_error_state)
{
    std::shared_ptr<GlobalParametersCore> world_core=std::dynamic_pointer_cast<GlobalParametersCore>(this->getMsfElementCoreSharedPtr());

    int dimension=0;

    if(world_core->isEstimationGravityEnabled())
    {
        this->gravity+=increment_error_state.block<3,1>(dimension, 0);
        dimension+=3;
    }


    return 0;
}
