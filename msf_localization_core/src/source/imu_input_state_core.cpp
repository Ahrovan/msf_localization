
#include "msf_localization_core/imu_input_state_core.h"


#include "msf_localization_core/imu_input_core.h"


ImuInputStateCore::ImuInputStateCore() :
    InputStateCore()
{
    init();

    return;
}

ImuInputStateCore::ImuInputStateCore(std::weak_ptr<MsfElementCore> msf_element_core_ptr) :
    InputStateCore(msf_element_core_ptr)
{
    init();

    return;
}

ImuInputStateCore::~ImuInputStateCore()
{
    return;
}

int ImuInputStateCore::init()
{
    this->setInputStateType(InputStateCoreTypes::imu);

    return 0;
}




Eigen::SparseMatrix<double> ImuInputStateCore::getJacobianErrorState()
{

    // End
    return errorStateJacobian;
}

Eigen::SparseMatrix<double> ImuInputStateCore::getJacobianErrorStateNoise()
{

    // End
    return errorStateNoiseJacobian;
}

int ImuInputStateCore::updateStateFromIncrementErrorState(Eigen::VectorXd increment_error_state)
{

    // TODO


    return 0;
}
