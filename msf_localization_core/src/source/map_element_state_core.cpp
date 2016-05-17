
#include "msf_localization_core/map_element_state_core.h"

MapElementStateCore::MapElementStateCore() :
    StateCore()
{
    init();

    return;
}

MapElementStateCore::MapElementStateCore(std::weak_ptr<MsfElementCore> msf_element_core_ptr) :
    StateCore(msf_element_core_ptr)
{
    init();

    return;
}

MapElementStateCore::~MapElementStateCore()
{
    return;
}

int MapElementStateCore::init()
{
    // State core Type
    this->setStateCoreType(StateCoreTypes::map);

    return 0;
}


Eigen::MatrixXd MapElementStateCore::getJacobianMappingRobotErrorState()
{
    return this->jacobian_mapping_error_state_.jacobian_mapping_robot_error_state_;
}

Eigen::MatrixXd MapElementStateCore::getJacobianMappingGlobalParametersErrorState()
{
    return this->jacobian_mapping_error_state_.jacobian_mapping_global_parameters_error_state_;
}

Eigen::MatrixXd MapElementStateCore::getJacobianMappingSensorErrorState()
{
    return this->jacobian_mapping_error_state_.jacobian_mapping_sensor_error_state_;
}

Eigen::MatrixXd MapElementStateCore::getJacobianMappingErrorStateNoise()
{
    return this->jacobian_mapping_error_state_noise_;
}
