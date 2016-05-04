
#include "msf_localization_core/state_core.h"


StateCore::StateCore()
{
    init();

    return;
}

StateCore::StateCore(std::weak_ptr<MsfElementCore> msf_element_core_ptr)
{
    init();

    this->msf_element_core_ptr_=msf_element_core_ptr;

    return;
}

StateCore::~StateCore()
{

    return;
}

int StateCore::init()
{
    state_core_type_=StateCoreTypes::undefined;

    return 0;
}

int StateCore::setMsfElementCorePtr(std::weak_ptr<MsfElementCore> msf_element_core_ptr)
{
    this->msf_element_core_ptr_=msf_element_core_ptr;
    return 0;
}

std::shared_ptr<MsfElementCore> StateCore::getMsfElementCoreSharedPtr() const
{
    std::shared_ptr<MsfElementCore> msf_element_core_ptr=this->msf_element_core_ptr_.lock();
    return msf_element_core_ptr;
}

std::weak_ptr<MsfElementCore> StateCore::getMsfElementCoreWeakPtr() const
{
    return this->msf_element_core_ptr_;
}

int StateCore::setStateCoreType(StateCoreTypes state_core_type)
{
    this->state_core_type_=state_core_type;
    return 0;
}

StateCoreTypes StateCore::getStateCoreType() const
{
    return this->state_core_type_;
}

bool StateCore::isCorrect()
{
    if(this->msf_element_core_ptr_.expired())
        return false;

    return true;
}

//Eigen::SparseMatrix<double> StateCore::getJacobianErrorState()
//{
//    return this->jacobian_error_state_;
//}

Eigen::SparseMatrix<double> StateCore::getJacobianErrorStateWorld()
{
    return this->jacobian_error_state_.world;
}

Eigen::SparseMatrix<double> StateCore::getJacobianErrorStateRobot()
{
    return this->jacobian_error_state_.robot;
}

Eigen::SparseMatrix<double> StateCore::getJacobianErrorStateInput(int input_number)
{
    if(input_number >= this->jacobian_error_state_.inputs.size())
        return Eigen::SparseMatrix<double> ();
    return this->jacobian_error_state_.inputs[input_number];
}

Eigen::SparseMatrix<double> StateCore::getJacobianErrorStateSensor(int sensor_number)
{
    if(sensor_number >= this->jacobian_error_state_.sensors.size())
    {
        return Eigen::SparseMatrix<double> ();
    }
    return this->jacobian_error_state_.sensors[sensor_number];
}

Eigen::SparseMatrix<double> StateCore::getJacobianErrorStateMapElement(int map_element_number)
{
    if(map_element_number >= this->jacobian_error_state_.map_elements.size())
        return Eigen::SparseMatrix<double> ();
    return this->jacobian_error_state_.map_elements[map_element_number];
}

int StateCore::setJacobianErrorStateWorld(Eigen::SparseMatrix<double> jacobian_error_state)
{
    this->jacobian_error_state_.world=jacobian_error_state;
    return 0;
}

int StateCore::setJacobianErrorStateRobot(Eigen::SparseMatrix<double> jacobian_error_state)
{
    this->jacobian_error_state_.robot=jacobian_error_state;
    return 0;
}

int StateCore::setJacobianErrorStateInput(Eigen::SparseMatrix<double> jacobian_error_state, int input_number)
{
    if(input_number >= this->jacobian_error_state_.inputs.size())
    {
        this->jacobian_error_state_.inputs.resize(input_number+1);
    }
    this->jacobian_error_state_.inputs[input_number]=jacobian_error_state;
    return 0;
}

int StateCore::setJacobianErrorStateSensor(Eigen::SparseMatrix<double> jacobian_error_state, int sensor_number)
{
    if(sensor_number >= this->jacobian_error_state_.sensors.size())
    {
        this->jacobian_error_state_.sensors.resize(sensor_number+1);
    }
    this->jacobian_error_state_.sensors[sensor_number]=jacobian_error_state;
    return 0;
}

int StateCore::setJacobianErrorStateMapElement(Eigen::SparseMatrix<double> jacobian_error_state, int map_element_number)
{
    if(map_element_number >= this->jacobian_error_state_.map_elements.size())
    {
        this->jacobian_error_state_.map_elements.resize(map_element_number+1);
    }
    this->jacobian_error_state_.map_elements[map_element_number]=jacobian_error_state;
    return 0;
}

int StateCore::setJacobianErrorStateInputsSize(int size_inputs)
{
    this->jacobian_error_state_.inputs.resize(size_inputs);
}

int StateCore::setJacobianErrorStateSensorsSize(int size_sensors)
{
    this->jacobian_error_state_.sensors.resize(size_sensors);
}

int StateCore::setJacobianErrorStateMapElementsSize(int size_map_elements)
{
    this->jacobian_error_state_.map_elements.resize(size_map_elements);
}

Eigen::SparseMatrix<double> StateCore::getJacobianErrorParameters()
{
    return this->jacobian_error_parameters_;
}

Eigen::SparseMatrix<double> StateCore::getJacobianErrorInputs()
{
    return this->jacobian_error_inputs_;
}

Eigen::SparseMatrix<double> StateCore::getJacobianErrorStateNoise()
{
    return this->jacobian_error_state_noise_;
}
