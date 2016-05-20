

#include "msf_localization_core/input_state_core.h"


InputStateCore::InputStateCore() :
    StateCore()
{
    init();

    return;
}

InputStateCore::InputStateCore(const std::weak_ptr<MsfElementCore> msf_element_core_ptr) :
    StateCore(msf_element_core_ptr)
{
    init();

    return;
}

InputStateCore::~InputStateCore()
{
    return;
}

int InputStateCore::init()
{
    this->setStateCoreType(StateCoreTypes::input);
    this->input_state_core_type_=InputStateCoreTypes::undefined;

    return 0;
}

int InputStateCore::setInputStateType(InputStateCoreTypes input_state_core_type)
{
    this->input_state_core_type_=input_state_core_type;
    return 0;
}

InputStateCoreTypes InputStateCore::getInputStateType()
{
    return input_state_core_type_;
}
