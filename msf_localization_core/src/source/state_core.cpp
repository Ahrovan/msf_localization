
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
