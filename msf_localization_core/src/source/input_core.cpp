
#include "msf_localization_core/input_core.h"


InputCore::InputCore() :
    MsfElementCore()
{
    init();

    return;
}

InputCore::InputCore(std::weak_ptr<MsfElementCore> msf_element_core_ptr, std::weak_ptr<MsfStorageCore> msf_storage_core_ptr) :
    MsfElementCore(msf_element_core_ptr, msf_storage_core_ptr)
{
    init();

    return;
}

InputCore::~InputCore()
{

    return;
}

int InputCore::init()
{
    // MsfElementType
    this->setMsfElementCoreType(MsfElementCoreTypes::input);


    return 0;
}

int InputCore::setInputType(InputTypes input_type)
{
    this->input_type_=input_type;
    return 0;
}

InputTypes InputCore::getInputType() const
{
    return this->input_type_;
}
