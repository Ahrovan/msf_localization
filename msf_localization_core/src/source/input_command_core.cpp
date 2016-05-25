
#include "msf_localization_core/input_command_core.h"


InputCommandCore::InputCommandCore()
{
    init();

    return;
}

InputCommandCore::InputCommandCore(const std::weak_ptr<InputCore> input_core_ptr)
{
    init();

    this->input_core_ptr_=input_core_ptr;

    return;
}

InputCommandCore::~InputCommandCore()
{

    return;
}

int InputCommandCore::init()
{
    this->input_command_type_=InputCommandTypes::undefined;

    return 0;
}

int InputCommandCore::setInputCorePtr(const std::weak_ptr<InputCore> input_core_ptr)
{
    this->input_core_ptr_=input_core_ptr;
    return 0;
}

std::weak_ptr<InputCore> InputCommandCore::getInputCoreWeakPtr() const
{
    return this->input_core_ptr_;
}

std::shared_ptr<InputCore> InputCommandCore::getInputCoreSharedPtr() const
{
    std::shared_ptr<InputCore> input_core_ptr=this->input_core_ptr_.lock();
    return input_core_ptr;
}

int InputCommandCore::setInputCommandType(InputCommandTypes input_command_type)
{
    this->input_command_type_=input_command_type;
    return 0;
}

InputCommandTypes InputCommandCore::getInputCommandType() const
{
    return this->input_command_type_;
}

Eigen::SparseMatrix<double> InputCommandCore::getCovarianceInputs(const TimeStamp& deltaTimeStamp)
{
    return this->getInputCoreSharedPtr()->getCovarianceInputs(deltaTimeStamp);
}
