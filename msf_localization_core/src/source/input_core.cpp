
#include "msf_localization_core/input_core.h"

#include "msf_localization_core/input_command_core.h"


InputCore::InputCore() :
    MsfElementCore()
{
    init();

    return;
}

InputCore::InputCore(MsfLocalizationCore *msf_localization_core_ptr) :
    MsfElementCore(msf_localization_core_ptr)
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


    // Name
    input_name_="";

    // Flags
    flag_input_enabled_=false;
    flag_input_active_request_=false; // By default is pasive

    // Id
    id_=-1;

    // Dimensions
    dimension_input_command_=0;
    dimension_error_input_command_=0;


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

int InputCore::setInputName(std::string input_name)
{
    input_name_=input_name;
    return 0;
}

std::string InputCore::getInputName() const
{
    return this->input_name_;
}

void InputCore::setInputId(int id)
{
    this->id_=id;
    return;
}

int InputCore::getInputId() const
{
    return this->id_;
}

bool InputCore::isInputEnabled() const
{
    return this->flag_input_enabled_;
}

int InputCore::setInputEnabled(bool flag_input_enabled)
{
    this->flag_input_enabled_=flag_input_enabled;
    return 0;
}

bool InputCore::isInputActiveRequest() const
{
    return this->flag_input_active_request_;
}

int InputCore::setInputActiveRequest(bool flag_input_active_request)
{
    this->flag_input_active_request_=flag_input_active_request;
    return 0;
}

unsigned int InputCore::getDimensionInputCommand() const
{
    return this->dimension_input_command_;
}

int InputCore::setDimensionInputCommand(unsigned int dimension_input_command)
{
    this->dimension_input_command_=dimension_input_command;
    return 0;
}

unsigned int InputCore::getDimensionErrorInputCommand() const
{
    return this->dimension_error_input_command_;
}

int InputCore::setDimensionErrorInputCommand(unsigned int dimension_error_input_command)
{
    this->dimension_error_input_command_=dimension_error_input_command;
    return 0;
}


