
#include "msf_localization_core/input_command_component.h"



// To avoid circular dependencies
#include "msf_localization_core/input_command_core.h"




InputCommandComponent::InputCommandComponent()
{
    return;
}

InputCommandComponent::~InputCommandComponent()
{
    // Be tidy

    // TheListInputCommandCore
    this->TheListInputCommandCore.clear();


    return;
}

bool InputCommandComponent::hasInputCommand() const
{
    if(this->TheListInputCommandCore.size()!=0)
        return true;
    else
        return false;
}

int InputCommandComponent::getDimensionInputCommand() const
{
    int dimensionInputCommand=0;

    for(std::list< std::shared_ptr<InputCommandCore> >::const_iterator itInputCommand=TheListInputCommandCore.begin();
        itInputCommand!=TheListInputCommandCore.end();
        ++itInputCommand)
    {
        dimensionInputCommand+=(*itInputCommand)->getInputCoreSharedPtr()->getDimensionInputCommand();
    }

    return dimensionInputCommand;
}

int InputCommandComponent::getDimensionErrorInputCommand() const
{
    int dimensionErrorInputCommand=0;

    for(std::list< std::shared_ptr<InputCommandCore> >::const_iterator itInputCommand=TheListInputCommandCore.begin();
        itInputCommand!=TheListInputCommandCore.end();
        ++itInputCommand)
    {
        dimensionErrorInputCommand+=(*itInputCommand)->getInputCoreSharedPtr()->getDimensionErrorInputCommand();
    }

    return dimensionErrorInputCommand;
}

int InputCommandComponent::getNumberInputCommand() const
{
    return this->TheListInputCommandCore.size();
}
