
#include "msf_localization_core/imu_input_command_core.h"


ImuInputCommandCore::ImuInputCommandCore() :
    InputCommandCore()
{

    return;
}

ImuInputCommandCore::ImuInputCommandCore(std::weak_ptr<InputCore> input_core_ptr) :
    InputCommandCore(input_core_ptr)
{

    return;
}

ImuInputCommandCore::~ImuInputCommandCore()
{

    return;
}
