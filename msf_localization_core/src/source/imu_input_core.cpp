
#include "msf_localization_core/imu_input_core.h"


ImuInputCore::ImuInputCore() :
    InputCore()
{
    init();

    return;
}

ImuInputCore::ImuInputCore(std::weak_ptr<MsfStorageCore> the_msf_storage_core) :
    InputCore(the_msf_storage_core)
{
    init();

    return;
}

ImuInputCore::~ImuInputCore()
{

    return;
}

int ImuInputCore::init()
{

    return 0;
}
