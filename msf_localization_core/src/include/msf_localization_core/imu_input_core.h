
#ifndef _IMU_INPUT_CORE_H
#define _IMU_INPUT_CORE_H



#include "msf_localization_core/input_core.h"


class ImuInputCore : public InputCore
{
public:
    ImuInputCore();
    ImuInputCore(std::weak_ptr<MsfStorageCore> the_msf_storage_core);
    ~ImuInputCore();

protected:
    int init();

};



#endif
