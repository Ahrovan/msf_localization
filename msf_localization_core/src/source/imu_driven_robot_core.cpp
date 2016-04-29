
#include "msf_localization_core/imu_driven_robot_core.h"


ImuDrivenRobotCore::ImuDrivenRobotCore() :
    RobotCore()
{
    init();

    return;
}

ImuDrivenRobotCore::ImuDrivenRobotCore(std::weak_ptr<MsfStorageCore> msf_storage_core_ptr) :
    RobotCore(msf_storage_core_ptr)
{
    init();

    return;
}

ImuDrivenRobotCore::~ImuDrivenRobotCore()
{

}

int ImuDrivenRobotCore::init()
{
    dimension_state_=6+4;
    dimension_error_state_=6+3;

    dimension_parameters_=0;
    dimension_error_parameters_=0;

    dimension_noise_=0;



    // Type
    this->setRobotCoreType(RobotCoreTypes::imu_driven);


    return 0;
}
