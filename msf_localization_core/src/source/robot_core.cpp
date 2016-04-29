
#include "msf_localization_core/robot_core.h"


#include "msf_localization_core/msf_storage_core.h"



RobotCore::RobotCore() :
    MsfElementCore()
{
    init();

    return;
}


RobotCore::RobotCore(std::weak_ptr<MsfStorageCore> msf_storage_core_ptr) :
    MsfElementCore(msf_storage_core_ptr)
{
    init();

    return;
}

RobotCore::~RobotCore()
{

    return;
}

int RobotCore::init()
{
    // Dimensions
    dimension_state_=0;
    dimension_error_state_=0;
    dimension_parameters_=0;
    dimension_error_parameters_=0;
    dimension_noise_=0;

    // Element Type
    this->setMsfElementCoreType(MsfElementCoreTypes::robot);

    // Robot Type
    robot_core_type_=RobotCoreTypes::undefined;


    return 0;
}

int RobotCore::setRobotCoreType(RobotCoreTypes robot_core_type)
{
    this->robot_core_type_=robot_core_type;
    return 0;
}
RobotCoreTypes RobotCore::getRobotCoreType() const
{
    return robot_core_type_;
}


