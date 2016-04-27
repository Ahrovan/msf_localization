
#include "msf_localization_core/robot_state_core.h"


RobotStateCore::RobotStateCore() :
    StateCore()
{
    init();

    return;
}

RobotStateCore::RobotStateCore(std::weak_ptr<MsfElementCore> msf_element_core_ptr) :
    StateCore(msf_element_core_ptr)
{
    init();

    return;
}

RobotStateCore::~RobotStateCore()
{
    return;
}

int RobotStateCore::init()
{
    this->state_core_type_=StateCoreTypes::robot;
    this->robot_state_core_type_=RobotStateCoreTypes::undefined;

    return 0;
}

int RobotStateCore::setRobotStateType(RobotStateCoreTypes robot_state_core_type)
{
    this->robot_state_core_type_=robot_state_core_type;
    return 0;
}

RobotStateCoreTypes RobotStateCore::getRobotStateType()
{
    return robot_state_core_type_;
}

/*
int RobotStateCore::setTheRobotCore(std::weak_ptr<const RobotCore> TheRobotCorePtr)
{
    this->TheRobotCorePtr=TheRobotCorePtr;
    return 0;
}
std::shared_ptr<const RobotCore> RobotStateCore::getTheRobotCore() const
{
    std::shared_ptr<const RobotCore> TheRobotCoreSharedPtr=this->TheRobotCorePtr.lock();
    return TheRobotCoreSharedPtr;
}
*/
