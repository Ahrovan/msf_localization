
#include "robot_state_core.h"


RobotStateCore::RobotStateCore()
{
    return;
}

RobotStateCore::~RobotStateCore()
{
    return;
}


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
