
#include "robot_core.h"


#include "msf_storage_core.h"



RobotCore::RobotCore()
{
    // Robot Type
    robotType=RobotTypes::undefined;

    return;
}

RobotCore::~RobotCore()
{
    return;
}


int RobotCore::setRobotType(RobotTypes robotType)
{
    this->robotType=robotType;
    return 0;
}
RobotTypes RobotCore::getRobotType() const
{
    return robotType;
}


int RobotCore::setTheRobotCore(std::weak_ptr<const RobotCore> TheRobotCorePtr)
{
    this->TheRobotCorePtr=TheRobotCorePtr;
    return 0;
}
std::shared_ptr<const RobotCore> RobotCore::getTheRobotCore() const
{
    std::shared_ptr<const RobotCore> TheRobotCoreSharedPtr=this->TheRobotCorePtr.lock();
    return TheRobotCoreSharedPtr;
}


int RobotCore::setTheMsfStorageCore(std::weak_ptr<MsfStorageCore> TheMsfStorageCore)
{
    this->TheMsfStorageCore=TheMsfStorageCore;
    return 0;
}


