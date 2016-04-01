
#include "msf_localization_core/robot_core.h"


#include "msf_localization_core/msf_storage_core.h"



RobotCore::RobotCore() :
    dimensionState(0),
    dimensionErrorState(0),
    dimensionParameters(0),
    dimensionErrorParameters(0),
    dimensionNoise(0)
{
    // Robot Type
    robotType=RobotTypes::undefined;

    // Robot name default
    robot_name_="robot";




    // LOG
    const char* env_p = std::getenv("FUSEON_STACK");

    logPath=std::string(env_p)+"/logs/"+"logRobotCoreFile.txt";

    logFile.open(logPath);

    if(!logFile.is_open())
    {
        std::cout<<"unable to open log file"<<std::endl;
    }

    return;
}

RobotCore::~RobotCore()
{
    // Log
    if(logFile.is_open())
    {
        logFile.close();
    }


    return;
}

int RobotCore::setRobotName(std::string robot_name)
{
    this->robot_name_=robot_name;
    return 0;
}

std::string RobotCore::getRobotName() const
{
    return this->robot_name_;
}

unsigned int RobotCore::getDimensionState() const
{
    return this->dimensionState;
}

int RobotCore::setDimensionState(unsigned int dimensionState)
{
    this->dimensionState=dimensionState;
    return 0;
}

unsigned int RobotCore::getDimensionErrorState() const
{
    return this->dimensionErrorState;
}

int RobotCore::setDimensionErrorState(unsigned int dimensionErrorState)
{
    this->dimensionErrorState=dimensionErrorState;
    return 0;
}

unsigned int RobotCore::getDimensionParameters() const
{
    return this->dimensionParameters;
}

int RobotCore::setDimensionParameters(unsigned int dimensionParameters)
{
    this->dimensionParameters=dimensionParameters;
    return 0;
}

unsigned int RobotCore::getDimensionErrorParameters() const
{
    return this->dimensionErrorParameters;
}

int RobotCore::setDimensionErrorParameters(unsigned int dimensionErrorParameters)
{
    this->dimensionErrorParameters=dimensionErrorParameters;
    return 0;
}

unsigned int RobotCore::getDimensionNoise() const
{
    return this->dimensionNoise;
}

int RobotCore::setDimensionNoise(unsigned int dimensionNoise)
{
    this->dimensionNoise=dimensionNoise;
    return 0;
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

std::shared_ptr<MsfStorageCore> RobotCore::getTheMsfStorageCore() const
{
    std::shared_ptr<MsfStorageCore> TheMsfStorageCoreSharedPtr=this->TheMsfStorageCore.lock();
    return TheMsfStorageCoreSharedPtr;
}


Eigen::MatrixXd RobotCore::getInitErrorStateVariance() const
{
    return this->InitErrorStateVariance;
}


int RobotCore::log(std::string logString)
{
    // Lock mutex
    TheLogFileMutex.lock();

    // Write in file
    logFile<<logString;

    // Unlock mutex
    TheLogFileMutex.unlock();

    return 0;
}
