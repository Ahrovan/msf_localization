
#include "sensor_core.h"

//#include "state_estimation_core.h"

#include "msf_storage_core.h"



SensorCore::SensorCore() :
    SensorBasics(),
    dimensionState(0),
    dimensionErrorState(0)
{


    // LOG
    const char* env_p = std::getenv("FUSEON_STACK");

    logPath=std::string(env_p)+"/logs/"+"SensorCoreLogFile.txt";

    logFile.open(logPath);

    if(!logFile.is_open())
    {
        std::cout<<"unable to open log file"<<std::endl;
    }


    return;
}

SensorCore::~SensorCore()
{
    // Log
    if(logFile.is_open())
    {
        logFile.close();
    }

    return;
}


int SensorCore::setTheSensorCore(std::weak_ptr<const SensorCore> TheSensorCorePtr)
{
    this->TheSensorCorePtr=TheSensorCorePtr;
    return 0;
}
std::shared_ptr<const SensorCore> SensorCore::getTheSensorCore() const
{
    std::shared_ptr<const SensorCore> TheSensorCoreSharedPtr=this->TheSensorCorePtr.lock();
    return TheSensorCoreSharedPtr;
}


int SensorCore::setTheMsfStorageCore(std::weak_ptr<MsfStorageCore> TheMsfStorageCore)
{
    this->TheMsfStorageCore=TheMsfStorageCore;
    return 0;
}



unsigned int SensorCore::getDimensionState() const
{
    return this->dimensionState;
}

//int SensorCore::setDimensionState(unsigned int dimensionState)
//{
//    this->dimensionState=dimensionState;
//    return 0;
//}

unsigned int SensorCore::getDimensionErrorState() const
{
    return this->dimensionErrorState;
}

//int SensorCore::setDimensionErrorState(unsigned int dimensionErrorState)
//{
//    this->dimensionErrorState=dimensionErrorState;
//    return 0;
//}



bool SensorCore::isEstimationAttitudeSensorWrtRobotEnabled() const
{
    return this->flagEstimationAttitudeSensorWrtRobot;
}

int SensorCore::enableEstimationAttitudeSensorWrtRobot()
{
    if(!this->flagEstimationAttitudeSensorWrtRobot)
    {
        // Enable
        this->flagEstimationAttitudeSensorWrtRobot=true;
        // Update State Dimension
        this->dimensionState+=4;
        // Update Error State Dimension
        this->dimensionErrorState+=3;
    }
    return 0;
}

bool SensorCore::isEstimationPositionSensorWrtRobotEnabled() const
{
    return this->flagEstimationPositionSensorWrtRobot;
}

int SensorCore::enableEstimationPositionSensorWrtRobot()
{
    if(!this->flagEstimationPositionSensorWrtRobot)
    {
        // Enable
        this->flagEstimationPositionSensorWrtRobot=true;
        // Update State Dimension
        this->dimensionState+=3;
        // Update Error State Dimension
        this->dimensionErrorState+=3;
    }
    return 0;
}



