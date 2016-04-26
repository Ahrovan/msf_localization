
#include "msf_localization_core/sensor_core.h"

//#include "state_estimation_core.h"

#include "msf_localization_core/msf_storage_core.h"



SensorCore::SensorCore() :
    SensorBasics(),
    MsfElementCore()
{
    init();

    return;
}

SensorCore::SensorCore(std::weak_ptr<SensorCore> TheSensorCorePtr, std::weak_ptr<MsfStorageCore> TheMsfStorageCore) :
    SensorBasics(),
    MsfElementCore(TheSensorCorePtr, TheMsfStorageCore)
{
    init();

    // end
    return;
}

SensorCore::~SensorCore()
{

    return;
}

int SensorCore::init()
{
    // Dimensions
    dimensionState=0;
    dimensionErrorState=0;
    dimensionParameters=4+3;
    dimensionErrorParameters=3+3;
    dimensionMeasurement=0;
    dimensionErrorMeasurement=0;
    dimensionNoise=0;

    // Element Type
    this->setMsfElementCoreType(MsfElementCoreTypes::sensor);

    // Sensor name
    sensor_name_="sensor";

    // Flags
    flagEstimationAttitudeSensorWrtRobot=false;
    flagEstimationPositionSensorWrtRobot=false;


    // Noise
    noiseAttitudeSensorWrtRobot.setZero();
    noisePositionSensorWrtRobot.setZero();

    return 0;
}

/*
int SensorCore::setTheSensorCore(std::weak_ptr<SensorCore> TheSensorCorePtr)
{
    this->TheSensorCorePtr=TheSensorCorePtr;
    return 0;
}
std::shared_ptr<SensorCore> SensorCore::getTheSensorCore() const
{
    std::shared_ptr<SensorCore> TheSensorCoreSharedPtr=this->TheSensorCorePtr.lock();
    return TheSensorCoreSharedPtr;
}

std::shared_ptr<SensorCore> SensorCore::getTheSensorCoreShared() const
{
    std::shared_ptr<SensorCore> TheSensorCoreSharedPtr=this->TheSensorCorePtr.lock();
    return TheSensorCoreSharedPtr;
}

std::weak_ptr<SensorCore> SensorCore::getTheSensorCoreWeak() const
{
    return this->TheSensorCorePtr;
}


int SensorCore::setTheMsfStorageCore(std::weak_ptr<MsfStorageCore> TheMsfStorageCore)
{
    this->TheMsfStorageCore=TheMsfStorageCore;
    return 0;
}

std::shared_ptr<MsfStorageCore> SensorCore::getTheMsfStorageCore() const
{
    std::shared_ptr<MsfStorageCore> TheMsfStorageCoreSharedPtr=this->TheMsfStorageCore.lock();
    return TheMsfStorageCoreSharedPtr;
}
*/


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


unsigned int SensorCore::getDimensionParameters() const
{
    return this->dimensionParameters;
}

unsigned int SensorCore::getDimensionErrorParameters() const
{
    return this->dimensionErrorParameters;
}

unsigned int SensorCore::getDimensionMeasurement() const
{
    return this->dimensionMeasurement;
}

unsigned int SensorCore::getDimensionErrorMeasurement() const
{
    return this->dimensionErrorMeasurement;
}

unsigned int SensorCore::getDimensionNoise() const
{
    return this->dimensionNoise;
}

int SensorCore::setSensorName(std::string sensor_name)
{
    this->sensor_name_=sensor_name;
    return 0;
}

std::string SensorCore::getSensorName() const
{
    return this->sensor_name_;
}



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
        // Update param
        this->dimensionParameters-=4;
        // Update error param
        this->dimensionErrorParameters-=3;
    }
    return 0;
}

int SensorCore::enableParameterAttitudeSensorWrtRobot()
{
    if(this->flagEstimationAttitudeSensorWrtRobot)
    {
        // Enable
        this->flagEstimationAttitudeSensorWrtRobot=false;
        // Update State Dimension
        this->dimensionState-=4;
        // Update Error State Dimension
        this->dimensionErrorState-=3;
        // Update param
        this->dimensionParameters+=4;
        // Update error param
        this->dimensionErrorParameters+=3;

    }
    return 0;
}

Eigen::Matrix3d SensorCore::getNoiseAttitudeSensorWrtRobot() const
{
    return this->noiseAttitudeSensorWrtRobot;
}

int SensorCore::setNoiseAttitudeSensorWrtRobot(Eigen::Matrix3d noiseAttitudeSensorWrtRobot)
{
    this->noiseAttitudeSensorWrtRobot=noiseAttitudeSensorWrtRobot;
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
        //
        this->dimensionParameters-=3;
        //
        this->dimensionErrorParameters-=3;
    }
    return 0;
}

int SensorCore::enableParameterPositionSensorWrtRobot()
{
    if(this->flagEstimationPositionSensorWrtRobot)
    {
        // Enable
        this->flagEstimationPositionSensorWrtRobot=false;
        // Update State Dimension
        this->dimensionState-=3;
        // Update Error State Dimension
        this->dimensionErrorState-=3;
        //
        this->dimensionParameters+=3;
        //
        this->dimensionErrorParameters+=3;
    }
    return 0;
}

Eigen::Matrix3d SensorCore::getNoisePositionSensorWrtRobot() const
{
    return this->noisePositionSensorWrtRobot;
}

int SensorCore::setNoisePositionSensorWrtRobot(Eigen::Matrix3d noisePositionSensorWrtRobot)
{
    this->noisePositionSensorWrtRobot=noisePositionSensorWrtRobot;
    return 0;
}


Eigen::MatrixXd SensorCore::getInitErrorStateVariance() const
{
    return this->InitErrorStateVariance;
}

int SensorCore::prepareInitErrorStateVariance()
{
    this->InitErrorStateVariance.resize(dimensionErrorState, dimensionErrorState);
    this->InitErrorStateVariance.setZero();
    return 0;
}

