
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

SensorCore::SensorCore(std::weak_ptr<MsfStorageCore> msf_storage_core_ptr) :
    SensorBasics(),
    MsfElementCore(msf_storage_core_ptr)
{
    //std::cout<<"SensorCore::SensorCore(std::weak_ptr<MsfStorageCore> msf_storage_core_ptr)"<<std::endl;

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
    dimension_state_=0;
    dimension_error_state_=0;
    dimension_parameters_=4+3;
    dimension_error_parameters_=3+3;
    dimension_measurement_=0;
    dimension_error_measurement_=0;
    dimension_noise_=0;

    // Element Type
    this->setMsfElementCoreType(MsfElementCoreTypes::sensor);

    // Sensor name
    //sensor_name_="sensor";

    // Flags
    flagEstimationAttitudeSensorWrtRobot=false;
    flagEstimationPositionSensorWrtRobot=false;


    // Noise
    noiseAttitudeSensorWrtRobot.setZero();
    noisePositionSensorWrtRobot.setZero();

    return 0;
}

int SensorCore::getDimensionMeasurement() const
{
    return this->dimension_measurement_;
}

int SensorCore::getDimensionErrorMeasurement() const
{
    return this->dimension_error_measurement_;
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
        this->dimension_state_+=4;
        // Update Error State Dimension
        this->dimension_error_state_+=3;
        // Update param
        this->dimension_parameters_-=4;
        // Update error param
        this->dimension_error_parameters_-=3;
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
        this->dimension_state_-=4;
        // Update Error State Dimension
        this->dimension_error_state_-=3;
        // Update param
        this->dimension_parameters_+=4;
        // Update error param
        this->dimension_error_parameters_+=3;

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
        this->dimension_state_+=3;
        // Update Error State Dimension
        this->dimension_error_state_+=3;
        //
        this->dimension_parameters_-=3;
        //
        this->dimension_error_parameters_-=3;
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
        this->dimension_state_-=3;
        // Update Error State Dimension
        this->dimension_error_state_-=3;
        //
        this->dimension_parameters_+=3;
        //
        this->dimension_error_parameters_+=3;
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

