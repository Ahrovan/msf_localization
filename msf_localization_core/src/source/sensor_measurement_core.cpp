
#include "msf_localization_core/sensor_measurement_core.h"



SensorMeasurementCore::SensorMeasurementCore()
{
    init();




    return;
}

SensorMeasurementCore::SensorMeasurementCore(std::weak_ptr<SensorCore> TheSensorCorePtr)
{
    init();

    setTheSensorCore(TheSensorCorePtr);


    return;
}

SensorMeasurementCore::~SensorMeasurementCore()
{
//    // Log
//    if(logFile.is_open())
//    {
//        logFile.close();
//    }

    return;
}


int SensorMeasurementCore::init()
{
    measurementType=MeasurementTypes::undefined;


    //    // LOG
    //    const char* env_p = std::getenv("FUSEON_STACK");

    //    logPath=std::string(env_p)+"/logs/"+"logSensorMeasurementCoreFile.txt";

    //    logFile.open(logPath);

    //    if(!logFile.is_open())
    //    {
    //        std::cout<<"unable to open log file"<<std::endl;
    //    }

    return 0;
}


int SensorMeasurementCore::setTheSensorCore(std::weak_ptr<SensorCore> TheSensorCorePtr)
{
    this->TheSensorCorePtr=TheSensorCorePtr;
    return 0;
}
std::shared_ptr<SensorCore> SensorMeasurementCore::getTheSensorCore() const
{
    std::shared_ptr<SensorCore> TheSensorCore=this->TheSensorCorePtr.lock();
    return TheSensorCore;
}

int SensorMeasurementCore::setMeasurementType(MeasurementTypes measurementType)
{
    this->measurementType=measurementType;
    return 0;
}

MeasurementTypes SensorMeasurementCore::getMeasurementType() const
{
    return this->measurementType;
}
