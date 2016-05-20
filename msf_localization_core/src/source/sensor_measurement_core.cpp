
#include "msf_localization_core/sensor_measurement_core.h"



SensorMeasurementCore::SensorMeasurementCore()
{
    init();




    return;
}

SensorMeasurementCore::SensorMeasurementCore(std::weak_ptr<SensorCore> sensor_core_ptr)
{
    init();

    this->setSensorCorePtr(sensor_core_ptr);


    return;
}

SensorMeasurementCore::~SensorMeasurementCore()
{

    return;
}


int SensorMeasurementCore::init()
{
    measurementType=MeasurementTypes::undefined;


    return 0;
}


int SensorMeasurementCore::setSensorCorePtr(std::weak_ptr<SensorCore> sensor_core_ptr)
{
    this->sensor_core_ptr_=sensor_core_ptr;
    return 0;
}

std::weak_ptr<SensorCore> SensorMeasurementCore::getSensorCoreWeakPtr() const
{
    return sensor_core_ptr_;
}

std::shared_ptr<SensorCore> SensorMeasurementCore::getSensorCoreSharedPtr() const
{
    std::shared_ptr<SensorCore> sensor_core_ptr=this->sensor_core_ptr_.lock();
    return sensor_core_ptr;
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

bool SensorMeasurementCore::isCorrect() const
{
    if(this->sensor_core_ptr_.expired())
        return false;

    return true;
}

Eigen::SparseMatrix<double> SensorMeasurementCore::getCovarianceMeasurement()
{
    return this->getSensorCoreSharedPtr()->getCovarianceMeasurement();
}
