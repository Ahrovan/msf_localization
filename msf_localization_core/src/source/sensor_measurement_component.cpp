
#include "msf_localization_core/sensor_measurement_component.h"



// To avoid circular dependencies
#include "msf_localization_core/sensor_measurement_core.h"



SensorMeasurementComponent::SensorMeasurementComponent()
{
    return;
}

SensorMeasurementComponent::~SensorMeasurementComponent()
{
    // Be tidy

    // list_sensor_measurement_core_
    this->list_sensor_measurement_core_.clear();


    return;
}

bool SensorMeasurementComponent::hasMeasurement() const
{
    if(this->list_sensor_measurement_core_.size()!=0)
        return true;
    else
        return false;
}

int SensorMeasurementComponent::getDimensionMeasurement() const
{
    int dimension_sensor_measurement=0;

    for(std::list< std::shared_ptr<SensorMeasurementCore> >::const_iterator it_sensor_measurement=list_sensor_measurement_core_.begin();
        it_sensor_measurement!=list_sensor_measurement_core_.end();
        ++it_sensor_measurement)
    {
        dimension_sensor_measurement+=(*it_sensor_measurement)->getSensorCoreSharedPtr()->getDimensionMeasurement();
    }

    return dimension_sensor_measurement;
}

int SensorMeasurementComponent::getDimensionErrorMeasurement() const
{
    int dimension_error_sensor_measurement=0;

    for(std::list< std::shared_ptr<SensorMeasurementCore> >::const_iterator it_sensor_measurement=list_sensor_measurement_core_.begin();
        it_sensor_measurement!=list_sensor_measurement_core_.end();
        ++it_sensor_measurement)
    {
        dimension_error_sensor_measurement+=(*it_sensor_measurement)->getSensorCoreSharedPtr()->getDimensionErrorMeasurement();
    }

    return dimension_error_sensor_measurement;
}

