
#include "msf_localization_core/px4flow_sensor_measurement_core.h"

#include "msf_localization_core/px4flow_sensor_core.h"


Px4FlowSensorMeasurementCore::Px4FlowSensorMeasurementCore() :
    SensorMeasurementCore()
{
    init();

    return;
}

Px4FlowSensorMeasurementCore::Px4FlowSensorMeasurementCore(const std::weak_ptr<SensorCore> sensor_core_ptr) :
    SensorMeasurementCore(sensor_core_ptr)
{
    init();

    return;
}

Px4FlowSensorMeasurementCore::~Px4FlowSensorMeasurementCore()
{
    return;
}

int Px4FlowSensorMeasurementCore::init()
{
    // Flags
    flag_velocity_set_=false;
    flag_ground_distance_set_=false;

    // Values
    this->velocity_.setZero();
    this->ground_distance_=-1;

    // Measurement type
    setMeasurementType(MeasurementTypes::px4flow);

    return 0;
}

bool Px4FlowSensorMeasurementCore::measurementSet()
{
    if(isVelocitySet())
        return true;
    if(isGroundDistanceSet())
        return true;
    return false;
}

bool Px4FlowSensorMeasurementCore::isVelocitySet() const
{
    return this->flag_velocity_set_;
}

void Px4FlowSensorMeasurementCore::setVelocity(const Eigen::Vector2d& velocity)
{
    this->velocity_=velocity;
    this->flag_velocity_set_=true;
    return;
}

Eigen::Vector2d Px4FlowSensorMeasurementCore::getVelocity() const
{
    return this->velocity_;
}

bool Px4FlowSensorMeasurementCore::isGroundDistanceSet() const
{
    return this->flag_ground_distance_set_;
}

void Px4FlowSensorMeasurementCore::setGroundDistance(double ground_distance)
{
    this->ground_distance_=ground_distance;
    this->flag_ground_distance_set_=true;
    return;
}

double Px4FlowSensorMeasurementCore::getGroundDistance() const
{
    return this->ground_distance_;
}

Eigen::VectorXd Px4FlowSensorMeasurementCore::getInnovation(const std::shared_ptr<SensorMeasurementCore> &theMatchedMeasurementI, const std::shared_ptr<SensorMeasurementCore> &thePredictedMeasurementI)
{
    // Create the Measurement
    Eigen::VectorXd TheInnovation;
    TheInnovation.resize(this->getSensorCoreSharedPtr()->getDimensionErrorMeasurement(), 1);
    TheInnovation.setZero();

    // Check
    if(theMatchedMeasurementI->getSensorCoreSharedPtr() != thePredictedMeasurementI->getSensorCoreSharedPtr())
    {
        std::cout<<"Px4FlowSensorMeasurementCore::getInnovation() error"<<std::endl;
    }


    // TODO Improve!
    TheInnovation=theMatchedMeasurementI->getMeasurement()-thePredictedMeasurementI->getMeasurement();



    return TheInnovation;
}

Eigen::VectorXd Px4FlowSensorMeasurementCore::getMeasurement()
{
    // Create the Measurement
    Eigen::VectorXd sensor_measurement;
    sensor_measurement.resize(this->getSensorCoreSharedPtr()->getDimensionMeasurement(), 1);
    sensor_measurement.setZero();

    // Sensor Core
    std::shared_ptr<Px4FlowSensorCore> px4flow_sensor_core=std::dynamic_pointer_cast<Px4FlowSensorCore>(this->getSensorCoreSharedPtr());

    // Fill
    unsigned int dimension=0;

    if(px4flow_sensor_core->isMeasurementVelocityEnabled())
    {
        sensor_measurement.block<2,1>(dimension,0)=getVelocity();
        dimension+=2;
    }

    if(px4flow_sensor_core->isMeasurementGroundDistanceEnabled())
    {
        sensor_measurement(dimension,0)=getGroundDistance();
        dimension+=1;
    }


    return sensor_measurement;
}
