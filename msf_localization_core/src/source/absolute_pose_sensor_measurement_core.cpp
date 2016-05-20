
#include "msf_localization_core/absolute_pose_sensor_measurement_core.h"

#include "msf_localization_core/absolute_pose_sensor_core.h"

AbsolutePoseSensorMeasurementCore::AbsolutePoseSensorMeasurementCore() :
    SensorMeasurementCore()
{
    init();

    return;
}

AbsolutePoseSensorMeasurementCore::AbsolutePoseSensorMeasurementCore(const std::weak_ptr<SensorCore> the_sensor_core) :
    SensorMeasurementCore(the_sensor_core)
{
    init();
    return;
}

AbsolutePoseSensorMeasurementCore::~AbsolutePoseSensorMeasurementCore()
{

    return;
}

int AbsolutePoseSensorMeasurementCore::init()
{
    // Initial values of the measurement
    this->position_mocap_sensor_wrt_mocap_world_.setZero();
    this->attitude_mocap_sensor_wrt_mocap_world_.setZero();

    // Initial values of the Jacobians

    // Measurement type
    measurementType=MeasurementTypes::mocap;

    return 0;
}


int AbsolutePoseSensorMeasurementCore::setPositionMocapSensorWrtMocapWorld(const Eigen::Vector3d &position_mocap_sensor_wrt_mocap_world)
{
    // Sensor Core -> to be able to do checks
    std::shared_ptr<AbsolutePoseSensorCore> sensor_core=std::dynamic_pointer_cast<AbsolutePoseSensorCore>(this->getSensorCoreSharedPtr());

    if(sensor_core->isMeasurementPositionMocapSensorWrtMocapWorldEnabled())
    {
        this->position_mocap_sensor_wrt_mocap_world_=position_mocap_sensor_wrt_mocap_world;
    }
    else
    {
        return 1;
    }
    return 0;
}

int AbsolutePoseSensorMeasurementCore::setAttitudeMocapSensorWrtMocapWorld(const Eigen::Vector4d& attitude_mocap_sensor_wrt_mocap_world)
{
    // Sensor Core -> to be able to do checks
    std::shared_ptr<AbsolutePoseSensorCore> sensor_core=std::dynamic_pointer_cast<AbsolutePoseSensorCore>(this->getSensorCoreSharedPtr());

    if(sensor_core->isMeasurementAttitudeMocapSensorWrtMocapWorldEnabled())
    {
        this->attitude_mocap_sensor_wrt_mocap_world_=attitude_mocap_sensor_wrt_mocap_world;
    }
    else
    {
        return 1;
    }

    return 0;
}

Eigen::Vector3d AbsolutePoseSensorMeasurementCore::getPositionMocapSensorWrtMocapWorld() const
{
    return this->position_mocap_sensor_wrt_mocap_world_;
}

Eigen::Vector4d AbsolutePoseSensorMeasurementCore::getAttitudeMocapSensorWrtMocapWorld() const
{
    return this->attitude_mocap_sensor_wrt_mocap_world_;
}

Eigen::VectorXd AbsolutePoseSensorMeasurementCore::getInnovation(std::shared_ptr<SensorMeasurementCore> theMatchedMeasurementI, std::shared_ptr<SensorMeasurementCore> thePredictedMeasurementI)
{
    // Create the Measurement
    Eigen::VectorXd the_innovation;
    the_innovation.resize(this->getSensorCoreSharedPtr()->getDimensionErrorMeasurement(), 1);
    the_innovation.setZero();

    // Check
    if(theMatchedMeasurementI->getSensorCoreSharedPtr() != thePredictedMeasurementI->getSensorCoreSharedPtr())
    {
        std::cout<<"AbsolutePoseSensorMeasurementCore::getInnovation() error"<<std::endl;
    }

    // Cast
    std::shared_ptr<AbsolutePoseSensorCore> sensor_core=std::dynamic_pointer_cast<AbsolutePoseSensorCore>(theMatchedMeasurementI->getSensorCoreSharedPtr());

    // Cast
    std::shared_ptr<AbsolutePoseSensorMeasurementCore> theMatchedMeasurement=std::dynamic_pointer_cast<AbsolutePoseSensorMeasurementCore>(theMatchedMeasurementI);
    std::shared_ptr<AbsolutePoseSensorMeasurementCore> thePredictedMeasurement=std::dynamic_pointer_cast<AbsolutePoseSensorMeasurementCore>(thePredictedMeasurementI);

    // Fill
    unsigned int dimension=0;
    if(sensor_core->isMeasurementPositionMocapSensorWrtMocapWorldEnabled())
    {
        the_innovation.block<3,1>(dimension,0)=theMatchedMeasurement->position_mocap_sensor_wrt_mocap_world_-thePredictedMeasurement->position_mocap_sensor_wrt_mocap_world_;
        dimension+=3;
    }
    if(sensor_core->isMeasurementAttitudeMocapSensorWrtMocapWorldEnabled())
    {
        Eigen::Vector4d quat_innov_attitude=Quaternion::cross(Quaternion::inv(thePredictedMeasurement->attitude_mocap_sensor_wrt_mocap_world_), theMatchedMeasurement->attitude_mocap_sensor_wrt_mocap_world_);

        the_innovation.block<3,1>(dimension,0)=2*quat_innov_attitude.block<3,1>(1,0);
        dimension+=3;
    }


    return the_innovation;
}


Eigen::VectorXd AbsolutePoseSensorMeasurementCore::getMeasurement()
{
    // Create the Measurement
    Eigen::VectorXd the_measurement;
    the_measurement.resize(this->getSensorCoreSharedPtr()->getDimensionMeasurement(), 1);
    the_measurement.setZero();

    // Sensor Core -> to be able to do checks
    std::shared_ptr<AbsolutePoseSensorCore> sensor_core=std::dynamic_pointer_cast<AbsolutePoseSensorCore>(this->getSensorCoreSharedPtr());

    // Fill
    unsigned int dimension=0;
    if(sensor_core->isMeasurementPositionMocapSensorWrtMocapWorldEnabled())
    {
        the_measurement.block<3,1>(dimension,0)=position_mocap_sensor_wrt_mocap_world_;
        dimension+=3;
    }
    if(sensor_core->isMeasurementAttitudeMocapSensorWrtMocapWorldEnabled())
    {
        the_measurement.block<4,1>(dimension,0)=attitude_mocap_sensor_wrt_mocap_world_;
        dimension+=4;
    }

    return the_measurement;
}
