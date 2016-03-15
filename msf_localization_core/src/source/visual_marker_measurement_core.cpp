
#include "msf_localization_core/visual_marker_measurement_core.h"

#include "msf_localization_core/visual_marker_eye_core.h"

VisualMarkerMeasurementCore::VisualMarkerMeasurementCore() :
    SensorMeasurementCore()
{
    // Initial values of the measurement
    this->id_=-1;
    this->position_.setZero();
    this->attitude_.setZero();

    // Initial values of the Jacobians

    return;
}

VisualMarkerMeasurementCore::VisualMarkerMeasurementCore(std::weak_ptr<SensorCore> the_sensor_core) :
    SensorMeasurementCore(the_sensor_core)
{

    return;
}

VisualMarkerMeasurementCore::~VisualMarkerMeasurementCore()
{

    return;
}

int VisualMarkerMeasurementCore::setVisualMarkerId(const int id)
{
    this->id_=id;
    return 0;
}

int VisualMarkerMeasurementCore::setVisualMarkerPosition(const Eigen::Vector3d position)
{
    // Sensor Core -> to be able to do checks
    std::shared_ptr<VisualMarkerEyeCore> the_visual_marker_eye_core=std::dynamic_pointer_cast<VisualMarkerEyeCore>(this->getTheSensorCore());

    if(the_visual_marker_eye_core->isMeasurementPositionEnabled())
    {
        this->position_=position;
    }
    else
    {
        return 1;
    }
    return 0;
}

int VisualMarkerMeasurementCore::setVisualMarkerAttitude(const Eigen::Vector4d attitude)
{
    // Sensor Core -> to be able to do checks
    std::shared_ptr<VisualMarkerEyeCore> the_visual_marker_eye_core=std::dynamic_pointer_cast<VisualMarkerEyeCore>(this->getTheSensorCore());

    if(the_visual_marker_eye_core->isMeasurementAttitudeEnabled())
    {
        this->attitude_=attitude;
    }
    else
    {
        return 1;
    }

    return 0;
}

int VisualMarkerMeasurementCore::setVisualMarkerMeasurement(const int id, const Eigen::Vector3d position, Eigen::Vector4d attitude)
{
    int error=0;
    error+=setVisualMarkerId(id);
    error+=setVisualMarkerPosition(position);
    error+=setVisualMarkerAttitude(attitude);
    return error;
}


int VisualMarkerMeasurementCore::getVisualMarkerId() const
{
    return this->id_;
}

Eigen::Vector3d VisualMarkerMeasurementCore::getVisualMarkerPosition() const
{
    return this->position_;
}

Eigen::Vector4d VisualMarkerMeasurementCore::getVisualMarkerAttitude() const
{
    return this->attitude_;
}


Eigen::VectorXd VisualMarkerMeasurementCore::getMeasurement()
{
    // Create the Measurement
    Eigen::VectorXd the_measurement;
    the_measurement.resize(this->getTheSensorCore()->getDimensionMeasurement(), 1);
    the_measurement.setZero();

    // Sensor Core -> to be able to do checks
    std::shared_ptr<VisualMarkerEyeCore> the_visual_marker_eye_core=std::dynamic_pointer_cast<VisualMarkerEyeCore>(this->getTheSensorCore());

    // Fill
    unsigned int dimension=0;
    if(the_visual_marker_eye_core->isMeasurementPositionEnabled())
    {
        the_measurement.block<3,1>(dimension,0)=position_;
        dimension+=3;
    }
    if(the_visual_marker_eye_core->isMeasurementAttitudeEnabled())
    {
        the_measurement.block<4,1>(dimension,0)=attitude_;
        dimension+=4;
    }
}
