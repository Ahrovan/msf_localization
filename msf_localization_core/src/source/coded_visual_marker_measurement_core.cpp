
#include "msf_localization_core/coded_visual_marker_measurement_core.h"

#include "msf_localization_core/coded_visual_marker_eye_core.h"

CodedVisualMarkerMeasurementCore::CodedVisualMarkerMeasurementCore() :
    SensorMeasurementCore()
{
    // Initial values of the measurement
    this->id_=-1;
    this->position_.setZero();
    this->attitude_.setZero();

    // Initial values of the Jacobians

    // Measurement type
    measurementType=MeasurementTypes::coded_visual_marker;

    return;
}

CodedVisualMarkerMeasurementCore::CodedVisualMarkerMeasurementCore(std::weak_ptr<SensorCore> the_sensor_core) :
    SensorMeasurementCore(the_sensor_core)
{

    return;
}

CodedVisualMarkerMeasurementCore::~CodedVisualMarkerMeasurementCore()
{

    return;
}

int CodedVisualMarkerMeasurementCore::setVisualMarkerId(const int id)
{
    this->id_=id;
    return 0;
}

int CodedVisualMarkerMeasurementCore::setVisualMarkerPosition(const Eigen::Vector3d position)
{
    // Sensor Core -> to be able to do checks
    std::shared_ptr<CodedVisualMarkerEyeCore> the_visual_marker_eye_core=std::dynamic_pointer_cast<CodedVisualMarkerEyeCore>(this->getTheSensorCore());

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

int CodedVisualMarkerMeasurementCore::setVisualMarkerAttitude(const Eigen::Vector4d attitude)
{
    // Sensor Core -> to be able to do checks
    std::shared_ptr<CodedVisualMarkerEyeCore> the_visual_marker_eye_core=std::dynamic_pointer_cast<CodedVisualMarkerEyeCore>(this->getTheSensorCore());

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

int CodedVisualMarkerMeasurementCore::setVisualMarkerMeasurement(const int id, const Eigen::Vector3d position, Eigen::Vector4d attitude)
{
    int error=0;
    error+=setVisualMarkerId(id);
    error+=setVisualMarkerPosition(position);
    error+=setVisualMarkerAttitude(attitude);
    return error;
}


int CodedVisualMarkerMeasurementCore::getVisualMarkerId() const
{
    return this->id_;
}

Eigen::Vector3d CodedVisualMarkerMeasurementCore::getVisualMarkerPosition() const
{
    return this->position_;
}

Eigen::Vector4d CodedVisualMarkerMeasurementCore::getVisualMarkerAttitude() const
{
    return this->attitude_;
}

Eigen::VectorXd CodedVisualMarkerMeasurementCore::getInnovation(std::shared_ptr<SensorMeasurementCore> theMatchedMeasurementI, std::shared_ptr<SensorMeasurementCore> thePredictedMeasurementI)
{
    // Create the Measurement
    Eigen::VectorXd the_innovation;
    the_innovation.resize(this->getTheSensorCore()->getDimensionErrorMeasurement(), 1);
    the_innovation.setZero();

    // Check
    if(theMatchedMeasurementI->getTheSensorCore() != thePredictedMeasurementI->getTheSensorCore())
    {
        std::cout<<"CodedVisualMarkerMeasurementCore::getInnovation() error"<<std::endl;
    }

    // Cast
    std::shared_ptr<CodedVisualMarkerEyeCore> the_visual_marker_eye_core=std::dynamic_pointer_cast<CodedVisualMarkerEyeCore>(theMatchedMeasurementI->getTheSensorCore());

    // Cast
    std::shared_ptr<CodedVisualMarkerMeasurementCore> theMatchedMeasurement=std::dynamic_pointer_cast<CodedVisualMarkerMeasurementCore>(theMatchedMeasurementI);
    std::shared_ptr<CodedVisualMarkerMeasurementCore> thePredictedMeasurement=std::dynamic_pointer_cast<CodedVisualMarkerMeasurementCore>(thePredictedMeasurementI);

    // Fill
    unsigned int dimension=0;
    if(the_visual_marker_eye_core->isMeasurementPositionEnabled())
    {
        the_innovation.block<3,1>(dimension,0)=theMatchedMeasurement->position_-thePredictedMeasurement->position_;
        dimension+=3;

//        logFile<<"theMatchedMeasurement->position_: "<<theMatchedMeasurement->position_.transpose()<<std::endl;
//        logFile<<"thePredictedMeasurement->position_: "<<thePredictedMeasurement->position_.transpose()<<std::endl;
    }
    if(the_visual_marker_eye_core->isMeasurementAttitudeEnabled())
    {
        Eigen::Vector4d quat_innov_attitude=Quaternion::cross(Quaternion::inv(thePredictedMeasurement->attitude_), theMatchedMeasurement->attitude_);

        the_innovation.block<3,1>(dimension,0)=2*quat_innov_attitude.block<3,1>(1,0);
        dimension+=3;

//        logFile<<"theMatchedMeasurement->attitude_: "<<theMatchedMeasurement->attitude_.transpose()<<std::endl;
//        logFile<<"thePredictedMeasurement->attitude_: "<<thePredictedMeasurement->attitude_.transpose()<<std::endl;
    }


    return the_innovation;
}


Eigen::VectorXd CodedVisualMarkerMeasurementCore::getMeasurement()
{
    // Create the Measurement
    Eigen::VectorXd the_measurement;
    the_measurement.resize(this->getTheSensorCore()->getDimensionMeasurement(), 1);
    the_measurement.setZero();

    // Sensor Core -> to be able to do checks
    std::shared_ptr<CodedVisualMarkerEyeCore> the_visual_marker_eye_core=std::dynamic_pointer_cast<CodedVisualMarkerEyeCore>(this->getTheSensorCore());

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

    return the_measurement;
}
