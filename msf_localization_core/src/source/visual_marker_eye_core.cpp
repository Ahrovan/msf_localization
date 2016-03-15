
#include "msf_localization_core/visual_marker_eye_core.h"

// Circular Dependency
#include "msf_localization_core/msf_storage_core.h"


VisualMarkerEyeCore::VisualMarkerEyeCore() :
    SensorCore()
{
    //
    init();

    // End
    return;
}

VisualMarkerEyeCore::VisualMarkerEyeCore(std::weak_ptr<SensorCore> the_sensor_core, std::weak_ptr<MsfStorageCore> the_msf_storage_core) :
    SensorCore(the_sensor_core, the_msf_storage_core)
{
    //
    init();

    return;
}

VisualMarkerEyeCore::~VisualMarkerEyeCore()
{

    return;
}

int VisualMarkerEyeCore::init()
{
    // Sensor Type
    setSensorType(SensorTypes::visual_marker_eye);


    // Sensor name -> Default
    sensor_name_="visual_marker_eye";

    // Flags measurement
    flag_measurement_position_=false;
    flag_measurement_attitude_=false;

    // Flags estimation -> By default are considered parameters
    // none

    // State -> Again just in case
    dimensionErrorState=0;
    dimensionState=0;

    // Parameters
    dimensionParameters+=0;
    dimensionErrorParameters+=0;

    // Dimension measurements -> Again just in case
    dimensionMeasurement=0;

    // Dimension noise -> Again just in case
    dimensionNoise=0;

    // Noises measurements
    noise_measurement_position_.setZero();
    noise_measurement_attitude_.setZero();

    // Noises parameters
    // None

    // Noises estimation
    // None

    return 0;
}

bool VisualMarkerEyeCore::isMeasurementPositionEnabled() const
{
    return this->flag_measurement_position_;
}

int VisualMarkerEyeCore::enableMeasurementPosition()
{
    if(!this->flag_measurement_position_)
    {
        this->flag_measurement_position_=true;
        this->dimensionMeasurement+=3;
    }
    return 0;
}

Eigen::Matrix3d VisualMarkerEyeCore::getNoiseMeasurementPosition() const
{
    return this->noise_measurement_position_;
}

int VisualMarkerEyeCore::setNoiseMeasurementPosition(Eigen::Matrix3d noise_measurement_position)
{
    this->noise_measurement_position_=noise_measurement_position;
    return 0;
}

bool VisualMarkerEyeCore::isMeasurementAttitudeEnabled() const
{
    return this->flag_measurement_attitude_;
}

int VisualMarkerEyeCore::enableMeasurementAttitude()
{
    if(!this->flag_measurement_attitude_)
    {
        this->flag_measurement_attitude_=true;
        this->dimensionMeasurement+=4;
    }
    return 0;
}

Eigen::Matrix3d VisualMarkerEyeCore::getNoiseMeasurementAttitude() const
{
    return this->noise_measurement_attitude_;
}

int VisualMarkerEyeCore::setNoiseMeasurementAttitude(Eigen::Matrix3d noise_measurement_attitude)
{
    this->noise_measurement_attitude_=noise_measurement_attitude;
    return 0;
}

int VisualMarkerEyeCore::setMeasurement(const TimeStamp the_time_stamp, std::shared_ptr<VisualMarkerMeasurementCore> the_visual_marker_measurement)
{
    if(!isSensorEnabled())
        return 0;

    if(this->getTheMsfStorageCore()->setMeasurement(the_time_stamp, the_visual_marker_measurement))
    {
        std::cout<<"VisualMarkerEyeCore::setMeasurement() error"<<std::endl;
        return 1;
    }

    return 0;
}

Eigen::MatrixXd VisualMarkerEyeCore::getCovarianceMeasurement()
{
    Eigen::MatrixXd covariances_matrix;
    covariances_matrix.resize(this->getDimensionMeasurement(), this->getDimensionMeasurement());
    covariances_matrix.setZero();

    unsigned int dimension=0;
    if(this->isMeasurementPositionEnabled())
    {
        covariances_matrix.block<3,3>(dimension, dimension)=this->getNoiseMeasurementPosition();
        dimension+=3;
    }
    if(this->isMeasurementAttitudeEnabled())
    {
        covariances_matrix.block<3,3>(dimension, dimension)=this->getNoiseMeasurementAttitude();
        dimension+=3;
    }

    return covariances_matrix;
}

Eigen::MatrixXd VisualMarkerEyeCore::getCovarianceParameters()
{
    Eigen::MatrixXd covariances_matrix;
    covariances_matrix.resize(this->getDimensionErrorParameters(), this->getDimensionErrorParameters());
    covariances_matrix.setZero();

    unsigned int dimension=0;
    if(!this->isEstimationPositionSensorWrtRobotEnabled())
    {
        covariances_matrix.block<3,3>(dimension, dimension)=this->getNoisePositionSensorWrtRobot();
        dimension+=3;
    }
    if(!this->isEstimationAttitudeSensorWrtRobotEnabled())
    {
        covariances_matrix.block<3,3>(dimension, dimension)=this->getNoiseAttitudeSensorWrtRobot();
        dimension+=3;
    }

    return covariances_matrix;
}

int VisualMarkerEyeCore::prepareInitErrorStateVariance()
{
    int error=SensorCore::prepareInitErrorStateVariance();

    if(error)
        return error;


    int point=0;
    if(this->isEstimationPositionSensorWrtRobotEnabled())
    {
        this->InitErrorStateVariance.block<3,3>(point,point)=this->getNoisePositionSensorWrtRobot();
        point+=3;
    }
    if(this->isEstimationAttitudeSensorWrtRobotEnabled())
    {
        this->InitErrorStateVariance.block<3,3>(point,point)=this->getNoiseAttitudeSensorWrtRobot();
        point+=3;
    }



    return 0;
}
