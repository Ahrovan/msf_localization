
#ifndef _CODED_VISUAL_MARKER_MEASUREMENT_CORE_H
#define _CODED_VISUAL_MARKER_MEASUREMENT_CORE_H



#include "msf_localization_core/sensor_measurement_core.h"


class CodedVisualMarkerMeasurementCore : public SensorMeasurementCore
{
public:
    CodedVisualMarkerMeasurementCore();
    CodedVisualMarkerMeasurementCore(std::weak_ptr<SensorCore> the_sensor_core);
public:
    ~CodedVisualMarkerMeasurementCore();



    // Measurement
protected:
    // Id of the visual marker -> Not measurement, but id
    int id_;
    // Position of the visual marker wrt visual marker detector
    Eigen::Vector3d position_;
    // Attitude of the visual marker wrt visual marker detector
    Eigen::Vector4d attitude_;


public:
    int setVisualMarkerId(const int id);
    int setVisualMarkerPosition(const Eigen::Vector3d position);
    int setVisualMarkerAttitude(const Eigen::Vector4d attitude);
    int setVisualMarkerMeasurement(const int id, const Eigen::Vector3d position, Eigen::Vector4d attitude);
public:
    int getVisualMarkerId() const;
    Eigen::Vector3d getVisualMarkerPosition() const;
    Eigen::Vector4d getVisualMarkerAttitude() const;




    //// Get the innovation vector as an Eigen::VectorXd
public:
    Eigen::VectorXd getInnovation(std::shared_ptr<SensorMeasurementCore> theMatchedMeasurement, std::shared_ptr<SensorMeasurementCore> thePredictedMeasurement);


    //// Get the full measurement as an Eigen::VectorXd
public:
    Eigen::VectorXd getMeasurement();

};



#endif
