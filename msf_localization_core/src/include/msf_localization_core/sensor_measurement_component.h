
#ifndef _SENSOR_MEASUREMENT_COMPONENT_H
#define _SENSOR_MEASUREMENT_COMPONENT_H


//I/O stream
//std::cout
#include <iostream>

//String
//std::string, std::getline()
#include <string>



//Vector
//std::vector
#include <vector>

// List
#include <list>


#include <memory>


#include <Eigen/Dense>




/// Forward declarations

// Measurements
class SensorMeasurementCore;





class SensorMeasurementComponent
{
public:
    SensorMeasurementComponent();
    ~SensorMeasurementComponent();



    /// Sensor Measurement Component

    // Check
public:
    bool hasMeasurement() const;


    // Dimension total of measurement and error measurement
public:
    int getDimensionMeasurement() const;
    int getDimensionErrorMeasurement() const;


    // Avaliable Inputs
public:
    std::list< std::shared_ptr<SensorMeasurementCore> > list_sensor_measurement_core_;




};



#endif
