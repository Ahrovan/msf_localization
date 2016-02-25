

#ifndef _STATE_ESTIMATION_CORE_H
#define _STATE_ESTIMATION_CORE_H


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



#include "robot_state_core.h"

#include "sensor_state_core.h"

#include "sensor_measurement_core.h"




// Class that stores the information for every time instant
class StateEstimationCore
{
public:
    StateEstimationCore();
    ~StateEstimationCore();



public:
    bool hasState() const;



public:
    // Robot State
    //bool flagHasRobotState;
    std::shared_ptr<RobotStateCore> TheRobotStateCore;



public:
    // Sensors State
    //bool flagHasSensorState;
    std::list<std::shared_ptr<SensorStateCore> > TheListSensorStateCore;


public:
    // Covariances Matrixes
//    typedef struct
//    {
//        std::vector< std::vector<Eigen::MatrixXd> > blockElement;
//    } BlockCovariance;



//    struct
//    {
//        BlockCovariance robot;
//        std::vector<BlockCovariance> robot_sensor;
//        //std::vector<BlockCovariance> robot_map; // TODO
//        std::vector< std::vector<BlockCovariance> > sensor;
//        //std::vector< std::vector<BlockCovariance> > sensor_map; // TODO
//        //std::vector< std::vector<BlockCovariance> > map; // TODO
//    } covariancesMatrixes;


    Eigen::MatrixXd covarianceMatrix;



public:
    // Available Measurements
    //bool flagHasMeasurement;
    std::list<std::shared_ptr<SensorMeasurementCore> > TheListMeasurementCore;

public:
    bool hasMeasurement() const;


};






#endif
