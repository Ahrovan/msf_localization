#ifndef _INPUT_CORE_H
#define _INPUT_CORE_H




//I/O stream
//std::cout
#include <iostream>


//String
//std::string, std::getline()
#include <string>

//String stream
//std::istringstream
#include <sstream>

//File Stream
//std::ofstream, std::ifstream
#include <fstream>


// Memory
#include <memory>



#include <Eigen/Dense>
#include <Eigen/Sparse>


#include "msf_localization_core/time_stamp.h"

#include "msf_localization_core/msf_element_core.h"


enum class InputTypes
{
    undefined=0,
    imu=1
};




//class MapElementStateCore;


class InputCore : public MsfElementCore
{

public:
    InputCore();
    InputCore(std::weak_ptr<MsfStorageCore> msf_storage_core_ptr);
    ~InputCore();

protected:
    int init();


    // InputTypes
protected:
    InputTypes input_type_;
public:
    int setInputType(InputTypes input_type);
    InputTypes getInputType() const;


/*
    // Dimension state
protected:
    unsigned int dimensionState;
public:
    unsigned int getDimensionState() const;
    int setDimensionState(unsigned int dimensionState);

    // Dimension error state
protected:
    unsigned int dimensionErrorState;
public:
    unsigned int getDimensionErrorState() const;
    int setDimensionErrorState(unsigned int dimensionErrorState);

    // Dimension parameters
protected:
    unsigned int dimensionParameters;
public:
    unsigned int getDimensionParameters() const;
    int setDimensionParameters(unsigned int dimensionParameters);

    // Dimension error parameters
protected:
    unsigned int dimensionErrorParameters;
public:
    unsigned int getDimensionErrorParameters() const;
    int setDimensionErrorParameters(unsigned int dimensionErrorParameters);


    // Dimension of the map noise
protected:
    unsigned int dimensionNoise;
public:
    unsigned int getDimensionNoise() const;
    int setDimensionNoise(unsigned int dimensionNoise);
*/





/*
    ////// Init error state variances -> Temporal, only for the initial configuration
public:
    virtual Eigen::MatrixXd getInitCovarianceErrorState()=0;




    ///// Get Covariances as a Eigen::MatrixXd
public:
    //virtual Eigen::SparseMatrix<double> getCovarianceMeasurement()=0;
    virtual Eigen::SparseMatrix<double> getCovarianceParameters()=0;


public:
    virtual Eigen::SparseMatrix<double> getCovarianceNoise(const TimeStamp deltaTimeStamp) const=0;
*/





    ///// Predict functions
/*
    // Prediction state function
public:
    virtual int predictState(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, const std::shared_ptr<MapElementStateCore> pastState, std::shared_ptr<MapElementStateCore>& predictedState)=0;

    // Jacobian
public:
    virtual int predictStateErrorStateJacobians(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, std::shared_ptr<MapElementStateCore> pastState, std::shared_ptr<MapElementStateCore>& predictedState)=0;
*/



};



#endif
