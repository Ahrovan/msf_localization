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




//class InputCommandCore;


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


    // Input Name
protected:
    std::string input_name_;
public:
    int setInputName(std::string input_name);
    std::string getInputName() const;


protected:
    bool flag_input_enabled_;
public:
    bool isInputEnabled() const;
    int setInputEnabled(bool flag_input_enabled);


    // Dimension input
protected:
    unsigned int dimension_input_command_;
public:
    unsigned int getDimensionInputCommand() const;
    int setDimensionInputCommand(unsigned int dimension_input_command);

    // Dimension error input
protected:
    unsigned int dimension_error_input_command_;
public:
    unsigned int getDimensionErrorInputCommand() const;
    int setDimensionErrorInputCommand(unsigned int dimension_error_input_command);





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
    virtual int predictErrorStateJacobians(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, std::shared_ptr<MapElementStateCore> pastState, std::shared_ptr<MapElementStateCore>& predictedState)=0;
*/



};



#endif
