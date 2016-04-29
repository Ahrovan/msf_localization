#ifndef _MSF_ELEMENT_CORE_H
#define _MSF_ELEMENT_CORE_H




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


// Mutex
#include <mutex>


// Eigen
#include <Eigen/Dense>
#include <Eigen/Sparse>



#include "msf_localization_core/time_stamp.h"




enum class MsfElementCoreTypes
{
    undefined=0,
    input,
    sensor,
    robot,
    map,
    world
};




class MsfStorageCore;




class MsfElementCore
{

public:
    MsfElementCore();
    MsfElementCore(std::weak_ptr<MsfStorageCore> msf_storage_core_ptr);
    ~MsfElementCore();

protected:
    int init();
    int destroy();


    // MsfElementCoreTypes
protected:
    MsfElementCoreTypes msf_element_core_type_;
public:
    int setMsfElementCoreType(MsfElementCoreTypes msf_element_core_type);
    MsfElementCoreTypes getMsfElementCoreType() const;



    // Pointer to itself
protected:
    std::weak_ptr<MsfElementCore> msf_element_core_ptr_;
public:
    int setMsfElementCorePtr(std::weak_ptr<MsfElementCore> msf_element_core_ptr);
    std::weak_ptr<MsfElementCore> getMsfElementCoreWeakPtr() const;
    std::shared_ptr<MsfElementCore> getMsfElementCoreSharedPtr() const;


    // Pointer to the MSF Storage Core
protected:
    std::weak_ptr<MsfStorageCore> msf_storage_core_ptr_;
public:
    int setMsfStorageCorePtr(std::weak_ptr<MsfStorageCore> msf_storage_core_ptr);
    std::weak_ptr<MsfStorageCore> getMsfStorageCoreWeakPtr() const;
    std::shared_ptr<MsfStorageCore> getMsfStorageCoreSharedPtr() const;


public:
    virtual bool isCorrect();




    /// Dimensions
    // Dimension state
protected:
    int dimension_state_;
public:
    int getDimensionState() const;
    int setDimensionState(int dimension_state);

    // Dimension error state
protected:
    int dimension_error_state_;
public:
    int getDimensionErrorState() const;
    int setDimensionErrorState(int dimension_error_state);

    // Dimension parameters
protected:
    int dimension_parameters_;
public:
    int getDimensionParameters() const;
    int setDimensionParameters(int dimension_parameters);

    // Dimension error parameters
protected:
    int dimension_error_parameters_;
public:
    int getDimensionErrorParameters() const;
    int setDimensionErrorParameters(int dimension_error_parameters);


    // Dimension of the noise (error noise)
protected:
    int dimension_noise_;
public:
    int getDimensionNoise() const;
    int setDimensionNoise(int dimension_noise);



    ////// Init error state variances -> Temporal, only for the initial configuration
protected:
    Eigen::MatrixXd InitErrorStateVariance;
public:
    virtual int prepareCovarianceInitErrorState();
    Eigen::MatrixXd getCovarianceInitErrorState() const;




    //// Debug log
protected:
    std::string log_path_;
    std::ofstream log_file_;
    // mutex to protect the log file
protected:
    std::mutex log_file_mutex_;
public:
    int log(std::string log_string);

};








#endif
