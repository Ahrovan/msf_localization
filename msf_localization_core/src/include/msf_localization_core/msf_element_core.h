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


    // Dimension of the noise (error noise)
protected:
    unsigned int dimensionNoise;
public:
    unsigned int getDimensionNoise() const;
    int setDimensionNoise(unsigned int dimensionNoise);



    ////// Init error state variances -> Temporal, only for the initial configuration
protected:
    Eigen::MatrixXd InitErrorStateVariance;
public:
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
