
#ifndef _ROBOT_CORE_H
#define _ROBOT_CORE_H



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


#include <Eigen/Dense>
#include <Eigen/Sparse>



#include "msf_localization_core/time_stamp.h"

//#include "msf_localization_core/robot_state_core.h"


#include "msf_localization_core/msf_element_core.h"


enum class RobotTypes
{
    undefined=0,
    free_model=1
};




class RobotStateCore;



class RobotCore : public MsfElementCore
{

public:
    RobotCore();
    RobotCore(std::weak_ptr<MsfStorageCore> msf_storage_core_ptr);
    ~RobotCore();

protected:
    int init();


protected:
    std::string robot_name_;
public:
    int setRobotName(std::string robot_name);
    std::string getRobotName() const;


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

    // Dimension of the noise
protected:
    unsigned int dimensionNoise;
public:
    unsigned int getDimensionNoise() const;
    int setDimensionNoise(unsigned int dimensionNoise);


    // Robot Core
protected:
    RobotTypes robotType;
public:
    int setRobotType(RobotTypes robotType);
    RobotTypes getRobotType() const;


/*
    // Pointer to itself
protected:
//public:
    std::weak_ptr<const RobotCore> TheRobotCorePtr;
public:
    int setTheRobotCore(std::weak_ptr<const RobotCore> TheRobotCorePtr);
    std::shared_ptr<const RobotCore> getTheRobotCore() const;


    // Pointer to the MSF Storage Core
protected:
    std::weak_ptr<MsfStorageCore> TheMsfStorageCore;
public:
    int setTheMsfStorageCore(std::weak_ptr<MsfStorageCore> TheMsfStorageCore);
    std::shared_ptr<MsfStorageCore> getTheMsfStorageCore() const;
*/


    ////// Init error state variances -> Temporal, only for the initial configuration
protected:
    Eigen::MatrixXd InitErrorStateVariance;
public:
    Eigen::MatrixXd getInitErrorStateVariance() const;




public:
    virtual Eigen::SparseMatrix<double> getCovarianceNoise(const TimeStamp deltaTimeStamp) =0;



    // Prediction state function
public:
    virtual int predictState(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, const std::shared_ptr<RobotStateCore> pastState, std::shared_ptr<RobotStateCore>& predictedState) =0;

    // Jacobian
public:
    virtual int predictStateErrorStateJacobians(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, std::shared_ptr<RobotStateCore> pastState, std::shared_ptr<RobotStateCore>& predictedState) =0;



};









#endif
