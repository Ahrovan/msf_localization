
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

#include "msf_localization_core/msf_element_core.h"


enum class RobotCoreTypes
{
    undefined=0,
    free_model=1,
    imu_driven
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


    // Robot Core Type
protected:
    RobotCoreTypes robot_core_type_;
public:
    int setRobotCoreType(RobotCoreTypes robotType);
    RobotCoreTypes getRobotCoreType() const;



    ///// Covariances Getters

    // Covariance Error Parameters: Rp = Qp
public:
    //virtual Eigen::SparseMatrix<double> getCovarianceParameters()=0;

    // Covariance Noise Estimation: Qn
public:
    virtual Eigen::SparseMatrix<double> getCovarianceNoise(const TimeStamp deltaTimeStamp)=0;





    ///// Predict Step Functions

    //



    ///// Update Step Functions

    //

};









#endif
