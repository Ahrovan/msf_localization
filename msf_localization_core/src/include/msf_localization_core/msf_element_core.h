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


// List
#include <list>


// Eigen
#include <Eigen/Dense>
#include <Eigen/Sparse>



#include "msf_localization_core/block_matrix.h"

#include "msf_localization_core/time_stamp.h"


//#include "msf_localization_core/state_estimation_core.h"
#include "msf_localization_core/state_component.h"

#include "msf_localization_core/input_command_component.h"

#include "msf_localization_core/sensor_measurement_component.h"


//#include "msf_localization_core/msfLocalization.h"






enum class MsfElementCoreTypes
{
    undefined=0,
    input,
    sensor,
    robot,
    map,
    world
};


// Forward declarations!


// Test?
class MsfLocalizationCore;


class MsfElementCore
{

public:
    MsfElementCore();
    MsfElementCore(MsfLocalizationCore* msf_localization_core_ptr);
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
    int setMsfElementCorePtr(const std::weak_ptr<MsfElementCore> msf_element_core_ptr);
    MsfElementCore* getMsfElementCoreRawPtr() const;
    std::weak_ptr<MsfElementCore> getMsfElementCoreWeakPtr() const;
    std::shared_ptr<MsfElementCore> getMsfElementCoreSharedPtr() const;


    // Pointer to Msf localization core
protected:
    MsfLocalizationCore* msf_localization_core_ptr_;
public:
    void setMsfLocalizationCorePtr(MsfLocalizationCore *msf_localization_core_ptr);
    MsfLocalizationCore* getMsfLocalizationCorePtr() const;




    // Check
public:
    virtual bool isCorrect() const;




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
    Eigen::MatrixXd covariance_init_error_state_;
public:
    int prepareCovarianceInitErrorState();
protected:
    virtual int prepareCovarianceInitErrorStateSpecific()=0;
public:
    Eigen::MatrixXd getCovarianceInitErrorState() const;



    //// Getters covariances

    // TODO

    // Covariance Error Parameters: Qp = Rp
public:
    virtual Eigen::SparseMatrix<double> getCovarianceParameters()
        {throw;};

    // Covariance Noise Estimation: Qn
public:
    virtual Eigen::SparseMatrix<double> getCovarianceNoise(const TimeStamp deltaTimeStamp)
        {throw;};



    //// Predict Step Functions

    // TODO

public:
    virtual int predictState(//Time
                             const TimeStamp& previousTimeStamp, const TimeStamp& currentTimeStamp,
                             // Previous State
                             const std::shared_ptr<StateComponent>& pastState,
                             // Inputs
                             const std::shared_ptr<InputCommandComponent>& inputCommand,
                             // Predicted State
                             std::shared_ptr<StateCore>& predictedState)
    {
        std::cout<<"MsfElementCore::predictState()"<<std::endl;
        return 1;
    };


public:
    virtual int predictErrorStateJacobian(//Time
                             const TimeStamp& previousTimeStamp, const TimeStamp& currentTimeStamp,
                             // Previous State
                             const std::shared_ptr<StateComponent>& pastState,
                             // Inputs
                             const std::shared_ptr<InputCommandComponent>& inputCommand,
                             // Predicted State
                             std::shared_ptr<StateCore>& predictedState)
    {
        std::cout<<"MsfElementCore::predictErrorStateJacobian()"<<std::endl;
        return 1;
    };

protected:
    int predictErrorStateJacobianInit(// Current State
                                      const std::shared_ptr<StateComponent>& past_state,
                                      // Input Commands
                                      const std::shared_ptr<InputCommandComponent>& input_commands,
                                      // Predicted State
                                      std::shared_ptr<StateCore>& predicted_state);





    //// Update Step Functions

    // TODO

public:
    virtual int resetErrorStateJacobian(// Time
                                        const TimeStamp& current_time_stamp,
                                        // Increment Error State
                                        const Eigen::VectorXd& increment_error_state,
                                        //const BlockMatrix::MatrixDense& block_increment_error_state,
                                        // Reset Error State Jacobian
                                        //Eigen::SparseMatrix<double>& jacobian_reset_error_state
                                        // Current State
                                        std::shared_ptr<StateCore>& current_state
                                        )
    {
        std::cout<<"MsfElementCore::resetErrorStateJacobian()"<<std::endl;
        return 1;
    };




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
