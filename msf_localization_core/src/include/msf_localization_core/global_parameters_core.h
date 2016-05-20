
#ifndef _GLOBAL_PARAMETERS_CORE_H
#define _GLOBAL_PARAMETERS_CORE_H



//I/O stream
//std::cout
#include <iostream>


#include <memory>


#include <Eigen/Dense>


#include "msf_localization_core/msf_element_core.h"



#include "pugixml/pugixml.hpp"



class GlobalParametersStateCore;


class GlobalParametersCore : public MsfElementCore
{


public:
    GlobalParametersCore();
    GlobalParametersCore(const std::weak_ptr<MsfStorageCore> msf_storage_core_ptr);
    virtual ~GlobalParametersCore();


protected:
    int init();


protected:
    std::string world_name_;
public:
    std::string getWorldName() const;
    int setWorldName(const std::string& world_name);


public:
    int readConfig(const pugi::xml_node& global_parameters, std::shared_ptr<GlobalParametersStateCore>& GlobalParametersInitStateCore);



    ///// State estimation and parameters



    /// Gravity: g (3x1)

    // Gravity: gx, gy, gz
protected:
    bool flagEstimationGravity;
public:
    bool isEstimationGravityEnabled() const;
    int enableEstimationGravity();
    int enableParameterGravity();


    // Gravity: Covariance (if enabled estimation -> P; if no enabled estimation -> Sigma_mu)
protected:
    Eigen::Matrix3d noiseGravity;
public:
    Eigen::Matrix3d getNoiseGravity() const;
    int setNoiseGravity(const Eigen::Matrix3d& noiseGravity);



    ////// Init error state variances -> Temporal, only for the initial configuration

public:
    int prepareCovarianceInitErrorStateSpecific();



    ///// Covariances Getters

    // Covariance Error Parameters: Rp = Qp
public:
    Eigen::SparseMatrix<double> getCovarianceParameters();

    // Covariance Noise Estimation: Qn
public:
    Eigen::SparseMatrix<double> getCovarianceNoise(const TimeStamp deltaTimeStamp);





    //// Predict Step Functions


    // Prediction state function: f()
public:
    int predictState(//Time
                     const TimeStamp& previousTimeStamp, const TimeStamp& currentTimeStamp,
                     // Previous State
                     const std::shared_ptr<StateEstimationCore>& pastState,
                     // Inputs
                     const std::shared_ptr<InputCommandComponent>& inputCommand,
                     // Predicted State
                     std::shared_ptr<StateCore>& predictedState);

protected:
    int predictStateSpecific(const TimeStamp& previousTimeStamp, const TimeStamp& currentTimeStamp,
                             const GlobalParametersStateCore* pastState,
                             GlobalParametersStateCore*& predictedState);

    // int predictStateSpecificCore();



    // Jacobian: F
public:
    int predictErrorStateJacobian(//Time
                                 const TimeStamp& previousTimeStamp, const TimeStamp& currentTimeStamp,
                                 // Previous State
                                 const std::shared_ptr<StateEstimationCore>& past_state,
                                  // Inputs
                                  const std::shared_ptr<InputCommandComponent>& input_command,
                                 // Predicted State
                                 std::shared_ptr<StateCore>& predicted_state);

protected:
    int predictErrorStateJacobianSpecific(const TimeStamp& previousTimeStamp, const TimeStamp& currentTimeStamp,
                                          const GlobalParametersStateCore* pastState,
                                          GlobalParametersStateCore*& predictedState,
                                          // Jacobians Error State: Fx, Fp
                                          // World
                                          Eigen::SparseMatrix<double>& jacobian_error_state_wrt_world_error_state,
                                          Eigen::SparseMatrix<double>& jacobian_error_state_wrt_world_error_parameters
                                          // Jacobians Noise: Hn
                                          // TODO
                                          );

    // int predictErrorStateJacobianSpecificCore();



    //// Update Step Functions

    // NONE

public:
    int resetErrorStateJacobian(// Time
                                const TimeStamp& current_time_stamp,
                                // Increment Error State
                                const Eigen::VectorXd& increment_error_state,
                                // Current State
                                std::shared_ptr<StateCore>& current_state
                                );

};




#endif
