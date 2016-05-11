
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
    GlobalParametersCore(std::weak_ptr<MsfStorageCore> msf_storage_core_ptr);
    virtual ~GlobalParametersCore();


protected:
    int init();


protected:
    std::string world_name_;
public:
    std::string getWorldName() const;
    int setWorldName(std::string world_name);


public:
    int readConfig(pugi::xml_node global_parameters, std::shared_ptr<GlobalParametersStateCore>& GlobalParametersInitStateCore);



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
    int setNoiseGravity(Eigen::Matrix3d noiseGravity);



    ////// Init error state variances -> Temporal, only for the initial configuration

public:
    int prepareCovarianceInitErrorStateSpecific();



    ///// Covariances Getters

    // Covariance Error Parameters: Rp = Qp
public:
    Eigen::SparseMatrix<double> getCovarianceParameters();
    //Eigen::MatrixXd getCovarianceGlobalParameters();

    // Covariance Noise Estimation: Qn
public:
    Eigen::SparseMatrix<double> getCovarianceNoise(const TimeStamp deltaTimeStamp);





    //// Predict Step Functions


    // Prediction state function: f()
public:
    int predictState(//Time
                     const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp,
                     // Previous State
                     const std::shared_ptr<StateEstimationCore> pastState,
                     // Inputs
                     const std::shared_ptr<InputCommandComponent> inputCommand,
                     // Predicted State
                     std::shared_ptr<StateCore>& predictedState);

protected:
    int predictStateSpecific(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp,
                             const std::shared_ptr<GlobalParametersStateCore> pastState,
                             std::shared_ptr<GlobalParametersStateCore>& predictedState);

    // int predictStateSpecificCore();



    // Jacobian: F
public:
    int predictErrorStateJacobian(//Time
                                 const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp,
                                 // Previous State
                                 const std::shared_ptr<StateEstimationCore> pastState,
                                  // Inputs
                                  const std::shared_ptr<InputCommandComponent> inputCommand,
                                 // Predicted State
                                 std::shared_ptr<StateCore>& predictedState);

protected:
    int predictErrorStateJacobianSpecific(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp,
                                          const std::shared_ptr<GlobalParametersStateCore> pastState,
                                          std::shared_ptr<GlobalParametersStateCore>& predictedState);

    // int predictErrorStateJacobianSpecificCore();



    //// Update Step Functions

    // NONE


};




#endif
