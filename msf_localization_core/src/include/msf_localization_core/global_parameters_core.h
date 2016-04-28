
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
    int prepareCovarianceInitErrorState();



public:
    Eigen::MatrixXd getCovarianceGlobalParameters();


public:
    int readConfig(pugi::xml_node global_parameters, std::shared_ptr<GlobalParametersStateCore>& GlobalParametersInitStateCore);

};








#endif
