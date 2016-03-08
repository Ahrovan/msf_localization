
#ifndef _GLOBAL_PARAMETERS_CORE_H
#define _GLOBAL_PARAMETERS_CORE_H



//I/O stream
//std::cout
#include <iostream>


#include <memory>


#include <Eigen/Dense>






class MsfStorageCore;


class GlobalParametersCore
{


public:
    GlobalParametersCore();
    GlobalParametersCore(std::weak_ptr<MsfStorageCore> TheMsfStorageCore);
    virtual ~GlobalParametersCore();


protected:
    std::string world_name_;
public:
    std::string getWorldName() const;
    int setWorldName(std::string world_name);


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



    // Pointer to itself
protected:
    std::weak_ptr<const GlobalParametersCore> TheGlobalParametersCorePtr;
public:
    int setTheGlobalParametersCore(std::weak_ptr<const GlobalParametersCore> TheGlobalParametersCorePtr);
    std::shared_ptr<const GlobalParametersCore> getTheGlobalParametersCore() const;


    // Pointer to the MSF Storage Core
protected:
    std::weak_ptr<MsfStorageCore> TheMsfStorageCore;
public:
    int setTheMsfStorageCore(std::weak_ptr<MsfStorageCore> TheMsfStorageCore);
    std::shared_ptr<MsfStorageCore> getTheMsfStorageCore() const;






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




public:
    Eigen::MatrixXd getCovarianceGlobalParameters();


};








#endif
