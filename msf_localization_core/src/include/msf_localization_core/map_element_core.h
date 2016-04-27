
#ifndef _MAP_ELEMENT_CORE_H
#define _MAP_ELEMENT_CORE_H



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


enum class MapElementTypes
{
    undefined=0,
    coded_visual_marker=1
};




class MapElementStateCore;


class MapElementCore : public MsfElementCore
{

public:
    MapElementCore();
    MapElementCore(std::weak_ptr<MsfStorageCore> msf_storage_core_ptr);
    ~MapElementCore();

protected:
    int init();


protected:
    std::string map_element_name_;
public:
    int setMapElementName(std::string map_element_name);
    std::string getMapElementName() const;


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


    // Dimension of the map noise
protected:
    unsigned int dimensionNoise;
public:
    unsigned int getDimensionNoise() const;
    int setDimensionNoise(unsigned int dimensionNoise);



    // MapElementTypes
protected:
    MapElementTypes map_element_type_;
public:
    int setMapElementType(MapElementTypes map_element_type);
    MapElementTypes getMapElementType() const;


    /*
    // Pointer to itself
protected:
    std::weak_ptr<const MapElementCore> the_map_element_core_ptr_;
public:
    int setTheMapElementCore(std::weak_ptr<const MapElementCore> the_map_element_core_ptr);
    std::shared_ptr<const MapElementCore> getTheMapElementCore() const;
    std::shared_ptr<const MapElementCore> getTheMapElementCoreShared() const;
    std::weak_ptr<const MapElementCore> getTheMapElementCoreWeak() const;
    */


    ////// Init error state variances -> Temporal, only for the initial configuration
//protected:
//    Eigen::MatrixXd InitErrorStateVariance;
//public:
//    Eigen::MatrixXd getInitErrorStateVariance() const;
//public:
//    virtual int prepareInitErrorStateVariance()=0;

public:
    virtual Eigen::MatrixXd getInitCovarianceErrorState()=0;




    ///// Get Covariances as a Eigen::MatrixXd
public:
    //virtual Eigen::SparseMatrix<double> getCovarianceMeasurement()=0;
    virtual Eigen::SparseMatrix<double> getCovarianceParameters()=0;


public:
    virtual Eigen::SparseMatrix<double> getCovarianceNoise(const TimeStamp deltaTimeStamp) const=0;






    ///// Predict functions

    // Prediction state function
public:
    virtual int predictState(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, const std::shared_ptr<MapElementStateCore> pastState, std::shared_ptr<MapElementStateCore>& predictedState)=0;

    // Jacobian
public:
    virtual int predictStateErrorStateJacobians(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, std::shared_ptr<MapElementStateCore> pastState, std::shared_ptr<MapElementStateCore>& predictedState)=0;




};









#endif
