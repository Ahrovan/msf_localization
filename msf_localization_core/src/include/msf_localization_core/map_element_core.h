
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



enum class MapElementTypes
{
    undefined=0,
    coded_visual_marker=1
};




class MsfStorageCore;

class MapElementCore
{

public:
    MapElementCore();
    MapElementCore(std::weak_ptr<MapElementCore> the_map_element_core_ptr, std::weak_ptr<MsfStorageCore> TheMsfStorageCore);
    virtual ~MapElementCore();


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



    // Robot Core
protected:
    MapElementTypes map_element_type_;
public:
    int setMapElementType(MapElementTypes map_element_type);
    MapElementTypes getMapElementType() const;


    // Pointer to itself
protected:
    std::weak_ptr<const MapElementCore> the_map_element_core_ptr_;
public:
    int setTheMapElementCore(std::weak_ptr<const MapElementCore> the_map_element_core_ptr);
    std::shared_ptr<const MapElementCore> getTheMapElementCore() const;
    std::shared_ptr<const MapElementCore> getTheMapElementCoreShared() const;
    std::weak_ptr<const MapElementCore> getTheMapElementCoreWeak() const;


    // Pointer to the MSF Storage Core
protected:
    std::weak_ptr<MsfStorageCore> TheMsfStorageCore;
public:
    int setTheMsfStorageCore(std::weak_ptr<MsfStorageCore> TheMsfStorageCore);
    std::shared_ptr<MsfStorageCore> getTheMsfStorageCore() const;


    ////// Init error state variances -> Temporal, only for the initial configuration
protected:
    Eigen::MatrixXd InitErrorStateVariance;
public:
    Eigen::MatrixXd getInitErrorStateVariance() const;





    //// Debug log
protected:
    std::string logPath;
    std::ofstream logFile;
    // mutex to protect the log file
protected:
    std::mutex TheLogFileMutex;
public:
    int log(std::string logString);

};









#endif