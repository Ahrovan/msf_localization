
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


#include "time_stamp/time_stamp.h"

#include "msf_localization_core/msf_element_core.h"


enum class MapElementTypes
{
    undefined=0,
    coded_visual_marker=1,
    world_ref_frame
};




class MapElementStateCore;


class MapElementCore : public MsfElementCore
{

public:
    MapElementCore();
    MapElementCore(MsfLocalizationCore* msf_localization_core_ptr);
    ~MapElementCore();

protected:
    int init();


protected:
    std::string map_element_name_;
public:
    int setMapElementName(std::string map_element_name);
    std::string getMapElementName() const;


    // MapElementTypes
protected:
    MapElementTypes map_element_type_;
public:
    int setMapElementType(MapElementTypes map_element_type);
    MapElementTypes getMapElementType() const;







    ///// Predict Step functions

    // None


    //// Update Step functions

    // None


};









#endif
