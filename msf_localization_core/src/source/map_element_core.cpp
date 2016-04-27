
#include "msf_localization_core/map_element_core.h"


#include "msf_localization_core/msf_storage_core.h"



MapElementCore::MapElementCore() :
    MsfElementCore()
{
    init();

    return;
}

MapElementCore::MapElementCore(std::weak_ptr<MsfStorageCore> msf_storage_core_ptr) :
    MsfElementCore(msf_storage_core_ptr)
{
    init();

    return;
}

MapElementCore::~MapElementCore()
{

    return;
}

int MapElementCore::init()
{
    // Dimensions
    dimensionState=0;
    dimensionErrorState=0;
    dimensionParameters=0;
    dimensionErrorParameters=0;
    dimensionNoise=0;


    // Element Type
    this->setMsfElementCoreType(MsfElementCoreTypes::map);


    // Default Type
    map_element_type_=MapElementTypes::undefined;

    // Default name
    map_element_name_="map_element";

}

int MapElementCore::setMapElementName(std::string map_element_name)
{
    this->map_element_name_=map_element_name;
    return 0;
}

std::string MapElementCore::getMapElementName() const
{
    return this->map_element_name_;
}

int MapElementCore::setMapElementType(MapElementTypes map_element_type)
{
    this->map_element_type_=map_element_type;
    return 0;
}
MapElementTypes MapElementCore::getMapElementType() const
{
    return this->map_element_type_;
}

