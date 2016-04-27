
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

unsigned int MapElementCore::getDimensionState() const
{
    return this->dimensionState;
}

int MapElementCore::setDimensionState(unsigned int dimensionState)
{
    this->dimensionState=dimensionState;
    return 0;
}

unsigned int MapElementCore::getDimensionErrorState() const
{
    return this->dimensionErrorState;
}

int MapElementCore::setDimensionErrorState(unsigned int dimensionErrorState)
{
    this->dimensionErrorState=dimensionErrorState;
    return 0;
}

unsigned int MapElementCore::getDimensionParameters() const
{
    return this->dimensionParameters;
}

int MapElementCore::setDimensionParameters(unsigned int dimensionParameters)
{
    this->dimensionParameters=dimensionParameters;
    return 0;
}

unsigned int MapElementCore::getDimensionErrorParameters() const
{
    return this->dimensionErrorParameters;
}

int MapElementCore::setDimensionErrorParameters(unsigned int dimensionErrorParameters)
{
    this->dimensionErrorParameters=dimensionErrorParameters;
    return 0;
}

unsigned int MapElementCore::getDimensionNoise() const
{
    return this->dimensionNoise;
}

int MapElementCore::setDimensionNoise(unsigned int dimensionNoise)
{
    this->dimensionNoise=dimensionNoise;
    return 0;
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

/*
int MapElementCore::setTheMapElementCore(std::weak_ptr<const MapElementCore> the_map_element_core_ptr)
{
    this->the_map_element_core_ptr_=the_map_element_core_ptr;
    return 0;
}
std::shared_ptr<const MapElementCore> MapElementCore::getTheMapElementCore() const
{
    std::shared_ptr<const MapElementCore> the_map_element_core_ptr=this->the_map_element_core_ptr_.lock();
    return the_map_element_core_ptr;
}

std::shared_ptr<const MapElementCore> MapElementCore::getTheMapElementCoreShared() const
{
    std::shared_ptr<const MapElementCore> the_map_element_core_ptr=this->the_map_element_core_ptr_.lock();
    return the_map_element_core_ptr;
}

std::weak_ptr<const MapElementCore> MapElementCore::getTheMapElementCoreWeak() const
{
    return this->the_map_element_core_ptr_;
}
*/
