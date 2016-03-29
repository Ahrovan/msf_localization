
#include "msf_localization_core/map_element_state_core.h"

MapElementStateCore::MapElementStateCore()
{
    return;
}

MapElementStateCore::MapElementStateCore(std::weak_ptr<MapElementCore> the_map_element_core_ptr) :
    MapElementStateCore()
{
    // set the core
    setTheMapElementCore(the_map_element_core_ptr);

    return;
}

MapElementStateCore::~MapElementStateCore()
{
    return;
}


int MapElementStateCore::setTheMapElementCore(std::weak_ptr<MapElementCore> the_map_element_core_ptr)
{
    this->the_map_element_core_ptr_=the_map_element_core_ptr;
    return 0;
}

std::shared_ptr<MapElementCore> MapElementStateCore::getTheMapElementCore() const
{
    std::shared_ptr<MapElementCore> the_map_element_core_ptr=this->the_map_element_core_ptr_.lock();
    return the_map_element_core_ptr;
}

std::shared_ptr<MapElementCore> MapElementStateCore::getTheMapElementCoreShared() const
{
    std::shared_ptr<MapElementCore> the_map_element_core_ptr=this->the_map_element_core_ptr_.lock();
    return the_map_element_core_ptr;
}

std::weak_ptr<MapElementCore> MapElementStateCore::getTheMapElementCoreWeak() const
{
    return this->the_map_element_core_ptr_;
}
