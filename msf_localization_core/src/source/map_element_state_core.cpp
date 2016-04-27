
#include "msf_localization_core/map_element_state_core.h"

MapElementStateCore::MapElementStateCore() :
    StateCore()
{
    init();

    return;
}

MapElementStateCore::MapElementStateCore(std::weak_ptr<MsfElementCore> msf_element_core_ptr) :
    StateCore(msf_element_core_ptr)
{
    init();

    return;
}

MapElementStateCore::~MapElementStateCore()
{
    return;
}

int MapElementStateCore::init()
{
    // State core Type
    this->setStateCoreType(StateCoreTypes::map);

    return 0;
}

