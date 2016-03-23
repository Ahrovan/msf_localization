
#ifndef _MAP_ELEMENT_STATE_CORE_H
#define _MAP_ELEMENT_STATE_CORE_H


#include "msf_localization_core/map_element_core.h"


class MapElementStateCore
{
public:
    MapElementStateCore();
    MapElementStateCore(std::weak_ptr<const MapElementCore> the_map_element_core_ptr);
    ~MapElementStateCore();



    // Ptr to the robot core
protected:
    // It is not the owner of this Pointer. it doesn't modify the pointer
    std::weak_ptr<const MapElementCore> the_map_element_core_ptr_;
public:
    int setTheMapElementCore(std::weak_ptr<const MapElementCore> the_map_element_core_ptr);
    std::shared_ptr<const MapElementCore> getTheMapElementCore() const;
    std::shared_ptr<const MapElementCore> getTheMapElementCoreShared() const;
    std::weak_ptr<const MapElementCore> getTheMapElementCoreWeak() const;




protected:
    // TODO RobotState
    // TODO RobotErrorState



protected:
    // TODO predictState()



public:
    virtual int updateStateFromIncrementErrorState(Eigen::VectorXd increment_error_state)=0;


};



#endif
