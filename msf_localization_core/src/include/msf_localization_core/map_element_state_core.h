
#ifndef _MAP_ELEMENT_STATE_CORE_H
#define _MAP_ELEMENT_STATE_CORE_H


#include <Eigen/Dense>
#include <Eigen/Sparse>


#include "msf_localization_core/map_element_core.h"


class MapElementStateCore
{
public:
    MapElementStateCore();
    MapElementStateCore(std::weak_ptr<MapElementCore> the_map_element_core_ptr);
    ~MapElementStateCore();



    // Ptr to the robot core
protected:
    // It is not the owner of this Pointer. it doesn't modify the pointer
    std::weak_ptr<MapElementCore> the_map_element_core_ptr_;
public:
    int setTheMapElementCore(std::weak_ptr<MapElementCore> the_map_element_core_ptr);
    std::shared_ptr<MapElementCore> getTheMapElementCore() const;
    std::shared_ptr<MapElementCore> getTheMapElementCoreShared() const;
    std::weak_ptr<MapElementCore> getTheMapElementCoreWeak() const;




protected:
    // TODO RobotState
    // TODO RobotErrorState



protected:
    // TODO predictState()


public:
    virtual Eigen::SparseMatrix<double> getJacobianErrorState()=0;
    virtual Eigen::SparseMatrix<double> getJacobianErrorStateNoise()=0;


public:
    virtual int updateStateFromIncrementErrorState(Eigen::VectorXd increment_error_state)=0;


};



#endif
