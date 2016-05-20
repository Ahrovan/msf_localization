
#ifndef _WORLD_REFERENCE_FRAME_STATE_CORE_H
#define _WORLD_REFERENCE_FRAME_STATE_CORE_H



#include <Eigen/Dense>
#include <Eigen/Sparse>


#include "msf_localization_core/map_element_state_core.h"

#include "msf_localization_core/map_element_core.h"


#include "msf_localization_core/quaternion_algebra.h"



class WorldReferenceFrameStateCore : public MapElementStateCore
{
public:
    WorldReferenceFrameStateCore();
    WorldReferenceFrameStateCore(const std::weak_ptr<MsfElementCore> msf_element_core_ptr);
    ~WorldReferenceFrameStateCore();

protected:
    int init();


    // State: xL=[pos (3/3), attit (4/3)]'

protected:
public:
    Eigen::Vector3d position_mocap_world_wrt_world_;
public:
    Eigen::Vector3d getPositionMocapWorldWrtWorld() const;
    int setPositionMocapWorldWrtWorld(const Eigen::Vector3d& position_mocap_world_wrt_world);



protected:
public:
    Eigen::Vector4d attitude_mocap_world_wrt_world_;
public:
    Eigen::Vector4d getAttitudeMocapWorldWrtWorld() const;
    int setAttitudeMocapWorldWrtWorld(const Eigen::Vector4d& attitudemocap_world_wrt_world);




public:
    int updateStateFromIncrementErrorState(const Eigen::VectorXd& increment_error_state);

};




#endif
