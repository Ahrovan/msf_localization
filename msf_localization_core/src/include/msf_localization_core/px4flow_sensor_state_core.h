#ifndef _PX4FLOW_SENSOR_STATE_CORE_H
#define _PX4FLOW_SENSOR_STATE_CORE_H


#include <Eigen/Dense>


#include "msf_localization_core/sensor_state_core.h"

#include "quaternion_algebra/quaternion_algebra.h"



class Px4FlowSensorStateCore : public SensorStateCore
{
public:
    Px4FlowSensorStateCore();
    Px4FlowSensorStateCore(const std::weak_ptr<MsfElementCore> msf_element_core_ptr);
    ~Px4FlowSensorStateCore();

protected:
    int init();


    ///// Px4Flow State if enabled (or Parameters if disabled)

    // if enabled
    // State: xs=[posi_sensor_wrt_robot, att_sensor_wrt_robot]'






public:
    int updateStateFromIncrementErrorState(const Eigen::VectorXd& increment_error_state);




};


#endif
