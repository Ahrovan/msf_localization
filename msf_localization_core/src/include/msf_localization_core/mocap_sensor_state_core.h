
#ifndef _MOCAP_STATE_SENSOR_CORE_H
#define _MOCAP_STATE_SENSOR_CORE_H



#include <Eigen/Dense>


#include "msf_localization_core/sensor_state_core.h"

#include "msf_localization_core/quaternion_algebra.h"



class MocapSensorStateCore : public SensorStateCore
{
public:
    MocapSensorStateCore();
    MocapSensorStateCore(std::weak_ptr<MsfElementCore> msf_element_core_ptr);
    ~MocapSensorStateCore();

protected:
    int init();


    ///// State if enabled (or Parameters if disabled)

    // if enabled
    // State: x_sen=[t_sens_wrt_robot, q_sens_wrt_robot]'




public:
    int updateStateFromIncrementErrorState(const Eigen::VectorXd& increment_error_state);



};




#endif
