
#ifndef _ABSOLUTE_POSE_SENSOR_STATE_CORE_H
#define _ABSOLUTE_POSE_SENSOR_STATE_CORE_H



#include <Eigen/Dense>


#include "msf_localization_core/sensor_state_core.h"

#include "msf_localization_core/quaternion_algebra.h"



class AbsolutePoseSensorStateCore : public SensorStateCore
{
public:
    AbsolutePoseSensorStateCore();
    AbsolutePoseSensorStateCore(const std::weak_ptr<MsfElementCore> msf_element_core_ptr);
    ~AbsolutePoseSensorStateCore();

protected:
    int init();


    ///// State if enabled (or Parameters if disabled)

    // if enabled
    // State: x_sen=[t_sens_wrt_robot, q_sens_wrt_robot]'




public:
    int updateStateFromIncrementErrorState(const Eigen::VectorXd& increment_error_state);



};




#endif
