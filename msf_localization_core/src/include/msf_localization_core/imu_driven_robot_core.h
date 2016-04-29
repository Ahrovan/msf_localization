
#ifndef _IMU_DRIVEN_ROBOT_CORE_H
#define _IMU_DRIVEN_ROBOT_CORE_H


// Math
#include "cmath"

// Time Stamp
#include "msf_localization_core/time_stamp.h"

// Quaternion algebra
#include "msf_localization_core/quaternion_algebra.h"

// Robot Core
#include "msf_localization_core/robot_core.h"

// State
#include "msf_localization_core/imu_driven_robot_state_core.h"


#include "pugixml/pugixml.hpp"



class ImuDrivenRobotCore : public RobotCore
{

public:
    ImuDrivenRobotCore();
    ImuDrivenRobotCore(std::weak_ptr<MsfStorageCore> msf_storage_core_ptr);
    ~ImuDrivenRobotCore();


protected:
    int init();



};



#endif
