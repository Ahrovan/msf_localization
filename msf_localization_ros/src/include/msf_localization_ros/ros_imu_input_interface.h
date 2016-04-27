
#ifndef _ROS_IMU_INPUT_INTERFACE_H
#define _ROS_IMU_INPUT_INTERFACE_H



#include "msf_localization_ros/ros_interface.h"

#include "msf_localization_core/imu_input_core.h"


class RosImuInputInterface : public RosInterface, public ImuInputCore
{



};



#endif
