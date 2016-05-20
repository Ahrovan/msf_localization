
#ifndef _ROS_ROBOT_INTERFACE_H
#define _ROS_ROBOT_INTERFACE_H



#include "msf_localization_ros/ros_interface.h"

#include "msf_localization_core/time_stamp.h"
#include "msf_localization_core/global_parameters_core.h"
#include "msf_localization_core/robot_state_core.h"


class RosRobotInterface : public RosInterface
{
protected:
    RosRobotInterface(ros::NodeHandle* nh, tf::TransformBroadcaster* tf_transform_broadcaster);
public:
    virtual ~RosRobotInterface();


    //// Name
protected:
    std::string robot_name_;
public:
    int setRobotName(const std::string robot_name);
    std::string getRobotName() const;


public:
    virtual int publish(const TimeStamp& time_stamp, const std::shared_ptr<GlobalParametersCore> world_core, const std::shared_ptr<RobotStateCore> robot_state_core, const Eigen::MatrixXd& covariance_robot_matrix)=0;


};



#endif
