
#ifndef _ROS_ABSOLUTE_POSE_DRIVEN_ROBOT_INTERFACE_H
#define _ROS_ABSOLUTE_POSE_DRIVEN_ROBOT_INTERFACE_H


// Pose
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>



// Time Stamp
#include "msf_localization_core/time_stamp.h"

// Robot Core
#include "msf_localization_core/absolute_pose_driven_robot_core.h"

// Robot State core
#include "msf_localization_core/absolute_pose_driven_robot_state_core.h"


// ROS Robot Interface
#include "msf_localization_ros/ros_robot_interface.h"


#include "pugixml/pugixml.hpp"



class RosAbsolutePoseDrivenRobotInterface : public RosRobotInterface, public AbsolutePoseDrivenRobotCore
{
public:
    RosAbsolutePoseDrivenRobotInterface(ros::NodeHandle* nh, tf::TransformBroadcaster *tf_transform_broadcaster, const std::weak_ptr<MsfStorageCore> the_msf_storage_core);
    ~RosAbsolutePoseDrivenRobotInterface();



    // Output -> Publishers
protected:
    /// Pose

    // Robot Pose with Covariance Stamped
    ros::Publisher robotPoseWithCovarianceStampedPub;
    std::string robotPoseWithCovarianceStampedTopicName;
    geometry_msgs::PoseWithCovarianceStamped robotPoseWithCovarianceStampedMsg;

    // Robot Pose Stamped
    ros::Publisher robotPoseStampedPub;
    std::string robotPoseStampedTopicName;
    geometry_msgs::PoseStamped robotPoseStampedMsg;




protected:
    int readParameters();

public:
    int open();


public:
    int publish(const TimeStamp& time_stamp,
                const std::shared_ptr<GlobalParametersCore>& world_core,
                const std::shared_ptr<RobotStateCore>& robot_state_core,
                const Eigen::MatrixXd& covariance_robot_matrix);


protected:
    int publishTfPoseRobotWrtWorld(const TimeStamp& time_stamp,
                                   const std::shared_ptr<GlobalParametersCore>& world_core,
                                   const std::shared_ptr<RobotStateCore>& robot_state_core);


public:
    int readConfig(const pugi::xml_node& robot, std::shared_ptr<AbsolutePoseDrivenRobotStateCore>& robot_state_core);


};


#endif
