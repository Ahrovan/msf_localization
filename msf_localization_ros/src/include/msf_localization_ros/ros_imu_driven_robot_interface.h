
#ifndef _ROS_IMU_DRIVEN_ROBOT_INTERFACE_H
#define _ROS_IMU_DRIVEN_ROBOT_INTERFACE_H


// ROS msg
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>


// ROS Msg geometry_msgs::Vector3
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Vector3.h>



// Time Stamp
#include "time_stamp/time_stamp.h"

// Robot Core
#include "msf_localization_core/imu_driven_robot_core.h"

// Robot State core
#include "msf_localization_core/imu_driven_robot_state_core.h"


// ROS Robot Interface
#include "msf_localization_ros/ros_robot_interface.h"


#include "pugixml/pugixml.hpp"



class RosImuDrivenRobotInterface : public RosRobotInterface, public ImuDrivenRobotCore
{
public:
    RosImuDrivenRobotInterface(ros::NodeHandle* nh, tf::TransformBroadcaster *tf_transform_broadcaster, MsfLocalizationCore* msf_localization_core_ptr);
    ~RosImuDrivenRobotInterface();



    // Output -> Publishers
protected:
    // Robot Pose with Covariance Stamped
    ros::Publisher robotPoseWithCovarianceStampedPub;
    std::string robotPoseWithCovarianceStampedTopicName;
    geometry_msgs::PoseWithCovarianceStamped robotPoseWithCovarianceStampedMsg;

    // Robot Pose Stamped
    ros::Publisher robotPoseStampedPub;
    std::string robotPoseStampedTopicName;
    geometry_msgs::PoseStamped robotPoseStampedMsg;

    // Robot Linear Speed
    ros::Publisher robotLinearSpeedStampedPub;
    std::string robotLinearSpeedStampedTopicName;
    geometry_msgs::Vector3Stamped robotLinearSpeedStampedMsg;

    // Robot Linear Acceleration
    ros::Publisher robotLinearAccelerationStampedPub;
    std::string robotLinearAccelerationStampedTopicName;
    geometry_msgs::Vector3Stamped robotLinearAccelerationStampedMsg;

    // Robot Angular Velocity
    ros::Publisher robotAngularVelocityStampedPub;
    std::string robotAngularVelocityStampedTopicName;
    geometry_msgs::Vector3Stamped robotAngularVelocityStampedMsg;



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
    int readConfig(const pugi::xml_node& robot, std::shared_ptr<ImuDrivenRobotStateCore>& robot_state_core);



};




#endif
