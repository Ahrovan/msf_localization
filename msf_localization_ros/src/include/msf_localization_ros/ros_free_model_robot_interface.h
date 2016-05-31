
#ifndef _ROS_FREE_MODEL_ROBOT_INTERFACE_H
#define _ROS_FREE_MODEL_ROBOT_INTERFACE_H


// Pose
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "msf_localization_ros_srvs/GetPoseWithCovarianceByStamp.h"

// Velocities
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>

// Accelerations
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <geometry_msgs/AccelStamped.h>


// ROS Msg geometry_msgs::Vector3
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Vector3.h>



// Time Stamp
#include "msf_localization_core/time_stamp.h"

// Robot Core
#include "msf_localization_core/free_model_robot_core.h"

// Robot State core
#include "msf_localization_core/free_model_robot_state_core.h"


// ROS Robot Interface
#include "msf_localization_ros/ros_robot_interface.h"


#include "pugixml/pugixml.hpp"



class RosFreeModelRobotInterface : public RosRobotInterface, public FreeModelRobotCore
{
public:
    RosFreeModelRobotInterface(ros::NodeHandle* nh, tf::TransformBroadcaster *tf_transform_broadcaster, MsfLocalizationCore* msf_localization_core_ptr);
    ~RosFreeModelRobotInterface();



    // Output -> Publishers and service servers
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

    // Robot Pose With Covariance Stamped Srv
    ros::ServiceServer robot_pose_with_covariance_stamped_srv_;
    std::string robot_pose_with_covariance_stamped_srv_name_;
    msf_localization_ros_srvs::GetPoseWithCovarianceByStamp robot_pose_with_covariance_stamped_srv_msg_;
    bool getPoseWithCovarianceByStamp(msf_localization_ros_srvs::GetPoseWithCovarianceByStamp::Request  &req,
                                      msf_localization_ros_srvs::GetPoseWithCovarianceByStamp::Response &res);


    /// Velocities

    // Velocities stamped
    ros::Publisher robot_velocities_stamped_pub_;
    std::string robot_velocities_stamped_topic_name_;
    geometry_msgs::TwistStamped robot_velocities_stamped_msg_;

    // Velocities with covariance stamped
    ros::Publisher robot_velocities_with_covariance_stamped_pub_;
    std::string robot_velocities_with_covariance_stamped_topic_name_;
    geometry_msgs::TwistWithCovarianceStamped robot_velocities_with_covariance_stamped_msg_;


    // Robot Linear Speed
    ros::Publisher robotLinearSpeedStampedPub;
    std::string robotLinearSpeedStampedTopicName;
    geometry_msgs::Vector3Stamped robotLinearSpeedStampedMsg;

    // Robot Angular Velocity
    ros::Publisher robotAngularVelocityStampedPub;
    std::string robotAngularVelocityStampedTopicName;
    geometry_msgs::Vector3Stamped robotAngularVelocityStampedMsg;


    /// Accelerations

    // Accelerations stamped
    ros::Publisher robot_accelerations_stamped_pub_;
    std::string robot_accelerations_stamped_topic_name_;
    geometry_msgs::AccelStamped robot_accelerations_stamped_msg_;

    // Accelerations with covariance stamped
    ros::Publisher robot_accelerations_with_covariance_stamped_pub_;
    std::string robot_accelerations_with_covariance_stamped_topic_name_;
    geometry_msgs::AccelWithCovarianceStamped robot_accelerations_with_covariance_stamped_msg_;


    // Robot Linear Acceleration
    ros::Publisher robotLinearAccelerationStampedPub;
    std::string robotLinearAccelerationStampedTopicName;
    geometry_msgs::Vector3Stamped robotLinearAccelerationStampedMsg;

    // Robot Angular Acceleration
    ros::Publisher robotAngularAccelerationStampedPub;
    std::string robotAngularAccelerationStampedTopicName;
    geometry_msgs::Vector3Stamped robotAngularAccelerationStampedMsg;




protected:
    int readParameters();

public:
    int open();


protected:
    int setRobotPoseMsg(const std::shared_ptr<RobotStateCore>& robot_state_core,
                        geometry_msgs::Pose& robot_pose_msg);
    int setRobotPoseWithCovarianceMsg(const std::shared_ptr<RobotStateCore>& robot_state_core,
                                      const Eigen::MatrixXd& covariance_robot_matrix,
                                      geometry_msgs::PoseWithCovariance& robot_pose_msg);


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
    int readConfig(const pugi::xml_node& robot, std::shared_ptr<FreeModelRobotStateCore>& robot_state_core);


};







#endif
