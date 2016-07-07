
#include "msf_localization_ros/ros_imu_driven_robot_interface.h"

RosImuDrivenRobotInterface::RosImuDrivenRobotInterface(ros::NodeHandle* nh, tf::TransformBroadcaster *tf_transform_broadcaster, MsfLocalizationCore* msf_localization_core_ptr) :
    RosRobotInterface(nh, tf_transform_broadcaster),
    ImuDrivenRobotCore(msf_localization_core_ptr)
{

    return;
}

RosImuDrivenRobotInterface::~RosImuDrivenRobotInterface()
{
    return;
}

int RosImuDrivenRobotInterface::readParameters()
{
    // Topic names
    //
    ros::param::param<std::string>("~robot_pose_with_cov_topic_name", robotPoseWithCovarianceStampedTopicName, "msf_localization/robot_pose_cov");
    std::cout<<"\t robot_pose_with_cov_topic_name="<<robotPoseWithCovarianceStampedTopicName<<std::endl;
    //
    ros::param::param<std::string>("~robot_pose_topic_name", robotPoseStampedTopicName, "msf_localization/robot_pose");
    std::cout<<"\t robot_pose_topic_name="<<robotPoseStampedTopicName<<std::endl;
    //
    ros::param::param<std::string>("~robot_linear_speed_topic_name", robotLinearSpeedStampedTopicName, "msf_localization/robot_linear_speed");
    std::cout<<"\t robot_linear_speed_topic_name="<<robotLinearSpeedStampedTopicName<<std::endl;
    //
    ros::param::param<std::string>("~robot_linear_acceleration_topic_name", robotLinearAccelerationStampedTopicName, "msf_localization/robot_linear_acceleration");
    std::cout<<"\t robot_linear_acceleration_topic_name="<<robotLinearAccelerationStampedTopicName<<std::endl;
    //
    ros::param::param<std::string>("~robot_angular_velocity_topic_name", robotAngularVelocityStampedTopicName, "msf_localization/robot_angular_velocity");
    std::cout<<"\t robot_angular_velocity_topic_name="<<robotAngularVelocityStampedTopicName<<std::endl;

    return 0;
}

int RosImuDrivenRobotInterface::open()
{
    // Read ROS Parameters
    this->readParameters();


    // Publishers
    //
    robotPoseWithCovarianceStampedPub = nh->advertise<geometry_msgs::PoseWithCovarianceStamped>(robotPoseWithCovarianceStampedTopicName, 1, true);
    //
    robotPoseStampedPub = nh->advertise<geometry_msgs::PoseStamped>(robotPoseStampedTopicName, 1, true);
    //
    robotLinearSpeedStampedPub = nh->advertise<geometry_msgs::Vector3Stamped>(robotLinearSpeedStampedTopicName, 1, true);
    //
    robotLinearAccelerationStampedPub = nh->advertise<geometry_msgs::Vector3Stamped>(robotLinearAccelerationStampedTopicName, 1, true);
    //
    robotAngularVelocityStampedPub = nh->advertise<geometry_msgs::Vector3Stamped>(robotAngularVelocityStampedTopicName, 1, true);


    return 0;
}

int RosImuDrivenRobotInterface::publish(const TimeStamp& time_stamp, const std::shared_ptr<GlobalParametersCore> &world_core, const std::shared_ptr<RobotStateCore> &robot_state_core, const Eigen::MatrixXd& covariance_robot_matrix)
{
    /// tf pose robot wrt world
    this->publishTfPoseRobotWrtWorld(time_stamp, world_core, robot_state_core);


    /// Other publishers

    // Getters

    std::shared_ptr<ImuDrivenRobotStateCore> TheRobotStateCore=std::static_pointer_cast<ImuDrivenRobotStateCore>(robot_state_core);

    Eigen::Vector3d robotPosition=TheRobotStateCore->getPositionRobotWrtWorld();
    Eigen::Vector4d robotAttitude=TheRobotStateCore->getAttitudeRobotWrtWorld();
    Eigen::Vector3d robotLinearSpeed=TheRobotStateCore->getLinearSpeedRobotWrtWorld();
    Eigen::Vector3d robotLinearAcceleration=TheRobotStateCore->getLinearAccelerationRobotWrtWorld();
    Eigen::Vector3d robotAngularVelocity=TheRobotStateCore->getAngularVelocityRobotWrtWorld();



    // Fill msg


    // Header

    // ROBOT POSE
    // Stamp
    robotPoseWithCovarianceStampedMsg.header.stamp=ros::Time(time_stamp.sec, time_stamp.nsec);
    // Frame id
    robotPoseWithCovarianceStampedMsg.header.frame_id=world_core->getWorldName();

    //
    robotPoseStampedMsg.header.stamp=ros::Time(time_stamp.sec, time_stamp.nsec);
    // Frame id
    robotPoseStampedMsg.header.frame_id=world_core->getWorldName();

    //
    robotLinearSpeedStampedMsg.header.stamp=ros::Time(time_stamp.sec, time_stamp.nsec);
    // Frame id
    robotLinearSpeedStampedMsg.header.frame_id=world_core->getWorldName();

    //
    robotLinearAccelerationStampedMsg.header.stamp=ros::Time(time_stamp.sec, time_stamp.nsec);
    // Frame id
    robotLinearAccelerationStampedMsg.header.frame_id=world_core->getWorldName();

    //
    robotAngularVelocityStampedMsg.header.stamp=ros::Time(time_stamp.sec, time_stamp.nsec);
    // Frame id
    robotAngularVelocityStampedMsg.header.frame_id=world_core->getWorldName();


    // Pose
    geometry_msgs::Pose RobotPose;

    // Position
    RobotPose.position.x=robotPosition[0];
    RobotPose.position.y=robotPosition[1];
    RobotPose.position.z=robotPosition[2];

    // Attitude
    RobotPose.orientation.w=robotAttitude[0];
    RobotPose.orientation.x=robotAttitude[1];
    RobotPose.orientation.y=robotAttitude[2];
    RobotPose.orientation.z=robotAttitude[3];


    // Fill Message

    //
    robotPoseWithCovarianceStampedMsg.pose.pose=RobotPose;


    // Covariance
    // TODO fix! Covariance of the attitude is not ok!
    Eigen::Matrix<double, 6, 6> robotPoseCovariance;//(6,6);
    robotPoseCovariance.setZero();
    robotPoseCovariance.block<3,3>(0,0)=covariance_robot_matrix.block<3,3>(0,0);
    robotPoseCovariance.block<3,3>(3,3)=covariance_robot_matrix.block<3,3>(6,6);
    double robotPoseCovarianceArray[36];
    Eigen::Map< Eigen::Matrix<double, 6, 6> >(robotPoseCovarianceArray, 6, 6) = robotPoseCovariance;
    for(unsigned int i=0; i<36; i++)
    {
        robotPoseWithCovarianceStampedMsg.pose.covariance[i]=robotPoseCovarianceArray[i];
    }


    //
   robotPoseStampedMsg.pose=RobotPose;

   //
   robotLinearSpeedStampedMsg.vector.x=robotLinearSpeed[0];
   robotLinearSpeedStampedMsg.vector.y=robotLinearSpeed[1];
   robotLinearSpeedStampedMsg.vector.z=robotLinearSpeed[2];

   //
   robotLinearAccelerationStampedMsg.vector.x=robotLinearAcceleration[0];
   robotLinearAccelerationStampedMsg.vector.y=robotLinearAcceleration[1];
   robotLinearAccelerationStampedMsg.vector.z=robotLinearAcceleration[2];

   //
   robotAngularVelocityStampedMsg.vector.x=robotAngularVelocity[0];
   robotAngularVelocityStampedMsg.vector.y=robotAngularVelocity[1];
   robotAngularVelocityStampedMsg.vector.z=robotAngularVelocity[2];




   // Publish Robot Pose
   if(robotPoseWithCovarianceStampedPub.getNumSubscribers()>0)
       robotPoseWithCovarianceStampedPub.publish(robotPoseWithCovarianceStampedMsg);

   if(robotPoseStampedPub.getNumSubscribers()>0)
       robotPoseStampedPub.publish(robotPoseStampedMsg);

   if(robotLinearSpeedStampedPub.getNumSubscribers()>0)
       robotLinearSpeedStampedPub.publish(robotLinearSpeedStampedMsg);

   if(robotLinearAccelerationStampedPub.getNumSubscribers()>0)
       robotLinearAccelerationStampedPub.publish(robotLinearAccelerationStampedMsg);

   if(robotAngularVelocityStampedPub.getNumSubscribers()>0)
       robotAngularVelocityStampedPub.publish(robotAngularVelocityStampedMsg);


    // end
    return 0;
}

int RosImuDrivenRobotInterface::publishTfPoseRobotWrtWorld(const TimeStamp& time_stamp, const std::shared_ptr<GlobalParametersCore> &world_core, const std::shared_ptr<RobotStateCore> &robot_state_core)
{
    std::shared_ptr<ImuDrivenRobotStateCore> TheRobotStateCore=std::dynamic_pointer_cast<ImuDrivenRobotStateCore>(robot_state_core);

    Eigen::Vector3d robotPosition=TheRobotStateCore->getPositionRobotWrtWorld();
    Eigen::Vector4d robotAttitude=TheRobotStateCore->getAttitudeRobotWrtWorld();

    tf::Quaternion tf_rot(robotAttitude[1], robotAttitude[2], robotAttitude[3], robotAttitude[0]);
    tf::Vector3 tf_tran(robotPosition[0], robotPosition[1], robotPosition[2]);

    tf::Transform transform(tf_rot, tf_tran);

    tf_transform_broadcaster_->sendTransform(tf::StampedTransform(transform, ros::Time(time_stamp.sec, time_stamp.nsec),
                                          world_core->getWorldName(), this->getRobotName()));


    return 0;
}

int RosImuDrivenRobotInterface::readConfig(const pugi::xml_node &robot, std::shared_ptr<ImuDrivenRobotStateCore>& robot_state_core)
{
    /// Imu Sensor Configs
    int errorReadConfig=this->ImuDrivenRobotCore::readConfig(robot, robot_state_core);

    if(errorReadConfig)
        return errorReadConfig;


    /// Ros Configs

    // Name
    std::string robot_name=robot.child_value("name");
    this->setRobotName(robot_name);


    /// Finish

    // Open
    this->open();

    // End
    return 0;
}
