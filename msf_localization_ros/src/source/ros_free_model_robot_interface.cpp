
#include "msf_localization_ros/ros_free_model_robot_interface.h"


RosFreeModelRobotInterface::RosFreeModelRobotInterface(ros::NodeHandle* nh, tf::TransformBroadcaster *tf_transform_broadcaster, const std::weak_ptr<MsfStorageCore> the_msf_storage_core) :
    RosRobotInterface(nh, tf_transform_broadcaster),
    FreeModelRobotCore(the_msf_storage_core)
{

    return;
}

RosFreeModelRobotInterface::~RosFreeModelRobotInterface()
{
    return;
}

int RosFreeModelRobotInterface::readParameters()
{
    // Topic names

    // Pose
    //
    ros::param::param<std::string>("~robot_pose_with_cov_topic_name", robotPoseWithCovarianceStampedTopicName, "msf_localization/robot_pose_cov");
    std::cout<<"\t robot_pose_with_cov_topic_name="<<robotPoseWithCovarianceStampedTopicName<<std::endl;
    //
    ros::param::param<std::string>("~robot_pose_topic_name", robotPoseStampedTopicName, "msf_localization/robot_pose");
    std::cout<<"\t robot_pose_topic_name="<<robotPoseStampedTopicName<<std::endl;

    // Velocities
    //
    ros::param::param<std::string>("~robot_velocities_stamped_topic_name_", robot_velocities_stamped_topic_name_, "msf_localization/robot_velocity");
    std::cout<<"\t robot_velocities_stamped_topic_name_="<<robot_velocities_stamped_topic_name_<<std::endl;
    //
    ros::param::param<std::string>("~robot_velocities_with_covariance_stamped_topic_name_", robot_velocities_with_covariance_stamped_topic_name_, "msf_localization/robot_velocity_cov");
    std::cout<<"\t robot_velocities_with_covariance_stamped_topic_name_="<<robot_velocities_with_covariance_stamped_topic_name_<<std::endl;
    //
    ros::param::param<std::string>("~robot_linear_speed_topic_name", robotLinearSpeedStampedTopicName, "msf_localization/robot_linear_speed");
    std::cout<<"\t robot_linear_speed_topic_name="<<robotLinearSpeedStampedTopicName<<std::endl;
    //
    ros::param::param<std::string>("~robot_angular_velocity_topic_name", robotAngularVelocityStampedTopicName, "msf_localization/robot_angular_velocity");
    std::cout<<"\t robot_angular_velocity_topic_name="<<robotAngularVelocityStampedTopicName<<std::endl;

    // Accelerations
    //
    ros::param::param<std::string>("~robot_accelerations_stamped_topic_name_", robot_accelerations_stamped_topic_name_, "msf_localization/robot_acceleration");
    std::cout<<"\t robot_accelerations_stamped_topic_name_="<<robot_accelerations_stamped_topic_name_<<std::endl;
    //
    ros::param::param<std::string>("~robot_accelerations_with_covariance_stamped_topic_name_", robot_accelerations_with_covariance_stamped_topic_name_, "msf_localization/robot_acceleration_cov");
    std::cout<<"\t robot_accelerations_with_covariance_stamped_topic_name_="<<robot_accelerations_with_covariance_stamped_topic_name_<<std::endl;
    //
    ros::param::param<std::string>("~robot_linear_acceleration_topic_name", robotLinearAccelerationStampedTopicName, "msf_localization/robot_linear_acceleration");
    std::cout<<"\t robot_linear_acceleration_topic_name="<<robotLinearAccelerationStampedTopicName<<std::endl;
    //
    ros::param::param<std::string>("~robot_angular_acceleration_topic_name", robotAngularAccelerationStampedTopicName, "msf_localization/robot_angular_acceleration");
    std::cout<<"\t robot_angular_acceleration_topic_name="<<robotAngularAccelerationStampedTopicName<<std::endl;


    return 0;
}

int RosFreeModelRobotInterface::open()
{
    // Read ROS Parameters
    this->readParameters();


    // Publishers


    // Pose
    //
    robotPoseStampedPub = nh->advertise<geometry_msgs::PoseStamped>(robotPoseStampedTopicName, 1, true);
    //
    robotPoseWithCovarianceStampedPub = nh->advertise<geometry_msgs::PoseWithCovarianceStamped>(robotPoseWithCovarianceStampedTopicName, 1, true);


    // Velocities

    //
    robot_velocities_stamped_pub_=nh->advertise<geometry_msgs::TwistStamped>(robot_velocities_stamped_topic_name_, 1, true);
    //
    robot_velocities_with_covariance_stamped_pub_=nh->advertise<geometry_msgs::TwistWithCovarianceStamped>(robot_velocities_with_covariance_stamped_topic_name_, 1, true);
    //
    robotLinearSpeedStampedPub = nh->advertise<geometry_msgs::Vector3Stamped>(robotLinearSpeedStampedTopicName, 1, true);
    //
    robotAngularVelocityStampedPub = nh->advertise<geometry_msgs::Vector3Stamped>(robotAngularVelocityStampedTopicName, 1, true);


    // Accelerations
    //
    robot_accelerations_stamped_pub_=nh->advertise<geometry_msgs::AccelStamped>(robot_accelerations_stamped_topic_name_, 1, true);
    //
    robot_accelerations_with_covariance_stamped_pub_=nh->advertise<geometry_msgs::AccelWithCovarianceStamped>(robot_accelerations_with_covariance_stamped_topic_name_, 1, true);
    //
    robotLinearAccelerationStampedPub = nh->advertise<geometry_msgs::Vector3Stamped>(robotLinearAccelerationStampedTopicName, 1, true);
    //
    robotAngularAccelerationStampedPub = nh->advertise<geometry_msgs::Vector3Stamped>(robotAngularAccelerationStampedTopicName, 1, true);



    return 0;
}

int RosFreeModelRobotInterface::publish(const TimeStamp& time_stamp, const std::shared_ptr<GlobalParametersCore> &world_core, const std::shared_ptr<RobotStateCore> &robot_state_core, const Eigen::MatrixXd& covariance_robot_matrix)
{
    /// tf pose robot wrt world
    this->publishTfPoseRobotWrtWorld(time_stamp, world_core, robot_state_core);


    /// Other publishers

    // Getters

    std::shared_ptr<FreeModelRobotStateCore> TheRobotStateCore=std::static_pointer_cast<FreeModelRobotStateCore>(robot_state_core);

    Eigen::Vector3d robotPosition=TheRobotStateCore->getPositionRobotWrtWorld();
    Eigen::Vector4d robotAttitude=TheRobotStateCore->getAttitudeRobotWrtWorld();
    Eigen::Vector3d robotLinearSpeed=TheRobotStateCore->getLinearSpeedRobotWrtWorld();
    Eigen::Vector3d robotLinearAcceleration=TheRobotStateCore->getLinearAccelerationRobotWrtWorld();
    Eigen::Vector3d robotAngularVelocity=TheRobotStateCore->getAngularVelocityRobotWrtWorld();
    Eigen::Vector3d robotAngularAcceleration=TheRobotStateCore->getAngularAccelerationRobotWrtWorld();



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


    // Velocities

    //
    robot_velocities_stamped_msg_.header.stamp=ros::Time(time_stamp.sec, time_stamp.nsec);
    //
    robot_velocities_stamped_msg_.header.frame_id=world_core->getWorldName();

    //
    robot_velocities_with_covariance_stamped_msg_.header.stamp=ros::Time(time_stamp.sec, time_stamp.nsec);
    //
    robot_velocities_with_covariance_stamped_msg_.header.frame_id=world_core->getWorldName();

    //
    robotLinearSpeedStampedMsg.header.stamp=ros::Time(time_stamp.sec, time_stamp.nsec);
    // Frame id
    robotLinearSpeedStampedMsg.header.frame_id=world_core->getWorldName();

    //
    robotAngularVelocityStampedMsg.header.stamp=ros::Time(time_stamp.sec, time_stamp.nsec);
    // Frame id
    robotAngularVelocityStampedMsg.header.frame_id=world_core->getWorldName();


    // Accelerations

    //
    robot_accelerations_stamped_msg_.header.stamp=ros::Time(time_stamp.sec, time_stamp.nsec);
    //
    robot_accelerations_stamped_msg_.header.frame_id=world_core->getWorldName();

    //
    robot_accelerations_with_covariance_stamped_msg_.header.stamp=ros::Time(time_stamp.sec, time_stamp.nsec);
    //
    robot_accelerations_with_covariance_stamped_msg_.header.frame_id=world_core->getWorldName();

    //
    robotLinearAccelerationStampedMsg.header.stamp=ros::Time(time_stamp.sec, time_stamp.nsec);
    // Frame id
    robotLinearAccelerationStampedMsg.header.frame_id=world_core->getWorldName();

    //
    robotAngularAccelerationStampedMsg.header.stamp=ros::Time(time_stamp.sec, time_stamp.nsec);
    // Frame id
    robotAngularAccelerationStampedMsg.header.frame_id=world_core->getWorldName();





    /// Pose
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
    Eigen::MatrixXd robotPoseCovariance(6,6);
    robotPoseCovariance.setZero();
    robotPoseCovariance.block<3,3>(0,0)=covariance_robot_matrix.block<3,3>(0,0);
    robotPoseCovariance.block<3,3>(3,3)=covariance_robot_matrix.block<3,3>(9,9);
    double robotPoseCovarianceArray[36];
    Eigen::Map<Eigen::MatrixXd>(robotPoseCovarianceArray, 6, 6) = robotPoseCovariance;
    for(unsigned int i=0; i<36; i++)
    {
        robotPoseWithCovarianceStampedMsg.pose.covariance[i]=robotPoseCovarianceArray[i];
    }


    //
   robotPoseStampedMsg.pose=RobotPose;



   /// Velocity

   geometry_msgs::Vector3 lin_vel;
   lin_vel.x=robotLinearSpeed[0];
   lin_vel.y=robotLinearSpeed[1];
   lin_vel.z=robotLinearSpeed[2];

   geometry_msgs::Vector3 ang_vel;
   ang_vel.x=robotAngularVelocity[0];
   ang_vel.y=robotAngularVelocity[1];
   ang_vel.z=robotAngularVelocity[2];


   geometry_msgs::Twist velocity;
   velocity.linear=lin_vel;
    velocity.angular=ang_vel;


    //
    robot_velocities_stamped_msg_.twist=velocity;

    //
    robot_velocities_with_covariance_stamped_msg_.twist.twist=velocity;

    {
        Eigen::MatrixXd covariance(6,6);
        covariance.setZero();
        covariance.block<3,3>(0,0)=covariance_robot_matrix.block<3,3>(3,3);
        covariance.block<3,3>(3,3)=covariance_robot_matrix.block<3,3>(12,12);
        double covarianceArray[36];
        Eigen::Map<Eigen::MatrixXd>(covarianceArray, 6, 6) = covariance;
        for(unsigned int i=0; i<36; i++)
        {
            robot_velocities_with_covariance_stamped_msg_.twist.covariance[i]=covarianceArray[i];
        }
    }


   //
   robotLinearSpeedStampedMsg.vector=lin_vel;


   //
   robotAngularVelocityStampedMsg.vector=ang_vel;




   /// Acceleration


   geometry_msgs::Vector3 lin_acc;
   lin_acc.x=robotLinearAcceleration[0];
   lin_acc.y=robotLinearAcceleration[1];
   lin_acc.z=robotLinearAcceleration[2];

   geometry_msgs::Vector3 ang_acc;
   ang_acc.x=robotAngularAcceleration[0];
   ang_acc.y=robotAngularAcceleration[1];
   ang_acc.z=robotAngularAcceleration[2];


   geometry_msgs::Accel acceleration;
   acceleration.linear=lin_acc;
    acceleration.angular=ang_acc;


    //
    robot_accelerations_stamped_msg_.accel=acceleration;

    //
    robot_accelerations_with_covariance_stamped_msg_.accel.accel=acceleration;

    {
        Eigen::MatrixXd covariance(6,6);
        covariance.setZero();
        covariance.block<3,3>(0,0)=covariance_robot_matrix.block<3,3>(6,6);
        covariance.block<3,3>(3,3)=covariance_robot_matrix.block<3,3>(15,15);
        double covarianceArray[36];
        Eigen::Map<Eigen::MatrixXd>(covarianceArray, 6, 6) = covariance;
        for(unsigned int i=0; i<36; i++)
        {
            robot_accelerations_with_covariance_stamped_msg_.accel.covariance[i]=covarianceArray[i];
        }
    }



   //
   robotLinearAccelerationStampedMsg.vector=lin_acc;

   //
   robotAngularAccelerationStampedMsg.vector=ang_acc;




   /// Publish Robot State

   // Pose
   if(robotPoseStampedPub.getNumSubscribers()>0)
       robotPoseStampedPub.publish(robotPoseStampedMsg);

   if(robotPoseWithCovarianceStampedPub.getNumSubscribers()>0)
       robotPoseWithCovarianceStampedPub.publish(robotPoseWithCovarianceStampedMsg);


    // Velocities

   if(robot_velocities_stamped_pub_.getNumSubscribers()>0)
       robot_velocities_stamped_pub_.publish(robot_velocities_stamped_msg_);

   if(robot_velocities_with_covariance_stamped_pub_.getNumSubscribers()>0)
       robot_velocities_with_covariance_stamped_pub_.publish(robot_velocities_with_covariance_stamped_msg_);

   if(robotLinearSpeedStampedPub.getNumSubscribers()>0)
        robotLinearSpeedStampedPub.publish(robotLinearSpeedStampedMsg);

   if(robotAngularVelocityStampedPub.getNumSubscribers()>0)
       robotAngularVelocityStampedPub.publish(robotAngularVelocityStampedMsg);


    // Accelerations

   if(robot_accelerations_stamped_pub_.getNumSubscribers()>0)
       robot_accelerations_stamped_pub_.publish(robot_accelerations_stamped_msg_);

   if(robot_accelerations_with_covariance_stamped_pub_.getNumSubscribers()>0)
       robot_accelerations_with_covariance_stamped_pub_.publish(robot_accelerations_with_covariance_stamped_msg_);

   if(robotLinearAccelerationStampedPub.getNumSubscribers()>0)
       robotLinearAccelerationStampedPub.publish(robotLinearAccelerationStampedMsg);

   if(robotAngularAccelerationStampedPub.getNumSubscribers()>0)
       robotAngularAccelerationStampedPub.publish(robotAngularAccelerationStampedMsg);


    // end
    return 0;
}

int RosFreeModelRobotInterface::publishTfPoseRobotWrtWorld(const TimeStamp& time_stamp, const std::shared_ptr<GlobalParametersCore>& world_core, const std::shared_ptr<RobotStateCore>& robot_state_core)
{
    std::shared_ptr<FreeModelRobotStateCore> TheRobotStateCore=std::dynamic_pointer_cast<FreeModelRobotStateCore>(robot_state_core);

    Eigen::Vector3d robotPosition=TheRobotStateCore->getPositionRobotWrtWorld();
    Eigen::Vector4d robotAttitude=TheRobotStateCore->getAttitudeRobotWrtWorld();

    tf::Quaternion tf_rot(robotAttitude[1], robotAttitude[2], robotAttitude[3], robotAttitude[0]);
    tf::Vector3 tf_tran(robotPosition[0], robotPosition[1], robotPosition[2]);

    tf::Transform transform(tf_rot, tf_tran);

    tf_transform_broadcaster_->sendTransform(tf::StampedTransform(transform, ros::Time(time_stamp.sec, time_stamp.nsec),
                                          world_core->getWorldName(), this->getRobotName()));


    return 0;
}

int RosFreeModelRobotInterface::readConfig(const pugi::xml_node &robot, std::shared_ptr<FreeModelRobotStateCore>& robot_state_core)
{
    /// Imu Sensor Configs
    int errorReadConfig=this->FreeModelRobotCore::readConfig(robot, robot_state_core);

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



