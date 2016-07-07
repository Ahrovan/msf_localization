
#include "msf_localization_ros/ros_absolute_pose_driven_robot_interface.h"


RosAbsolutePoseDrivenRobotInterface::RosAbsolutePoseDrivenRobotInterface(ros::NodeHandle* nh, tf::TransformBroadcaster *tf_transform_broadcaster, MsfLocalizationCore* msf_localization_core_ptr) :
    RosRobotInterface(nh, tf_transform_broadcaster),
    AbsolutePoseDrivenRobotCore(msf_localization_core_ptr)
{

    return;
}

RosAbsolutePoseDrivenRobotInterface::~RosAbsolutePoseDrivenRobotInterface()
{
    return;
}

int RosAbsolutePoseDrivenRobotInterface::readParameters()
{
    // Topic names

    // Pose
    //
    ros::param::param<std::string>("~robot_pose_with_cov_topic_name", robotPoseWithCovarianceStampedTopicName, "msf_localization/robot_pose_cov");
    std::cout<<"\t robot_pose_with_cov_topic_name="<<robotPoseWithCovarianceStampedTopicName<<std::endl;
    //
    ros::param::param<std::string>("~robot_pose_topic_name", robotPoseStampedTopicName, "msf_localization/robot_pose");
    std::cout<<"\t robot_pose_topic_name="<<robotPoseStampedTopicName<<std::endl;


    return 0;
}

int RosAbsolutePoseDrivenRobotInterface::open()
{
    // Read ROS Parameters
    this->readParameters();


    // Publishers


    // Pose
    //
    robotPoseStampedPub = nh->advertise<geometry_msgs::PoseStamped>(robotPoseStampedTopicName, 1, true);
    //
    robotPoseWithCovarianceStampedPub = nh->advertise<geometry_msgs::PoseWithCovarianceStamped>(robotPoseWithCovarianceStampedTopicName, 1, true);



    return 0;
}

int RosAbsolutePoseDrivenRobotInterface::publish(const TimeStamp& time_stamp, const std::shared_ptr<GlobalParametersCore> &world_core, const std::shared_ptr<RobotStateCore> &robot_state_core, const Eigen::MatrixXd& covariance_robot_matrix)
{
    /// tf pose robot wrt world
    this->publishTfPoseRobotWrtWorld(time_stamp, world_core, robot_state_core);


    /// Other publishers

    // Getters

    std::shared_ptr<AbsolutePoseDrivenRobotStateCore> TheRobotStateCore=std::dynamic_pointer_cast<AbsolutePoseDrivenRobotStateCore>(robot_state_core);

    Eigen::Vector3d robotPosition=TheRobotStateCore->getPositionRobotWrtWorld();
    Eigen::Vector4d robotAttitude=TheRobotStateCore->getAttitudeRobotWrtWorld();



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
    Eigen::Matrix<double, 6, 6> robotPoseCovariance;//(6,6);
    robotPoseCovariance.setZero();
    robotPoseCovariance.block<3,3>(0,0)=covariance_robot_matrix.block<3,3>(0,0);
    robotPoseCovariance.block<3,3>(3,3)=covariance_robot_matrix.block<3,3>(3,3);
    robotPoseCovariance.block<3,3>(3,0)=covariance_robot_matrix.block<3,3>(3,0);
    robotPoseCovariance.block<3,3>(0,3)=covariance_robot_matrix.block<3,3>(0,3);
    double robotPoseCovarianceArray[36];
    Eigen::Map< Eigen::Matrix<double, 6, 6> >(robotPoseCovarianceArray, 6, 6) = robotPoseCovariance;
    for(unsigned int i=0; i<36; i++)
    {
        robotPoseWithCovarianceStampedMsg.pose.covariance[i]=robotPoseCovarianceArray[i];
    }


    //
   robotPoseStampedMsg.pose=RobotPose;





   /// Publish Robot State

   // Pose
   if(robotPoseStampedPub.getNumSubscribers()>0)
       robotPoseStampedPub.publish(robotPoseStampedMsg);

   if(robotPoseWithCovarianceStampedPub.getNumSubscribers()>0)
       robotPoseWithCovarianceStampedPub.publish(robotPoseWithCovarianceStampedMsg);




    // end
    return 0;
}

int RosAbsolutePoseDrivenRobotInterface::publishTfPoseRobotWrtWorld(const TimeStamp& time_stamp, const std::shared_ptr<GlobalParametersCore>& world_core, const std::shared_ptr<RobotStateCore>& robot_state_core)
{
    std::shared_ptr<AbsolutePoseDrivenRobotStateCore> TheRobotStateCore=std::dynamic_pointer_cast<AbsolutePoseDrivenRobotStateCore>(robot_state_core);

    Eigen::Vector3d robotPosition=TheRobotStateCore->getPositionRobotWrtWorld();
    Eigen::Vector4d robotAttitude=TheRobotStateCore->getAttitudeRobotWrtWorld();

    tf::Quaternion tf_rot(robotAttitude[1], robotAttitude[2], robotAttitude[3], robotAttitude[0]);
    tf::Vector3 tf_tran(robotPosition[0], robotPosition[1], robotPosition[2]);

    tf::Transform transform(tf_rot, tf_tran);

    tf_transform_broadcaster_->sendTransform(tf::StampedTransform(transform, ros::Time(time_stamp.sec, time_stamp.nsec),
                                          world_core->getWorldName(), this->getRobotName()));


    return 0;
}

int RosAbsolutePoseDrivenRobotInterface::readConfig(const pugi::xml_node &robot, std::shared_ptr<AbsolutePoseDrivenRobotStateCore>& robot_state_core)
{
    /// Imu Sensor Configs
    int errorReadConfig=this->AbsolutePoseDrivenRobotCore::readConfig(robot, robot_state_core);

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



