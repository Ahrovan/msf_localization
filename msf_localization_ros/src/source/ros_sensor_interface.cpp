
#include "msf_localization_ros/ros_sensor_interface.h"


RosSensorInterface::RosSensorInterface(ros::NodeHandle* nh, tf::TransformBroadcaster *tf_transform_broadcaster) :
    RosInterface()
{
    // Node Handle
    this->nh=nh;

    // Tf broadcaster
    this->tf_transform_broadcaster_=tf_transform_broadcaster;

    return;
}

RosSensorInterface::~RosSensorInterface()
{
    return;
}

int RosSensorInterface::publishTfPoseSensorWrtRobot(TimeStamp time_stamp, std::shared_ptr<RosRobotInterface> robot_core, std::shared_ptr<SensorStateCore> sensor_state_core)
{
    Eigen::Vector3d sensorPosition=sensor_state_core->getPositionSensorWrtRobot();
    Eigen::Vector4d sensorAttitude=sensor_state_core->getAttitudeSensorWrtRobot();

    tf::Quaternion tf_rot(sensorAttitude[1], sensorAttitude[2], sensorAttitude[3], sensorAttitude[0]);
    tf::Vector3 tf_tran(sensorPosition[0], sensorPosition[1], sensorPosition[2]);

    tf::Transform transform(tf_rot, tf_tran);


    tf_transform_broadcaster_->sendTransform(tf::StampedTransform(transform, ros::Time(time_stamp.sec, time_stamp.nsec),
                                          robot_core->getRobotName(), std::dynamic_pointer_cast<SensorCore>(sensor_state_core->getMsfElementCoreSharedPtr())->getSensorName()));

    return 0;
}
