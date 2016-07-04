#include "msf_localization_ros/ros_aruco_eye_interface.h"


RosArucoEyeInterface::RosArucoEyeInterface(ros::NodeHandle* nh, tf::TransformBroadcaster *tf_transform_broadcaster, MsfLocalizationCore* msf_localization_core_ptr) :
    RosSensorInterface(nh, tf_transform_broadcaster),
    CodedVisualMarkerEyeCore(msf_localization_core_ptr)
{

    // Frequencies
    frequency_desired_=-2.0; // Hz
    previous_time_stamp_=ros::Time(0,0);


    return;
}

int RosArucoEyeInterface::setMarkerListTopicName(const std::string marker_list_topic_name)
{
    this->marker_list_topic_name_=marker_list_topic_name;
    return 0;
}


int RosArucoEyeInterface::setMeasurementRos(const aruco_eye_msgs::MarkerListPtr& msg)
{
    if(!isSensorEnabled())
        return 0;


    // Frequency
    if(frequency_desired_>0)
    {
        // Set time stamps
        ros::Time current_time_stamp=msg->header.stamp;

        ros::Duration diference_time_stamps=current_time_stamp-previous_time_stamp_;


        if(diference_time_stamps.toSec() < 1/frequency_desired_)
            return 0;

        // Update for the next iteration
        previous_time_stamp_=current_time_stamp;
    }



    // Publish ack
    // TODO


    // Time Stamp
    TimeStamp the_time_stamp(msg->header.stamp.sec, msg->header.stamp.nsec);


    // Iteration over all the measurements
    std::list< std::shared_ptr<SensorMeasurementCore> > the_visual_marker_measurement_core_list;
    for(auto it_visual_markers=msg->markers.begin();
        it_visual_markers!=msg->markers.end();
        ++it_visual_markers)
    {

        // Check if the visual marker is 3d reconstructed
        if(!(it_visual_markers)->is3dReconstructed)
            continue;

        // Value with sensor core
        std::shared_ptr<CodedVisualMarkerEyeCore> the_coded_visual_marker_eye_core=std::dynamic_pointer_cast<CodedVisualMarkerEyeCore>(this->getMsfElementCoreSharedPtr());
        std::shared_ptr<CodedVisualMarkerMeasurementCore> the_visual_marker_measurement_core=std::make_shared<CodedVisualMarkerMeasurementCore>(the_coded_visual_marker_eye_core);

        // Measured Id
        if(the_visual_marker_measurement_core->setVisualMarkerId((it_visual_markers)->id))
            std::cout<<"Error setting id"<<std::endl;


        // Measurement Position
        if(this->isMeasurementPositionEnabled())
        {
            Eigen::Vector3d position((it_visual_markers)->pose.pose.position.x, (it_visual_markers)->pose.pose.position.y, (it_visual_markers)->pose.pose.position.z);

            if(the_visual_marker_measurement_core->setVisualMarkerPosition(position))
                std::cout<<"Error setting position"<<std::endl;

        }

        // Measurement Attitude
        if(this->isMeasurementAttitudeEnabled())
        {
            Eigen::Vector4d orientation;
            Eigen::Vector4d orientation_aux;

            orientation_aux<<(it_visual_markers)->pose.pose.orientation.w,
                        (it_visual_markers)->pose.pose.orientation.x,
                        (it_visual_markers)->pose.pose.orientation.y,
                        (it_visual_markers)->pose.pose.orientation.z;

            if(orientation_aux[0]<0)
            {
                orientation=-orientation_aux;
            }
            else
            {
                orientation=orientation_aux;
            }

            if(the_visual_marker_measurement_core->setVisualMarkerAttitude(orientation))
                std::cout<<"Error setting orientation"<<std::endl;
        }



        // Push back
        the_visual_marker_measurement_core_list.push_back(the_visual_marker_measurement_core);

    }


    // Set
    this->setMeasurementList(the_time_stamp, the_visual_marker_measurement_core_list);



    return 0;
}


void RosArucoEyeInterface::markerListCallback(const aruco_eye_msgs::MarkerListPtr& msg)
{

    this->setMeasurementRos(msg);


    return;
}


int RosArucoEyeInterface::open()
{

    // Subscriber
    marker_list_sub_=nh->subscribe(marker_list_topic_name_, 10, &RosArucoEyeInterface::markerListCallback, this);


    return 0;
}

int RosArucoEyeInterface::publish(const TimeStamp& time_stamp, const std::shared_ptr<RosRobotInterface>& robot_core, const std::shared_ptr<SensorStateCore>& sensor_state_core)
{
    // tf pose sensor wrt robot
    this->publishTfPoseSensorWrtRobot(time_stamp, robot_core, sensor_state_core);

    // end
    return 0;
}

int RosArucoEyeInterface::readConfig(const pugi::xml_node& sensor, unsigned int sensorId, std::shared_ptr<CodedVisualMarkerEyeStateCore>& SensorInitStateCore)
{
    /// Sensor General Configs
    int errorReadConfig=this->CodedVisualMarkerEyeCore::readConfig(sensor, sensorId, SensorInitStateCore);

    if(errorReadConfig)
        return errorReadConfig;



    /// ROS Configs

    // Sensor Topic
    std::string sensor_topic=sensor.child_value("ros_topic");
    this->setMarkerListTopicName(sensor_topic);


    /// Finish

    // Open
    this->open();

    // End
    return 0;
}
