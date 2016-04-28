#include "msf_localization_ros/ros_aruco_eye_interface.h"


RosArucoEyeInterface::RosArucoEyeInterface(ros::NodeHandle* nh, std::weak_ptr<MsfStorageCore> the_msf_storage_core) :
    RosSensorInterface(nh),
    CodedVisualMarkerEyeCore(the_msf_storage_core)
{


    return;
}

int RosArucoEyeInterface::setMarkerListTopicName(std::string marker_list_topic_name)
{
    this->marker_list_topic_name_=marker_list_topic_name;
    return 0;
}


int RosArucoEyeInterface::setMeasurementRos(const aruco_eye_msgs::MarkerListPtr& msg)
{
    if(!isSensorEnabled())
        return 0;


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
    //logFile<<"RosSensorImuInterface::imuTopicCallback()"<<std::endl;

    this->setMeasurementRos(msg);


    //logFile<<"RosSensorImuInterface::imuTopicCallback() ended"<<std::endl;
    return;
}


int RosArucoEyeInterface::open()
{

    //std::cout<<"RosSensorImuInterface::open()"<<std::endl;

    // Subscriber
    marker_list_sub_=nh->subscribe(marker_list_topic_name_, 10, &RosArucoEyeInterface::markerListCallback, this);


    return 0;
}

int RosArucoEyeInterface::publish()
{
    // TODO

    return 0;
}

int RosArucoEyeInterface::readConfig(pugi::xml_node sensor, unsigned int sensorId, std::shared_ptr<CodedVisualMarkerEyeStateCore>& SensorInitStateCore)
{
    /// Sensor General Configs
    int errorReadConfig=this->CodedVisualMarkerEyeCore::readConfig(sensor, sensorId, SensorInitStateCore);

    if(errorReadConfig)
        return errorReadConfig;



    /// ROS Configs

    // Sensor Topic
    std::string sensorTopic=sensor.child_value("ros_topic");
    this->setMarkerListTopicName(sensorTopic);



    /// Finish

    // Open
    this->open();

    // End
    return 0;
}
