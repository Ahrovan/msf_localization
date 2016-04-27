#include "msf_localization_ros/ros_aruco_eye_interface.h"


RosArucoEyeInterface::RosArucoEyeInterface(ros::NodeHandle* nh, std::weak_ptr<MsfStorageCore> the_msf_storage_core) :
    RosSensorInterface(),
    CodedVisualMarkerEyeCore(the_msf_storage_core)
{
    //std::cout<<"RosArucoEyeInterface::RosArucoEyeInterface(ros::NodeHandle* nh, std::weak_ptr<MsfStorageCore> the_msf_storage_core)"<<std::endl;

    // Node Handle
    this->nh=nh;


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

int RosArucoEyeInterface::readConfig(pugi::xml_node sensor, unsigned int sensorId, std::shared_ptr<CodedVisualMarkerEyeStateCore>& SensorInitStateCore)
{
    // Sensor Core Pointer
    //std::shared_ptr<SensorCore> TheSensorCore(this);
    std::shared_ptr<CodedVisualMarkerEyeCore> TheSensorCore=std::dynamic_pointer_cast<CodedVisualMarkerEyeCore>(this->getMsfElementCoreSharedPtr());



    // Set pointer to the SensorCore
    //this->setMsfElementCorePtr(TheSensorCore);

    // Create a class for the SensorStateCore
    if(!SensorInitStateCore)
        SensorInitStateCore=std::make_shared<CodedVisualMarkerEyeStateCore>();

    // Set pointer to the SensorCore
    SensorInitStateCore->setTheSensorCore(TheSensorCore);


    // Set sensor type
    //this->setSensorType(SensorTypes::coded_visual_marker_eye);

    // Set Id
    this->setSensorId(sensorId);

    // Set the access to the Storage core
    //TheRosSensorImuInterface->setTheMsfStorageCore(std::make_shared<MsfStorageCore>(this->TheStateEstimationCore));
    //this->setMsfStorageCorePtr(TheMsfStorageCore);


    // Sensor Topic
    std::string sensorTopic=sensor.child_value("ros_topic");
    this->setMarkerListTopicName(sensorTopic);


    // Auxiliar reading value
    std::string readingValue;


    // Name
    readingValue=sensor.child_value("name");
    this->setSensorName(readingValue);


    //// Sensor configurations


    /// Pose of the sensor wrt robot
    pugi::xml_node pose_in_robot=sensor.child("pose_in_robot");

    // Position of the sensor wrt robot
    readingValue=pose_in_robot.child("position").child_value("enabled");
    if(std::stoi(readingValue))
        this->enableEstimationPositionSensorWrtRobot();

    // Attitude of the sensor wrt robot
    readingValue=pose_in_robot.child("attitude").child_value("enabled");
    if(std::stoi(readingValue))
        this->enableEstimationAttitudeSensorWrtRobot();


    /// Other Parameters
    pugi::xml_node parameters = sensor.child("parameters");

    // None



    //// Measurements
    pugi::xml_node measurements = sensor.child("measurements");

    /// Orientation
    pugi::xml_node meas_orientation = measurements.child("orientation");

    readingValue=meas_orientation.child_value("enabled");
    if(std::stoi(readingValue))
        this->enableMeasurementAttitude();

    readingValue=meas_orientation.child_value("var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoiseMeasurementAttitude(variance.asDiagonal());
    }


    /// Position
    pugi::xml_node meas_position = measurements.child("position");

    readingValue=meas_position.child_value("enabled");
    if(std::stoi(readingValue))
        this->enableMeasurementPosition();

    readingValue=meas_position.child_value("var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoiseMeasurementPosition(variance.asDiagonal());
    }




    //// Init State

    /// Pose of the sensor wrt robot

    // Position of the sensor wrt robot
    readingValue=pose_in_robot.child("position").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d init_estimation;
        stm>>init_estimation[0]>>init_estimation[1]>>init_estimation[2];
        SensorInitStateCore->setPositionSensorWrtRobot(init_estimation);
    }

    // Attitude of the sensor wrt robot
    readingValue=pose_in_robot.child("attitude").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector4d init_estimation;
        stm>>init_estimation[0]>>init_estimation[1]>>init_estimation[2]>>init_estimation[3];
        SensorInitStateCore->setAttitudeSensorWrtRobot(init_estimation);
    }


    /// Parameters

    // None



    //// Init Variances


    /// Pose of the sensor wrt robot

    // Position of the sensor wrt robot
    readingValue=pose_in_robot.child("position").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoisePositionSensorWrtRobot(variance.asDiagonal());
    }


    // Attitude of the sensor wrt robot
    readingValue=pose_in_robot.child("attitude").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoiseAttitudeSensorWrtRobot(variance.asDiagonal());
    }



    /// Other Parameters

    // None


    // Noises in the estimation (if enabled)

    // None


    // Prepare covariance matrix
    this->prepareInitErrorStateVariance();


    /// Finish

    // Open
    this->open();

    // End
    return 0;
}
