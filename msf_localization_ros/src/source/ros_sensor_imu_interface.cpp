
#include "msf_localization_ros/ros_sensor_imu_interface.h"





RosSensorImuInterface::RosSensorImuInterface(ros::NodeHandle* nh, std::weak_ptr<MsfStorageCore> the_msf_storage_core) :
    RosSensorInterface(),
    ImuSensorCore(the_msf_storage_core)
{
    //std::cout<<"RosSensorImuInterface::RosSensorImuInterface(ros::NodeHandle* nh, std::weak_ptr<MsfStorageCore> the_msf_storage_core)"<<std::endl;

    // Node Handle
    this->nh=nh;

    // Create the variable in the MSF Localization Core
    //TheSensorCore = new ImuSensorCore;

    //ImuSensorCore TheImuSensorCore;
    //TheMsfLocalizationCore->TheListOfSensorCore.push_back(TheImuSensorCore);



    return;
}



int RosSensorImuInterface::setImuTopicName(std::string ImuTopicName)
{
    this->ImuTopicName=ImuTopicName;
    return 0;
}


int RosSensorImuInterface::setMeasurementRos(const sensor_msgs::ImuConstPtr& msg)
{
    if(!isSensorEnabled())
        return 0;

    // PROVISIONAL! -> Dischart part of the measurements
    if(msg->header.seq  % 4 != 0)
        return 0;

    //std::cout<<"ROS Imu Measured"<<std::endl;

    //ImuSensorCore* TheSensorCore=dynamic_cast<ImuSensorCore*>(this->TheSensorCore);
    //TheSensorCore->setMeasurement();


    // Sensor Id
    //std::cout<<"Sensor Id="<<this->sensorId<<std::endl;


    // Time Stamp
    TimeStamp TheTimeStamp(msg->header.stamp.sec, msg->header.stamp.nsec);


    // Sensor Core
    std::shared_ptr<ImuSensorCore> TheImuSensorCore=std::dynamic_pointer_cast<ImuSensorCore>(this->getMsfElementCoreSharedPtr());

    // Value
    std::shared_ptr<ImuSensorMeasurementCore> TheImuSensorMeasurementCore=std::make_shared<ImuSensorMeasurementCore>(TheImuSensorCore);

    // Set the sensor core -> Associate it to the SensorCore
    //TheImuSensorMeasurementCore->setTheSensorCore(std::dynamic_pointer_cast<ImuSensorCore>(this));
    //std::weak_ptr<const ImuSensorCore> TheImuSensorCorePtrAux=std::static_pointer_cast<const ImuSensorCore>((this));
    //TheImuSensorMeasurementCore->setTheSensorCore(TheImuSensorCorePtrAux);

    //TheImuSensorMeasurementCore->setTheSensorCore(TheImuSensorCore);


    // Orientation if enabled
    if(this->isMeasurementOrientationEnabled())
    {
        Eigen::Vector4d orientation(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

        if(TheImuSensorMeasurementCore->setOrientation(orientation))
            std::cout<<"Error setting orientation"<<std::endl;
    }

    //msg->orientation_covariance;
    // TODO


    // Angular velocity if enabled
    if(this->isMeasurementAngularVelocityEnabled())
    {
        Eigen::Vector3d angular_velocity(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
        if(TheImuSensorMeasurementCore->setAngularVelocity(angular_velocity))
            std::cout<<"Error setting angular_velocity"<<std::endl;
    }

    //msg->angular_velocity_covariance;
    // TODO


    // Linear acceleration if enabled
    if(this->isMeasurementLinearAccelerationEnabled())
    {
        Eigen::Vector3d linear_acceleration(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);

        if(TheImuSensorMeasurementCore->setLinearAcceleration(linear_acceleration))
            std::cout<<"Error setting linear_acceleration"<<std::endl;
    }

    //msg->linear_acceleration_covariance;
    // TODO

    // Set
    this->setMeasurement(TheTimeStamp, TheImuSensorMeasurementCore);

    return 0;
}


void RosSensorImuInterface::imuTopicCallback(const sensor_msgs::ImuConstPtr& msg)
{
    //logFile<<"RosSensorImuInterface::imuTopicCallback()"<<std::endl;

    this->setMeasurementRos(msg);


    //logFile<<"RosSensorImuInterface::imuTopicCallback() ended"<<std::endl;
    return;
}


int RosSensorImuInterface::open()
{

    //std::cout<<"RosSensorImuInterface::open()"<<std::endl;

    // Node handler
    //ros::NodeHandle nh;

    // Subscriber
    ImuTopicSub=nh->subscribe(ImuTopicName, 10, &RosSensorImuInterface::imuTopicCallback, this);


    return 0;
}

int RosSensorImuInterface::readConfig(pugi::xml_node sensor, unsigned int sensorId, std::shared_ptr<ImuSensorStateCore>& SensorInitStateCore)
{
    // Sensor Core Pointer
    //std::shared_ptr<ImuSensorCore> TheImuSensorCore(this);
    std::shared_ptr<ImuSensorCore> TheImuSensorCore=std::dynamic_pointer_cast<ImuSensorCore>(this->getMsfElementCoreSharedPtr());

    // Set pointer to the SensorCore
    //this->setMsfElementCorePtr(TheRosSensorImuInterface);
    //this->setMsfElementCorePtr(TheImuSensorCore);

    // Create a class for the SensorStateCore
    if(!SensorInitStateCore)
        SensorInitStateCore=std::make_shared<ImuSensorStateCore>();

    // Set pointer to the SensorCore
    //SensorInitStateCore->setTheSensorCore(TheRosSensorImuInterface);
    SensorInitStateCore->setTheSensorCore(TheImuSensorCore);


    // Set sensor type
    //this->setSensorType(SensorTypes::imu);

    // Set Id
    this->setSensorId(sensorId);

    // Set the access to the Storage core
    //TheRosSensorImuInterface->setTheMsfStorageCore(std::make_shared<MsfStorageCore>(this->TheStateEstimationCore));
    //this->setMsfStorageCorePtr(TheMsfStorageCore);


    // Sensor Topic
    std::string sensorTopic=sensor.child_value("ros_topic");
    this->setImuTopicName(sensorTopic);


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

    // Angular Velocity
    pugi::xml_node param_angular_velocity = parameters.child("angular_velocity");

    // Angular Velocity Biases
    readingValue=param_angular_velocity.child("biases").child_value("enabled");
    if(std::stoi(readingValue))
    {
        this->enableEstimationBiasAngularVelocity();
    }

    // Angular Velocity Scale
    readingValue=param_angular_velocity.child("scale").child_value("enabled");
    if(std::stoi(readingValue))
    {
        this->enableEstimationScaleAngularVelocity();
    }

    // Linear Acceleration
    pugi::xml_node param_linear_acceleration = parameters.child("linear_acceleration");

    // Linear Acceleration Biases
    readingValue=param_linear_acceleration.child("biases").child_value("enabled");
    if(std::stoi(readingValue))
        this->enableEstimationBiasLinearAcceleration();

    // Linear Acceleration Scale
    readingValue=param_linear_acceleration.child("scale").child_value("enabled");
    if(std::stoi(readingValue))
        this->enableEstimationScaleLinearAcceleration();




    //// Measurements
    pugi::xml_node measurements = sensor.child("measurements");


    /// Linear Acceleration
    pugi::xml_node meas_linear_acceleration = measurements.child("linear_acceleration");

    readingValue=meas_linear_acceleration.child_value("enabled");
    if(std::stoi(readingValue))
        this->enableMeasurementLinearAcceleration();

    readingValue=meas_linear_acceleration.child_value("var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoiseMeasurementLinearAcceleration(variance.asDiagonal());
    }


    /// Orientation
    pugi::xml_node orientation = measurements.child("orientation");

    readingValue=orientation.child_value("enabled");
    // TODO

    readingValue=orientation.child_value("var");
    // TODO


    /// Angular Velocity
    pugi::xml_node meas_angular_velocity = measurements.child("angular_velocity");

    readingValue=meas_angular_velocity.child_value("enabled");
    if(std::stoi(readingValue))
        this->enableMeasurementAngularVelocity();

    readingValue=meas_angular_velocity.child_value("var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoiseMeasurementAngularVelocity(variance.asDiagonal());
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

    // Bias Angular Velocity
    readingValue=param_angular_velocity.child("biases").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d init_estimation;
        stm>>init_estimation[0]>>init_estimation[1]>>init_estimation[2];
        SensorInitStateCore->setBiasesAngularVelocity(init_estimation);
    }

    // Scale Angular Velocity
    readingValue=param_angular_velocity.child("scale").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d init_estimation;
        stm>>init_estimation[0]>>init_estimation[1]>>init_estimation[2];
        SensorInitStateCore->setScaleAngularVelocity(init_estimation);
    }

    // Bias Linear Acceleration
    readingValue=param_linear_acceleration.child("biases").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d init_estimation;
        stm>>init_estimation[0]>>init_estimation[1]>>init_estimation[2];
        SensorInitStateCore->setBiasesLinearAcceleration(init_estimation);
    }

    // Scale Linear Acceleration
    readingValue=param_linear_acceleration.child("scale").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d init_estimation;
        stm>>init_estimation[0]>>init_estimation[1]>>init_estimation[2];
        SensorInitStateCore->setScaleLinearAcceleration(init_estimation);
    }




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

    // Bias Linear Acceleration
    readingValue=param_linear_acceleration.child("biases").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoiseBiasLinearAcceleration(variance.asDiagonal());
    }

    // Scale Linear Acceleration
    readingValue=param_linear_acceleration.child("scale").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoiseScaleLinearAcceleration(variance.asDiagonal());
    }


    // Bias Angular Velocity
    readingValue=param_angular_velocity.child("biases").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoiseBiasAngularVelocity(variance.asDiagonal());
    }

    // Scale Angular Velocity
    readingValue=param_angular_velocity.child("scale").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoiseScaleAngularVelocity(variance.asDiagonal());
    }



    // Noises in the estimation (if enabled)

    // Bias Linear Acceleration
    if(this->isEstimationBiasLinearAccelerationEnabled())
    {
        readingValue=param_linear_acceleration.child("biases").child_value("noise");
        {
            std::istringstream stm(readingValue);
            Eigen::Vector3d variance;
            stm>>variance[0]>>variance[1]>>variance[2];
            this->setNoiseEstimationBiasLinearAcceleration(variance.asDiagonal());
        }
    }

    // Bias Angular Velocity
    if(this->isEstimationBiasAngularVelocityEnabled())
    {
        readingValue=param_angular_velocity.child("biases").child_value("noise");
        {
            std::istringstream stm(readingValue);
            Eigen::Vector3d variance;
            stm>>variance[0]>>variance[1]>>variance[2];
            this->setNoiseEstimationBiasAngularVelocity(variance.asDiagonal());
        }
    }


    // Prepare covariance matrix
    this->prepareInitErrorStateVariance();



    /// Finish

    // Open
    this->open();

    // End
    return 0;
}
