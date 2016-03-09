
#include "msf_localization_ros/droneMsfLocalizationROSModule.h"

MsfLocalizationROS::MsfLocalizationROS(int argc,char **argv)
{
    //Ros Init
    ros::init(argc, argv, ros::this_node::getName());

    // Node handle
    nh=new ros::NodeHandle();

    // tf broadcaster
    tfTransformBroadcaster=new tf::TransformBroadcaster;

    // Init
    init();

    return;
}

MsfLocalizationROS::~MsfLocalizationROS()
{
    // Close
    close();

    // Delete
    delete tfTransformBroadcaster;

    delete nh;

    return;
}


int MsfLocalizationROS::setConfigFile(std::string configFile)
{
    this->configFile=configFile;
    return 0;
}


int MsfLocalizationROS::readConfigFile()
{
    // ROS Node Handle
    //ros::NodeHandle nh;

    // File
    if ( !boost::filesystem::exists( configFile ) )
    {
        std::cout << "Can't find the file: "<< configFile << std::endl;
        return 1;
    }

    //XML document
    pugi::xml_document doc;
    std::ifstream nameFile(configFile.c_str());
    pugi::xml_parse_result result = doc.load(nameFile);

    if(!result)
    {
        std::cout<<"I cannot open xml file: "<<configFile<<std::endl;
        return 2;
    }

    //
    pugi::xml_node msf_localization = doc.child("msf_localization");

    // Aux vars
    std::string readingValue;


    // Create Initial Element of the buffer with the initial state
    std::shared_ptr<StateEstimationCore> InitialState=std::make_shared<StateEstimationCore>();



    // Reading Configs
    // TODO
    predictRateValue=10.0; // In Hz



    //// Global Parameters

    {
        // Reading Robot
        pugi::xml_node global_parameters = msf_localization.child("global_parameters");

        // Create the CoreAux
        std::shared_ptr<GlobalParametersCore> TheGlobalParametersCoreAux;
        // Create a class for the RobotStateCore
        std::shared_ptr<GlobalParametersStateCore> TheGlobalParametersStateCore;


        // Read configs
        if(readGlobalParametersConfig(global_parameters, this->TheMsfStorageCore, TheGlobalParametersCoreAux, TheGlobalParametersStateCore))
            return -2;


        // Finish

        // Push the robot core
        TheGlobalParametersCore=TheGlobalParametersCoreAux;

        // Push the init state of the robot
        InitialState->TheGlobalParametersStateCore=TheGlobalParametersStateCore;

    }


    ///// Robot

    {
        // Reading Robot
        pugi::xml_node robot = msf_localization.child("robot");

        // Create the RobotCoreAux
        std::shared_ptr<FreeModelRobotCore> TheRobotCoreAux;
        // Create a class for the RobotStateCore
        std::shared_ptr<FreeModelRobotStateCore> RobotInitStateCore;
//        // Robot Init State Covariance Matrix
//        Eigen::MatrixXd InitStateCovatrianceMatrix;

        // Read configs
        if(readFreeModelRobotConfig(robot, this->TheMsfStorageCore, TheRobotCoreAux, RobotInitStateCore))
            return -2;


//        // Update covariance
//        unsigned int previousNumCols=InitialState->covarianceMatrix.cols();
//        unsigned int previousNumRows=InitialState->covarianceMatrix.rows();

//        InitialState->covarianceMatrix.conservativeResize(previousNumRows+InitStateCovatrianceMatrix.rows(), previousNumCols+InitStateCovatrianceMatrix.cols());
//        InitialState->covarianceMatrix.block(previousNumRows, previousNumCols, InitStateCovatrianceMatrix.rows(), InitStateCovatrianceMatrix.cols())=InitStateCovatrianceMatrix;


        // Finish

        // Push the robot core
        TheRobotCore=TheRobotCoreAux;

        // Push the init state of the robot
        InitialState->TheRobotStateCore=RobotInitStateCore;

    }


    ///// Sensors
    // Reading sensors
    for(pugi::xml_node sensor = msf_localization.child("sensor"); sensor; sensor = sensor.next_sibling("sensor"))
    {
        std::cout<<"sensor; "<<std::endl;

        // Sensor Type
        std::string sensorType=sensor.child_value("type");


        //// IMU Sensor Type
        if(sensorType=="imu")
        {

            // Create a class for the SensoreCore
            std::shared_ptr<RosSensorImuInterface> TheRosSensorImuInterface;
            // Create a class for the SensorStateCore
            std::shared_ptr<ImuSensorStateCore> SensorInitStateCore;

            // Create a matrix with the additional covariance
            //Eigen::MatrixXd InitStateCovatrianceMatrix;

            // Read configs
            if(readImuConfig(sensor, firstAvailableId, this->TheMsfStorageCore, TheRosSensorImuInterface, SensorInitStateCore))
                return -2;

            // Update id
            firstAvailableId++;

//            // Update covariance
//            unsigned int previousNumCols=InitialState->covarianceMatrix.cols();
//            unsigned int previousNumRows=InitialState->covarianceMatrix.rows();

//            InitialState->covarianceMatrix.conservativeResize(previousNumRows+InitStateCovatrianceMatrix.rows(), previousNumCols+InitStateCovatrianceMatrix.cols());
//            InitialState->covarianceMatrix.block(previousNumRows, previousNumCols, InitStateCovatrianceMatrix.rows(), InitStateCovatrianceMatrix.cols())=InitStateCovatrianceMatrix;


            // Finish

            // Push to the list of sensors
            this->TheListOfSensorCore.push_back(TheRosSensorImuInterface);

            // Push the init state of the sensor
            InitialState->TheListSensorStateCore.push_back(SensorInitStateCore);
        }


    }


    // Initial State Prepare
    InitialState->prepareInitErrorStateVariance();



    // Display
#if _DEBUG_MSF_LOCALIZATION_CORE
    logFile<<"Covariance Matrix"<<std::endl;
    logFile<<InitialState->covarianceMatrix<<std::endl;
#endif




    // Add init state to the buffer
    TheMsfStorageCore->addElement(TimeStamp(0,0), InitialState);


    return 0;
}


int MsfLocalizationROS::readGlobalParametersConfig(pugi::xml_node global_parameters, std::shared_ptr<MsfStorageCore> TheMsfStorageCore, std::shared_ptr<GlobalParametersCore>& TheGlobalParametersCoreAux, std::shared_ptr<GlobalParametersStateCore>& GlobalParametersInitStateCore)
{
    // Create the GlobalParameters
    if(!TheGlobalParametersCoreAux)
        TheGlobalParametersCoreAux=std::make_shared<GlobalParametersCore>(TheMsfStorageCore);

    // Set the access to the Storage core -> Not needed
    TheGlobalParametersCoreAux->setTheMsfStorageCore(TheMsfStorageCore);

    // Set pointer to the TheGlobalParametersCoreAux
    TheGlobalParametersCoreAux->setTheGlobalParametersCore(TheGlobalParametersCoreAux);


    // Create a class for the TheGlobalParametersCore
    if(!GlobalParametersInitStateCore)
        GlobalParametersInitStateCore=std::make_shared<GlobalParametersStateCore>(TheGlobalParametersCoreAux);

    // Set pointer to the Core -> Not needed!
    GlobalParametersInitStateCore->setTheGlobalParametersCore(TheGlobalParametersCoreAux);


    // Aux vars
    std::string readingValue;


    /// World name
    readingValue=global_parameters.child_value("name");
    TheGlobalParametersCoreAux->setWorldName(readingValue);


    /// Init State

    // Gravity
    readingValue=global_parameters.child("gravity").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d init_state;
        stm>>init_state[0]>>init_state[1]>>init_state[2];
        GlobalParametersInitStateCore->setGravity(init_state);
    }



    //// Init Variances


    // Gravity
    readingValue=global_parameters.child("gravity").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        TheGlobalParametersCoreAux->setNoiseGravity(variance.asDiagonal());
    }


    // End
    return 0;
}



int MsfLocalizationROS::readFreeModelRobotConfig(pugi::xml_node robot, std::shared_ptr<MsfStorageCore> TheMsfStorageCore, std::shared_ptr<FreeModelRobotCore>& TheRobotCoreAux, std::shared_ptr<FreeModelRobotStateCore>& RobotInitStateCore)
{

    // Create the RobotCoreAux
    if(!TheRobotCoreAux)
        TheRobotCoreAux=std::make_shared<FreeModelRobotCore>();
    // Set pointer to the RobotCore
    TheRobotCoreAux->setTheRobotCore(TheRobotCoreAux);

    // Set robot type
    TheRobotCoreAux->setRobotType(RobotTypes::free_model);

    // Set the access to the Storage core
    TheRobotCoreAux->setTheMsfStorageCore(TheMsfStorageCore);

    // Create a class for the RobotStateCore
    if(!RobotInitStateCore)
        RobotInitStateCore=std::make_shared<FreeModelRobotStateCore>();
    // Set pointer to the SensorCore
    RobotInitStateCore->setTheRobotCore(TheRobotCoreAux);


    // Aux vars
    std::string readingValue;


    // Name
    readingValue=robot.child_value("name");
    TheRobotCoreAux->setRobotName(readingValue);


    /// Init State

    // Position
    readingValue=robot.child("position").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d position;
        stm>>position[0]>>position[1]>>position[2];
        RobotInitStateCore->setPosition(position);
    }

    // Linear Speed
    readingValue=robot.child("lin_speed").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d lin_speed;
        stm>>lin_speed[0]>>lin_speed[1]>>lin_speed[2];
        RobotInitStateCore->setLinearSpeed(lin_speed);
    }

    // Linear Acceleration
    readingValue=robot.child("lin_accel").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d lin_accel;
        stm>>lin_accel[0]>>lin_accel[1]>>lin_accel[2];
        RobotInitStateCore->setLinearAcceleration(lin_accel);
    }

    // Attitude
    readingValue=robot.child("attitude").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector4d attitude;
        stm>>attitude[0]>>attitude[1]>>attitude[2]>>attitude[3];
        RobotInitStateCore->setAttitude(attitude);
    }

    // Angular Velocity
    readingValue=robot.child("ang_velocity").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d ang_velocity;
        stm>>ang_velocity[0]>>ang_velocity[1]>>ang_velocity[2];
        RobotInitStateCore->setAngularVelocity(ang_velocity);
    }

    // Angular Acceleration
    readingValue=robot.child("ang_accel").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d auxVec;
        stm>>auxVec[0]>>auxVec[1]>>auxVec[2];
        RobotInitStateCore->setAngularAcceleration(auxVec);
    }


    /// Init Variances

    // Position
    readingValue=robot.child("position").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        TheRobotCoreAux->setInitErrorStateVariancePosition(variance);
    }

    // Linear Speed
    readingValue=robot.child("lin_speed").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        TheRobotCoreAux->setInitErrorStateVarianceLinearSpeed(variance);
    }

    // Linear acceleration
    readingValue=robot.child("lin_accel").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        TheRobotCoreAux->setInitErrorStateVarianceLinearAcceleration(variance);
    }

    // Attitude
    readingValue=robot.child("attitude").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        TheRobotCoreAux->setInitErrorStateVarianceAttitude(variance);
    }

    // Angular velocity
    readingValue=robot.child("ang_velocity").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        TheRobotCoreAux->setInitErrorStateVarianceAngularVelocity(variance);
    }

    // Angular acceleration
    readingValue=robot.child("ang_accel").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        TheRobotCoreAux->setInitErrorStateVarianceAngularAcceleration(variance);
    }



    // Noises Estimation

    // Linear acceleration
    readingValue=robot.child("lin_accel").child_value("noise");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        TheRobotCoreAux->setNoiseLinearAcceleration(variance.asDiagonal());
    }

//    // Angular velocity -> Not used
//    readingValue=robot.child("ang_velocity").child_value("noise");
//    {
//        std::istringstream stm(readingValue);
//        Eigen::Vector3d variance;
//        stm>>variance[0]>>variance[1]>>variance[2];
//        TheRobotCoreAux->setNoiseAngularVelocity(variance.asDiagonal());
//    }

    // Angular acceleration
    readingValue=robot.child("ang_accel").child_value("noise");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        TheRobotCoreAux->setNoiseAngularAcceleration(variance.asDiagonal());
    }


    // End
    return 0;
}


int MsfLocalizationROS::readImuConfig(pugi::xml_node sensor, unsigned int sensorId, std::shared_ptr<MsfStorageCore> TheMsfStorageCore, std::shared_ptr<RosSensorImuInterface>& TheRosSensorImuInterface, std::shared_ptr<ImuSensorStateCore>& SensorInitStateCore)
{
    // Create a class for the SensoreCore
    if(!TheRosSensorImuInterface)
        TheRosSensorImuInterface=std::make_shared<RosSensorImuInterface>(nh);

    // Set pointer to the SensorCore
    TheRosSensorImuInterface->setTheSensorCore(TheRosSensorImuInterface);

    // Create a class for the SensorStateCore
    if(!SensorInitStateCore)
        SensorInitStateCore=std::make_shared<ImuSensorStateCore>();

    // Set pointer to the SensorCore
    SensorInitStateCore->setTheSensorCore(TheRosSensorImuInterface);


    // Set sensor type
    TheRosSensorImuInterface->setSensorType(SensorTypes::imu);

    // Set Id
    TheRosSensorImuInterface->setSensorId(sensorId);

    // Set the access to the Storage core
    //TheRosSensorImuInterface->setTheMsfStorageCore(std::make_shared<MsfStorageCore>(this->TheStateEstimationCore));
    TheRosSensorImuInterface->setTheMsfStorageCore(TheMsfStorageCore);


    // Sensor Topic
    std::string sensorTopic=sensor.child_value("ros_topic");
    TheRosSensorImuInterface->setImuTopicName(sensorTopic);


    // Auxiliar reading value
    std::string readingValue;


    // Name
    readingValue=sensor.child_value("name");
    TheRosSensorImuInterface->setSensorName(readingValue);


    //// Sensor configurations


    /// Pose of the sensor wrt robot
    pugi::xml_node pose_in_robot=sensor.child("pose_in_robot");

    // Position of the sensor wrt robot
    readingValue=pose_in_robot.child("position").child_value("enabled");
    if(std::stoi(readingValue))
        TheRosSensorImuInterface->enableEstimationPositionSensorWrtRobot();

    // Attitude of the sensor wrt robot
    readingValue=pose_in_robot.child("attitude").child_value("enabled");
    if(std::stoi(readingValue))
        TheRosSensorImuInterface->enableEstimationAttitudeSensorWrtRobot();


    /// Other Parameters
    pugi::xml_node parameters = sensor.child("parameters");

    // Angular Velocity
    pugi::xml_node param_angular_velocity = parameters.child("angular_velocity");

    // Angular Velocity Biases
    readingValue=param_angular_velocity.child("biases").child_value("enabled");
    if(std::stoi(readingValue))
    {
        TheRosSensorImuInterface->enableEstimationBiasAngularVelocity();
    }

    // Angular Velocity Scale
    readingValue=param_angular_velocity.child("scale").child_value("enabled");
    if(std::stoi(readingValue))
    {
        TheRosSensorImuInterface->enableEstimationScaleAngularVelocity();
    }

    // Linear Acceleration
    pugi::xml_node param_linear_acceleration = parameters.child("linear_acceleration");

    // Linear Acceleration Biases
    readingValue=param_linear_acceleration.child("biases").child_value("enabled");
    if(std::stoi(readingValue))
        TheRosSensorImuInterface->enableEstimationBiasLinearAcceleration();

    // Linear Acceleration Scale
    readingValue=param_linear_acceleration.child("scale").child_value("enabled");
    if(std::stoi(readingValue))
        TheRosSensorImuInterface->enableEstimationScaleLinearAcceleration();




    //// Measurements
    pugi::xml_node measurements = sensor.child("measurements");

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
        TheRosSensorImuInterface->enableMeasurementAngularVelocity();

    readingValue=meas_angular_velocity.child_value("var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        TheRosSensorImuInterface->setNoiseMeasurementAngularVelocity(variance.asDiagonal());
    }



    /// Linear Acceleration
    pugi::xml_node meas_linear_acceleration = measurements.child("linear_acceleration");

    readingValue=meas_linear_acceleration.child_value("enabled");
    if(std::stoi(readingValue))
        TheRosSensorImuInterface->enableMeasurementLinearAcceleration();

    readingValue=meas_linear_acceleration.child_value("var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        TheRosSensorImuInterface->setNoiseMeasurementLinearAcceleration(variance.asDiagonal());
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
        TheRosSensorImuInterface->setNoisePositionSensorWrtRobot(variance.asDiagonal());
    }


    // Attitude of the sensor wrt robot
    readingValue=pose_in_robot.child("attitude").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        TheRosSensorImuInterface->setNoiseAttitudeSensorWrtRobot(variance.asDiagonal());
    }



    /// Other Parameters

    // Bias Linear Acceleration
    readingValue=param_linear_acceleration.child("biases").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        TheRosSensorImuInterface->setNoiseBiasLinearAcceleration(variance.asDiagonal());
    }

    // Scale Linear Acceleration
    readingValue=param_linear_acceleration.child("scale").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        TheRosSensorImuInterface->setNoiseScaleLinearAcceleration(variance.asDiagonal());
    }


    // Bias Angular Velocity
    readingValue=param_angular_velocity.child("biases").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        TheRosSensorImuInterface->setNoiseBiasAngularVelocity(variance.asDiagonal());
    }

    // Scale Angular Velocity
    readingValue=param_angular_velocity.child("scale").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        TheRosSensorImuInterface->setNoiseScaleAngularVelocity(variance.asDiagonal());
    }



    // Noises in the estimation (if enabled)

    // Bias Linear Acceleration
    if(TheRosSensorImuInterface->isEstimationBiasLinearAccelerationEnabled())
    {
        readingValue=param_linear_acceleration.child("biases").child_value("noise");
        {
            std::istringstream stm(readingValue);
            Eigen::Vector3d variance;
            stm>>variance[0]>>variance[1]>>variance[2];
            TheRosSensorImuInterface->setNoiseEstimationBiasLinearAcceleration(variance.asDiagonal());
        }
    }

    // Bias Angular Velocity
    if(TheRosSensorImuInterface->isEstimationBiasAngularVelocityEnabled())
    {
        readingValue=param_angular_velocity.child("biases").child_value("noise");
        {
            std::istringstream stm(readingValue);
            Eigen::Vector3d variance;
            stm>>variance[0]>>variance[1]>>variance[2];
            TheRosSensorImuInterface->setNoiseEstimationBiasAngularVelocity(variance.asDiagonal());
        }
    }


    // Prepare covariance matrix
    TheRosSensorImuInterface->prepareInitErrorStateVariance();



    /// Finish

    // Open
    TheRosSensorImuInterface->open();

    // End
    return 0;
}




int MsfLocalizationROS::readParameters()
{
    // Config files
    //
    ros::param::param<std::string>("~msf_localization_config_file", configFile, "msf_localization_config_file.xml");
    std::cout<<"msf_localization_config_file="<<configFile<<std::endl;

    // Topic names
    //
    ros::param::param<std::string>("~robot_pose_topic_name", robotPoseTopicName, "msf_localization/robot_pose");
    std::cout<<"robot_pose_topic_name="<<robotPoseTopicName<<std::endl;

    // Service names
    //
    ros::param::param<std::string>("~set_state_estimation_enabled_service_name", setStateEstimationEnabledServiceName, "msf_localization/set_state_estimation_enabled");
    std::cout<<"set_state_estimation_enabled_service_name="<<setStateEstimationEnabledServiceName<<std::endl;


    // Others configs
    //
    ros::param::param<double>("~robot_pose_rate", robotPoseRateVal, 50);
    std::cout<<"robot_pose_rate="<<robotPoseRateVal<<std::endl;

    return 0;
}


int MsfLocalizationROS::init()
{
    return 0;
}

int MsfLocalizationROS::close()
{

    return 0;
}

int MsfLocalizationROS::open()
{
    // ROS Node Handle
    //ros::NodeHandle nh;

    // Read parameters
    if(readParameters())
        ROS_ERROR("Error Reading Parameters");

    // Read config file
    if(readConfigFile())
        ROS_ERROR("Error Reading Config File");


    // Robot Pose Publisher
    robotPosePub = nh->advertise<geometry_msgs::PoseWithCovarianceStamped>(robotPoseTopicName, 1, true);


    // Service
    setStateEstimationEnabledSrv = nh->advertiseService(setStateEstimationEnabledServiceName, &MsfLocalizationROS::setStateEstimationEnabledCallback, this);


    return 0;
}


bool MsfLocalizationROS::setStateEstimationEnabledCallback(msf_localization_ros_srvs::SetBool::Request  &req, msf_localization_ros_srvs::SetBool::Response &res)
{
    if(!this->setStateEstimationEnabled(req.data))
    {
        res.success=true;
    }
    else
    {
        res.success=false;
    }


    return true;
}


int MsfLocalizationROS::robotPoseThreadFunction()
{
#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationROS::robotPoseThreadFunction()"<<std::endl;
        this->log(logString.str());
    }
#endif

//return 0;
    robotPoseRate=new ros::Rate(robotPoseRateVal);
try
{
    while(ros::ok())
    {
//        if(!this->isStateEstimationEnabled())
//        {
//            // Sleep
//            robotPoseRate->sleep();
//            // continue
//            continue;
//        }
#if _DEBUG_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationROS::robotPoseThreadFunction() loop init"<<std::endl;
            this->log(logString.str());
        }
#endif

        // Get Robot Pose
        // TODO
        TimeStamp TheTimeStamp;
        std::shared_ptr<StateEstimationCore> PredictedState;



        if(this->isStateEstimationEnabled())
        {

            TheTimeStamp=getTimeStamp();

        this->TheMsfStorageCore->getElement(TheTimeStamp, PredictedState);

        if(!PredictedState)
        {
//            this->robotPoseThreadState.setProcessing(TheTimeStamp);
            // TODO this should be a while and being carefully with the memory
            if(this->predict(TheTimeStamp))
            {
                // Error
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
                {
                    std::ostringstream logString;
                    logString<<"MsfLocalizationROS::robotPoseThreadFunction() error in predict()"<<std::endl;
                    this->log(logString.str());
                }
#endif
                continue;
            }
//            this->robotPoseThreadState.setNotProcessing();
        }


//        if()
//        {
//            std::cout<<"error getting robot pose"<<std::endl;
//            continue;
//        }



        // Get the outdated element to do the update
        if(this->TheMsfStorageCore->getElement(TheTimeStamp, PredictedState))
        {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationROS::robotPoseThreadFunction() error 1!"<<std::endl;
                this->log(logString.str());
            }

#endif
            continue;
        }

        }
        else
        {
            // Get the last state estimation
            this->TheMsfStorageCore->getLastElementWithStateEstimate(TheTimeStamp, PredictedState);


            // Time Stamp is null, put the current one
            if(TheTimeStamp==TimeStamp(0,0))
            {
                TheTimeStamp=getTimeStamp();
            }

        }


        if(!PredictedState)
        {
            continue;
        }


        // Fill msg
        // Header
        robotPoseMsg.header.stamp=ros::Time(TheTimeStamp.sec, TheTimeStamp.nsec);

        // Frame id
        // TODO put as a ros param
        robotPoseMsg.header.frame_id=this->TheGlobalParametersCore->getWorldName();
//if(0)
//{
        // Robot Pose
        std::shared_ptr<FreeModelRobotStateCore> TheRobotStateCore=std::static_pointer_cast<FreeModelRobotStateCore>(PredictedState->TheRobotStateCore);
        Eigen::Vector3d robotPosition=TheRobotStateCore->getPosition();
        Eigen::Vector4d robotAttitude=TheRobotStateCore->getAttitude();

        // Position
        robotPoseMsg.pose.pose.position.x=robotPosition[0];
        robotPoseMsg.pose.pose.position.y=robotPosition[1];
        robotPoseMsg.pose.pose.position.z=robotPosition[2];

        // Attitude
        robotPoseMsg.pose.pose.orientation.w=robotAttitude[0];
        robotPoseMsg.pose.pose.orientation.x=robotAttitude[1];
        robotPoseMsg.pose.pose.orientation.y=robotAttitude[2];
        robotPoseMsg.pose.pose.orientation.z=robotAttitude[3];

        // Covariance
        // TODO fix! Covariance of the attitude is not ok!
        Eigen::MatrixXd robotPoseCovariance(6,6);
        robotPoseCovariance.setZero();
        robotPoseCovariance.block<3,3>(0,0)=PredictedState->covarianceMatrix.block<3,3>(0,0);
        robotPoseCovariance.block<3,3>(3,3)=PredictedState->covarianceMatrix.block<3,3>(9,9);
        double robotPoseCovarianceArray[36];
        Eigen::Map<Eigen::MatrixXd>(robotPoseCovarianceArray, 6, 6) = robotPoseCovariance;
        for(unsigned int i=0; i<36; i++)
        {
            robotPoseMsg.pose.covariance[i]=robotPoseCovarianceArray[i];
        }
//}


        // TF

        tf::Quaternion tf_rot(robotAttitude[1], robotAttitude[2], robotAttitude[3], robotAttitude[0]);
        tf::Vector3 tf_tran(robotPosition[0], robotPosition[1], robotPosition[2]);

        tf::Transform transform(tf_rot, tf_tran);

        tfTransformBroadcaster->sendTransform(tf::StampedTransform(transform, ros::Time(TheTimeStamp.sec, TheTimeStamp.nsec),
                                              this->TheGlobalParametersCore->getWorldName(), this->TheRobotCore->getRobotName()));




        for(std::list< std::shared_ptr<SensorStateCore> >::const_iterator itSensorState=PredictedState->TheListSensorStateCore.begin();
            itSensorState!=PredictedState->TheListSensorStateCore.end();
            ++itSensorState)
        {

            Eigen::Vector3d sensorPosition=(*itSensorState)->getPositionSensorWrtRobot();
            Eigen::Vector4d sensorAttitude=(*itSensorState)->getAttitudeSensorWrtRobot();

            tf::Quaternion tf_rot(sensorAttitude[1], sensorAttitude[2], sensorAttitude[3], sensorAttitude[0]);
            tf::Vector3 tf_tran(sensorPosition[0], sensorPosition[1], sensorPosition[2]);

            tf::Transform transform(tf_rot, tf_tran);


            tfTransformBroadcaster->sendTransform(tf::StampedTransform(transform, ros::Time(TheTimeStamp.sec, TheTimeStamp.nsec),
                                                  this->TheRobotCore->getRobotName(), (*itSensorState)->getTheSensorCore()->getSensorName()));

        }




//std::cout<<"aqui"<<std::endl;
        // Free the ownership
        if(PredictedState)
            PredictedState.reset();


        // Publish Robot Pose
        robotPosePub.publish(robotPoseMsg);

#if _DEBUG_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationROS::robotPoseThreadFunction() loop end"<<std::endl;
            this->log(logString.str());
        }
#endif


        // Sleep
        robotPoseRate->sleep();
    }

}
catch(std::exception &ex)
{
    std::cout<<"[ROSNODE] Exception :"<<ex.what()<<std::endl;
}
catch(...)
{
    std::cout<<"EXCEPTION ON Get robot pose Thread"<<std::endl;
}

#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationROS::robotPoseThreadFunction() ended"<<std::endl;
        this->log(logString.str());
    }
#endif

    return 0;
}

TimeStamp MsfLocalizationROS::getTimeStamp()
{
    ros::Time RosTimeStamp=ros::Time::now();

    return TimeStamp(RosTimeStamp.sec, RosTimeStamp.nsec);
}

int MsfLocalizationROS::predictThreadFunction()
{   
#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationROS::predictThreadFunction()"<<std::endl;
        this->log(logString.str());
    }
#endif

//return 0;
    ros::Rate predictRate(predictRateValue);
    //ros::Rate predictRate(100);

    try{

    while(ros::ok())
    {
        // Check if enabled
        if(!this->isStateEstimationEnabled())
        {
            // Sleep
            predictRate.sleep();
            // continue
            continue;
        }

#if _DEBUG_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationROS::predictThreadFunction() loop init"<<std::endl;
            this->log(logString.str());
        }
#endif


        // Search if element already exists
        // TODO
        TimeStamp TheTimeStamp=getTimeStamp();
        std::shared_ptr<StateEstimationCore> ThePredictedState;

        // Predict. Typically it will not be any
        //std::cout<<"Calling predict TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
        this->TheMsfStorageCore->getElement(TheTimeStamp, ThePredictedState);

        if(!ThePredictedState)
        {
//            predictThreadState.setProcessing(TheTimeStamp);
            // TODO Fix. This should be a while but being careful with the memory
            if(this->predict(TheTimeStamp))
            {
                // Error
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
                {
                    std::ostringstream logString;
                    logString<<"MsfLocalizationROS::predictThreadFunction() error in predict()"<<std::endl;
                    this->log(logString.str());
                }
#endif
                continue;
//            predictThreadState.setNotProcessing();
            }
        }

        if(ThePredictedState)
            ThePredictedState.reset();


        // Purge the buffer
        //this->TheMsfStorageCore->purgeRingBuffer(20);


        // Display the buffer
        //this->TheMsfStorageCore->displayRingBuffer();

#if _DEBUG_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationROS::predictThreadFunction() loop end"<<std::endl;
            this->log(logString.str());
        }
#endif


        // Sleep
        predictRate.sleep();
    }

}
catch(std::exception &ex)
{
    std::cout<<"[ROSNODE] Exception :"<<ex.what()<<std::endl;
}
catch(...)
{
    std::cout<<"EXCEPTION ON Predict Thread"<<std::endl;
}

#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationROS::predictThreadFunction() ended"<<std::endl;
        this->log(logString.str());
    }
#endif

    return 0;

}


int MsfLocalizationROS::bufferManagerThreadFunction()
{
#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationROS::bufferManagerThreadFunction()"<<std::endl;
        this->log(logString.str());
    }
#endif


    while(ros::ok())
    {

#if _DEBUG_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationROS::bufferManagerThreadFunction() loop init"<<std::endl;
            this->log(logString.str());
        }
#endif


        // Get oldest element
        TimeStamp OldestTimeStamp;
        if(this->TheMsfStorageCore->getOldestOutdatedElement(OldestTimeStamp))
        {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationROS::bufferManagerThreadFunction() error 0!"<<std::endl;
                this->log(logString.str());
            }
#endif
            continue;
        }

#if _DEBUG_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationROS::bufferManagerThreadFunction() updating TS: sec="<<OldestTimeStamp.sec<<" s; nsec="<<OldestTimeStamp.nsec<<" ns"<<std::endl;
            this->log(logString.str());
        }
#endif

#if _DEBUG_MSF_LOCALIZATION_CORE
        {
            this->log(this->TheMsfStorageCore->getDisplayOutdatedElements());
        }
#endif

//        // Get state estimation core associated to the oldest element on the buffer
//        std::shared_ptr<StateEstimationCore> TheOutdatedElement;
//        if(this->TheMsfStorageCore->getElement(OldestTimeStamp, TheOutdatedElement))
//        {
//#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
//            {
//                std::ostringstream logString;
//                logString<<"MsfLocalizationROS::bufferManagerThreadFunction() error 1!"<<std::endl;
//                this->log(logString.str());
//            }

//#endif
//            continue;
//        }

//        // Check
//        if(!TheOutdatedElement)
//        {
//#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
//            {
//                std::ostringstream logString;
//                logString<<"MsfLocalizationROS::bufferManagerThreadFunction() error2!"<<std::endl;
//                this->log(logString.str());
//            }

//#endif
//            continue;
//        }


        // Run predict and store updated predicted element
        if(this->predict(OldestTimeStamp))
        {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE

            {
                std::ostringstream logString;
                logString<<"MsfLocalizationROS::bufferManagerThreadFunction() error 5!"<<std::endl;
                this->log(logString.str());
            }

#endif
            continue;
        }

//        // Not needed, it should never happend
//        if(!TheOutdatedElement)
//        {
//#if _DEBUG_MSF_LOCALIZATION_CORE
//            {
//                std::ostringstream logString;
//                logString<<"MsfLocalizationROS::bufferManagerThreadFunction() error4!"<<std::endl;
//                this->log(logString.str());
//            }
//#endif
//            continue;
//        }


//        if(TheOutdatedElement)
//            TheOutdatedElement.reset();


//        // Get the outdated element to do the update
//        if(this->TheMsfStorageCore->getElement(OldestTimeStamp, TheOutdatedElement))
//        {
//#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
//            {
//                std::ostringstream logString;
//                logString<<"MsfLocalizationROS::bufferManagerThreadFunction() error 11!"<<std::endl;
//                this->log(logString.str());
//            }

//#endif
//            continue;
//        }

//        // Check
//        if(!TheOutdatedElement)
//        {
//#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
//            {
//                std::ostringstream logString;
//                logString<<"MsfLocalizationROS::bufferManagerThreadFunction() error2!"<<std::endl;
//                this->log(logString.str());
//            }

//#endif
//            continue;
//        }



        // Run update if there are measurements
        // TODO
        int errorUpdate=0;
//        try
//        {
            errorUpdate=this->update(OldestTimeStamp);
//        }
//        catch(...)
//        {
//            std::cout<<"EXCEPTION ON UPDATE"<<std::endl;
//        }



#if _DEBUG_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationROS::bufferManagerThreadFunction() update response: "<<errorUpdate<<std::endl;
            this->log(logString.str());
        }
#endif


//        // Free the ownership
//        if(TheOutdatedElement)
//            TheOutdatedElement.reset();


        if(errorUpdate)
        {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationROS::bufferManagerThreadFunction() error in the update"<<std::endl;
                this->log(logString.str());
            }
#endif

            // Add to the processing list
            this->TheMsfStorageCore->addOutdatedElement(OldestTimeStamp);

            // Continue
            continue;

        }


        // If some thread is predicting something, better to put this element in the outdated element, just in case
        // Not working! I will need to wait until they finish of processing them before working again
        // Thread robot pose
//        if(robotPoseThreadState.isWorking())
//        {
//            TimeStamp TheProcesingTimeStamp;
//            TheProcesingTimeStamp=robotPoseThreadState.getProcessingTimeStamp();
//            this->TheMsfStorageCore->addOutdatedElement(TheProcesingTimeStamp);
//        }
//        // Thread predict
//        if(predictThreadState.isWorking())
//        {
//            TimeStamp TheProcesingTimeStamp;
//            TheProcesingTimeStamp=predictThreadState.getProcessingTimeStamp();
//            this->TheMsfStorageCore->addOutdatedElement(TheProcesingTimeStamp);
//        }


        // Find the next element in the buffer and mark it as outdated
        TimeStamp TheNewOutdatedTimeStamp;
        // TODO
        //std::cout<<"MsfLocalizationROS::bufferManagerThreadFunction() updating TS: sec="<<OldestTimeStamp.sec<<" s; nsec="<<OldestTimeStamp.nsec<<" ns"<<std::endl;

#if _DEBUG_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationROS::bufferManagerThreadFunction() Going to get next time stamp"<<std::endl;
            this->log(logString.str());
        }
#endif

        if(!this->TheMsfStorageCore->getNextTimeStamp(OldestTimeStamp, TheNewOutdatedTimeStamp))
        {
#if _DEBUG_MSF_LOCALIZATION_CORE
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationROS::bufferManagerThreadFunction() Adding to be processed TS: sec="<<TheNewOutdatedTimeStamp.sec<<" s; nsec="<<TheNewOutdatedTimeStamp.nsec<<" ns"<<std::endl;
                this->log(logString.str());
            }
#endif

            // Set the following element of the buffer as outdated
            this->TheMsfStorageCore->addOutdatedElement(TheNewOutdatedTimeStamp);


        }
        else
        {
#if _DEBUG_MSF_LOCALIZATION_CORE
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationROS::bufferManagerThreadFunction() Nothing new to be added"<<std::endl;
                this->log(logString.str());
            }
#endif
        }

#if _DEBUG_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationROS::bufferManagerThreadFunction() Going to purge the ring"<<std::endl;
            this->log(logString.str());
        }
#endif

        // Purge the buffer ??
        this->TheMsfStorageCore->purgeRingBuffer(50);


        // Display the buffer
        //this->TheMsfStorageCore->displayRingBuffer();


        // Purge the buffer
        //this->TheMsfStorageCore->purgeRingBuffer(20);


        // Sleep
        //bufferManagerRate.sleep();



#if _DEBUG_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationROS::bufferManagerThreadFunction() loop end"<<std::endl;
            this->log(logString.str());
        }
#endif




    }
//    }
//    catch(std::exception &ex)
//    {
//        std::cout<<"[ROSNODE] Exception :"<<ex.what()<<std::endl;
//    }
//    catch(...)
//    {
//        std::cout<<"EXCEPTION ON UPDATE Thread"<<std::endl;
//    }

#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationROS::bufferManagerThreadFunction() ended"<<std::endl;
        this->log(logString.str());
    }
#endif

    return 0;

}


int MsfLocalizationROS::run()
{
    // Core threads
    startThreads();

    // Start ROS threads
    robotPoseThread=new std::thread(&MsfLocalizationROS::robotPoseThreadFunction, this);

    // TODO ROS-Service
    // Enable prediction
    //setPredictEnabled(true);

    // Loop to get measurements
    try
    {
        ros::spin();
    }
    catch(std::exception &ex)
    {
        std::cout<<"[ROSNODE] Exception :"<<ex.what()<<std::endl;
    }
    catch(...)
    {
        std::cout<<"EXCEPTION ON main thread"<<std::endl;
    }

    return 0;
}
