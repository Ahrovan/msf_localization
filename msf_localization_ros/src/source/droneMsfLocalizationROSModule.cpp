
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
    predictRateValue=50.0; // In Hz



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

        // Read configs
        if(readFreeModelRobotConfig(robot, this->TheMsfStorageCore, TheRobotCoreAux, RobotInitStateCore))
            return -2;

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

            // Read configs
            if(readImuConfig(sensor, firstAvailableSensorId, this->TheMsfStorageCore, TheRosSensorImuInterface, SensorInitStateCore))
                return -2;

            // Update id
            firstAvailableSensorId++;


            // Finish

            // Push to the list of sensors
            this->TheListOfSensorCore.push_back(TheRosSensorImuInterface);

            // Push the init state of the sensor
            InitialState->TheListSensorStateCore.push_back(SensorInitStateCore);
        }

        //// Aruco Eye Sensor Type
        if(sensorType=="aruco_eye")
        {
            // Create a class for the SensoreCore
            std::shared_ptr<RosArucoEyeInterface> TheRosSensorInterface;
            // Create a class for the SensorStateCore
            std::shared_ptr<CodedVisualMarkerEyeStateCore> TheSensorStateCore;

            // Read configs
            if(readArucoEyeConfig(sensor, firstAvailableSensorId, this->TheMsfStorageCore, TheRosSensorInterface, TheSensorStateCore))
                return -2;

            // Update id
            firstAvailableSensorId++;


            // Finish

            // Push to the list of sensors
            this->TheListOfSensorCore.push_back(TheRosSensorInterface);

            // Push the init state of the sensor
            InitialState->TheListSensorStateCore.push_back(TheSensorStateCore);

        }


    }



    ///// Map elements


    // Reading map elements
    for(pugi::xml_node map_element = msf_localization.child("map_element"); map_element; map_element = map_element.next_sibling("map_element"))
    {
        std::cout<<"map_element; "<<std::endl;

        // Sensor Type
        std::string mapElementType=map_element.child_value("type");


        // Coded visual marker
        if(mapElementType=="coded_visual_marker")
        {
            // Create a class for the SensoreCore
            std::shared_ptr<CodedVisualMarkerLandmarkCore> TheMapElementCore;
            // Create a class for the SensorStateCore
            std::shared_ptr<CodedVisualMarkerLandmarkStateCore> MapElementInitStateCore;

            // Read configs
            if(readCodedVisualMarkerConfig(map_element, TheMsfStorageCore, TheMapElementCore, MapElementInitStateCore))
                return -2;


            // Finish

            // Push to the list of sensors
            this->TheListOfMapElementCore.push_back(TheMapElementCore);

            // Push the init state of the sensor
            InitialState->TheListMapElementStateCore.push_back(MapElementInitStateCore);
        }


    }


    //// Finish

    // Initial State Prepare
    InitialState->prepareInitErrorStateVariance();



    // Display
#if 0 && _DEBUG_MSF_LOCALIZATION_CORE
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
    readingValue=robot.child("position").child_value("noise");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        TheRobotCoreAux->setNoisePosition(variance.asDiagonal());
    }

    // Linear velocity
    readingValue=robot.child("lin_speed").child_value("noise");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        TheRobotCoreAux->setNoiseLinearSpeed(variance.asDiagonal());
    }

    // Linear acceleration
    readingValue=robot.child("lin_accel").child_value("noise");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        TheRobotCoreAux->setNoiseLinearAcceleration(variance.asDiagonal());
    }

    // Attitude
    readingValue=robot.child("attitude").child_value("noise");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        TheRobotCoreAux->setNoiseAttitude(variance.asDiagonal());
    }

    // Angular velocity
    readingValue=robot.child("ang_velocity").child_value("noise");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        TheRobotCoreAux->setNoiseAngularVelocity(variance.asDiagonal());
    }

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


int MsfLocalizationROS::readArucoEyeConfig(pugi::xml_node sensor, unsigned int sensorId, std::shared_ptr<MsfStorageCore> TheMsfStorageCore, std::shared_ptr<RosArucoEyeInterface>& TheRosArucoEyeInterface, std::shared_ptr<CodedVisualMarkerEyeStateCore>& SensorInitStateCore)
{
    // Create a class for the SensoreCore
    if(!TheRosArucoEyeInterface)
        TheRosArucoEyeInterface=std::make_shared<RosArucoEyeInterface>(nh);

    // Set pointer to the SensorCore
    TheRosArucoEyeInterface->setTheSensorCore(TheRosArucoEyeInterface);

    // Create a class for the SensorStateCore
    if(!SensorInitStateCore)
        SensorInitStateCore=std::make_shared<CodedVisualMarkerEyeStateCore>();

    // Set pointer to the SensorCore
    SensorInitStateCore->setTheSensorCore(TheRosArucoEyeInterface);


    // Set sensor type
    TheRosArucoEyeInterface->setSensorType(SensorTypes::coded_visual_marker_eye);

    // Set Id
    TheRosArucoEyeInterface->setSensorId(sensorId);

    // Set the access to the Storage core
    //TheRosSensorImuInterface->setTheMsfStorageCore(std::make_shared<MsfStorageCore>(this->TheStateEstimationCore));
    TheRosArucoEyeInterface->setTheMsfStorageCore(TheMsfStorageCore);


    // Sensor Topic
    std::string sensorTopic=sensor.child_value("ros_topic");
    TheRosArucoEyeInterface->setMarkerListTopicName(sensorTopic);


    // Auxiliar reading value
    std::string readingValue;


    // Name
    readingValue=sensor.child_value("name");
    TheRosArucoEyeInterface->setSensorName(readingValue);


    //// Sensor configurations


    /// Pose of the sensor wrt robot
    pugi::xml_node pose_in_robot=sensor.child("pose_in_robot");

    // Position of the sensor wrt robot
    readingValue=pose_in_robot.child("position").child_value("enabled");
    if(std::stoi(readingValue))
        TheRosArucoEyeInterface->enableEstimationPositionSensorWrtRobot();

    // Attitude of the sensor wrt robot
    readingValue=pose_in_robot.child("attitude").child_value("enabled");
    if(std::stoi(readingValue))
        TheRosArucoEyeInterface->enableEstimationAttitudeSensorWrtRobot();


    /// Other Parameters
    pugi::xml_node parameters = sensor.child("parameters");

    // None



    //// Measurements
    pugi::xml_node measurements = sensor.child("measurements");

    /// Orientation
    pugi::xml_node meas_orientation = measurements.child("orientation");

    readingValue=meas_orientation.child_value("enabled");
    if(std::stoi(readingValue))
        TheRosArucoEyeInterface->enableMeasurementAttitude();

    readingValue=meas_orientation.child_value("var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        TheRosArucoEyeInterface->setNoiseMeasurementAttitude(variance.asDiagonal());
    }


    /// Position
    pugi::xml_node meas_position = measurements.child("position");

    readingValue=meas_position.child_value("enabled");
    if(std::stoi(readingValue))
        TheRosArucoEyeInterface->enableMeasurementPosition();

    readingValue=meas_position.child_value("var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        TheRosArucoEyeInterface->setNoiseMeasurementPosition(variance.asDiagonal());
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
        TheRosArucoEyeInterface->setNoisePositionSensorWrtRobot(variance.asDiagonal());
    }


    // Attitude of the sensor wrt robot
    readingValue=pose_in_robot.child("attitude").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        TheRosArucoEyeInterface->setNoiseAttitudeSensorWrtRobot(variance.asDiagonal());
    }



    /// Other Parameters

    // None


    // Noises in the estimation (if enabled)

    // None


    // Prepare covariance matrix
    TheRosArucoEyeInterface->prepareInitErrorStateVariance();


    /// Finish

    // Open
    TheRosArucoEyeInterface->open();

    // End
    return 0;
}


int MsfLocalizationROS::readCodedVisualMarkerConfig(pugi::xml_node map_element, std::shared_ptr<MsfStorageCore> TheMsfStorageCore, std::shared_ptr<CodedVisualMarkerLandmarkCore>& TheMapElementCore, std::shared_ptr<CodedVisualMarkerLandmarkStateCore>& MapElementInitStateCore)
{
    // Create a class for the SensoreCore
    if(!TheMapElementCore)
        TheMapElementCore=std::make_shared<CodedVisualMarkerLandmarkCore>();

    // Set pointer to the SensorCore
    TheMapElementCore->setTheMapElementCore(TheMapElementCore);

    // Create a class for the SensorStateCore
    if(!MapElementInitStateCore)
        MapElementInitStateCore=std::make_shared<CodedVisualMarkerLandmarkStateCore>();

    // Set pointer to the SensorCore
    MapElementInitStateCore->setTheMapElementCore(TheMapElementCore);


    // Set the access to the Storage core
    //TheRosSensorImuInterface->setTheMsfStorageCore(std::make_shared<MsfStorageCore>(this->TheStateEstimationCore));
    TheMapElementCore->setTheMsfStorageCore(TheMsfStorageCore);


    // Visual landmark
    pugi::xml_node visual_landmark=map_element.child("visual_landmark");


    // Auxiliar reading value
    std::string readingValue;


    /// id
    std::string idString=visual_landmark.child_value("id");
    TheMapElementCore->setId(std::stoi(idString));


    /// Name
    TheMapElementCore->setMapElementName("visual_marker_"+idString);


    //// Configs

    /// Pose of the map element wrt worl
    pugi::xml_node pose_in_world=visual_landmark.child("pose_in_world");

    // Position of the sensor wrt robot
    readingValue=pose_in_world.child("position").child_value("enabled");
    if(std::stoi(readingValue))
        TheMapElementCore->enableEstimationPositionVisualMarkerWrtWorld();

    // Attitude of the sensor wrt robot
    readingValue=pose_in_world.child("attitude").child_value("enabled");
    if(std::stoi(readingValue))
        TheMapElementCore->enableEstimationAttitudeVisualMarkerWrtWorld();



    //// Init State

    /// Pose of the visual marker wrt world

    // Position of the visual marker wrt world
    readingValue=pose_in_world.child("position").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d init_estimation;
        stm>>init_estimation[0]>>init_estimation[1]>>init_estimation[2];
        MapElementInitStateCore->setPosition(init_estimation);
    }

    // Attitude of the visual marker wrt world
    readingValue=pose_in_world.child("attitude").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector4d init_estimation;
        stm>>init_estimation[0]>>init_estimation[1]>>init_estimation[2]>>init_estimation[3];
        MapElementInitStateCore->setAttitude(init_estimation);
    }


    /// Parameters

    // None


    //// Init Variances


    /// Pose of the visual marker wrt world

    // Position of the visual marker wrt world
    readingValue=pose_in_world.child("position").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        TheMapElementCore->setCovariancePositionVisualMarkerWrtWorld(variance.asDiagonal());
    }


    // Attitude of the visual marker wrt world
    readingValue=pose_in_world.child("attitude").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        TheMapElementCore->setCovarianceAttitudeVisualMarkerWrtWorld(variance.asDiagonal());
    }



    /// Other Parameters

    // None


    // Noises in the estimation (if enabled)

    // None


    /// Finish


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
    ros::param::param<std::string>("~robot_pose_with_cov_topic_name", robotPoseWithCovarianceStampedTopicName, "msf_localization/robot_pose_cov");
    std::cout<<"robot_pose_with_cov_topic_name="<<robotPoseWithCovarianceStampedTopicName<<std::endl;
    //
    ros::param::param<std::string>("~robot_pose_topic_name", robotPoseStampedTopicName, "msf_localization/robot_pose");
    std::cout<<"robot_pose_topic_name="<<robotPoseStampedTopicName<<std::endl;
    //
    ros::param::param<std::string>("~robot_linear_speed_topic_name", robotLinearSpeedStampedTopicName, "msf_localization/robot_linear_speed");
    std::cout<<"robot_linear_speed_topic_name="<<robotLinearSpeedStampedTopicName<<std::endl;
    //
    ros::param::param<std::string>("~robot_linear_acceleration_topic_name", robotLinearAccelerationStampedTopicName, "msf_localization/robot_linear_acceleration");
    std::cout<<"robot_linear_acceleration_topic_name="<<robotLinearAccelerationStampedTopicName<<std::endl;
    //
    ros::param::param<std::string>("~robot_angular_velocity_topic_name", robotAngularVelocityStampedTopicName, "msf_localization/robot_angular_velocity");
    std::cout<<"robot_angular_velocity_topic_name="<<robotAngularVelocityStampedTopicName<<std::endl;
    //
    ros::param::param<std::string>("~robot_angular_acceleration_topic_name", robotAngularAccelerationStampedTopicName, "msf_localization/robot_angular_acceleration");
    std::cout<<"robot_angular_acceleration_topic_name="<<robotAngularAccelerationStampedTopicName<<std::endl;

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


    // Robot Pose With Covariance Publisher
    robotPoseWithCovarianceStampedPub = nh->advertise<geometry_msgs::PoseWithCovarianceStamped>(robotPoseWithCovarianceStampedTopicName, 1, true);
    //
    robotPoseStampedPub = nh->advertise<geometry_msgs::PoseStamped>(robotPoseStampedTopicName, 1, true);
    //
    robotLinearSpeedStampedPub = nh->advertise<geometry_msgs::Vector3Stamped>(robotLinearSpeedStampedTopicName, 1, true);
    //
    robotLinearAccelerationStampedPub = nh->advertise<geometry_msgs::Vector3Stamped>(robotLinearAccelerationStampedTopicName, 1, true);
    //
    robotAngularVelocityStampedPub = nh->advertise<geometry_msgs::Vector3Stamped>(robotAngularVelocityStampedTopicName, 1, true);
    //
    robotAngularAccelerationStampedPub = nh->advertise<geometry_msgs::Vector3Stamped>(robotAngularAccelerationStampedTopicName, 1, true);


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

#if 1 || _DEBUG_MSF_LOCALIZATION_CORE
                {
                    std::ostringstream logString;
                    logString<<"MsfLocalizationROS::robotPoseThreadFunction() predicting TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
                    this->log(logString.str());
                }
#endif

                // TODO this should be a while and being carefully with the memory
                if(this->predictNoAddBuffer(TheTimeStamp, PredictedState))
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


        // Error with predicted state
        if(!PredictedState)
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



        // Getters

        std::shared_ptr<FreeModelRobotStateCore> TheRobotStateCore=std::static_pointer_cast<FreeModelRobotStateCore>(PredictedState->TheRobotStateCore);

        Eigen::Vector3d robotPosition=TheRobotStateCore->getPosition();
        Eigen::Vector4d robotAttitude=TheRobotStateCore->getAttitude();
        Eigen::Vector3d robotLinearSpeed=TheRobotStateCore->getLinearSpeed();
        Eigen::Vector3d robotLinearAcceleration=TheRobotStateCore->getLinearAcceleration();
        Eigen::Vector3d robotAngularVelocity=TheRobotStateCore->getAngularVelocity();
        Eigen::Vector3d robotAngularAcceleration=TheRobotStateCore->getAngularAcceleration();



        // Fill msg


        // Header

        // ROBOT POSE
        // Stamp
        robotPoseWithCovarianceStampedMsg.header.stamp=ros::Time(TheTimeStamp.sec, TheTimeStamp.nsec);
        // Frame id
        robotPoseWithCovarianceStampedMsg.header.frame_id=this->TheGlobalParametersCore->getWorldName();

        //
        robotPoseStampedMsg.header.stamp=ros::Time(TheTimeStamp.sec, TheTimeStamp.nsec);
        // Frame id
        robotPoseStampedMsg.header.frame_id=this->TheGlobalParametersCore->getWorldName();

        //
        robotLinearSpeedStampedMsg.header.stamp=ros::Time(TheTimeStamp.sec, TheTimeStamp.nsec);
        // Frame id
        robotLinearSpeedStampedMsg.header.frame_id=this->TheGlobalParametersCore->getWorldName();

        //
        robotLinearAccelerationStampedMsg.header.stamp=ros::Time(TheTimeStamp.sec, TheTimeStamp.nsec);
        // Frame id
        robotLinearAccelerationStampedMsg.header.frame_id=this->TheGlobalParametersCore->getWorldName();

        //
        robotAngularVelocityStampedMsg.header.stamp=ros::Time(TheTimeStamp.sec, TheTimeStamp.nsec);
        // Frame id
        robotAngularVelocityStampedMsg.header.frame_id=this->TheGlobalParametersCore->getWorldName();

        //
        robotAngularAccelerationStampedMsg.header.stamp=ros::Time(TheTimeStamp.sec, TheTimeStamp.nsec);
        // Frame id
        robotAngularAccelerationStampedMsg.header.frame_id=this->TheGlobalParametersCore->getWorldName();





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
        Eigen::MatrixXd robotPoseCovariance(6,6);
        robotPoseCovariance.setZero();
        robotPoseCovariance.block<3,3>(0,0)=PredictedState->covarianceMatrix.block<3,3>(0,0);
        robotPoseCovariance.block<3,3>(3,3)=PredictedState->covarianceMatrix.block<3,3>(9,9);
        double robotPoseCovarianceArray[36];
        Eigen::Map<Eigen::MatrixXd>(robotPoseCovarianceArray, 6, 6) = robotPoseCovariance;
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


       //
       robotAngularAccelerationStampedMsg.vector.x=robotAngularAcceleration[0];
       robotAngularAccelerationStampedMsg.vector.y=robotAngularAcceleration[1];
       robotAngularAccelerationStampedMsg.vector.z=robotAngularAcceleration[2];


        // TF ROBOT
        tf::Quaternion tf_rot(robotAttitude[1], robotAttitude[2], robotAttitude[3], robotAttitude[0]);
        tf::Vector3 tf_tran(robotPosition[0], robotPosition[1], robotPosition[2]);

        tf::Transform transform(tf_rot, tf_tran);

        tfTransformBroadcaster->sendTransform(tf::StampedTransform(transform, ros::Time(TheTimeStamp.sec, TheTimeStamp.nsec),
                                              this->TheGlobalParametersCore->getWorldName(), this->TheRobotCore->getRobotName()));


        // TF SENSORS
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


        // TF Map elements
        for(std::list< std::shared_ptr<MapElementStateCore> >::const_iterator itMapElementState=PredictedState->TheListMapElementStateCore.begin();
            itMapElementState!=PredictedState->TheListMapElementStateCore.end();
            ++itMapElementState)
        {

            switch((*itMapElementState)->getTheMapElementCore()->getMapElementType())
            {
                case MapElementTypes::coded_visual_marker:
                {
                    // Cast
                    std::shared_ptr<CodedVisualMarkerLandmarkStateCore> theCodedVisualMarkersLandamarkState=std::dynamic_pointer_cast<CodedVisualMarkerLandmarkStateCore>(*itMapElementState);

                    Eigen::Vector3d mapElementPosition=theCodedVisualMarkersLandamarkState->getPosition();
                    Eigen::Vector4d mapElementAttitude=theCodedVisualMarkersLandamarkState->getAttitude();


#if 1 || _DEBUG_MSF_LOCALIZATION_CORE
                    {
                        std::ostringstream logString;
                        logString<<"MsfLocalizationROS::robotPoseThreadFunction()"<<std::endl;

                        logString<<"Visual Marker id="<<std::dynamic_pointer_cast<CodedVisualMarkerLandmarkCore>(theCodedVisualMarkersLandamarkState->getTheMapElementCore())->getId()<<std::endl;
                        logString<<"  - Attitude: "<<mapElementAttitude.transpose()<<std::endl;


                        this->log(logString.str());
                    }
#endif



                    tf::Quaternion tf_rot(mapElementAttitude[1], mapElementAttitude[2], mapElementAttitude[3], mapElementAttitude[0]);
                    tf::Vector3 tf_tran(mapElementPosition[0], mapElementPosition[1], mapElementPosition[2]);

                    tf::Transform transform(tf_rot, tf_tran);


                    tfTransformBroadcaster->sendTransform(tf::StampedTransform(transform, ros::Time(TheTimeStamp.sec, TheTimeStamp.nsec),
                                                          this->TheGlobalParametersCore->getWorldName(), (*itMapElementState)->getTheMapElementCore()->getMapElementName()));

                }
            }
        }



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

        if(robotAngularAccelerationStampedPub.getNumSubscribers()>0)
            robotAngularAccelerationStampedPub.publish(robotAngularAccelerationStampedMsg);



        // Free the ownership
        if(PredictedState)
            PredictedState.reset();


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
        TimeStamp TheTimeStamp=getTimeStamp();
        std::shared_ptr<StateEstimationCore> ThePredictedState;

        // Predict. Typically it will not be any
        //std::cout<<"Calling predict TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
        this->TheMsfStorageCore->getElement(TheTimeStamp, ThePredictedState);

        if(!ThePredictedState)
        {
//            predictThreadState.setProcessing(TheTimeStamp);
            // TODO Fix. This should be a while but being careful with the memory


#if 1 || _DEBUG_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationROS::predictThreadFunction() predicting TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
            this->log(logString.str());
        }
#endif

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

#if 1 || _DEBUG_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationROS::bufferManagerThreadFunction() updating TS: sec="<<OldestTimeStamp.sec<<" s; nsec="<<OldestTimeStamp.nsec<<" ns"<<std::endl;
            this->log(logString.str());
        }
#endif


#if 1 || _DEBUG_MSF_LOCALIZATION_CORE
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

#if _DEBUG_TIME_MSF_LOCALIZATION_ROS
        {
            ros::Time begin = ros::Time::now();
#endif

            int errorPredict=this->predict(OldestTimeStamp);

        if(errorPredict)
        {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE

            {
                std::ostringstream logString;
                logString<<"MsfLocalizationROS::bufferManagerThreadFunction() error 5!"<<std::endl;
                this->log(logString.str());
            }
#endif

            // Add to the processing list -> No
            //this->TheMsfStorageCore->addOutdatedElement(OldestTimeStamp);

            // Delete from buffer to avoid using it! -> No
            //this->TheMsfStorageCore->purgeElementRingBuffer(OldestTimeStamp);

            continue;
        }

#if _DEBUG_TIME_MSF_LOCALIZATION_ROS
            std::ostringstream logString;
            logString<<"MsfLocalizationROS::bufferManagerThreadFunction -> predict() time: "<<(ros::Time::now()-begin).toNSec()<<" ns"<<std::endl;
            this->log(logString.str());

        }
#endif

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

#if _DEBUG_TIME_MSF_LOCALIZATION_ROS
        {
            ros::Time begin = ros::Time::now();
#endif

            errorUpdate=this->update(OldestTimeStamp);
//        }
//        catch(...)
//        {
//            std::cout<<"EXCEPTION ON UPDATE"<<std::endl;
//        }

#if _DEBUG_TIME_MSF_LOCALIZATION_ROS
            std::ostringstream logString;
            logString<<"MsfLocalizationROS::bufferManagerThreadFunction -> update() time: "<<(ros::Time::now()-begin).toNSec()<<" ns"<<std::endl;
            this->log(logString.str());
            }
#endif


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
                logString<<"MsfLocalizationROS::bufferManagerThreadFunction() error in the update: "<<errorUpdate<<std::endl;
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
