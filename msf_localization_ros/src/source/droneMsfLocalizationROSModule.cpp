
#include "droneMsfLocalizationROSModule.h"

MsfLocalizationROS::MsfLocalizationROS(int argc,char **argv)
{
    //Ros Init
    ros::init(argc, argv, ros::this_node::getName());
    ros::NodeHandle nh;

    // Init
    init();

    return;
}

MsfLocalizationROS::~MsfLocalizationROS()
{
    // Close
    close();

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
    ros::NodeHandle nh;

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
    predictRateVale=100.0; // In Hz




    ///// Robot

    {
        // Reading Robot
        pugi::xml_node robot = msf_localization.child("robot");

        // Create the RobotCoreAux
        std::shared_ptr<FreeModelRobotCore> TheRobotCoreAux;
        // Create a class for the RobotStateCore
        std::shared_ptr<FreeModelRobotStateCore> RobotInitStateCore;
        // Robot Init State Covariance Matrix
        Eigen::MatrixXd InitStateCovatrianceMatrix;

        // Read configs
        if(readFreeModelRobotConfig(robot, this->TheMsfStorageCore, TheRobotCoreAux, RobotInitStateCore, InitStateCovatrianceMatrix))
            return -2;


        // Update covariance
        unsigned int previousNumCols=InitialState->covarianceMatrix.cols();
        unsigned int previousNumRows=InitialState->covarianceMatrix.rows();

        InitialState->covarianceMatrix.conservativeResize(previousNumRows+InitStateCovatrianceMatrix.rows(), previousNumCols+InitStateCovatrianceMatrix.cols());
        InitialState->covarianceMatrix.block(previousNumRows, previousNumCols, InitStateCovatrianceMatrix.rows(), InitStateCovatrianceMatrix.cols())=InitStateCovatrianceMatrix;


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
            Eigen::MatrixXd InitStateCovatrianceMatrix;

            // Read configs
            if(readImuConfig(sensor, firstAvailableId, this->TheMsfStorageCore, TheRosSensorImuInterface, SensorInitStateCore, InitStateCovatrianceMatrix))
                return -2;

            // Update id
            firstAvailableId++;

            // Update covariance
            unsigned int previousNumCols=InitialState->covarianceMatrix.cols();
            unsigned int previousNumRows=InitialState->covarianceMatrix.rows();

            InitialState->covarianceMatrix.conservativeResize(previousNumRows+InitStateCovatrianceMatrix.rows(), previousNumCols+InitStateCovatrianceMatrix.cols());
            InitialState->covarianceMatrix.block(previousNumRows, previousNumCols, InitStateCovatrianceMatrix.rows(), InitStateCovatrianceMatrix.cols())=InitStateCovatrianceMatrix;


            // Finish

            // Push to the list of sensors
            this->TheListOfSensorCore.push_back(TheRosSensorImuInterface);

            // Push the init state of the sensor
            InitialState->TheListSensorStateCore.push_back(SensorInitStateCore);
        }


    }



    // Display
    logFile<<"Covariance Matrix"<<std::endl;
    logFile<<InitialState->covarianceMatrix<<std::endl;




    // Add init state to the buffer
    TheMsfStorageCore->addElement(TimeStamp(0,0), InitialState);


    return 0;
}



int MsfLocalizationROS::readFreeModelRobotConfig(pugi::xml_node robot, std::shared_ptr<MsfStorageCore> TheMsfStorageCore, std::shared_ptr<FreeModelRobotCore>& TheRobotCoreAux, std::shared_ptr<FreeModelRobotStateCore>& RobotInitStateCore, Eigen::MatrixXd& InitStateCovarianceMatrix)
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


    // Dimension
    unsigned int dimensionErrorState=TheRobotCoreAux->getDimensionErrorState();


    // Aux vars
    std::string readingValue;


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


    /// Init Variances

    InitStateCovarianceMatrix.resize(dimensionErrorState, dimensionErrorState);
    InitStateCovarianceMatrix.setZero();

    // Position
    readingValue=robot.child("position").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        InitStateCovarianceMatrix.block<3,3>(0,0)=variance.asDiagonal();
    }

    // Linear Speed
    readingValue=robot.child("lin_speed").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        InitStateCovarianceMatrix.block<3,3>(3,3)=variance.asDiagonal();
    }

    // Linear acceleration
    readingValue=robot.child("lin_accel").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        InitStateCovarianceMatrix.block<3,3>(6,6)=variance.asDiagonal();
    }

    // Attitude
    readingValue=robot.child("attitude").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        InitStateCovarianceMatrix.block<3,3>(9,9)=variance.asDiagonal();
    }

    // Angular velocity
    readingValue=robot.child("ang_velocity").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        InitStateCovarianceMatrix.block<3,3>(12,12)=variance.asDiagonal();
    }


    // Noises

    // Linear acceleration
    readingValue=robot.child("lin_accel").child_value("noise");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        TheRobotCoreAux->setNoiseLinearAcceleration(variance.asDiagonal());
    }

    // Angular velocity
    readingValue=robot.child("ang_velocity").child_value("noise");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        TheRobotCoreAux->setNoiseAngularVelocity(variance.asDiagonal());
    }


    // End
    return 0;
}


int MsfLocalizationROS::readImuConfig(pugi::xml_node sensor, unsigned int sensorId, std::shared_ptr<MsfStorageCore> TheMsfStorageCore, std::shared_ptr<RosSensorImuInterface>& TheRosSensorImuInterface, std::shared_ptr<ImuSensorStateCore>& SensorInitStateCore, Eigen::MatrixXd& InitStateCovarianceMatrix)
{
    // Create a class for the SensoreCore
    if(!TheRosSensorImuInterface)
        TheRosSensorImuInterface=std::make_shared<RosSensorImuInterface>();

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

    // Pose of the sensor wrt robot
    pugi::xml_node pose_in_robot=sensor.child("pose_in_robot");

    // Position of the sensor wrt robot
    readingValue=pose_in_robot.child("position").child_value("enabled");
    if(std::stoi(readingValue))
        TheRosSensorImuInterface->enableEstimationPositionSensorWrtRobot();

    // Attitude of the sensor wrt robot
    readingValue=pose_in_robot.child("attitude").child_value("enabled");
    if(std::stoi(readingValue))
        TheRosSensorImuInterface->enableEstimationAttitudeSensorWrtRobot();


    // Parameters
    pugi::xml_node parameters = sensor.child("parameters");

    // Angular Velocity
    pugi::xml_node param_angular_velocity = parameters.child("angular_velocity");

    readingValue=param_angular_velocity.child("biases").child_value("enabled");
    if(std::stoi(readingValue))
    {
        TheRosSensorImuInterface->enableEstimationBiasAngularVelocity();
    }

    // Linear Acceleration
    pugi::xml_node param_linear_acceleration = parameters.child("linear_acceleration");

    readingValue=param_linear_acceleration.child("biases").child_value("enabled");
    if(std::stoi(readingValue))
        TheRosSensorImuInterface->enableEstimationBiasLinearAcceleration();


    // Finish
    unsigned int dimensionErrorState=TheRosSensorImuInterface->getDimensionErrorState();






    /// Measurements
    pugi::xml_node measurements = sensor.child("measurements");

    // Orientation
    pugi::xml_node orientation = measurements.child("orientation");

    readingValue=orientation.child_value("enabled");
    // TODO

    readingValue=orientation.child_value("var");
    // TODO


    // Angular Velocity
    pugi::xml_node meas_angular_velocity = measurements.child("angular_velocity");

    readingValue=meas_angular_velocity.child_value("enabled");
    if(std::stoi(readingValue))
        TheRosSensorImuInterface->enableAngularVelocity();

    readingValue=meas_angular_velocity.child_value("var");
    // TODO



    // Linear Acceleration
    pugi::xml_node meas_linear_acceleration = measurements.child("linear_acceleration");

    readingValue=meas_linear_acceleration.child_value("enabled");
    if(std::stoi(readingValue))
        TheRosSensorImuInterface->enableLinearAcceleration();

    readingValue=meas_linear_acceleration.child_value("var");
    // TODO


    //// Init State

    /// Pose of the sensor wrt robot

    // Position of the sensor wrt robot
    readingValue=pose_in_robot.child("position").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d init_estimation;
        stm>>init_estimation[0]>>init_estimation[1]>>init_estimation[2];
        SensorInitStateCore->setPositionSensorWrtRobot(init_estimation); //TODO
    }

    // Attitude of the sensor wrt robot
    readingValue=pose_in_robot.child("attitude").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector4d init_estimation;
        stm>>init_estimation[0]>>init_estimation[1]>>init_estimation[2]>>init_estimation[3];
        SensorInitStateCore->setAttitudeSensorWrtRobot(init_estimation); //TODO
    }


    /// Parameters

    // Bias Angular Velocity
    readingValue=param_angular_velocity.child("biases").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d biasesAngularVelocity;
        stm>>biasesAngularVelocity[0]>>biasesAngularVelocity[1]>>biasesAngularVelocity[2];
        SensorInitStateCore->setBiasesAngularVelocity(biasesAngularVelocity);
    }

    // Bias Linear Acceleration
    readingValue=param_linear_acceleration.child("biases").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d biasesLinearAcceleration;
        stm>>biasesLinearAcceleration[0]>>biasesLinearAcceleration[1]>>biasesLinearAcceleration[2];
        SensorInitStateCore->setBiasesLinearAcceleration(biasesLinearAcceleration);
    }




    //// Init Variances

    // Resize Matrix
    InitStateCovarianceMatrix.resize(dimensionErrorState, dimensionErrorState);
    InitStateCovarianceMatrix.setZero();
    //InitialState.covarianceMatrix.conservativeResize(dimensionErrorState, dimensionErrorState);

    unsigned int dimensionErrorStateAdditional=0;

    /// Pose of the sensor wrt robot

    // Position of the sensor wrt robot
    if(TheRosSensorImuInterface->isEstimationPositionSensorWrtRobotEnabled())
    {
        readingValue=pose_in_robot.child("position").child_value("init_var");
        {
            std::istringstream stm(readingValue);
            Eigen::Vector3d variance;
            stm>>variance[0]>>variance[1]>>variance[2];
            InitStateCovarianceMatrix.block<3,3>(dimensionErrorStateAdditional,dimensionErrorStateAdditional)=variance.asDiagonal();
        }
        dimensionErrorStateAdditional+=3;
    }

    // Attitude of the sensor wrt robot
    if(TheRosSensorImuInterface->isEstimationAttitudeSensorWrtRobotEnabled())
    {
        readingValue=pose_in_robot.child("attitude").child_value("init_var");
        {
            std::istringstream stm(readingValue);
            Eigen::Vector3d variance;
            stm>>variance[0]>>variance[1]>>variance[2];
            InitStateCovarianceMatrix.block<3,3>(dimensionErrorStateAdditional,dimensionErrorStateAdditional)=variance.asDiagonal();
        }
        dimensionErrorStateAdditional+=3;
    }



    /// Parameters

    // Bias Linear Acceleration
    if(TheRosSensorImuInterface->isEstimationBiasLinearAccelerationEnabled())
    {
        readingValue=param_linear_acceleration.child("biases").child_value("init_var");
        {
            std::istringstream stm(readingValue);
            Eigen::Vector3d variance;
            stm>>variance[0]>>variance[1]>>variance[2];
            InitStateCovarianceMatrix.block<3,3>(dimensionErrorStateAdditional,dimensionErrorStateAdditional)=variance.asDiagonal();
        }
        dimensionErrorStateAdditional+=3;
    }

    // Bias Angular Velocity
    if(TheRosSensorImuInterface->isEstimationBiasAngularVelocityEnabled())
    {
        readingValue=param_angular_velocity.child("biases").child_value("init_var");
        {
            std::istringstream stm(readingValue);
            Eigen::Vector3d variance;
            stm>>variance[0]>>variance[1]>>variance[2];
            InitStateCovarianceMatrix.block<3,3>(dimensionErrorStateAdditional,dimensionErrorStateAdditional)=variance.asDiagonal();
        }
        dimensionErrorStateAdditional+=3;
    }


    // Noises

    // Bias Linear Acceleration
    if(TheRosSensorImuInterface->isEstimationBiasLinearAccelerationEnabled())
    {
        readingValue=param_linear_acceleration.child("biases").child_value("noise");
        {
            std::istringstream stm(readingValue);
            Eigen::Vector3d variance;
            stm>>variance[0]>>variance[1]>>variance[2];
            TheRosSensorImuInterface->setNoiseBiasLinearAcceleration(variance.asDiagonal());
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
            TheRosSensorImuInterface->setNoiseBiasAngularVelocity(variance.asDiagonal());
        }
    }



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
    ros::NodeHandle nh;

    // Read parameters
    if(readParameters())
        ROS_ERROR("Error Reading Parameters");

    // Read config file
    if(readConfigFile())
        ROS_ERROR("Error Reading Config File");


    // Robot Pose Publisher
    robotPosePub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(robotPoseTopicName, 1, true);


    // Service
    setStateEstimationEnabledSrv = nh.advertiseService(setStateEstimationEnabledServiceName, &MsfLocalizationROS::setStateEstimationEnabledCallback, this);


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

    {
        std::ostringstream logString;
        logString<<"MsfLocalizationROS::robotPoseThreadFunction()"<<std::endl;
        this->log(logString.str());
    }

//return 0;
    robotPoseRate=new ros::Rate(robotPoseRateVal);

    while(ros::ok())
    {
        if(!this->isStateEstimationEnabled())
        {
            // Sleep
            robotPoseRate->sleep();
            // continue
            continue;
        }

        // Get Robot Pose
        // TODO
        TimeStamp TheTimeStamp=getTimeStamp();
        std::shared_ptr<StateEstimationCore> PredictedState;

        this->TheMsfStorageCore->getElement(TheTimeStamp, PredictedState);

        while(!PredictedState)
        {
//            this->robotPoseThreadState.setProcessing(TheTimeStamp);
            this->predict(TheTimeStamp, PredictedState);
//            this->robotPoseThreadState.setNotProcessing();
        }


//        if()
//        {
//            std::cout<<"error getting robot pose"<<std::endl;
//            continue;
//        }

        // Fill msg
        // Header
        robotPoseMsg.header.stamp=ros::Time(TheTimeStamp.sec, TheTimeStamp.nsec);

        // Frame id
        // TODO put as a ros param
        robotPoseMsg.header.frame_id="world";

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


        // Free the ownership
        PredictedState.reset();


        // Publish Robot Pose
        robotPosePub.publish(robotPoseMsg);

        // Sleep
        robotPoseRate->sleep();
    }


    {
        std::ostringstream logString;
        logString<<"MsfLocalizationROS::robotPoseThreadFunction() ended"<<std::endl;
        this->log(logString.str());
    }

    return 0;
}

TimeStamp MsfLocalizationROS::getTimeStamp()
{
    ros::Time RosTimeStamp=ros::Time::now();

    return TimeStamp(RosTimeStamp.sec, RosTimeStamp.nsec);
}

int MsfLocalizationROS::predictThreadFunction()
{   
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationROS::predictThreadFunction()"<<std::endl;
        this->log(logString.str());
    }

//return 0;
    ros::Rate predictRate(predictRateVale);
    //ros::Rate predictRate(100);

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


        // Search if element already exists
        // TODO
        TimeStamp TheTimeStamp=getTimeStamp();
        std::shared_ptr<StateEstimationCore> ThePredictedState;

        // Predict. Typically it will not be any
        //std::cout<<"Calling predict TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
        this->TheMsfStorageCore->getElement(TheTimeStamp, ThePredictedState);

        while(!ThePredictedState)
        {
//            predictThreadState.setProcessing(TheTimeStamp);
            this->predict(TheTimeStamp, ThePredictedState);
//            predictThreadState.setNotProcessing();
        }

        ThePredictedState.reset();


        // Purge the buffer
        //this->TheMsfStorageCore->purgeRingBuffer(20);


        // Display the buffer
        //this->TheMsfStorageCore->displayRingBuffer();

        // Sleep
        predictRate.sleep();
    }


    {
        std::ostringstream logString;
        logString<<"MsfLocalizationROS::predictThreadFunction() ended"<<std::endl;
        this->log(logString.str());
    }

    return 0;

}


int MsfLocalizationROS::bufferManagerThreadFunction()
{

    {
        std::ostringstream logString;
        logString<<"MsfLocalizationROS::bufferManagerThreadFunction()"<<std::endl;
        this->log(logString.str());
    }

    //ros::Rate bufferManagerRate(10000);

    while(ros::ok())
    {



        // Get oldest element
        TimeStamp OldestTimeStamp;
        if(this->TheMsfStorageCore->getOldestOutdatedElement(OldestTimeStamp))
        {
            std::cout<<"MsfLocalizationROS::bufferManagerThreadFunction() error 0!"<<std::endl;
            continue;
        }

        {
            std::ostringstream logString;
            logString<<"MsfLocalizationROS::bufferManagerThreadFunction() updating TS: sec="<<OldestTimeStamp.sec<<" s; nsec="<<OldestTimeStamp.nsec<<" ns"<<std::endl;
            this->log(logString.str());
        }

        {
            this->log(this->TheMsfStorageCore->getDisplayOutdatedElements());
        }

        // Get state estimation core associated to the oldest element on the buffer
        std::shared_ptr<StateEstimationCore> TheOutdatedElement;
        if(this->TheMsfStorageCore->getElement(OldestTimeStamp, TheOutdatedElement))
        {
            std::cout<<"MsfLocalizationROS::bufferManagerThreadFunction() error1!"<<std::endl;
            continue;
        }

        // Check
        if(!TheOutdatedElement)
        {
            std::cout<<"MsfLocalizationROS::bufferManagerThreadFunction() error2!"<<std::endl;
            continue;
        }


        // Run predict and store updated predicted element
        if(this->predict(OldestTimeStamp, TheOutdatedElement))
        {
            std::cout<<"MsfLocalizationROS::bufferManagerThreadFunction() error 5!"<<std::endl;
            continue;
        }

        // Not needed
        if(!TheOutdatedElement)
        {
            std::cout<<"MsfLocalizationROS::bufferManagerThreadFunction() error4!"<<std::endl;
            continue;
        }


        // Run update if there are measurements
        // TODO
        this->update(OldestTimeStamp, TheOutdatedElement);


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




        if(!this->TheMsfStorageCore->getNextTimeStamp(OldestTimeStamp, TheNewOutdatedTimeStamp))
        {
            // Set the following element of the buffer as outdated
            this->TheMsfStorageCore->addOutdatedElement(TheNewOutdatedTimeStamp);

            {
                std::ostringstream logString;
                logString<<"MsfLocalizationROS::bufferManagerThreadFunction() Adding to be processed TS: sec="<<TheNewOutdatedTimeStamp.sec<<" s; nsec="<<TheNewOutdatedTimeStamp.nsec<<" ns"<<std::endl;
                this->log(logString.str());
            }
        }
        else
        {

            {
                std::ostringstream logString;
                logString<<"MsfLocalizationROS::bufferManagerThreadFunction() Nothing new to be added"<<std::endl;
                this->log(logString.str());
            }

        }




        // Purge the buffer ??
        this->TheMsfStorageCore->purgeRingBuffer(50);


        // Display the buffer
        //this->TheMsfStorageCore->displayRingBuffer();


        // Purge the buffer
        //this->TheMsfStorageCore->purgeRingBuffer(20);


        // Sleep
        //bufferManagerRate.sleep();
    }

    {
        std::ostringstream logString;
        logString<<"MsfLocalizationROS::bufferManagerThreadFunction() ended"<<std::endl;
        this->log(logString.str());
    }

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
    catch (std::exception &ex)
    {
        std::cout<<"[ROSNODE] Exception :"<<ex.what()<<std::endl;
    }

    return 0;
}
