
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
    StateEstimationCore InitialState;



    // Reading Configs
    // TODO
    predictRateVale=100.0; // In Hz



    ///// Robot
    // Reading Robot
    pugi::xml_node robot = msf_localization.child("robot");



    // Create the RobotCore
    TheRobotCore=std::make_shared<FreeModelRobotCore>();
    // Set pointer to the RobotCore
    TheRobotCore->setTheRobotCore(TheRobotCore);

    // Set robot type
    TheRobotCore->setRobotType(RobotTypes::free_model);

    // Set the access to the Storage core
    TheRobotCore->setTheMsfStorageCore(this->TheMsfStorageCore);

    // Create a class for the RobotStateCore
    std::shared_ptr<FreeModelRobotStateCore> RobotInitStateCore=std::make_shared<FreeModelRobotStateCore>();
    // Set pointer to the SensorCore
    RobotInitStateCore->setTheRobotCore(TheRobotCore);

    // Init State

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


    // TODO Variances



    // Push the init state of the robot
    InitialState.TheRobotStateCore=RobotInitStateCore;



    ///// Sensors
    // Reading sensors
    for(pugi::xml_node sensor = msf_localization.child("sensor"); sensor; sensor = sensor.next_sibling("sensor"))
    {
        std::cout<<"sensor; "<<std::endl;

        // Sensor Type
        std::string sensorType=sensor.child_value("type");


        // IMU Sensor Type
        if(sensorType=="imu")
        {
            // Create a class for the SensoreCore
            std::shared_ptr<RosSensorImuInterface> TheRosSensorImuInterface=std::make_shared<RosSensorImuInterface>();
            // Set pointer to the SensorCore
            TheRosSensorImuInterface->setTheSensorCore(TheRosSensorImuInterface);

            // Create a class for the SensorStateCore
            std::shared_ptr<ImuSensorStateCore> SensorInitStateCore=std::make_shared<ImuSensorStateCore>();
            // Set pointer to the SensorCore
            SensorInitStateCore->setTheSensorCore(TheRosSensorImuInterface);


            // Set sensor type
            TheRosSensorImuInterface->setSensorType(SensorTypes::imu);

            // Set Id
            TheRosSensorImuInterface->setSensorId(this->firstAvailableId);
            firstAvailableId++;

            // Set the access to the Storage core
            //TheRosSensorImuInterface->setTheMsfStorageCore(std::make_shared<MsfStorageCore>(this->TheStateEstimationCore));
            TheRosSensorImuInterface->setTheMsfStorageCore(this->TheMsfStorageCore);


            // Sensor Topic
            std::string sensorTopic=sensor.child_value("ros_topic");
            TheRosSensorImuInterface->setImuTopicName(sensorTopic);


            //// Pose of the sensor wrt robot
            pugi::xml_node pose_in_robot=sensor.child("pose_in_robot");

            // Position of the sensor wrt robot
            readingValue=pose_in_robot.child("position").child_value("enabled");
            if(std::stoi(readingValue))
                TheRosSensorImuInterface->enableEstimationPositionSensorWrtRobot();

            readingValue=pose_in_robot.child("position").child_value("init_estimation");
            {
                std::istringstream stm(readingValue);
                Eigen::Vector3d init_estimation;
                stm>>init_estimation[0]>>init_estimation[1]>>init_estimation[2];
                SensorInitStateCore->setPositionSensorWrtRobot(init_estimation); //TODO
            }

            readingValue=pose_in_robot.child("position").child_value("init_var");
            // TODO


            // Attitude of the sensor wrt robot
            readingValue=pose_in_robot.child("attitude").child_value("enabled");
            if(std::stoi(readingValue))
                TheRosSensorImuInterface->enableEstimationAttitudeSensorWrtRobot();

            readingValue=pose_in_robot.child("attitude").child_value("init_estimation");
            {
                std::istringstream stm(readingValue);
                Eigen::Vector4d init_estimation;
                stm>>init_estimation[0]>>init_estimation[1]>>init_estimation[2]>>init_estimation[3];
                SensorInitStateCore->setAttitudeSensorWrtRobot(init_estimation); //TODO
            }

            readingValue=pose_in_robot.child("attitude").child_value("init_var");
            // TODO



            //// Measurements
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




            /// Parameters
            pugi::xml_node parameters = sensor.child("parameters");


            // Angular Velocity
            pugi::xml_node param_angular_velocity = parameters.child("angular_velocity");

            readingValue=param_angular_velocity.child("biases").child_value("enabled");
            if(std::stoi(readingValue))
            {
                TheRosSensorImuInterface->enableEstimationBiasAngularVelocity();
            }

            readingValue=param_angular_velocity.child("biases").child_value("init_estimation");
            {
                std::istringstream stm(readingValue);

                Eigen::Vector3d biasesAngularVelocity;

                stm>>biasesAngularVelocity[0]>>biasesAngularVelocity[1]>>biasesAngularVelocity[2];

                SensorInitStateCore->setBiasesAngularVelocity(biasesAngularVelocity);
            }

            readingValue=param_angular_velocity.child("biases").child_value("var");


            // Linear Acceleration
            pugi::xml_node param_linear_acceleration = parameters.child("linear_acceleration");

            readingValue=param_linear_acceleration.child("biases").child_value("enabled");
            if(std::stoi(readingValue))
                TheRosSensorImuInterface->enableEstimationBiasLinearAcceleration();

            readingValue=param_linear_acceleration.child("biases").child_value("init_estimation");
            {
                std::istringstream stm(readingValue);
                Eigen::Vector3d biasesLinearAcceleration;
                stm>>biasesLinearAcceleration[0]>>biasesLinearAcceleration[1]>>biasesLinearAcceleration[2];
                SensorInitStateCore->setBiasesLinearAcceleration(biasesLinearAcceleration);
            }

            readingValue=param_linear_acceleration.child("biases").child_value("var");
            // TODO



            // Open
            TheRosSensorImuInterface->open();

            // Push to the list of sensors
            this->TheListOfSensorCore.push_back(TheRosSensorImuInterface);

            // Push the init state of the sensor
            InitialState.TheListSensorStateCore.push_back(SensorInitStateCore);
        }




    }


    // Add init state to the buffer
    TheMsfStorageCore->addElement(TimeStamp(0,0),InitialState);


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


    return 0;
}


int MsfLocalizationROS::robotPoseThreadFunction()
{
    robotPoseRate=new ros::Rate(robotPoseRateVal);

    while(ros::ok())
    {
        // Get Robot Pose
        // TODO
        TimeStamp TheTimeStamp;
        StateEstimationCore PreviousState;
        this->TheMsfStorageCore->getLastElementWithStateEstimate(TheTimeStamp, PreviousState);

        // Fill msg
        robotPoseMsg.header.stamp=ros::Time(TheTimeStamp.sec, TheTimeStamp.nsec);

        // Publish Robot Pose
        robotPosePub.publish(robotPoseMsg);

        // Sleep
        robotPoseRate->sleep();
    }


    return 0;
}

int MsfLocalizationROS::predictThreadFunction()
{
    std::cout<<"MsfLocalizationROS::predictThreadFunction()"<<std::endl;

    ros::Rate predictRate(predictRateVale);

    while(ros::ok() && predictEnabled)
    {
        // Get Time Stamp
        ros::Time RosTimeStamp=ros::Time::now();
        TimeStamp TheTimeStamp(RosTimeStamp.sec, RosTimeStamp.nsec);
        this->predict(TheTimeStamp);

        // Sleep
        predictRate.sleep();
    }

    std::cout<<"MsfLocalizationROS::predictThreadFunction() ended"<<std::endl;

    return 0;

}


int MsfLocalizationROS::bufferManagerThreadFunction()
{
    std::cout<<"MsfLocalizationROS::bufferManagerThreadFunction()"<<std::endl;

    ros::Rate bufferManagerRate(50);

    while(ros::ok())
    {
        // Purge the buffer
        //this->TheMsfStorageCore->purgeRingBuffer(20);


        // Display the buffer
        this->TheMsfStorageCore->displayRingBuffer();


        // Purge the buffer
        this->TheMsfStorageCore->purgeRingBuffer(20);


        // Sleep
        bufferManagerRate.sleep();
    }


    return 0;

}


int MsfLocalizationROS::run()
{
    // Enable prediction
    setPredictEnabled(true);

    // Core threads
    startThreads();

    // Start ROS threads
    robotPoseThread=new std::thread(&MsfLocalizationROS::robotPoseThreadFunction, this);


    // Loop
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
