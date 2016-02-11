
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




    // Reading Robot
    // TODO



    // Reading sensors
    for(pugi::xml_node sensor = msf_localization.child("sensor"); sensor; sensor = sensor.next_sibling("sensor"))
    {
        std::cout<<"sensor; "<<std::endl;

        // Sensor Type
        std::string sensorType=sensor.child_value("type");

        // Pose of the sensor wrt robot
        pugi::xml_node pose_in_robot=sensor.child("pose_in_robot");

        readingValue=pose_in_robot.child_value("position");
        readingValue=pose_in_robot.child_value("attitude");
        readingValue=pose_in_robot.child_value("var_position");
        readingValue=pose_in_robot.child_value("var_attitude");


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

            // Pose of the sensor wrt robot
            // TODO. Antes?

            // Measurements
            pugi::xml_node measurements = sensor.child("measurements");

            // Orientation
            pugi::xml_node orientation = measurements.child("orientation");

            readingValue=orientation.child_value("enabled");
            readingValue=orientation.child_value("var");


            // Angular Velocity
            pugi::xml_node angular_velocity = measurements.child("angular_velocity");

            readingValue=angular_velocity.child_value("enabled");

            TheRosSensorImuInterface->enableAngularVelocity();


            readingValue=angular_velocity.child_value("var");

            readingValue=angular_velocity.child("biases").child_value("init_estimation");
            // TODO fill

            readingValue=angular_velocity.child("biases").child_value("var");


            // Linear Acceleration
            pugi::xml_node linear_acceleration = measurements.child("linear_acceleration");

            readingValue=linear_acceleration.child_value("enabled");
            readingValue=linear_acceleration.child_value("var");

            readingValue=linear_acceleration.child("biases").child_value("init_estimation");
            readingValue=linear_acceleration.child("biases").child_value("var");


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
    // Start ROS threads
    robotPoseThread=new std::thread(&MsfLocalizationROS::robotPoseThreadFunction, this);

    // Core threads
    startThreads();

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
