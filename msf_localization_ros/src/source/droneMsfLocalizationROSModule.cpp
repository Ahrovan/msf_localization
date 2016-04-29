
#include "msf_localization_ros/droneMsfLocalizationROSModule.h"

MsfLocalizationROS::MsfLocalizationROS(int argc,char **argv)
{
    //Ros Init
    ros::init(argc, argv, ros::this_node::getName());

    // Node handle
    nh=new ros::NodeHandle();

    // tf broadcaster
    tf_transform_broadcaster_=new tf::TransformBroadcaster;

    // Init
    init();

    return;
}

MsfLocalizationROS::~MsfLocalizationROS()
{
    // Close
    close();

    // Delete
    delete tf_transform_broadcaster_;

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



    //// Global Parameters [world]

    {
        // Reading Robot
        pugi::xml_node global_parameters = msf_localization.child("global_parameters");

        std::cout<<"global parameters"<<std::endl;

        // Create the CoreAux
        std::shared_ptr<GlobalParametersCore> TheGlobalParametersCoreAux;
        // Create a class for the RobotStateCore
        std::shared_ptr<GlobalParametersStateCore> TheGlobalParametersStateCore;

        // Create the GlobalParameters
        if(!TheGlobalParametersCoreAux)
            TheGlobalParametersCoreAux=std::make_shared<GlobalParametersCore>(this->TheMsfStorageCore);

        // Set the pointer to itself
        TheGlobalParametersCoreAux->setMsfElementCorePtr(TheGlobalParametersCoreAux);


        // Read configs
        if(TheGlobalParametersCoreAux->readConfig(global_parameters, TheGlobalParametersStateCore))
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

        // Robot Type
        std::string robotType=robot.child_value("type");

        /// Free Model Type
        if(robotType=="free_model")
        {
            std::cout<<"robot = free_model"<<std::endl;

            // Create the RobotCoreAux
            std::shared_ptr<RosFreeModelRobotInterface> TheRobotCoreAux;
            // Create a class for the RobotStateCore
            std::shared_ptr<FreeModelRobotStateCore> RobotInitStateCore;

            // Create the RobotCoreAux
            if(!TheRobotCoreAux)
            {
                TheRobotCoreAux=std::make_shared<RosFreeModelRobotInterface>(nh, this->tf_transform_broadcaster_, this->TheMsfStorageCore);
            }

            // Set the pointer to itself
            TheRobotCoreAux->setMsfElementCorePtr(TheRobotCoreAux);


            // Read configs
            if(TheRobotCoreAux->readConfig(robot, RobotInitStateCore))
                return -2;

            // Finish

            // Push the robot core
            TheRobotCore=TheRobotCoreAux;

            // Push the init state of the robot
            InitialState->TheRobotStateCore=RobotInitStateCore;

        }

    }


    ///// Sensors
    // Reading sensors
    for(pugi::xml_node sensor = msf_localization.child("sensor"); sensor; sensor = sensor.next_sibling("sensor"))
    {
        // Sensor Type
        std::string sensorType=sensor.child_value("type");


        //// IMU Sensor Type
        if(sensorType=="imu")
        {
            std::cout<<"sensor = imu"<<std::endl;

            // Create a class for the SensoreCore
            std::shared_ptr<RosSensorImuInterface> TheRosSensorInterface;
            // Create a class for the SensorStateCore
            std::shared_ptr<ImuSensorStateCore> TheSensorStateCore;

            // Create a class for the SensoreCore
            if(!TheRosSensorInterface)
            {
                TheRosSensorInterface=std::make_shared<RosSensorImuInterface>(nh, this->tf_transform_broadcaster_, this->TheMsfStorageCore);
            }

            // Set the pointer to itself
            TheRosSensorInterface->setMsfElementCorePtr(TheRosSensorInterface);

            // Read configs
            if(TheRosSensorInterface->readConfig(sensor, firstAvailableSensorId, TheSensorStateCore))
                return -2;

            // Update id
            firstAvailableSensorId++;


            // Finish

            // Push to the list of sensors
            this->TheListOfSensorCore.push_back(TheRosSensorInterface);

            // Push the init state of the sensor
            InitialState->TheListSensorStateCore.push_back(TheSensorStateCore);
        }

        //// Aruco Eye Sensor Type
        if(sensorType=="aruco_eye")
        {
            std::cout<<"sensor = aruco_eye"<<std::endl;

            // Create a class for the SensoreCore
            std::shared_ptr<RosArucoEyeInterface> TheRosSensorInterface;
            // Create a class for the SensorStateCore
            std::shared_ptr<CodedVisualMarkerEyeStateCore> TheSensorStateCore;

            // Create a class for the SensoreCore
            if(!TheRosSensorInterface)
                TheRosSensorInterface=std::make_shared<RosArucoEyeInterface>(nh, this->tf_transform_broadcaster_, this->TheMsfStorageCore);

            // Set the pointer to itself
            TheRosSensorInterface->setMsfElementCorePtr(TheRosSensorInterface);

            // Read configs
            if(TheRosSensorInterface->readConfig(sensor, firstAvailableSensorId, TheSensorStateCore))
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


    ///// Inputs
    // Reading inputs
    for(pugi::xml_node input = msf_localization.child("input"); input; input = input.next_sibling("input"))
    {
        // Input Type
        std::string inputType=input.child_value("type");


        //// IMU Sensor Type
        if(inputType=="imu")
        {
            std::cout<<"input = imu"<<std::endl;

            // Create a class for the SensoreCore
            std::shared_ptr<RosImuInputInterface> ros_input_interface;
            // Create a class for the SensorStateCore
            std::shared_ptr<ImuInputStateCore> input_state_core;

            // Create a class for the SensoreCore
            if(!ros_input_interface)
            {
                ros_input_interface=std::make_shared<RosImuInputInterface>(nh, this->tf_transform_broadcaster_, this->TheMsfStorageCore);
            }

            // Set the pointer to itself
            ros_input_interface->setMsfElementCorePtr(ros_input_interface);

            // Read configs
            if(ros_input_interface->readConfig(input, input_state_core))
                return -2;


            // Finish

            // Push to the list of sensors
            this->TheListOfInputCore.push_back(ros_input_interface);

            // Push the init state of the sensor
            InitialState->TheListInputStateCore.push_back(input_state_core);
        }


    }



    ///// Map elements


    // Reading map elements
    for(pugi::xml_node map_element = msf_localization.child("map_element"); map_element; map_element = map_element.next_sibling("map_element"))
    {
        // Sensor Type
        std::string mapElementType=map_element.child_value("type");


        // Coded visual marker
        if(mapElementType=="coded_visual_marker")
        {
            std::cout<<"map element = coded_visual_marker"<<std::endl;

            // Create a class for the SensoreCore
            std::shared_ptr<CodedVisualMarkerLandmarkCore> TheMapElementCore;
            // Create a class for the SensorStateCore
            std::shared_ptr<CodedVisualMarkerLandmarkStateCore> MapElementInitStateCore;


            // Create a class for the MapElementCore
            if(!TheMapElementCore)
                TheMapElementCore=std::make_shared<CodedVisualMarkerLandmarkCore>(this->TheMsfStorageCore);

            // Set the pointer to itself
            TheMapElementCore->setMsfElementCorePtr(TheMapElementCore);

            // Read configs
            if(TheMapElementCore->readConfig(map_element, MapElementInitStateCore))
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
    InitialState->prepareCovarianceInitErrorState();


    // Display
#if 0 && _DEBUG_MSF_LOCALIZATION_CORE
    logFile<<"Covariance Matrix"<<std::endl;
    logFile<<InitialState->covarianceMatrix<<std::endl;
#endif


    // Add init state to the buffer
    TheMsfStorageCore->addElement(TimeStamp(0,0), InitialState);

    return 0;
}

int MsfLocalizationROS::readParameters()
{
    // Config files
    //
    ros::param::param<std::string>("~msf_localization_config_file", configFile, "msf_localization_config_file.xml");
    std::cout<<"msf_localization_config_file="<<configFile<<std::endl;


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
    // Read parameters
    if(readParameters())
        ROS_ERROR("Error Reading Parameters");

    // Read config file
    if(readConfigFile())
        ROS_ERROR("Error Reading Config File");


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

#if _DEBUG_MODE
    return 0;
#endif

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




        // Robot
        int covRobotPointInit=this->TheGlobalParametersCore->getDimensionErrorState();
        int covRobotSize=this->TheRobotCore->getDimensionErrorState();
        std::dynamic_pointer_cast<RosRobotInterface>(this->TheRobotCore)->publish(TheTimeStamp,
                                                                                  this->TheGlobalParametersCore,
                                                                                  PredictedState->TheRobotStateCore,
                                                                                  PredictedState->covarianceMatrix.block(covRobotPointInit, covRobotPointInit, covRobotSize, covRobotSize));




        // Sensors
        for(std::list< std::shared_ptr<SensorStateCore> >::const_iterator itSensorState=PredictedState->TheListSensorStateCore.begin();
            itSensorState!=PredictedState->TheListSensorStateCore.end();
            ++itSensorState)
        {
            std::dynamic_pointer_cast<RosSensorInterface>((*itSensorState)->getMsfElementCoreSharedPtr())->publish(TheTimeStamp,
                                                                                                                   std::dynamic_pointer_cast<RosRobotInterface>(this->TheRobotCore),
                                                                                                                   (*itSensorState));
        }


        // TF Map elements
        for(std::list< std::shared_ptr<MapElementStateCore> >::const_iterator itMapElementState=PredictedState->TheListMapElementStateCore.begin();
            itMapElementState!=PredictedState->TheListMapElementStateCore.end();
            ++itMapElementState)
        {

            switch(std::dynamic_pointer_cast<MapElementCore>((*itMapElementState)->getMsfElementCoreSharedPtr())->getMapElementType())
            {
                case MapElementTypes::coded_visual_marker:
                {
                    // Cast
                    std::shared_ptr<CodedVisualMarkerLandmarkStateCore> theCodedVisualMarkersLandamarkState=std::dynamic_pointer_cast<CodedVisualMarkerLandmarkStateCore>(*itMapElementState);

                    Eigen::Vector3d mapElementPosition=theCodedVisualMarkersLandamarkState->getPosition();
                    Eigen::Vector4d mapElementAttitude=theCodedVisualMarkersLandamarkState->getAttitude();


#if _DEBUG_MSF_LOCALIZATION_ROBOT_POSE_THREAD
                    {
                        std::ostringstream logString;
                        logString<<"MsfLocalizationROS::robotPoseThreadFunction()"<<std::endl;

                        logString<<"Visual Marker id="<<std::dynamic_pointer_cast<CodedVisualMarkerLandmarkCore>(theCodedVisualMarkersLandamarkState->getTheMapElementCore())->getId()<<std::endl;
                        logString<<"  - Position: "<<mapElementPosition.transpose()<<std::endl;
                        logString<<"  - Attitude: "<<mapElementAttitude.transpose()<<std::endl;


                        this->log(logString.str());
                    }
#endif



                    tf::Quaternion tf_rot(mapElementAttitude[1], mapElementAttitude[2], mapElementAttitude[3], mapElementAttitude[0]);
                    tf::Vector3 tf_tran(mapElementPosition[0], mapElementPosition[1], mapElementPosition[2]);

                    tf::Transform transform(tf_rot, tf_tran);


                    tf_transform_broadcaster_->sendTransform(tf::StampedTransform(transform, ros::Time(TheTimeStamp.sec, TheTimeStamp.nsec),
                                                          this->TheGlobalParametersCore->getWorldName(), std::dynamic_pointer_cast<MapElementCore>((*itMapElementState)->getMsfElementCoreSharedPtr())->getMapElementName()));

                }
            }
        }




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

#if _DEBUG_MODE
    return 0;
#endif
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


#if _DEBUG_TIME_MSF_LOCALIZATION_ROS
        {
            ros::Time begin = ros::Time::now();
#endif

            errorUpdate=this->update(OldestTimeStamp);


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
        this->TheMsfStorageCore->purgeRingBuffer(30);


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
