
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
    predict_model_time_=0.02; // In seconds



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

        // Push the core
        TheGlobalParametersCore=TheGlobalParametersCoreAux;

        // Push the init state
        InitialState->TheGlobalParametersStateCore=TheGlobalParametersStateCore;

    }


    ///// Robot

    // Reading Robot
    for(pugi::xml_node robot = msf_localization.child("robot"); robot; robot = robot.next_sibling("robot"))
    {
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

            // break to ensure than only one robot model is read
            break;
        }

        /// Free Model Type
        if(robotType=="imu_driven")
        {
            std::cout<<"robot = imu_driven"<<std::endl;

            // Create the RobotCoreAux
            std::shared_ptr<RosImuDrivenRobotInterface> TheRobotCoreAux;
            // Create a class for the RobotStateCore
            std::shared_ptr<ImuDrivenRobotStateCore> RobotInitStateCore;

            // Create the RobotCoreAux
            if(!TheRobotCoreAux)
            {
                TheRobotCoreAux=std::make_shared<RosImuDrivenRobotInterface>(nh, this->tf_transform_broadcaster_, this->TheMsfStorageCore);
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

            // break to ensure than only one robot model is read
            break;
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

        //// Mocap Sensor Type
        if(sensorType=="absolute_pose")
        {
            std::cout<<"sensor = absolute_pose"<<std::endl;

            // Create a class for the SensoreCore
            std::shared_ptr<RosMocapSensorInterface> TheRosSensorInterface;
            // Create a class for the SensorStateCore
            std::shared_ptr<AbsolutePoseSensorStateCore> TheSensorStateCore;

            // Create a class for the SensoreCore
            if(!TheRosSensorInterface)
                TheRosSensorInterface=std::make_shared<RosMocapSensorInterface>(nh, this->tf_transform_broadcaster_, this->TheMsfStorageCore);

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

        /// Coded visual marker
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

        /// Mocap World
        if(mapElementType=="world_ref_frame")
        {
            std::cout<<"map element = world_ref_frame"<<std::endl;

            // Create a class for the SensoreCore
            std::shared_ptr<WorldReferenceFrameCore> TheMapElementCore;
            // Create a class for the SensorStateCore
            std::shared_ptr<WorldReferenceFrameStateCore> MapElementInitStateCore;


            // Create a class for the MapElementCore
            if(!TheMapElementCore)
                TheMapElementCore=std::make_shared<WorldReferenceFrameCore>(this->TheMsfStorageCore);

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
                                                                                  std::dynamic_pointer_cast<RobotStateCore>(PredictedState->TheRobotStateCore),
                                                                                  PredictedState->covarianceMatrix->block(covRobotPointInit, covRobotPointInit, covRobotSize, covRobotSize));




        // Sensors
        for(std::list< std::shared_ptr<StateCore> >::const_iterator itSensorState=PredictedState->TheListSensorStateCore.begin();
            itSensorState!=PredictedState->TheListSensorStateCore.end();
            ++itSensorState)
        {
            std::dynamic_pointer_cast<RosSensorInterface>((*itSensorState)->getMsfElementCoreSharedPtr())->publish(TheTimeStamp,
                                                                                                                   std::dynamic_pointer_cast<RosRobotInterface>(this->TheRobotCore),
                                                                                                                   std::dynamic_pointer_cast<SensorStateCore>((*itSensorState)));
        }


        // TF Map elements
        for(std::list< std::shared_ptr<StateCore> >::const_iterator itMapElementState=PredictedState->TheListMapElementStateCore.begin();
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

                    break;
                }
            case MapElementTypes::world_ref_frame:
            {
                // Cast
                std::shared_ptr<WorldReferenceFrameStateCore> theCodedVisualMarkersLandamarkState=std::dynamic_pointer_cast<WorldReferenceFrameStateCore>(*itMapElementState);

                Eigen::Vector3d mapElementPosition=theCodedVisualMarkersLandamarkState->getPositionMocapWorldWrtWorld();
                Eigen::Vector4d mapElementAttitude=theCodedVisualMarkersLandamarkState->getAttitudeMocapWorldWrtWorld();




                tf::Quaternion tf_rot(mapElementAttitude[1], mapElementAttitude[2], mapElementAttitude[3], mapElementAttitude[0]);
                tf::Vector3 tf_tran(mapElementPosition[0], mapElementPosition[1], mapElementPosition[2]);

                tf::Transform transform(tf_rot, tf_tran);


                tf_transform_broadcaster_->sendTransform(tf::StampedTransform(transform, ros::Time(TheTimeStamp.sec, TheTimeStamp.nsec),
                                                      this->TheGlobalParametersCore->getWorldName(), std::dynamic_pointer_cast<MapElementCore>((*itMapElementState)->getMsfElementCoreSharedPtr())->getMapElementName()));

                break;
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

bool MsfLocalizationROS::isAlive()
{
    return ros::ok();
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

    ros::Rate predictRate(1/predict_model_time_);


    while(isAlive())
    {
        // Check if enabled
        if(!this->isStateEstimationEnabled())
        {
            // Sleep
            predictRate.sleep();
            // continue
            continue;
        }

        // Predict thread step
        int error_predict_thread_step=predictThreadStep();
        if(error_predict_thread_step)
        {
            continue;
        }

        // Sleep
        predictRate.sleep();
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
