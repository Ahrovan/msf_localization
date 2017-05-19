
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
    if(tf_transform_broadcaster_)
        delete tf_transform_broadcaster_;
    if(nh)
        delete nh;
    if(publish_rate_)
        delete publish_rate_;

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
    std::shared_ptr<StateComponent> InitialState=std::make_shared<StateComponent>();



    // Reading Configs
    // TODO
    //predict_model_time_=0.02; // In seconds


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
            TheGlobalParametersCoreAux=std::make_shared<GlobalParametersCore>(this);

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
                TheRobotCoreAux=std::make_shared<RosFreeModelRobotInterface>(nh, this->tf_transform_broadcaster_, this);
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

        /// Imu Driven Type
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
                TheRobotCoreAux=std::make_shared<RosImuDrivenRobotInterface>(nh, this->tf_transform_broadcaster_, this);
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

        /// Absolute Pose Driven Model Type
        if(robotType=="absolute_pose_driven")
        {
            std::cout<<"robot = absolute_pose_driven"<<std::endl;

            // Create the RobotCoreAux
            std::shared_ptr<RosAbsolutePoseDrivenRobotInterface> TheRobotCoreAux;
            // Create a class for the RobotStateCore
            std::shared_ptr<AbsolutePoseDrivenRobotStateCore> RobotInitStateCore;

            // Create the RobotCoreAux
            if(!TheRobotCoreAux)
            {
                TheRobotCoreAux=std::make_shared<RosAbsolutePoseDrivenRobotInterface>(nh, this->tf_transform_broadcaster_, this);
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
            std::shared_ptr<RosImuSensorInterface> TheRosSensorInterface;
            // Create a class for the SensorStateCore
            std::shared_ptr<ImuSensorStateCore> TheSensorStateCore;

            // Create a class for the SensoreCore
            if(!TheRosSensorInterface)
            {
                TheRosSensorInterface=std::make_shared<RosImuSensorInterface>(nh, this->tf_transform_broadcaster_, this);
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
                TheRosSensorInterface=std::make_shared<RosArucoEyeInterface>(nh, this->tf_transform_broadcaster_, this);

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
            std::shared_ptr<RosAbsolutePoseSensorInterface> TheRosSensorInterface;
            // Create a class for the SensorStateCore
            std::shared_ptr<AbsolutePoseSensorStateCore> TheSensorStateCore;

            // Create a class for the SensoreCore
            if(!TheRosSensorInterface)
                TheRosSensorInterface=std::make_shared<RosAbsolutePoseSensorInterface>(nh, this->tf_transform_broadcaster_, this);

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

        //// Px4Flow Sensor Type
        if(sensorType=="px4flow")
        {
            std::cout<<"sensor = px4flow"<<std::endl;

            // Create a class for the SensoreCore
            std::shared_ptr<RosPx4FlowSensorInterface> TheRosSensorInterface;
            // Create a class for the SensorStateCore
            std::shared_ptr<Px4FlowSensorStateCore> TheSensorStateCore;

            // Create a class for the SensoreCore
            if(!TheRosSensorInterface)
                TheRosSensorInterface=std::make_shared<RosPx4FlowSensorInterface>(nh, this->tf_transform_broadcaster_, this);

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


        //// IMU Input Type
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
                ros_input_interface=std::make_shared<RosImuInputInterface>(nh, this->tf_transform_broadcaster_, this);
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

        //// Absolute Pose Input Type
        if(inputType=="absolute_pose")
        {
            std::cout<<"input = absolute_pose"<<std::endl;

            // Create a class for the SensoreCore
            std::shared_ptr<RosAbsolutePoseInputInterface> ros_input_interface;
            // Create a class for the SensorStateCore
            std::shared_ptr<AbsolutePoseInputStateCore> input_state_core;

            // Create a class for the SensoreCore
            if(!ros_input_interface)
            {
                ros_input_interface=std::make_shared<RosAbsolutePoseInputInterface>(nh, this->tf_transform_broadcaster_, this);
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
                TheMapElementCore=std::make_shared<CodedVisualMarkerLandmarkCore>(this);

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

        /// World Ref Frame
        if(mapElementType=="world_ref_frame")
        {
            std::cout<<"map element = world_ref_frame"<<std::endl;

            // Create a class for the SensoreCore
            std::shared_ptr<WorldReferenceFrameCore> TheMapElementCore;
            // Create a class for the SensorStateCore
            std::shared_ptr<WorldReferenceFrameStateCore> MapElementInitStateCore;


            // Create a class for the MapElementCore
            if(!TheMapElementCore)
                TheMapElementCore=std::make_shared<WorldReferenceFrameCore>(this);

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


#if _USE_BUFFER_IN_STATE_ESTIMATION
    // Add init state to the buffer
    std::shared_ptr<StateEstimationCore> init_state_estimation_core=std::make_shared<StateEstimationCore>();
    init_state_estimation_core->state_component_=InitialState;
    TheMsfStorageCore->addElement(TimeStamp(0,0), init_state_estimation_core);
#else
    // Set the init state as the current state
    current_state_=InitialState;
    current_time_stamp_=TimeStamp(0,0);
#endif

    // End
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

    // Publishers Topics
    //
    ros::param::param<std::string>("~new_measurement_notification_topic_name", new_measurement_notification_topic_name_, "msf_localization/new_measurement_notification");
    std::cout<<"new_measurement_notification_topic_name="<<new_measurement_notification_topic_name_<<std::endl;

    // Others configs
    //
    ros::param::param<double>("~robot_pose_rate", publish_rate_val_, 50);
    std::cout<<"robot_pose_rate="<<publish_rate_val_<<std::endl;
    //
    ros::param::param<double>("~predict_model_time", predict_model_time_, 0.02);
    std::cout<<"predict_model_time="<<predict_model_time_<<std::endl;


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

    // Publishers
    new_measurement_notification_pub_ = nh->advertise<std_msgs::Time>(new_measurement_notification_topic_name_, 10);

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


int MsfLocalizationROS::publishThreadFunction()
{
#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationROS::publishThreadFunction()"<<std::endl;
        this->log(logString.str());
    }
#endif

#if _DEBUG_MODE
    return 0;
#endif

    // Syncro Rate
    if( publish_rate_val_ > 0 )
    {
        // ROS rate
        publish_rate_=new ros::Rate(publish_rate_val_);
    }

    // variables for current_state
    TimeStamp current_time_stamp;
#if _USE_BUFFER_IN_STATE_ESTIMATION
    std::shared_ptr<StateEstimationCore> current_state_estimation_core;
#endif
    std::shared_ptr<StateComponent> current_state;



    // Loop
    while(ros::ok())
    {

#if _DEBUG_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationROS::publishThreadFunction() loop init"<<std::endl;
            this->log(logString.str());
        }
#endif

#if _USE_BUFFER_IN_STATE_ESTIMATION
        // Free the ownership of current_state (if any)
        if(current_state_estimation_core)
            current_state_estimation_core.reset();
#endif

        // Free the ownership of current_state (if any)
        if(current_state)
            current_state.reset();



        // State estimation enabled
        if(this->isStateEstimationEnabled())
        {
            // Async wait
            if( publish_rate_val_ == 0 )
            {

#if _USE_BUFFER_IN_STATE_ESTIMATION
                // Sleep until we have an updated state
                this->TheMsfStorageCore->semaphoreBufferUpdated();

                // Get last element with state -> Do not do the predict (safe)
                int error_get_last_element_with_state_estimate=this->TheMsfStorageCore->getLastElementWithStateEstimate(current_time_stamp, current_state_estimation_core);
                if(error_get_last_element_with_state_estimate<0)
                {

#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
                    {
                        std::ostringstream logString;
                        logString<<"MsfLocalizationROS::publishThreadFunction() error -100 in getLastElementWithStateEstimate()"<<std::endl;
                        this->log(logString.str());
                    }
#endif
                    continue;
                }
#else
                // TODO
                this->semaphoreUpdatedStateWait(current_time_stamp);

                current_time_stamp=this->current_time_stamp_;
                current_state=this->current_state_;
#endif

            }
            else if( publish_rate_val_ > 0 )
            {

                // Get current element
                current_time_stamp=getTimeStamp();

#if 1 || _DEBUG_MSF_LOCALIZATION_CORE
                {
                    std::ostringstream logString;
                    logString<<"MsfLocalizationROS::publishThreadFunction() going to publish TS: sec="<<current_time_stamp.getSec()<<" s; nsec="<<current_time_stamp.getNSec()<<" ns"<<std::endl;
                    this->log(logString.str());
                }
#endif

#if _USE_BUFFER_IN_STATE_ESTIMATION
                // Find in the buffer the element with the current_time_stamp
                int error_get_element=this->TheMsfStorageCore->getElement(current_time_stamp, current_state_estimation_core);
                if(error_get_element<0)
                {
#if 1 || _DEBUG_MSF_LOCALIZATION_CORE
                    //std::cout<<getTimeStampString()<<"MsfLocalizationROS::publishThreadFunction() error -100 getElement()"<<std::endl;
                    {
                        std::ostringstream logString;
                        logString<<"MsfLocalizationROS::publishThreadFunction() error -100 getElement()"<<std::endl;
                        this->log(logString.str());
                    }
#endif

                    // Sleep
                    publish_rate_->sleep();

                    // continue
                    continue;
                }


                // If no current_state -> predict state but no add buffer
                if(!current_state_estimation_core)
                {

#if 1 || _DEBUG_MSF_LOCALIZATION_CORE
                    {
                        std::ostringstream logString;
                        logString<<"MsfLocalizationROS::publishThreadFunction() predicting TS: sec="<<current_time_stamp.getSec()<<" s; nsec="<<current_time_stamp.getNSec()<<" ns"<<std::endl;
                        this->log(logString.str());
                    }
#endif

                    if(this->predictInBufferNoAddBuffer(current_time_stamp, current_state_estimation_core))
                    {
                        // Error
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
                        {
                            std::ostringstream logString;
                            logString<<"MsfLocalizationROS::publishThreadFunction() error in predictInBufferNoAddBuffer()"<<std::endl;
                            this->log(logString.str());
                        }
#endif
                        continue;
                    }
                }
#else
                // TODO
                current_time_stamp=this->current_time_stamp_;
                current_state=this->current_state_;
#endif
            }
            else
            {
                // Do nothing: Sleep

            }
        }
        // State Estimation disabled
        else
        {
            // Get the last state estimation
#if _USE_BUFFER_IN_STATE_ESTIMATION
            // (safe)
            int error_get_last_element_with_state_estimate=this->TheMsfStorageCore->getLastElementWithStateEstimate(current_time_stamp, current_state_estimation_core);
            if(error_get_last_element_with_state_estimate<0)
            {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
                {
                    std::ostringstream logString;
                    logString<<"MsfLocalizationROS::publishThreadFunction() error -100 in getLastElementWithStateEstimate()"<<std::endl;
                    this->log(logString.str());
                }
#endif
                continue;
            }
#else
            current_time_stamp=this->current_time_stamp_;
            current_state=this->current_state_;
#endif


            // Time Stamp is null, put the current one
            if(current_time_stamp==TimeStamp(0,0))
            {
                current_time_stamp=getTimeStamp();
            }

        }


#if _USE_BUFFER_IN_STATE_ESTIMATION
        // Check Error with predicted state
        if(!current_state_estimation_core)
        {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationROS::publishThreadFunction() error 1!"<<std::endl;
                this->log(logString.str());
            }
#endif
            continue;
        }

        // Set current_state
        current_state=current_state_estimation_core->state_component_;
#endif

        // Check current_state
        if(!current_state)
        {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationROS::publishThreadFunction() error 2!"<<std::endl;
                this->log(logString.str());
            }
#endif
            continue;
        }



        // Publish
        if(publishState(current_time_stamp, current_state))
        {
            std::cout<<getTimeStampString()<<"error publishing state"<<std::endl;
            continue;
        }


#if _USE_BUFFER_IN_STATE_ESTIMATION
        // Free the ownership of current_state_estimation_core (not really needed)
        if(current_state_estimation_core)
            current_state_estimation_core.reset();
#endif

        // Free the ownership of current_state (not really needed)
        if(current_state)
            current_state.reset();


#if _DEBUG_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationROS::publishThreadFunction() loop end"<<std::endl;
            this->log(logString.str());
        }
#endif

        // Wait for the next iteration
        if(this->isStateEstimationEnabled())
        {
            // Sync
            if( publish_rate_val_ > 0 )
            {
                // Sleep
                publish_rate_->sleep();
            }
            // Asyn
            else
            {
                // Do nothing
            }
        }
        else
        {
            // Sleep
            ros::Rate(20).sleep();
        }
    }



#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationROS::publishThreadFunction() ended"<<std::endl;
        this->log(logString.str());
    }
#endif

    return 0;
}


int MsfLocalizationROS::publishState(const TimeStamp& current_time_stamp,
                                    const std::shared_ptr<StateComponent> &current_state)
{
    //Checks
    if(!current_state)
        return -1;
    if(!current_state->hasState())
        return -2;

    // Robot
    int covRobotPointInit=this->TheGlobalParametersCore->getDimensionErrorState();
    int covRobotSize=this->TheRobotCore->getDimensionErrorState();
    std::dynamic_pointer_cast<RosRobotInterface>(this->TheRobotCore)->publish(current_time_stamp,
                                                                              this->TheGlobalParametersCore,
                                                                              std::dynamic_pointer_cast<RobotStateCore>(current_state->TheRobotStateCore),
                                                                              current_state->covarianceMatrix->block(covRobotPointInit, covRobotPointInit, covRobotSize, covRobotSize));




    // Sensors
    for(std::list< std::shared_ptr<StateCore> >::const_iterator itSensorState=current_state->TheListSensorStateCore.begin();
        itSensorState!=current_state->TheListSensorStateCore.end();
        ++itSensorState)
    {
        std::dynamic_pointer_cast<RosSensorInterface>((*itSensorState)->getMsfElementCoreSharedPtr())->publish(current_time_stamp,
                                                                                                               std::dynamic_pointer_cast<RosRobotInterface>(this->TheRobotCore),
                                                                                                               std::dynamic_pointer_cast<SensorStateCore>((*itSensorState)));
    }


    // TF Map elements
    for(std::list< std::shared_ptr<StateCore> >::const_iterator itMapElementState=current_state->TheListMapElementStateCore.begin();
        itMapElementState!=current_state->TheListMapElementStateCore.end();
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


                tf::Quaternion tf_rot(mapElementAttitude[1], mapElementAttitude[2], mapElementAttitude[3], mapElementAttitude[0]);
                tf::Vector3 tf_tran(mapElementPosition[0], mapElementPosition[1], mapElementPosition[2]);

                tf::Transform transform(tf_rot, tf_tran);


                tf_transform_broadcaster_->sendTransform(tf::StampedTransform(transform, ros::Time(current_time_stamp.getSec(), current_time_stamp.getNSec()),
                                                      this->TheGlobalParametersCore->getWorldName(), std::dynamic_pointer_cast<MapElementCore>((*itMapElementState)->getMsfElementCoreSharedPtr())->getMapElementName()));

                break;
            }
            case MapElementTypes::world_ref_frame:
            {
                // Cast
                std::shared_ptr<WorldReferenceFrameStateCore> theCodedVisualMarkersLandamarkState=std::dynamic_pointer_cast<WorldReferenceFrameStateCore>(*itMapElementState);

                Eigen::Vector3d mapElementPosition=theCodedVisualMarkersLandamarkState->getPositionReferenceFrameWorldWrtWorld();
                Eigen::Vector4d mapElementAttitude=theCodedVisualMarkersLandamarkState->getAttitudeReferenceFrameWorldWrtWorld();


                tf::Quaternion tf_rot(mapElementAttitude[1], mapElementAttitude[2], mapElementAttitude[3], mapElementAttitude[0]);
                tf::Vector3 tf_tran(mapElementPosition[0], mapElementPosition[1], mapElementPosition[2]);

                tf::Transform transform(tf_rot, tf_tran);


                tf_transform_broadcaster_->sendTransform(tf::StampedTransform(transform, ros::Time(current_time_stamp.getSec(), current_time_stamp.getNSec()),
                                                      this->TheGlobalParametersCore->getWorldName(), std::dynamic_pointer_cast<MapElementCore>((*itMapElementState)->getMsfElementCoreSharedPtr())->getMapElementName()));

                break;
            }
        }
    }




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

    // Check if prediction is needed
    if(predict_model_time_ <= 0)
        return 0;


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

int MsfLocalizationROS::setNewMeasurementNotificationTopicName(std::string new_measurement_notification_topic_name)
{
    new_measurement_notification_topic_name_=new_measurement_notification_topic_name;
    return 0;
}

int MsfLocalizationROS::publishNewMeasurementNotification(const TimeStamp& measurement_time_stamp)
{
    new_measurement_notification_msgs_.data=ros::Time(measurement_time_stamp.getSec(), measurement_time_stamp.getNSec());

    new_measurement_notification_pub_.publish(new_measurement_notification_msgs_);

    return 0;
}
