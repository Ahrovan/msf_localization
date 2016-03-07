
#include "msf_localization_core/msfLocalization.h"



/*
SyncThreadState::SyncThreadState() :
    flagIsWorking(false)
{

}

bool SyncThreadState::isWorking()
{
    bool aux;
    protectionMutex.lock();
    aux=this->flagIsWorking;
    protectionMutex.unlock();
    return aux;
}

TimeStamp SyncThreadState::getProcessingTimeStamp()
{
    TimeStamp aux;
    protectionMutex.lock();
    aux=this->procesingTimeStamp;
    protectionMutex.unlock();
    return aux;
}


int SyncThreadState::setProcessing(TimeStamp procesingTimeStamp)
{
    protectionMutex.lock();
    this->flagIsWorking=true;
    this->procesingTimeStamp=procesingTimeStamp;
    protectionMutex.unlock();
    return 0;
}

int SyncThreadState::setNotProcessing()
{
    protectionMutex.lock();
    this->flagIsWorking=false;
    protectionMutex.unlock();
    return 0;
}


*/







MsfLocalizationCore::MsfLocalizationCore()
{
    // Flags
    stateEstimationEnabled=false;

    // Create Robot Core
    TheRobotCore=std::make_shared<RobotCore>();

    // Create Global Parameters Core
    TheGlobalParametersCore=std::make_shared<GlobalParametersCore>();

    // Sensors
    //Id
    firstAvailableId=0;


    // Create Storage Core
    TheMsfStorageCore=std::make_shared<MsfStorageCore>();



    // LOG
    const char* env_p = std::getenv("FUSEON_STACK");

    logPath=std::string(env_p)+"/logs/"+"logMsfLocalizationCoreFile.txt";

    logFile.open(logPath);

    if(!logFile.is_open())
    {
        std::cout<<"unable to open log file"<<std::endl;
    }



    return;
}

MsfLocalizationCore::~MsfLocalizationCore()
{
    // Cleaning

    // TheListOfSensorCore
    TheListOfSensorCore.clear();


    // Log
    if(logFile.is_open())
    {
        logFile.close();
    }


    return;
}


int MsfLocalizationCore::init()
{
    return 0;
}

int MsfLocalizationCore::close()
{
    return 0;
}

int MsfLocalizationCore::open()
{
    return 0;
}

int MsfLocalizationCore::run()
{
    return 0;
}

int MsfLocalizationCore::setStateEstimationEnabled(bool predictEnabled)
{
    // Check
    if(this->stateEstimationEnabled==predictEnabled)
        return 0;

#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::setStateEstimationEnabled()"<<std::endl;
        this->log(logString.str());
    }
#endif

    // Clear the buffer except the last state estimation that will be used as init state
    TimeStamp InitTimeStamp;
    std::shared_ptr<StateEstimationCore> InitState;
    if(TheMsfStorageCore->getLastElementWithStateEstimate(InitTimeStamp, InitState))
    {
#if _DEBUG_MSF_LOCALIZATION_CORE
        logFile<<"MsfLocalizationCore::setStateEstimationEnabled() error in getLastElementWithStateEstimate"<<std::endl;
#endif
        return 2;
    }

    if(TheMsfStorageCore->purgeRingBuffer(-1))
    {
#if _DEBUG_MSF_LOCALIZATION_CORE
        logFile<<"MsfLocalizationCore::setStateEstimationEnabled() error in purgeRingBuffer"<<std::endl;
#endif
        return -2;
    }


    // Set the init time stamp and init state
    InitTimeStamp=getTimeStamp();
    if(TheMsfStorageCore->addElement(InitTimeStamp, InitState))
    {
#if _DEBUG_MSF_LOCALIZATION_CORE
        logFile<<"MsfLocalizationCore::setStateEstimationEnabled() error in addElement"<<std::endl;
#endif
        return -3;
    }


    // Change Flag in the core
    this->stateEstimationEnabled=predictEnabled;

    // Change flag in the sensors
    for(std::list< std::shared_ptr<SensorCore> >::iterator itListOfSensors=TheListOfSensorCore.begin();
        itListOfSensors!=TheListOfSensorCore.end();
        ++itListOfSensors)
    {
        (*itListOfSensors)->setSensorEnabled(predictEnabled);
    }

#if _DEBUG_MSF_LOCALIZATION_CORE
    logFile<<"MsfLocalizationCore::setStateEstimationEnabled() ended"<<std::endl;
#endif


    return 0;
}

bool MsfLocalizationCore::isStateEstimationEnabled() const
{
    return this->stateEstimationEnabled;
}

TimeStamp MsfLocalizationCore::getTimeStamp()
{
#if _DEBUG_MSF_LOCALIZATION_CORE
    std::cout<<"MsfLocalizationCore::getTimeStamp()"<<std::endl;
#endif

    TimeStamp TheTimeStamp;
    return TheTimeStamp;
}

int MsfLocalizationCore::predictThreadFunction()
{
#if _DEBUG_MSF_LOCALIZATION_CORE
    logFile<<"MsfLocalizationCore::predictThreadFunction()"<<std::endl;
#endif

    // TODO Finish

    return 0;
}





int MsfLocalizationCore::bufferManagerThreadFunction()
{

    // TODO Finish

    return 0;
}


int MsfLocalizationCore::getPreviousState(TimeStamp TheTimeStamp, TimeStamp& ThePreviousTimeStamp, std::shared_ptr<StateEstimationCore>& ThePreviousState)
{
    // Get the previous not the last!
    if(TheMsfStorageCore->getPreviousElementWithStateEstimateByStamp(TheTimeStamp, ThePreviousTimeStamp, ThePreviousState))
    {
#if _DEBUG_MSF_LOCALIZATION_CORE
        logFile<<"MsfLocalizationCore::getPreviousState() error getPreviousElementWithStateEstimateByStamp"<<std::endl;
#endif
        return 1;
    }

    // Checks
    if(!ThePreviousState)
    {
#if _DEBUG_MSF_LOCALIZATION_CORE
        logFile<<"MsfLocalizationCore::getPreviousState() error !PreviousState"<<std::endl;
#endif
        return 2;
    }


    // Check
    if(!ThePreviousState->hasState())
    {
#if _DEBUG_MSF_LOCALIZATION_CORE
        logFile<<"MsfLocalizationCore::getPreviousState() error !PreviousState->hasState()"<<std::endl;
#endif
        return 3;
    }

    return 0;
}


int MsfLocalizationCore::predict(TimeStamp TheTimeStamp)
{
    if(!isStateEstimationEnabled())
        return 0;

#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::predict() TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
        this->log(logString.str());
    }
#endif


    // Get the predicted element if any
    std::shared_ptr<StateEstimationCore> PredictedState;
    if(this->TheMsfStorageCore->getElement(TheTimeStamp, PredictedState))
    {
#if _DEBUG_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationROS::predict() no predicted element found, must be created one!"<<std::endl;
            this->log(logString.str());
        }

#endif
        //return -10;
    }


    // New element that is going to be added to the buffer
    if(!PredictedState)
        PredictedState=std::make_shared<StateEstimationCore>();

#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"predict pre: number of users of predicted state="<<PredictedState.use_count()<<std::endl;
        this->log(logString.str());
    }
#endif

    while(PredictedState.use_count()>2)
    {
        // Do nothig. Sleep a little
        // TODO optimize this!
        std::this_thread::sleep_for( std::chrono::nanoseconds( 50 ) );

#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"predict: number of users of predicted state="<<PredictedState.use_count()<<std::endl;
        this->log(logString.str());
    }
#endif

    }

#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"predict post: number of users of predicted state="<<PredictedState.use_count()<<std::endl;
        this->log(logString.str());
    }
#endif

    // Get the last state from the buffer
    TimeStamp PreviousTimeStamp;
    std::shared_ptr<StateEstimationCore> PreviousState;

    if(this->getPreviousState(TheTimeStamp, PreviousTimeStamp, PreviousState))
    {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
        logFile<<"MsfLocalizationCore::predict() error getPreviousState"<<std::endl;
#endif
        return 1;
    }


    if(!PreviousState)
    {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
        logFile<<"MsfLocalizationCore::predict() error !PreviousState"<<std::endl;
#endif
        return 2;
    }


    // Check
    if(!PreviousState->hasState())
    {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
        logFile<<"MsfLocalizationCore::predict() error !PreviousState->hasState()"<<std::endl;
#endif
        return -1;
    }

//    // Check
//    if(PreviousState->covarianceMatrix.size()==0)
//    {
//        std::cout<<"!!covariance matrix of the previous state is not set"<<std::endl;
//        return -2;
//    }

#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::predict() TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
        logString<<"MsfLocalizationCore::predict() TS prev: sec="<<PreviousTimeStamp.sec<<" s; nsec="<<PreviousTimeStamp.nsec<<" ns"<<std::endl;
        this->log(logString.str());
    }
#endif


    /////// State
#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::predict() state TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
        this->log(logString.str());
    }
#endif

    // Dimension of the state
    //unsigned int dimensionOfState=0;
    //unsigned int dimensionOfErrorState=0;


    ///// Global Parameters

    // TODO Improve
    std::shared_ptr<GlobalParametersStateCore> predictedStateGlobalParameters=std::make_shared<GlobalParametersStateCore>(PreviousState->TheGlobalParametersStateCore->getTheGlobalParametersCore());


    // Copy the same values
    predictedStateGlobalParameters->setGravity(PreviousState->TheGlobalParametersStateCore->getGravity());


    // Add
    PredictedState->TheGlobalParametersStateCore=predictedStateGlobalParameters;




    ///// Robot

    // Dimension of state
    //dimensionOfState+=TheRobotCore->getDimensionState();
    // Dimension of error state
    //dimensionOfErrorState+=TheRobotCore->getDimensionErrorState();

    // Robot Type
    switch(TheRobotCore->getRobotType())
    {
        case RobotTypes::free_model:
        {
            std::shared_ptr<FreeModelRobotStateCore> predictedStateRobot;
            std::shared_ptr<FreeModelRobotStateCore> pastStateRobot=std::static_pointer_cast<FreeModelRobotStateCore>(PreviousState->TheRobotStateCore);

            if(!pastStateRobot)
            {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
                std::cout<<"!!previous state for robot not found!"<<std::endl;
#endif
                return -2;
            }


            // Polymorphic
            std::shared_ptr<FreeModelRobotCore> TheFreeModelRobotCore=std::dynamic_pointer_cast<FreeModelRobotCore>(TheRobotCore);

            // State
            if(TheFreeModelRobotCore->predictState(PreviousTimeStamp, TheTimeStamp, pastStateRobot, predictedStateRobot))
            {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
                std::cout<<"!!Error predicting state of the robot"<<std::endl;
#endif
                return 1;
            }

            // Jacobians
            if(TheFreeModelRobotCore->predictStateErrorStateJacobians(PreviousTimeStamp, TheTimeStamp, pastStateRobot, predictedStateRobot))
            {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
                std::cout<<"!!Error predicting error state jacobians of the robot"<<std::endl;
#endif
                return 1;
            }


            // Add
            PredictedState->TheRobotStateCore=predictedStateRobot;

            // End
            break;
        }

        default:
        {
            return -1000;
            break;
        }
    }


    ///// Sensors

    // Clean the list
    PredictedState->TheListSensorStateCore.clear();

    // Iterate
    for(std::list< std::shared_ptr<SensorCore> >::iterator itSens=TheListOfSensorCore.begin();
        itSens!=TheListOfSensorCore.end();
        ++itSens)
    {
        // Dimension of state
        //dimensionOfState+=(*itSens)->getDimensionState();
        // Dimension of the error state
        //dimensionOfErrorState+=(*itSens)->getDimensionErrorState();


        // Auxiliar
        std::shared_ptr<SensorStateCore> pastStateSensor;

        // Find
        if(findSensorStateCoreFromList(PreviousState->TheListSensorStateCore, (*itSens), pastStateSensor))
        {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
            logFile<<"MsfLocalizationCore::predict() error predict state sensors findSensorStateCoreFromList"<<std::endl;
#endif
            return -2;
        }


        // Switch according to the sensor type
        switch((*itSens)->getSensorType())
        {
            /// IMU
            case SensorTypes::imu:
            {
                //Auxiliar
                std::shared_ptr<ImuSensorStateCore> predictedImuStateSensor;

                // Polymorphic
                std::shared_ptr<ImuSensorCore> TheImuSensorCore=std::dynamic_pointer_cast<ImuSensorCore>(*itSens);
                std::shared_ptr<ImuSensorStateCore> pastImuStateSensor=std::static_pointer_cast<ImuSensorStateCore>(pastStateSensor);

                // State
                if(TheImuSensorCore->predictState(PreviousTimeStamp, TheTimeStamp, pastImuStateSensor, predictedImuStateSensor))
                {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
                    std::cout<<"!!Error predicting state of sensor"<<std::endl;
#endif
                    return 1;
                }

                // Jacobians
                if(TheImuSensorCore->predictStateErrorStateJacobians(PreviousTimeStamp, TheTimeStamp, pastImuStateSensor, predictedImuStateSensor))
                {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
                    std::cout<<"!!Error predicting error state jacobians of the sensor"<<std::endl;
#endif
                    return 1;
                }

                // Add
                PredictedState->TheListSensorStateCore.push_back(predictedImuStateSensor);

                // End
                break;
            }

            default:
            {
                return -1000;
                break;
            }

        }


        // OTHER SENSORS
        // TODO

    }


    ///// Map
    // TODO



    /////// Covariances
#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::predict() covariances TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
        this->log(logString.str());
    }
#endif

    // Auxiliar variables
//    unsigned int previousDimensionErrorState_col;
//    unsigned int previousDimensionErrorState_row;

    // Dimension
    MatrixPoint WorkingInitPoint;


    // Delta Time Stamp
    TimeStamp DeltaTime=TheTimeStamp-PreviousTimeStamp;


//    std::cout<<"Delta TS: sec="<<DeltaTime.sec<<" s; nsec="<<DeltaTime.nsec<<" ns"<<std::endl;
//    std::cout<<" ->dt="<<DeltaTime.get_double()<<std::endl;


    // Resize the covariance Matrix
    int dimensionOfErrorState=PredictedState->getDimensionErrorState();
    PredictedState->covarianceMatrix.resize(dimensionOfErrorState,dimensionOfErrorState);
    PredictedState->covarianceMatrix.setZero();


    /// Robot
#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::predict() covariance robot TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
        this->log(logString.str());
    }
#endif

    switch(TheRobotCore->getRobotType())
    {
        case RobotTypes::free_model:
        {
            // Covariance
            if(predictedFreeModelRobotCovariance(DeltaTime, std::static_pointer_cast<FreeModelRobotCore>(TheRobotCore), std::static_pointer_cast<FreeModelRobotStateCore>(PredictedState->TheRobotStateCore), &PreviousState->covarianceMatrix, &PredictedState->covarianceMatrix))
            {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
                std::cout<<"!!Error predicting covariances"<<std::endl;
#endif
                return -2;
            }

            // End
            break;
        }
        default:
        {
            return -1000;
            break;
        }
    }


    /// Robot-Sensors
#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::predict() covariance robot-sensor TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
        this->log(logString.str());
    }
#endif


    // Dimensions
    MatrixPoint RobotSensorsInitPoint;
    MatrixPoint RobotSensorsEndPoint;

    // Init Point
    RobotSensorsInitPoint.col=TheRobotCore->getDimensionErrorState();
    RobotSensorsInitPoint.row=0;

    // Working Point
    WorkingInitPoint=RobotSensorsInitPoint;


    switch(TheRobotCore->getRobotType())
    {
        case RobotTypes::free_model:
        {

            // Dimension
            //previousDimensionErrorState_row=0;
            //previousDimensionErrorState_col=TheRobotCore->getDimensionErrorState();


            // Iterate on the sensors
            for(std::list< std::shared_ptr<SensorCore> >::iterator it1Sens=TheListOfSensorCore.begin();
                it1Sens!=TheListOfSensorCore.end();
                ++it1Sens)
            {

                // Auxiliar
                std::shared_ptr<SensorStateCore> predictedStateSensor1;

                // Find
                if(findSensorStateCoreFromList(PredictedState->TheListSensorStateCore, (*it1Sens), predictedStateSensor1))
                {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
                    std::cout<<"!!Error findSensorStateCoreFromList"<<std::endl;
#endif
                    return -2;
                }


                // Switch type of sensor it1Sens
                switch((*it1Sens)->getSensorType())
                {
                    /// Imu
                    case SensorTypes::imu:
                    {
                        // Covariances update
                        if(predictedFreeModelRobotImuCovariance(DeltaTime, std::dynamic_pointer_cast<FreeModelRobotCore>(TheRobotCore), std::dynamic_pointer_cast<ImuSensorCore>(*it1Sens),
                                                  std::static_pointer_cast<FreeModelRobotStateCore>(PredictedState->TheRobotStateCore), std::static_pointer_cast<ImuSensorStateCore>(predictedStateSensor1),
                                                  &PreviousState->covarianceMatrix, WorkingInitPoint, &PredictedState->covarianceMatrix, RobotSensorsEndPoint))
                        {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
                            std::cout<<"!!Error predicting covariances"<<std::endl;
#endif
                            return 2;
                        }

                        // End
                        break;
                    }
                    default:
                    {
                        return -1000;
                        break;
                    }
                }

                // Update Point
                WorkingInitPoint.col=RobotSensorsEndPoint.col;
            }

            // End
            break;
        }
        default:
        {
            return -1000;
            break;
        }
    }



    /// Sensors
#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::predict() covariance sensors TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
        this->log(logString.str());
    }
#endif

    // Dimension
    MatrixPoint SensorsInitPoint;
    MatrixPoint SensorsEndPoint;


    // Init Point
    SensorsInitPoint.col=TheRobotCore->getDimensionErrorState();
    SensorsInitPoint.row=SensorsInitPoint.col;

    // Working Point
    WorkingInitPoint=SensorsInitPoint;


    // Iterate
    for(std::list< std::shared_ptr<SensorCore> >::iterator it1Sens=TheListOfSensorCore.begin();
        it1Sens!=TheListOfSensorCore.end();
        ++it1Sens)
    {
        // it1Sens Auxiliar
        std::shared_ptr<SensorStateCore> predictedStateSensor1;

        // Find
        if(findSensorStateCoreFromList(PredictedState->TheListSensorStateCore, (*it1Sens), predictedStateSensor1))
        {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
            std::cout<<"!!Error findSensorStateCoreFromList"<<std::endl;
#endif
            return -2;
        }


        // Iterate again
        for(std::list< std::shared_ptr<SensorCore> >::iterator it2Sens=it1Sens;
            it2Sens!=TheListOfSensorCore.end();
            ++it2Sens)
        {
            // it2Sens Auxiliar
            std::shared_ptr<SensorStateCore> predictedStateSensor2;

            // Find
            if(findSensorStateCoreFromList(PredictedState->TheListSensorStateCore, (*it2Sens), predictedStateSensor2))
            {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
                std::cout<<"!!Error findSensorStateCoreFromList"<<std::endl;
#endif
                return -2;
            }


            // Switch type of sensor it1Sens
            switch((*it1Sens)->getSensorType())
            {
                /// Imu
                case SensorTypes::imu:
                {
                    // Switch type of sensor it2Sens
                    switch((*it2Sens)->getSensorType())
                    {
                        /// Imu
                        case SensorTypes::imu:
                        {
                            // Do the covariance update
                            if(predictedImuImuCovariance(DeltaTime, std::dynamic_pointer_cast<ImuSensorCore>(*it1Sens), std::dynamic_pointer_cast<ImuSensorCore>(*it2Sens),
                                                      std::static_pointer_cast<ImuSensorStateCore>(predictedStateSensor1), std::static_pointer_cast<ImuSensorStateCore>(predictedStateSensor2),
                                                      &PreviousState->covarianceMatrix, WorkingInitPoint, &PredictedState->covarianceMatrix, SensorsEndPoint))
                            {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
                                std::cout<<"!!Error predicting covariances"<<std::endl;
#endif
                                return 2;
                            }

                            // End
                            break;
                        }
                        default:
                        {
                            return -1000;
                            break;
                        }
                    }
                    // End
                    break;
                }
                default:
                {
                    return -1000;
                    break;
                }
            }

            // Update Working Point
            WorkingInitPoint.col=SensorsEndPoint.col;

        }

        // Update the dimension for the next sensor
        WorkingInitPoint.row=SensorsEndPoint.row;
        WorkingInitPoint.col=WorkingInitPoint.row;

    }


    /// Robot-Map
    // TODO

    /// Sensors-Map
    // TODO

    /// Map
    // TODO


    /// Display
    //logFile<<"Covariances Matrix of the predicted state="<<std::endl;
    //logFile<<PredictedState->covarianceMatrix<<std::endl;



    /////// Add element to the buffer
    if(TheMsfStorageCore->addElement(TheTimeStamp, PredictedState))
    {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
        std::cout<<"!!Error addElement"<<std::endl;
#endif
        return -2;
    }

#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::predict() ended TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
        this->log(logString.str());
    }
#endif

    // End
    return 0;
}



int MsfLocalizationCore::update(TimeStamp TheTimeStamp)
{
#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::update() TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;

        this->log(logString.str());
    }
#endif

    // Get
    std::shared_ptr<StateEstimationCore> UpdatedState;


    // Get the outdated element to do the update
    if(this->TheMsfStorageCore->getElement(TheTimeStamp, UpdatedState))
    {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::update() error 11!"<<std::endl;
            this->log(logString.str());
        }

#endif
        return -11;
    }

    // Check
    if(!UpdatedState)
    {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::update() error2!"<<std::endl;
            this->log(logString.str());
        }

#endif
        return -11;
    }



#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"number of users of updated state="<<UpdatedState.use_count()<<std::endl;

        this->log(logString.str());
    }
#endif


    while(UpdatedState.use_count()>2)
    {
        // Do nothig. Sleep a little
        // TODO optimize this!
        std::this_thread::sleep_for( std::chrono::nanoseconds( 50 ) );
    }

#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"number of users of updated state="<<UpdatedState.use_count()<<std::endl;
        this->log(logString.str());
    }
#endif


    // Check if there are measurements
    if(UpdatedState->TheListMeasurementCore.size()==0)
    {
#if _DEBUG_MSF_LOCALIZATION_CORE
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::update() ended without measurements TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
        this->log(logString.str());
#endif
        return 0;
    }


    // Checks
    // TODO finish

    // Updated State
    if(!UpdatedState)
    {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
        std::cout<<"MsfLocalizationCore::update() error 1"<<std::endl;
#endif
        return 1;
    }

    if(!UpdatedState->TheRobotStateCore)
    {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
        std::cout<<"MsfLocalizationCore::update() error 11"<<std::endl;
#endif
        return 11;
    }


    // TODO No!
    if(UpdatedState->TheListSensorStateCore.size()==0)
    {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
        std::cout<<"MsfLocalizationCore::update() error 13"<<std::endl;
#endif
        return 13;
    }




    ///// Measurement prediction
    std::list<std::shared_ptr<SensorMeasurementCore> > TheListPredictedMeasurements;
    // TODO
    std::list<std::shared_ptr<SensorMeasurementCore> > TheListMatchedMeasurements;
    std::list<std::shared_ptr<SensorMeasurementCore> > TheListUnmatchedMeasurements;

    for(std::list<std::shared_ptr<SensorMeasurementCore> >::iterator itListMeas=UpdatedState->TheListMeasurementCore.begin();
        itListMeas!=UpdatedState->TheListMeasurementCore.end();
        ++itListMeas)
    {
        // Check
        if(!(*itListMeas)->getTheSensorCore())
        {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
            std::cout<<"MsfLocalizationCore::update() error 2"<<std::endl;
#endif
            return 2;
        }

        // Find the type
        switch((*itListMeas)->getTheSensorCore()->getSensorType())
        {
            case SensorTypes::imu:
            {
                // Cast the imu sensor core
                std::shared_ptr<ImuSensorCore> TheImuSensorCore=std::dynamic_pointer_cast<ImuSensorCore>((*itListMeas)->getTheSensorCore());


                // Find the sensor state
                std::shared_ptr<SensorStateCore> TheSensorStateCore;
                if(findSensorStateCoreFromList(UpdatedState->TheListSensorStateCore, (*itListMeas)->getTheSensorCore(), TheSensorStateCore))
                {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
                    std::cout<<"MsfLocalizationCore::update() error 4"<<std::endl;
#endif
                    return 4;
                }
                if(!TheSensorStateCore)
                {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
                    std::cout<<"MsfLocalizationCore::update() error 5"<<std::endl;
#endif
                    return 5;
                }

                // Cast the imu sensor state
                std::shared_ptr<ImuSensorStateCore> TheImuSensorStateCore=std::static_pointer_cast<ImuSensorStateCore>(TheSensorStateCore);;


                // Create a pointer
                std::shared_ptr<ImuSensorMeasurementCore> TheImuSensorPredictedMeasurement;

                // Call measurement prediction
                if(TheImuSensorCore->predictMeasurement(TheTimeStamp, UpdatedState->TheGlobalParametersStateCore, UpdatedState->TheRobotStateCore, TheImuSensorStateCore, TheImuSensorPredictedMeasurement))
                {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
                    std::cout<<"MsfLocalizationCore::update() error 3"<<std::endl;
#endif
                    return 3;
                }


                // Compute Jacobians of the Measurement
                if(TheImuSensorCore->jacobiansMeasurements(TheTimeStamp, UpdatedState->TheGlobalParametersStateCore, UpdatedState->TheRobotStateCore, TheImuSensorStateCore, TheImuSensorPredictedMeasurement))
                {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
                    std::cout<<"MsfLocalizationCore::update() error 3"<<std::endl;
#endif
                    return 3;
                }


                // Push to the predicted measurements
                TheListPredictedMeasurements.push_back(TheImuSensorPredictedMeasurement);



                // Push to the Matched Measurements -> The imu measurement is always matched
                TheListMatchedMeasurements.push_back((*itListMeas));



                // End
                break;
            }
            default:
            {
                return -1000;
                break;
            }

        }


    }



    ///////// Get dimensions
    // Measurements
    unsigned int dimensionMeasurements=0;
    for(std::list<std::shared_ptr<SensorMeasurementCore> >::const_iterator itListMatchedMeas=TheListMatchedMeasurements.begin();
        itListMatchedMeas!=TheListMatchedMeasurements.end();
        ++itListMatchedMeas)
    {
        dimensionMeasurements+=(*itListMatchedMeas)->getTheSensorCore()->getDimensionMeasurement();
    }

    // Error State
    unsigned int dimensionErrorState=0;
    dimensionErrorState+=UpdatedState->TheRobotStateCore->getTheRobotCore()->getDimensionErrorState();
    dimensionErrorState+=UpdatedState->TheGlobalParametersStateCore->getTheGlobalParametersCore()->getDimensionErrorState();
    for(std::list<std::shared_ptr<SensorMeasurementCore> >::const_iterator itListMatchedMeas=TheListMatchedMeasurements.begin();
        itListMatchedMeas!=TheListMatchedMeasurements.end();
        ++itListMatchedMeas)
    {
        dimensionErrorState+=(*itListMatchedMeas)->getTheSensorCore()->getDimensionErrorState();
    }


    // Global Parameters
    unsigned int dimensionGlobalParameters=0;
    dimensionGlobalParameters=UpdatedState->TheGlobalParametersStateCore->getTheGlobalParametersCore()->getDimensionErrorParameters();

    // Sensor Parameters
    unsigned int dimensionSensorParameters=0;
    for(std::list<std::shared_ptr<SensorMeasurementCore> >::const_iterator itListMatchedMeas=TheListMatchedMeasurements.begin();
        itListMatchedMeas!=TheListMatchedMeasurements.end();
        ++itListMatchedMeas)
    {
        dimensionSensorParameters+=(*itListMatchedMeas)->getTheSensorCore()->getDimensionErrorParameters();
    }




    ///// Innovation vector

    // Vector
    Eigen::VectorXd innovationVector;
    innovationVector.resize(dimensionMeasurements, 1);
    innovationVector.setZero();

    // Fill
    {
        unsigned int dimension=0;
        for(std::list<std::shared_ptr<SensorMeasurementCore> >::const_iterator itListMatchedMeas=TheListMatchedMeasurements.begin(), itListPredictedMeas=TheListPredictedMeasurements.begin();
            itListMatchedMeas!=TheListMatchedMeasurements.end() && itListPredictedMeas!=TheListMatchedMeasurements.end();
            ++itListMatchedMeas, ++itListPredictedMeas)
        {
            unsigned int dimensionMeasurementSensorI=(*itListMatchedMeas)->getTheSensorCore()->getDimensionMeasurement();

            innovationVector.block(dimension, 0, dimensionMeasurementSensorI, 1)=(*itListMatchedMeas)->getMeasurement()-(*itListPredictedMeas)->getMeasurement();

            dimension+=dimensionMeasurementSensorI;
        }
    }


#if 0 || _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::update() Innovation vector for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
        logString<<innovationVector.transpose()<<std::endl;
        this->log(logString.str());
    }
#endif



    //// Total Jacobians Measurement

    // Jacobian Measurement - Error State
    Eigen::MatrixXd JacobianMeasurementErrorState;
    JacobianMeasurementErrorState.resize(dimensionMeasurements, dimensionErrorState);
    JacobianMeasurementErrorState.setZero();

    // Fill
    {
        unsigned int dimensionTotalMeasurementI=0;
        for(std::list<std::shared_ptr<SensorMeasurementCore> >::const_iterator itListPredictedMeas=TheListPredictedMeasurements.begin();
            itListPredictedMeas!=TheListPredictedMeasurements.end();
            ++itListPredictedMeas)
        {
            unsigned int dimensionMeasurementI=(*itListPredictedMeas)->getTheSensorCore()->getDimensionMeasurement();
            unsigned int dimensionTotalErrorStateI=0;
            unsigned int dimensionErrorStateI=0;

            // Robot
            dimensionErrorStateI=this->TheRobotCore->getDimensionErrorState();
            if(dimensionErrorStateI)
                JacobianMeasurementErrorState.block(dimensionTotalMeasurementI, dimensionTotalErrorStateI, dimensionMeasurementI, dimensionErrorStateI)=(*itListPredictedMeas)->jacobianMeasurementErrorState.jacobianMeasurementRobotErrorState;
            dimensionTotalErrorStateI+=dimensionErrorStateI;

            // Global Parameters
            dimensionErrorStateI=this->TheGlobalParametersCore->getDimensionErrorState();
            if(dimensionErrorStateI)
                JacobianMeasurementErrorState.block(dimensionTotalMeasurementI, dimensionTotalErrorStateI, dimensionMeasurementI, dimensionErrorStateI)=(*itListPredictedMeas)->jacobianMeasurementErrorState.jacobianMeasurementGlobalParametersErrorState;
            dimensionTotalErrorStateI+=dimensionErrorStateI;

            // Sensors
            for(std::list< std::shared_ptr<SensorCore> >::const_iterator itListSensorCore=this->TheListOfSensorCore.begin();
                itListSensorCore!=this->TheListOfSensorCore.end();
                ++itListSensorCore)
            {
                dimensionErrorStateI=(*itListSensorCore)->getDimensionErrorState();
                if((*itListSensorCore) == (*itListPredictedMeas)->getTheSensorCore())
                {
                    if(dimensionErrorStateI)
                        JacobianMeasurementErrorState.block(dimensionTotalMeasurementI, dimensionTotalErrorStateI, dimensionMeasurementI, dimensionErrorStateI)=(*itListPredictedMeas)->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState;
                }
                else
                {
                    // Do nothing -> Set Zeros (already set)
                }
                dimensionTotalErrorStateI+=dimensionErrorStateI;
            }


            // Map
            // TODO


            // Update dimension
            dimensionTotalMeasurementI+=dimensionMeasurementI;
        }
    }



    // Jacobian Measurement - Measurement Noise
    Eigen::MatrixXd JacobianMeasurementNoise;
    JacobianMeasurementNoise.resize(dimensionMeasurements, dimensionMeasurements);
    JacobianMeasurementNoise.setZero();

    // Fill
    {
        unsigned int dimensionTotalMeasurementI=0;
        for(std::list<std::shared_ptr<SensorMeasurementCore> >::const_iterator itListPredictedMeas=TheListPredictedMeasurements.begin();
            itListPredictedMeas!=TheListPredictedMeasurements.end();
            ++itListPredictedMeas)
        {
            unsigned int dimensionMeasurementI=(*itListPredictedMeas)->getTheSensorCore()->getDimensionMeasurement();
            unsigned int dimensionTotalMeasurementJ=0;

            for(std::list< std::shared_ptr<SensorMeasurementCore> >::const_iterator itListMatchedMeas=TheListMatchedMeasurements.begin();
                itListMatchedMeas!=TheListMatchedMeasurements.end();
                ++itListMatchedMeas)
            {
                unsigned int dimensionMeasurementJ=(*itListMatchedMeas)->getTheSensorCore()->getDimensionMeasurement();
                if((*itListMatchedMeas)->getTheSensorCore() == (*itListPredictedMeas)->getTheSensorCore())
                {
                    if(dimensionMeasurementJ)
                        JacobianMeasurementNoise.block(dimensionTotalMeasurementI, dimensionTotalMeasurementJ, dimensionMeasurementI, dimensionMeasurementJ)=(*itListPredictedMeas)->jacobianMeasurementSensorNoise.jacobianMeasurementSensorNoise;
                }
                else
                {
                    // Do nothing -> Set Zeros (already set)
                }
                // Update dimension
                dimensionTotalMeasurementJ+=dimensionMeasurementJ;
            }
            // Update dimension
            dimensionTotalMeasurementI+=dimensionMeasurementI;
        }
    }



    // Jacobian Measurement - Global Parameters
    Eigen::MatrixXd JacobianMeasurementGlobalParameters;
    JacobianMeasurementGlobalParameters.resize(dimensionMeasurements, dimensionGlobalParameters);
    JacobianMeasurementGlobalParameters.setZero();

    // Fill
    {
        unsigned int dimensionTotalMeasurementI=0;
        for(std::list<std::shared_ptr<SensorMeasurementCore> >::const_iterator itListPredictedMeas=TheListPredictedMeasurements.begin();
            itListPredictedMeas!=TheListPredictedMeasurements.end();
            ++itListPredictedMeas)
        {
            unsigned int dimensionMeasurementI=(*itListPredictedMeas)->getTheSensorCore()->getDimensionMeasurement();
            unsigned int dimensionGlobalParametersI=this->TheGlobalParametersCore->getDimensionErrorParameters();
            JacobianMeasurementNoise.block(dimensionTotalMeasurementI, 0, dimensionMeasurementI, dimensionGlobalParametersI)=(*itListPredictedMeas)->jacobianMeasurementGlobalParameters.jacobianMeasurementGlobalParameters;
            dimensionTotalMeasurementI+=dimensionMeasurementI;
        }
    }



    // Jacobian Measurement - Ohter Parameters (Robot Parameters)
    // TODO


    // Jacobian Measurement - Sensor Parameters
    Eigen::MatrixXd JacobianMeasurementSensorParameters;
    JacobianMeasurementSensorParameters.resize(dimensionMeasurements, dimensionSensorParameters);
    JacobianMeasurementSensorParameters.setZero();

    // Fill
    {
        unsigned int dimensionTotalMeasurementI=0;
        for(std::list<std::shared_ptr<SensorMeasurementCore> >::const_iterator itListPredictedMeas=TheListPredictedMeasurements.begin();
            itListPredictedMeas!=TheListPredictedMeasurements.end();
            ++itListPredictedMeas)
        {
            unsigned int dimensionMeasurementI=(*itListPredictedMeas)->getTheSensorCore()->getDimensionMeasurement();
            unsigned int dimensionTotalSensorParametersJ=0;

            for(std::list< std::shared_ptr<SensorMeasurementCore> >::const_iterator itListMatchedMeas=TheListMatchedMeasurements.begin();
                itListMatchedMeas!=TheListMatchedMeasurements.end();
                ++itListMatchedMeas)
            {
                unsigned int dimensionSensorParametersJ=(*itListMatchedMeas)->getTheSensorCore()->getDimensionErrorParameters();
                if((*itListMatchedMeas)->getTheSensorCore() == (*itListPredictedMeas)->getTheSensorCore())
                {
                    if(dimensionSensorParametersJ)
                        JacobianMeasurementNoise.block(dimensionTotalMeasurementI, dimensionTotalSensorParametersJ, dimensionMeasurementI, dimensionSensorParametersJ)=(*itListPredictedMeas)->jacobianMeasurementSensorParameters.jacobianMeasurementSensorParameters;
                }
                else
                {
                    // Do nothing -> Set Zeros (already set)
                }
                // Update dimension
                dimensionTotalSensorParametersJ+=dimensionSensorParametersJ;
            }
            // Update dimension
            dimensionTotalMeasurementI+=dimensionMeasurementI;
        }
    }




    ///// Noises and covariances

    // Covariance Measurement
    Eigen::MatrixXd CovarianceMeasurement;
    CovarianceMeasurement.resize(dimensionMeasurements, dimensionMeasurements);
    CovarianceMeasurement.setZero();

    // Fill
    {
        unsigned int dimensionTotalMeasurementI=0;
        for(std::list<std::shared_ptr<SensorMeasurementCore> >::const_iterator itListPredictedMeas=TheListPredictedMeasurements.begin();
            itListPredictedMeas!=TheListPredictedMeasurements.end();
            ++itListPredictedMeas)
        {
            unsigned int dimensionMeasurementI=(*itListPredictedMeas)->getTheSensorCore()->getDimensionMeasurement();
            unsigned int dimensionTotalMeasurementJ=0;

            for(std::list< std::shared_ptr<SensorMeasurementCore> >::const_iterator itListMatchedMeas=TheListMatchedMeasurements.begin();
                itListMatchedMeas!=TheListMatchedMeasurements.end();
                ++itListMatchedMeas)
            {
                unsigned int dimensionMeasurementJ=(*itListMatchedMeas)->getTheSensorCore()->getDimensionMeasurement();
                if((*itListMatchedMeas)->getTheSensorCore() == (*itListPredictedMeas)->getTheSensorCore())
                {
                    if(dimensionMeasurementJ)
                        CovarianceMeasurement.block(dimensionTotalMeasurementI, dimensionTotalMeasurementJ, dimensionMeasurementI, dimensionMeasurementJ)=(*itListPredictedMeas)->getTheSensorCore()->getCovarianceMeasurement();
                }
                else
                {
                    // Do nothing -> Set Zeros (already set)
                }
                // Update dimension
                dimensionTotalMeasurementJ+=dimensionMeasurementJ;
            }
            // Update dimension
            dimensionTotalMeasurementI+=dimensionMeasurementI;
        }
    }



    // Covariance Global Parameters
    Eigen::MatrixXd CovarianceGlobalParameters;
    CovarianceGlobalParameters.resize(dimensionGlobalParameters, dimensionGlobalParameters);
    CovarianceGlobalParameters.setZero();

    // Fill
    CovarianceGlobalParameters=this->TheGlobalParametersCore->getCovarianceGlobalParameters();




    // Covariance Other Parameters (Robot Parameters)
    // TODO



    // Covariance Sensor Parameters
    Eigen::MatrixXd CovarianceSensorParameters;
    CovarianceSensorParameters.resize(dimensionSensorParameters, dimensionSensorParameters);
    CovarianceSensorParameters.setZero();

    // Fill
    {
        unsigned int dimensionTotalSensorParametersI=0;
        for(std::list<std::shared_ptr<SensorMeasurementCore> >::const_iterator itListMatchedMeasI=TheListMatchedMeasurements.begin();
            itListMatchedMeasI!=TheListMatchedMeasurements.end();
            ++itListMatchedMeasI)
        {
            unsigned int dimensionSensorParametersI=(*itListMatchedMeasI)->getTheSensorCore()->getDimensionErrorParameters();
            unsigned int dimensionTotalSensorParametersJ=0;

            for(std::list< std::shared_ptr<SensorMeasurementCore> >::const_iterator itListMatchedMeasJ=TheListMatchedMeasurements.begin();
                itListMatchedMeasJ!=TheListMatchedMeasurements.end();
                ++itListMatchedMeasJ)
            {
                unsigned int dimensionSensorParametersJ=(*itListMatchedMeasJ)->getTheSensorCore()->getDimensionErrorParameters();
                if((*itListMatchedMeasI)->getTheSensorCore() == (*itListMatchedMeasJ)->getTheSensorCore())
                {
                    if(dimensionSensorParametersJ)
                        CovarianceSensorParameters.block(dimensionTotalSensorParametersI, dimensionTotalSensorParametersJ, dimensionSensorParametersI, dimensionSensorParametersJ)=(*itListMatchedMeasI)->getTheSensorCore()->getCovarianceParameters();
                }
                else
                {
                    // Do nothing -> Set Zeros (already set)
                }
                // Update dimension
                dimensionTotalSensorParametersJ+=dimensionSensorParametersJ;
            }
            // Update dimension
            dimensionTotalSensorParametersI+=dimensionSensorParametersI;
        }
    }






    ///// Innovation (or residual) covariance: S

    // Matrix
    Eigen::MatrixXd innovationCovariance;

    // Equation
    innovationCovariance=JacobianMeasurementErrorState*UpdatedState->covarianceMatrix*JacobianMeasurementErrorState.transpose()+
            JacobianMeasurementNoise*CovarianceMeasurement*JacobianMeasurementNoise.transpose()+
            JacobianMeasurementGlobalParameters*CovarianceGlobalParameters*JacobianMeasurementGlobalParameters.transpose()+
            JacobianMeasurementSensorParameters*CovarianceSensorParameters*JacobianMeasurementSensorParameters.transpose();

    Eigen::MatrixXd innovation_covariance_inverse=innovationCovariance.inverse();



    ///// Mahalanobis Distance

    // distanceMahalanobis=p_innovation_total'*inv(S)*p_innovation_total;
    double distance_mahalanobis=0;

    // Equation
    distance_mahalanobis=innovationVector.transpose()*innovation_covariance_inverse*innovationVector;


#if 0 || _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::update() distance_mahalanobis ="<<distance_mahalanobis<<" for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
        this->log(logString.str());
    }
#endif



    ///// Near-optimal Kalman gain: K

    // Matrix
    Eigen::MatrixXd kalmanGain;

    // Equation
    kalmanGain=UpdatedState->covarianceMatrix*JacobianMeasurementErrorState.transpose()*innovation_covariance_inverse;


#if 0 || _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::update() Kalman Gain for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
        logString<<kalmanGain<<std::endl;
        this->log(logString.str());
    }
#endif



    ///// Updated error state estimate: x(k+1|k+1)

    // Equation
    Eigen::VectorXd incrementErrorState=kalmanGain*innovationVector;


#if 0 || _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::update() Increment Delta State vector for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
        logString<<incrementErrorState.transpose()<<std::endl;
        this->log(logString.str());
    }
#endif


    // Get the updated state from the increment of the Error State

    {
        unsigned int dimension=0;

        // Robot
        unsigned int dimensionRobotErrorState=this->TheRobotCore->getDimensionErrorState();
        Eigen::VectorXd incrementErrorStateRobot;
        if(dimensionRobotErrorState)
        {
            incrementErrorStateRobot=incrementErrorState.block(dimension, 0, dimensionRobotErrorState, 1);
            UpdatedState->TheRobotStateCore->updateStateFromIncrementErrorState(incrementErrorStateRobot);
        }
        dimension+=dimensionRobotErrorState;

#if 0 || _DEBUG_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::update() incrementDeltaStateRobot for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
            logString<<incrementErrorStateRobot.transpose()<<std::endl;
            this->log(logString.str());
        }
#endif


        // Global Parameters
        unsigned int dimensionGlobalParametersErrorState=this->TheGlobalParametersCore->getDimensionErrorState();
        Eigen::VectorXd incrementErrorStateGlobalParameters;
        if(dimensionGlobalParametersErrorState)
        {
            incrementErrorStateGlobalParameters=incrementErrorState.block(dimension, 0, dimensionGlobalParametersErrorState, 1);
            UpdatedState->TheGlobalParametersStateCore->updateStateFromIncrementErrorState(incrementErrorStateGlobalParameters);
        }
        dimension+=dimensionGlobalParametersErrorState;

#if 0 || _DEBUG_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::update() incrementDeltaStateGlobalParameters for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
            logString<<incrementErrorStateGlobalParameters.transpose()<<std::endl;
            this->log(logString.str());
        }
#endif


        // Sensors
        for(std::list< std::shared_ptr<SensorStateCore> >::iterator itListSensorState=UpdatedState->TheListSensorStateCore.begin();
            itListSensorState!=UpdatedState->TheListSensorStateCore.end();
            ++itListSensorState)
        {
            unsigned int dimensionSensorErrorState=(*itListSensorState)->getTheSensorCore()->getDimensionErrorState();
            Eigen::VectorXd incrementErrorStateSensor;
            if(dimensionSensorErrorState)
            {
                incrementErrorStateSensor=incrementErrorState.block(dimension, 0, dimensionSensorErrorState, 1);
                (*itListSensorState)->updateStateFromIncrementErrorState(incrementErrorStateSensor);
            }
            dimension+=dimensionSensorErrorState;

#if 0 || _DEBUG_MSF_LOCALIZATION_CORE
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::update() incrementDeltaStateSensor for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
                logString<<incrementErrorStateSensor.transpose()<<std::endl;
                this->log(logString.str());
            }
#endif
        }


        // Map
        // TODO



    }



    ///// Updated covariance estimate: P(k+1|k+1)
    //P_error_estimated_k1k1=(eye(dim_state,dim_state)-K*H)*P_error_estimated_k1k*(eye(dim_state,dim_state)-K*H)'+K*S*K';

    Eigen::MatrixXd AuxiliarMatrix(dimensionErrorState, dimensionErrorState);
    AuxiliarMatrix=Eigen::MatrixXd::Identity(dimensionErrorState, dimensionErrorState)-kalmanGain*JacobianMeasurementErrorState;

    // Equation
    UpdatedState->covarianceMatrix=AuxiliarMatrix*UpdatedState->covarianceMatrix*AuxiliarMatrix.transpose()+kalmanGain*innovationCovariance*kalmanGain.transpose();

#if 0 || _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::update() updated covariance for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
        logString<<UpdatedState->covarianceMatrix<<std::endl;
        this->log(logString.str());
    }
#endif


    ///// Reset Error State

    // TODO





    /////// Add element to the buffer
    if(TheMsfStorageCore->addElement(TheTimeStamp, UpdatedState))
    {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
        std::cout<<"!!Error addElement"<<std::endl;
#endif
        return -2;
    }


//    // Free update vector -> NO!
//    if(UpdatedState)
//        UpdatedState.reset();


#if _DEBUG_MSF_LOCALIZATION_CORE
    std::ostringstream logString;
    logString<<"MsfLocalizationCore::update() ended TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
    this->log(logString.str());
#endif

    // End
    return 0;
}




int MsfLocalizationCore::findSensorStateCoreFromList(std::list<std::shared_ptr<SensorStateCore> > TheListSensorStateCore, std::shared_ptr<SensorCore> TheSensorCore, std::shared_ptr<SensorStateCore>& TheSensorStateCore)
{

    // Match with the predicted state
    // Get predicted state of the sensor itSens
    for(std::list<std::shared_ptr<SensorStateCore> >::iterator itSensState=TheListSensorStateCore.begin();
        itSensState!=TheListSensorStateCore.end();
        ++itSensState)
    {
        //std::cout<<"matching previous sensor state"<<std::endl;

        if(!(*itSensState))
        {
#if _DEBUG_MSF_LOCALIZATION_CORE
            std::cout<<"!!pointer not set properly"<<std::endl;
#endif
            return -200;
        }

        if(!(*itSensState)->getTheSensorCore())
        {
#if _DEBUG_MSF_LOCALIZATION_CORE
            std::cout<<"!!Error getting the core"<<std::endl;
#endif
            return -100;
        }


        //std::cout<<"matching previous sensor state: going to check the type"<<std::endl;
        if((*itSensState)->getTheSensorCore()->getSensorId() == TheSensorCore->getSensorId())
        {
            // Polymorphic
            TheSensorStateCore=(*itSensState);
            break;
        }
    }

    // Check if not found
    if(!TheSensorStateCore)
    {
#if _DEBUG_MSF_LOCALIZATION_CORE
        logFile<<"MsfLocalizationCore::findSensorStateCoreFromList() error !TheSensorStateCore"<<std::endl;
#endif
        return -2;
    }




    return 0;
}



int MsfLocalizationCore::predictedFreeModelRobotCovariance(TimeStamp DeltaTime, std::shared_ptr<FreeModelRobotCore> robotCore, std::shared_ptr<FreeModelRobotStateCore> predictedStateRobot, Eigen::MatrixXd* previousStateCovarianceMatrix, Eigen::MatrixXd* predictedStateCovarianceMatrix)
{
    //check
    if(!predictedStateRobot)
    {
        return -2;
    }

    // Covariances update
    // Linear part
    predictedStateCovarianceMatrix->block<9,9>(0,0)=predictedStateRobot->errorStateJacobian.linear*previousStateCovarianceMatrix->block<9,9>(0,0)*predictedStateRobot->errorStateJacobian.linear.transpose();

    // Angular part
    predictedStateCovarianceMatrix->block<9,9>(9,9)=predictedStateRobot->errorStateJacobian.angular*previousStateCovarianceMatrix->block<9,9>(9,9)*predictedStateRobot->errorStateJacobian.angular.transpose();

    // Linear - angular
    predictedStateCovarianceMatrix->block<9,9>(0,9)=predictedStateRobot->errorStateJacobian.linear*previousStateCovarianceMatrix->block<9,9>(0,9)*predictedStateRobot->errorStateJacobian.angular.transpose();

    // Angular - linear
    predictedStateCovarianceMatrix->block<9,9>(9,0)=predictedStateCovarianceMatrix->block<9,9>(0,9).transpose();



    // Adding noise

    // Noise in the linear acceleration * Dt
    predictedStateCovarianceMatrix->block<3,3>(6,6)+=robotCore->getNoiseLinearAcceleration()*DeltaTime.get_double();

    // Noise in the angular velocity * Dt
    predictedStateCovarianceMatrix->block<3,3>(15,15)+=robotCore->getNoiseAngularAcceleration()*DeltaTime.get_double();


    // end
    return 0;

}


int MsfLocalizationCore::predictedImuImuCovariance(TimeStamp DeltaTime, std::shared_ptr<ImuSensorCore> TheImuSensor1Core, std::shared_ptr<ImuSensorCore> TheImuSensor2Core, std::shared_ptr<ImuSensorStateCore> predictedImuStateSensor1, std::shared_ptr<ImuSensorStateCore> predictedImuStateSensor2, Eigen::MatrixXd* previousStateCovarianceMatrix, MatrixPoint InitPoint, Eigen::MatrixXd* predictedStateCovarianceMatrix, MatrixPoint& EndPoint)
{
    // Check
    if(!predictedImuStateSensor1)
        return -2;
    if(!predictedImuStateSensor2)
        return -2;



    // Points
    MatrixPoint WorkingPoint=InitPoint;

    // Covariances update

    // Position sensor wrt robot
    if(TheImuSensor1Core->isEstimationPositionSensorWrtRobotEnabled())
    {
        // Reset col
        //previousDimensionErrorState_col=initDimension_col;
        WorkingPoint.col=InitPoint.col;

        // Position sensor wrt robot
        if(TheImuSensor2Core->isEstimationPositionSensorWrtRobotEnabled())
        {
            // Update variances
            predictedStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)=predictedImuStateSensor1->errorStateJacobian.positionSensorWrtRobot*previousStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)*predictedImuStateSensor2->errorStateJacobian.positionSensorWrtRobot.transpose();

            // Update dimension for next
            WorkingPoint.col+=3;
        }

        // Attitude sensor wrt robot
        if(TheImuSensor2Core->isEstimationAttitudeSensorWrtRobotEnabled())
        {
            // Update variances
            predictedStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)=predictedImuStateSensor1->errorStateJacobian.positionSensorWrtRobot*previousStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)*predictedImuStateSensor2->errorStateJacobian.attitudeSensorWrtRobot.transpose();

            // Update dimension for next
            WorkingPoint.col+=3;
        }

        // bias linear acceleration
        if(TheImuSensor2Core->isEstimationBiasLinearAccelerationEnabled())
        {
            // Update variances
            predictedStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)=predictedImuStateSensor1->errorStateJacobian.positionSensorWrtRobot*previousStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)*predictedImuStateSensor2->errorStateJacobian.biasesLinearAcceleration.transpose();

            // Update dimension for next
            WorkingPoint.col+=3;
        }

        // bias angular velocity
        if(TheImuSensor2Core->isEstimationBiasAngularVelocityEnabled())
        {
            // Update variances
            predictedStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)=predictedImuStateSensor1->errorStateJacobian.positionSensorWrtRobot*previousStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)*predictedImuStateSensor2->errorStateJacobian.biasesAngularVelocity.transpose();

            // Update dimension for next
            WorkingPoint.col+=3;
        }

        // Update dimension for next
        // Update row
        WorkingPoint.row+=3;
    }

    // Attitude sensor wrt robot
    if(TheImuSensor1Core->isEstimationAttitudeSensorWrtRobotEnabled())
    {
        // Reset col
        WorkingPoint.col=InitPoint.col;

        // Position sensor wrt robot
        if(TheImuSensor2Core->isEstimationPositionSensorWrtRobotEnabled())
        {
            // Update variances
            predictedStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)=predictedImuStateSensor1->errorStateJacobian.attitudeSensorWrtRobot*previousStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)*predictedImuStateSensor2->errorStateJacobian.positionSensorWrtRobot.transpose();

            // Update dimension for next
            WorkingPoint.col+=3;
        }

        // Attitude sensor wrt robot
        if(TheImuSensor2Core->isEstimationAttitudeSensorWrtRobotEnabled())
        {
            // Update variances
            predictedStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)=predictedImuStateSensor1->errorStateJacobian.attitudeSensorWrtRobot*previousStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)*predictedImuStateSensor2->errorStateJacobian.attitudeSensorWrtRobot.transpose();

            // Update dimension for next
            WorkingPoint.col+=3;
        }

        // bias linear acceleration
        if(TheImuSensor2Core->isEstimationBiasLinearAccelerationEnabled())
        {
            // Update variances
            predictedStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)=predictedImuStateSensor1->errorStateJacobian.attitudeSensorWrtRobot*previousStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)*predictedImuStateSensor2->errorStateJacobian.biasesLinearAcceleration.transpose();

            // Update dimension for next
            WorkingPoint.col+=3;
        }

        // bias angular velocity
        if(TheImuSensor2Core->isEstimationBiasAngularVelocityEnabled())
        {
            // Update variances
            predictedStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)=predictedImuStateSensor1->errorStateJacobian.attitudeSensorWrtRobot*previousStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)*predictedImuStateSensor2->errorStateJacobian.biasesAngularVelocity.transpose();

            // Update dimension for next
            WorkingPoint.col+=3;
        }

        // Update dimension for next
        // Update row
        WorkingPoint.row+=3;
    }

    // bias linear acceleration
    if(TheImuSensor1Core->isEstimationBiasLinearAccelerationEnabled())
    {
        // Reset col
        WorkingPoint.col=InitPoint.col;

        // Position sensor wrt robot
        if(TheImuSensor2Core->isEstimationPositionSensorWrtRobotEnabled())
        {
            // Update variances
            predictedStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)=predictedImuStateSensor1->errorStateJacobian.biasesLinearAcceleration*previousStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)*predictedImuStateSensor2->errorStateJacobian.positionSensorWrtRobot.transpose();

            // Update dimension for next
            WorkingPoint.col+=3;
        }

        // Attitude sensor wrt robot
        if(TheImuSensor2Core->isEstimationAttitudeSensorWrtRobotEnabled())
        {
            // Update variances
            predictedStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)=predictedImuStateSensor1->errorStateJacobian.biasesLinearAcceleration*previousStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)*predictedImuStateSensor2->errorStateJacobian.attitudeSensorWrtRobot.transpose();

            // Update dimension for next
            WorkingPoint.col+=3;
        }

        // bias linear acceleration
        if(TheImuSensor2Core->isEstimationBiasLinearAccelerationEnabled())
        {
            // Update variances
            predictedStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)=predictedImuStateSensor1->errorStateJacobian.biasesLinearAcceleration*previousStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)*predictedImuStateSensor2->errorStateJacobian.biasesLinearAcceleration.transpose();

            // Update dimension for next
            WorkingPoint.col+=3;
        }

        // bias angular velocity
        if(TheImuSensor2Core->isEstimationBiasAngularVelocityEnabled())
        {
            // Update variances
            predictedStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)=predictedImuStateSensor1->errorStateJacobian.biasesLinearAcceleration*previousStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)*predictedImuStateSensor2->errorStateJacobian.biasesAngularVelocity.transpose();

            // Update dimension for next
            WorkingPoint.col+=3;
        }

        // Update dimension for next
        // Update row
        WorkingPoint.row+=3;
    }

    // bias angular velocity
    if(TheImuSensor1Core->isEstimationBiasAngularVelocityEnabled())
    {
        // Reset col
        WorkingPoint.col=InitPoint.col;

        // Position sensor wrt robot
        if(TheImuSensor2Core->isEstimationPositionSensorWrtRobotEnabled())
        {
            // Update variances
            predictedStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)=predictedImuStateSensor1->errorStateJacobian.biasesAngularVelocity*previousStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)*predictedImuStateSensor2->errorStateJacobian.positionSensorWrtRobot.transpose();

            // Update dimension for next
            WorkingPoint.col+=3;
        }

        // Attitude sensor wrt robot
        if(TheImuSensor2Core->isEstimationAttitudeSensorWrtRobotEnabled())
        {
            // Update variances
            predictedStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)=predictedImuStateSensor1->errorStateJacobian.biasesAngularVelocity*previousStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)*predictedImuStateSensor2->errorStateJacobian.attitudeSensorWrtRobot.transpose();

            // Update dimension for next
            WorkingPoint.col+=3;
        }

        // bias linear acceleration
        if(TheImuSensor2Core->isEstimationBiasLinearAccelerationEnabled())
        {
            // Update variances
            predictedStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)=predictedImuStateSensor1->errorStateJacobian.biasesAngularVelocity*previousStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)*predictedImuStateSensor2->errorStateJacobian.biasesLinearAcceleration.transpose();

            // Update dimension for next
            WorkingPoint.col+=3;
        }

        // bias angular velocity
        if(TheImuSensor2Core->isEstimationBiasAngularVelocityEnabled())
        {
            // Update variances
            predictedStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)=predictedImuStateSensor1->errorStateJacobian.biasesAngularVelocity*previousStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)*predictedImuStateSensor2->errorStateJacobian.biasesAngularVelocity.transpose();

            // Update dimension for next
            WorkingPoint.col+=3;
        }

        // Update dimension for next
        // Update row
        WorkingPoint.row+=3;
    }

    // Update End Point
    EndPoint=WorkingPoint;


    // Symmetric value of the covariance matrix
    if(TheImuSensor1Core!=TheImuSensor2Core)
    {
        // Poner symmetric value
        predictedStateCovarianceMatrix->block(InitPoint.col,
                                              InitPoint.row,
                                              EndPoint.col-InitPoint.col,
                                              EndPoint.row-InitPoint.row) = predictedStateCovarianceMatrix->block(InitPoint.row,
                                                                                                                     InitPoint.col,
                                                                                                                     EndPoint.row-InitPoint.row,
                                                                                                                     EndPoint.col-InitPoint.col).transpose();

    }


    // Add noise
    if(TheImuSensor1Core==TheImuSensor2Core)
    {
        // Previous dimensions
        unsigned int previousErrorStateDimension=0;

        if(TheImuSensor1Core->isEstimationPositionSensorWrtRobotEnabled())
        {
            previousErrorStateDimension+=3;
        }

        if(TheImuSensor1Core->isEstimationAttitudeSensorWrtRobotEnabled())
        {
            previousErrorStateDimension+=3;
        }

        if(TheImuSensor1Core->isEstimationBiasLinearAccelerationEnabled())
        {
            predictedStateCovarianceMatrix->block<3,3>(InitPoint.col+previousErrorStateDimension,InitPoint.row+previousErrorStateDimension)+=TheImuSensor1Core->getNoiseEstimationBiasLinearAcceleration()*DeltaTime.get_double();
            previousErrorStateDimension+=3;
        }

        if(TheImuSensor1Core->isEstimationBiasAngularVelocityEnabled())
        {
            predictedStateCovarianceMatrix->block<3,3>(InitPoint.col+previousErrorStateDimension,InitPoint.row+previousErrorStateDimension)+=TheImuSensor1Core->getNoiseEstimationBiasAngularVelocity()*DeltaTime.get_double();
        }


    }




    return 0;
}



int MsfLocalizationCore::predictedFreeModelRobotImuCovariance(TimeStamp DeltaTime, std::shared_ptr<FreeModelRobotCore> robotCore, std::shared_ptr<ImuSensorCore> TheImuSensor1Core, std::shared_ptr<FreeModelRobotStateCore> predictedStateRobot, std::shared_ptr<ImuSensorStateCore> predictedImuStateSensor1, Eigen::MatrixXd* previousStateCovarianceMatrix, MatrixPoint InitPoint, Eigen::MatrixXd* predictedStateCovarianceMatrix, MatrixPoint& EndPoint)
{
    //check
    if(!predictedStateRobot)
    {
        return -2;
    }
    if(!predictedImuStateSensor1)
        return -2;



    // Dimension
    MatrixPoint WorkingPoint=InitPoint;

    // R-Si
    // Robot linear

    // Position sensor wrt robot
    if(TheImuSensor1Core->isEstimationPositionSensorWrtRobotEnabled())
    {
        // Update variances
        predictedStateCovarianceMatrix->block<9,3>(WorkingPoint.row, WorkingPoint.col)=predictedStateRobot->errorStateJacobian.linear*previousStateCovarianceMatrix->block<9,3>(WorkingPoint.row, WorkingPoint.col)*predictedImuStateSensor1->errorStateJacobian.positionSensorWrtRobot.transpose();

        // Update dimension for next
        WorkingPoint.col+=3;
    }

    // Attitude sensor wrt robot
    if(TheImuSensor1Core->isEstimationAttitudeSensorWrtRobotEnabled())
    {
        // Update variances
        predictedStateCovarianceMatrix->block<9,3>(WorkingPoint.row, WorkingPoint.col)=predictedStateRobot->errorStateJacobian.linear*previousStateCovarianceMatrix->block<9,3>(WorkingPoint.row, WorkingPoint.col)*predictedImuStateSensor1->errorStateJacobian.attitudeSensorWrtRobot.transpose();

        // Update dimension for next
        WorkingPoint.col+=3;
    }

    // bias linear acceleration
    if(TheImuSensor1Core->isEstimationBiasLinearAccelerationEnabled())
    {
        // Update variances
        predictedStateCovarianceMatrix->block<9,3>(WorkingPoint.row, WorkingPoint.col)=predictedStateRobot->errorStateJacobian.linear*previousStateCovarianceMatrix->block<9,3>(WorkingPoint.row, WorkingPoint.col)*predictedImuStateSensor1->errorStateJacobian.biasesLinearAcceleration.transpose();

        // Update dimension for next
        WorkingPoint.col+=3;
    }

    // bias angular velocity
    if(TheImuSensor1Core->isEstimationBiasAngularVelocityEnabled())
    {
        // Update variances
        predictedStateCovarianceMatrix->block<9,3>(WorkingPoint.row, WorkingPoint.col)=predictedStateRobot->errorStateJacobian.linear*previousStateCovarianceMatrix->block<9,3>(WorkingPoint.row, WorkingPoint.col)*predictedImuStateSensor1->errorStateJacobian.biasesAngularVelocity.transpose();

        // Update dimension for next
        WorkingPoint.col+=3;
    }

    // Dimension
    WorkingPoint.row+=9;


    // Robot angular

    // Reset col
    WorkingPoint.col=InitPoint.col;


    // Position sensor wrt robot
    if(TheImuSensor1Core->isEstimationPositionSensorWrtRobotEnabled())
    {
        // Update variances
        predictedStateCovarianceMatrix->block<9,3>(WorkingPoint.row, WorkingPoint.col)=predictedStateRobot->errorStateJacobian.angular*previousStateCovarianceMatrix->block<9,3>(WorkingPoint.row, WorkingPoint.col)*predictedImuStateSensor1->errorStateJacobian.positionSensorWrtRobot.transpose();

        // Update dimension for next
        WorkingPoint.col+=3;
    }

    // Attitude sensor wrt robot
    if(TheImuSensor1Core->isEstimationAttitudeSensorWrtRobotEnabled())
    {
        // Update variances
        predictedStateCovarianceMatrix->block<9,3>(WorkingPoint.row, WorkingPoint.col)=predictedStateRobot->errorStateJacobian.angular*previousStateCovarianceMatrix->block<9,3>(WorkingPoint.row, WorkingPoint.col)*predictedImuStateSensor1->errorStateJacobian.attitudeSensorWrtRobot.transpose();

        // Update dimension for next
        WorkingPoint.col+=3;
    }

    // bias linear acceleration
    if(TheImuSensor1Core->isEstimationBiasLinearAccelerationEnabled())
    {
        // Update variances
        predictedStateCovarianceMatrix->block<9,3>(WorkingPoint.row, WorkingPoint.col)=predictedStateRobot->errorStateJacobian.angular*previousStateCovarianceMatrix->block<9,3>(WorkingPoint.row, WorkingPoint.col)*predictedImuStateSensor1->errorStateJacobian.biasesLinearAcceleration.transpose();

        // Update dimension for next
        WorkingPoint.col+=3;
    }

    // bias angular velocity
    if(TheImuSensor1Core->isEstimationBiasAngularVelocityEnabled())
    {
        // Update variances
        predictedStateCovarianceMatrix->block<9,3>(WorkingPoint.row, WorkingPoint.col)=predictedStateRobot->errorStateJacobian.angular*previousStateCovarianceMatrix->block<9,3>(WorkingPoint.row, WorkingPoint.col)*predictedImuStateSensor1->errorStateJacobian.biasesAngularVelocity.transpose();

        // Update dimension for next
        WorkingPoint.col+=3;
    }

    // Dimension
    WorkingPoint.row+=9;


    // Final dimension
    EndPoint=WorkingPoint;



    // Si-R
    // Update the symmetric part
    predictedStateCovarianceMatrix->block(InitPoint.col,
                                          InitPoint.row,
                                          EndPoint.col-InitPoint.col,
                                          EndPoint.row-InitPoint.row) = predictedStateCovarianceMatrix->block(InitPoint.row,
                                                                                                             InitPoint.col,
                                                                                                             EndPoint.row-InitPoint.row,
                                                                                                             EndPoint.col-InitPoint.col).transpose();


    // End
    return 0;
}




int MsfLocalizationCore::startThreads()
{
    // Prediction state thread
    predictThread=new std::thread(&MsfLocalizationCore::predictThreadFunction, this);

    // Buffer Manager thread
    bufferManagerThread=new std::thread(&MsfLocalizationCore::bufferManagerThreadFunction, this);


    return 0;
}



int MsfLocalizationCore::log(std::string logString)
{
    // Lock mutex
    TheLogFileMutex.lock();

    // Write in file
    logFile<<logString;

    // Unlock mutex
    TheLogFileMutex.unlock();

    return 0;
}
