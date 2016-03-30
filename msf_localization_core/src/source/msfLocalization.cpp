
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
    // TODO FIX! Must be done when reading the config file!
    TheRobotCore=std::make_shared<FreeModelRobotCore>();

    // Create Global Parameters Core
    TheGlobalParametersCore=std::make_shared<GlobalParametersCore>();

    // Sensors
    //Id
    firstAvailableSensorId=0;

    // Map elements
    // Id
    firstAvailableMapElementId=0;


    // Create Storage Core
    TheMsfStorageCore=std::make_shared<MsfStorageCore>();



    // LOG
    const char* env_p = std::getenv("FUSEON_STACK");

    logPath=std::string(env_p)+"/logs/"+"logMsfLocalizationCoreFile.txt";

    //std::cout<<"logPath: "<<logPath<<std::endl;

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
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::setStateEstimationEnabled() ended"<<std::endl;
        this->log(logString.str());
    }
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



    ///// Predict Core
    int error=predictSemiCore(TheTimeStamp, PredictedState);
    if(error)
        return error;




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


int MsfLocalizationCore::predictNoAddBuffer(TimeStamp TheTimeStamp, std::shared_ptr<StateEstimationCore>& ThePredictedState)
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
    if(this->TheMsfStorageCore->getElement(TheTimeStamp, ThePredictedState))
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
    if(!ThePredictedState)
        ThePredictedState=std::make_shared<StateEstimationCore>();

#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"predict pre: number of users of predicted state="<<ThePredictedState.use_count()<<std::endl;
        this->log(logString.str());
    }
#endif

    while(ThePredictedState.use_count()>2)
    {
        // Do nothig. Sleep a little
        // TODO optimize this!
        std::this_thread::sleep_for( std::chrono::nanoseconds( 50 ) );

#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"predict: number of users of predicted state="<<ThePredictedState.use_count()<<std::endl;
        this->log(logString.str());
    }
#endif

    }

#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"predict post: number of users of predicted state="<<ThePredictedState.use_count()<<std::endl;
        this->log(logString.str());
    }
#endif



    ///// Predict Core
    int error=predictSemiCore(TheTimeStamp, ThePredictedState);
    if(error)
        return error;




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




int MsfLocalizationCore::predictSemiCore(TimeStamp ThePredictedTimeStamp, std::shared_ptr<StateEstimationCore>& ThePredictedState)
{


    // Get the last state from the buffer
    TimeStamp PreviousTimeStamp;
    std::shared_ptr<StateEstimationCore> PreviousState;

    if(this->getPreviousState(ThePredictedTimeStamp, PreviousTimeStamp, PreviousState))
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
        logString<<"MsfLocalizationCore::predict() TS: sec="<<ThePredictedTimeStamp.sec<<" s; nsec="<<ThePredictedTimeStamp.nsec<<" ns"<<std::endl;
        logString<<"MsfLocalizationCore::predict() TS prev: sec="<<PreviousTimeStamp.sec<<" s; nsec="<<PreviousTimeStamp.nsec<<" ns"<<std::endl;
        this->log(logString.str());
    }
#endif



    int error=predictCore(PreviousTimeStamp, ThePredictedTimeStamp, PreviousState, ThePredictedState);

    if(error)
        return error;



    return 0;
}




int MsfLocalizationCore::predictCore(TimeStamp ThePreviousTimeStamp, TimeStamp ThePredictedTimeStamp, std::shared_ptr<StateEstimationCore> ThePreviousState, std::shared_ptr<StateEstimationCore>& ThePredictedState)
{

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
    TimeStamp beginTimePredictCore=getTimeStamp();
#endif

    /////// State
#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::predict() state TS: sec="<<ThePredictedTimeStamp.sec<<" s; nsec="<<ThePredictedTimeStamp.nsec<<" ns"<<std::endl;
        this->log(logString.str());
    }
#endif

    // Dimension of the state
    //unsigned int dimensionOfState=0;
    //unsigned int dimensionOfErrorState=0;


    ///// Global Parameters

    // TODO Improve
    std::shared_ptr<GlobalParametersStateCore> predictedStateGlobalParameters=std::make_shared<GlobalParametersStateCore>(ThePreviousState->TheGlobalParametersStateCore->getTheGlobalParametersCore());


    // Copy the same values
    predictedStateGlobalParameters->setGravity(ThePreviousState->TheGlobalParametersStateCore->getGravity());


    // Add
    ThePredictedState->TheGlobalParametersStateCore=predictedStateGlobalParameters;




    ///// Robot

    TimeStamp beginTimePredictRobot=getTimeStamp();
    {
        // State
        std::shared_ptr<RobotStateCore> predictedStateRobot;
        std::shared_ptr<RobotStateCore> pastStateRobot=ThePreviousState->TheRobotStateCore;

        if(!pastStateRobot)
        {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
            std::cout<<"!!previous state for robot not found!"<<std::endl;
#endif
            return -2;
        }

        // State
        if(TheRobotCore->predictState(ThePreviousTimeStamp, ThePredictedTimeStamp, pastStateRobot, predictedStateRobot))
        {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
            std::cout<<"!!Error predicting state of the robot"<<std::endl;
#endif
            return 1;
        }

        // Jacobians
        if(TheRobotCore->predictStateErrorStateJacobians(ThePreviousTimeStamp, ThePredictedTimeStamp, pastStateRobot, predictedStateRobot))
        {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
            std::cout<<"!!Error predicting error state jacobians of the robot"<<std::endl;
#endif
            return 1;
        }


        // Add
        ThePredictedState->TheRobotStateCore=predictedStateRobot;
    }
#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::predict() predict Robot time state and jac pred: "<<(getTimeStamp()-beginTimePredictRobot).nsec<<std::endl;
        this->log(logString.str());
    }
#endif



    ///// Sensors
#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
    TimeStamp beginTimePredictSensors=getTimeStamp();
#endif

    // Clean the list
    ThePredictedState->TheListSensorStateCore.clear();

    // Iterate
    for(std::list< std::shared_ptr<SensorCore> >::iterator itSens=TheListOfSensorCore.begin();
        itSens!=TheListOfSensorCore.end();
        ++itSens)
    {

        // Sensor Core
        std::shared_ptr<SensorCore> theSensorCore=(*itSens);

        // Auxiliar
        std::shared_ptr<SensorStateCore> pastStateSensor;
        std::shared_ptr<SensorStateCore> predictedStateSensor;

        // Find
        if(findSensorStateCoreFromList(ThePreviousState->TheListSensorStateCore, (*itSens), pastStateSensor))
        {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
            logFile<<"MsfLocalizationCore::predict() error predict state sensors findSensorStateCoreFromList"<<std::endl;
#endif
            return -2;
        }


        // State
        if(theSensorCore->predictState(ThePreviousTimeStamp, ThePredictedTimeStamp, pastStateSensor, predictedStateSensor))
        {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
            std::cout<<"!!Error predicting state of sensor"<<std::endl;
#endif
            return 1;
        }

        // Jacobians
        if(theSensorCore->predictStateErrorStateJacobians(ThePreviousTimeStamp, ThePredictedTimeStamp, pastStateSensor, predictedStateSensor))
        {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
            std::cout<<"!!Error predicting error state jacobians of the sensor"<<std::endl;
#endif
            return 1;
        }

        // Add
        ThePredictedState->TheListSensorStateCore.push_back(predictedStateSensor);

    }

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::predict() predict Sensors time state and jac pred: "<<(getTimeStamp()-beginTimePredictSensors).nsec<<std::endl;
        this->log(logString.str());
    }
#endif


    ///// Map

    // Clean the list
    ThePredictedState->TheListMapElementStateCore.clear();

    // Iterate
    for(std::list< std::shared_ptr<MapElementStateCore> >::iterator itMapElement=ThePreviousState->TheListMapElementStateCore.begin();
        itMapElement!=ThePreviousState->TheListMapElementStateCore.end();
        ++itMapElement)
    {
        //
        std::shared_ptr<MapElementCore> theMapElementCore=(*itMapElement)->getTheMapElementCore();

        // Auxiliar
        std::shared_ptr<MapElementStateCore> predictedMapElementState;



        // Find
        // Not needed


        // State
        if(theMapElementCore->predictState(ThePreviousTimeStamp, ThePredictedTimeStamp, (*itMapElement), predictedMapElementState))
        {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
            std::cout<<"!!Error predicting state of map element"<<std::endl;
#endif
            return 1;
        }

        // Jacobians
        if(theMapElementCore->predictStateErrorStateJacobians(ThePreviousTimeStamp, ThePredictedTimeStamp, (*itMapElement), predictedMapElementState))
        {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
            std::cout<<"!!Error predicting error state jacobians of the map element"<<std::endl;
#endif
            return 1;
        }



        // Add
        ThePredictedState->TheListMapElementStateCore.push_back(predictedMapElementState);
    }



#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::predict() predict Core time state and jac pred: "<<(getTimeStamp()-beginTimePredictCore).nsec<<std::endl;
        this->log(logString.str());
    }
#endif



    /////// Covariances
#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::predict() covariances TS: sec="<<ThePredictedTimeStamp.sec<<" s; nsec="<<ThePredictedTimeStamp.nsec<<" ns"<<std::endl;
        this->log(logString.str());
    }
#endif

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
    TimeStamp beginTimePredictCoreCov=getTimeStamp();
#endif


    // Dimension
    MatrixPoint WorkingInitPoint;


    // Delta Time Stamp
    TimeStamp DeltaTime=ThePredictedTimeStamp-ThePreviousTimeStamp;


    // Resize the covariance Matrix
    int dimensionOfErrorState=ThePredictedState->getDimensionErrorState();
    ThePredictedState->covarianceMatrix.resize(dimensionOfErrorState,dimensionOfErrorState);
    ThePredictedState->covarianceMatrix.setZero();



    // Aux Vars
    Eigen::SparseMatrix<double> jacobianRobotErrorState=ThePredictedState->TheRobotStateCore->getJacobianErrorState();


#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::predict() predict Core time cov in: "<<(getTimeStamp()-beginTimePredictCoreCov).nsec<<std::endl;
        this->log(logString.str());
    }
#endif




    /// Robot
#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::predict() covariance robot TS: sec="<<ThePredictedTimeStamp.sec<<" s; nsec="<<ThePredictedTimeStamp.nsec<<" ns"<<std::endl;
        this->log(logString.str());
    }
#endif


    {
        int dimensionRobotErrorState=TheRobotCore->getDimensionErrorState();

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
        TimeStamp beginTimePredictCoreCovRob=getTimeStamp();
#endif


        try
        {
            //Eigen::SparseMatrix<double> jacobianErrorStateNoise=ThePredictedState->TheRobotStateCore->getJacobianErrorStateNoise();

            Eigen::SparseMatrix<double> covarianceErrorStateNoise=ThePredictedState->TheRobotStateCore->getJacobianErrorStateNoise()*TheRobotCore->getCovarianceNoise(DeltaTime)*ThePredictedState->TheRobotStateCore->getJacobianErrorStateNoise().transpose();

            ThePredictedState->covarianceMatrix.block(0,0,dimensionRobotErrorState,dimensionRobotErrorState)=
                    jacobianRobotErrorState*ThePreviousState->covarianceMatrix.block(0,0,dimensionRobotErrorState,dimensionRobotErrorState)*jacobianRobotErrorState.transpose() +
                    Eigen::MatrixXd(covarianceErrorStateNoise);



//            Eigen::SparseMatrix<double> covarianceRobotKK=ThePreviousState->covarianceMatrix.block(0,0,dimensionRobotErrorState,dimensionRobotErrorState).sparseView();
//            Eigen::SparseMatrix<double> covarianceRobotK1K(dimensionRobotErrorState,dimensionRobotErrorState);


//            covarianceRobotK1K=
//                    jacobianRobotErrorState*covarianceRobotKK*jacobianRobotErrorState.transpose() +
//                    ThePredictedState->TheRobotStateCore->getJacobianErrorStateNoise()*TheRobotCore->getCovarianceNoise(DeltaTime)*ThePredictedState->TheRobotStateCore->getJacobianErrorStateNoise().transpose();


//            ThePredictedState->covarianceMatrix.block(0,0,dimensionRobotErrorState,dimensionRobotErrorState)=Eigen::MatrixXd(covarianceRobotK1K);


//            {
//                std::ostringstream logString;
//                logString<<"MsfLocalizationCore::predict() covarianceRobotK1K:"<<std::endl;
//                logString<<Eigen::MatrixXd(covarianceRobotK1K)<<std::endl;
//                this->log(logString.str());
//            }


//            {
//                std::ostringstream logString;

//                logString<<"ThePredictedState->TheRobotStateCore->getJacobianErrorStateNoise()"<<std::endl;
//                logString<<Eigen::MatrixXd(ThePredictedState->TheRobotStateCore->getJacobianErrorStateNoise())<<std::endl;
//                logString<<"MsfLocalizationCore::predict() covarianceRobotK1K:"<<std::endl;
//                logString<<ThePredictedState->covarianceMatrix.block(0,0,dimensionRobotErrorState,dimensionRobotErrorState)<<std::endl;
//                this->log(logString.str());
//            }


        }
        catch(...)
        {
            std::cout<<"Error in prediction covariances robot"<<std::endl;
        }


#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::predict() predict Core time cov rob: "<<(getTimeStamp()-beginTimePredictCoreCovRob).nsec<<std::endl;
            this->log(logString.str());
        }
#endif


    }





    /// Robot-Sensors
#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::predict() covariance robot-sensor TS: sec="<<ThePredictedTimeStamp.sec<<" s; nsec="<<ThePredictedTimeStamp.nsec<<" ns"<<std::endl;
        this->log(logString.str());
    }
#endif

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
    TimeStamp beginTimePredictCoreCovRobSen=getTimeStamp();
#endif

    // Init Point
    WorkingInitPoint.col=TheRobotCore->getDimensionErrorState();
    WorkingInitPoint.row=0;





    // Iterate on the sensors
    for(std::list< std::shared_ptr<SensorCore> >::iterator it1Sens=TheListOfSensorCore.begin();
        it1Sens!=TheListOfSensorCore.end();
        ++it1Sens)
    {

        // Auxiliar
        std::shared_ptr<SensorCore> TheSensorCore=(*it1Sens);
        std::shared_ptr<SensorStateCore> predictedStateSensor;

        // Find
        if(findSensorStateCoreFromList(ThePredictedState->TheListSensorStateCore, (*it1Sens), predictedStateSensor))
        {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
            std::cout<<"!!Error findSensorStateCoreFromList"<<std::endl;
#endif
            return -2;
        }

        // Dimensions
        int dimensionRobotErrorState=TheRobotCore->getDimensionErrorState();
        int dimensionSensorErrorState=TheSensorCore->getDimensionErrorState();


        // Calculate
        ThePredictedState->covarianceMatrix.block(WorkingInitPoint.row, WorkingInitPoint.col,dimensionRobotErrorState,dimensionSensorErrorState)=
                jacobianRobotErrorState*ThePreviousState->covarianceMatrix.block(WorkingInitPoint.row, WorkingInitPoint.col,dimensionRobotErrorState,dimensionSensorErrorState)*predictedStateSensor->getJacobianErrorState().transpose();


//        logFile<<"ThePredictedState->covarianceMatrix.block(WorkingInitPoint.row, WorkingInitPoint.col,dimensionRobotErrorState,dimensionSensorErrorState)"<<std::endl;
//        logFile<<ThePredictedState->covarianceMatrix.block(WorkingInitPoint.row, WorkingInitPoint.col,dimensionRobotErrorState,dimensionSensorErrorState)<<std::endl;



        // Set the Symmetric
        Eigen::MatrixXd auxMatSym=ThePredictedState->covarianceMatrix.block(WorkingInitPoint.row, WorkingInitPoint.col,dimensionRobotErrorState,dimensionSensorErrorState).transpose();
        ThePredictedState->covarianceMatrix.block(WorkingInitPoint.col, WorkingInitPoint.row,dimensionSensorErrorState, dimensionRobotErrorState)=
                auxMatSym;


//        logFile<<"ThePredictedState->covarianceMatrix.block(WorkingInitPoint.row, WorkingInitPoint.col,dimensionRobotErrorState,dimensionSensorErrorState)"<<std::endl;
//        logFile<<ThePredictedState->covarianceMatrix.block(WorkingInitPoint.row, WorkingInitPoint.col,dimensionRobotErrorState,dimensionSensorErrorState)<<std::endl;



        // Update Point
        WorkingInitPoint.row=0;
        WorkingInitPoint.col+=dimensionSensorErrorState;
    }

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::predict() predict Core time cov rob-sen: "<<(getTimeStamp()-beginTimePredictCoreCovRobSen).nsec<<std::endl;
        this->log(logString.str());
    }
#endif


    /// Sensors
#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::predict() covariance sensors TS: sec="<<ThePredictedTimeStamp.sec<<" s; nsec="<<ThePredictedTimeStamp.nsec<<" ns"<<std::endl;
        this->log(logString.str());
    }
#endif

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
    TimeStamp beginTimePredictCoreCovSenSen=getTimeStamp();
#endif

    // Init Point
    WorkingInitPoint.col=TheRobotCore->getDimensionErrorState();
    WorkingInitPoint.row=WorkingInitPoint.col;

    // Iterate
    for(std::list< std::shared_ptr<SensorCore> >::iterator it1Sens=TheListOfSensorCore.begin();
        it1Sens!=TheListOfSensorCore.end();
        ++it1Sens)
    {
        // it1Sens Auxiliar
        std::shared_ptr<SensorStateCore> predictedStateSensor1;

        // Find
        if(findSensorStateCoreFromList(ThePredictedState->TheListSensorStateCore, (*it1Sens), predictedStateSensor1))
        {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
            std::cout<<"!!Error findSensorStateCoreFromList"<<std::endl;
#endif
            return -2;
        }

        // Dimension sensor1
        int dimensionSensor1ErrorState=(*it1Sens)->getDimensionErrorState();


        // Aux vars -> Jacobians
        Eigen::MatrixXd jacobianSensor1ErrorState=predictedStateSensor1->getJacobianErrorState();


        // Iterate again
        for(std::list< std::shared_ptr<SensorCore> >::iterator it2Sens=it1Sens;
            it2Sens!=TheListOfSensorCore.end();
            ++it2Sens)
        {
            // it2Sens Auxiliar
            std::shared_ptr<SensorStateCore> predictedStateSensor2;

            // Find
            if(findSensorStateCoreFromList(ThePredictedState->TheListSensorStateCore, (*it2Sens), predictedStateSensor2))
            {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
                std::cout<<"!!Error findSensorStateCoreFromList"<<std::endl;
#endif
                return -2;
            }

            // Dimension sensor2
            int dimensionSensor2ErrorState=(*it2Sens)->getDimensionErrorState();


            // Calculate
            ThePredictedState->covarianceMatrix.block(WorkingInitPoint.row, WorkingInitPoint.col,dimensionSensor1ErrorState,dimensionSensor2ErrorState)=
                    jacobianSensor1ErrorState*ThePreviousState->covarianceMatrix.block(WorkingInitPoint.row, WorkingInitPoint.col,dimensionSensor1ErrorState,dimensionSensor2ErrorState)*predictedStateSensor2->getJacobianErrorState().transpose();


            // Noise if the same
            if((*it1Sens) == (*it2Sens))
            {
                Eigen::SparseMatrix<double> covarianceErrorStateNoise=predictedStateSensor1->getJacobianErrorStateNoise() * (*it1Sens)->getCovarianceNoise(DeltaTime) * predictedStateSensor1->getJacobianErrorStateNoise().transpose();
                ThePredictedState->covarianceMatrix.block(WorkingInitPoint.row, WorkingInitPoint.col,dimensionSensor1ErrorState,dimensionSensor2ErrorState)+=
                        Eigen::MatrixXd(covarianceErrorStateNoise);
            }


            // Symmetric if different
            if((*it1Sens) != (*it2Sens))
            {
                Eigen::MatrixXd AuxMatSym=ThePredictedState->covarianceMatrix.block(WorkingInitPoint.row, WorkingInitPoint.col, dimensionSensor1ErrorState, dimensionSensor2ErrorState).transpose();
                ThePredictedState->covarianceMatrix.block(WorkingInitPoint.col, WorkingInitPoint.row, dimensionSensor2ErrorState, dimensionSensor1ErrorState)=
                        AuxMatSym;

            }

            // Update Working Point
            WorkingInitPoint.col+=dimensionSensor2ErrorState;

        }

        // Update the dimension for the next sensor
        WorkingInitPoint.row+=dimensionSensor1ErrorState;
        WorkingInitPoint.col=WorkingInitPoint.row;

    }
#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::predict() predict Core time cov sen-sen: "<<(getTimeStamp()-beginTimePredictCoreCovSenSen).nsec<<std::endl;
        this->log(logString.str());
    }
#endif

    /// Robot-Map
    // TODO

    /// Sensors-Map
    // TODO

    /// Map
    // TODO


    /// Display
    //logFile<<"Covariances Matrix of the predicted state="<<std::endl;
    //logFile<<PredictedState->covarianceMatrix<<std::endl;


#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::predict() predict Core time: "<<(getTimeStamp()-beginTimePredictCore).nsec<<std::endl;
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

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;

        TimeStamp begin=getTimeStamp();
#endif

    while(UpdatedState.use_count()>2)
    {
        // Do nothig. Sleep a little
        // TODO optimize this!
        std::this_thread::sleep_for( std::chrono::nanoseconds( 50 ) );

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
        logString<<"MsfLocalizationCore::update -> waiting!"<<std::endl;
#endif

    }

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
        logString<<"MsfLocalizationCore::update -> waiting time: "<<(getTimeStamp()-begin).nsec<<" ns"<<std::endl;
        this->log(logString.str());

    }
#endif

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




    ///// Measurement prediction and matching

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
    TimeStamp beginMeasurementPrediction=getTimeStamp();
#endif


    std::list<std::shared_ptr<SensorMeasurementCore> > TheListPredictedMeasurements;
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

                // Cast the imu sensor core
                std::shared_ptr<ImuSensorCore> TheImuSensorCore=std::dynamic_pointer_cast<ImuSensorCore>((*itListMeas)->getTheSensorCore());


                // Cast the imu sensor state
                std::shared_ptr<ImuSensorStateCore> TheImuSensorStateCore=std::static_pointer_cast<ImuSensorStateCore>(TheSensorStateCore);


                // Create a pointer for the measurement prediction
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
                continue;
            }

            /// Coded Visual Marker
            case SensorTypes::coded_visual_marker_eye:
            {
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

                // Find the map element state
                std::shared_ptr<MapElementStateCore> TheMapElementStateCore;
                if(findMapElementStateCoreFromList(UpdatedState->TheListMapElementStateCore, (*itListMeas), TheMapElementStateCore))
                {
                    //std::cout<<"error"<<std::endl;
                    break;
                }
                if(!TheMapElementStateCore)
                    break;


                // Cast the sensor core
                std::shared_ptr<CodedVisualMarkerEyeCore> TheVisualMarkerSensorCore=std::dynamic_pointer_cast<CodedVisualMarkerEyeCore>((*itListMeas)->getTheSensorCore());

                // Create a pointer for the measurement prediction
                std::shared_ptr<CodedVisualMarkerMeasurementCore> TheCodedVisualMarkerPredictedMeasurement;


                // Measurement prediction
                if(TheVisualMarkerSensorCore->predictMeasurement(TheTimeStamp, UpdatedState->TheGlobalParametersStateCore, UpdatedState->TheRobotStateCore, TheSensorStateCore, TheMapElementStateCore, TheCodedVisualMarkerPredictedMeasurement))
                {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
                    std::cout<<"MsfLocalizationCore::update() error 3"<<std::endl;
#endif
                    return 3;
                }


                // Jacobian Measurement prediction
                // TODO
                if(TheVisualMarkerSensorCore->jacobiansMeasurements(TheTimeStamp, UpdatedState->TheGlobalParametersStateCore, UpdatedState->TheRobotStateCore, TheSensorStateCore, TheMapElementStateCore, (*itListMeas), TheCodedVisualMarkerPredictedMeasurement))
                {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
                    std::cout<<"MsfLocalizationCore::update() error 3"<<std::endl;
#endif
                    return 3;
                }


                // Push to the predicted measurements
                TheListPredictedMeasurements.push_back(TheCodedVisualMarkerPredictedMeasurement);

                // Push to the Matched Measurements
                TheListMatchedMeasurements.push_back((*itListMeas));


                // End
                continue;
            }

            /// Default
            default:
            {
                break;
            }

        }

        // If no matched add to the unmatched

        // Push to the Matched Measurements -> The imu measurement is always matched
        TheListUnmatchedMeasurements.push_back((*itListMeas));

    }

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::update() measurement prediction time: "<<(getTimeStamp()-beginMeasurementPrediction).nsec<<std::endl;
        this->log(logString.str());
    }
#endif



    ///////// Get dimensions

    /// Measurements
    unsigned int dimensionMeasurements=0;
    for(std::list<std::shared_ptr<SensorMeasurementCore> >::const_iterator itListMatchedMeas=TheListMatchedMeasurements.begin();
        itListMatchedMeas!=TheListMatchedMeasurements.end();
        ++itListMatchedMeas)
    {
        dimensionMeasurements+=(*itListMatchedMeas)->getTheSensorCore()->getDimensionMeasurement();
    }

    /// Error Measurement
    unsigned int dimensionErrorMeasurements=0;
    for(std::list<std::shared_ptr<SensorMeasurementCore> >::const_iterator itListMatchedMeas=TheListMatchedMeasurements.begin();
        itListMatchedMeas!=TheListMatchedMeasurements.end();
        ++itListMatchedMeas)
    {
        dimensionErrorMeasurements+=(*itListMatchedMeas)->getTheSensorCore()->getDimensionErrorMeasurement();
    }

    /// Error State
    unsigned int dimensionErrorState=0;
    // Robot
    dimensionErrorState+=UpdatedState->TheRobotStateCore->getTheRobotCore()->getDimensionErrorState();
    // Global Parameters
    dimensionErrorState+=UpdatedState->TheGlobalParametersStateCore->getTheGlobalParametersCore()->getDimensionErrorState();
    // Sensors
    for(std::list<std::shared_ptr<SensorStateCore> >::const_iterator itListMatchedMeas=UpdatedState->TheListSensorStateCore.begin();
        itListMatchedMeas!=UpdatedState->TheListSensorStateCore.end();
        ++itListMatchedMeas)
    {
        dimensionErrorState+=(*itListMatchedMeas)->getTheSensorCore()->getDimensionErrorState();
    }
    // Map
    for(std::list<std::shared_ptr<MapElementStateCore> >::const_iterator itListMatchedMeas=UpdatedState->TheListMapElementStateCore.begin();
        itListMatchedMeas!=UpdatedState->TheListMapElementStateCore.end();
        ++itListMatchedMeas)
    {
        dimensionErrorState+=(*itListMatchedMeas)->getTheMapElementCore()->getDimensionErrorState();
    }

#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::update() dimension error state="<<dimensionErrorState<<" for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
        this->log(logString.str());
    }
#endif


    /// Global Parameters
    unsigned int dimensionGlobalParameters=0;
    dimensionGlobalParameters=UpdatedState->TheGlobalParametersStateCore->getTheGlobalParametersCore()->getDimensionErrorParameters();

    /// Sensor Parameters
    unsigned int dimensionSensorParameters=0;
    for(std::list<std::shared_ptr<SensorMeasurementCore> >::const_iterator itListMatchedMeas=TheListMatchedMeasurements.begin();
        itListMatchedMeas!=TheListMatchedMeasurements.end();
        ++itListMatchedMeas)
    {
        dimensionSensorParameters+=(*itListMatchedMeas)->getTheSensorCore()->getDimensionErrorParameters();
    }

    /// Map parameters
    unsigned int dimensionMapErrorParameters=0;
    for(std::list<std::shared_ptr<SensorMeasurementCore> >::const_iterator itListMatchedMeas=TheListMatchedMeasurements.begin();
        itListMatchedMeas!=TheListMatchedMeasurements.end();
        ++itListMatchedMeas)
    {
        switch((*itListMatchedMeas)->getTheSensorCore()->getSensorType())
        {
            case SensorTypes::imu:
            default:
            {
                // Do nothing
                break;
            }

            case SensorTypes::coded_visual_marker_eye:
            {
                // Match map element
                std::shared_ptr<MapElementStateCore> TheMapElementStateCore;
                if(findMapElementStateCoreFromList(UpdatedState->TheListMapElementStateCore, (*itListMatchedMeas), TheMapElementStateCore))
                {
                    std::cout<<"Error"<<std::endl;
                    return -200;
                }
                if(!TheMapElementStateCore)
                {
                    std::cout<<"Error"<<std::endl;
                    return -201;
                }


                // Get dimension
                dimensionMapErrorParameters+=TheMapElementStateCore->getTheMapElementCore()->getDimensionErrorParameters();


                // End
                break;
            }
        }

    }




    ///// Innovation vector

    // Vector
    Eigen::VectorXd innovationVector;
    innovationVector.resize(dimensionErrorMeasurements, 1);
    innovationVector.setZero();

    // Fill
    {
        unsigned int dimension=0;
        for(std::list<std::shared_ptr<SensorMeasurementCore> >::const_iterator itListMatchedMeas=TheListMatchedMeasurements.begin(), itListPredictedMeas=TheListPredictedMeasurements.begin();
            itListMatchedMeas!=TheListMatchedMeasurements.end() && itListPredictedMeas!=TheListMatchedMeasurements.end();
            ++itListMatchedMeas, ++itListPredictedMeas)
        {
            unsigned int dimensionErrorMeasurementSensorI=(*itListMatchedMeas)->getTheSensorCore()->getDimensionErrorMeasurement();


            innovationVector.block(dimension, 0, dimensionErrorMeasurementSensorI, 1)=(*itListMatchedMeas)->getInnovation((*itListMatchedMeas), (*itListPredictedMeas));


            dimension+=dimensionErrorMeasurementSensorI;
        }
    }


#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::update() Innovation vector for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
        logString<<innovationVector.transpose()<<std::endl;
        this->log(logString.str());
    }
#endif



    //// Total Jacobians Measurement

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
    TimeStamp beginJacobians=getTimeStamp();
#endif


    /// Jacobian Measurement - Error State
    Eigen::MatrixXd JacobianMeasurementErrorState;
    JacobianMeasurementErrorState.resize(dimensionErrorMeasurements, dimensionErrorState);
    JacobianMeasurementErrorState.setZero();

    // Fill
    {
        unsigned int dimensionTotalMeasurementI=0;
        for(std::list<std::shared_ptr<SensorMeasurementCore> >::const_iterator itListPredictedMeas=TheListPredictedMeasurements.begin();
            itListPredictedMeas!=TheListPredictedMeasurements.end();
            ++itListPredictedMeas)
        {
            unsigned int dimensionErrorMeasurementI=(*itListPredictedMeas)->getTheSensorCore()->getDimensionErrorMeasurement();
            unsigned int dimensionTotalErrorStateI=0;
            unsigned int dimensionErrorStateI=0;

            // Robot
            dimensionErrorStateI=this->TheRobotCore->getDimensionErrorState();
            if(dimensionErrorStateI)
                JacobianMeasurementErrorState.block(dimensionTotalMeasurementI, dimensionTotalErrorStateI, dimensionErrorMeasurementI, dimensionErrorStateI)=(*itListPredictedMeas)->jacobianMeasurementErrorState.jacobianMeasurementRobotErrorState;
            dimensionTotalErrorStateI+=dimensionErrorStateI;

            // Global Parameters
            dimensionErrorStateI=this->TheGlobalParametersCore->getDimensionErrorState();
            if(dimensionErrorStateI)
                JacobianMeasurementErrorState.block(dimensionTotalMeasurementI, dimensionTotalErrorStateI, dimensionErrorMeasurementI, dimensionErrorStateI)=(*itListPredictedMeas)->jacobianMeasurementErrorState.jacobianMeasurementGlobalParametersErrorState;
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
                        JacobianMeasurementErrorState.block(dimensionTotalMeasurementI, dimensionTotalErrorStateI, dimensionErrorMeasurementI, dimensionErrorStateI)=(*itListPredictedMeas)->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState;
                }
                else
                {
                    // Do nothing -> Set Zeros (already set)
                }
                dimensionTotalErrorStateI+=dimensionErrorStateI;
            }


            // Map
            for(std::list< std::shared_ptr<MapElementCore> >::const_iterator itListMapElementCore=this->TheListOfMapElementCore.begin();
                itListMapElementCore!=this->TheListOfMapElementCore.end();
                ++itListMapElementCore)
            {

                dimensionErrorStateI=(*itListMapElementCore)->getDimensionErrorState();

                //
                switch((*itListMapElementCore)->getMapElementType())
                {
                    case MapElementTypes::coded_visual_marker:
                    {
                        //
                        switch((*itListPredictedMeas)->getMeasurementType())
                        {
                            case MeasurementTypes::coded_visual_marker:
                            {
                                // Cast
                                std::shared_ptr<CodedVisualMarkerMeasurementCore> TheVisualMarkerMeasurementCore=std::dynamic_pointer_cast<CodedVisualMarkerMeasurementCore>((*itListPredictedMeas));
                                std::shared_ptr<CodedVisualMarkerLandmarkCore> TheVisualMarkerLandmarkCore=std::dynamic_pointer_cast<CodedVisualMarkerLandmarkCore>((*itListMapElementCore));

                                if(TheVisualMarkerLandmarkCore->getId() == TheVisualMarkerMeasurementCore->getVisualMarkerId())
                                {
                                    if(dimensionErrorStateI)
                                        JacobianMeasurementErrorState.block(dimensionTotalMeasurementI, dimensionTotalErrorStateI, dimensionErrorMeasurementI, dimensionErrorStateI)=(*itListPredictedMeas)->jacobianMeasurementErrorState.jacobianMeasurementMapElementErrorState;
                                }
                                else
                                {
                                    // Do nothing -> Set Zeros (already set)
                                }

                                // End
                                break;
                            }

                            default:
                            {
                                // End
                                break;
                            }

                        }
                        // End
                        break;
                    }
                    default:
                    {
                        // Zeros

                        // End
                        break;
                    }
                }
                // Dimension
                dimensionTotalErrorStateI+=dimensionErrorStateI;
            }


            // Update dimension
            dimensionTotalMeasurementI+=dimensionErrorMeasurementI;
        }
    }

#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::update() JacobianMeasurementErrorState for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
        logString<<JacobianMeasurementErrorState<<std::endl;
        this->log(logString.str());
    }
#endif


    /// Jacobian Measurement - Measurement Noise
    Eigen::MatrixXd JacobianMeasurementNoise;
    JacobianMeasurementNoise.resize(dimensionErrorMeasurements, dimensionErrorMeasurements);
    JacobianMeasurementNoise.setZero();

    // Fill
    {
        unsigned int dimensionTotalMeasurementI=0;
        for(std::list<std::shared_ptr<SensorMeasurementCore> >::const_iterator itListPredictedMeas=TheListPredictedMeasurements.begin();
            itListPredictedMeas!=TheListPredictedMeasurements.end();
            ++itListPredictedMeas)
        {
            unsigned int dimensionErrorMeasurementI=(*itListPredictedMeas)->getTheSensorCore()->getDimensionErrorMeasurement();
            unsigned int dimensionTotalMeasurementJ=0;

            for(std::list< std::shared_ptr<SensorMeasurementCore> >::const_iterator itListMatchedMeas=TheListMatchedMeasurements.begin();
                itListMatchedMeas!=TheListMatchedMeasurements.end();
                ++itListMatchedMeas)
            {
                unsigned int dimensionErrorMeasurementJ=(*itListMatchedMeas)->getTheSensorCore()->getDimensionErrorMeasurement();
                if((*itListMatchedMeas)->getTheSensorCore() == (*itListPredictedMeas)->getTheSensorCore())
                {
                    if(dimensionErrorMeasurementJ)
                        JacobianMeasurementNoise.block(dimensionTotalMeasurementI, dimensionTotalMeasurementJ, dimensionErrorMeasurementI, dimensionErrorMeasurementJ)=(*itListPredictedMeas)->jacobianMeasurementSensorNoise.jacobianMeasurementSensorNoise;
                }
                else
                {
                    // Do nothing -> Set Zeros (already set)
                }
                // Update dimension
                dimensionTotalMeasurementJ+=dimensionErrorMeasurementJ;
            }
            // Update dimension
            dimensionTotalMeasurementI+=dimensionErrorMeasurementI;
        }
    }

#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::update() JacobianMeasurementNoise for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
        logString<<JacobianMeasurementNoise<<std::endl;
        this->log(logString.str());
    }
#endif


    /// Jacobian Measurement - Global Parameters
    Eigen::MatrixXd JacobianMeasurementGlobalParameters;
    JacobianMeasurementGlobalParameters.resize(dimensionErrorMeasurements, dimensionGlobalParameters);
    JacobianMeasurementGlobalParameters.setZero();

    // Fill
    {
        unsigned int dimensionTotalMeasurementI=0;
        for(std::list<std::shared_ptr<SensorMeasurementCore> >::const_iterator itListPredictedMeas=TheListPredictedMeasurements.begin();
            itListPredictedMeas!=TheListPredictedMeasurements.end();
            ++itListPredictedMeas)
        {
            unsigned int dimensionErrorMeasurementI=(*itListPredictedMeas)->getTheSensorCore()->getDimensionErrorMeasurement();
            unsigned int dimensionGlobalParametersI=this->TheGlobalParametersCore->getDimensionErrorParameters();
            JacobianMeasurementNoise.block(dimensionTotalMeasurementI, 0, dimensionErrorMeasurementI, dimensionGlobalParametersI)=(*itListPredictedMeas)->jacobianMeasurementErrorParameters.jacobianMeasurementGlobalParameters;
            dimensionTotalMeasurementI+=dimensionErrorMeasurementI;
        }
    }

#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::update() JacobianMeasurementGlobalParameters for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
        logString<<JacobianMeasurementGlobalParameters<<std::endl;
        this->log(logString.str());
    }
#endif


    /// Jacobian Measurement - Ohter Parameters (Robot Parameters)
    // TODO


    /// Jacobian Measurement - Sensor Parameters
    Eigen::MatrixXd JacobianMeasurementSensorParameters;
    JacobianMeasurementSensorParameters.resize(dimensionErrorMeasurements, dimensionSensorParameters);
    JacobianMeasurementSensorParameters.setZero();

    // Fill
    {
        unsigned int dimensionTotalMeasurementI=0;
        for(std::list<std::shared_ptr<SensorMeasurementCore> >::const_iterator itListPredictedMeas=TheListPredictedMeasurements.begin();
            itListPredictedMeas!=TheListPredictedMeasurements.end();
            ++itListPredictedMeas)
        {
            unsigned int dimensionErrorMeasurementI=(*itListPredictedMeas)->getTheSensorCore()->getDimensionErrorMeasurement();
            unsigned int dimensionTotalSensorParametersJ=0;

            for(std::list< std::shared_ptr<SensorMeasurementCore> >::const_iterator itListMatchedMeas=TheListMatchedMeasurements.begin();
                itListMatchedMeas!=TheListMatchedMeasurements.end();
                ++itListMatchedMeas)
            {
                unsigned int dimensionSensorParametersJ=(*itListMatchedMeas)->getTheSensorCore()->getDimensionErrorParameters();
                if((*itListMatchedMeas)->getTheSensorCore() == (*itListPredictedMeas)->getTheSensorCore())
                {
                    if(dimensionSensorParametersJ)
                    {
                        JacobianMeasurementSensorParameters.block(dimensionTotalMeasurementI, dimensionTotalSensorParametersJ, dimensionErrorMeasurementI, dimensionSensorParametersJ)=
                                (*itListPredictedMeas)->jacobianMeasurementErrorParameters.jacobianMeasurementSensorParameters;
                    }
                }
                else
                {
                    // Do nothing -> Set Zeros (already set)
                }
                // Update dimension
                dimensionTotalSensorParametersJ+=dimensionSensorParametersJ;
            }
            // Update dimension
            dimensionTotalMeasurementI+=dimensionErrorMeasurementI;
        }
    }


#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::update() JacobianMeasurementSensorParameters for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
        logString<<JacobianMeasurementSensorParameters<<std::endl;
        this->log(logString.str());
    }
#endif



    /// Jacobian Measurement - Map Parameters
    Eigen::MatrixXd JacobianMeasurementMapParameters;
    JacobianMeasurementMapParameters.resize(dimensionErrorMeasurements, dimensionMapErrorParameters);
    JacobianMeasurementMapParameters.setZero();

    // TODO


#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::update() JacobianMeasurementMapParameters for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
        logString<<JacobianMeasurementMapParameters<<std::endl;
        this->log(logString.str());
    }
#endif


#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::update() jacobians time: "<<(getTimeStamp()-beginJacobians).nsec<<std::endl;
        this->log(logString.str());
    }
#endif



    ///// Noises and covariances

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
    TimeStamp beginNoisesCovariances=getTimeStamp();
#endif


    /// Covariance Measurement
    Eigen::MatrixXd CovarianceMeasurement;
    CovarianceMeasurement.resize(dimensionErrorMeasurements, dimensionErrorMeasurements);
    CovarianceMeasurement.setZero();

    // Fill
    {
        unsigned int dimensionTotalMeasurementI=0;
        for(std::list<std::shared_ptr<SensorMeasurementCore> >::const_iterator itListPredictedMeas=TheListPredictedMeasurements.begin();
            itListPredictedMeas!=TheListPredictedMeasurements.end();
            ++itListPredictedMeas)
        {
            unsigned int dimensionErrorMeasurementI=(*itListPredictedMeas)->getTheSensorCore()->getDimensionErrorMeasurement();
            unsigned int dimensionTotalMeasurementJ=0;

            for(std::list< std::shared_ptr<SensorMeasurementCore> >::const_iterator itListMatchedMeas=TheListMatchedMeasurements.begin();
                itListMatchedMeas!=TheListMatchedMeasurements.end();
                ++itListMatchedMeas)
            {
                unsigned int dimensionErrorMeasurementJ=(*itListMatchedMeas)->getTheSensorCore()->getDimensionErrorMeasurement();
                if((*itListMatchedMeas)->getTheSensorCore() == (*itListPredictedMeas)->getTheSensorCore())
                {
                    if(dimensionErrorMeasurementJ)
                        CovarianceMeasurement.block(dimensionTotalMeasurementI, dimensionTotalMeasurementJ, dimensionErrorMeasurementI, dimensionErrorMeasurementJ)=
                                Eigen::MatrixXd((*itListPredictedMeas)->getTheSensorCore()->getCovarianceMeasurement());
                }
                else
                {
                    // Do nothing -> Set Zeros (already set)
                }
                // Update dimension
                dimensionTotalMeasurementJ+=dimensionErrorMeasurementJ;
            }
            // Update dimension
            dimensionTotalMeasurementI+=dimensionErrorMeasurementI;
        }
    }


#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::update() CovarianceMeasurement for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
        logString<<CovarianceMeasurement<<std::endl;
        this->log(logString.str());
    }
#endif



    /// Covariance Global Parameters
    Eigen::MatrixXd CovarianceGlobalParameters;
    CovarianceGlobalParameters.resize(dimensionGlobalParameters, dimensionGlobalParameters);
    CovarianceGlobalParameters.setZero();

    // Fill
    CovarianceGlobalParameters=this->TheGlobalParametersCore->getCovarianceGlobalParameters();


#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::update() CovarianceGlobalParameters for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
        logString<<CovarianceGlobalParameters<<std::endl;
        this->log(logString.str());
    }
#endif




    /// Covariance Other Parameters (Robot Parameters)
    // TODO



    /// Covariance Sensor Parameters
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
                        CovarianceSensorParameters.block(dimensionTotalSensorParametersI, dimensionTotalSensorParametersJ, dimensionSensorParametersI, dimensionSensorParametersJ)=
                                Eigen::MatrixXd((*itListMatchedMeasI)->getTheSensorCore()->getCovarianceParameters());
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


#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::update() UpdatedState->covarianceMatrix for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
        logString<<UpdatedState->covarianceMatrix<<std::endl;
        this->log(logString.str());
    }
#endif


#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::update() CovarianceSensorParameters for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
        logString<<CovarianceSensorParameters<<std::endl;
        this->log(logString.str());
    }
#endif


    /// Covariance Map parameters
    Eigen::MatrixXd CovarianceMapParameters;
    CovarianceMapParameters.resize(dimensionMapErrorParameters, dimensionMapErrorParameters);
    CovarianceMapParameters.setZero();

    // TODO



#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::update() CovarianceMapParameters for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
        logString<<CovarianceMapParameters<<std::endl;
        this->log(logString.str());
    }
#endif


#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::update() noises and covariances time: "<<(getTimeStamp()-beginNoisesCovariances).nsec<<std::endl;
        this->log(logString.str());
    }
#endif



    ///// Innovation (or residual) covariance: S

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
    TimeStamp beginInnovationCovariance=getTimeStamp();
#endif

    // Matrix
    Eigen::MatrixXd innovationCovariance;

    // Equation
    innovationCovariance=JacobianMeasurementErrorState*UpdatedState->covarianceMatrix*JacobianMeasurementErrorState.transpose()+
            JacobianMeasurementNoise*CovarianceMeasurement*JacobianMeasurementNoise.transpose()+
            JacobianMeasurementGlobalParameters*CovarianceGlobalParameters*JacobianMeasurementGlobalParameters.transpose()+
            JacobianMeasurementSensorParameters*CovarianceSensorParameters*JacobianMeasurementSensorParameters.transpose()+
            JacobianMeasurementMapParameters*CovarianceMapParameters*JacobianMeasurementMapParameters.transpose();

    Eigen::MatrixXd innovation_covariance_inverse=innovationCovariance.inverse();


//#if 1 || _DEBUG_MSF_LOCALIZATION_CORE
//    {
//        std::ostringstream logString;
//        logString<<"MsfLocalizationCore::update() JacobianMeasurementErrorState for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
//        logString<<JacobianMeasurementErrorState<<std::endl;
//        this->log(logString.str());
//    }
//#endif

    // TODO REMOVE RIGTH NOW
    //innovation_covariance_inverse.setZero();

#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::update() innovationCovariance for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
        logString<<innovationCovariance<<std::endl;
        this->log(logString.str());
    }
#endif

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::update() innovation covariance time: "<<(getTimeStamp()-beginInnovationCovariance).nsec<<std::endl;
        this->log(logString.str());
    }
#endif


    ///// Mahalanobis Distance

    // distanceMahalanobis=p_innovation_total'*inv(S)*p_innovation_total;
    double distance_mahalanobis=0;

    // Equation
    distance_mahalanobis=innovationVector.transpose()*innovation_covariance_inverse*innovationVector;


#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::update() distance_mahalanobis ="<<distance_mahalanobis<<" for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
        this->log(logString.str());
    }
#endif



    ///// Near-optimal Kalman gain: K

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
    TimeStamp beginKalmanGain=getTimeStamp();
#endif

    // Matrix
    Eigen::MatrixXd kalmanGain;

    // Equation
    kalmanGain=UpdatedState->covarianceMatrix*JacobianMeasurementErrorState.transpose()*innovation_covariance_inverse;


#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::update() Kalman Gain for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
        logString<<kalmanGain<<std::endl;
        this->log(logString.str());
    }
#endif

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::update() kalman gain time: "<<(getTimeStamp()-beginKalmanGain).nsec<<std::endl;
        this->log(logString.str());
    }
#endif



    ///// Updated error state estimate: x(k+1|k+1)

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
    TimeStamp beginUpdatedState=getTimeStamp();
#endif

    // Equation
    Eigen::VectorXd incrementErrorState=kalmanGain*innovationVector;


#if _DEBUG_MSF_LOCALIZATION_CORE
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

#if _DEBUG_MSF_LOCALIZATION_CORE
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

#if _DEBUG_MSF_LOCALIZATION_CORE
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

#if _DEBUG_MSF_LOCALIZATION_CORE
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::update() incrementDeltaStateSensor for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
                logString<<incrementErrorStateSensor.transpose()<<std::endl;
                this->log(logString.str());
            }
#endif
        }


        // Map
        for(std::list< std::shared_ptr<MapElementStateCore> >::iterator itListMapElementState=UpdatedState->TheListMapElementStateCore.begin();
            itListMapElementState!=UpdatedState->TheListMapElementStateCore.end();
            ++itListMapElementState)
        {
            unsigned int dimensionMapElementErrorState=(*itListMapElementState)->getTheMapElementCore()->getDimensionErrorState();
            Eigen::VectorXd incrementErrorStateMapElement;
            if(dimensionMapElementErrorState)
            {
                incrementErrorStateMapElement=incrementErrorState.block(dimension, 0, dimensionMapElementErrorState, 1);
                (*itListMapElementState)->updateStateFromIncrementErrorState(incrementErrorStateMapElement);
            }
            dimension+=dimensionMapElementErrorState;

#if _DEBUG_MSF_LOCALIZATION_CORE
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::update() incrementDeltaStateMapElement for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
                logString<<incrementErrorStateMapElement.transpose()<<std::endl;
                this->log(logString.str());
            }
#endif
        }



    }

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::update() updated state time: "<<(getTimeStamp()-beginUpdatedState).nsec<<std::endl;
        this->log(logString.str());
    }
#endif


    ///// Updated covariance estimate: P(k+1|k+1)
    //P_error_estimated_k1k1=(eye(dim_state,dim_state)-K*H)*P_error_estimated_k1k*(eye(dim_state,dim_state)-K*H)'+K*S*K';

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
    TimeStamp beginUpdatedCovariance=getTimeStamp();
#endif

    Eigen::MatrixXd oldUpdatedCovarianceMatrix=UpdatedState->covarianceMatrix;


//    Eigen::MatrixXd AuxiliarMatrix;//(dimensionErrorState, dimensionErrorState);
//    AuxiliarMatrix=Eigen::MatrixXd::Identity(dimensionErrorState, dimensionErrorState)-kalmanGain*JacobianMeasurementErrorState;


//#if 1 || _DEBUG_MSF_LOCALIZATION_CORE
//    {
//        std::ostringstream logString;
//        logString<<"MsfLocalizationCore::update() AuxiliarMatrix for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
//        logString<<AuxiliarMatrix<<std::endl;
//        this->log(logString.str());
//    }
//#endif

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::update() updated covariance interm time: "<<(getTimeStamp()-beginUpdatedCovariance).nsec<<std::endl;
        this->log(logString.str());
    }
#endif


#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
    TimeStamp beginUpdatedCovariance1=getTimeStamp();
#endif

    // To avoid Eigen Aliasing: http://eigen.tuxfamily.org/dox-devel/group__TopicAliasing.html
    //Eigen::MatrixXd AuxiliarMatrix2;//(dimensionErrorState, dimensionErrorState);

    //AuxiliarMatrix2=AuxiliarMatrix*oldUpdatedCovarianceMatrix*AuxiliarMatrix.transpose();



    //UpdatedState->covarianceMatrix.noalias()=AuxiliarMatrix*oldUpdatedCovarianceMatrix*AuxiliarMatrix.transpose()+kalmanGain*innovationCovariance*kalmanGain.transpose();


    UpdatedState->covarianceMatrix+=
            oldUpdatedCovarianceMatrix*JacobianMeasurementErrorState.transpose()*( -innovation_covariance_inverse + innovation_covariance_inverse* JacobianMeasurementErrorState*oldUpdatedCovarianceMatrix*JacobianMeasurementErrorState.transpose()  *innovation_covariance_inverse.transpose()  )*JacobianMeasurementErrorState*oldUpdatedCovarianceMatrix;

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::update() updated covariance interm 2 time: "<<(getTimeStamp()-beginUpdatedCovariance1).nsec<<std::endl;
        this->log(logString.str());
    }
#endif

    // Equation
    //UpdatedState->covarianceMatrix=AuxiliarMatrix*UpdatedState->covarianceMatrix.eval()*AuxiliarMatrix.transpose()+kalmanGain*innovationCovariance*kalmanGain.transpose();


    //UpdatedState->covarianceMatrix=AuxiliarMatrix2+kalmanGain*innovationCovariance*kalmanGain.transpose();



#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::update() updated covariance for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
        logString<<UpdatedState->covarianceMatrix<<std::endl;
        this->log(logString.str());
    }
#endif

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::update() updated covariance time: "<<(getTimeStamp()-beginUpdatedCovariance).nsec<<std::endl;
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


int MsfLocalizationCore::findMapElementStateCoreFromList(std::list< std::shared_ptr<MapElementStateCore> > TheListMapElementStateCore, std::shared_ptr<SensorMeasurementCore> TheSensorMeasurementCore, std::shared_ptr<MapElementStateCore>& TheMapElementStateCore)
{
    // Cast
    std::shared_ptr<CodedVisualMarkerMeasurementCore> the_coded_visual_marker_measurement=std::dynamic_pointer_cast<CodedVisualMarkerMeasurementCore>(TheSensorMeasurementCore);


    // Match with the map element
    for(std::list<std::shared_ptr<MapElementStateCore>>::iterator itVisualMarkerLandmark=TheListMapElementStateCore.begin();
        itVisualMarkerLandmark!=TheListMapElementStateCore.end();
        ++itVisualMarkerLandmark)
    {
        switch((*itVisualMarkerLandmark)->getTheMapElementCore()->getMapElementType())
        {
            // Coded visual markers
            case MapElementTypes::coded_visual_marker:
            {
                // Cast
                std::shared_ptr<CodedVisualMarkerLandmarkCore> the_coded_visual_marker_landmark_core=std::dynamic_pointer_cast<CodedVisualMarkerLandmarkCore>((*itVisualMarkerLandmark)->getTheMapElementCore());

                // Check ids
                if(the_coded_visual_marker_landmark_core->getId() == the_coded_visual_marker_measurement->getVisualMarkerId())
                {
                    TheMapElementStateCore=*itVisualMarkerLandmark;
                    return 0;
                }

                // End
                break;
            }
            // Default
            case MapElementTypes::undefined:
            default:
                break;
        }

    }



    return 1;
}


/*
int MsfLocalizationCore::predictedFreeModelRobotCovariance(TimeStamp DeltaTime, std::shared_ptr<FreeModelRobotCore> robotCore, std::shared_ptr<FreeModelRobotStateCore> predictedStateRobot, Eigen::MatrixXd* previousStateCovarianceMatrix, Eigen::MatrixXd* predictedStateCovarianceMatrix)
{
    //check
    if(!predictedStateRobot)
    {
        return -2;
    }

    // Covariances update

    predictedStateCovarianceMatrix->block<18,18>(0,0)=predictedStateRobot->errorStateJacobian*predictedStateCovarianceMatrix->block<18,18>(0,0)*predictedStateRobot->errorStateJacobian.transpose();



//    // Linear part
//    predictedStateCovarianceMatrix->block<9,9>(0,0)=predictedStateRobot->errorStateJacobian.linear*previousStateCovarianceMatrix->block<9,9>(0,0)*predictedStateRobot->errorStateJacobian.linear.transpose();

//    // Angular part
//    predictedStateCovarianceMatrix->block<9,9>(9,9)=predictedStateRobot->errorStateJacobian.angular*previousStateCovarianceMatrix->block<9,9>(9,9)*predictedStateRobot->errorStateJacobian.angular.transpose();

//    // Linear - angular
//    predictedStateCovarianceMatrix->block<9,9>(0,9)=predictedStateRobot->errorStateJacobian.linear*previousStateCovarianceMatrix->block<9,9>(0,9)*predictedStateRobot->errorStateJacobian.angular.transpose();

//    // Angular - linear
//    predictedStateCovarianceMatrix->block<9,9>(9,0)=predictedStateCovarianceMatrix->block<9,9>(0,9).transpose();



    // Adding noise

    // Noise in the linear acceleration * Dt
    predictedStateCovarianceMatrix->block<3,3>(6,6)+=robotCore->getNoiseLinearAcceleration()*DeltaTime.get_double();

    // Noise in the angular velocity * Dt
    predictedStateCovarianceMatrix->block<3,3>(15,15)+=robotCore->getNoiseAngularAcceleration()*DeltaTime.get_double();


    // end
    return 0;

}
*/

/*
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
*/


/*
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
*/



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
