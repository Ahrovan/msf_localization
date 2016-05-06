
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

    // TheListOfInputCore
    TheListOfInputCore.clear();

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
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
        logFile<<"MsfLocalizationCore::setStateEstimationEnabled() error in getLastElementWithStateEstimate"<<std::endl;
#endif
        return 2;
    }

    if(TheMsfStorageCore->purgeRingBuffer(-1))
    {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
        logFile<<"MsfLocalizationCore::setStateEstimationEnabled() error in purgeRingBuffer"<<std::endl;
#endif
        return -2;
    }


    // Set the init time stamp and init state
    InitTimeStamp=getTimeStamp();
    if(TheMsfStorageCore->addElement(InitTimeStamp, InitState))
    {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
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

    // Change flag in the inputs
    for(std::list< std::shared_ptr<InputCore> >::iterator itListOfInput=TheListOfInputCore.begin();
        itListOfInput!=TheListOfInputCore.end();
        ++itListOfInput)
    {
        (*itListOfInput)->setInputEnabled(predictEnabled);
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

int MsfLocalizationCore::findInputCommands(const TimeStamp TheTimeStamp,
                                           //const std::shared_ptr<StateEstimationCore> ThePreviousState,
                                           std::shared_ptr<InputCommandComponent>& input_command)
{
    // Checks
//    if(!ThePreviousState)
//        return -1;

    // Create Pointer
    if(!input_command)
        input_command=std::make_shared<InputCommandComponent>();

    // Clear the list -> Just in case
    input_command->TheListInputCommandCore.clear();

    // Iterate over the input cores
    for(std::list<std::shared_ptr<InputCore>>::iterator itInputCore=this->TheListOfInputCore.begin();
        itInputCore!=this->TheListOfInputCore.end();
        ++itInputCore)
    {
        /*
        // Find the input command in ThePreviousState if exist
        bool flag_input_command_already_set=false;
        for(std::list<std::shared_ptr<InputCommandCore>>::iterator itInputCommand=ThePreviousState->TheListInputCommandCore.begin();
            itInputCommand!=ThePreviousState->TheListInputCommandCore.end();
            ++itInputCommand)
        {
            if((*itInputCommand)->getInputCoreSharedPtr() == (*itInputCore))
            {
                flag_input_command_already_set=true;
                input_command->TheListInputCommandCore.push_back((*itInputCommand));
                break;
            }
        }

        // Input Command Already Set
        if(flag_input_command_already_set)
            continue;
        */

        // Search the last input command in the buffer
        std::shared_ptr<InputCommandCore> input_command_core;
        int error_get_previous_input_command=TheMsfStorageCore->getPreviousInputCommandByStampAndInputCore(TheTimeStamp, *itInputCore, input_command_core);

        if(error_get_previous_input_command)
        {
            //std::cout<<"MsfLocalizationCore::findInputCommands error_get_previous_input_command"<<std::endl;
            return error_get_previous_input_command;
        }

        if(!input_command_core)
            return -1;

        // Set in the variable ThePreviousState
        input_command->TheListInputCommandCore.push_back(input_command_core);

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

    // The predicted state -> New element to be added to the buffer
    std::shared_ptr<StateEstimationCore> PredictedState;
    PredictedState=std::make_shared<StateEstimationCore>();


    // Get the predicted element if any
    std::shared_ptr<StateEstimationCore> OldPredictedState;
    if(this->TheMsfStorageCore->getElement(TheTimeStamp, OldPredictedState))
    {
#if _DEBUG_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::predict() no predicted element found, must be created one!"<<std::endl;
            this->log(logString.str());
        }

#endif
        //return -10;
    }

    // Check if it exists
    if(!OldPredictedState)
    {
        // Nothing to do

    }
    else
    {
        // Copy elements

        // Measurements
        // std::list<std::shared_ptr<SensorMeasurementCore> > TheListMeasurementCore;
        PredictedState->TheListMeasurementCore=OldPredictedState->TheListMeasurementCore;

        // Inputs
        PredictedState->TheListInputCommandCore=OldPredictedState->TheListInputCommandCore;

        // Nothing else needed
    }

    // Release the old predicted state pointer
    OldPredictedState.reset();


    /*
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
    */


    ///// Predict Core
    int error=predictSemiCore(TheTimeStamp, PredictedState);
    if(error)
        return error;




    /////// Add element to the buffer -> Check if the element that is going to be added is not used
    if(TheMsfStorageCore->addElement(TheTimeStamp, PredictedState))
    {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
        std::cout<<"!!Error addElement"<<std::endl;
#endif
        return -2;
    }

    // Release the predicted state pointer -> Not really needed
    PredictedState.reset();


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
        logString<<"MsfLocalizationCore::predictNoAddBuffer() TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
        this->log(logString.str());
    }
#endif


    // The predicted state
    if(!ThePredictedState)
        ThePredictedState=std::make_shared<StateEstimationCore>();



    // Get the predicted element if any
    std::shared_ptr<StateEstimationCore> OldPredictedState;
    if(this->TheMsfStorageCore->getElement(TheTimeStamp, OldPredictedState))
    {
#if _DEBUG_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationROS::predictNoAddBuffer() no predicted element found, must be created one!"<<std::endl;
            this->log(logString.str());
        }

#endif
        //return -10;
    }


    // Check if exist old predicted state
    if(!OldPredictedState)
    {
        // Nothing to do
    }
    else
    {
        // Copy elements

        // Measurements
        // std::list<std::shared_ptr<SensorMeasurementCore> > TheListMeasurementCore;
        ThePredictedState->TheListMeasurementCore=OldPredictedState->TheListMeasurementCore;

        // Inputs
        ThePredictedState->TheListInputCommandCore=OldPredictedState->TheListInputCommandCore;

        // Nothing else needed
    }

    // Release the old predicted state pointer
    OldPredictedState.reset();


    /*
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
    */


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


    // Get the last predicted state from the buffer
    TimeStamp PreviousTimeStamp;
    std::shared_ptr<StateEstimationCore> PreviousState;

    if(this->getPreviousState(ThePredictedTimeStamp, PreviousTimeStamp, PreviousState))
    {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
        logFile<<"MsfLocalizationCore::predictSemiCore() error getPreviousState"<<std::endl;
#endif
        return 1;
    }


    if(!PreviousState)
    {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
        logFile<<"MsfLocalizationCore::predictSemiCore() error !PreviousState"<<std::endl;
#endif
        return 2;
    }


    // Check
    if(!PreviousState->hasState())
    {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
        logFile<<"MsfLocalizationCore::predictSemiCore() error !PreviousState->hasState()"<<std::endl;
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
        logString<<"MsfLocalizationCore::predictSemiCore() TS: sec="<<ThePredictedTimeStamp.sec<<" s; nsec="<<ThePredictedTimeStamp.nsec<<" ns"<<std::endl;
        logString<<"MsfLocalizationCore::predictSemiCore() TS prev: sec="<<PreviousTimeStamp.sec<<" s; nsec="<<PreviousTimeStamp.nsec<<" ns"<<std::endl;
        this->log(logString.str());
    }
#endif


    // Set inputs
    std::shared_ptr<InputCommandComponent> inputs;


    int error_find_input_commands = findInputCommands(ThePredictedTimeStamp,
                                                      //PreviousState,
                                                      inputs);

    if(error_find_input_commands)
    {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
        logFile<<"MsfLocalizationCore::predictSemiCore() error find_input_commands"<<std::endl;
#endif
        return error_find_input_commands;
    }




    // Predict Core
    int error=predictCore(PreviousTimeStamp, ThePredictedTimeStamp,
                          PreviousState,
                          inputs,
                          ThePredictedState);

    if(error)
        return error;


    // End
    return 0;
}




int MsfLocalizationCore::predictCore(const TimeStamp ThePreviousTimeStamp, const TimeStamp ThePredictedTimeStamp,
                                     // Previous State
                                     const std::shared_ptr<StateEstimationCore> ThePreviousState,
                                     // Inputs
                                     const std::shared_ptr<InputCommandComponent> inputCommand,
                                     // Predicted State
                                     std::shared_ptr<StateEstimationCore>& ThePredictedState)
{

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
    TimeStamp beginTimePredictCore=getTimeStamp();
#endif


    /////// State
#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::predictCore() state TS: sec="<<ThePredictedTimeStamp.sec<<" s; nsec="<<ThePredictedTimeStamp.nsec<<" ns"<<std::endl;
        this->log(logString.str());
    }
#endif



    ///// World -> Global Parameters

    {
        //
        std::shared_ptr<StateCore> predicted_state;

        // State
        int error_predict_state=ThePreviousState->TheGlobalParametersStateCore->getMsfElementCoreSharedPtr()->
                predictState(//Time
                             ThePreviousTimeStamp,
                             ThePredictedTimeStamp,
                             // Previous State
                             ThePreviousState,
                             // Input
                             inputCommand,
                             // Predicted State
                             predicted_state);
        if(error_predict_state)
        {
            std::cout<<"world: error_predict_state"<<std::endl;
            return error_predict_state;
        }


        // Jacobian Error State
        int error_predict_error_state_jacobian=ThePreviousState->TheGlobalParametersStateCore->getMsfElementCoreSharedPtr()->
                predictErrorStateJacobian(//Time
                                         ThePreviousTimeStamp,
                                         ThePredictedTimeStamp,
                                         // Previous State
                                         ThePreviousState,
                                          // Input
                                          inputCommand,
                                         // Predicted State
                                         predicted_state);
        if(error_predict_error_state_jacobian)
        {
            std::cout<<"world: error_predict_error_state_jacobian"<<std::endl;
            return error_predict_error_state_jacobian;
        }

        // Set
        ThePredictedState->TheGlobalParametersStateCore=std::dynamic_pointer_cast<GlobalParametersStateCore>(predicted_state);
    }


    ///// Robot

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
    TimeStamp beginTimePredictRobot=getTimeStamp();
#endif
    {
        //
        std::shared_ptr<StateCore> predicted_state;

        // State
        int error_predict_state=ThePreviousState->TheRobotStateCore->getMsfElementCoreSharedPtr()->
                predictState(//Time
                             ThePreviousTimeStamp,
                             ThePredictedTimeStamp,
                             // Previous State
                             ThePreviousState,
                             // Input
                             inputCommand,
                             // Predicted State
                             predicted_state);
        if(error_predict_state)
        {
            std::cout<<"robot: error_predict_state"<<std::endl;
            return error_predict_state;
        }


        // Jacobian Error State
        int error_predict_error_state_jacobian=ThePreviousState->TheRobotStateCore->getMsfElementCoreSharedPtr()->
                predictErrorStateJacobian(//Time
                                         ThePreviousTimeStamp,
                                         ThePredictedTimeStamp,
                                         // Previous State
                                         ThePreviousState,
                                          // Input
                                          inputCommand,
                                         // Predicted State
                                         predicted_state);
        if(error_predict_error_state_jacobian)
        {
            std::cout<<"robot: error_predict_error_state_jacobian"<<std::endl;
            return error_predict_error_state_jacobian;
        }

        // Set
        ThePredictedState->TheRobotStateCore=std::dynamic_pointer_cast<RobotStateCore>(predicted_state);
    }

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::predictCore() predict Robot time state and jac pred: "<<(getTimeStamp()-beginTimePredictRobot).nsec<<std::endl;
        this->log(logString.str());
    }
#endif


    ///// Inputs

    // Clean the list
    ThePredictedState->TheListInputStateCore.clear();

    // Iterate
    for(std::list< std::shared_ptr<InputStateCore> >::iterator itInput=ThePreviousState->TheListInputStateCore.begin();
        itInput!=ThePreviousState->TheListInputStateCore.end();
        ++itInput)
    {
        //
        std::shared_ptr<StateCore> predicted_state;

        // State
        int error_predict_state=(*itInput)->getMsfElementCoreSharedPtr()->
                predictState(//Time
                             ThePreviousTimeStamp,
                             ThePredictedTimeStamp,
                             // Previous State
                             ThePreviousState,
                             // Input
                             inputCommand,
                             // Predicted State
                             predicted_state);
        if(error_predict_state)
        {
            std::cout<<"input: error_predict_state"<<std::endl;
            return error_predict_state;
        }


        // Jacobian Error State
        int error_predict_error_state_jacobian=(*itInput)->getMsfElementCoreSharedPtr()->
                predictErrorStateJacobian(//Time
                                         ThePreviousTimeStamp,
                                         ThePredictedTimeStamp,
                                         // Previous State
                                         ThePreviousState,
                                         // Input
                                         inputCommand,
                                         // Predicted State
                                         predicted_state);
        if(error_predict_error_state_jacobian)
        {
            std::cout<<"input: error_predict_error_state_jacobian"<<std::endl;
            return error_predict_error_state_jacobian;
        }

        // Set
        ThePredictedState->TheListInputStateCore.push_back(std::dynamic_pointer_cast<InputStateCore>(predicted_state));
    }


    ///// Sensors
#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
    TimeStamp beginTimePredictSensors=getTimeStamp();
#endif

    // Clean the list
    ThePredictedState->TheListSensorStateCore.clear();


    // Iterate
    for(std::list< std::shared_ptr<SensorStateCore> >::iterator itSensorElement=ThePreviousState->TheListSensorStateCore.begin();
        itSensorElement!=ThePreviousState->TheListSensorStateCore.end();
        ++itSensorElement)
    {
        //
        std::shared_ptr<StateCore> predicted_state;

        // State
        int error_predict_state=(*itSensorElement)->getMsfElementCoreSharedPtr()->
                predictState(//Time
                             ThePreviousTimeStamp,
                             ThePredictedTimeStamp,
                             // Previous State
                             ThePreviousState,
                             // Input
                             inputCommand,
                             // Predicted State
                             predicted_state);
        if(error_predict_state)
        {
            std::cout<<"sensor: error_predict_state"<<std::endl;
            return error_predict_state;
        }


        // Jacobian Error State
        int error_predict_error_state_jacobian=(*itSensorElement)->getMsfElementCoreSharedPtr()->
                predictErrorStateJacobian(//Time
                                         ThePreviousTimeStamp,
                                         ThePredictedTimeStamp,
                                         // Previous State
                                         ThePreviousState,
                                         // Input
                                         inputCommand,
                                         // Predicted State
                                         predicted_state);
        if(error_predict_error_state_jacobian)
        {
            std::cout<<"sensor: error_predict_error_state_jacobian"<<std::endl;
            return error_predict_error_state_jacobian;
        }

        // Set
        ThePredictedState->TheListSensorStateCore.push_back(std::dynamic_pointer_cast<SensorStateCore>(predicted_state));
    }


#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::predictCore() predict Sensors time state and jac pred: "<<(getTimeStamp()-beginTimePredictSensors).nsec<<std::endl;
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
        std::shared_ptr<StateCore> predicted_state;

        // State
        int error_predict_state=(*itMapElement)->getMsfElementCoreSharedPtr()->
                predictState(//Time
                             ThePreviousTimeStamp,
                             ThePredictedTimeStamp,
                             // Previous State
                             ThePreviousState,
                             // Input
                             inputCommand,
                             // Predicted State
                             predicted_state);
        if(error_predict_state)
        {
            std::cout<<"map: error_predict_state"<<std::endl;
            return error_predict_state;
        }


        // Jacobian Error State
        int error_predict_error_state_jacobian=(*itMapElement)->getMsfElementCoreSharedPtr()->
                predictErrorStateJacobian(//Time
                                         ThePreviousTimeStamp,
                                         ThePredictedTimeStamp,
                                         // Previous State
                                         ThePreviousState,
                                         // Input
                                         inputCommand,
                                         // Predicted State
                                         predicted_state);
        if(error_predict_error_state_jacobian)
        {
            std::cout<<"map: error_predict_error_state_jacobian"<<std::endl;
            return error_predict_error_state_jacobian;
        }

        // Set
        ThePredictedState->TheListMapElementStateCore.push_back(std::dynamic_pointer_cast<MapElementStateCore>(predicted_state));
    }



#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::predictCore() predict Core time state and jac pred: "<<(getTimeStamp()-beginTimePredictCore).nsec<<std::endl;
        this->log(logString.str());
    }
#endif



    /////// Covariances
#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::predictCore() covariances TS: sec="<<ThePredictedTimeStamp.sec<<" s; nsec="<<ThePredictedTimeStamp.nsec<<" ns"<<std::endl;
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


    // Create and Resize the covariance Matrix
    int dimensionOfErrorState=ThePredictedState->getDimensionErrorState();
    if(!ThePredictedState->covarianceMatrix)
        ThePredictedState->covarianceMatrix=std::make_shared<Eigen::MatrixXd>();
    ThePredictedState->covarianceMatrix->resize(dimensionOfErrorState,dimensionOfErrorState);
    ThePredictedState->covarianceMatrix->setZero();



    // Aux Vars
    Eigen::SparseMatrix<double> jacobianRobotErrorState=ThePredictedState->TheRobotStateCore->getJacobianErrorStateRobot();


#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::predictCore() predict Core time cov in: "<<(getTimeStamp()-beginTimePredictCoreCov).nsec<<std::endl;
        this->log(logString.str());
    }
#endif




    /// Robot
#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::predictCore() covariance robot TS: sec="<<ThePredictedTimeStamp.sec<<" s; nsec="<<ThePredictedTimeStamp.nsec<<" ns"<<std::endl;
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

            // Fn * Qn * Fnt
            Eigen::SparseMatrix<double> covarianceErrorStateNoise=ThePredictedState->TheRobotStateCore->getJacobianErrorStateNoise()*TheRobotCore->getCovarianceNoise(DeltaTime)*ThePredictedState->TheRobotStateCore->getJacobianErrorStateNoise().transpose();

            ThePredictedState->covarianceMatrix->block(0,0,dimensionRobotErrorState,dimensionRobotErrorState)=
                    jacobianRobotErrorState*ThePreviousState->covarianceMatrix->block(0,0,dimensionRobotErrorState,dimensionRobotErrorState)*jacobianRobotErrorState.transpose() +
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
            logString<<"MsfLocalizationCore::predictCore() predict Core time cov rob: "<<(getTimeStamp()-beginTimePredictCoreCovRob).nsec<<std::endl;
            this->log(logString.str());
        }
#endif


    }


    /// Robot-Sensors
#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::predictCore() covariance robot-sensor TS: sec="<<ThePredictedTimeStamp.sec<<" s; nsec="<<ThePredictedTimeStamp.nsec<<" ns"<<std::endl;
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
        ThePredictedState->covarianceMatrix->block(WorkingInitPoint.row, WorkingInitPoint.col,dimensionRobotErrorState,dimensionSensorErrorState)=
                jacobianRobotErrorState*ThePreviousState->covarianceMatrix->block(WorkingInitPoint.row, WorkingInitPoint.col,dimensionRobotErrorState,dimensionSensorErrorState)*predictedStateSensor->getJacobianErrorStateSensor(0).transpose();


//        logFile<<"ThePredictedState->covarianceMatrix.block(WorkingInitPoint.row, WorkingInitPoint.col,dimensionRobotErrorState,dimensionSensorErrorState)"<<std::endl;
//        logFile<<ThePredictedState->covarianceMatrix.block(WorkingInitPoint.row, WorkingInitPoint.col,dimensionRobotErrorState,dimensionSensorErrorState)<<std::endl;



        // Set the Symmetric
        Eigen::MatrixXd auxMatSym=ThePredictedState->covarianceMatrix->block(WorkingInitPoint.row, WorkingInitPoint.col,dimensionRobotErrorState,dimensionSensorErrorState);
        ThePredictedState->covarianceMatrix->block(WorkingInitPoint.col, WorkingInitPoint.row,dimensionSensorErrorState, dimensionRobotErrorState)=
                auxMatSym.transpose();


//        logFile<<"ThePredictedState->covarianceMatrix->block(WorkingInitPoint.row, WorkingInitPoint.col,dimensionRobotErrorState,dimensionSensorErrorState)"<<std::endl;
//        logFile<<ThePredictedState->covarianceMatrix->block(WorkingInitPoint.row, WorkingInitPoint.col,dimensionRobotErrorState,dimensionSensorErrorState)<<std::endl;



        // Update Point
        WorkingInitPoint.row=0;
        WorkingInitPoint.col+=dimensionSensorErrorState;
    }

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::predictCore() predict Core time cov rob-sen: "<<(getTimeStamp()-beginTimePredictCoreCovRobSen).nsec<<std::endl;
        this->log(logString.str());
    }
#endif

//return 0;


    /// Sensors
#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::predictCore() covariance sensors TS: sec="<<ThePredictedTimeStamp.sec<<" s; nsec="<<ThePredictedTimeStamp.nsec<<" ns"<<std::endl;
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
        Eigen::MatrixXd jacobianSensor1ErrorState=predictedStateSensor1->getJacobianErrorStateSensor(0);


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
            Eigen::MatrixXd CovarianceAuxMatrix=
                    jacobianSensor1ErrorState*ThePreviousState->covarianceMatrix->block(WorkingInitPoint.row, WorkingInitPoint.col,dimensionSensor1ErrorState,dimensionSensor2ErrorState)*predictedStateSensor2->getJacobianErrorStateSensor(0).transpose();



            // Noise if the same
            if((*it1Sens) == (*it2Sens))
            {
                Eigen::SparseMatrix<double> covarianceErrorStateNoise=predictedStateSensor1->getJacobianErrorStateNoise() * (*it1Sens)->getCovarianceNoise(DeltaTime) * predictedStateSensor1->getJacobianErrorStateNoise().transpose();
                ThePredictedState->covarianceMatrix->block(WorkingInitPoint.row, WorkingInitPoint.col, dimensionSensor1ErrorState, dimensionSensor2ErrorState)=
                        CovarianceAuxMatrix+
                        Eigen::MatrixXd(covarianceErrorStateNoise);
            }


            // Value and Symmetric if different
            if((*it1Sens) != (*it2Sens))
            {
                //Eigen::MatrixXd AuxMatSym=ThePredictedState->covarianceMatrix->block(WorkingInitPoint.row, WorkingInitPoint.col, dimensionSensor1ErrorState, dimensionSensor2ErrorState).eval();
                ThePredictedState->covarianceMatrix->block(WorkingInitPoint.row, WorkingInitPoint.col, dimensionSensor1ErrorState, dimensionSensor2ErrorState)=
                        CovarianceAuxMatrix;

                ThePredictedState->covarianceMatrix->block(WorkingInitPoint.col, WorkingInitPoint.row, dimensionSensor2ErrorState, dimensionSensor1ErrorState)=
                        CovarianceAuxMatrix.transpose().eval();
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
        logString<<"MsfLocalizationCore::predictCore() predict Core time cov sen-sen: "<<(getTimeStamp()-beginTimePredictCoreCovSenSen).nsec<<std::endl;
        this->log(logString.str());
    }
#endif


    /// Robot-Map

    {
        // dimensions
        int dimensionSensorsErrorState=0;
        for(std::list< std::shared_ptr<SensorCore> >::iterator it1Sens=TheListOfSensorCore.begin();
            it1Sens!=TheListOfSensorCore.end();
            ++it1Sens)
        {
            dimensionSensorsErrorState+=(*it1Sens)->getDimensionErrorState();
        }

        // Init Point
        WorkingInitPoint.col=TheRobotCore->getDimensionErrorState()+dimensionSensorsErrorState;
        WorkingInitPoint.row=0;


        // Iterate on the map elements
        for(std::list< std::shared_ptr<MapElementCore> >::iterator it1MapElement=TheListOfMapElementCore.begin();
            it1MapElement!=TheListOfMapElementCore.end();
            ++it1MapElement)
        {

            // Auxiliar
            std::shared_ptr<MapElementCore> TheMapElementCore=(*it1MapElement);
            std::shared_ptr<MapElementStateCore> predictedStateMapElement;

            // Find
            if(findMapElementStateCoreWithMapCoreFromList(ThePredictedState->TheListMapElementStateCore, (*it1MapElement), predictedStateMapElement))
            {
//    #if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
//                std::cout<<"!!Error findMapElementStateCoreFromList Robot-Map TS: sec="<<ThePredictedTimeStamp.sec<<" s; nsec="<<ThePredictedTimeStamp.nsec<<" ns"<<std::endl;
//    #endif
                //return -2;
                // The core has been added to the the msfLocalization, but we don't have the predicted state yet. No problema
                continue;
            }

            // Dimensions
            int dimensionRobotErrorState=TheRobotCore->getDimensionErrorState();
            int dimensionMapElementErrorState=TheMapElementCore->getDimensionErrorState();


            // Calculate
            ThePredictedState->covarianceMatrix->block(WorkingInitPoint.row, WorkingInitPoint.col,dimensionRobotErrorState,dimensionMapElementErrorState)=
                    jacobianRobotErrorState*ThePreviousState->covarianceMatrix->block(WorkingInitPoint.row, WorkingInitPoint.col,dimensionRobotErrorState,dimensionMapElementErrorState)*predictedStateMapElement->getJacobianErrorStateMapElement(0).transpose();



            // Set the Symmetric
            Eigen::MatrixXd auxMatSym=ThePredictedState->covarianceMatrix->block(WorkingInitPoint.row, WorkingInitPoint.col,dimensionRobotErrorState,dimensionMapElementErrorState);
            ThePredictedState->covarianceMatrix->block(WorkingInitPoint.col, WorkingInitPoint.row,dimensionMapElementErrorState, dimensionRobotErrorState)=
                    auxMatSym.transpose();



            // Update Point
            WorkingInitPoint.row=0;
            WorkingInitPoint.col+=dimensionMapElementErrorState;
        }
    }



    /// Sensors-Map
    // TODO


    /// Map

    {
        // dimension
        int dimensionSensorsErrorState=0;
        for(std::list< std::shared_ptr<SensorCore> >::iterator it1Sens=TheListOfSensorCore.begin();
            it1Sens!=TheListOfSensorCore.end();
            ++it1Sens)
        {
            dimensionSensorsErrorState+=(*it1Sens)->getDimensionErrorState();
        }

        // Init Point
        WorkingInitPoint.col=TheRobotCore->getDimensionErrorState()+dimensionSensorsErrorState;
        WorkingInitPoint.row=WorkingInitPoint.col;

        // Iterate
        for(std::list< std::shared_ptr<MapElementCore> >::iterator it1MapElement=TheListOfMapElementCore.begin();
            it1MapElement!=TheListOfMapElementCore.end();
            ++it1MapElement)
        {
            // it1MapElement Auxiliar
            std::shared_ptr<MapElementStateCore> predictedStateMapElement1;

            // Find
            if(findMapElementStateCoreWithMapCoreFromList(ThePredictedState->TheListMapElementStateCore, (*it1MapElement), predictedStateMapElement1))
            {
//    #if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
//                std::cout<<"!!Error findMapElementStateCoreFromList for it1MapElement Map-Map"<<std::endl;
//    #endif
                //return -2;
                // The core has been added to the the msfLocalization, but we don't have the predicted state yet. No problema
                continue;
            }


            // Dimension mapElement1
            int dimensionMapElement1ErrorState=(*it1MapElement)->getDimensionErrorState();


            // Aux vars -> Jacobians
            Eigen::MatrixXd jacobianMapElement1ErrorState=predictedStateMapElement1->getJacobianErrorStateMapElement(0);


            // Iterate again
            for(std::list< std::shared_ptr<MapElementCore> >::iterator it2MapElement=it1MapElement;
                it2MapElement!=TheListOfMapElementCore.end();
                ++it2MapElement)
            {
                // it2MapElement Auxiliar
                std::shared_ptr<MapElementStateCore> predictedStateMapElement2;

                // Find
                if(findMapElementStateCoreWithMapCoreFromList(ThePredictedState->TheListMapElementStateCore, (*it2MapElement), predictedStateMapElement2))
                {
//    #if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
//                    std::cout<<"!!Error findMapElementStateCoreFromList for it2MapElement Map-Map"<<std::endl;
//    #endif
                    //return -2;
                    // The core has been added to the the msfLocalization, but we don't have the predicted state yet. No problema
                    continue;
                }


                // Dimension mapElement2
                int dimensionMapElement2ErrorState=(*it2MapElement)->getDimensionErrorState();


                // Calculate
                Eigen::MatrixXd CovarianceAuxMatrix=
                        jacobianMapElement1ErrorState*ThePreviousState->covarianceMatrix->block(WorkingInitPoint.row, WorkingInitPoint.col,dimensionMapElement1ErrorState,dimensionMapElement2ErrorState)*predictedStateMapElement2->getJacobianErrorStateMapElement(0).transpose();



                // Noise if the same
                if((*it1MapElement) == (*it2MapElement))
                {
                    if((*it2MapElement)->getDimensionNoise() != 0)
                    {
                        Eigen::SparseMatrix<double> covarianceErrorStateNoise=predictedStateMapElement1->getJacobianErrorStateNoise() * (*it1MapElement)->getCovarianceNoise(DeltaTime) * predictedStateMapElement1->getJacobianErrorStateNoise().transpose();

                        ThePredictedState->covarianceMatrix->block(WorkingInitPoint.row, WorkingInitPoint.col, dimensionMapElement1ErrorState, dimensionMapElement2ErrorState)=
                                CovarianceAuxMatrix+
                                Eigen::MatrixXd(covarianceErrorStateNoise);
                    }
                    else
                    {
                        ThePredictedState->covarianceMatrix->block(WorkingInitPoint.row, WorkingInitPoint.col, dimensionMapElement1ErrorState, dimensionMapElement2ErrorState)=
                                CovarianceAuxMatrix;
                    }
                }


                // Value and Symmetric if different
                if((*it1MapElement) != (*it2MapElement))
                {
                    //Eigen::MatrixXd AuxMatSym=ThePredictedState->covarianceMatrix->block(WorkingInitPoint.row, WorkingInitPoint.col, dimensionSensor1ErrorState, dimensionSensor2ErrorState).eval();
                    ThePredictedState->covarianceMatrix->block(WorkingInitPoint.row, WorkingInitPoint.col, dimensionMapElement1ErrorState, dimensionMapElement2ErrorState)=
                            CovarianceAuxMatrix;

                    ThePredictedState->covarianceMatrix->block(WorkingInitPoint.col, WorkingInitPoint.row, dimensionMapElement2ErrorState, dimensionMapElement1ErrorState)=
                            CovarianceAuxMatrix.transpose().eval();
                }


                // Update Working Point
                WorkingInitPoint.col+=dimensionMapElement2ErrorState;

            }

            // Update the dimension for the next map element
            WorkingInitPoint.row+=dimensionMapElement1ErrorState;
            WorkingInitPoint.col=WorkingInitPoint.row;

        }
    }




    /// Display
#if _DEBUG_MSF_LOCALIZATION_ALGORITHM
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::predictCore() predicted covariance for TS: sec="<<ThePredictedTimeStamp.sec<<" s; nsec="<<ThePredictedTimeStamp.nsec<<" ns"<<std::endl;
        logString<<ThePredictedState->covarianceMatrix<<std::endl;
        this->log(logString.str());
    }
#endif


#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::predictCore() predict Core time: "<<(getTimeStamp()-beginTimePredictCore).nsec<<std::endl;
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


    // Get the outdated element to do the update
    std::shared_ptr<StateEstimationCore> OldState;
    if(this->TheMsfStorageCore->getElement(TheTimeStamp, OldState))
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
    if(!OldState)
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
        logString<<"number of users of updated state="<<OldState.use_count()<<std::endl;

        this->log(logString.str());
    }
#endif

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;

        TimeStamp begin=getTimeStamp();
#endif

    while(OldState.use_count()>2)
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
        logString<<"number of users of updated state="<<OldState.use_count()<<std::endl;
        this->log(logString.str());
    }
#endif


    // Check if there are measurements
    if(OldState->TheListMeasurementCore.size()==0)
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


    // Check the Robot State Core -> Not really needed
    if(!OldState->TheRobotStateCore)
    {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
        std::cout<<"MsfLocalizationCore::update() error 11"<<std::endl;
#endif
        return 11;
    }






    // Create new updated state
    std::shared_ptr<StateEstimationCore> UpdatedState=std::make_shared<StateEstimationCore>();


    // Copy all elements in updated state


    // Measurements
    UpdatedState->TheListMeasurementCore=OldState->TheListMeasurementCore;

    // Inputs
    UpdatedState->TheListInputCommandCore=OldState->TheListInputCommandCore;


    // Covariance: Copy constructor
    UpdatedState->covarianceMatrix=std::make_shared<Eigen::MatrixXd>();
    *UpdatedState->covarianceMatrix=*OldState->covarianceMatrix;


    // Global Parameters
    //std::shared_ptr<GlobalParametersStateCore> TheGlobalParametersStateCore;
    UpdatedState->TheGlobalParametersStateCore=OldState->TheGlobalParametersStateCore;
    //UpdatedState->TheGlobalParametersStateCore=std::make_shared<GlobalParametersStateCore>();
    //*UpdatedState->TheGlobalParametersStateCore=*OldState->TheGlobalParametersStateCore;

    // Robot Stat
    //std::shared_ptr<RobotStateCore> TheRobotStateCore;
    UpdatedState->TheRobotStateCore=OldState->TheRobotStateCore;
//    UpdatedState->TheRobotStateCore=std::make_shared<RobotStateCore>();
//    *UpdatedState->TheRobotStateCore=*OldState->TheRobotStateCore;

    // Input State
    UpdatedState->TheListInputStateCore=OldState->TheListInputStateCore;

    // Sensors State
    //std::list< std::shared_ptr<SensorStateCore> > TheListSensorStateCore;
    UpdatedState->TheListSensorStateCore=OldState->TheListSensorStateCore;

    // Map State
    //std::list< std::shared_ptr<MapElementStateCore> > TheListMapElementStateCore;
    UpdatedState->TheListMapElementStateCore=OldState->TheListMapElementStateCore;




    ///// Update Core
    if(this->updateCore(TheTimeStamp, OldState, UpdatedState))
    {
        std::cout<<"Error updating state"<<std::endl;
        return -10;
    }


    // Release old state -> Not really needed
    if(OldState)
        OldState.reset();



    /////// Add element to the buffer -> Check that nobody is using the OldState because is is going to be overwritten
    if(TheMsfStorageCore->addElement(TheTimeStamp, UpdatedState))
    {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
        std::cout<<"!!Error addElement"<<std::endl;
#endif
        return -2;
    }


    // Release update state -> Not really needed
    if(UpdatedState)
        UpdatedState.reset();


#if _DEBUG_MSF_LOCALIZATION_CORE
    std::ostringstream logString;
    logString<<"MsfLocalizationCore::update() ended TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
    this->log(logString.str());
#endif

    // End
    return 0;
}



int MsfLocalizationCore::updateCore(TimeStamp TheTimeStamp, std::shared_ptr<StateEstimationCore> OldState, std::shared_ptr<StateEstimationCore>& UpdatedState)
{

    //std::cout<<"update started"<<std::endl;


//return 0;

    // Iterative EKF Vars and preparation
    bool iterativeEkfEnabled=false;
    bool iterativeEkfEnd=false;

    int numMaxIterations=5;
    int numIterations=0;

    double toleranceDistanceMahalanobis=1e-10;



    // Mahalanobis Distance
    double mahalanobisDistanceOld=std::numeric_limits<double>::infinity();


    // Loop
    while(!iterativeEkfEnd)
    {

        // Iterative EKF
        if(!iterativeEkfEnabled)
        {
            iterativeEkfEnd=true;
        }



    ///// Measurement prediction and matching

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
    TimeStamp beginMeasurementPrediction=getTimeStamp();
#endif


    // Predicted measurements
    std::shared_ptr<SensorMeasurementComponent> predicted_sensor_measurements=std::make_shared<SensorMeasurementComponent>();
    // Matched measurements
    std::shared_ptr<SensorMeasurementComponent> matched_sensor_measurements=std::make_shared<SensorMeasurementComponent>();

    // Unmatched measurements
    std::shared_ptr<SensorMeasurementComponent> unmatched_sensor_measurements=std::make_shared<SensorMeasurementComponent>();



    // Lists
    // TODO REMOVE!
    std::list<std::shared_ptr<SensorMeasurementCore> > TheListPredictedMeasurements;
    TheListPredictedMeasurements.resize(0); // Not needed, just in case
    std::list<std::shared_ptr<SensorMeasurementCore> > TheListMatchedMeasurements;
    TheListMatchedMeasurements.resize(0); // Not needed, just in case

    std::list<std::shared_ptr<SensorMeasurementCore> > TheListUnmatchedMeasurements;
    TheListUnmatchedMeasurements.resize(0); // Not needed, just in case



    // Predict Measurements and Jacobians
    for(std::list<std::shared_ptr<SensorMeasurementCore> >::iterator itListMeas=UpdatedState->TheListMeasurementCore.begin();
        itListMeas!=UpdatedState->TheListMeasurementCore.end();
        ++itListMeas)
    {
        // Predicted Measurement
        std::shared_ptr<SensorMeasurementCore> predicted_sensor_measurement;

        // Predict Measurements
        int error_predict_measurement=(*itListMeas)->getSensorCoreSharedPtr()->
                predictMeasurement(// Time
                                   TheTimeStamp,
                                   // Current State
                                   OldState,
                                   // Measurement to match
                                   (*itListMeas),
                                   // Predicted Measurements
                                   predicted_sensor_measurement);

        if(error_predict_measurement<0)
        {
            // Error
            std::cout<<"error_predict_measurement"<<std::endl;
            return error_predict_measurement;
        }
        else if(error_predict_measurement>0)
        {
            // Need to be mapped
            // Add to the list of unmatched
            unmatched_sensor_measurements->list_sensor_measurement_core_.push_back((*itListMeas));
            TheListUnmatchedMeasurements.push_back((*itListMeas));
            // Continue
            continue;
        }


        // Predict Jacobians
        int error_predict_error_measurement_jacobian=(*itListMeas)->getSensorCoreSharedPtr()->
                predictErrorMeasurementJacobian(// Time
                                               TheTimeStamp,
                                               // Current State
                                               OldState,
                                               // Measurement to match
                                               (*itListMeas),
                                               // Predicted Measurements
                                               predicted_sensor_measurement);

        if(error_predict_error_measurement_jacobian)
        {
            std::cout<<"error_predict_error_measurement_jacobian"<<std::endl;
            return error_predict_error_measurement_jacobian;
        }


        // Add to the list of matched Measurements
        matched_sensor_measurements->list_sensor_measurement_core_.push_back((*itListMeas));
        TheListMatchedMeasurements.push_back((*itListMeas));
        // Add to the list of predicted measurements
        predicted_sensor_measurements->list_sensor_measurement_core_.push_back(predicted_sensor_measurement);
        TheListPredictedMeasurements.push_back(predicted_sensor_measurement);

    }


#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::update() measurement prediction time: "<<(getTimeStamp()-beginMeasurementPrediction).nsec<<std::endl;
        this->log(logString.str());
    }
#endif



    ///////// Get dimensions

    /// Matched Measurements
    int dimensionMeasurements=0;
    for(std::list<std::shared_ptr<SensorMeasurementCore> >::const_iterator itListMatchedMeas=TheListMatchedMeasurements.begin();
        itListMatchedMeas!=TheListMatchedMeasurements.end();
        ++itListMatchedMeas)
    {
        dimensionMeasurements+=(*itListMatchedMeas)->getSensorCoreSharedPtr()->getDimensionMeasurement();
    }

    /// Matched Error Measurement
    unsigned int dimensionErrorMeasurements=0;
    for(std::list<std::shared_ptr<SensorMeasurementCore> >::const_iterator itListMatchedMeas=TheListMatchedMeasurements.begin();
        itListMatchedMeas!=TheListMatchedMeasurements.end();
        ++itListMatchedMeas)
    {
        dimensionErrorMeasurements+=(*itListMatchedMeas)->getSensorCoreSharedPtr()->getDimensionErrorMeasurement();
    }

    /// Error State
    int dimensionErrorState=OldState->getDimensionErrorState();


#if _DEBUG_MSF_LOCALIZATION_ALGORITHM
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::update() dimension error state="<<dimensionErrorState<<" for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
        this->log(logString.str());
    }
#endif

    /// Error Parameters
    int dimensionErrorParameters=OldState->getDimensionErrorParameters();

    /// Global Parameters
    unsigned int dimensionGlobalParameters=0;
    dimensionGlobalParameters=OldState->TheGlobalParametersStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorParameters();

    /// Sensor Parameters
    unsigned int dimensionSensorParameters=0;
    for(std::list<std::shared_ptr<SensorMeasurementCore> >::const_iterator itListMatchedMeas=TheListMatchedMeasurements.begin();
        itListMatchedMeas!=TheListMatchedMeasurements.end();
        ++itListMatchedMeas)
    {
        dimensionSensorParameters+=(*itListMatchedMeas)->getSensorCoreSharedPtr()->getDimensionErrorParameters();
    }

    /// Map parameters
    unsigned int dimensionMapErrorParameters=0;
    for(std::list<std::shared_ptr<SensorMeasurementCore> >::const_iterator itListMatchedMeas=TheListMatchedMeasurements.begin();
        itListMatchedMeas!=TheListMatchedMeasurements.end();
        ++itListMatchedMeas)
    {
        switch((*itListMatchedMeas)->getSensorCoreSharedPtr()->getSensorType())
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
                if(findMapElementStateCoreWithMeasurementFromList(OldState->TheListMapElementStateCore, (*itListMatchedMeas), TheMapElementStateCore))
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
                dimensionMapErrorParameters+=TheMapElementStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorParameters();


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
            unsigned int dimensionErrorMeasurementSensorI=(*itListMatchedMeas)->getSensorCoreSharedPtr()->getDimensionErrorMeasurement();


            innovationVector.block(dimension, 0, dimensionErrorMeasurementSensorI, 1)=(*itListMatchedMeas)->getInnovation((*itListMatchedMeas), (*itListPredictedMeas));


            dimension+=dimensionErrorMeasurementSensorI;
        }
    }


#if _DEBUG_MSF_LOCALIZATION_ALGORITHM
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
            int dimensionErrorMeasurementI=(*itListPredictedMeas)->getSensorCoreSharedPtr()->getDimensionErrorMeasurement();
            int dimensionTotalErrorStateI=0;
            int dimensionErrorStateI=0;

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
                if((*itListSensorCore) == (*itListPredictedMeas)->getSensorCoreSharedPtr())
                {
                    if(dimensionErrorMeasurementI && dimensionErrorStateI)
                        JacobianMeasurementErrorState.block(dimensionTotalMeasurementI, dimensionTotalErrorStateI, dimensionErrorMeasurementI, dimensionErrorStateI)=(*itListPredictedMeas)->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState;
                }
                else
                {
                    // Do nothing -> Set Zeros (already set)
                }
                dimensionTotalErrorStateI+=dimensionErrorStateI;
            }

            // Inputs
            // TODO


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
                                    if(dimensionErrorMeasurementI && dimensionErrorStateI)
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

#if _DEBUG_MSF_LOCALIZATION_ALGORITHM
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
            unsigned int dimensionErrorMeasurementI=(*itListPredictedMeas)->getSensorCoreSharedPtr()->getDimensionErrorMeasurement();
            unsigned int dimensionTotalMeasurementJ=0;

            for(std::list< std::shared_ptr<SensorMeasurementCore> >::const_iterator itListMatchedMeas=TheListMatchedMeasurements.begin();
                itListMatchedMeas!=TheListMatchedMeasurements.end();
                ++itListMatchedMeas)
            {
                unsigned int dimensionErrorMeasurementJ=(*itListMatchedMeas)->getSensorCoreSharedPtr()->getDimensionErrorMeasurement();
                if((*itListMatchedMeas)->getSensorCoreSharedPtr() == (*itListPredictedMeas)->getSensorCoreSharedPtr())
                {
                    if(dimensionErrorMeasurementI && dimensionErrorMeasurementJ)
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

#if _DEBUG_MSF_LOCALIZATION_ALGORITHM
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
            unsigned int dimensionErrorMeasurementI=(*itListPredictedMeas)->getSensorCoreSharedPtr()->getDimensionErrorMeasurement();
            unsigned int dimensionGlobalParametersI=this->TheGlobalParametersCore->getDimensionErrorParameters();

            if(dimensionErrorMeasurementI && dimensionGlobalParametersI)
                JacobianMeasurementNoise.block(dimensionTotalMeasurementI, 0, dimensionErrorMeasurementI, dimensionGlobalParametersI)=(*itListPredictedMeas)->jacobianMeasurementErrorParameters.jacobianMeasurementGlobalParameters;

            dimensionTotalMeasurementI+=dimensionErrorMeasurementI;
        }
    }

#if _DEBUG_MSF_LOCALIZATION_ALGORITHM
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
            unsigned int dimensionErrorMeasurementI=(*itListPredictedMeas)->getSensorCoreSharedPtr()->getDimensionErrorMeasurement();
            unsigned int dimensionTotalSensorParametersJ=0;

            for(std::list< std::shared_ptr<SensorMeasurementCore> >::const_iterator itListMatchedMeas=TheListMatchedMeasurements.begin();
                itListMatchedMeas!=TheListMatchedMeasurements.end();
                ++itListMatchedMeas)
            {
                unsigned int dimensionSensorParametersJ=(*itListMatchedMeas)->getSensorCoreSharedPtr()->getDimensionErrorParameters();
                if((*itListMatchedMeas)->getSensorCoreSharedPtr() == (*itListPredictedMeas)->getSensorCoreSharedPtr())
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


#if _DEBUG_MSF_LOCALIZATION_ALGORITHM
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


#if _DEBUG_MSF_LOCALIZATION_ALGORITHM
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
            unsigned int dimensionErrorMeasurementI=(*itListPredictedMeas)->getSensorCoreSharedPtr()->getDimensionErrorMeasurement();
            unsigned int dimensionTotalMeasurementJ=0;

            for(std::list< std::shared_ptr<SensorMeasurementCore> >::const_iterator itListMatchedMeas=TheListMatchedMeasurements.begin();
                itListMatchedMeas!=TheListMatchedMeasurements.end();
                ++itListMatchedMeas)
            {
                unsigned int dimensionErrorMeasurementJ=(*itListMatchedMeas)->getSensorCoreSharedPtr()->getDimensionErrorMeasurement();
                if((*itListMatchedMeas)->getSensorCoreSharedPtr() == (*itListPredictedMeas)->getSensorCoreSharedPtr())
                {
                    if(dimensionErrorMeasurementJ)
                        CovarianceMeasurement.block(dimensionTotalMeasurementI, dimensionTotalMeasurementJ, dimensionErrorMeasurementI, dimensionErrorMeasurementJ)=
                                Eigen::MatrixXd((*itListPredictedMeas)->getSensorCoreSharedPtr()->getCovarianceMeasurement());
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


#if _DEBUG_MSF_LOCALIZATION_ALGORITHM
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


#if _DEBUG_MSF_LOCALIZATION_ALGORITHM
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
            unsigned int dimensionSensorParametersI=(*itListMatchedMeasI)->getSensorCoreSharedPtr()->getDimensionErrorParameters();
            unsigned int dimensionTotalSensorParametersJ=0;

            for(std::list< std::shared_ptr<SensorMeasurementCore> >::const_iterator itListMatchedMeasJ=TheListMatchedMeasurements.begin();
                itListMatchedMeasJ!=TheListMatchedMeasurements.end();
                ++itListMatchedMeasJ)
            {
                unsigned int dimensionSensorParametersJ=(*itListMatchedMeasJ)->getSensorCoreSharedPtr()->getDimensionErrorParameters();
                if((*itListMatchedMeasI)->getSensorCoreSharedPtr() == (*itListMatchedMeasJ)->getSensorCoreSharedPtr())
                {
                    if(dimensionSensorParametersJ)
                        CovarianceSensorParameters.block(dimensionTotalSensorParametersI, dimensionTotalSensorParametersJ, dimensionSensorParametersI, dimensionSensorParametersJ)=
                                Eigen::MatrixXd((*itListMatchedMeasI)->getSensorCoreSharedPtr()->getCovarianceParameters());
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


#if _DEBUG_MSF_LOCALIZATION_ALGORITHM
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::update() OldState->covarianceMatrix for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
        logString<<OldState->covarianceMatrix<<std::endl;
        this->log(logString.str());
    }
#endif


#if _DEBUG_MSF_LOCALIZATION_ALGORITHM
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



#if _DEBUG_MSF_LOCALIZATION_ALGORITHM
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
    innovationCovariance=JacobianMeasurementErrorState*(*OldState->covarianceMatrix)*JacobianMeasurementErrorState.transpose()+
            JacobianMeasurementNoise*CovarianceMeasurement*JacobianMeasurementNoise.transpose()+
            JacobianMeasurementGlobalParameters*CovarianceGlobalParameters*JacobianMeasurementGlobalParameters.transpose()+
            JacobianMeasurementSensorParameters*CovarianceSensorParameters*JacobianMeasurementSensorParameters.transpose()+
            JacobianMeasurementMapParameters*CovarianceMapParameters*JacobianMeasurementMapParameters.transpose();

    Eigen::MatrixXd innovation_covariance_inverse=innovationCovariance.inverse();


#if _DEBUG_MSF_LOCALIZATION_ALGORITHM
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::update() JacobianMeasurementErrorState for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
        logString<<JacobianMeasurementErrorState<<std::endl;
        this->log(logString.str());
    }
#endif

    // TODO REMOVE RIGTH NOW
    //innovation_covariance_inverse.setZero();

#if _DEBUG_MSF_LOCALIZATION_ALGORITHM
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

    //std::cout<<"distance_mahalanobis: "<<distance_mahalanobis<<std::endl;

    // Comparation with mahalanobisDistanceOld;
//    std::cout<<"distance_mahalanobis-mahalanobisDistanceOld="<<distance_mahalanobis-mahalanobisDistanceOld<<std::endl;
    //std::cout<<"abs(distance_mahalanobis-mahalanobisDistanceOld)="<<std::abs(distance_mahalanobis-mahalanobisDistanceOld)<<std::endl;
    if(std::abs(distance_mahalanobis-mahalanobisDistanceOld)<toleranceDistanceMahalanobis)
    {
       iterativeEkfEnd=true;
       //std::cout<<"num iter ekf: "<<numIterations<<std::endl;
    }
    else
       mahalanobisDistanceOld=distance_mahalanobis;


#if _DEBUG_MSF_LOCALIZATION_ALGORITHM
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
    kalmanGain=(*OldState->covarianceMatrix)*JacobianMeasurementErrorState.transpose()*innovation_covariance_inverse;


#if _DEBUG_MSF_LOCALIZATION_ALGORITHM
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


#if _DEBUG_MSF_LOCALIZATION_ALGORITHM
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::update() Increment Delta State vector for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
        logString<<incrementErrorState.transpose()<<std::endl;
        this->log(logString.str());
    }
#endif



    // Error Reset Matrixes
    Eigen::Matrix3d G_update_theta_robot=Eigen::Matrix3d::Identity(3,3);



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

            // Ojo, signo cambiado por la definicion de incrementError!
            G_update_theta_robot-=Quaternion::skewSymMat(0.5*incrementErrorStateRobot.block<3,1>(9,0));

        }
        dimension+=dimensionRobotErrorState;

#if _DEBUG_MSF_LOCALIZATION_ALGORITHM
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::update() incrementDeltaStateRobot for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
            logString<<incrementErrorStateRobot.transpose()<<std::endl;
            logString<<"G_update_theta_robot="<<std::endl;
            logString<<G_update_theta_robot<<std::endl;
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

#if _DEBUG_MSF_LOCALIZATION_ALGORITHM
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
            unsigned int dimensionSensorErrorState=(*itListSensorState)->getMsfElementCoreSharedPtr()->getDimensionErrorState();
            Eigen::VectorXd incrementErrorStateSensor;
            if(dimensionSensorErrorState)
            {
                incrementErrorStateSensor=incrementErrorState.block(dimension, 0, dimensionSensorErrorState, 1);
                (*itListSensorState)->updateStateFromIncrementErrorState(incrementErrorStateSensor);
            }
            dimension+=dimensionSensorErrorState;

#if _DEBUG_MSF_LOCALIZATION_ALGORITHM
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
            unsigned int dimensionMapElementErrorState=(*itListMapElementState)->getMsfElementCoreSharedPtr()->getDimensionErrorState();
            Eigen::VectorXd incrementErrorStateMapElement;
            if(dimensionMapElementErrorState)
            {
                incrementErrorStateMapElement=incrementErrorState.block(dimension, 0, dimensionMapElementErrorState, 1);
                (*itListMapElementState)->updateStateFromIncrementErrorState(incrementErrorStateMapElement);
            }
            dimension+=dimensionMapElementErrorState;

#if _DEBUG_MSF_LOCALIZATION_ALGORITHM
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

    //Eigen::MatrixXd oldUpdatedCovarianceMatrix=OldState->covarianceMatrix;
    //UpdatedState->covarianceMatrix=OldState->covarianceMatrix;




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


    Eigen::MatrixXd AuxiliarMatrix(dimensionErrorState, dimensionErrorState);
    AuxiliarMatrix=Eigen::MatrixXd::Identity(dimensionErrorState, dimensionErrorState)-kalmanGain*JacobianMeasurementErrorState;

    // OP1
    *UpdatedState->covarianceMatrix=
            AuxiliarMatrix*(*OldState->covarianceMatrix)*AuxiliarMatrix.transpose()+kalmanGain*innovationCovariance*kalmanGain.transpose();

    // OP2
//    UpdatedState->covarianceMatrix=
//            AuxiliarMatrix*OldState->covarianceMatrix;


//    UpdatedState->covarianceMatrix=OldState->covarianceMatrix+
//            OldState->covarianceMatrix*JacobianMeasurementErrorState.transpose()*( -innovation_covariance_inverse + innovation_covariance_inverse* JacobianMeasurementErrorState*OldState->covarianceMatrix*JacobianMeasurementErrorState.transpose()  *innovation_covariance_inverse.transpose()  )*JacobianMeasurementErrorState*OldState->covarianceMatrix;

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



#if _DEBUG_MSF_LOCALIZATION_ALGORITHM
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::update() updated covariance for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
        logString<<*UpdatedState->covarianceMatrix<<std::endl;
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



    ////////////////////////
    ///// Reset Error State
    ////////////////////////

    // Aux Covariance
    Eigen::MatrixXd AuxiliarCovarianceMatrix(dimensionErrorState, dimensionErrorState);
    AuxiliarCovarianceMatrix.setZero();

    AuxiliarCovarianceMatrix=*UpdatedState->covarianceMatrix;



    // Robot
    // TODO FIX!
    UpdatedState->covarianceMatrix->block<3,9>(9,0)=
            G_update_theta_robot*AuxiliarCovarianceMatrix.block<3,9>(9,0);

    UpdatedState->covarianceMatrix->block<9,3>(0,9)=
            AuxiliarCovarianceMatrix.block<9,3>(0,9)*G_update_theta_robot.transpose();


    UpdatedState->covarianceMatrix->block(12,9,3,dimensionErrorState-12)=
            G_update_theta_robot*AuxiliarCovarianceMatrix.block(12,9,3,dimensionErrorState-12);

    UpdatedState->covarianceMatrix->block(9,12,dimensionErrorState-12,3)=
            AuxiliarCovarianceMatrix.block(9,12,dimensionErrorState-12,3)*G_update_theta_robot.transpose();


    UpdatedState->covarianceMatrix->block<3,3>(9,9)=
            G_update_theta_robot*AuxiliarCovarianceMatrix.block<3,3>(9,9)*G_update_theta_robot.transpose();


    // Map
    // TODO



    /*
    {
        unsigned int dimension=0;

        // Robot
        unsigned int dimensionRobotErrorState=this->TheRobotCore->getDimensionErrorState();

        if(dimensionRobotErrorState)
        {

        }
        dimension+=dimensionRobotErrorState;


        // Global Parameters
        unsigned int dimensionGlobalParametersErrorState=this->TheGlobalParametersCore->getDimensionErrorState();

        if(dimensionGlobalParametersErrorState)
        {

        }
        dimension+=dimensionGlobalParametersErrorState;


        // Sensors
        for(std::list< std::shared_ptr<SensorStateCore> >::iterator itListSensorState=UpdatedState->TheListSensorStateCore.begin();
            itListSensorState!=UpdatedState->TheListSensorStateCore.end();
            ++itListSensorState)
        {
            unsigned int dimensionSensorErrorState=(*itListSensorState)->getTheSensorCore()->getDimensionErrorState();

            if(dimensionSensorErrorState)
            {

            }
            dimension+=dimensionSensorErrorState;


        }


        // Map
        for(std::list< std::shared_ptr<MapElementStateCore> >::iterator itListMapElementState=UpdatedState->TheListMapElementStateCore.begin();
            itListMapElementState!=UpdatedState->TheListMapElementStateCore.end();
            ++itListMapElementState)
        {
            unsigned int dimensionMapElementErrorState=(*itListMapElementState)->getTheMapElementCore()->getDimensionErrorState();

            if(dimensionMapElementErrorState)
            {

            }
            dimension+=dimensionMapElementErrorState;

        }



    }



    // Update Covariance Matrix
    UpdatedState->covarianceMatrix=AuxiliarCovarianceMatrix;

*/



    ////////////////////////
    ///// Mapping new elements based on unmatched measurements
    /////////////////////////

    if(TheListUnmatchedMeasurements.size() != 0)
    {

        //std::cout<<"mapping init for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;

#if _DEBUG_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::update() mapping new element for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
            this->log(logString.str());
        }
#endif


        // Aux lists
        std::list< std::shared_ptr<MapElementStateCore> > TheListNewMapElementsStateCore;
        TheListNewMapElementsStateCore.resize(0); // Not needed but just in case


        /// New Map Elements State Prediction

        // Iterate over the unmatched measurements
        std::list< std::shared_ptr<SensorMeasurementCore> > TheListUnmatchedMeasurementsWithMapElement;
        TheListUnmatchedMeasurementsWithMapElement.resize(0); // Not needed, but just in case
        for(std::list<std::shared_ptr<SensorMeasurementCore>>::iterator itUnmatchedMeas=TheListUnmatchedMeasurements.begin();
            itUnmatchedMeas!=TheListUnmatchedMeasurements.end();
            ++itUnmatchedMeas)
        {
            // Aux vars
            std::shared_ptr<MapElementStateCore> TheNewMapElementStateCore;

            // Map measurement
            int error_map_measurement=(*itUnmatchedMeas)->getSensorCoreSharedPtr()->
                    mapMeasurement(// Time
                                   TheTimeStamp,
                                   // Current State
                                   UpdatedState,
                                   // Current Measurement
                                   (*itUnmatchedMeas),
                                   // List Map Element Core -> New will be added if not available
                                   this->TheListOfMapElementCore,
                                   // New Map Element State Core
                                   TheNewMapElementStateCore);

            if(error_map_measurement)
                return -1;

            // Jacobians map measurements
            int error_jacobians_map_measurement=(*itUnmatchedMeas)->getSensorCoreSharedPtr()->
                    jacobiansMapMeasurement(// Time
                                   TheTimeStamp,
                                   // Current State
                                   UpdatedState,
                                   // Current Measurement
                                   (*itUnmatchedMeas),
                                   // New Map Element State Core
                                   TheNewMapElementStateCore);

            if(error_jacobians_map_measurement)
                return -1;


            // Push into the lists
            TheListUnmatchedMeasurementsWithMapElement.push_back((*itUnmatchedMeas));
            TheListNewMapElementsStateCore.push_back(TheNewMapElementStateCore);

        }




        /// New Map Elements Covariance Prediction


        // Dimensions

        // Dimension Error State
        int dimension_error_state_total=OldState->getDimensionErrorState();


#if _DEBUG_MSF_LOCALIZATION_ALGORITHM
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::update() dimension error state="<<dimension_error_state_total<<" for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
            this->log(logString.str());
        }
#endif


        // Dimension New Map Elements Error State
        int dimension_new_map_elements_error_state_total=0;
        for(std::list< std::shared_ptr<MapElementStateCore> >::iterator itNewMapElementsState=TheListNewMapElementsStateCore.begin();
            itNewMapElementsState!=TheListNewMapElementsStateCore.end();
            ++itNewMapElementsState)
        {
            dimension_new_map_elements_error_state_total+=(*itNewMapElementsState)->getMsfElementCoreSharedPtr()->getDimensionErrorState();
        }


        // Dimension New Map Elements Noise Error State
        int dimension_new_map_elements_noise_measurement_total=0;
        for(std::list<std::shared_ptr<SensorMeasurementCore>>::iterator itUnmatchedMeas=TheListUnmatchedMeasurementsWithMapElement.begin();
            itUnmatchedMeas!=TheListUnmatchedMeasurementsWithMapElement.end();
            ++itUnmatchedMeas)
        {
            dimension_new_map_elements_noise_measurement_total+=(*itUnmatchedMeas)->getSensorCoreSharedPtr()->getDimensionErrorMeasurement();
        }


#if 1 || _DEBUG_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            //logString<<"MsfLocalizationCore::update() dimension_new_map_elements_error_state_total="<<dimension_new_map_elements_error_state_total<<" for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
            logString<<"MsfLocalizationCore::update() dimension_new_map_elements_noise_measurement_total="<<dimension_new_map_elements_noise_measurement_total<<" for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
            this->log(logString.str());
        }
 #endif


        // Jacobian Error-State

        Eigen::MatrixXd jacobianMapErrorState(dimension_new_map_elements_error_state_total, dimension_error_state_total);
        jacobianMapErrorState.setZero();

        // Fill
        {
            int dimension_new_map_element_error_state_total_i=0;
            std::list< std::shared_ptr<MapElementStateCore> >::iterator itNewMapElementsState;
            std::list< std::shared_ptr<SensorMeasurementCore> >::iterator itUnmatchedMeasurementsWithMapElement;
            for(itNewMapElementsState=TheListNewMapElementsStateCore.begin(), itUnmatchedMeasurementsWithMapElement=TheListUnmatchedMeasurementsWithMapElement.begin();
                itNewMapElementsState!=TheListNewMapElementsStateCore.end() || itUnmatchedMeasurementsWithMapElement!=TheListUnmatchedMeasurementsWithMapElement.end();
                ++itNewMapElementsState, ++itUnmatchedMeasurementsWithMapElement)
            {
                int dimension_new_map_element_error_state_i=(*itNewMapElementsState)->getMsfElementCoreSharedPtr()->getDimensionErrorState();
                int dimension_error_state_total_i=0;
                int dimension_error_state_i=0;


                // Robot
                dimension_error_state_i=this->TheRobotCore->getDimensionErrorState();
                if(dimension_error_state_i)
                    jacobianMapErrorState.block(dimension_new_map_element_error_state_total_i, dimension_error_state_total_i, dimension_new_map_element_error_state_i, dimension_error_state_i)=
                            (*itNewMapElementsState)->getJacobianMappingRobotErrorState();
                dimension_error_state_total_i+=dimension_error_state_i;


                // Global Parameters
                dimension_error_state_i=this->TheGlobalParametersCore->getDimensionErrorState();
                if(dimension_error_state_i)
                    jacobianMapErrorState.block(dimension_new_map_element_error_state_total_i, dimension_error_state_total_i, dimension_new_map_element_error_state_i, dimension_error_state_i)=
                            (*itNewMapElementsState)->getJacobianMappingGlobalParametersErrorState();
                dimension_error_state_total_i+=dimension_error_state_i;


                // Sensors
                for(std::list< std::shared_ptr<SensorCore> >::const_iterator itListSensorCore=this->TheListOfSensorCore.begin();
                    itListSensorCore!=this->TheListOfSensorCore.end();
                    ++itListSensorCore)
                {
                    dimension_error_state_i=(*itListSensorCore)->getDimensionErrorState();

                    if((*itListSensorCore) == (*itUnmatchedMeasurementsWithMapElement)->getSensorCoreSharedPtr())
                    {
                        if(dimension_new_map_element_error_state_i && dimension_error_state_i)
                            jacobianMapErrorState.block(dimension_new_map_element_error_state_total_i, dimension_error_state_total_i, dimension_new_map_element_error_state_i, dimension_error_state_i)=
                                    (*itNewMapElementsState)->getJacobianMappingSensorErrorState();
                    }
                    else
                    {
                        // Do nothing -> Set Zeros (already set)
                    }

                    dimension_error_state_total_i+=dimension_error_state_i;
                }


                // Map Elements
                for(std::list< std::shared_ptr<MapElementCore> >::const_iterator itListMapElementCore=this->TheListOfMapElementCore.begin();
                    itListMapElementCore!=this->TheListOfMapElementCore.end();
                    ++itListMapElementCore)
                {

                    dimension_error_state_i=(*itListMapElementCore)->getDimensionErrorState();

                    // Do nothing -> Set Zeros (already set)

                    // Dimension
                    dimension_error_state_total_i+=dimension_error_state_i;
                }


                // Update dimension
                dimension_new_map_element_error_state_total_i+=dimension_new_map_element_error_state_i;
            }
        }

#if _DEBUG_MSF_LOCALIZATION_ALGORITHM
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::update() jacobianMapErrorState for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
            logString<<jacobianMapErrorState<<std::endl;
            this->log(logString.str());
        }
 #endif


        // Jacobian Noise

        Eigen::MatrixXd jacobianMapNoise(dimension_new_map_elements_error_state_total, dimension_new_map_elements_noise_measurement_total);
        jacobianMapNoise.setZero();

        // Fill
        {
            int dimension_new_map_element_error_state_total_i=0;
            std::list< std::shared_ptr<MapElementStateCore> >::iterator itNewMapElementsState;
            std::list< std::shared_ptr<SensorMeasurementCore> >::iterator itUnmatchedMeasurementsWithMapElement;
            for(itNewMapElementsState=TheListNewMapElementsStateCore.begin(), itUnmatchedMeasurementsWithMapElement=TheListUnmatchedMeasurementsWithMapElement.begin();
                itNewMapElementsState!=TheListNewMapElementsStateCore.end() || itUnmatchedMeasurementsWithMapElement!=TheListUnmatchedMeasurementsWithMapElement.end();
                ++itNewMapElementsState, ++itUnmatchedMeasurementsWithMapElement)
            {
                int dimension_new_map_element_error_state_i=(*itNewMapElementsState)->getMsfElementCoreSharedPtr()->getDimensionErrorState();
                int dimension_error_measurement_total_i=0;
                int dimension_error_measurement_i=0;


                // Measurement
                for(std::list< std::shared_ptr<SensorMeasurementCore> >::iterator it2UnmatchedMeasurementsWithMapElement=TheListUnmatchedMeasurementsWithMapElement.begin();
                    it2UnmatchedMeasurementsWithMapElement!=TheListUnmatchedMeasurementsWithMapElement.end();
                    ++it2UnmatchedMeasurementsWithMapElement)
                {
                    dimension_error_measurement_i=(*it2UnmatchedMeasurementsWithMapElement)->getSensorCoreSharedPtr()->getDimensionErrorMeasurement();

                    if((*it2UnmatchedMeasurementsWithMapElement)->getSensorCoreSharedPtr() == (*itUnmatchedMeasurementsWithMapElement)->getSensorCoreSharedPtr())
                    {
                        if(dimension_new_map_element_error_state_i && dimension_error_measurement_i)
                            jacobianMapNoise.block(dimension_new_map_element_error_state_total_i, dimension_error_measurement_total_i, dimension_new_map_element_error_state_i, dimension_error_measurement_i)=
                                    (*itNewMapElementsState)->getJacobianMappingErrorStateNoise();
                    }
                    else
                    {
                        // Do nothing -> Set Zeros (already set)
                    }

                    dimension_error_measurement_total_i+=dimension_error_measurement_i;
                }


                // Update dimension
                dimension_new_map_element_error_state_total_i+=dimension_new_map_element_error_state_i;
            }
        }


#if _DEBUG_MSF_LOCALIZATION_ALGORITHM
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::update() jacobianMapNoise for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
            logString<<jacobianMapNoise<<std::endl;
            this->log(logString.str());
        }
 #endif


        // Covariance Noise

        Eigen::MatrixXd covarianceMapNewElements(dimension_new_map_elements_noise_measurement_total, dimension_new_map_elements_noise_measurement_total);
        covarianceMapNewElements.setZero();

        // Fill
        int dimension_error_measurement_total_i=0;
        for(std::list< std::shared_ptr<SensorMeasurementCore> >::iterator itUnmatchedMeasurementsWithMapElement=TheListUnmatchedMeasurementsWithMapElement.begin();
            itUnmatchedMeasurementsWithMapElement!=TheListUnmatchedMeasurementsWithMapElement.end();
            ++itUnmatchedMeasurementsWithMapElement)
        {
            int dimension_error_measurement_i=(*itUnmatchedMeasurementsWithMapElement)->getSensorCoreSharedPtr()->getDimensionErrorMeasurement();
            int dimension_error_measurement_total_j=0;
            int dimension_error_measurement_j=0;



            for(std::list< std::shared_ptr<SensorMeasurementCore> >::iterator it2UnmatchedMeasurementsWithMapElement=TheListUnmatchedMeasurementsWithMapElement.begin();
                it2UnmatchedMeasurementsWithMapElement!=TheListUnmatchedMeasurementsWithMapElement.end();
                ++it2UnmatchedMeasurementsWithMapElement)
            {
                dimension_error_measurement_j=(*it2UnmatchedMeasurementsWithMapElement)->getSensorCoreSharedPtr()->getDimensionErrorMeasurement();

                if((*it2UnmatchedMeasurementsWithMapElement)->getSensorCoreSharedPtr() == (*itUnmatchedMeasurementsWithMapElement)->getSensorCoreSharedPtr())
                {
                    if(dimension_error_measurement_i && dimension_error_measurement_j)
                        covarianceMapNewElements.block(dimension_error_measurement_total_i, dimension_error_measurement_total_j, dimension_error_measurement_i, dimension_error_measurement_j)=
                                (*itUnmatchedMeasurementsWithMapElement)->getSensorCoreSharedPtr()->getCovarianceMeasurement();
                }
                else
                {
                    // Do nothing -> Set Zeros (already set)
                }

                dimension_error_measurement_total_j+=dimension_error_measurement_j;
            }

            // Update dimension
            dimension_error_measurement_total_i+=dimension_error_measurement_i;
        }

#if _DEBUG_MSF_LOCALIZATION_ALGORITHM
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::update() covarianceMapNewElements for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
            logString<<covarianceMapNewElements<<std::endl;
            this->log(logString.str());
        }
 #endif



        /// Add State of new map elements

        // New Map Cores.
        // Already Added!


        // New Map State
        for(std::list<std::shared_ptr<MapElementStateCore>>::iterator itNewMapElementsStateCore=TheListNewMapElementsStateCore.begin();
            itNewMapElementsStateCore!=TheListNewMapElementsStateCore.end();
            ++itNewMapElementsStateCore)
        {

            UpdatedState->TheListMapElementStateCore.push_back((*itNewMapElementsStateCore));
        }





        /// State Covariance Update

        Eigen::MatrixXd covarianceUpdated(dimension_error_state_total+dimension_new_map_elements_error_state_total, dimension_error_state_total+dimension_new_map_elements_error_state_total);
        covarianceUpdated.setZero();

        //
        covarianceUpdated.block(0, 0, dimension_error_state_total, dimension_error_state_total)=*UpdatedState->covarianceMatrix;

        //
        //
        covarianceUpdated.block(dimension_error_state_total, dimension_error_state_total, dimension_new_map_elements_error_state_total, dimension_new_map_elements_error_state_total)=
                jacobianMapErrorState*(*UpdatedState->covarianceMatrix)*jacobianMapErrorState.transpose()+
                jacobianMapNoise*covarianceMapNewElements*jacobianMapNoise.transpose();

        //
        Eigen::MatrixXd covarianceUpdatedCrossElem(dimension_error_state_total, dimension_new_map_elements_error_state_total);
        covarianceUpdatedCrossElem.setZero();
        covarianceUpdatedCrossElem=(*UpdatedState->covarianceMatrix)*jacobianMapErrorState.transpose();
        covarianceUpdated.block(0, dimension_error_state_total, dimension_error_state_total, dimension_new_map_elements_error_state_total)=
                covarianceUpdatedCrossElem;


        // Sym Value
        covarianceUpdated.block(dimension_error_state_total, 0, dimension_new_map_elements_error_state_total, dimension_error_state_total)=
                covarianceUpdatedCrossElem.transpose();


        // Update
        *UpdatedState->covarianceMatrix=covarianceUpdated;


#if _DEBUG_MSF_LOCALIZATION_ALGORITHM
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::updateCore() UpdatedState->covarianceMatrix after mapping for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
            logString<<*UpdatedState->covarianceMatrix<<std::endl;
            this->log(logString.str());
        }
 #endif


        //std::cout<<"mapping ended"<<std::endl;

    }





    ////////////////////////
    //// Iterative EKF - Checks and Reset
    ///////////////////////

    // Check number iterations
    numIterations++;
    if(numIterations>=numMaxIterations)
    {
        iterativeEkfEnd=true;
        //std::cout<<"max num iter reached!"<<std::endl;
    }

    // Prepare next iteration
    if(!iterativeEkfEnd)
    {
        // OldState <- Updated State


        // Measurements
        OldState->TheListMeasurementCore=UpdatedState->TheListMeasurementCore;

        // Inputs
        OldState->TheListInputCommandCore=UpdatedState->TheListInputCommandCore;


        // Covariance
        OldState->covarianceMatrix=UpdatedState->covarianceMatrix;


        // Global Parameters
        //std::shared_ptr<GlobalParametersStateCore> TheGlobalParametersStateCore;
        OldState->TheGlobalParametersStateCore=UpdatedState->TheGlobalParametersStateCore;
        //UpdatedState->TheGlobalParametersStateCore=std::make_shared<GlobalParametersStateCore>();
        //*UpdatedState->TheGlobalParametersStateCore=*OldState->TheGlobalParametersStateCore;

        // Robot State
        //std::shared_ptr<RobotStateCore> TheRobotStateCore;
        OldState->TheRobotStateCore=UpdatedState->TheRobotStateCore;
    //    UpdatedState->TheRobotStateCore=std::make_shared<RobotStateCore>();
    //    *UpdatedState->TheRobotStateCore=*OldState->TheRobotStateCore;

        // Input State
        OldState->TheListInputStateCore=UpdatedState->TheListInputStateCore;

        // Sensors State
        //std::list< std::shared_ptr<SensorStateCore> > TheListSensorStateCore;
        OldState->TheListSensorStateCore=UpdatedState->TheListSensorStateCore;

        // Map State
        //std::list< std::shared_ptr<MapElementStateCore> > TheListMapElementStateCore;
        OldState->TheListMapElementStateCore=UpdatedState->TheListMapElementStateCore;


    }



    }




#if _DEBUG_MSF_LOCALIZATION_ALGORITHM
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::updateCore() UpdatedState->covarianceMatrix for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
            logString<<*UpdatedState->covarianceMatrix<<std::endl;
            this->log(logString.str());
        }
 #endif



    //std::cout<<"update ended"<<std::endl;


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

        if(!(*itSensState)->isCorrect())
        {
#if _DEBUG_MSF_LOCALIZATION_CORE
            std::cout<<"!!Error getting the core"<<std::endl;
#endif
            return -100;
        }


        //std::cout<<"matching previous sensor state: going to check the type"<<std::endl;
        if(std::dynamic_pointer_cast<SensorCore>((*itSensState)->getMsfElementCoreSharedPtr())->getSensorId() == TheSensorCore->getSensorId())
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


int MsfLocalizationCore::findMapElementStateCoreWithMapCoreFromList(std::list< std::shared_ptr<MapElementStateCore> > TheListMapElementStateCore, std::shared_ptr<MapElementCore> TheMapElementCore, std::shared_ptr<MapElementStateCore>& TheMapElementStateCore)
{

    // Match with the map element
    for(std::list<std::shared_ptr<MapElementStateCore>>::iterator itVisualMarkerLandmark=TheListMapElementStateCore.begin();
        itVisualMarkerLandmark!=TheListMapElementStateCore.end();
        ++itVisualMarkerLandmark)
    {

        if(std::dynamic_pointer_cast<MapElementCore>((*itVisualMarkerLandmark)->getMsfElementCoreSharedPtr()) == TheMapElementCore)
        {
            TheMapElementStateCore=(*itVisualMarkerLandmark);
            break;
        }

    }


    if(!TheMapElementStateCore)
        return 1;

    // End
    return 0;
}


int MsfLocalizationCore::findMapElementStateCoreWithMeasurementFromList(std::list< std::shared_ptr<MapElementStateCore> > TheListMapElementStateCore, std::shared_ptr<SensorMeasurementCore> TheSensorMeasurementCore, std::shared_ptr<MapElementStateCore>& TheMapElementStateCore)
{
    // Cast
    std::shared_ptr<CodedVisualMarkerMeasurementCore> the_coded_visual_marker_measurement=std::dynamic_pointer_cast<CodedVisualMarkerMeasurementCore>(TheSensorMeasurementCore);


    // Match with the map element
    for(std::list<std::shared_ptr<MapElementStateCore>>::iterator itVisualMarkerLandmark=TheListMapElementStateCore.begin();
        itVisualMarkerLandmark!=TheListMapElementStateCore.end();
        ++itVisualMarkerLandmark)
    {
        switch(std::dynamic_pointer_cast<MapElementCore>((*itVisualMarkerLandmark)->getMsfElementCoreSharedPtr())->getMapElementType())
        {
            // Coded visual markers
            case MapElementTypes::coded_visual_marker:
            {
                // Cast
                std::shared_ptr<CodedVisualMarkerLandmarkCore> the_coded_visual_marker_landmark_core=std::dynamic_pointer_cast<CodedVisualMarkerLandmarkCore>((*itVisualMarkerLandmark)->getMsfElementCoreSharedPtr());

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
                std::cout<<"MsfLocalizationCore::findMapElementStateCoreFromList() something happened"<<std::endl;
                break;
        }

    }



    return 1;
}


int MsfLocalizationCore::findMapElementCoreWithMeasurementFromList(std::list< std::shared_ptr<MapElementCore> > TheListMapElementCore, std::shared_ptr<SensorMeasurementCore> TheSensorMeasurementCore, std::shared_ptr<MapElementCore>& TheMapElementCore)
{
    // Cast
    std::shared_ptr<CodedVisualMarkerMeasurementCore> the_coded_visual_marker_measurement=std::dynamic_pointer_cast<CodedVisualMarkerMeasurementCore>(TheSensorMeasurementCore);


    // Match with the map element
    for(std::list<std::shared_ptr<MapElementCore>>::iterator itVisualMarkerLandmark=TheListMapElementCore.begin();
        itVisualMarkerLandmark!=TheListMapElementCore.end();
        ++itVisualMarkerLandmark)
    {
        switch((*itVisualMarkerLandmark)->getMapElementType())
        {
            // Coded visual markers
            case MapElementTypes::coded_visual_marker:
            {
                // Cast
                std::shared_ptr<CodedVisualMarkerLandmarkCore> the_coded_visual_marker_landmark_core=std::dynamic_pointer_cast<CodedVisualMarkerLandmarkCore>((*itVisualMarkerLandmark));

                // Check ids
                if(the_coded_visual_marker_landmark_core->getId() == the_coded_visual_marker_measurement->getVisualMarkerId())
                {
                    TheMapElementCore=*itVisualMarkerLandmark;
                    return 0;
                }

                // End
                break;
            }
            // Default
            case MapElementTypes::undefined:
            default:
                std::cout<<"MsfLocalizationCore::findMapElementStateCoreFromList() something happened"<<std::endl;
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
