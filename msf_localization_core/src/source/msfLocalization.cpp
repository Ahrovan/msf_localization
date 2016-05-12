
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
        // Dimensions
        int dimension_world_error_state=ThePredictedState->TheGlobalParametersStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorState();
        int dimension_robot_error_state=ThePredictedState->TheRobotStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorState();
        int dimension_inputs_error_state=0;
        for(std::list< std::shared_ptr<InputStateCore> >::iterator itInputState=ThePredictedState->TheListInputStateCore.begin();
            itInputState!=ThePredictedState->TheListInputStateCore.end();
            ++itInputState)
        {
            dimension_inputs_error_state+=(*itInputState)->getMsfElementCoreSharedPtr()->getDimensionErrorState();
        }
        int dimension_total_robot_error_state=dimension_world_error_state+dimension_robot_error_state+dimension_inputs_error_state;

        // Sizes
        int size_inputs=ThePredictedState->TheListInputStateCore.size();
        int size_total_robot=1+1+size_inputs;


        // Jacobians


        /// Jacobian Error State

        BlockMatrix::MatrixSparse jacobian_total_robot_error_state;
        jacobian_total_robot_error_state.resize(size_total_robot, size_total_robot);


        int jacobian_row=0;

        // World
        {
            int jacobian_column=0;

            // World / World
            jacobian_total_robot_error_state(jacobian_row,jacobian_column)=ThePredictedState->TheGlobalParametersStateCore->getJacobianErrorStateWorld();

            // World / Robot
            jacobian_total_robot_error_state(jacobian_row,jacobian_column)=ThePredictedState->TheGlobalParametersStateCore->getJacobianErrorStateRobot();

            // World / Inputs
            // TODO

        }
        jacobian_row++;

        // Robot
        {
            int jacobian_column=0;

            // Robot / World
            jacobian_total_robot_error_state(jacobian_row,jacobian_column)=ThePredictedState->TheRobotStateCore->getJacobianErrorStateWorld();

            // Robot / Robot
            jacobian_total_robot_error_state(jacobian_row,jacobian_column)=ThePredictedState->TheRobotStateCore->getJacobianErrorStateRobot();

            // Robot / Inputs
            // TODO

        }

        // Inputs
        {
            // Inputs / World

            // Inputs / Robot

            // Inputs / Inputs

        }





        /// Jacobian Error State Noise Estimation: Fn

        BlockMatrix::MatrixSparse jacobian_total_robot_error_noise_estimation;
        jacobian_total_robot_error_noise_estimation.resize(size_total_robot, size_total_robot);

        {
            // World
            jacobian_total_robot_error_noise_estimation(0,0)=ThePredictedState->TheGlobalParametersStateCore->getJacobianErrorStateNoise();

            // Robot
            jacobian_total_robot_error_noise_estimation(1,1)=ThePredictedState->TheRobotStateCore->getJacobianErrorStateNoise();

            // Inputs
            int size_input_i=0;
            for(std::list< std::shared_ptr<InputStateCore> >::iterator itInputState=ThePredictedState->TheListInputStateCore.begin();
                itInputState!=ThePredictedState->TheListInputStateCore.end();
                ++itInputState)
            {
                jacobian_total_robot_error_noise_estimation(2+size_input_i, 2+size_input_i)=(*itInputState)->getJacobianErrorStateNoise();
                size_input_i++;
            }
        }

        /// Covariance Error State Noise Estimation: Qn

        BlockMatrix::MatrixSparse covariance_total_robot_error_noise_estimation;
        covariance_total_robot_error_noise_estimation.resize(size_total_robot, size_total_robot);

        {
            // World
            covariance_total_robot_error_noise_estimation(0,0)=std::dynamic_pointer_cast<GlobalParametersCore>(ThePredictedState->TheGlobalParametersStateCore->getMsfElementCoreSharedPtr())->getCovarianceNoise(DeltaTime);

            // Robot
            covariance_total_robot_error_noise_estimation(1,1)=std::dynamic_pointer_cast<RobotCore>(ThePredictedState->TheRobotStateCore->getMsfElementCoreSharedPtr())->getCovarianceNoise(DeltaTime);

            // Inputs
            int size_input_i=0;
            for(std::list< std::shared_ptr<InputStateCore> >::iterator itInputState=ThePredictedState->TheListInputStateCore.begin();
                itInputState!=ThePredictedState->TheListInputStateCore.end();
                ++itInputState)
            {
                covariance_total_robot_error_noise_estimation(2+size_input_i, 2+size_input_i)=std::dynamic_pointer_cast<InputCore>((*itInputState)->getMsfElementCoreSharedPtr())->getCovarianceNoise(DeltaTime);
                size_input_i++;
            }
        }

        //jacobian_total_robot_error_noise_estimation.transpose();

        //jacobian_total_robot_error_noise_estimation*covariance_total_robot_error_noise_estimation*jacobian_total_robot_error_noise_estimation.transpose();


//        std::cout<<"Matrix jacobian_total_robot_error_noise_estimation:"<<std::endl;
//        for(int i=0; i<jacobian_total_robot_error_noise_estimation.rows(); i++)
//            for(int j=0; j<jacobian_total_robot_error_noise_estimation.cols(); j++)
//            {
//                std::cout<<"("<<i<<";"<<j<<")="<<std::endl<<Eigen::MatrixXd(jacobian_total_robot_error_noise_estimation(i,j))<<std::endl;
//            }

//        std::cout<<"Matrix covariance_total_robot_error_noise_estimation:"<<std::endl;
//        for(int i=0; i<covariance_total_robot_error_noise_estimation.rows(); i++)
//            for(int j=0; j<covariance_total_robot_error_noise_estimation.cols(); j++)
//            {
//                std::cout<<"("<<i<<";"<<j<<")="<<std::endl<<Eigen::MatrixXd(covariance_total_robot_error_noise_estimation(i,j))<<std::endl;
//            }



        BlockMatrix::MatrixSparse covariance_total_robot_error;
        covariance_total_robot_error.resize(size_total_robot, size_total_robot);



        //covariance_total_robot_error=jacobian_total_robot_error_noise_estimation;
        //covariance_total_robot_error=jacobian_total_robot_error_noise_estimation*covariance_total_robot_error_noise_estimation;
        //covariance_total_robot_error=jacobian_total_robot_error_noise_estimation.transpose();
        //covariance_total_robot_error=covariance_total_robot_error_noise_estimation*jacobian_total_robot_error_noise_estimation.transpose();

        //covariance_total_robot_error=jacobian_total_robot_error_noise_estimation + jacobian_total_robot_error_noise_estimation;


        covariance_total_robot_error=jacobian_total_robot_error_noise_estimation*covariance_total_robot_error_noise_estimation*jacobian_total_robot_error_noise_estimation.transpose();



//        if(covariance_total_robot_error.analyse())
//            std::cout<<"Error!"<<std::endl;

        //std::cout<<"size_cols="<<covariance_total_robot_error.getColsSize().transpose()<<std::endl;
        //std::cout<<"size_rows="<<covariance_total_robot_error.getRowsSize().transpose()<<std::endl;



//        std::cout<<"Matrix covariance_total_robot_error:"<<std::endl;
//        for(int i=0; i<covariance_total_robot_error.rows(); i++)
//            for(int j=0; j<covariance_total_robot_error.cols(); j++)
//            {
//                std::cout<<"("<<i<<";"<<j<<")="<<std::endl<<Eigen::MatrixXd(covariance_total_robot_error(i,j))<<std::endl;
//            }



//        Eigen::SparseMatrix<double> aux=convertToEigenSparse(covariance_total_robot_error);
//        std::cout<<"full matrix="<<std::endl;
//        std::cout<<Eigen::MatrixXd(aux)<<std::endl;



#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
        TimeStamp beginTimePredictCoreCovRob=getTimeStamp();
#endif





        try
        {
            //Eigen::SparseMatrix<double> jacobianErrorStateNoise=ThePredictedState->TheRobotStateCore->getJacobianErrorStateNoise();

            // Fu * Qu * Fu^t
            // TODO


            // Fp * Qp * Fp^t
            // TODO


            // Fn * Qn * Fn^t
            Eigen::SparseMatrix<double> covarianceErrorStateNoise=ThePredictedState->TheRobotStateCore->getJacobianErrorStateNoise()*TheRobotCore->getCovarianceNoise(DeltaTime)*ThePredictedState->TheRobotStateCore->getJacobianErrorStateNoise().transpose();



            // P(k+1|k)
            ThePredictedState->covarianceMatrix->block(0,0,dimension_robot_error_state,dimension_robot_error_state)=
                    jacobianRobotErrorState*ThePreviousState->covarianceMatrix->block(0,0,dimension_robot_error_state,dimension_robot_error_state)*jacobianRobotErrorState.transpose() +
                    Eigen::MatrixXd(covarianceErrorStateNoise);



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
    WorkingInitPoint.col=ThePredictedState->TheRobotStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorState();
    WorkingInitPoint.row=0;


    // Iterate on the sensors
    for(std::list< std::shared_ptr<SensorStateCore> >::iterator it1Sens=ThePredictedState->TheListSensorStateCore.begin();
        it1Sens!=ThePredictedState->TheListSensorStateCore.end();
        ++it1Sens)
    {

        // Dimensions
        int dimensionRobotErrorState=ThePredictedState->TheRobotStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorState();
        int dimensionSensorErrorState=(*it1Sens)->getMsfElementCoreSharedPtr()->getDimensionErrorState();


        // Calculate
        ThePredictedState->covarianceMatrix->block(WorkingInitPoint.row, WorkingInitPoint.col,dimensionRobotErrorState,dimensionSensorErrorState)=
                jacobianRobotErrorState*ThePreviousState->covarianceMatrix->block(WorkingInitPoint.row, WorkingInitPoint.col,dimensionRobotErrorState,dimensionSensorErrorState)*(*it1Sens)->getJacobianErrorStateSensor(0).transpose();



        // Set the Symmetric
        Eigen::MatrixXd auxMatSym=ThePredictedState->covarianceMatrix->block(WorkingInitPoint.row, WorkingInitPoint.col,dimensionRobotErrorState,dimensionSensorErrorState);
        ThePredictedState->covarianceMatrix->block(WorkingInitPoint.col, WorkingInitPoint.row,dimensionSensorErrorState, dimensionRobotErrorState)=
                auxMatSym.transpose();


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
    WorkingInitPoint.col=ThePredictedState->TheRobotStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorState();
    WorkingInitPoint.row=WorkingInitPoint.col;

    // Iterate
    for(std::list< std::shared_ptr<SensorStateCore> >::iterator it1Sens=ThePredictedState->TheListSensorStateCore.begin();
        it1Sens!=ThePredictedState->TheListSensorStateCore.end();
        ++it1Sens)
    {

        // Dimension sensor1
        int dimensionSensor1ErrorState=(*it1Sens)->getMsfElementCoreSharedPtr()->getDimensionErrorState();


        // Aux vars -> Jacobians
        Eigen::MatrixXd jacobianSensor1ErrorState=(*it1Sens)->getJacobianErrorStateSensor(0);


        // Iterate again
        for(std::list< std::shared_ptr<SensorStateCore> >::iterator it2Sens=it1Sens;
            it2Sens!=ThePredictedState->TheListSensorStateCore.end();
            ++it2Sens)
        {

            // Dimension sensor2
            int dimensionSensor2ErrorState=(*it2Sens)->getMsfElementCoreSharedPtr()->getDimensionErrorState();


            // Calculate
            Eigen::MatrixXd CovarianceAuxMatrix=
                    jacobianSensor1ErrorState*ThePreviousState->covarianceMatrix->block(WorkingInitPoint.row, WorkingInitPoint.col,dimensionSensor1ErrorState,dimensionSensor2ErrorState)*(*it2Sens)->getJacobianErrorStateSensor(0).transpose();



            // Noise if the same
            if((*it1Sens) == (*it2Sens))
            {
                Eigen::SparseMatrix<double> covarianceErrorStateNoise=(*it1Sens)->getJacobianErrorStateNoise() * std::dynamic_pointer_cast<SensorCore>((*it1Sens)->getMsfElementCoreSharedPtr())->getCovarianceNoise(DeltaTime) * (*it1Sens)->getJacobianErrorStateNoise().transpose();
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
        for(std::list< std::shared_ptr<SensorStateCore> >::iterator it1Sens=ThePredictedState->TheListSensorStateCore.begin();
            it1Sens!=ThePredictedState->TheListSensorStateCore.end();
            ++it1Sens)
        {
            dimensionSensorsErrorState+=(*it1Sens)->getMsfElementCoreSharedPtr()->getDimensionErrorState();
        }

        // Init Point
        WorkingInitPoint.col=ThePredictedState->TheRobotStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorState()+dimensionSensorsErrorState;
        WorkingInitPoint.row=0;


        // Iterate on the map elements
        for(std::list< std::shared_ptr<MapElementStateCore> >::iterator it1MapElement=ThePredictedState->TheListMapElementStateCore.begin();
            it1MapElement!=ThePredictedState->TheListMapElementStateCore.end();
            ++it1MapElement)
        {
            // Dimensions
            int dimensionRobotErrorState=ThePredictedState->TheRobotStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorState();
            int dimensionMapElementErrorState=(*it1MapElement)->getMsfElementCoreSharedPtr()->getDimensionErrorState();


            // Calculate
            ThePredictedState->covarianceMatrix->block(WorkingInitPoint.row, WorkingInitPoint.col,dimensionRobotErrorState,dimensionMapElementErrorState)=
                    jacobianRobotErrorState*ThePreviousState->covarianceMatrix->block(WorkingInitPoint.row, WorkingInitPoint.col,dimensionRobotErrorState,dimensionMapElementErrorState)*(*it1MapElement)->getJacobianErrorStateMapElement(0).transpose();



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
        for(std::list< std::shared_ptr<SensorStateCore> >::iterator it1Sens=ThePredictedState->TheListSensorStateCore.begin();
            it1Sens!=ThePredictedState->TheListSensorStateCore.end();
            ++it1Sens)
        {
            dimensionSensorsErrorState+=(*it1Sens)->getMsfElementCoreSharedPtr()->getDimensionErrorState();
        }

        // Init Point
        WorkingInitPoint.col=ThePredictedState->TheRobotStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorState()+dimensionSensorsErrorState;
        WorkingInitPoint.row=WorkingInitPoint.col;

        // Iterate
        for(std::list< std::shared_ptr<MapElementStateCore> >::iterator it1MapElement=ThePredictedState->TheListMapElementStateCore.begin();
            it1MapElement!=ThePredictedState->TheListMapElementStateCore.end();
            ++it1MapElement)
        {

            // Dimension mapElement1
            int dimensionMapElement1ErrorState=(*it1MapElement)->getMsfElementCoreSharedPtr()->getDimensionErrorState();


            // Aux vars -> Jacobians
            Eigen::MatrixXd jacobianMapElement1ErrorState=(*it1MapElement)->getJacobianErrorStateMapElement(0);


            // Iterate again
            for(std::list< std::shared_ptr<MapElementStateCore> >::iterator it2MapElement=it1MapElement;
                it2MapElement!=ThePredictedState->TheListMapElementStateCore.end();
                ++it2MapElement)
            {

                // Dimension mapElement2
                int dimensionMapElement2ErrorState=(*it2MapElement)->getMsfElementCoreSharedPtr()->getDimensionErrorState();


                // Calculate
                Eigen::MatrixXd CovarianceAuxMatrix=
                        jacobianMapElement1ErrorState*ThePreviousState->covarianceMatrix->block(WorkingInitPoint.row, WorkingInitPoint.col,dimensionMapElement1ErrorState,dimensionMapElement2ErrorState)*(*it2MapElement)->getJacobianErrorStateMapElement(0).transpose();



                // Noise if the same
                if((*it1MapElement) == (*it2MapElement))
                {
                    if((*it2MapElement)->getMsfElementCoreSharedPtr()->getDimensionNoise() != 0)
                    {
                        Eigen::SparseMatrix<double> covarianceErrorStateNoise=(*it1MapElement)->getJacobianErrorStateNoise() * std::dynamic_pointer_cast<MapElementCore>((*it1MapElement)->getMsfElementCoreSharedPtr())->getCovarianceNoise(DeltaTime) * (*it1MapElement)->getJacobianErrorStateNoise().transpose();

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
        logString<<*ThePredictedState->covarianceMatrix<<std::endl;
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



int MsfLocalizationCore::update(const TimeStamp TheTimeStamp)
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


    // State

    // Global Parameters
    //std::shared_ptr<GlobalParametersStateCore> TheGlobalParametersStateCore;
    UpdatedState->TheGlobalParametersStateCore=OldState->TheGlobalParametersStateCore;
    //UpdatedState->TheGlobalParametersStateCore=std::make_shared<GlobalParametersStateCore>();
    //*UpdatedState->TheGlobalParametersStateCore=*OldState->TheGlobalParametersStateCore;

    // Robot State
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



int MsfLocalizationCore::updateCore(const TimeStamp TheTimeStamp, std::shared_ptr<StateEstimationCore> OldState, std::shared_ptr<StateEstimationCore>& UpdatedState)
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


        /////////////////////////////////////////
        ///// Measurement prediction and matching
        /// ////////////////////////////

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
        TimeStamp beginMeasurementPrediction=getTimeStamp();
#endif

#if _DEBUG_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::update() measurement prediction and matching for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
            this->log(logString.str());
        }
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




        /////////////////////////////////////
        ///// Correct State if any matched measurements
        //////////////////////////////////
        if(TheListMatchedMeasurements.size() > 0)
        {

            ///////// Get dimensions

            //// Measurements

            /// Matched Error Measurement
            unsigned int dimensionErrorMeasurements=0;
            for(std::list<std::shared_ptr<SensorMeasurementCore> >::const_iterator itListMatchedMeas=TheListMatchedMeasurements.begin();
                itListMatchedMeas!=TheListMatchedMeasurements.end();
                ++itListMatchedMeas)
            {
                dimensionErrorMeasurements+=(*itListMatchedMeas)->getSensorCoreSharedPtr()->getDimensionErrorMeasurement();
            }


            //// Error State

            /// Error State
            int dimensionErrorState=OldState->getDimensionErrorState();



            // Num Error State blocks
            int num_error_states=0;
            // World
            num_error_states++;
            // Robot
            num_error_states++;
            // Inputs
            num_error_states+=OldState->getNumberInputStates();
            // Sensors
            num_error_states+=OldState->getNumberSensorStates();
            // Map Elements
            num_error_states+=OldState->getNumberMapElementStates();

            // Num Measurement blocks
            int num_error_measurements=0;
            num_error_measurements=TheListMatchedMeasurements.size();



#if _DEBUG_MSF_LOCALIZATION_ALGORITHM
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::update() dimension error state="<<dimensionErrorState<<" for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
                this->log(logString.str());
            }
#endif




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



            //// Jacobians Error Measurement: H

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
            TimeStamp beginJacobians=getTimeStamp();
#endif


            /// Jacobian Error Measurement - Error State & Jacobian Error Measurements - Error Parameters
            /// Hx & Hp

            // Init and resize Block Matrix
            BlockMatrix::MatrixSparse block_jacobian_error_measurement_wrt_error_state;
            block_jacobian_error_measurement_wrt_error_state.resize(num_error_measurements, num_error_states);

            // Init and resize Block Matrix
            BlockMatrix::MatrixSparse block_jacobian_error_measurement_wrt_error_parameters;
            block_jacobian_error_measurement_wrt_error_parameters.resize(num_error_measurements, num_error_states);

            // Fill Blocks
            {
                int num_error_measurements_i=0;
                for(std::list<std::shared_ptr<SensorMeasurementCore> >::const_iterator itListPredictedMeas=TheListPredictedMeasurements.begin();
                    itListPredictedMeas!=TheListPredictedMeasurements.end();
                    ++itListPredictedMeas)
                {
                    int num_error_state_i=0;

                    // World (Global Parameters)
                    {
                        block_jacobian_error_measurement_wrt_error_state(num_error_measurements_i, num_error_state_i)=
                                (*itListPredictedMeas)->jacobian_error_measurement_wrt_error_state_.world;
                        block_jacobian_error_measurement_wrt_error_parameters(num_error_measurements_i, num_error_state_i)=
                                (*itListPredictedMeas)->jacobian_error_measurement_wrt_error_parameters_.world;
                        num_error_state_i++;
                    }

                    // Robot
                    {
                        block_jacobian_error_measurement_wrt_error_state(num_error_measurements_i, num_error_state_i)=
                                (*itListPredictedMeas)->jacobian_error_measurement_wrt_error_state_.robot;
                        block_jacobian_error_measurement_wrt_error_parameters(num_error_measurements_i, num_error_state_i)=
                                (*itListPredictedMeas)->jacobian_error_measurement_wrt_error_parameters_.robot;
                        num_error_state_i++;
                    }

                    // Inputs
                    for(int num_input_state_i=0; num_input_state_i<OldState->getNumberInputStates(); num_input_state_i++)
                    {
                        block_jacobian_error_measurement_wrt_error_state(num_error_measurements_i, num_error_state_i)=
                                (*itListPredictedMeas)->jacobian_error_measurement_wrt_error_state_.inputs[num_input_state_i];
                        block_jacobian_error_measurement_wrt_error_parameters(num_error_measurements_i, num_error_state_i)=
                                (*itListPredictedMeas)->jacobian_error_measurement_wrt_error_parameters_.inputs[num_input_state_i];
                        num_error_state_i++;
                    }

                    // Sensors
                    for(int num_sensor_state_i=0; num_sensor_state_i<OldState->getNumberSensorStates(); num_sensor_state_i++)
                    {
                        block_jacobian_error_measurement_wrt_error_state(num_error_measurements_i, num_error_state_i)=
                                (*itListPredictedMeas)->jacobian_error_measurement_wrt_error_state_.sensors[num_sensor_state_i];
                        block_jacobian_error_measurement_wrt_error_parameters(num_error_measurements_i, num_error_state_i)=
                                (*itListPredictedMeas)->jacobian_error_measurement_wrt_error_parameters_.sensors[num_sensor_state_i];
                        num_error_state_i++;
                    }

                    // Map
                    for(int num_map_element_state_i=0; num_map_element_state_i<OldState->getNumberMapElementStates(); num_map_element_state_i++)
                    {
                        block_jacobian_error_measurement_wrt_error_state(num_error_measurements_i, num_error_state_i)=
                                (*itListPredictedMeas)->jacobian_error_measurement_wrt_error_state_.map_elements[num_map_element_state_i];
                        block_jacobian_error_measurement_wrt_error_parameters(num_error_measurements_i, num_error_state_i)=
                                (*itListPredictedMeas)->jacobian_error_measurement_wrt_error_parameters_.map_elements[num_map_element_state_i];
                        num_error_state_i++;
                    }

                    // Update num error measurement
                    num_error_measurements_i++;
                }
            }

            // Sparse Matrix from block matrix
            Eigen::SparseMatrix<double> jacobian_error_measurement_wrt_error_state;
            jacobian_error_measurement_wrt_error_state=BlockMatrix::convertToEigenSparse(block_jacobian_error_measurement_wrt_error_state);

            // Sparse Matrix from block matrix
            Eigen::SparseMatrix<double> jacobian_error_measurement_wrt_error_parameters;
            jacobian_error_measurement_wrt_error_parameters=BlockMatrix::convertToEigenSparse(block_jacobian_error_measurement_wrt_error_parameters);


#if _DEBUG_MSF_LOCALIZATION_ALGORITHM
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::update() jacobian_error_measurement_wrt_error_state for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
                logString<<Eigen::MatrixXd(jacobian_error_measurement_wrt_error_state)<<std::endl;
                this->log(logString.str());
            }
#endif

#if _DEBUG_MSF_LOCALIZATION_ALGORITHM
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::update() jacobian_error_measurement_wrt_error_parameters for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
                logString<<Eigen::MatrixXd(jacobian_error_measurement_wrt_error_parameters)<<std::endl;
                this->log(logString.str());
            }
#endif



            /// Jacobian Error Measurement - Error Measurement Noise
            /// Hn

            // Init and resize
            BlockMatrix::MatrixSparse block_jacobian_error_measurement_wrt_error_measurement;
            block_jacobian_error_measurement_wrt_error_measurement.resize(num_error_measurements, num_error_measurements);

            // Fill Blocks
            {
                int num_error_measurements_i=0;
                for(std::list<std::shared_ptr<SensorMeasurementCore> >::const_iterator itListPredictedMeas=TheListPredictedMeasurements.begin();
                    itListPredictedMeas!=TheListPredictedMeasurements.end();
                    ++itListPredictedMeas)
                {
                    block_jacobian_error_measurement_wrt_error_measurement(num_error_measurements_i, num_error_measurements_i)=
                            (*itListPredictedMeas)->jacobian_error_measurement_wrt_error_measurement_.measurement;
                    num_error_measurements_i++;
                }
            }

            // Sparse matrix
            Eigen::SparseMatrix<double> jacobian_error_measurement_wrt_error_measurement;
            jacobian_error_measurement_wrt_error_measurement=BlockMatrix::convertToEigenSparse(block_jacobian_error_measurement_wrt_error_measurement);


#if _DEBUG_MSF_LOCALIZATION_ALGORITHM
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::update() jacobian_error_measurement_wrt_error_measurement for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
                logString<<Eigen::MatrixXd(jacobian_error_measurement_wrt_error_measurement)<<std::endl;
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


            /// Covariance Error State
            /// P(k+1|k)

#if _DEBUG_MSF_LOCALIZATION_ALGORITHM
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::update() OldState->covarianceMatrix for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
                logString<<*OldState->covarianceMatrix<<std::endl;
                this->log(logString.str());
            }
#endif



            /// Covariance Error Parameters
            /// Rp

            // Init and resize
            BlockMatrix::MatrixSparse block_covariance_error_parameters;
            block_covariance_error_parameters.resize(num_error_states, num_error_states);

            // Fill blocks
            {
                int num_error_state_i=0;

                // World (Global Parameters)
                {
                    block_covariance_error_parameters(num_error_state_i, num_error_state_i)=
                            OldState->TheGlobalParametersStateCore->getMsfElementCoreSharedPtr()->getCovarianceParameters();
                    num_error_state_i++;
                }

                // Robot
                {
                    block_covariance_error_parameters(num_error_state_i, num_error_state_i)=
                            OldState->TheRobotStateCore->getMsfElementCoreSharedPtr()->getCovarianceParameters();
                    num_error_state_i++;
                }

                // Inputs
                for(std::list< std::shared_ptr<InputStateCore> >::iterator itListInputStateCore=OldState->TheListInputStateCore.begin();
                    itListInputStateCore!=OldState->TheListInputStateCore.end();
                    ++itListInputStateCore)
                {
                    block_covariance_error_parameters(num_error_state_i, num_error_state_i)=
                            (*itListInputStateCore)->getMsfElementCoreSharedPtr()->getCovarianceParameters();
                    num_error_state_i++;
                }

                // Sensors
                for(std::list<std::shared_ptr<SensorStateCore> >::const_iterator itListSensorStateCore=OldState->TheListSensorStateCore.begin();
                    itListSensorStateCore!=OldState->TheListSensorStateCore.end();
                    ++itListSensorStateCore)
                {
                    block_covariance_error_parameters(num_error_state_i, num_error_state_i)=
                            (*itListSensorStateCore)->getMsfElementCoreSharedPtr()->getCovarianceParameters();
                    num_error_state_i++;
                }

                // Map
                for(std::list<std::shared_ptr<MapElementStateCore> >::const_iterator itListMapElementStateCore=OldState->TheListMapElementStateCore.begin();
                    itListMapElementStateCore!=OldState->TheListMapElementStateCore.end();
                    ++itListMapElementStateCore)
                {
                    block_covariance_error_parameters(num_error_state_i, num_error_state_i)=
                           (*itListMapElementStateCore)->getMsfElementCoreSharedPtr()->getCovarianceParameters();
                    num_error_state_i++;
                }
            }

            // Sparse matrix
            Eigen::SparseMatrix<double> covariance_error_parameters;
            covariance_error_parameters=BlockMatrix::convertToEigenSparse(block_covariance_error_parameters);


#if _DEBUG_MSF_LOCALIZATION_ALGORITHM
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::update() covariance_error_parameters for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
                logString<<Eigen::MatrixXd(covariance_error_parameters)<<std::endl;
                this->log(logString.str());
            }
#endif




            /// Covariance Error Measurement
            /// Rn

            // Init and resize
            BlockMatrix::MatrixSparse block_covariance_error_measurement;
            block_covariance_error_measurement.resize(num_error_measurements, num_error_measurements);

            // Fill Blocks
            {
                int num_error_measurements_i=0;
                for(std::list<std::shared_ptr<SensorMeasurementCore> >::const_iterator itListPredictedMeas=TheListMatchedMeasurements.begin();
                    itListPredictedMeas!=TheListMatchedMeasurements.end();
                    ++itListPredictedMeas)
                {
                    block_covariance_error_measurement(num_error_measurements_i, num_error_measurements_i)=
                            (*itListPredictedMeas)->getSensorCoreSharedPtr()->getCovarianceMeasurement();
                    num_error_measurements_i++;
                }
            }

            // Sparse matrix
            Eigen::SparseMatrix<double> covariance_error_measurement;
            covariance_error_measurement=BlockMatrix::convertToEigenSparse(block_covariance_error_measurement);



#if _DEBUG_MSF_LOCALIZATION_ALGORITHM
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::update() CovarianceMeasurement for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
                logString<<Eigen::MatrixXd(covariance_error_measurement)<<std::endl;
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

            // Equation (sparse part): Hp * Rp * Hp^t + Hn * Rn * Hn^t
            Eigen::SparseMatrix<double> innovation_covariance_aux_sparse;
            innovation_covariance_aux_sparse=
                    // Hp * Rp * Hp^t
                    jacobian_error_measurement_wrt_error_parameters*covariance_error_parameters*jacobian_error_measurement_wrt_error_parameters.transpose() +
                    // Hn * Rn * Hn^t
                    jacobian_error_measurement_wrt_error_measurement*covariance_error_measurement*jacobian_error_measurement_wrt_error_measurement.transpose();

            // Equation (dense part): S = Hx * P * Hx^t + ( Hp * Rp * Hp^t + Hn * Rn * Hn^t )
            innovationCovariance=
                    // Hx * P * Hx^t
                    jacobian_error_measurement_wrt_error_state*(*OldState->covarianceMatrix)*jacobian_error_measurement_wrt_error_state.transpose()+
                    // ( Hp * Rp * Hp^t + Hn * Rn * Hn^t )
                    Eigen::MatrixXd(innovation_covariance_aux_sparse);


            // Inverse
            Eigen::MatrixXd innovation_covariance_inverse=innovationCovariance.inverse();



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
            //std::cout<<"distance_mahalanobis-mahalanobisDistanceOld="<<distance_mahalanobis-mahalanobisDistanceOld<<std::endl;
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
            kalmanGain=(*OldState->covarianceMatrix)*jacobian_error_measurement_wrt_error_state.transpose()*innovation_covariance_inverse;


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

            /// Equation: Dx=K*v
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



            /// Get the updated state from the increment of the Error State

            // Init and resize block matrix
            BlockMatrix::MatrixDense block_increment_error_state;

            // TODO

            {
                unsigned int dimension=0;


                // Global Parameters
                unsigned int dimensionGlobalParametersErrorState=UpdatedState->TheGlobalParametersStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorState();
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

                // Robot
                unsigned int dimensionRobotErrorState=UpdatedState->TheRobotStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorState();
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


                // Inputs
                // TODO


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
            //P_error_estimated_k1k1=(eye(dim_state,dim_state)-K*H)*P_error_estimated_k1k*(eye(dim_state,dim_state)-K*H)'+K*R*K';

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
            TimeStamp beginUpdatedCovariance=getTimeStamp();
#endif


            // Auxiliar Matrix: I-K*Hx
            Eigen::MatrixXd AuxiliarMatrix(dimensionErrorState, dimensionErrorState);
            AuxiliarMatrix=Eigen::MatrixXd::Identity(dimensionErrorState, dimensionErrorState)-kalmanGain*jacobian_error_measurement_wrt_error_state;


#if _DEBUG_MSF_LOCALIZATION_ALGORITHM
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::update() updating covariance AuxiliarMatrix for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
                logString<<AuxiliarMatrix<<std::endl;
                this->log(logString.str());
            }
#endif


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



            // OP1: Joseph Form
            *UpdatedState->covarianceMatrix=
                    AuxiliarMatrix*(*OldState->covarianceMatrix)*AuxiliarMatrix.transpose()+kalmanGain*covariance_error_measurement*kalmanGain.transpose();

            // OP2: Simple Form
        //    *UpdatedState->covarianceMatrix=
        //            AuxiliarMatrix*(*OldState->covarianceMatrix);



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
                logString<<"MsfLocalizationCore::update() updated covariance interm 2 time: "<<(getTimeStamp()-beginUpdatedCovariance1).nsec<<std::endl;
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



        }
        else
        {
            // No matched measurements

            // Updated State <- Old State


            // Measurements
            UpdatedState->TheListMeasurementCore=OldState->TheListMeasurementCore;


            // Inputs
            UpdatedState->TheListInputCommandCore=OldState->TheListInputCommandCore;


            // Covariance of the error state
            *UpdatedState->covarianceMatrix=*OldState->covarianceMatrix;


            // State
            // Global Parameters
            UpdatedState->TheGlobalParametersStateCore=OldState->TheGlobalParametersStateCore;

            // Robot State
            UpdatedState->TheRobotStateCore=OldState->TheRobotStateCore;

            // Input State
            UpdatedState->TheListInputStateCore=OldState->TheListInputStateCore;

            // Sensors State
            UpdatedState->TheListSensorStateCore=OldState->TheListSensorStateCore;

            // Map State
            UpdatedState->TheListMapElementStateCore=OldState->TheListMapElementStateCore;


        }





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


#if _DEBUG_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            //logString<<"MsfLocalizationCore::update() dimension_new_map_elements_error_state_total="<<dimension_new_map_elements_error_state_total<<" for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
            logString<<"MsfLocalizationCore::update() dimension_new_map_elements_noise_measurement_total="<<dimension_new_map_elements_noise_measurement_total<<" for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
            this->log(logString.str());
        }
 #endif


        /// Jacobian Error-State
        /// Gx

        Eigen::MatrixXd jacobianMapErrorState(dimension_new_map_elements_error_state_total, dimension_error_state_total);
        jacobianMapErrorState.setZero();

        // Fill
        {
            int dimension_new_map_element_error_state_total_i=0;
            std::list< std::shared_ptr<MapElementStateCore> >::iterator itNewMapElementsState;
            std::list< std::shared_ptr<SensorMeasurementCore> >::iterator itUnmatchedMeasurementsWithMapElement;
            for(itNewMapElementsState=TheListNewMapElementsStateCore.begin(), itUnmatchedMeasurementsWithMapElement=TheListUnmatchedMeasurementsWithMapElement.begin();
                itNewMapElementsState!=TheListNewMapElementsStateCore.end() && itUnmatchedMeasurementsWithMapElement!=TheListUnmatchedMeasurementsWithMapElement.end();
                ++itNewMapElementsState, ++itUnmatchedMeasurementsWithMapElement)
            {
                int dimension_new_map_element_error_state_i=(*itNewMapElementsState)->getMsfElementCoreSharedPtr()->getDimensionErrorState();
                int dimension_error_state_total_i=0;
                int dimension_error_state_i=0;


                // Global Parameters
                dimension_error_state_i=UpdatedState->TheGlobalParametersStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorState();
                if(dimension_error_state_i)
                    jacobianMapErrorState.block(dimension_new_map_element_error_state_total_i, dimension_error_state_total_i, dimension_new_map_element_error_state_i, dimension_error_state_i)=
                            (*itNewMapElementsState)->getJacobianMappingGlobalParametersErrorState();
                dimension_error_state_total_i+=dimension_error_state_i;

                // Robot
                dimension_error_state_i=UpdatedState->TheRobotStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorState();
                if(dimension_error_state_i)
                    jacobianMapErrorState.block(dimension_new_map_element_error_state_total_i, dimension_error_state_total_i, dimension_new_map_element_error_state_i, dimension_error_state_i)=
                            (*itNewMapElementsState)->getJacobianMappingRobotErrorState();
                dimension_error_state_total_i+=dimension_error_state_i;


                // Inputs
                // TODO

                // Sensors
                for(std::list< std::shared_ptr<SensorStateCore> >::const_iterator itListSensorCore=UpdatedState->TheListSensorStateCore.begin();
                    itListSensorCore!=UpdatedState->TheListSensorStateCore.end();
                    ++itListSensorCore)
                {
                    dimension_error_state_i=(*itListSensorCore)->getMsfElementCoreSharedPtr()->getDimensionErrorState();

                    if((*itListSensorCore)->getMsfElementCoreSharedPtr() == (*itUnmatchedMeasurementsWithMapElement)->getSensorCoreSharedPtr())
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
                for(std::list< std::shared_ptr<MapElementStateCore> >::const_iterator itListMapElementCore=UpdatedState->TheListMapElementStateCore.begin();
                    itListMapElementCore!=UpdatedState->TheListMapElementStateCore.end();
                    ++itListMapElementCore)
                {

                    dimension_error_state_i=(*itListMapElementCore)->getMsfElementCoreSharedPtr()->getDimensionErrorState();

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


        /// Jacobian Noise
        /// Gn

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

                    if((*it2UnmatchedMeasurementsWithMapElement) == (*itUnmatchedMeasurementsWithMapElement))
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


        /// Covariance Noise
        /// Rnu

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

                if((*it2UnmatchedMeasurementsWithMapElement) == (*itUnmatchedMeasurementsWithMapElement))
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


        // New Map State
        for(std::list<std::shared_ptr<MapElementStateCore>>::iterator itNewMapElementsStateCore=TheListNewMapElementsStateCore.begin();
            itNewMapElementsStateCore!=TheListNewMapElementsStateCore.end();
            ++itNewMapElementsStateCore)
        {
            UpdatedState->TheListMapElementStateCore.push_back((*itNewMapElementsStateCore));
        }





        /// State Covariance Update

#if _DEBUG_MSF_LOCALIZATION_ALGORITHM
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::updateCore() UpdatedState->covarianceMatrix before mapping for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
            logString<<*UpdatedState->covarianceMatrix<<std::endl;
            this->log(logString.str());
        }
 #endif



        Eigen::MatrixXd covarianceUpdated(dimension_error_state_total+dimension_new_map_elements_error_state_total, dimension_error_state_total+dimension_new_map_elements_error_state_total);
        covarianceUpdated.setZero();

        //
        covarianceUpdated.block(0, 0, dimension_error_state_total, dimension_error_state_total)=
                *UpdatedState->covarianceMatrix;

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

            // Covariance of error state
            *OldState->covarianceMatrix=*UpdatedState->covarianceMatrix;


            // State

            // Global Parameters
            OldState->TheGlobalParametersStateCore=UpdatedState->TheGlobalParametersStateCore;

            // Robot State
            OldState->TheRobotStateCore=UpdatedState->TheRobotStateCore;

            // Input State
            OldState->TheListInputStateCore=UpdatedState->TheListInputStateCore;

            // Sensors State
            OldState->TheListSensorStateCore=UpdatedState->TheListSensorStateCore;

            // Map State
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
