
#include "msf_localization_core/msfLocalization.h"




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
        ThePredictedState->TheGlobalParametersStateCore=predicted_state;
    }


    ///// Robot

    {
#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
        TimeStamp beginTimePredictRobot=getTimeStamp();
#endif

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
        ThePredictedState->TheRobotStateCore=predicted_state;


#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::predictCore() predict Robot time state and jac pred: "<<(getTimeStamp()-beginTimePredictRobot).nsec<<std::endl;
            this->log(logString.str());
        }
#endif
    }


    ///// Inputs

    {
        // Clean the list
        ThePredictedState->TheListInputStateCore.clear();

        // Iterate
        for(std::list< std::shared_ptr<StateCore> >::iterator itInput=ThePreviousState->TheListInputStateCore.begin();
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
            ThePredictedState->TheListInputStateCore.push_back(predicted_state);
        }
    }


    ///// Sensors

    {
#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
        TimeStamp beginTimePredictSensors=getTimeStamp();
#endif

        // Clean the list
        ThePredictedState->TheListSensorStateCore.clear();


        // Iterate
        for(std::list< std::shared_ptr<StateCore> >::iterator itSensorElement=ThePreviousState->TheListSensorStateCore.begin();
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
            ThePredictedState->TheListSensorStateCore.push_back(predicted_state);
        }


#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::predictCore() predict Sensors time state and jac pred: "<<(getTimeStamp()-beginTimePredictSensors).nsec<<std::endl;
            this->log(logString.str());
        }
#endif

    }


    ///// Map

    {
        // Clean the list
        ThePredictedState->TheListMapElementStateCore.clear();

        // Iterate
        for(std::list< std::shared_ptr<StateCore> >::iterator itMapElement=ThePreviousState->TheListMapElementStateCore.begin();
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
            ThePredictedState->TheListMapElementStateCore.push_back(predicted_state);
        }
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
    // TODO FIX!
    Eigen::SparseMatrix<double> jacobianRobotErrorState=ThePredictedState->TheRobotStateCore->getJacobianErrorStateRobot();







    /// Cov Total Robot

    {
#if _DEBUG_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::predictCore() covariance robot TS: sec="<<ThePredictedTimeStamp.sec<<" s; nsec="<<ThePredictedTimeStamp.nsec<<" ns"<<std::endl;
            this->log(logString.str());
        }
#endif


        /*
        /// Dimensions total robot error state

        // Dimensions
        int dimension_world_error_state=ThePredictedState->TheGlobalParametersStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorState();
        int dimension_robot_error_state=ThePredictedState->TheRobotStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorState();
        int dimension_inputs_error_state=0;
        for(std::list< std::shared_ptr<StateCore> >::iterator itInputState=ThePredictedState->TheListInputStateCore.begin();
            itInputState!=ThePredictedState->TheListInputStateCore.end();
            ++itInputState)
        {
            dimension_inputs_error_state+=(*itInputState)->getMsfElementCoreSharedPtr()->getDimensionErrorState();
        }
        int dimension_total_robot_error_state=dimension_world_error_state+dimension_robot_error_state+dimension_inputs_error_state;


        // Size total robot error state
        int num_robot_error_states=0;
        {
            // World
            num_robot_error_states++;
            // Robot
            num_robot_error_states++;
            // Inputs
            num_robot_error_states+=ThePredictedState->getNumberInputStates();
        }
        */


        // Num Error State blocks
        int num_error_states=0;
        {
            // World
            num_error_states++;
            // Robot
            num_error_states++;
            // Inputs
            num_error_states+=ThePredictedState->getNumberInputStates();
            // Sensors
            num_error_states+=ThePredictedState->getNumberSensorStates();
            // Map Elements
            num_error_states+=ThePredictedState->getNumberMapElementStates();
        }

        // Create vectors of state dimensions
        Eigen::VectorXi size_error_state;
        size_error_state.resize(num_error_states);
        // Fill
        {
            int num_error_state_i=0;
            // World
            size_error_state(num_error_state_i)=ThePredictedState->TheGlobalParametersStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorState();
            num_error_state_i++;
            // Robot
            size_error_state(num_error_state_i)=ThePredictedState->TheRobotStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorState();
            num_error_state_i++;
            // Inputs
            for(std::list< std::shared_ptr<StateCore> >::iterator itListInputState=ThePredictedState->TheListInputStateCore.begin();
                itListInputState!=ThePredictedState->TheListInputStateCore.end();
                ++itListInputState)
            {
                size_error_state(num_error_state_i)=(*itListInputState)->getMsfElementCoreSharedPtr()->getDimensionErrorState();
                num_error_state_i++;
            }
            // Sensors
            for(std::list< std::shared_ptr<StateCore> >::iterator itListSensorState=ThePredictedState->TheListSensorStateCore.begin();
                itListSensorState!=ThePredictedState->TheListSensorStateCore.end();
                ++itListSensorState)
            {
                size_error_state(num_error_state_i)=(*itListSensorState)->getMsfElementCoreSharedPtr()->getDimensionErrorState();
                num_error_state_i++;
            }
            // Map
            for(std::list< std::shared_ptr<StateCore> >::iterator itListMapElementState=ThePredictedState->TheListMapElementStateCore.begin();
                itListMapElementState!=ThePredictedState->TheListMapElementStateCore.end();
                ++itListMapElementState)
            {
                size_error_state(num_error_state_i)=(*itListMapElementState)->getMsfElementCoreSharedPtr()->getDimensionErrorState();
                num_error_state_i++;
            }
        }



        // Size input commands
        int num_input_commands=inputCommand->getNumberInputCommand();


        ///// Jacobians

        /// Jacobian Error State: Fx & Jacobian Error Parameters: Fp
//std::cout<<"Fx & Fp"<<std::endl;
        // Resize and init
        BlockMatrix::MatrixSparse block_jacobian_total_robot_error_state;
        block_jacobian_total_robot_error_state.resize(num_error_states, num_error_states);

        BlockMatrix::MatrixSparse block_jacobian_total_robot_error_parameters;
        block_jacobian_total_robot_error_parameters.resize(num_error_states, num_error_states);

        // Fill
        {
            int jacobian_row=0;
//std::cout<<"world"<<std::endl;
            // World
            {
                int jacobian_column=0;

                // World / World
                block_jacobian_total_robot_error_state(jacobian_row, jacobian_column)=
                        ThePredictedState->TheGlobalParametersStateCore->getJacobianErrorStateWorld();
                block_jacobian_total_robot_error_parameters(jacobian_row, jacobian_column)=
                        ThePredictedState->TheGlobalParametersStateCore->getJacobianErrorParametersWorld();
                jacobian_column++;

                // World / Robot
                block_jacobian_total_robot_error_state(jacobian_row, jacobian_column)=
                        ThePredictedState->TheGlobalParametersStateCore->getJacobianErrorStateRobot();
                block_jacobian_total_robot_error_parameters(jacobian_row, jacobian_column)=
                        ThePredictedState->TheGlobalParametersStateCore->getJacobianErrorParametersRobot();
                jacobian_column++;

                // World / Inputs
                int size_inputs_i=0;
                for(std::list< std::shared_ptr<StateCore> >::iterator itInputState2=ThePredictedState->TheListInputStateCore.begin();
                    itInputState2!=ThePredictedState->TheListInputStateCore.end();
                    ++itInputState2, size_inputs_i++)
                {
                    block_jacobian_total_robot_error_state(jacobian_row, jacobian_column)=
                            ThePredictedState->TheGlobalParametersStateCore->getJacobianErrorStateInput(size_inputs_i);
                    block_jacobian_total_robot_error_parameters(jacobian_row, jacobian_column)=
                            ThePredictedState->TheGlobalParametersStateCore->getJacobianErrorParametersInput(size_inputs_i);
                    jacobian_column++;
                }

                // World / Sensors
                int num_sensor_i=0;
                for(std::list< std::shared_ptr<StateCore> >::iterator itSensorState2=ThePredictedState->TheListSensorStateCore.begin();
                    itSensorState2!=ThePredictedState->TheListSensorStateCore.end();
                    ++itSensorState2, num_sensor_i++)
                {
//                    block_jacobian_total_robot_error_state(jacobian_row, jacobian_column)=
//                            ThePredictedState->TheGlobalParametersStateCore->getJacobianErrorStateSensor(num_sensor_i);
//                    block_jacobian_total_robot_error_parameters(jacobian_row, jacobian_column)=
//                            ThePredictedState->TheGlobalParametersStateCore->getJacobianErrorParametersSensor(num_sensor_i);
                    jacobian_column++;
                }

                // World / Map Elements
                int num_map_element_i=0;
                for(std::list< std::shared_ptr<StateCore> >::iterator itMapElementState2=ThePredictedState->TheListMapElementStateCore.begin();
                    itMapElementState2!=ThePredictedState->TheListMapElementStateCore.end();
                    ++itMapElementState2, num_map_element_i++)
                {
//                    block_jacobian_total_robot_error_state(jacobian_row, jacobian_column)=
//                            ThePredictedState->TheGlobalParametersStateCore->getJacobianErrorStateMapElement(num_map_element_i);
//                    block_jacobian_total_robot_error_parameters(jacobian_row, jacobian_column)=
//                            ThePredictedState->TheGlobalParametersStateCore->getJacobianErrorParametersMapElement(num_map_element_i);
                    jacobian_column++;
                }

                jacobian_row++;
            }

//std::cout<<"robot"<<std::endl;
            // Robot
            {
                int jacobian_column=0;

                // Robot / World
                block_jacobian_total_robot_error_state(jacobian_row, jacobian_column)=
                        ThePredictedState->TheRobotStateCore->getJacobianErrorStateWorld();
                block_jacobian_total_robot_error_parameters(jacobian_row, jacobian_column)=
                        ThePredictedState->TheRobotStateCore->getJacobianErrorParametersWorld();
                jacobian_column++;

                // Robot / Robot
                block_jacobian_total_robot_error_state(jacobian_row, jacobian_column)=
                        ThePredictedState->TheRobotStateCore->getJacobianErrorStateRobot();
                block_jacobian_total_robot_error_parameters(jacobian_row, jacobian_column)=
                        ThePredictedState->TheRobotStateCore->getJacobianErrorParametersRobot();
                jacobian_column++;

                // Robot / Inputs
                int size_inputs_i=0;
                for(std::list< std::shared_ptr<StateCore> >::iterator itInputState2=ThePredictedState->TheListInputStateCore.begin();
                    itInputState2!=ThePredictedState->TheListInputStateCore.end();
                    ++itInputState2, size_inputs_i++)
                {
                    block_jacobian_total_robot_error_state(jacobian_row, jacobian_column)=
                            ThePredictedState->TheRobotStateCore->getJacobianErrorStateInput(size_inputs_i);
                    block_jacobian_total_robot_error_parameters(jacobian_row, jacobian_column)=
                            ThePredictedState->TheRobotStateCore->getJacobianErrorParametersInput(size_inputs_i);
                    jacobian_column++;
                }

                // Robot / Sensors
                int num_sensor_i=0;
                for(std::list< std::shared_ptr<StateCore> >::iterator itSensorState2=ThePredictedState->TheListSensorStateCore.begin();
                    itSensorState2!=ThePredictedState->TheListSensorStateCore.end();
                    ++itSensorState2, num_sensor_i++)
                {
//                    block_jacobian_total_robot_error_state(jacobian_row, jacobian_column)=
//                            ThePredictedState->TheRobotStateCore->getJacobianErrorStateSensor(num_sensor_i);
//                    block_jacobian_total_robot_error_parameters(jacobian_row, jacobian_column)=
//                            ThePredictedState->TheRobotStateCore->getJacobianErrorParametersSensor(num_sensor_i);
                    jacobian_column++;
                }

                // Robot / Map Elements
                int num_map_element_i=0;
                for(std::list< std::shared_ptr<StateCore> >::iterator itMapElementState2=ThePredictedState->TheListMapElementStateCore.begin();
                    itMapElementState2!=ThePredictedState->TheListMapElementStateCore.end();
                    ++itMapElementState2, num_map_element_i++)
                {
//                    block_jacobian_total_robot_error_state(jacobian_row, jacobian_column)=
//                            ThePredictedState->TheRobotStateCore->getJacobianErrorStateMapElement(num_map_element_i);
//                    block_jacobian_total_robot_error_parameters(jacobian_row, jacobian_column)=
//                            ThePredictedState->TheRobotStateCore->getJacobianErrorParametersMapElement(num_map_element_i);
                    jacobian_column++;
                }

                jacobian_row++;
            }
//std::cout<<"inputs"<<std::endl;
            // Inputs
            for(std::list< std::shared_ptr<StateCore> >::iterator itInputState=ThePredictedState->TheListInputStateCore.begin();
                itInputState!=ThePredictedState->TheListInputStateCore.end();
                ++itInputState)
            {
                int jacobian_column=0;

                // Inputs / World
                block_jacobian_total_robot_error_state(jacobian_row, jacobian_column)=
                        (*itInputState)->getJacobianErrorStateWorld();
                block_jacobian_total_robot_error_parameters(jacobian_row, jacobian_column)=
                        (*itInputState)->getJacobianErrorParametersWorld();
                jacobian_column++;

                // Inputs / Robot
                block_jacobian_total_robot_error_state(jacobian_row, jacobian_column)=
                        (*itInputState)->getJacobianErrorStateRobot();
                block_jacobian_total_robot_error_parameters(jacobian_row, jacobian_column)=
                        (*itInputState)->getJacobianErrorParametersRobot();
                jacobian_column++;

                // Inputs / Inputs
                int size_inputs_i=0;
                for(std::list< std::shared_ptr<StateCore> >::iterator itInputState2=ThePredictedState->TheListInputStateCore.begin();
                    itInputState2!=ThePredictedState->TheListInputStateCore.end();
                    ++itInputState2, size_inputs_i++)
                {
                    block_jacobian_total_robot_error_state(jacobian_row, jacobian_column)=
                            (*itInputState)->getJacobianErrorStateInput(size_inputs_i);
                    block_jacobian_total_robot_error_parameters(jacobian_row, jacobian_column)=
                            (*itInputState)->getJacobianErrorParametersInput(size_inputs_i);
                    jacobian_column++;
                }

                // Inputs / Sensors
                int num_sensor_i=0;
                for(std::list< std::shared_ptr<StateCore> >::iterator itSensorState2=ThePredictedState->TheListSensorStateCore.begin();
                    itSensorState2!=ThePredictedState->TheListSensorStateCore.end();
                    ++itSensorState2, num_sensor_i++)
                {
//                    block_jacobian_total_robot_error_state(jacobian_row, jacobian_column)=
//                            (*itInputState)->getJacobianErrorStateSensor(num_sensor_i);
//                    block_jacobian_total_robot_error_parameters(jacobian_row, jacobian_column)=
//                            (*itInputState)->getJacobianErrorParametersSensor(num_sensor_i);
                    jacobian_column++;
                }

                // Inputs / Map Elements
                int num_map_element_i=0;
                for(std::list< std::shared_ptr<StateCore> >::iterator itMapElementState2=ThePredictedState->TheListMapElementStateCore.begin();
                    itMapElementState2!=ThePredictedState->TheListMapElementStateCore.end();
                    ++itMapElementState2, num_map_element_i++)
                {
//                    block_jacobian_total_robot_error_state(jacobian_row, jacobian_column)=
//                            (*itInputState)->getJacobianErrorStateMapElement(num_map_element_i);
//                    block_jacobian_total_robot_error_parameters(jacobian_row, jacobian_column)=
//                            (*itInputState)->getJacobianErrorParametersMapElement(num_map_element_i);
                    jacobian_column++;
                }

                jacobian_row++;
            }
//std::cout<<"sensors"<<std::endl;
            // Sensors
            for(std::list< std::shared_ptr<StateCore> >::iterator itSensorState=ThePredictedState->TheListSensorStateCore.begin();
                itSensorState!=ThePredictedState->TheListSensorStateCore.end();
                ++itSensorState)
            {
                int jacobian_column=0;

                // Sensors / World
//                block_jacobian_total_robot_error_state(jacobian_row, jacobian_column)=
//                        (*itSensorState)->getJacobianErrorStateWorld();
//                block_jacobian_total_robot_error_parameters(jacobian_row, jacobian_column)=
//                        (*itSensorState)->getJacobianErrorParametersWorld();
                jacobian_column++;

                // Sensors / Robot
//                block_jacobian_total_robot_error_state(jacobian_row, jacobian_column)=
//                        (*itSensorState)->getJacobianErrorStateRobot();
//                block_jacobian_total_robot_error_parameters(jacobian_row, jacobian_column)=
//                        (*itSensorState)->getJacobianErrorParametersRobot();
                jacobian_column++;

                // Sensors / Inputs
                int size_inputs_i=0;
                for(std::list< std::shared_ptr<StateCore> >::iterator itInputState2=ThePredictedState->TheListInputStateCore.begin();
                    itInputState2!=ThePredictedState->TheListInputStateCore.end();
                    ++itInputState2, size_inputs_i++)
                {
//                    block_jacobian_total_robot_error_state(jacobian_row, jacobian_column)=
//                            (*itSensorState)->getJacobianErrorStateInput(size_inputs_i);
//                    block_jacobian_total_robot_error_parameters(jacobian_row, jacobian_column)=
//                            (*itSensorState)->getJacobianErrorParametersInput(size_inputs_i);
                    jacobian_column++;
                }

                // Sensors / Sensors
                int num_sensor_i=0;
                for(std::list< std::shared_ptr<StateCore> >::iterator itSensorState2=ThePredictedState->TheListSensorStateCore.begin();
                    itSensorState2!=ThePredictedState->TheListSensorStateCore.end();
                    ++itSensorState2, num_sensor_i++)
                {
                    block_jacobian_total_robot_error_state(jacobian_row, jacobian_column)=
                            (*itSensorState)->getJacobianErrorStateSensor(num_sensor_i);
                    block_jacobian_total_robot_error_parameters(jacobian_row, jacobian_column)=
                            (*itSensorState)->getJacobianErrorParametersSensor(num_sensor_i);
                    jacobian_column++;
                }

                // Sensors / Map Elements
                int num_map_element_i=0;
                for(std::list< std::shared_ptr<StateCore> >::iterator itMapElementState2=ThePredictedState->TheListMapElementStateCore.begin();
                    itMapElementState2!=ThePredictedState->TheListMapElementStateCore.end();
                    ++itMapElementState2, num_map_element_i++)
                {
//                    block_jacobian_total_robot_error_state(jacobian_row, jacobian_column)=
//                            (*itSensorState)->getJacobianErrorStateMapElement(num_map_element_i);
//                    block_jacobian_total_robot_error_parameters(jacobian_row, jacobian_column)=
//                            (*itSensorState)->getJacobianErrorParametersMapElement(num_map_element_i);
                    jacobian_column++;
                }

                jacobian_row++;
            }
//std::cout<<"map elements"<<std::endl;
            // Map Elements
            for(std::list< std::shared_ptr<StateCore> >::iterator itMapElementState=ThePredictedState->TheListMapElementStateCore.begin();
                itMapElementState!=ThePredictedState->TheListMapElementStateCore.end();
                ++itMapElementState)
            {
                int jacobian_column=0;

                // Map Elements / World
//                block_jacobian_total_robot_error_state(jacobian_row, jacobian_column)=
//                        (*itMapElementState)->getJacobianErrorStateWorld();
//                block_jacobian_total_robot_error_parameters(jacobian_row, jacobian_column)=
//                        (*itMapElementState)->getJacobianErrorParametersWorld();
                jacobian_column++;

                // Map Elements / Robot
//                block_jacobian_total_robot_error_state(jacobian_row, jacobian_column)=
//                        (*itMapElementState)->getJacobianErrorStateRobot();
//                block_jacobian_total_robot_error_parameters(jacobian_row, jacobian_column)=
//                        (*itMapElementState)->getJacobianErrorParametersRobot();
                jacobian_column++;

                // Map Elements / Inputs
                int size_inputs_i=0;
                for(std::list< std::shared_ptr<StateCore> >::iterator itInputState2=ThePredictedState->TheListInputStateCore.begin();
                    itInputState2!=ThePredictedState->TheListInputStateCore.end();
                    ++itInputState2, size_inputs_i++)
                {
//                    block_jacobian_total_robot_error_state(jacobian_row, jacobian_column)=
//                            (*itMapElementState)->getJacobianErrorStateInput(size_inputs_i);
//                    block_jacobian_total_robot_error_parameters(jacobian_row, jacobian_column)=
//                            (*itMapElementState)->getJacobianErrorParametersInput(size_inputs_i);
                    jacobian_column++;
                }

                // Map Elements / Sensors
                int num_sensor_i=0;
                for(std::list< std::shared_ptr<StateCore> >::iterator itSensorState2=ThePredictedState->TheListSensorStateCore.begin();
                    itSensorState2!=ThePredictedState->TheListSensorStateCore.end();
                    ++itSensorState2, num_sensor_i++)
                {
//                    block_jacobian_total_robot_error_state(jacobian_row, jacobian_column)=
//                            (*itMapElementState)->getJacobianErrorStateSensor(num_sensor_i);
//                    block_jacobian_total_robot_error_parameters(jacobian_row, jacobian_column)=
//                            (*itMapElementState)->getJacobianErrorParametersSensor(num_sensor_i);
                    jacobian_column++;
                }


                // Map Elements / Map Elements
                int num_map_element_i=0;
                for(std::list< std::shared_ptr<StateCore> >::iterator itMapElementState2=ThePredictedState->TheListMapElementStateCore.begin();
                    itMapElementState2!=ThePredictedState->TheListMapElementStateCore.end();
                    ++itMapElementState2, num_map_element_i++)
                {
                    block_jacobian_total_robot_error_state(jacobian_row, jacobian_column)=
                            (*itMapElementState)->getJacobianErrorStateMapElement(num_map_element_i);
                    block_jacobian_total_robot_error_parameters(jacobian_row, jacobian_column)=
                            (*itMapElementState)->getJacobianErrorParametersMapElement(num_map_element_i);
                    jacobian_column++;
                }

                jacobian_row++;
            }
        }
//std::cout<<"ended"<<std::endl;
        block_jacobian_total_robot_error_state.analyse();
        block_jacobian_total_robot_error_parameters.analyse();



        /// Jacobian Error State wrt Error Input Commands: Fu
//std::cout<<"Fu"<<std::endl;
        BlockMatrix::MatrixSparse block_jacobian_total_robot_error_state_wrt_error_input_commands;
        block_jacobian_total_robot_error_state_wrt_error_input_commands.resize(num_error_states, num_input_commands);

        // Fill
        {
            int jacobian_row=0;

            // World
            {
                int jacobian_column=0;
                int num_input_commands_i=0;
                for(std::list< std::shared_ptr<InputCommandCore> >::iterator itInputCommand=inputCommand->TheListInputCommandCore.begin();
                    itInputCommand!=inputCommand->TheListInputCommandCore.end();
                    ++itInputCommand, num_input_commands_i++)
                {
                    block_jacobian_total_robot_error_state_wrt_error_input_commands(jacobian_row, jacobian_column)=
                            ThePredictedState->TheGlobalParametersStateCore->getJacobianErrorInputCommands(num_input_commands_i);
                    jacobian_column++;
                }
                jacobian_row++;
            }

            // Robot
            {
                int jacobian_column=0;
                int num_input_commands_i=0;
                for(std::list< std::shared_ptr<InputCommandCore> >::iterator itInputCommand=inputCommand->TheListInputCommandCore.begin();
                    itInputCommand!=inputCommand->TheListInputCommandCore.end();
                    ++itInputCommand, num_input_commands_i++)
                {
                    block_jacobian_total_robot_error_state_wrt_error_input_commands(jacobian_row, jacobian_column)=
                            ThePredictedState->TheRobotStateCore->getJacobianErrorInputCommands(num_input_commands_i);
                    jacobian_column++;
                }
                jacobian_row++;
            }

            // Inputs
            for(std::list< std::shared_ptr<StateCore> >::iterator itInputState=ThePredictedState->TheListInputStateCore.begin();
                itInputState!=ThePredictedState->TheListInputStateCore.end();
                ++itInputState)
            {
                int jacobian_column=0;
                int num_input_commands_i=0;
                for(std::list< std::shared_ptr<InputCommandCore> >::iterator itInputCommand=inputCommand->TheListInputCommandCore.begin();
                    itInputCommand!=inputCommand->TheListInputCommandCore.end();
                    ++itInputCommand, num_input_commands_i++)
                {
                    block_jacobian_total_robot_error_state_wrt_error_input_commands(jacobian_row, jacobian_column)=
                            (*itInputState)->getJacobianErrorInputCommands(num_input_commands_i);
                    jacobian_column++;
                }
                jacobian_row++;
            }

            // Sensors
            for(std::list< std::shared_ptr<StateCore> >::iterator itSensorState=ThePredictedState->TheListSensorStateCore.begin();
                itSensorState!=ThePredictedState->TheListSensorStateCore.end();
                ++itSensorState)
            {
                int jacobian_column=0;
                int num_input_commands_i=0;
                for(std::list< std::shared_ptr<InputCommandCore> >::iterator itInputCommand=inputCommand->TheListInputCommandCore.begin();
                    itInputCommand!=inputCommand->TheListInputCommandCore.end();
                    ++itInputCommand, num_input_commands_i++)
                {
                    block_jacobian_total_robot_error_state_wrt_error_input_commands(jacobian_row, jacobian_column)=
                            (*itSensorState)->getJacobianErrorInputCommands(num_input_commands_i);
                    jacobian_column++;
                }
                jacobian_row++;
            }

            // Map Elements
            for(std::list< std::shared_ptr<StateCore> >::iterator itMapElementState=ThePredictedState->TheListMapElementStateCore.begin();
                itMapElementState!=ThePredictedState->TheListMapElementStateCore.end();
                ++itMapElementState)
            {
                int jacobian_column=0;
                int num_input_commands_i=0;
                for(std::list< std::shared_ptr<InputCommandCore> >::iterator itInputCommand=inputCommand->TheListInputCommandCore.begin();
                    itInputCommand!=inputCommand->TheListInputCommandCore.end();
                    ++itInputCommand, num_input_commands_i++)
                {
                    block_jacobian_total_robot_error_state_wrt_error_input_commands(jacobian_row, jacobian_column)=
                            (*itMapElementState)->getJacobianErrorInputCommands(num_input_commands_i);
                    jacobian_column++;
                }
                jacobian_row++;
            }

        }

        block_jacobian_total_robot_error_state_wrt_error_input_commands.analyse();



        /// Jacobian Error State Noise Estimation: Fn
//std::cout<<"Fn"<<std::endl;
        BlockMatrix::MatrixSparse block_jacobian_total_robot_error_noise_estimation;
        block_jacobian_total_robot_error_noise_estimation.resize(num_error_states, num_error_states);

        // Fill
        {
            int num_robot_error_states_i=0;

            // World
            block_jacobian_total_robot_error_noise_estimation(num_robot_error_states_i, num_robot_error_states_i)=
                    ThePredictedState->TheGlobalParametersStateCore->getJacobianErrorStateNoise();
            num_robot_error_states_i++;

            // Robot
            block_jacobian_total_robot_error_noise_estimation(num_robot_error_states_i, num_robot_error_states_i)=
                    ThePredictedState->TheRobotStateCore->getJacobianErrorStateNoise();
            num_robot_error_states_i++;

            // Inputs
            for(std::list< std::shared_ptr<StateCore> >::iterator itInputState=ThePredictedState->TheListInputStateCore.begin();
                itInputState!=ThePredictedState->TheListInputStateCore.end();
                ++itInputState)
            {
                block_jacobian_total_robot_error_noise_estimation(num_robot_error_states_i, num_robot_error_states_i)=
                        (*itInputState)->getJacobianErrorStateNoise();
                num_robot_error_states_i++;
            }

            // Sensors
            for(std::list< std::shared_ptr<StateCore> >::iterator itSensorState=ThePredictedState->TheListSensorStateCore.begin();
                itSensorState!=ThePredictedState->TheListSensorStateCore.end();
                ++itSensorState)
            {
                block_jacobian_total_robot_error_noise_estimation(num_robot_error_states_i, num_robot_error_states_i)=
                        (*itSensorState)->getJacobianErrorStateNoise();
                num_robot_error_states_i++;
            }

            // Map Element
            for(std::list< std::shared_ptr<StateCore> >::iterator itMapElementState=ThePredictedState->TheListMapElementStateCore.begin();
                itMapElementState!=ThePredictedState->TheListMapElementStateCore.end();
                ++itMapElementState)
            {
                block_jacobian_total_robot_error_noise_estimation(num_robot_error_states_i, num_robot_error_states_i)=
                        (*itMapElementState)->getJacobianErrorStateNoise();
                num_robot_error_states_i++;
            }
        }

        block_jacobian_total_robot_error_noise_estimation.analyse();



        ///// Covariances Total Robot


        /// Covariance Error State: P
//std::cout<<"P"<<std::endl;


#if 1 || _DEBUG_TIME_MSF_LOCALIZATION_CORE
        TimeStamp beginTimePredictedCovarianceErrorStateAsBlock=getTimeStamp();
#endif





        BlockMatrix::MatrixDense block_previous_covariance_error_state;
        block_previous_covariance_error_state.resize(num_error_states, num_error_states);

        block_previous_covariance_error_state.createFromEigen((*ThePreviousState->covarianceMatrix), size_error_state, size_error_state);


#if 1 || _DEBUG_TIME_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::predictCore() predict covariance error state as block: "<<(getTimeStamp()-beginTimePredictedCovarianceErrorStateAsBlock).nsec<<std::endl;
            this->log(logString.str());
        }
#endif



//        std::cout<<"block_previous_covariance_error_state"<<std::endl;
//        std::cout<<"size_rows="<<block_previous_covariance_error_state.getRowsSize().transpose()<<std::endl;
//        std::cout<<"size_cols="<<block_previous_covariance_error_state.getColsSize().transpose()<<std::endl;


        /// Covariance Error Parameters: Qp
//std::cout<<"Qp"<<std::endl;
        BlockMatrix::MatrixSparse block_covariance_total_robot_error_parameters;
        block_covariance_total_robot_error_parameters.resize(num_error_states, num_error_states);

        // Fill
        {
            int num_robot_error_states_i=0;

            // World
            block_covariance_total_robot_error_parameters(num_robot_error_states_i, num_robot_error_states_i)=
                    ThePredictedState->TheGlobalParametersStateCore->getMsfElementCoreSharedPtr()->getCovarianceParameters();
            num_robot_error_states_i++;

            // Robot
            block_covariance_total_robot_error_parameters(num_robot_error_states_i, num_robot_error_states_i)=
                    ThePredictedState->TheRobotStateCore->getMsfElementCoreSharedPtr()->getCovarianceParameters();
            num_robot_error_states_i++;

            // Inputs
            for(std::list< std::shared_ptr<StateCore> >::iterator itInputState=ThePredictedState->TheListInputStateCore.begin();
                itInputState!=ThePredictedState->TheListInputStateCore.end();
                ++itInputState)
            {
                block_covariance_total_robot_error_parameters(num_robot_error_states_i, num_robot_error_states_i)=
                        (*itInputState)->getMsfElementCoreSharedPtr()->getCovarianceParameters();
                num_robot_error_states_i++;
            }

            // Sensors
            for(std::list< std::shared_ptr<StateCore> >::iterator itSensorState=ThePredictedState->TheListSensorStateCore.begin();
                itSensorState!=ThePredictedState->TheListSensorStateCore.end();
                ++itSensorState)
            {
                block_covariance_total_robot_error_parameters(num_robot_error_states_i, num_robot_error_states_i)=
                        (*itSensorState)->getMsfElementCoreSharedPtr()->getCovarianceParameters();
                num_robot_error_states_i++;
            }

            // Map Elements
            for(std::list< std::shared_ptr<StateCore> >::iterator itMapElementState=ThePredictedState->TheListMapElementStateCore.begin();
                itMapElementState!=ThePredictedState->TheListMapElementStateCore.end();
                ++itMapElementState)
            {
                block_covariance_total_robot_error_parameters(num_robot_error_states_i, num_robot_error_states_i)=
                        (*itMapElementState)->getMsfElementCoreSharedPtr()->getCovarianceParameters();
                num_robot_error_states_i++;
            }
        }

        block_covariance_total_robot_error_parameters.analyse();



        /// Covariance Error Inputs: Qu
//std::cout<<"Qu"<<std::endl;
        BlockMatrix::MatrixSparse block_covariance_total_robot_error_inputs;
        block_covariance_total_robot_error_inputs.resize(num_input_commands, num_input_commands);

        // Fill
        {
            int num_input_commands_i=0;

            for(std::list< std::shared_ptr<InputCommandCore> >::iterator itInputCommand=inputCommand->TheListInputCommandCore.begin();
                itInputCommand!=inputCommand->TheListInputCommandCore.end();
                ++itInputCommand++)
            {
                block_covariance_total_robot_error_inputs(num_input_commands_i, num_input_commands_i)=
                        (*itInputCommand)->getInputCoreSharedPtr()->getCovarianceInputs(DeltaTime);
                num_input_commands_i++;
            }

        }

        block_covariance_total_robot_error_inputs.analyse();



        /// Covariance Error State Noise Estimation: Qn
//std::cout<<"Qn"<<std::endl;
        BlockMatrix::MatrixSparse block_covariance_total_robot_error_noise_estimation;
        block_covariance_total_robot_error_noise_estimation.resize(num_error_states, num_error_states);

        // Fill
        {
            int num_robot_error_states_i=0;

            // World
            block_covariance_total_robot_error_noise_estimation(num_robot_error_states_i, num_robot_error_states_i)=
                    ThePredictedState->TheGlobalParametersStateCore->getMsfElementCoreSharedPtr()->getCovarianceNoise(DeltaTime);
            num_robot_error_states_i++;

            // Robot
            block_covariance_total_robot_error_noise_estimation(num_robot_error_states_i, num_robot_error_states_i)=
                    ThePredictedState->TheRobotStateCore->getMsfElementCoreSharedPtr()->getCovarianceNoise(DeltaTime);
            num_robot_error_states_i++;

            // Inputs
            for(std::list< std::shared_ptr<StateCore> >::iterator itInputState=ThePredictedState->TheListInputStateCore.begin();
                itInputState!=ThePredictedState->TheListInputStateCore.end();
                ++itInputState)
            {
                block_covariance_total_robot_error_noise_estimation(num_robot_error_states_i, num_robot_error_states_i)=
                        (*itInputState)->getMsfElementCoreSharedPtr()->getCovarianceNoise(DeltaTime);
                num_robot_error_states_i++;
            }

            // Sensors
            for(std::list< std::shared_ptr<StateCore> >::iterator itSensorState=ThePredictedState->TheListSensorStateCore.begin();
                itSensorState!=ThePredictedState->TheListSensorStateCore.end();
                ++itSensorState)
            {
                block_covariance_total_robot_error_noise_estimation(num_robot_error_states_i, num_robot_error_states_i)=
                        (*itSensorState)->getMsfElementCoreSharedPtr()->getCovarianceNoise(DeltaTime);
                num_robot_error_states_i++;
            }

            // Map Elements
            for(std::list< std::shared_ptr<StateCore> >::iterator itMapElementState=ThePredictedState->TheListMapElementStateCore.begin();
                itMapElementState!=ThePredictedState->TheListMapElementStateCore.end();
                ++itMapElementState)
            {
                block_covariance_total_robot_error_noise_estimation(num_robot_error_states_i, num_robot_error_states_i)=
                        (*itMapElementState)->getMsfElementCoreSharedPtr()->getCovarianceNoise(DeltaTime);
                num_robot_error_states_i++;
            }
        }

        block_covariance_total_robot_error_noise_estimation.analyse();



//#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
//        TimeStamp beginTimePredictCoreCovRob=getTimeStamp();
//#endif




        //// P(k+1|k)


#if 1 || _DEBUG_TIME_MSF_LOCALIZATION_CORE
        TimeStamp beginTimePredictedCovarianceErrorState=getTimeStamp();
#endif


        BlockMatrix::MatrixDense block_predicted_covariance_error_state;
        block_predicted_covariance_error_state.resize(num_error_states, num_error_states);


//std::cout<<"before operations"<<std::endl;
        try
        {

            BlockMatrix::MatrixSparse block_covariance_total_robot_error;
            block_covariance_total_robot_error.resize(num_error_states, num_error_states);


            block_covariance_total_robot_error= // Fn * Qn * Fn^t
                                                block_jacobian_total_robot_error_noise_estimation*block_covariance_total_robot_error_noise_estimation*block_jacobian_total_robot_error_noise_estimation.transpose() +
                                                // Fp * Qp * Fp^t
                                                block_jacobian_total_robot_error_parameters*block_covariance_total_robot_error_parameters*block_jacobian_total_robot_error_parameters.transpose();


            BlockMatrix::MatrixSparse aux;
            aux=block_jacobian_total_robot_error_noise_estimation.transpose();


            /*
            std::cout<<"block_jacobian_total_robot_error_noise_estimation"<<std::endl;
            std::cout<<"size_rows="<<block_jacobian_total_robot_error_noise_estimation.getRowsSize().transpose()<<std::endl;
            std::cout<<"size_cols="<<block_jacobian_total_robot_error_noise_estimation.getColsSize().transpose()<<std::endl;


            std::cout<<"block_covariance_total_robot_error_noise_estimation"<<std::endl;
            std::cout<<"size_rows="<<block_covariance_total_robot_error_noise_estimation.getRowsSize().transpose()<<std::endl;
            std::cout<<"size_cols="<<block_covariance_total_robot_error_noise_estimation.getColsSize().transpose()<<std::endl;

            std::cout<<"aux"<<std::endl;
            std::cout<<"size_rows="<<aux.getRowsSize().transpose()<<std::endl;
            std::cout<<"size_cols="<<aux.getColsSize().transpose()<<std::endl;


            std::cout<<"block_covariance_total_robot_error"<<std::endl;
            std::cout<<"size_rows="<<block_covariance_total_robot_error.getRowsSize().transpose()<<std::endl;
            std::cout<<"size_cols="<<block_covariance_total_robot_error.getColsSize().transpose()<<std::endl;
            */



            if(num_input_commands > 0)
                block_covariance_total_robot_error+=// Fu * Qu * Fu^t
                                                    block_jacobian_total_robot_error_state_wrt_error_input_commands*block_covariance_total_robot_error_inputs*block_jacobian_total_robot_error_state_wrt_error_input_commands.transpose();


            //Eigen::MatrixXd covariance_total_robot_error=BlockMatrix::convertToEigenDense(block_covariance_total_robot_error);



            // P(k+1|k)
#if 1 || _DEBUG_TIME_MSF_LOCALIZATION_CORE
        TimeStamp beginTimePredictedCovarianceErrorStateLastSum=getTimeStamp();
#endif

            block_predicted_covariance_error_state= // Fx * P * Fx^t
                                                    block_jacobian_total_robot_error_state*block_previous_covariance_error_state*block_jacobian_total_robot_error_state.transpose()+
                                                    // Fp * Qp * Fp^t + Fu * Qu * Fu^t + Fn * Qn * Fn^t
                                                    block_covariance_total_robot_error;

#if 1 || _DEBUG_TIME_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::predictCore() predict covariance error state last sum: "<<(getTimeStamp()-beginTimePredictedCovarianceErrorStateLastSum).nsec<<std::endl;
            this->log(logString.str());
        }
#endif




//            std::cout<<"block_predicted_covariance_error_state"<<std::endl;
//            std::cout<<"size_rows="<<block_predicted_covariance_error_state.getRowsSize().transpose()<<std::endl;
//            std::cout<<"size_cols="<<block_predicted_covariance_error_state.getColsSize().transpose()<<std::endl;



            /*
            ThePredictedState->covarianceMatrix->block(0,0,dimension_robot_error_state,dimension_robot_error_state)=
                    jacobianRobotErrorState*ThePreviousState->covarianceMatrix->block(0,0,dimension_robot_error_state,dimension_robot_error_state)*jacobianRobotErrorState.transpose() +
                    covariance_total_robot_error;
                    */

            /*
            ThePredictedState->covarianceMatrix->block(0,0,dimension_robot_error_state,dimension_robot_error_state)=
                    jacobianRobotErrorState*ThePreviousState->covarianceMatrix->block(0,0,dimension_robot_error_state,dimension_robot_error_state)*jacobianRobotErrorState.transpose() +
                    covariance_total_robot_error;
                    */



        }
        catch(...)
        {
            std::cout<<"Error in prediction covariances robot"<<std::endl;
        }
//std::cout<<"after operations"<<std::endl;




#if 1 || _DEBUG_TIME_MSF_LOCALIZATION_CORE
        TimeStamp beginTimePredictedCovarianceErrorStateAsEigen=getTimeStamp();
#endif

        // Store Covariance Error State as Eigen::MatrixXd
        (*ThePredictedState->covarianceMatrix)=BlockMatrix::convertToEigenDense(block_predicted_covariance_error_state);

#if 1 || _DEBUG_TIME_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::predictCore() predict covariance error state as eigen: "<<(getTimeStamp()-beginTimePredictedCovarianceErrorStateAsEigen).nsec<<std::endl;
            this->log(logString.str());
        }
#endif



#if 1 || _DEBUG_TIME_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::predictCore() predict covariance error state: "<<(getTimeStamp()-beginTimePredictedCovarianceErrorState).nsec<<std::endl;
            this->log(logString.str());
        }
#endif





    }


#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::predictCore() predict Core time cov in: "<<(getTimeStamp()-beginTimePredictCoreCov).nsec<<std::endl;
        this->log(logString.str());
    }
#endif



/*
    /// Cov Robot-Sensors

    {

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
        int sensor_i=0;
        for(std::list< std::shared_ptr<StateCore> >::iterator it1Sens=ThePredictedState->TheListSensorStateCore.begin();
            it1Sens!=ThePredictedState->TheListSensorStateCore.end();
            ++it1Sens)
        {

            // Dimensions
            int dimensionRobotErrorState=ThePredictedState->TheRobotStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorState();
            int dimensionSensorErrorState=(*it1Sens)->getMsfElementCoreSharedPtr()->getDimensionErrorState();


            // Calculate
            ThePredictedState->covarianceMatrix->block(WorkingInitPoint.row, WorkingInitPoint.col,dimensionRobotErrorState,dimensionSensorErrorState)=
                    jacobianRobotErrorState*ThePreviousState->covarianceMatrix->block(WorkingInitPoint.row, WorkingInitPoint.col,dimensionRobotErrorState,dimensionSensorErrorState)*(*it1Sens)->getJacobianErrorStateSensor(sensor_i).transpose();



            // Set the Symmetric
            Eigen::MatrixXd auxMatSym=ThePredictedState->covarianceMatrix->block(WorkingInitPoint.row, WorkingInitPoint.col,dimensionRobotErrorState,dimensionSensorErrorState);
            ThePredictedState->covarianceMatrix->block(WorkingInitPoint.col, WorkingInitPoint.row,dimensionSensorErrorState, dimensionRobotErrorState)=
                    auxMatSym.transpose();


            // Update Point
            WorkingInitPoint.row=0;
            WorkingInitPoint.col+=dimensionSensorErrorState;

            sensor_i++;
        }

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::predictCore() predict Core time cov rob-sen: "<<(getTimeStamp()-beginTimePredictCoreCovRobSen).nsec<<std::endl;
            this->log(logString.str());
        }
#endif

    }


    /// Cov Sensors


    /// Cov Robot-Map

    {
        // dimensions
        int dimensionSensorsErrorState=0;
        for(std::list< std::shared_ptr<StateCore> >::iterator it1Sens=ThePredictedState->TheListSensorStateCore.begin();
            it1Sens!=ThePredictedState->TheListSensorStateCore.end();
            ++it1Sens)
        {
            dimensionSensorsErrorState+=(*it1Sens)->getMsfElementCoreSharedPtr()->getDimensionErrorState();
        }

        // Init Point
        WorkingInitPoint.col=ThePredictedState->TheRobotStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorState()+dimensionSensorsErrorState;
        WorkingInitPoint.row=0;


        // Iterate on the map elements
        int map_element_i=0;
        for(std::list< std::shared_ptr<StateCore> >::iterator it1MapElement=ThePredictedState->TheListMapElementStateCore.begin();
            it1MapElement!=ThePredictedState->TheListMapElementStateCore.end();
            ++it1MapElement)
        {
            // Dimensions
            int dimensionRobotErrorState=ThePredictedState->TheRobotStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorState();
            int dimensionMapElementErrorState=(*it1MapElement)->getMsfElementCoreSharedPtr()->getDimensionErrorState();


            // Calculate
            ThePredictedState->covarianceMatrix->block(WorkingInitPoint.row, WorkingInitPoint.col,dimensionRobotErrorState,dimensionMapElementErrorState)=
                    jacobianRobotErrorState*ThePreviousState->covarianceMatrix->block(WorkingInitPoint.row, WorkingInitPoint.col,dimensionRobotErrorState,dimensionMapElementErrorState)*(*it1MapElement)->getJacobianErrorStateMapElement(map_element_i).transpose();



            // Set the Symmetric
            Eigen::MatrixXd auxMatSym=ThePredictedState->covarianceMatrix->block(WorkingInitPoint.row, WorkingInitPoint.col,dimensionRobotErrorState,dimensionMapElementErrorState);
            ThePredictedState->covarianceMatrix->block(WorkingInitPoint.col, WorkingInitPoint.row,dimensionMapElementErrorState, dimensionRobotErrorState)=
                    auxMatSym.transpose();



            // Update Point
            WorkingInitPoint.row=0;
            WorkingInitPoint.col+=dimensionMapElementErrorState;

            map_element_i++;
        }
    }



    /// Cov Sensors-Map
    // TODO
    {

    }


    /// Map

    {
        // dimension
        int dimensionSensorsErrorState=0;
        for(std::list< std::shared_ptr<StateCore> >::iterator it1Sens=ThePredictedState->TheListSensorStateCore.begin();
            it1Sens!=ThePredictedState->TheListSensorStateCore.end();
            ++it1Sens)
        {
            dimensionSensorsErrorState+=(*it1Sens)->getMsfElementCoreSharedPtr()->getDimensionErrorState();
        }

        // Init Point
        WorkingInitPoint.col=ThePredictedState->TheRobotStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorState()+dimensionSensorsErrorState;
        WorkingInitPoint.row=WorkingInitPoint.col;

        // Iterate
        int map_element_i=0;
        for(std::list< std::shared_ptr<StateCore> >::iterator it1MapElement=ThePredictedState->TheListMapElementStateCore.begin();
            it1MapElement!=ThePredictedState->TheListMapElementStateCore.end();
            ++it1MapElement)
        {

            // Dimension mapElement1
            int dimensionMapElement1ErrorState=(*it1MapElement)->getMsfElementCoreSharedPtr()->getDimensionErrorState();


            // Aux vars -> Jacobians
            Eigen::SparseMatrix<double> jacobianMapElement1ErrorState=(*it1MapElement)->getJacobianErrorStateMapElement(map_element_i);


            // Iterate again
            int map_element_j=map_element_i;
            for(std::list< std::shared_ptr<StateCore> >::iterator it2MapElement=it1MapElement;
                it2MapElement!=ThePredictedState->TheListMapElementStateCore.end();
                ++it2MapElement)
            {

                // Dimension mapElement2
                int dimensionMapElement2ErrorState=(*it2MapElement)->getMsfElementCoreSharedPtr()->getDimensionErrorState();


                // Calculate
                Eigen::MatrixXd CovarianceAuxMatrix=
                        jacobianMapElement1ErrorState*ThePreviousState->covarianceMatrix->block(WorkingInitPoint.row, WorkingInitPoint.col,dimensionMapElement1ErrorState,dimensionMapElement2ErrorState)*(*it2MapElement)->getJacobianErrorStateMapElement(map_element_j).transpose();



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

                map_element_j++;
            }

            // Update the dimension for the next map element
            WorkingInitPoint.row+=dimensionMapElement1ErrorState;
            WorkingInitPoint.col=WorkingInitPoint.row;

            map_element_i++;
        }
    }
*/



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


            /// Error Measurements

            // Num Measurement blocks
            int num_error_measurements=0;
            num_error_measurements=TheListMatchedMeasurements.size();


            //// Error State

            // Error State
            int dimensionErrorState=OldState->getDimensionErrorState();

            // Num Error State blocks
            int num_error_states=0;
            {
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
            }

            // Create vectors of state dimensions
            Eigen::VectorXi size_error_state;
            size_error_state.resize(num_error_states);
            // Fill
            {
                int num_error_state_i=0;
                // World
                size_error_state(num_error_state_i)=OldState->TheGlobalParametersStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorState();
                num_error_state_i++;
                // Robot
                size_error_state(num_error_state_i)=OldState->TheRobotStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorState();
                num_error_state_i++;
                // Inputs
                for(std::list< std::shared_ptr<StateCore> >::iterator itListInputState=OldState->TheListInputStateCore.begin();
                    itListInputState!=OldState->TheListInputStateCore.end();
                    ++itListInputState)
                {
                    size_error_state(num_error_state_i)=(*itListInputState)->getMsfElementCoreSharedPtr()->getDimensionErrorState();
                    num_error_state_i++;
                }
                // Sensors
                for(std::list< std::shared_ptr<StateCore> >::iterator itListSensorState=OldState->TheListSensorStateCore.begin();
                    itListSensorState!=OldState->TheListSensorStateCore.end();
                    ++itListSensorState)
                {
                    size_error_state(num_error_state_i)=(*itListSensorState)->getMsfElementCoreSharedPtr()->getDimensionErrorState();
                    num_error_state_i++;
                }
                // Map
                for(std::list< std::shared_ptr<StateCore> >::iterator itListMapElementState=OldState->TheListMapElementStateCore.begin();
                    itListMapElementState!=OldState->TheListMapElementStateCore.end();
                    ++itListMapElementState)
                {
                    size_error_state(num_error_state_i)=(*itListMapElementState)->getMsfElementCoreSharedPtr()->getDimensionErrorState();
                    num_error_state_i++;
                }
            }



#if _DEBUG_MSF_LOCALIZATION_ALGORITHM
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::update() dimension error state="<<dimensionErrorState<<" for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
                this->log(logString.str());
            }
#endif




            ///// Innovation vector

            // Block matrix
            BlockMatrix::MatrixDense block_innovation_vector;
            block_innovation_vector.resize(num_error_measurements, 1);

            // Fill
            {
                int num_error_measurements_i=0;
                for(std::list<std::shared_ptr<SensorMeasurementCore> >::const_iterator itListMatchedMeas=TheListMatchedMeasurements.begin(), itListPredictedMeas=TheListPredictedMeasurements.begin();
                    itListMatchedMeas!=TheListMatchedMeasurements.end() && itListPredictedMeas!=TheListMatchedMeasurements.end();
                    ++itListMatchedMeas, ++itListPredictedMeas)
                {
                    block_innovation_vector(num_error_measurements_i, 0)=
                            (*itListMatchedMeas)->getInnovation((*itListMatchedMeas), (*itListPredictedMeas));

                    num_error_measurements_i++;
                }
            }
            block_innovation_vector.analyse();

            // Dense vector
            Eigen::VectorXd innovationVector;

            innovationVector=BlockMatrix::convertToEigenDense(block_innovation_vector);





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
            block_jacobian_error_measurement_wrt_error_state.analyse();
            block_jacobian_error_measurement_wrt_error_parameters.analyse();


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
            block_jacobian_error_measurement_wrt_error_measurement.analyse();

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

            /*
            BlockMatrix::MatrixDense block_old_covariance_error_state;
            block_old_covariance_error_state.resize(num_error_states, num_error_states);

            block_old_covariance_error_state.createFromEigen((*OldState->covarianceMatrix), size_error_state, size_error_state);

            */


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
                for(std::list< std::shared_ptr<StateCore> >::iterator itListInputStateCore=OldState->TheListInputStateCore.begin();
                    itListInputStateCore!=OldState->TheListInputStateCore.end();
                    ++itListInputStateCore)
                {
                    block_covariance_error_parameters(num_error_state_i, num_error_state_i)=
                            (*itListInputStateCore)->getMsfElementCoreSharedPtr()->getCovarianceParameters();
                    num_error_state_i++;
                }

                // Sensors
                for(std::list<std::shared_ptr<StateCore> >::const_iterator itListSensorStateCore=OldState->TheListSensorStateCore.begin();
                    itListSensorStateCore!=OldState->TheListSensorStateCore.end();
                    ++itListSensorStateCore)
                {
                    block_covariance_error_parameters(num_error_state_i, num_error_state_i)=
                            (*itListSensorStateCore)->getMsfElementCoreSharedPtr()->getCovarianceParameters();
                    num_error_state_i++;
                }

                // Map
                for(std::list<std::shared_ptr<StateCore> >::const_iterator itListMapElementStateCore=OldState->TheListMapElementStateCore.begin();
                    itListMapElementStateCore!=OldState->TheListMapElementStateCore.end();
                    ++itListMapElementStateCore)
                {
                    block_covariance_error_parameters(num_error_state_i, num_error_state_i)=
                           (*itListMapElementStateCore)->getMsfElementCoreSharedPtr()->getCovarianceParameters();
                    num_error_state_i++;
                }
            }
            block_covariance_error_parameters.analyse();

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
            block_covariance_error_measurement.analyse();

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

            /*
            // Block
            BlockMatrix::MatrixDense block_innovation_covariance;

            block_innovation_covariance=
                    // Hp * Rp * Hp^t
                    block_jacobian_error_measurement_wrt_error_parameters*block_covariance_error_parameters*block_jacobian_error_measurement_wrt_error_parameters.transpose() +
                    // Hn * Rn * Hn^t
                    block_jacobian_error_measurement_wrt_error_measurement*block_covariance_error_measurement*block_jacobian_error_measurement_wrt_error_measurement.transpose() +
                    // Hx * P * Hx^t
                    block_jacobian_error_measurement_wrt_error_state*block_old_covariance_error_state*block_jacobian_error_measurement_wrt_error_state.transpose();


            // Inverse
            BlockMatrix::MatrixDense block_inverse_innovation_covariance;

            // TODO
            // block_inverse_innovation_covariance=block_innovation_covariance.inverse();

            */




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



            /// Get the updated state from the increment of the Error State

            // Init and resize block matrix
            BlockMatrix::MatrixDense block_increment_error_state;
            block_increment_error_state.resize(num_error_states,1);

            // Fill
#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
            TimeStamp begin_time_block_increment_error_state=getTimeStamp();
#endif

            block_increment_error_state.createFromEigen(incrementErrorState, size_error_state);

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::update() block_increment_error_state time: "<<(getTimeStamp()-begin_time_block_increment_error_state).nsec<<std::endl;
                this->log(logString.str());
            }
#endif

            // Update states
            {
#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
                TimeStamp begin_time_update_error_state_from_increment_error_state=getTimeStamp();
#endif

                int num_error_state_i=0;

                // World (Global Parameters)
                {
                    UpdatedState->TheGlobalParametersStateCore->updateStateFromIncrementErrorState(block_increment_error_state(num_error_state_i, 0));
                    num_error_state_i++;
                }

                // Robot
                {
                    UpdatedState->TheRobotStateCore->updateStateFromIncrementErrorState(block_increment_error_state(num_error_state_i, 0));
                    num_error_state_i++;
                }

                // Inputs
                for(std::list< std::shared_ptr<StateCore> >::iterator itListInputState=UpdatedState->TheListInputStateCore.begin();
                    itListInputState!=UpdatedState->TheListInputStateCore.end();
                    ++itListInputState)
                {
                    (*itListInputState)->updateStateFromIncrementErrorState(block_increment_error_state(num_error_state_i, 0));
                    num_error_state_i++;
                }

                // Sensors
                for(std::list< std::shared_ptr<StateCore> >::iterator itListSensorState=UpdatedState->TheListSensorStateCore.begin();
                    itListSensorState!=UpdatedState->TheListSensorStateCore.end();
                    ++itListSensorState)
                {
                    (*itListSensorState)->updateStateFromIncrementErrorState(block_increment_error_state(num_error_state_i, 0));
                    num_error_state_i++;
                }

                // Map
                for(std::list< std::shared_ptr<StateCore> >::iterator itListMapElementState=UpdatedState->TheListMapElementStateCore.begin();
                    itListMapElementState!=UpdatedState->TheListMapElementStateCore.end();
                    ++itListMapElementState)
                {
                    (*itListMapElementState)->updateStateFromIncrementErrorState(block_increment_error_state(num_error_state_i, 0));
                    num_error_state_i++;
                }

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
                {
                    std::ostringstream logString;
                    logString<<"MsfLocalizationCore::update() update_error_state_from_increment_error_state time: "<<(getTimeStamp()-begin_time_update_error_state_from_increment_error_state).nsec<<std::endl;
                    this->log(logString.str());
                }
#endif
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


#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
            TimeStamp beginErrorStateReset=getTimeStamp();
#endif


            /// Calculate jacobians reset error state

            {
                int num_error_states_i=0;

                // Global Parameters
                {
                    int error_jacobian_reset_error_state=UpdatedState->TheGlobalParametersStateCore->getMsfElementCoreSharedPtr()->
                            resetErrorStateJacobian(// time stamp
                                                    TheTimeStamp,
                                                    // Increment error state
                                                    block_increment_error_state(num_error_states_i, 0),
                                                    // current state
                                                    UpdatedState->TheGlobalParametersStateCore);
                    if(error_jacobian_reset_error_state)
                        return error_jacobian_reset_error_state;
                    num_error_states_i++;
                }

                // Robot
                {
                    int error_jacobian_reset_error_state=UpdatedState->TheRobotStateCore->getMsfElementCoreSharedPtr()->
                            resetErrorStateJacobian(// time stamp
                                                    TheTimeStamp,
                                                    // Increment error state
                                                    block_increment_error_state(num_error_states_i, 0),
                                                    // current state
                                                    UpdatedState->TheRobotStateCore);
                    if(error_jacobian_reset_error_state)
                        return error_jacobian_reset_error_state;
                    num_error_states_i++;
                }

                // Inputs
                for(std::list< std::shared_ptr<StateCore> >::iterator itListInputState=UpdatedState->TheListInputStateCore.begin();
                    itListInputState!=UpdatedState->TheListInputStateCore.end();
                    ++itListInputState)
                {
                    int error_jacobian_reset_error_state=(*itListInputState)->getMsfElementCoreSharedPtr()->
                            resetErrorStateJacobian(// time stamp
                                                    TheTimeStamp,
                                                    // Increment error state
                                                    block_increment_error_state(num_error_states_i, 0),
                                                    // current state
                                                    (*itListInputState));
                    if(error_jacobian_reset_error_state)
                        return error_jacobian_reset_error_state;
                    num_error_states_i++;
                }

                // Sensors
                for(std::list< std::shared_ptr<StateCore> >::iterator itListSensorState=UpdatedState->TheListSensorStateCore.begin();
                    itListSensorState!=UpdatedState->TheListSensorStateCore.end();
                    ++itListSensorState)
                {
                    int error_jacobian_reset_error_state=(*itListSensorState)->getMsfElementCoreSharedPtr()->
                            resetErrorStateJacobian(// time stamp
                                                    TheTimeStamp,
                                                    // Increment error state
                                                    block_increment_error_state(num_error_states_i, 0),
                                                    // current state
                                                    (*itListSensorState));
                    if(error_jacobian_reset_error_state)
                        return error_jacobian_reset_error_state;
                    num_error_states_i++;
                }

                // Map
                for(std::list< std::shared_ptr<StateCore> >::iterator itListMapElementState=UpdatedState->TheListMapElementStateCore.begin();
                    itListMapElementState!=UpdatedState->TheListMapElementStateCore.end();
                    ++itListMapElementState)
                {
                    int error_jacobian_reset_error_state=(*itListMapElementState)->getMsfElementCoreSharedPtr()->
                            resetErrorStateJacobian(// time stamp
                                                    TheTimeStamp,
                                                    // Increment error state
                                                    block_increment_error_state(num_error_states_i, 0),
                                                    // current state
                                                    (*itListMapElementState));
                    if(error_jacobian_reset_error_state)
                        return error_jacobian_reset_error_state;
                    num_error_states_i++;
                }
            }


            /// Create the full block jacobian

            BlockMatrix::MatrixSparse block_jacobian_reset_error_state;
            block_jacobian_reset_error_state.resize(num_error_states, num_error_states);

            // Fill blocks
            {
                int num_error_states_i=0;

                // World
                {
                    block_jacobian_reset_error_state(num_error_states_i, num_error_states_i)=
                            UpdatedState->TheGlobalParametersStateCore->jacobian_error_state_reset_;
                    num_error_states_i++;
                }
                // Robot
                {
                    block_jacobian_reset_error_state(num_error_states_i, num_error_states_i)=
                            UpdatedState->TheRobotStateCore->jacobian_error_state_reset_;
                    num_error_states_i++;
                }
                // Inputs
                for(std::list< std::shared_ptr<StateCore> >::iterator itListInputState=UpdatedState->TheListInputStateCore.begin();
                    itListInputState!=UpdatedState->TheListInputStateCore.end();
                    ++itListInputState)
                {
                    block_jacobian_reset_error_state(num_error_states_i, num_error_states_i)=
                            (*itListInputState)->jacobian_error_state_reset_;
                    num_error_states_i++;
                }
                // Sensors
                for(std::list< std::shared_ptr<StateCore> >::iterator itListSensorState=UpdatedState->TheListSensorStateCore.begin();
                    itListSensorState!=UpdatedState->TheListSensorStateCore.end();
                    ++itListSensorState)
                {
                    block_jacobian_reset_error_state(num_error_states_i, num_error_states_i)=
                            (*itListSensorState)->jacobian_error_state_reset_;
                    num_error_states_i++;
                }
                // Map Elements
                for(std::list< std::shared_ptr<StateCore> >::iterator itListMapElementState=UpdatedState->TheListMapElementStateCore.begin();
                    itListMapElementState!=UpdatedState->TheListMapElementStateCore.end();
                    ++itListMapElementState)
                {
                    block_jacobian_reset_error_state(num_error_states_i, num_error_states_i)=
                            (*itListMapElementState)->jacobian_error_state_reset_;
                    num_error_states_i++;
                }
            }
            block_jacobian_reset_error_state.analyse();


            /// Create sparse jacobian

            Eigen::SparseMatrix<double> jacobian_reset_error_state;

            jacobian_reset_error_state=BlockMatrix::convertToEigenSparse(block_jacobian_reset_error_state);



            /// Update Covariance Error State

            // Aux Covariance
            Eigen::MatrixXd AuxiliarCovarianceMatrix(dimensionErrorState, dimensionErrorState);
            AuxiliarCovarianceMatrix.setZero();

            AuxiliarCovarianceMatrix=*UpdatedState->covarianceMatrix;


            // Update Covariance Matrix
            *UpdatedState->covarianceMatrix=jacobian_reset_error_state*AuxiliarCovarianceMatrix*jacobian_reset_error_state.transpose();



#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::update() error state reset time: "<<(getTimeStamp()-beginErrorStateReset).nsec<<std::endl;
                this->log(logString.str());
            }
#endif


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
            std::shared_ptr<StateCore> TheNewMapElementStateCore;

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
            TheListNewMapElementsStateCore.push_back(std::dynamic_pointer_cast<MapElementStateCore>(TheNewMapElementStateCore));

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
                for(std::list< std::shared_ptr<StateCore> >::const_iterator itListSensorCore=UpdatedState->TheListSensorStateCore.begin();
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
                for(std::list< std::shared_ptr<StateCore> >::const_iterator itListMapElementCore=UpdatedState->TheListMapElementStateCore.begin();
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
            UpdatedState->TheListMapElementStateCore.push_back(std::dynamic_pointer_cast<StateCore>(*itNewMapElementsStateCore));
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
