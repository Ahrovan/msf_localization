
#include "msf_localization_core/msfLocalization.h"




MsfLocalizationCore::MsfLocalizationCore()
{
    // Flags
    stateEstimationEnabled=false;

    // Create Robot Core
    // No. Created when reading config files

    // Create Global Parameters Core
    TheGlobalParametersCore=std::make_shared<GlobalParametersCore>();

    // Sensors
    //Id
    firstAvailableSensorId=0;

    // Map elements
    // Id
    firstAvailableMapElementId=0;



#if _USE_BUFFER_IN_STATE_ESTIMATION
    // Create Storage Core
    TheMsfStorageCore=std::make_shared<MsfStorageCore>();
#else
    // TODO
#endif


    // New Measurement
    new_measurement_lock_=new std::unique_lock<std::mutex>(new_measurement_mutex_);


    // LOG
    const char* env_p = std::getenv("FUSEON_STACK");


    time_t rawtime;
      struct tm * timeinfo;
      char buffer[80];

      time (&rawtime);
      timeinfo = localtime(&rawtime);

      strftime(buffer,80,"%d-%m-%Y_%I:%M:%S",timeinfo);
      std::string str(buffer);



    logPath=std::string(env_p)+"/logs/"+"logMsfLocalizationCoreFile"+str+".txt";

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
    // Close
    close();

    // End
    return;
}


int MsfLocalizationCore::init()
{
    return 0;
}

int MsfLocalizationCore::close()
{

    // Stop threads
    stopThreads();

    // Delete
    delete new_measurement_lock_;

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


    // Set the initial state with the correct time stamp

#if _USE_BUFFER_IN_STATE_ESTIMATION

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


#else
    // TODO


#endif



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

int MsfLocalizationCore::setMeasurement(const TimeStamp& time_stamp,
                                        const std::shared_ptr<SensorMeasurementCore>& sensor_measurement)
{
    int errorSetMeasurement=0;

#if _USE_BUFFER_IN_STATE_ESTIMATION
    // Add to buffer
    errorSetMeasurement=this->TheMsfStorageCore->setMeasurement(time_stamp, sensor_measurement);
#else
    // Create new Sensor Measurement Component
    std::shared_ptr<SensorMeasurementComponent> new_measurement=std::make_shared<SensorMeasurementComponent>();

    // Fill it with the measurement
    new_measurement->list_sensor_measurement_core_.push_back(sensor_measurement);

    // Push back to the list
    list_sensor_measurements_.push_back(new_measurement);
#endif

    if(errorSetMeasurement)
        return -1;

    this->semaphoreNewMeasurementNotify(time_stamp);

    return 0;
}

int MsfLocalizationCore::setMeasurementList(const TimeStamp& time_stamp,
                                            const std::list< std::shared_ptr<SensorMeasurementCore> >& list_sensor_measurement)
{
    int errorSetMeasurement=0;

#if _USE_BUFFER_IN_STATE_ESTIMATION
    // Add to buffer
    errorSetMeasurement=this->TheMsfStorageCore->setMeasurementList(time_stamp, list_sensor_measurement);
#else
    // Create new Sensor Measurement Component
    std::shared_ptr<SensorMeasurementComponent> new_measurement=std::make_shared<SensorMeasurementComponent>();

    // Fill it with the list of measurement
    for(std::list< std::shared_ptr<SensorMeasurementCore> >::const_iterator it_list_sensor_measurement=list_sensor_measurement.begin();
        it_list_sensor_measurement!=list_sensor_measurement.end();
        ++it_list_sensor_measurement)
    {
        new_measurement->list_sensor_measurement_core_.push_back(*it_list_sensor_measurement);
    }

    // Push back to the list
    list_sensor_measurements_.push_back(new_measurement);
#endif

    if(errorSetMeasurement)
        return -1;

    this->semaphoreNewMeasurementNotify(time_stamp);

    return 0;
}

int MsfLocalizationCore::semaphoreNewMeasurementWait(TimeStamp& new_measurement_time_stamp)
{
    // Wait
    new_measurement_condition_variable_.wait(*new_measurement_lock_);

    // Get time stamp
    new_measurement_time_stamp=new_measurement_time_stamp_;

    return 0;
}

int MsfLocalizationCore::semaphoreNewMeasurementNotify(const TimeStamp& new_measurement_time_stamp)
{
    // Set time stamp
    new_measurement_time_stamp_=new_measurement_time_stamp;

    // Notify
    new_measurement_condition_variable_.notify_all();

    return 0;
}

int MsfLocalizationCore::setInputCommand(const TimeStamp& time_stamp,
                                        const std::shared_ptr<InputCommandCore>& input_command)
{
    int errorSetInputCommand=0;

#if _USE_BUFFER_IN_STATE_ESTIMATION
    // Add to buffer
    errorSetInputCommand=this->TheMsfStorageCore->setInputCommand(time_stamp, input_command);
#else
    // Create new Sensor Measurement Component
    std::shared_ptr<InputCommandComponent> new_input_command=std::make_shared<InputCommandComponent>();

    // Fill it with the measurement
    new_input_command->list_input_command_core_.push_back(input_command);

    // Push back to the list
    list_input_commands_.push_back(new_input_command);
#endif

    if(errorSetInputCommand)
        return -1;

    return 0;
}

int MsfLocalizationCore::setInputCommandList(const TimeStamp& time_stamp,
                        const std::list< std::shared_ptr<InputCommandCore> >& list_input_command)
{
    int errorSetInputCommand=0;

#if _USE_BUFFER_IN_STATE_ESTIMATION
    // Add to the buffer
    errorSetInputCommand=this->TheMsfStorageCore->setInputCommandList(time_stamp, list_input_command);
#else
    // Create new Sensor Measurement Component
    std::shared_ptr<InputCommandComponent> new_input_command=std::make_shared<InputCommandComponent>();

    // Fill it with the list of measurement
    for(std::list< std::shared_ptr<InputCommandCore> >::const_iterator it_list_input_command=list_input_command.begin();
        it_list_input_command!=list_input_command.end();
        ++it_list_input_command)
    {
        new_input_command->list_input_command_core_.push_back(*it_list_input_command);
    }

    // Push back to the list
    list_input_commands_.push_back(new_input_command);
#endif

    if(errorSetInputCommand)
        return -1;

    return 0;
}

TimeStamp MsfLocalizationCore::getTimeStamp()
{
#if _DEBUG_MSF_LOCALIZATION_CORE
    std::cout<<"MsfLocalizationCore::getTimeStamp()"<<std::endl;
#endif

    TimeStamp TheTimeStamp;
    return TheTimeStamp;
}

bool MsfLocalizationCore::isAlive()
{
    return true;
}

int MsfLocalizationCore::predictThreadFunction()
{
#if _DEBUG_MSF_LOCALIZATION_CORE
    logFile<<"MsfLocalizationCore::predictThreadFunction()"<<std::endl;
#endif

    // TODO Finish

    return 0;
}


int MsfLocalizationCore::predictThreadStep()
{
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


#if _USE_BUFFER_IN_STATE_ESTIMATION
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

            if(this->predictInBuffer(TheTimeStamp))
            {
                // Error
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
                {
                    std::ostringstream logString;
                    logString<<"MsfLocalizationROS::predictThreadFunction() error in predictInBuffer()"<<std::endl;
                    this->log(logString.str());
                }
#endif
                return -1;
//            predictThreadState.setNotProcessing();
            }
        }

#else

        // TODO

#endif


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


        return 0;
}

#if _USE_BUFFER_IN_STATE_ESTIMATION
int MsfLocalizationCore::bufferManagerThreadFunction()
{
#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::bufferManagerThreadFunction()"<<std::endl;
        this->log(logString.str());
    }
#endif


    while(isAlive())
    {

#if _DEBUG_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::bufferManagerThreadFunction() loop init"<<std::endl;
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
                logString<<"MsfLocalizationCore::bufferManagerThreadFunction() error 0!"<<std::endl;
                this->log(logString.str());
            }
#endif
            continue;
        }

#if 1 || _DEBUG_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::bufferManagerThreadFunction() updating TS: sec="<<OldestTimeStamp.sec<<" s; nsec="<<OldestTimeStamp.nsec<<" ns"<<std::endl;
            this->log(logString.str());
        }
#endif


#if 1 || _DEBUG_MSF_LOCALIZATION_CORE
        {
            this->log(this->TheMsfStorageCore->getDisplayOutdatedElements());
        }
#endif


#if _BUFFER_PROPAGATION_MULTI_THREADING
        // Multi-threading
        std::thread propagation_step_thread(&MsfLocalizationCore::bufferPropagationStep, this, OldestTimeStamp);
        propagation_step_thread.detach();
#else
        // Single-thread
        int error_propagating_buffer=bufferPropagationStep(OldestTimeStamp);

        if(error_propagating_buffer)
        {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::bufferManagerThreadFunction() error "<<error_propagating_buffer<<" bufferPropagationStep()"<<std::endl;
                this->log(logString.str());
            }
#endif
            continue;
        }
#endif

    }


#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::bufferManagerThreadFunction() ended"<<std::endl;
        this->log(logString.str());
    }
#endif

    return 0;
}
#endif


#if _USE_BUFFER_IN_STATE_ESTIMATION
int MsfLocalizationCore::bufferPropagationStep(const TimeStamp &time_stamp)
{
#if _BUFFER_PROPAGATION_MULTI_THREADING
    num_buffer_propagation_threads++;
    std::cout<<"num_buffer_propagation_threads= "<<num_buffer_propagation_threads<<std::endl;
#endif

    // Check if the current element needs to be updated or if it can be deleted
    {
        int error_remove_unnecessary_state=removeUnnecessaryStateFromBuffer(time_stamp);
        if(error_remove_unnecessary_state < 0)
        {
            // Error getting elements
            return -1;
        }
        else if(error_remove_unnecessary_state == 0)
        {
            // Succed in removing element.
            // Find the next element in the buffer and mark it as outdated
            findNextElementInBufferAndAddOutdatedList(time_stamp);
            // Finish
            return 0;
        }
        else
        {
            // Cannot be removed. Need to continue
        }
    }


    // Run predict and store updated predicted element
    {
#if _DEBUG_TIME_MSF_LOCALIZATION_THREAD
        TimeStamp begin = this->getTimeStamp();
#endif


        int errorPredict=this->predictInBuffer(time_stamp);

        if(errorPredict)
        {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::bufferPropagationStep() error predict() "<<errorPredict<<std::endl;
                this->log(logString.str());
            }
#endif

            // Add to the processing list -> No
            //this->TheMsfStorageCore->addOutdatedElement(OldestTimeStamp);

            // Delete from buffer to avoid using it! -> No
            //this->TheMsfStorageCore->purgeElementRingBuffer(OldestTimeStamp);

#if _BUFFER_PROPAGATION_MULTI_THREADING
            num_buffer_propagation_threads--;
#endif

            return -1;
        }

#if _DEBUG_TIME_MSF_LOCALIZATION_THREAD
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::bufferPropagationStep() -> predict() time: "<<(getTimeStamp()-begin).toNSec()<<" ns"<<std::endl;
        this->log(logString.str());
#endif
    }


    // Request for updates of pseudo-measurements (if any)
    // TODO ???
    // This will block the buffer update. It should do the request, mark this request and leave to be able to process other measurements.


    // Run update (if there are measurements)
    {
        int errorUpdate=0;


#if _DEBUG_TIME_MSF_LOCALIZATION_THREAD
        TimeStamp begin = getTimeStamp();
#endif

        errorUpdate=this->updateInBuffer(time_stamp);


#if _DEBUG_TIME_MSF_LOCALIZATION_THREAD
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::bufferPropagationStep() -> updateInBuffer() time: "<<(getTimeStamp()-begin).toNSec()<<" ns"<<std::endl;
        this->log(logString.str());
#endif


#if _DEBUG_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::bufferPropagationStep() updateInBuffer response: "<<errorUpdate<<std::endl;
            this->log(logString.str());
        }
#endif



        if(errorUpdate)
        {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::bufferPropagationStep() error updateInBuffer() "<<errorUpdate<<std::endl;
                this->log(logString.str());
            }
#endif

            // Add to the processing list
            this->TheMsfStorageCore->addOutdatedElement(time_stamp);

#if _BUFFER_PROPAGATION_MULTI_THREADING
            num_buffer_propagation_threads--;
#endif

            // Continue
            return -2;
        }

    }


    // Find the next element in the buffer and mark it as outdated
    {
        findNextElementInBufferAndAddOutdatedList(time_stamp);
    }


    // Purge the buffer
    {
#if _DEBUG_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::bufferPropagationStep() Going to purge the ring"<<std::endl;
            this->log(logString.str());
        }
#endif

        // Purge the buffer ??
        this->TheMsfStorageCore->purgeRingBuffer(50);


        // Display the buffer
        //this->TheMsfStorageCore->displayRingBuffer();


        // Purge the buffer
        //this->TheMsfStorageCore->purgeRingBuffer(20);


        // Sleep
        //bufferManagerRate.sleep();

    }


#if _BUFFER_PROPAGATION_MULTI_THREADING
    num_buffer_propagation_threads--;
#endif

#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::bufferPropagationStep() end"<<std::endl;
        this->log(logString.str());
    }
#endif


    // End
    return 0;
}
#endif

#if _USE_BUFFER_IN_STATE_ESTIMATION
int MsfLocalizationCore::removeUnnecessaryStateFromBuffer(const TimeStamp &time_stamp)
{
    // Element cannot be removed and need to be processed
    if(predict_model_time_<0)
    {
        return 1;
    }

    // Get current element (k)
    std::shared_ptr<StateEstimationCore> current_element;
    int error_get_current_element=this->TheMsfStorageCore->getElement(time_stamp, current_element);
    if( error_get_current_element || !current_element )
    {
        std::cout<<"Error getElement"<<std::endl;
        return -10;
    }
    //std::cout<<"time_stamp: sec="<<time_stamp.sec<<" s; nsec="<<time_stamp.nsec<<" ns"<<std::endl;

    // No prediction requested. Only if it has measurements
    if(predict_model_time_==0)
    {
        if( current_element->hasInputCommand() && !current_element->hasMeasurement() )
        {
            return 0;
        }
        else
        {
            return 1;
        }
    }

    // Checks if has input commands or measurements
    if( current_element->hasInputCommand() || current_element->hasMeasurement() )
    {
        //std::cout<<"\t has measurements or input commands"<<std::endl;
        // Cannot be removed
        return 1;
    }

    // Free the pointer
    if(current_element)
        current_element.reset();

    // Get previous element (k-1)
    TimeStamp time_stamp_previous_element;
    int error_get_previous_time_stamp=this->TheMsfStorageCore->getPreviousTimeStamp(time_stamp, time_stamp_previous_element);
    //std::cout<<"\t time_stamp_previous_element: sec="<<time_stamp_previous_element.sec<<" s; nsec="<<time_stamp_previous_element.nsec<<" ns"<<std::endl;

    // Check if no error getting timestamps
    if( error_get_previous_time_stamp )
    {
        return 1;
    }


    // Get following element (k+1)
    TimeStamp time_stamp_following_element;
    int error_get_next_time_stamp=this->TheMsfStorageCore->getNextTimeStamp(time_stamp, time_stamp_following_element);
    //std::cout<<"\t time_stamp_following_element: sec="<<time_stamp_following_element.sec<<" s; nsec="<<time_stamp_following_element.nsec<<" ns"<<std::endl;

    // Check if no error getting timestamps
    if( error_get_next_time_stamp )
    {
        return 2;
    }


    //std::cout<<"\t increment: sec="<<(time_stamp_following_element-time_stamp_previous_element).sec<<" s; nsec="<<(time_stamp_following_element-time_stamp_previous_element).nsec<<" ns"<<std::endl;

    // check if delta time stamps beetween k-1 and k+1 <= predict_model_time_
    if( (time_stamp_following_element-time_stamp_previous_element) <= TimeStamp(predict_model_time_) )
    {
        //std::cout<<"\t\t deleting!"<<std::endl;

        // Delete current_element and finish
        int error_purge_element=this->TheMsfStorageCore->purgeElementRingBuffer(time_stamp);
        if(error_purge_element)
        {
            std::cout<<"!!!!! ERROR PURGE ELEMENT !!!!!!"<<std::endl;
            return -3;
        }
        return 0;
    }
    else
    {
        // Cannot be removed
        return 10;
    }



    return -100;
}
#endif

#if _USE_BUFFER_IN_STATE_ESTIMATION
int MsfLocalizationCore::findNextElementInBufferAndAddOutdatedList(const TimeStamp& time_stamp)
{
    TimeStamp TheNewOutdatedTimeStamp;

#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::findNextElementInBufferAndAddOutdatedList() Going to get next time stamp"<<std::endl;
        this->log(logString.str());
    }
#endif

    if(!this->TheMsfStorageCore->getNextTimeStamp(time_stamp, TheNewOutdatedTimeStamp))
    {
#if _DEBUG_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::findNextElementInBufferAndAddOutdatedList() Adding to be processed TS: sec="<<TheNewOutdatedTimeStamp.sec<<" s; nsec="<<TheNewOutdatedTimeStamp.nsec<<" ns"<<std::endl;
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
            logString<<"MsfLocalizationCore::findNextElementInBufferAndAddOutdatedList() Nothing new to be added"<<std::endl;
            this->log(logString.str());
        }
#endif
    }

    // End
    return 0;
}
#endif

int MsfLocalizationCore::publishThreadFunction()
{
    return -1;
}

int MsfLocalizationCore::publishNewMeasurementNotificationThreadFunction()
{
    TimeStamp new_measurement_stamp;

    while(isAlive())
    {
        // Wait until new measurement arrives
        this->semaphoreNewMeasurementWait(new_measurement_stamp);

        // Publish new measurement notification
        publishNewMeasurementNotification(new_measurement_stamp);

    }


    return 0;
}

int MsfLocalizationCore::publishNewMeasurementNotification(const TimeStamp& measurement_time_stamp)
{
    return -1;
}

int MsfLocalizationCore::startThreads()
{
    // Prediction state thread
    predictThread=new std::thread(&MsfLocalizationCore::predictThreadFunction, this);

#if _USE_BUFFER_IN_STATE_ESTIMATION
    // Buffer Manager thread
    bufferManagerThread=new std::thread(&MsfLocalizationCore::bufferManagerThreadFunction, this);
#endif

    // Publish thread
    publish_thread_=new std::thread(&MsfLocalizationCore::publishThreadFunction, this);

    // New measurement notification thread
    new_measurement_notification_thread_=new std::thread(&MsfLocalizationCore::publishNewMeasurementNotificationThreadFunction, this);


    return 0;
}

int MsfLocalizationCore::stopThreads()
{
    delete predictThread;
#if _USE_BUFFER_IN_STATE_ESTIMATION
    delete bufferManagerThread;
#endif
    delete publish_thread_;
    delete new_measurement_notification_thread_;

    return 0;
}

int MsfLocalizationCore::getStateByStamp(const TimeStamp& requested_time_stamp,
                                        TimeStamp& received_time_stamp,
                                        std::shared_ptr<StateEstimationCore>& received_state)
{
    // Check if state estimation is enabled
    if(this->isStateEstimationEnabled())
    {

        // Get previous element with state
        int error_get_element=-1;

#if _USE_BUFFER_IN_STATE_ESTIMATION
        error_get_element=this->TheMsfStorageCore->getElementWithStateEstimateByStamp(requested_time_stamp,
                                                                                          received_time_stamp, received_state);
#else
        // TODO
#endif

        // Check
        if(error_get_element || !received_state)
            return -1;


        // Check time stamps
        if( received_time_stamp < requested_time_stamp )
        {
            // Do not predict
            if( predict_model_time_ < 0 )
            {
                // Do nothing
            }
            // Predict to adjust
            else
            {
                // Reset received_state
                received_state.reset();

#if _DEBUG_MSF_LOCALIZATION_CORE
                {
                    std::ostringstream logString;
                    logString<<"MsfLocalizationCore::getStateByStamp() predicting TS: sec="<<requested_time_stamp.sec<<" s; nsec="<<requested_time_stamp.nsec<<" ns"<<std::endl;
                    this->log(logString.str());
                }
#endif

                // Predict
#if _USE_BUFFER_IN_STATE_ESTIMATION
                if(this->predictInBufferNoAddBuffer(requested_time_stamp, received_state))
                {
                    // Error
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
                    {
                        std::ostringstream logString;
                        logString<<"MsfLocalizationROS::publishThreadFunction() error in predictInBufferNoAddBuffer()"<<std::endl;
                        this->log(logString.str());
                    }
#endif
                    return -2;
                }
#else
                // TODO
#endif

                // Set the time stamp
                received_time_stamp=requested_time_stamp;

            }

        }
        else
        {
            // Do nothing
        }
    }
    // State Estimation disabled
    else
    {
        // Get the last state estimation
#if _USE_BUFFER_IN_STATE_ESTIMATION
        this->TheMsfStorageCore->getLastElementWithStateEstimate(received_time_stamp, received_state);
#else
        // TODO
#endif

        // Time Stamp is null, put the current one
        if(received_time_stamp==TimeStamp(0,0))
        {
            received_time_stamp=requested_time_stamp;
        }

    }

    // Error with received state
    if(!received_state)
    {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::getStateByStamp() error !received_state"<<std::endl;
            this->log(logString.str());
        }
#endif
        return -3;
    }


//    std::cout<<"req time: sec="<<requested_time_stamp.sec<<"s; nsec="<<requested_time_stamp.nsec<<std::endl;
//    std::cout<<"rec time: sec="<<received_time_stamp.sec<<"s; nsec="<<received_time_stamp.nsec<<std::endl;

    // End ok
    return 0;
}


#if _USE_BUFFER_IN_STATE_ESTIMATION
int MsfLocalizationCore::getPreviousState(const TimeStamp &TheTimeStamp, TimeStamp& ThePreviousTimeStamp, std::shared_ptr<StateEstimationCore>& ThePreviousState)
{
    // Get the previous not the last!
    if(TheMsfStorageCore->getPreviousElementWithStateEstimateByStamp(TheTimeStamp, ThePreviousTimeStamp, ThePreviousState))
    {
#if _DEBUG_MSF_LOCALIZATION_CORE
        logFile<<"MsfLocalizationCore::getPreviousState() error getPreviousElementWithStateEstimateByStamp"<<std::endl;
#endif
        return -1;
    }

    // Checks
    if(!ThePreviousState)
    {
#if _DEBUG_MSF_LOCALIZATION_CORE
        logFile<<"MsfLocalizationCore::getPreviousState() error !PreviousState"<<std::endl;
#endif
        return -2;
    }


    // Check
    if(!ThePreviousState->hasState())
    {
#if _DEBUG_MSF_LOCALIZATION_CORE
        logFile<<"MsfLocalizationCore::getPreviousState() error !PreviousState->hasState()"<<std::endl;
#endif
        return -3;
    }

    return 0;
}
#endif


int MsfLocalizationCore::findInputCommands(const TimeStamp &TheTimeStamp,
                                           std::shared_ptr<InputCommandComponent>& input_command)
{

    // Create Pointer
    if(!input_command)
        input_command=std::make_shared<InputCommandComponent>();

    // Clear the list -> Just in case
    input_command->list_input_command_core_.clear();

    // Iterate over the input cores
    for(std::list<std::shared_ptr<InputCore>>::iterator itInputCore=this->TheListOfInputCore.begin();
        itInputCore!=this->TheListOfInputCore.end();
        ++itInputCore)
    {

        // Command
        std::shared_ptr<InputCommandCore> input_command_core;
        int error_get_previous_input_command;


        // Check if input core stores commands automatically in the buffer, or if it needs to request it
        if((*itInputCore)->isInputActiveRequest())
        {
            // Request input command
            TimeStamp received_time_stamp;

            error_get_previous_input_command=(*itInputCore)->getInputCommand(TheTimeStamp,
                                                                             received_time_stamp,
                                                                             input_command_core);

        }
        else
        {
            // Search input command in buffer
#if _USE_BUFFER_IN_STATE_ESTIMATION
            error_get_previous_input_command=TheMsfStorageCore->getPreviousInputCommandByStampAndInputCore(TheTimeStamp, *itInputCore, input_command_core);
#else
            // TODO
#endif

        }



        // Check if acquired
        if(error_get_previous_input_command || !input_command_core)
        {
            //std::cout<<"MsfLocalizationCore::findInputCommands error_get_previous_input_command"<<std::endl;
            //return error_get_previous_input_command;

            // Create empty for reference
            input_command_core=std::make_shared<InputCommandCore>(std::dynamic_pointer_cast<InputCore>((*itInputCore)->getMsfElementCoreSharedPtr()));

        }



        // Set in the variable ThePreviousState
        input_command->list_input_command_core_.push_back(input_command_core);

    }


    return 0;
}


#if _USE_BUFFER_IN_STATE_ESTIMATION
int MsfLocalizationCore::predictInBuffer(const TimeStamp &TheTimeStamp)
{
    // The predicted state -> New element to be added to the buffer
    std::shared_ptr<StateEstimationCore> PredictedState;


    // Predict Add buffer
    int error_predict_add_buffer=this->predictInBufferAddBuffer(TheTimeStamp, PredictedState);


    // Check error
    if(error_predict_add_buffer)
        return error_predict_add_buffer;


    // Release the predicted state pointer -> Not really needed
    if(PredictedState)
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


int MsfLocalizationCore::predictInBufferAddBuffer(const TimeStamp& TheTimeStamp,
                                                  std::shared_ptr<StateEstimationCore>& PredictedState)
{
    if(!isStateEstimationEnabled())
        return 0;

#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::predictInBufferAddBuffer() TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
        this->log(logString.str());
    }
#endif

    // The predicted state -> New element to be added to the buffer
    if(!PredictedState)
        PredictedState=std::make_shared<StateEstimationCore>();


    // Get the predicted element if any
    std::shared_ptr<StateEstimationCore> OldPredictedState;
    if(this->TheMsfStorageCore->getElement(TheTimeStamp, OldPredictedState))
    {
#if _DEBUG_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::predictInBufferAddBuffer() no predicted element found, must be created one!"<<std::endl;
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
        PredictedState->sensor_measurement_component_=OldPredictedState->sensor_measurement_component_;

        // Inputs
        PredictedState->input_command_component_=OldPredictedState->input_command_component_;

        // Nothing else needed
    }


    // Release the old predicted state pointer. Not used anymore
    if(OldPredictedState)
        OldPredictedState.reset();



    ///// Predict Core
    int error=predictInBufferSemiCore(TheTimeStamp, PredictedState);
    if(error)
        return error;




    /////// Add element to the buffer -> Check if the element that is going to be added is not used

    // More or less sure that there are no more users of updated state
    {
        int error_msf_storage_add_element=TheMsfStorageCore->addElement(TheTimeStamp, PredictedState);
        if(error_msf_storage_add_element)
        {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
            std::cout<<"!!Error addElement predictInBufferAddBuffer()"<<error_msf_storage_add_element<<std::endl;
#endif
            return -2;
        }
    }



#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::predictInBufferAddBuffer() ended TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
        this->log(logString.str());
    }
#endif

    // End
    return 0;
}

int MsfLocalizationCore::predictInBufferNoAddBuffer(const TimeStamp& TheTimeStamp,
                                            std::shared_ptr<StateEstimationCore>& ThePredictedState)
{
    if(!isStateEstimationEnabled())
        return 0;

#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::predictInBufferNoAddBuffer() TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
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
            logString<<"MsfLocalizationROS::predictInBufferNoAddBuffer() no predicted element found, must be created one!"<<std::endl;
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
        ThePredictedState->sensor_measurement_component_=OldPredictedState->sensor_measurement_component_;

        // Inputs
        ThePredictedState->input_command_component_=OldPredictedState->input_command_component_;

        // Nothing else needed
    }

    // Release the old predicted state pointer
    OldPredictedState.reset();




    ///// Predict Core
    int error=predictInBufferSemiCore(TheTimeStamp, ThePredictedState);
    if(error)
        return error;




#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::predictInBufferNoAddBuffer() ended TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
        this->log(logString.str());
    }
#endif

    // End
    return 0;
}




int MsfLocalizationCore::predictInBufferSemiCore(const TimeStamp &ThePredictedTimeStamp, std::shared_ptr<StateEstimationCore>& ThePredictedState)
{

    // Get the last predicted state from the buffer
    TimeStamp PreviousTimeStamp;
    std::shared_ptr<StateEstimationCore> PreviousState;

    int error_get_previous_state=this->getPreviousState(ThePredictedTimeStamp, PreviousTimeStamp, PreviousState);
    if(error_get_previous_state)
    {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::predictInBufferSemiCore() error error_get_previous_state TS: sec="<<ThePredictedTimeStamp.sec<<" s; nsec="<<ThePredictedTimeStamp.nsec<<" ns"<<std::endl;
        this->log(logString.str());
    }
#endif
        return -10;
    }


    if(!PreviousState)
    {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::predictInBufferSemiCore() error !PreviousState TS: sec="<<ThePredictedTimeStamp.sec<<" s; nsec="<<ThePredictedTimeStamp.nsec<<" ns"<<std::endl;
        this->log(logString.str());
    }
#endif
        return -20;
    }


    // Check
    if(!PreviousState->hasState())
    {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::predictInBufferSemiCore() error !PreviousState->hasState() TS: sec="<<ThePredictedTimeStamp.sec<<" s; nsec="<<ThePredictedTimeStamp.nsec<<" ns"<<std::endl;
        this->log(logString.str());
    }
#endif
        return -1;
    }


#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::predictInBufferSemiCore() TS: sec="<<ThePredictedTimeStamp.sec<<" s; nsec="<<ThePredictedTimeStamp.nsec<<" ns"<<std::endl;
        logString<<"MsfLocalizationCore::predictInBufferSemiCore() prev TS: sec="<<PreviousTimeStamp.sec<<" s; nsec="<<PreviousTimeStamp.nsec<<" ns"<<std::endl;
        this->log(logString.str());
    }
#endif


    // Set inputs
    std::shared_ptr<InputCommandComponent> input_commands;

#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::predictInBufferSemiCore() findInputCommands() pre TS: sec="<<ThePredictedTimeStamp.sec<<" s; nsec="<<ThePredictedTimeStamp.nsec<<" ns"<<std::endl;
        this->log(logString.str());
    }
#endif
    int error_find_input_commands = findInputCommands(ThePredictedTimeStamp,
                                                      //PreviousState,
                                                      input_commands);

#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::predictInBufferSemiCore() findInputCommands() post TS: sec="<<ThePredictedTimeStamp.sec<<" s; nsec="<<ThePredictedTimeStamp.nsec<<" ns"<<std::endl;
        this->log(logString.str());
    }
#endif
    if(error_find_input_commands)
    {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::predictInBufferSemiCore() error find_input_commands TS: sec="<<ThePredictedTimeStamp.sec<<" s; nsec="<<ThePredictedTimeStamp.nsec<<" ns"<<std::endl;
        this->log(logString.str());
    }
#endif
        return error_find_input_commands;
    }




    // Predict Core
    int error=predictCore(PreviousTimeStamp, ThePredictedTimeStamp,
                          PreviousState->state_component_,
                          input_commands,
                          ThePredictedState->state_component_);

    if(error)
    {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::predictInBufferSemiCore() error predictCore() "<<error<<" TS: sec="<<ThePredictedTimeStamp.sec<<" s; nsec="<<ThePredictedTimeStamp.nsec<<" ns"<<std::endl;
            this->log(logString.str());
        }
#endif
        return error;
    }


    // End
    return 0;
}
#endif



int MsfLocalizationCore::predictCore(const TimeStamp &previous_time_stamp, const TimeStamp &predicted_time_stamp,
                                     // Previous State
                                     const std::shared_ptr<StateComponent> &previous_state,
                                     // Inputs
                                     const std::shared_ptr<InputCommandComponent> &input_commands,
                                     // Predicted State
                                     std::shared_ptr<StateComponent>& predicted_state)
{

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
    TimeStamp beginTimePredictCore=getTimeStamp();
#endif

    // Checks
    if(!previous_state)
        return -1;
    if(!previous_state->checkState())
        return -1;


    // Predicted State
    if(!predicted_state)
        predicted_state=std::make_shared<StateComponent>();


    /////// State
#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::predictCore() state TS: sec="<<predicted_time_stamp.sec<<" s; nsec="<<predicted_time_stamp.nsec<<" ns"<<std::endl;
        this->log(logString.str());
    }
#endif



    ///// World -> Global Parameters

    {
#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
        TimeStamp beginTimePredictWorld=getTimeStamp();
#endif

        //
        std::shared_ptr<StateCore> component_predicted_state;

        // State
        int error_predict_state=previous_state->TheGlobalParametersStateCore->getMsfElementCoreSharedPtr()->
                predictState(//Time
                             previous_time_stamp,
                             predicted_time_stamp,
                             // Previous State
                             previous_state,
                             // Input
                             input_commands,
                             // Predicted State
                             component_predicted_state);
        if(error_predict_state)
        {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::predictCore() error predict state [World] TS: sec="<<predicted_time_stamp.sec<<" s; nsec="<<predicted_time_stamp.nsec<<" ns"<<std::endl;
                this->log(logString.str());
            }
#endif
            return error_predict_state;
        }


        // Jacobian Error State
        int error_predict_error_state_jacobian=previous_state->TheGlobalParametersStateCore->getMsfElementCoreSharedPtr()->
                predictErrorStateJacobian(//Time
                                         previous_time_stamp,
                                         predicted_time_stamp,
                                         // Previous State
                                         previous_state,
                                          // Input
                                          input_commands,
                                         // Predicted State
                                         component_predicted_state);
        if(error_predict_error_state_jacobian)
        {
            std::cout<<"world: error_predict_error_state_jacobian"<<std::endl;
            return error_predict_error_state_jacobian;
        }

        // Set
        predicted_state->TheGlobalParametersStateCore=component_predicted_state;

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::predictCore() predict [World] state + jac: "<<(getTimeStamp()-beginTimePredictWorld).nsec<<std::endl;
            this->log(logString.str());
        }
#endif
    }


    ///// Robot

    {
#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
        TimeStamp beginTimePredictRobot=getTimeStamp();
#endif

        //
        std::shared_ptr<StateCore> component_predicted_state;

        // State
        int error_predict_state=previous_state->TheRobotStateCore->getMsfElementCoreSharedPtr()->
                predictState(//Time
                             previous_time_stamp,
                             predicted_time_stamp,
                             // Previous State
                             previous_state,
                             // Input
                             input_commands,
                             // Predicted State
                             component_predicted_state);
        if(error_predict_state)
        {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::predictCore() error predict state [Robot] TS: sec="<<predicted_time_stamp.sec<<" s; nsec="<<predicted_time_stamp.nsec<<" ns"<<std::endl;
                this->log(logString.str());
            }
#endif
            return error_predict_state;
        }

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::predictCore() predict [Robot] state: "<<(getTimeStamp()-beginTimePredictRobot).nsec<<std::endl;
            this->log(logString.str());
        }
#endif


        // Jacobian Error State

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
        TimeStamp beginTimePredictRobotJacobian=getTimeStamp();
#endif

        int error_predict_error_state_jacobian=previous_state->TheRobotStateCore->getMsfElementCoreSharedPtr()->
                predictErrorStateJacobian(//Time
                                         previous_time_stamp,
                                         predicted_time_stamp,
                                         // Previous State
                                         previous_state,
                                          // Input
                                          input_commands,
                                         // Predicted State
                                         component_predicted_state);
        if(error_predict_error_state_jacobian)
        {
            std::cout<<"robot: error_predict_error_state_jacobian"<<std::endl;
            return error_predict_error_state_jacobian;
        }

        // Set
        predicted_state->TheRobotStateCore=component_predicted_state;


#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::predictCore() predict [Robot] jacobian: "<<(getTimeStamp()-beginTimePredictRobotJacobian).nsec<<std::endl;
            this->log(logString.str());
        }
#endif

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::predictCore() predict [Robot] state + jac: "<<(getTimeStamp()-beginTimePredictRobot).nsec<<std::endl;
            this->log(logString.str());
        }
#endif
    }


    ///// Inputs

    {
#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
        TimeStamp beginTimePredictInputs=getTimeStamp();
#endif

        // Clean the list
        predicted_state->TheListInputStateCore.clear();

        // Iterate
        for(std::list< std::shared_ptr<StateCore> >::iterator itInput=previous_state->TheListInputStateCore.begin();
            itInput!=previous_state->TheListInputStateCore.end();
            ++itInput)
        {
            //
            std::shared_ptr<StateCore> component_predicted_state;

            // State
            int error_predict_state=(*itInput)->getMsfElementCoreSharedPtr()->
                    predictState(//Time
                                 previous_time_stamp,
                                 predicted_time_stamp,
                                 // Previous State
                                 previous_state,
                                 // Input
                                 input_commands,
                                 // Predicted State
                                 component_predicted_state);
            if(error_predict_state)
            {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
                {
                    std::ostringstream logString;
                    logString<<"MsfLocalizationCore::predictCore() error predict state [Input] TS: sec="<<predicted_time_stamp.sec<<" s; nsec="<<predicted_time_stamp.nsec<<" ns"<<std::endl;
                    this->log(logString.str());
                }
#endif
                return error_predict_state;
            }


            // Jacobian Error State
            int error_predict_error_state_jacobian=(*itInput)->getMsfElementCoreSharedPtr()->
                    predictErrorStateJacobian(//Time
                                             previous_time_stamp,
                                             predicted_time_stamp,
                                             // Previous State
                                             previous_state,
                                             // Input
                                             input_commands,
                                             // Predicted State
                                             component_predicted_state);
            if(error_predict_error_state_jacobian)
            {
                std::cout<<"input: error_predict_error_state_jacobian"<<std::endl;
                return error_predict_error_state_jacobian;
            }

            // Set
            predicted_state->TheListInputStateCore.push_back(component_predicted_state);
        }

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::predictCore() predict [inputs] state + jac: "<<(getTimeStamp()-beginTimePredictInputs).nsec<<std::endl;
            this->log(logString.str());
        }
#endif
    }


    ///// Sensors

    {
#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
        TimeStamp beginTimePredictSensors=getTimeStamp();
#endif

        // Clean the list
        predicted_state->TheListSensorStateCore.clear();


        // Iterate
        for(std::list< std::shared_ptr<StateCore> >::iterator itSensorElement=previous_state->TheListSensorStateCore.begin();
            itSensorElement!=previous_state->TheListSensorStateCore.end();
            ++itSensorElement)
        {
            //
            std::shared_ptr<StateCore> component_predicted_state;

            // State
            int error_predict_state=(*itSensorElement)->getMsfElementCoreSharedPtr()->
                    predictState(//Time
                                 previous_time_stamp,
                                 predicted_time_stamp,
                                 // Previous State
                                 previous_state,
                                 // Input
                                 input_commands,
                                 // Predicted State
                                 component_predicted_state);
            if(error_predict_state)
            {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
                {
                    std::ostringstream logString;
                    logString<<"MsfLocalizationCore::predictCore() error predict state [Sensors] TS: sec="<<predicted_time_stamp.sec<<" s; nsec="<<predicted_time_stamp.nsec<<" ns"<<std::endl;
                    this->log(logString.str());
                }
#endif
                return error_predict_state;
            }


            // Jacobian Error State
            int error_predict_error_state_jacobian=(*itSensorElement)->getMsfElementCoreSharedPtr()->
                    predictErrorStateJacobian(//Time
                                             previous_time_stamp,
                                             predicted_time_stamp,
                                             // Previous State
                                             previous_state,
                                             // Input
                                             input_commands,
                                             // Predicted State
                                             component_predicted_state);
            if(error_predict_error_state_jacobian)
            {
                std::cout<<"sensor: error_predict_error_state_jacobian"<<std::endl;
                return error_predict_error_state_jacobian;
            }

            // Set
            predicted_state->TheListSensorStateCore.push_back(component_predicted_state);
        }


#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::predictCore() predict [Sensors] state + jac: "<<(getTimeStamp()-beginTimePredictSensors).nsec<<std::endl;
            this->log(logString.str());
        }
#endif

    }


    ///// Map

    {

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
        TimeStamp beginTimePredictMaps=getTimeStamp();
#endif

        // Clean the list
        predicted_state->TheListMapElementStateCore.clear();

        // Iterate
        for(std::list< std::shared_ptr<StateCore> >::iterator itMapElement=previous_state->TheListMapElementStateCore.begin();
            itMapElement!=previous_state->TheListMapElementStateCore.end();
            ++itMapElement)
        {
            //
            std::shared_ptr<StateCore> component_predicted_state;

            // State
            int error_predict_state=(*itMapElement)->getMsfElementCoreSharedPtr()->
                    predictState(//Time
                                 previous_time_stamp,
                                 predicted_time_stamp,
                                 // Previous State
                                 previous_state,
                                 // Input
                                 input_commands,
                                 // Predicted State
                                 component_predicted_state);
            if(error_predict_state)
            {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
                {
                    std::ostringstream logString;
                    logString<<"MsfLocalizationCore::predictCore() error predict state [Map Elements] TS: sec="<<predicted_time_stamp.sec<<" s; nsec="<<predicted_time_stamp.nsec<<" ns"<<std::endl;
                    this->log(logString.str());
                }
#endif
                return error_predict_state;
            }


            // Jacobian Error State
            int error_predict_error_state_jacobian=(*itMapElement)->getMsfElementCoreSharedPtr()->
                    predictErrorStateJacobian(//Time
                                             previous_time_stamp,
                                             predicted_time_stamp,
                                             // Previous State
                                             previous_state,
                                             // Input
                                             input_commands,
                                             // Predicted State
                                             component_predicted_state);
            if(error_predict_error_state_jacobian)
            {
                std::cout<<"map: error_predict_error_state_jacobian"<<std::endl;
                return error_predict_error_state_jacobian;
            }

            // Set
            predicted_state->TheListMapElementStateCore.push_back(component_predicted_state);
        }

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::predictCore() predict [map] state + jac: "<<(getTimeStamp()-beginTimePredictMaps).nsec<<std::endl;
            this->log(logString.str());
        }
#endif
    }



#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::predictCore() predict [TOTAL] state + jac: "<<(getTimeStamp()-beginTimePredictCore).nsec<<std::endl;
        this->log(logString.str());
    }
#endif



    /////// Covariances

    {
#if _DEBUG_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::predictCore() covariances TS: sec="<<predicted_time_stamp.sec<<" s; nsec="<<predicted_time_stamp.nsec<<" ns"<<std::endl;
            this->log(logString.str());
        }
#endif



        // Delta Time Stamp
        TimeStamp DeltaTime=predicted_time_stamp-previous_time_stamp;


        // Create and Resize the covariance Matrix
        if(!predicted_state->covarianceMatrix)
            predicted_state->covarianceMatrix=std::make_shared<Eigen::MatrixXd>();



        // Num Error State blocks
        int num_error_states=0;
        {
            // World
            num_error_states++;
            // Robot
            num_error_states++;
            // Inputs
            num_error_states+=predicted_state->getNumberInputStates();
            // Sensors
            num_error_states+=predicted_state->getNumberSensorStates();
            // Map Elements
            num_error_states+=predicted_state->getNumberMapElementStates();
        }

        // Create vectors of state dimensions
        Eigen::VectorXi size_error_state;
        size_error_state.resize(num_error_states);
        // Fill
        {
            int num_error_state_i=0;
            // World
            size_error_state(num_error_state_i)=predicted_state->TheGlobalParametersStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorState();
            num_error_state_i++;
            // Robot
            size_error_state(num_error_state_i)=predicted_state->TheRobotStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorState();
            num_error_state_i++;
            // Inputs
            for(std::list< std::shared_ptr<StateCore> >::iterator itListInputState=predicted_state->TheListInputStateCore.begin();
                itListInputState!=predicted_state->TheListInputStateCore.end();
                ++itListInputState)
            {
                size_error_state(num_error_state_i)=(*itListInputState)->getMsfElementCoreSharedPtr()->getDimensionErrorState();
                num_error_state_i++;
            }
            // Sensors
            for(std::list< std::shared_ptr<StateCore> >::iterator itListSensorState=predicted_state->TheListSensorStateCore.begin();
                itListSensorState!=predicted_state->TheListSensorStateCore.end();
                ++itListSensorState)
            {
                size_error_state(num_error_state_i)=(*itListSensorState)->getMsfElementCoreSharedPtr()->getDimensionErrorState();
                num_error_state_i++;
            }
            // Map
            for(std::list< std::shared_ptr<StateCore> >::iterator itListMapElementState=predicted_state->TheListMapElementStateCore.begin();
                itListMapElementState!=predicted_state->TheListMapElementStateCore.end();
                ++itListMapElementState)
            {
                size_error_state(num_error_state_i)=(*itListMapElementState)->getMsfElementCoreSharedPtr()->getDimensionErrorState();
                num_error_state_i++;
            }
        }


        // Size input commands
        int num_input_commands=input_commands->getNumberInputCommand();



        ///// Jacobians

        /// Jacobian Error State: Fx & Jacobian Error Parameters: Fp


#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
        TimeStamp beginTimePredictedFxFp=getTimeStamp();
#endif

        // Resize and init
        BlockMatrix::MatrixSparse block_jacobian_total_robot_error_state;
        block_jacobian_total_robot_error_state.resize(num_error_states, num_error_states);

        BlockMatrix::MatrixSparse block_jacobian_total_robot_error_parameters;
        block_jacobian_total_robot_error_parameters.resize(num_error_states, num_error_states);

        // Fill
        {
            int jacobian_row=0;
            // World
            {
                int jacobian_column=0;

                // World / World
                block_jacobian_total_robot_error_state(jacobian_row, jacobian_column)=
                        predicted_state->TheGlobalParametersStateCore->getJacobianErrorStateWorld();
                block_jacobian_total_robot_error_parameters(jacobian_row, jacobian_column)=
                        predicted_state->TheGlobalParametersStateCore->getJacobianErrorParametersWorld();
                jacobian_column++;

                // World / Robot
                block_jacobian_total_robot_error_state(jacobian_row, jacobian_column)=
                        predicted_state->TheGlobalParametersStateCore->getJacobianErrorStateRobot();
                block_jacobian_total_robot_error_parameters(jacobian_row, jacobian_column)=
                        predicted_state->TheGlobalParametersStateCore->getJacobianErrorParametersRobot();
                jacobian_column++;

                // World / Inputs
                int size_inputs_i=0;
                for(std::list< std::shared_ptr<StateCore> >::iterator itInputState2=predicted_state->TheListInputStateCore.begin();
                    itInputState2!=predicted_state->TheListInputStateCore.end();
                    ++itInputState2, size_inputs_i++)
                {
                    block_jacobian_total_robot_error_state(jacobian_row, jacobian_column)=
                            predicted_state->TheGlobalParametersStateCore->getJacobianErrorStateInput(size_inputs_i);
                    block_jacobian_total_robot_error_parameters(jacobian_row, jacobian_column)=
                            predicted_state->TheGlobalParametersStateCore->getJacobianErrorParametersInput(size_inputs_i);
                    jacobian_column++;
                }

                // World / Sensors
                int num_sensor_i=0;
                for(std::list< std::shared_ptr<StateCore> >::iterator itSensorState2=predicted_state->TheListSensorStateCore.begin();
                    itSensorState2!=predicted_state->TheListSensorStateCore.end();
                    ++itSensorState2, num_sensor_i++)
                {
                    block_jacobian_total_robot_error_state(jacobian_row, jacobian_column)=
                            predicted_state->TheGlobalParametersStateCore->getJacobianErrorStateSensor(num_sensor_i);
                    block_jacobian_total_robot_error_parameters(jacobian_row, jacobian_column)=
                            predicted_state->TheGlobalParametersStateCore->getJacobianErrorParametersSensor(num_sensor_i);
                    jacobian_column++;
                }

                // World / Map Elements
                int num_map_element_i=0;
                for(std::list< std::shared_ptr<StateCore> >::iterator itMapElementState2=predicted_state->TheListMapElementStateCore.begin();
                    itMapElementState2!=predicted_state->TheListMapElementStateCore.end();
                    ++itMapElementState2, num_map_element_i++)
                {
                    block_jacobian_total_robot_error_state(jacobian_row, jacobian_column)=
                            predicted_state->TheGlobalParametersStateCore->getJacobianErrorStateMapElement(num_map_element_i);
                    block_jacobian_total_robot_error_parameters(jacobian_row, jacobian_column)=
                            predicted_state->TheGlobalParametersStateCore->getJacobianErrorParametersMapElement(num_map_element_i);
                    jacobian_column++;
                }

                jacobian_row++;
            }

            // Robot
            {
                int jacobian_column=0;

                // Robot / World
                block_jacobian_total_robot_error_state(jacobian_row, jacobian_column)=
                        predicted_state->TheRobotStateCore->getJacobianErrorStateWorld();
                block_jacobian_total_robot_error_parameters(jacobian_row, jacobian_column)=
                        predicted_state->TheRobotStateCore->getJacobianErrorParametersWorld();
                jacobian_column++;

                // Robot / Robot
                block_jacobian_total_robot_error_state(jacobian_row, jacobian_column)=
                        predicted_state->TheRobotStateCore->getJacobianErrorStateRobot();
                block_jacobian_total_robot_error_parameters(jacobian_row, jacobian_column)=
                        predicted_state->TheRobotStateCore->getJacobianErrorParametersRobot();
                jacobian_column++;

                // Robot / Inputs
                int size_inputs_i=0;
                for(std::list< std::shared_ptr<StateCore> >::iterator itInputState2=predicted_state->TheListInputStateCore.begin();
                    itInputState2!=predicted_state->TheListInputStateCore.end();
                    ++itInputState2, size_inputs_i++)
                {
                    block_jacobian_total_robot_error_state(jacobian_row, jacobian_column)=
                            predicted_state->TheRobotStateCore->getJacobianErrorStateInput(size_inputs_i);
                    block_jacobian_total_robot_error_parameters(jacobian_row, jacobian_column)=
                            predicted_state->TheRobotStateCore->getJacobianErrorParametersInput(size_inputs_i);
                    jacobian_column++;
                }

                // Robot / Sensors
                int num_sensor_i=0;
                for(std::list< std::shared_ptr<StateCore> >::iterator itSensorState2=predicted_state->TheListSensorStateCore.begin();
                    itSensorState2!=predicted_state->TheListSensorStateCore.end();
                    ++itSensorState2, num_sensor_i++)
                {
                    block_jacobian_total_robot_error_state(jacobian_row, jacobian_column)=
                            predicted_state->TheRobotStateCore->getJacobianErrorStateSensor(num_sensor_i);
                    block_jacobian_total_robot_error_parameters(jacobian_row, jacobian_column)=
                            predicted_state->TheRobotStateCore->getJacobianErrorParametersSensor(num_sensor_i);
                    jacobian_column++;
                }

                // Robot / Map Elements
                int num_map_element_i=0;
                for(std::list< std::shared_ptr<StateCore> >::iterator itMapElementState2=predicted_state->TheListMapElementStateCore.begin();
                    itMapElementState2!=predicted_state->TheListMapElementStateCore.end();
                    ++itMapElementState2, num_map_element_i++)
                {
                    block_jacobian_total_robot_error_state(jacobian_row, jacobian_column)=
                            predicted_state->TheRobotStateCore->getJacobianErrorStateMapElement(num_map_element_i);
                    block_jacobian_total_robot_error_parameters(jacobian_row, jacobian_column)=
                            predicted_state->TheRobotStateCore->getJacobianErrorParametersMapElement(num_map_element_i);
                    jacobian_column++;
                }

                jacobian_row++;
            }

            // Inputs
            for(std::list< std::shared_ptr<StateCore> >::iterator itInputState=predicted_state->TheListInputStateCore.begin();
                itInputState!=predicted_state->TheListInputStateCore.end();
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
                for(std::list< std::shared_ptr<StateCore> >::iterator itInputState2=predicted_state->TheListInputStateCore.begin();
                    itInputState2!=predicted_state->TheListInputStateCore.end();
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
                for(std::list< std::shared_ptr<StateCore> >::iterator itSensorState2=predicted_state->TheListSensorStateCore.begin();
                    itSensorState2!=predicted_state->TheListSensorStateCore.end();
                    ++itSensorState2, num_sensor_i++)
                {
                    block_jacobian_total_robot_error_state(jacobian_row, jacobian_column)=
                            (*itInputState)->getJacobianErrorStateSensor(num_sensor_i);
                    block_jacobian_total_robot_error_parameters(jacobian_row, jacobian_column)=
                            (*itInputState)->getJacobianErrorParametersSensor(num_sensor_i);
                    jacobian_column++;
                }

                // Inputs / Map Elements
                int num_map_element_i=0;
                for(std::list< std::shared_ptr<StateCore> >::iterator itMapElementState2=predicted_state->TheListMapElementStateCore.begin();
                    itMapElementState2!=predicted_state->TheListMapElementStateCore.end();
                    ++itMapElementState2, num_map_element_i++)
                {
                    block_jacobian_total_robot_error_state(jacobian_row, jacobian_column)=
                            (*itInputState)->getJacobianErrorStateMapElement(num_map_element_i);
                    block_jacobian_total_robot_error_parameters(jacobian_row, jacobian_column)=
                            (*itInputState)->getJacobianErrorParametersMapElement(num_map_element_i);
                    jacobian_column++;
                }

                jacobian_row++;
            }

            // Sensors
            for(std::list< std::shared_ptr<StateCore> >::iterator itSensorState=predicted_state->TheListSensorStateCore.begin();
                itSensorState!=predicted_state->TheListSensorStateCore.end();
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
                for(std::list< std::shared_ptr<StateCore> >::iterator itInputState2=predicted_state->TheListInputStateCore.begin();
                    itInputState2!=predicted_state->TheListInputStateCore.end();
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
                for(std::list< std::shared_ptr<StateCore> >::iterator itSensorState2=predicted_state->TheListSensorStateCore.begin();
                    itSensorState2!=predicted_state->TheListSensorStateCore.end();
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
                for(std::list< std::shared_ptr<StateCore> >::iterator itMapElementState2=predicted_state->TheListMapElementStateCore.begin();
                    itMapElementState2!=predicted_state->TheListMapElementStateCore.end();
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

            // Map Elements
            for(std::list< std::shared_ptr<StateCore> >::iterator itMapElementState=predicted_state->TheListMapElementStateCore.begin();
                itMapElementState!=predicted_state->TheListMapElementStateCore.end();
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
                for(std::list< std::shared_ptr<StateCore> >::iterator itInputState2=predicted_state->TheListInputStateCore.begin();
                    itInputState2!=predicted_state->TheListInputStateCore.end();
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
                for(std::list< std::shared_ptr<StateCore> >::iterator itSensorState2=predicted_state->TheListSensorStateCore.begin();
                    itSensorState2!=predicted_state->TheListSensorStateCore.end();
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
                for(std::list< std::shared_ptr<StateCore> >::iterator itMapElementState2=predicted_state->TheListMapElementStateCore.begin();
                    itMapElementState2!=predicted_state->TheListMapElementStateCore.end();
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

        block_jacobian_total_robot_error_state.analyse();
        block_jacobian_total_robot_error_parameters.analyse();

#if _DEBUG_MSF_LOCALIZATION_ALGORITHM_PREDICT
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::predictCore() Fx: block_jacobian_total_robot_error_state for TS: sec="<<predicted_time_stamp.sec<<" s; nsec="<<predicted_time_stamp.nsec<<" ns"<<std::endl;
            logString<<BlockMatrix::convertToEigenDense(block_jacobian_total_robot_error_state)<<std::endl;
            this->log(logString.str());
        }
#endif

#if 0 && _DEBUG_MSF_LOCALIZATION_ALGORITHM_PREDICT
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::predictCore() Fp: block_jacobian_total_robot_error_parameters for TS: sec="<<predicted_time_stamp.sec<<" s; nsec="<<predicted_time_stamp.nsec<<" ns"<<std::endl;
            logString<<BlockMatrix::convertToEigenDense(block_jacobian_total_robot_error_parameters)<<std::endl;
            this->log(logString.str());
        }
#endif

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::predictCore() predict Fx Fp: "<<(getTimeStamp()-beginTimePredictedFxFp).nsec<<std::endl;
            this->log(logString.str());
        }
#endif



        /// Jacobian Error State wrt Error Input Commands: Fu

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
        TimeStamp beginTimePredictedFu=getTimeStamp();
#endif

        BlockMatrix::MatrixSparse block_jacobian_total_robot_error_state_wrt_error_input_commands;
        block_jacobian_total_robot_error_state_wrt_error_input_commands.resize(num_error_states, num_input_commands);

        // Fill
        {
            int jacobian_row=0;

            // World
            {
                int jacobian_column=0;
                int num_input_commands_i=0;
                for(std::list< std::shared_ptr<InputCommandCore> >::iterator itInputCommand=input_commands->list_input_command_core_.begin();
                    itInputCommand!=input_commands->list_input_command_core_.end();
                    ++itInputCommand, num_input_commands_i++)
                {
                    block_jacobian_total_robot_error_state_wrt_error_input_commands(jacobian_row, jacobian_column)=
                            predicted_state->TheGlobalParametersStateCore->getJacobianErrorInputCommands(num_input_commands_i);
                    jacobian_column++;
                }
                jacobian_row++;
            }

            // Robot
            {
                int jacobian_column=0;
                int num_input_commands_i=0;
                for(std::list< std::shared_ptr<InputCommandCore> >::iterator itInputCommand=input_commands->list_input_command_core_.begin();
                    itInputCommand!=input_commands->list_input_command_core_.end();
                    ++itInputCommand, num_input_commands_i++)
                {
                    block_jacobian_total_robot_error_state_wrt_error_input_commands(jacobian_row, jacobian_column)=
                            predicted_state->TheRobotStateCore->getJacobianErrorInputCommands(num_input_commands_i);
                    jacobian_column++;
                }
                jacobian_row++;
            }

            // Inputs
            for(std::list< std::shared_ptr<StateCore> >::iterator itInputState=predicted_state->TheListInputStateCore.begin();
                itInputState!=predicted_state->TheListInputStateCore.end();
                ++itInputState)
            {
                int jacobian_column=0;
                int num_input_commands_i=0;
                for(std::list< std::shared_ptr<InputCommandCore> >::iterator itInputCommand=input_commands->list_input_command_core_.begin();
                    itInputCommand!=input_commands->list_input_command_core_.end();
                    ++itInputCommand, num_input_commands_i++)
                {
                    block_jacobian_total_robot_error_state_wrt_error_input_commands(jacobian_row, jacobian_column)=
                            (*itInputState)->getJacobianErrorInputCommands(num_input_commands_i);
                    jacobian_column++;
                }
                jacobian_row++;
            }

            // Sensors
            for(std::list< std::shared_ptr<StateCore> >::iterator itSensorState=predicted_state->TheListSensorStateCore.begin();
                itSensorState!=predicted_state->TheListSensorStateCore.end();
                ++itSensorState)
            {
                int jacobian_column=0;
                int num_input_commands_i=0;
                for(std::list< std::shared_ptr<InputCommandCore> >::iterator itInputCommand=input_commands->list_input_command_core_.begin();
                    itInputCommand!=input_commands->list_input_command_core_.end();
                    ++itInputCommand, num_input_commands_i++)
                {
                    block_jacobian_total_robot_error_state_wrt_error_input_commands(jacobian_row, jacobian_column)=
                            (*itSensorState)->getJacobianErrorInputCommands(num_input_commands_i);
                    jacobian_column++;
                }
                jacobian_row++;
            }

            // Map Elements
            for(std::list< std::shared_ptr<StateCore> >::iterator itMapElementState=predicted_state->TheListMapElementStateCore.begin();
                itMapElementState!=predicted_state->TheListMapElementStateCore.end();
                ++itMapElementState)
            {
                int jacobian_column=0;
                int num_input_commands_i=0;
                for(std::list< std::shared_ptr<InputCommandCore> >::iterator itInputCommand=input_commands->list_input_command_core_.begin();
                    itInputCommand!=input_commands->list_input_command_core_.end();
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

#if _DEBUG_MSF_LOCALIZATION_ALGORITHM_PREDICT
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::predictCore() Fu: block_jacobian_total_robot_error_state_wrt_error_input_commands for TS: sec="<<predicted_time_stamp.sec<<" s; nsec="<<predicted_time_stamp.nsec<<" ns"<<std::endl;
            logString<<BlockMatrix::convertToEigenDense(block_jacobian_total_robot_error_state_wrt_error_input_commands)<<std::endl;
            this->log(logString.str());
        }
#endif

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::predictCore() predict Fu: "<<(getTimeStamp()-beginTimePredictedFu).nsec<<std::endl;
            this->log(logString.str());
        }
#endif



        /// Jacobian Error State Noise Estimation: Fn


#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
        TimeStamp beginTimePredictedFn=getTimeStamp();
#endif

        BlockMatrix::MatrixSparse block_jacobian_total_robot_error_noise_estimation;
        block_jacobian_total_robot_error_noise_estimation.resize(num_error_states, num_error_states);

        // Fill
        {
            int num_robot_error_states_i=0;

            // World
            block_jacobian_total_robot_error_noise_estimation(num_robot_error_states_i, num_robot_error_states_i)=
                    predicted_state->TheGlobalParametersStateCore->getJacobianErrorStateNoise();
            num_robot_error_states_i++;

            // Robot
            block_jacobian_total_robot_error_noise_estimation(num_robot_error_states_i, num_robot_error_states_i)=
                    predicted_state->TheRobotStateCore->getJacobianErrorStateNoise();
            num_robot_error_states_i++;

            // Inputs
            for(std::list< std::shared_ptr<StateCore> >::iterator itInputState=predicted_state->TheListInputStateCore.begin();
                itInputState!=predicted_state->TheListInputStateCore.end();
                ++itInputState)
            {
                block_jacobian_total_robot_error_noise_estimation(num_robot_error_states_i, num_robot_error_states_i)=
                        (*itInputState)->getJacobianErrorStateNoise();
                num_robot_error_states_i++;
            }

            // Sensors
            for(std::list< std::shared_ptr<StateCore> >::iterator itSensorState=predicted_state->TheListSensorStateCore.begin();
                itSensorState!=predicted_state->TheListSensorStateCore.end();
                ++itSensorState)
            {
                block_jacobian_total_robot_error_noise_estimation(num_robot_error_states_i, num_robot_error_states_i)=
                        (*itSensorState)->getJacobianErrorStateNoise();
                num_robot_error_states_i++;
            }

            // Map Element
            for(std::list< std::shared_ptr<StateCore> >::iterator itMapElementState=predicted_state->TheListMapElementStateCore.begin();
                itMapElementState!=predicted_state->TheListMapElementStateCore.end();
                ++itMapElementState)
            {
                block_jacobian_total_robot_error_noise_estimation(num_robot_error_states_i, num_robot_error_states_i)=
                        (*itMapElementState)->getJacobianErrorStateNoise();
                num_robot_error_states_i++;
            }
        }

        block_jacobian_total_robot_error_noise_estimation.analyse();

#if _DEBUG_MSF_LOCALIZATION_ALGORITHM_PREDICT
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::predictCore() Fn: block_jacobian_total_robot_error_noise_estimation for TS: sec="<<predicted_time_stamp.sec<<" s; nsec="<<predicted_time_stamp.nsec<<" ns"<<std::endl;
            logString<<BlockMatrix::convertToEigenDense(block_jacobian_total_robot_error_noise_estimation)<<std::endl;
            this->log(logString.str());
        }
#endif

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::predictCore() predict Fn: "<<(getTimeStamp()-beginTimePredictedFn).nsec<<std::endl;
            this->log(logString.str());
        }
#endif



        ///// Covariances


        /// Covariance Error State: P


#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
        TimeStamp beginTimePredictedCovarianceErrorStateAsBlock=getTimeStamp();
#endif


        BlockMatrix::MatrixDense block_previous_covariance_error_state;
        block_previous_covariance_error_state.resize(num_error_states, num_error_states);

        block_previous_covariance_error_state.createFromEigen((*previous_state->covarianceMatrix), size_error_state, size_error_state);


#if _DEBUG_MSF_LOCALIZATION_ALGORITHM_PREDICT
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::predictCore() P(k|k): block_previous_covariance_error_state for TS: sec="<<predicted_time_stamp.sec<<" s; nsec="<<predicted_time_stamp.nsec<<" ns"<<std::endl;
        logString<<BlockMatrix::convertToEigenDense(block_previous_covariance_error_state)<<std::endl;
        this->log(logString.str());
    }
#endif


#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::predictCore() predict covariance error state (P) as block: "<<(getTimeStamp()-beginTimePredictedCovarianceErrorStateAsBlock).nsec<<std::endl;
            this->log(logString.str());
        }
#endif



        /// Covariance Error Parameters: Qp


#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
        TimeStamp beginTimePredictedQp=getTimeStamp();
#endif

        BlockMatrix::MatrixSparse block_covariance_total_robot_error_parameters;
        block_covariance_total_robot_error_parameters.resize(num_error_states, num_error_states);

        // Fill
        {
            int num_robot_error_states_i=0;

            // World
            block_covariance_total_robot_error_parameters(num_robot_error_states_i, num_robot_error_states_i)=
                    predicted_state->TheGlobalParametersStateCore->getCovarianceParameters();
            num_robot_error_states_i++;

            // Robot
            block_covariance_total_robot_error_parameters(num_robot_error_states_i, num_robot_error_states_i)=
                    predicted_state->TheRobotStateCore->getCovarianceParameters();
            num_robot_error_states_i++;

            // Inputs
            for(std::list< std::shared_ptr<StateCore> >::iterator itInputState=predicted_state->TheListInputStateCore.begin();
                itInputState!=predicted_state->TheListInputStateCore.end();
                ++itInputState)
            {
                block_covariance_total_robot_error_parameters(num_robot_error_states_i, num_robot_error_states_i)=
                        (*itInputState)->getCovarianceParameters();
                num_robot_error_states_i++;
            }

            // Sensors
            for(std::list< std::shared_ptr<StateCore> >::iterator itSensorState=predicted_state->TheListSensorStateCore.begin();
                itSensorState!=predicted_state->TheListSensorStateCore.end();
                ++itSensorState)
            {
                block_covariance_total_robot_error_parameters(num_robot_error_states_i, num_robot_error_states_i)=
                        (*itSensorState)->getCovarianceParameters();
                num_robot_error_states_i++;
            }

            // Map Elements
            for(std::list< std::shared_ptr<StateCore> >::iterator itMapElementState=predicted_state->TheListMapElementStateCore.begin();
                itMapElementState!=predicted_state->TheListMapElementStateCore.end();
                ++itMapElementState)
            {
                block_covariance_total_robot_error_parameters(num_robot_error_states_i, num_robot_error_states_i)=
                        (*itMapElementState)->getCovarianceParameters();
                num_robot_error_states_i++;
            }
        }

        block_covariance_total_robot_error_parameters.analyse();


#if 0 && _DEBUG_MSF_LOCALIZATION_ALGORITHM_PREDICT
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::predictCore() Qp: block_covariance_total_robot_error_parameters for TS: sec="<<predicted_time_stamp.sec<<" s; nsec="<<predicted_time_stamp.nsec<<" ns"<<std::endl;
        logString<<BlockMatrix::convertToEigenDense(block_covariance_total_robot_error_parameters)<<std::endl;
        this->log(logString.str());
    }
#endif

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::predictCore() predict Qp: "<<(getTimeStamp()-beginTimePredictedQp).nsec<<std::endl;
            this->log(logString.str());
        }
#endif


        /// Covariance Error Inputs: Qu


#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
        TimeStamp beginTimePredictedQu=getTimeStamp();
#endif

        BlockMatrix::MatrixSparse block_covariance_total_robot_error_inputs;
        block_covariance_total_robot_error_inputs.resize(num_input_commands, num_input_commands);

        // Fill
        {
            int num_input_commands_i=0;

            for(std::list< std::shared_ptr<InputCommandCore> >::iterator itInputCommand=input_commands->list_input_command_core_.begin();
                itInputCommand!=input_commands->list_input_command_core_.end();
                ++itInputCommand++)
            {
                block_covariance_total_robot_error_inputs(num_input_commands_i, num_input_commands_i)=
                        (*itInputCommand)->getCovarianceInputs(DeltaTime);
                num_input_commands_i++;
            }

        }

        block_covariance_total_robot_error_inputs.analyse();


#if _DEBUG_MSF_LOCALIZATION_ALGORITHM_PREDICT
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::predictCore() Qu: block_covariance_total_robot_error_inputs for TS: sec="<<predicted_time_stamp.sec<<" s; nsec="<<predicted_time_stamp.nsec<<" ns"<<std::endl;
        logString<<BlockMatrix::convertToEigenDense(block_covariance_total_robot_error_inputs)<<std::endl;
        this->log(logString.str());
    }
#endif

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::predictCore() predict Qu: "<<(getTimeStamp()-beginTimePredictedQu).nsec<<std::endl;
            this->log(logString.str());
        }
#endif



        /// Covariance Error State Noise Estimation: Qn

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
        TimeStamp beginTimePredictedQn=getTimeStamp();
#endif

        BlockMatrix::MatrixSparse block_covariance_total_robot_error_noise_estimation;
        block_covariance_total_robot_error_noise_estimation.resize(num_error_states, num_error_states);

        // Fill
        {
            int num_robot_error_states_i=0;

            // World
            block_covariance_total_robot_error_noise_estimation(num_robot_error_states_i, num_robot_error_states_i)=
                    predicted_state->TheGlobalParametersStateCore->getCovarianceNoise(DeltaTime);
            num_robot_error_states_i++;

            // Robot
            block_covariance_total_robot_error_noise_estimation(num_robot_error_states_i, num_robot_error_states_i)=
                    predicted_state->TheRobotStateCore->getCovarianceNoise(DeltaTime);
            num_robot_error_states_i++;

            // Inputs
            for(std::list< std::shared_ptr<StateCore> >::iterator itInputState=predicted_state->TheListInputStateCore.begin();
                itInputState!=predicted_state->TheListInputStateCore.end();
                ++itInputState)
            {
                block_covariance_total_robot_error_noise_estimation(num_robot_error_states_i, num_robot_error_states_i)=
                        (*itInputState)->getCovarianceNoise(DeltaTime);
                num_robot_error_states_i++;
            }

            // Sensors
            for(std::list< std::shared_ptr<StateCore> >::iterator itSensorState=predicted_state->TheListSensorStateCore.begin();
                itSensorState!=predicted_state->TheListSensorStateCore.end();
                ++itSensorState)
            {
                block_covariance_total_robot_error_noise_estimation(num_robot_error_states_i, num_robot_error_states_i)=
                        (*itSensorState)->getCovarianceNoise(DeltaTime);
                num_robot_error_states_i++;
            }

            // Map Elements
            for(std::list< std::shared_ptr<StateCore> >::iterator itMapElementState=predicted_state->TheListMapElementStateCore.begin();
                itMapElementState!=predicted_state->TheListMapElementStateCore.end();
                ++itMapElementState)
            {
                block_covariance_total_robot_error_noise_estimation(num_robot_error_states_i, num_robot_error_states_i)=
                        (*itMapElementState)->getCovarianceNoise(DeltaTime);
                num_robot_error_states_i++;
            }
        }

        block_covariance_total_robot_error_noise_estimation.analyse();


#if _DEBUG_MSF_LOCALIZATION_ALGORITHM_PREDICT
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::predictCore() Qn: block_covariance_total_robot_error_noise_estimation for TS: sec="<<predicted_time_stamp.sec<<" s; nsec="<<predicted_time_stamp.nsec<<" ns"<<std::endl;
            logString<<BlockMatrix::convertToEigenDense(block_covariance_total_robot_error_noise_estimation)<<std::endl;
            this->log(logString.str());
        }
#endif

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::predictCore() predict Qn: "<<(getTimeStamp()-beginTimePredictedQn).nsec<<std::endl;
            this->log(logString.str());
        }
#endif



        //// Covariance Error State: P(k+1|k)


#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
        TimeStamp beginTimePredictedCovarianceErrorState=getTimeStamp();
#endif



        BlockMatrix::MatrixDense block_predicted_covariance_error_state;
        block_predicted_covariance_error_state.resize(num_error_states, num_error_states);



        try
        {

            BlockMatrix::MatrixSparse block_covariance_total_robot_error;
            block_covariance_total_robot_error.resize(num_error_states, num_error_states);


            block_covariance_total_robot_error= // Fn * Qn * Fn^t
                                                block_jacobian_total_robot_error_noise_estimation*block_covariance_total_robot_error_noise_estimation*block_jacobian_total_robot_error_noise_estimation.transpose() +
                                                // Fp * Qp * Fp^t
                                                block_jacobian_total_robot_error_parameters*block_covariance_total_robot_error_parameters*block_jacobian_total_robot_error_parameters.transpose();




            /*
            std::cout<<"block_jacobian_total_robot_error_noise_estimation"<<std::endl;
            std::cout<<"size_rows="<<block_jacobian_total_robot_error_noise_estimation.getRowsSize().transpose()<<std::endl;
            std::cout<<"size_cols="<<block_jacobian_total_robot_error_noise_estimation.getColsSize().transpose()<<std::endl;

            std::cout<<"block_covariance_total_robot_error_noise_estimation"<<std::endl;
            std::cout<<"size_rows="<<block_covariance_total_robot_error_noise_estimation.getRowsSize().transpose()<<std::endl;
            std::cout<<"size_cols="<<block_covariance_total_robot_error_noise_estimation.getColsSize().transpose()<<std::endl;

            std::cout<<"block_covariance_total_robot_error"<<std::endl;
            std::cout<<"size_rows="<<block_covariance_total_robot_error.getRowsSize().transpose()<<std::endl;
            std::cout<<"size_cols="<<block_covariance_total_robot_error.getColsSize().transpose()<<std::endl;
            */


#if 0 && _DEBUG_MSF_LOCALIZATION_ALGORITHM_PREDICT
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::predictCore() P(k+1|k) I: block_covariance_total_robot_error pre inputs for TS: sec="<<predicted_time_stamp.sec<<" s; nsec="<<predicted_time_stamp.nsec<<" ns"<<std::endl;
                logString<<BlockMatrix::convertToEigenDense(block_covariance_total_robot_error)<<std::endl;
                this->log(logString.str());
            }
#endif


            if(num_input_commands > 0)
            {
                block_covariance_total_robot_error+=// Fu * Qu * Fu^t
                                                    block_jacobian_total_robot_error_state_wrt_error_input_commands*block_covariance_total_robot_error_inputs*block_jacobian_total_robot_error_state_wrt_error_input_commands.transpose();


#if 0 && _DEBUG_MSF_LOCALIZATION_ALGORITHM_PREDICT
                {
                    std::ostringstream logString;
                    logString<<"MsfLocalizationCore::predictCore() P(k+1|k) II: block_covariance_total_robot_error only inputs for TS: sec="<<predicted_time_stamp.sec<<" s; nsec="<<predicted_time_stamp.nsec<<" ns"<<std::endl;
                    logString<<BlockMatrix::convertToEigenDense(block_jacobian_total_robot_error_state_wrt_error_input_commands*block_covariance_total_robot_error_inputs*block_jacobian_total_robot_error_state_wrt_error_input_commands.transpose())<<std::endl;
                    this->log(logString.str());
                }
#endif

            }


            //Eigen::MatrixXd covariance_total_robot_error=BlockMatrix::convertToEigenDense(block_covariance_total_robot_error);


#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::predictCore() predict covariance error state (P) first part (sparse): "<<(getTimeStamp()-beginTimePredictedCovarianceErrorState).nsec<<std::endl;
            this->log(logString.str());
        }
#endif



#if 0 && _DEBUG_MSF_LOCALIZATION_ALGORITHM_PREDICT
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::predictCore() P(k+1|k) III: block_covariance_total_robot_error for TS: sec="<<predicted_time_stamp.sec<<" s; nsec="<<predicted_time_stamp.nsec<<" ns"<<std::endl;
                logString<<BlockMatrix::convertToEigenDense(block_covariance_total_robot_error)<<std::endl;
                this->log(logString.str());
            }
#endif



            // P(k+1|k)

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
            TimeStamp beginTimePredictedCovarianceErrorStateLastSum=getTimeStamp();
#endif

            block_predicted_covariance_error_state= // Fx * P * Fx^t
                                                    block_jacobian_total_robot_error_state*block_previous_covariance_error_state*block_jacobian_total_robot_error_state.transpose()+
                                                    // Fp * Qp * Fp^t + Fu * Qu * Fu^t + Fn * Qn * Fn^t
                                                    block_covariance_total_robot_error;


//#if _DEBUG_MSF_LOCALIZATION_ALGORITHM_PREDICT
//        {
//            std::ostringstream logString;
//            logString<<"MsfLocalizationCore::predictCore() P(k+1|k) pre-predicted covariance for TS: sec="<<predicted_time_stamp.sec<<" s; nsec="<<predicted_time_stamp.nsec<<" ns"<<std::endl;
//            logString<<BlockMatrix::convertToEigenDense(block_jacobian_total_robot_error_state*block_previous_covariance_error_state*block_jacobian_total_robot_error_state.transpose())<<std::endl;
//            this->log(logString.str());
//        }
//#endif


#if 0 && _DEBUG_MSF_LOCALIZATION_ALGORITHM_PREDICT
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::predictCore() P(k+1|k) predicted covariance for TS: sec="<<predicted_time_stamp.sec<<" s; nsec="<<predicted_time_stamp.nsec<<" ns"<<std::endl;
            logString<<BlockMatrix::convertToEigenDense(block_predicted_covariance_error_state)<<std::endl;
            this->log(logString.str());
        }
#endif



#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::predictCore() predict covariance error state (P) second part (dense + last sum): "<<(getTimeStamp()-beginTimePredictedCovarianceErrorStateLastSum).nsec<<std::endl;
                this->log(logString.str());
            }
#endif



        }
        catch(...)
        {
            std::cout<<"Error in prediction covariances robot"<<std::endl;
        }





#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
        TimeStamp beginTimePredictedCovarianceErrorStateAsEigen=getTimeStamp();
#endif

        // Store Covariance Error State as Eigen::MatrixXd
        (*predicted_state->covarianceMatrix)=BlockMatrix::convertToEigenDense(block_predicted_covariance_error_state);

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::predictCore() predict covariance error state (P) un-block: "<<(getTimeStamp()-beginTimePredictedCovarianceErrorStateAsEigen).nsec<<std::endl;
            this->log(logString.str());
        }
#endif



#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::predictCore() predict covariance error state (P) total: "<<(getTimeStamp()-beginTimePredictedCovarianceErrorState).nsec<<std::endl;
            this->log(logString.str());
        }
#endif



    }





    /// Display
#if _DEBUG_MSF_LOCALIZATION_ALGORITHM_PREDICT
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::predictCore() P(k+1|k) predicted covariance for TS: sec="<<predicted_time_stamp.sec<<" s; nsec="<<predicted_time_stamp.nsec<<" ns"<<std::endl;
        logString<<*predicted_state->covarianceMatrix<<std::endl;
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


#if _USE_BUFFER_IN_STATE_ESTIMATION
int MsfLocalizationCore::updateInBuffer(const TimeStamp &TheTimeStamp)
{
#if _DEBUG_MSF_LOCALIZATION_CORE
    {
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::updateInBuffer() TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;

        this->log(logString.str());
    }
#endif


    // Get the reading outdated element to do the update
    std::shared_ptr<StateEstimationCore> OldState;
    int error_get_element=this->TheMsfStorageCore->getElement(TheTimeStamp, OldState);
    if(error_get_element)
    {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::updateInBuffer() error getElement() "<<error_get_element<<std::endl;
            this->log(logString.str());
        }

#endif
        return -13;
    }

    // Check
    if(!OldState)
    {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::updateInBuffer() error -11!"<<std::endl;
            this->log(logString.str());
        }
#endif
        return -11;
    }

    if(!OldState->hasState())
    {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::updateInBuffer() error -12!"<<std::endl;
            this->log(logString.str());
        }
#endif
        return -12;
    }


    // Check if there are measurements
    if(!OldState->hasMeasurement())
    {
#if _DEBUG_MSF_LOCALIZATION_CORE
        std::ostringstream logString;
        logString<<"MsfLocalizationCore::updateInBuffer() ended without measurements TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
        this->log(logString.str());
#endif
        return 0;
    }


    // Checks
    // TODO finish





    // Create new updated state
    std::shared_ptr<StateEstimationCore> UpdatedState=std::make_shared<StateEstimationCore>();


    // Copy all elements in updated state


    // Measurements
    UpdatedState->sensor_measurement_component_=OldState->sensor_measurement_component_;

    // Inputs
    UpdatedState->input_command_component_=OldState->input_command_component_;


    // State
    UpdatedState->state_component_=OldState->state_component_;
    /*
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
    */



    ///// Update Core
    if(this->updateCore(TheTimeStamp, OldState->state_component_, OldState->sensor_measurement_component_, UpdatedState->state_component_))
    {
        std::cout<<"Error updating state"<<std::endl;
        return -10;
    }



    // Release old state
    if(OldState)
        OldState.reset();




    /////// Add element to the buffer -> Check that nobody is using the OldState because is is going to be overwritten

    // More or less sure that there are no more users of updated state
    {
        int error_msf_storage_add_element=TheMsfStorageCore->addElement(TheTimeStamp, UpdatedState);
        if(error_msf_storage_add_element)
        {
#if _DEBUG_ERROR_MSF_LOCALIZATION_CORE
            std::cout<<"!!Error addElement updateInBuffer() "<<error_msf_storage_add_element<<std::endl;
#endif
            return -2;
        }
    }



    // Release update state -> Not really needed
    if(UpdatedState)
        UpdatedState.reset();


#if _DEBUG_MSF_LOCALIZATION_CORE
    std::ostringstream logString;
    logString<<"MsfLocalizationCore::updateInBuffer() ended TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
    this->log(logString.str());
#endif

    // End
    return 0;
}
#endif //_USE_BUFFER_IN_STATE_ESTIMATION


int MsfLocalizationCore::updateCore(const TimeStamp &TheTimeStamp,
                                    const std::shared_ptr<StateComponent> &OldState,
                                    const std::shared_ptr<SensorMeasurementComponent> &sensor_measurement_component,
                                    std::shared_ptr<StateComponent> &UpdatedState)
{

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
            logString<<"MsfLocalizationCore::updateCore() measurement prediction and matching for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
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
        for(std::list<std::shared_ptr<SensorMeasurementCore> >::iterator itListMeas=sensor_measurement_component->list_sensor_measurement_core_.begin();
            itListMeas!=sensor_measurement_component->list_sensor_measurement_core_.end();
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
            logString<<"MsfLocalizationCore::updateCore() measurement prediction time: "<<(getTimeStamp()-beginMeasurementPrediction).nsec<<std::endl;
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



#if _DEBUG_MSF_LOCALIZATION_ALGORITHM_UPDATE
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::updateCore() dimension error state="<<dimensionErrorState<<" for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
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





#if _DEBUG_MSF_LOCALIZATION_ALGORITHM_UPDATE
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::updateCore() Innovation vector for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
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


#if _DEBUG_MSF_LOCALIZATION_ALGORITHM_UPDATE
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::updateCore() Hx: jacobian_error_measurement_wrt_error_state for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
                logString<<Eigen::MatrixXd(jacobian_error_measurement_wrt_error_state)<<std::endl;
                this->log(logString.str());
            }
#endif

#if 0 && _DEBUG_MSF_LOCALIZATION_ALGORITHM_UPDATE
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::updateCore() Hp: jacobian_error_measurement_wrt_error_parameters for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
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


#if _DEBUG_MSF_LOCALIZATION_ALGORITHM_UPDATE
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::updateCore() Hn: jacobian_error_measurement_wrt_error_measurement for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
                logString<<Eigen::MatrixXd(jacobian_error_measurement_wrt_error_measurement)<<std::endl;
                this->log(logString.str());
            }
#endif




#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::updateCore() jacobians time: "<<(getTimeStamp()-beginJacobians).nsec<<std::endl;
                this->log(logString.str());
            }
#endif



            ///// Noises and covariances

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
            TimeStamp beginNoisesCovariances=getTimeStamp();
#endif


            /// Covariance Error State
            /// P(k+1|k)

#if _DEBUG_MSF_LOCALIZATION_ALGORITHM_UPDATE
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::updateCore() P(k+1|k): OldState->covarianceMatrix for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
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
                            OldState->TheGlobalParametersStateCore->getCovarianceParameters();
                    num_error_state_i++;
                }

                // Robot
                {
                    block_covariance_error_parameters(num_error_state_i, num_error_state_i)=
                            OldState->TheRobotStateCore->getCovarianceParameters();
                    num_error_state_i++;
                }

                // Inputs
                for(std::list< std::shared_ptr<StateCore> >::iterator itListInputStateCore=OldState->TheListInputStateCore.begin();
                    itListInputStateCore!=OldState->TheListInputStateCore.end();
                    ++itListInputStateCore)
                {
                    block_covariance_error_parameters(num_error_state_i, num_error_state_i)=
                            (*itListInputStateCore)->getCovarianceParameters();
                    num_error_state_i++;
                }

                // Sensors
                for(std::list<std::shared_ptr<StateCore> >::const_iterator itListSensorStateCore=OldState->TheListSensorStateCore.begin();
                    itListSensorStateCore!=OldState->TheListSensorStateCore.end();
                    ++itListSensorStateCore)
                {
                    block_covariance_error_parameters(num_error_state_i, num_error_state_i)=
                            (*itListSensorStateCore)->getCovarianceParameters();
                    num_error_state_i++;
                }

                // Map
                for(std::list<std::shared_ptr<StateCore> >::const_iterator itListMapElementStateCore=OldState->TheListMapElementStateCore.begin();
                    itListMapElementStateCore!=OldState->TheListMapElementStateCore.end();
                    ++itListMapElementStateCore)
                {
                    block_covariance_error_parameters(num_error_state_i, num_error_state_i)=
                           (*itListMapElementStateCore)->getCovarianceParameters();
                    num_error_state_i++;
                }
            }
            block_covariance_error_parameters.analyse();

            // Sparse matrix
            Eigen::SparseMatrix<double> covariance_error_parameters;
            covariance_error_parameters=BlockMatrix::convertToEigenSparse(block_covariance_error_parameters);


#if 0 && _DEBUG_MSF_LOCALIZATION_ALGORITHM_UPDATE
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::updateCore() Rp: covariance_error_parameters for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
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
                            (*itListPredictedMeas)->getCovarianceMeasurement();
                    num_error_measurements_i++;
                }
            }
            block_covariance_error_measurement.analyse();

            // Sparse matrix
            Eigen::SparseMatrix<double> covariance_error_measurement;
            covariance_error_measurement=BlockMatrix::convertToEigenSparse(block_covariance_error_measurement);



#if _DEBUG_MSF_LOCALIZATION_ALGORITHM_UPDATE
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::updateCore() Rn: CovarianceMeasurement for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
                logString<<Eigen::MatrixXd(covariance_error_measurement)<<std::endl;
                this->log(logString.str());
            }
#endif



#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::updateCore() noises and covariances time: "<<(getTimeStamp()-beginNoisesCovariances).nsec<<std::endl;
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



#if _DEBUG_MSF_LOCALIZATION_ALGORITHM_UPDATE
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::updateCore() S(k): innovationCovariance for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
                logString<<innovationCovariance<<std::endl;
                this->log(logString.str());
            }
#endif

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::updateCore() innovation covariance time: "<<(getTimeStamp()-beginInnovationCovariance).nsec<<std::endl;
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


#if _DEBUG_MSF_LOCALIZATION_ALGORITHM_UPDATE
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::updateCore() distance_mahalanobis ="<<distance_mahalanobis<<" for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
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


#if _DEBUG_MSF_LOCALIZATION_ALGORITHM_UPDATE
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::updateCore() K: Kalman Gain for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
                logString<<kalmanGain<<std::endl;
                this->log(logString.str());
            }
#endif

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::updateCore() kalman gain time: "<<(getTimeStamp()-beginKalmanGain).nsec<<std::endl;
                this->log(logString.str());
            }
#endif



            ///// Updated error state estimate: x(k+1|k+1)

#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
            TimeStamp beginUpdatedState=getTimeStamp();
#endif

            /// Equation: Dx=K*v
            Eigen::VectorXd incrementErrorState=kalmanGain*innovationVector;


#if _DEBUG_MSF_LOCALIZATION_ALGORITHM_UPDATE
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::updateCore() Dx: Increment Delta State vector for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
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
                logString<<"MsfLocalizationCore::updateCore() block_increment_error_state time: "<<(getTimeStamp()-begin_time_block_increment_error_state).nsec<<std::endl;
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
                    logString<<"MsfLocalizationCore::updateCore() update_error_state_from_increment_error_state time: "<<(getTimeStamp()-begin_time_update_error_state_from_increment_error_state).nsec<<std::endl;
                    this->log(logString.str());
                }
#endif
            }



#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::updateCore() updated state time: "<<(getTimeStamp()-beginUpdatedState).nsec<<std::endl;
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


#if 0 && _DEBUG_MSF_LOCALIZATION_ALGORITHM_UPDATE
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::updateCore() P(k+1|k+1) I: updating covariance AuxiliarMatrix for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
                logString<<AuxiliarMatrix<<std::endl;
                this->log(logString.str());
            }
#endif


#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::updateCore() updated covariance interm time: "<<(getTimeStamp()-beginUpdatedCovariance).nsec<<std::endl;
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



#if _DEBUG_MSF_LOCALIZATION_ALGORITHM_UPDATE
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::updateCore() P(k+1|k+1): updated covariance for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
                logString<<*UpdatedState->covarianceMatrix<<std::endl;
                this->log(logString.str());
            }
#endif



#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::updateCore() updated covariance interm 2 time: "<<(getTimeStamp()-beginUpdatedCovariance1).nsec<<std::endl;
                this->log(logString.str());
            }
#endif



#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::updateCore() updated covariance time: "<<(getTimeStamp()-beginUpdatedCovariance).nsec<<std::endl;
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


#if _DEBUG_MSF_LOCALIZATION_ALGORITHM_UPDATE
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::updateCore() P(k+1|k+1): updated covariance post error reset for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
                logString<<*UpdatedState->covarianceMatrix<<std::endl;
                this->log(logString.str());
            }
#endif


#if _DEBUG_TIME_MSF_LOCALIZATION_CORE
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::updateCore() error state reset time: "<<(getTimeStamp()-beginErrorStateReset).nsec<<std::endl;
                this->log(logString.str());
            }
#endif


        }
        else
        {
            // No matched measurements

            // Updated State <- Old State


            // Measurements
            //UpdatedState->sensor_measurement_component_=OldState->sensor_measurement_component_;


            // Inputs
            //UpdatedState->input_command_component_=OldState->input_command_component_;


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
                logString<<"MsfLocalizationCore::updateCore() mapping new element for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
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

            // Num Unmatched Measurement blocks
            int num_unmatched_error_measurements=0;
            num_unmatched_error_measurements=TheListUnmatchedMeasurementsWithMapElement.size();


            // Num New Map Element State blocks
            int num_new_map_elements_error_state=0;
            num_new_map_elements_error_state=TheListNewMapElementsStateCore.size();


            // Dimension Error State
            int dimension_error_state_total=OldState->getDimensionErrorState();


#if _DEBUG_MSF_LOCALIZATION_ALGORITHM_MAPPING
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::updateCore() dimension error state="<<dimension_error_state_total<<" for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
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
                //logString<<"MsfLocalizationCore::updateCore() dimension_new_map_elements_error_state_total="<<dimension_new_map_elements_error_state_total<<" for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
                logString<<"MsfLocalizationCore::updateCore() dimension_new_map_elements_noise_measurement_total="<<dimension_new_map_elements_noise_measurement_total<<" for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
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

#if _DEBUG_MSF_LOCALIZATION_ALGORITHM_MAPPING
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::updateCore() Gx: jacobianMapErrorState for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
                logString<<jacobianMapErrorState<<std::endl;
                this->log(logString.str());
            }
 #endif


            /// Jacobian Noise
            /// Gn

            BlockMatrix::MatrixSparse block_jacobian_mapping_new_error_state_wrt_error_measurement;
            block_jacobian_mapping_new_error_state_wrt_error_measurement.resize(num_new_map_elements_error_state, num_unmatched_error_measurements);


            // Fill
            {
                int num_new_map_elements_error_state_i=0;
                std::list< std::shared_ptr<MapElementStateCore> >::iterator itNewMapElementsState;
                std::list< std::shared_ptr<SensorMeasurementCore> >::iterator itUnmatchedMeasurementsWithMapElement;
                for(itNewMapElementsState=TheListNewMapElementsStateCore.begin(), itUnmatchedMeasurementsWithMapElement=TheListUnmatchedMeasurementsWithMapElement.begin();
                    itNewMapElementsState!=TheListNewMapElementsStateCore.end() || itUnmatchedMeasurementsWithMapElement!=TheListUnmatchedMeasurementsWithMapElement.end();
                    ++itNewMapElementsState, ++itUnmatchedMeasurementsWithMapElement)
                {
                    // Measurement
                    int num_unmatched_error_measurements_i=0;
                    for(std::list< std::shared_ptr<SensorMeasurementCore> >::iterator it2UnmatchedMeasurementsWithMapElement=TheListUnmatchedMeasurementsWithMapElement.begin();
                        it2UnmatchedMeasurementsWithMapElement!=TheListUnmatchedMeasurementsWithMapElement.end();
                        ++it2UnmatchedMeasurementsWithMapElement)
                    {
                        if((*it2UnmatchedMeasurementsWithMapElement) == (*itUnmatchedMeasurementsWithMapElement))
                        {
                            block_jacobian_mapping_new_error_state_wrt_error_measurement(num_new_map_elements_error_state_i, num_unmatched_error_measurements_i)=
                                    (*itNewMapElementsState)->getJacobianMappingErrorStateNoise();
                        }
                        else
                        {
                            // Do nothing -> Set Zeros (already set)
                        }
                        num_unmatched_error_measurements_i++;
                    }
                    num_new_map_elements_error_state_i++;
                }
            }
            block_jacobian_mapping_new_error_state_wrt_error_measurement.analyse();

            // Convert to Eigen::SparseMatrix
            Eigen::SparseMatrix<double> jacobian_mapping_new_error_state_wrt_error_measurement=BlockMatrix::convertToEigenSparse(block_jacobian_mapping_new_error_state_wrt_error_measurement);


#if _DEBUG_MSF_LOCALIZATION_ALGORITHM_MAPPING
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::updateCore() Gn: jacobian_mapping_new_error_state_wrt_error_measurement for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
                logString<<Eigen::MatrixXd(jacobian_mapping_new_error_state_wrt_error_measurement)<<std::endl;
                this->log(logString.str());
            }
 #endif


            /// Covariance Noise
            /// Rnu

            BlockMatrix::MatrixSparse block_covariance_unmatched_measurements;
            block_covariance_unmatched_measurements.resize(num_unmatched_error_measurements, num_unmatched_error_measurements);


            // Fill
            int num_unmatched_error_measurements_i=0;
            for(std::list< std::shared_ptr<SensorMeasurementCore> >::iterator itUnmatchedMeasurementsWithMapElement=TheListUnmatchedMeasurementsWithMapElement.begin();
                itUnmatchedMeasurementsWithMapElement!=TheListUnmatchedMeasurementsWithMapElement.end();
                ++itUnmatchedMeasurementsWithMapElement)
            {
                int num_unmatched_error_measurements_j=0;
                for(std::list< std::shared_ptr<SensorMeasurementCore> >::iterator it2UnmatchedMeasurementsWithMapElement=TheListUnmatchedMeasurementsWithMapElement.begin();
                    it2UnmatchedMeasurementsWithMapElement!=TheListUnmatchedMeasurementsWithMapElement.end();
                    ++it2UnmatchedMeasurementsWithMapElement)
                {
                    if((*it2UnmatchedMeasurementsWithMapElement) == (*itUnmatchedMeasurementsWithMapElement))
                    {
                            block_covariance_unmatched_measurements(num_unmatched_error_measurements_i, num_unmatched_error_measurements_j)=
                                    (*itUnmatchedMeasurementsWithMapElement)->getCovarianceMeasurement();
                    }
                    else
                    {
                        // Do nothing. Set Zeros
                    }
                    num_unmatched_error_measurements_j++;
                }

                // Update dimension
                num_unmatched_error_measurements_i++;
            }
            block_covariance_unmatched_measurements.analyse();

            // Sparse Matrix
            Eigen::SparseMatrix<double> covariance_unmatched_measurements=BlockMatrix::convertToEigenSparse(block_covariance_unmatched_measurements);


#if _DEBUG_MSF_LOCALIZATION_ALGORITHM_MAPPING
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::updateCore() Rn: covariance_unmatched_measurements for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
                logString<<Eigen::MatrixXd(covariance_unmatched_measurements)<<std::endl;
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

#if _DEBUG_MSF_LOCALIZATION_ALGORITHM_MAPPING
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::updateCore() P(k1|k+1): UpdatedState->covarianceMatrix before mapping for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
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
            // Gn Rnu Gn^t
            Eigen::SparseMatrix<double> covariance_updated_aux
                    =jacobian_mapping_new_error_state_wrt_error_measurement*covariance_unmatched_measurements*jacobian_mapping_new_error_state_wrt_error_measurement.transpose();
            // P'(k+1|k+1)
            covarianceUpdated.block(dimension_error_state_total, dimension_error_state_total, dimension_new_map_elements_error_state_total, dimension_new_map_elements_error_state_total)=
                    // Gn Rnu Gn^t
                    Eigen::MatrixXd(covariance_updated_aux)+
                    // Gp Rpu Gp^t
                    // TODO
                    // Gx P(k+1|k+1) Gx^t
                    jacobianMapErrorState*(*UpdatedState->covarianceMatrix)*jacobianMapErrorState.transpose();

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


#if _DEBUG_MSF_LOCALIZATION_ALGORITHM_MAPPING
            {
                std::ostringstream logString;
                logString<<"MsfLocalizationCore::updateCore() P(k+1|k+1): UpdatedState->covarianceMatrix after mapping for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
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
            //OldState->sensor_measurement_component_=UpdatedState->sensor_measurement_component_;

            // Inputs
            //OldState->input_command_component_=UpdatedState->input_command_component_;

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




#if _DEBUG_MSF_LOCALIZATION_ALGORITHM_UPDATE
        {
            std::ostringstream logString;
            logString<<"MsfLocalizationCore::updateCore() P(k+1|k+1): UpdatedState->covarianceMatrix for TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
            logString<<*UpdatedState->covarianceMatrix<<std::endl;
            this->log(logString.str());
        }
 #endif




    // End
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
