
#include "msf_localization_core/msf_storage_core.h"


// Required for display!
#include "msf_localization_core/robot_core.h"
#include "msf_localization_core/robot_state_core.h"
#include "msf_localization_core/imu_sensor_core.h"
#include "msf_localization_core/imu_sensor_measurement_core.h"
#include "msf_localization_core/imu_input_core.h"
#include "msf_localization_core/imu_input_command_core.h"


//#define _DEBUG_DISPLAY


MsfStorageCore::MsfStorageCore()
{
    // Mutex
    outdatedBufferElementsLock=new std::unique_lock<std::mutex>(outdatedBufferElementsMutex);

    updated_buffer_lock_=new std::unique_lock<std::mutex>(updated_buffer_mutex_);


    // Flags
    flag_new_measurement_=false;


    // Log
    const char* env_p = std::getenv("FUSEON_STACK");

    logPath=std::string(env_p)+"/logs/"+"logMsfStorageCoreFile.txt";

    //std::cout<<"log file path="<<logPath<<std::endl;

    logFile.open(logPath);

    if(!logFile.is_open())
    {
        std::cout<<"unable to open log file"<<std::endl;
    }

    return;
}


MsfStorageCore::~MsfStorageCore()
{
    // Log file
    if(logFile.is_open())
    {
        logFile.close();
    }


    // Delete
    delete outdatedBufferElementsLock;
    delete updated_buffer_lock_;

    return;
}

int MsfStorageCore::setMeasurement(const TimeStamp &TheTimeStamp, const std::shared_ptr<SensorMeasurementCore> &TheSensorMeasurement)
{
#if _DEBUG_MSF_STORAGE
    {
        std::ostringstream logString;
        logString<<"MsfStorageCore::setMeasurement() TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
        this->log(logString.str());
    }
#endif

    std::shared_ptr<StateEstimationCore> TheStateEstimationCore;


    // Get the oldest time stamp in the buffer
    TimeStamp oldest_time_stamp;

    if(getOldestTimeStamp(oldest_time_stamp))
        return -1;

    // If the new element is older than the oldest time stamp in buffer, we discard the element
    if(TheTimeStamp<oldest_time_stamp)
        return 1;


    // This is already safe
    this->getElement(TheTimeStamp, TheStateEstimationCore);

    if(!TheStateEstimationCore)
    {
        TheStateEstimationCore=std::make_shared<StateEstimationCore>();
    }


    // Measure
    if(!TheStateEstimationCore->sensor_measurement_component_)
        TheStateEstimationCore->sensor_measurement_component_=std::make_shared<SensorMeasurementComponent>();
    TheStateEstimationCore->sensor_measurement_component_->list_sensor_measurement_core_.push_back(TheSensorMeasurement);
    //TheMeasurementToTheBuffer.object.flagHasMeasurement=true;



    // Add measurement to the MSF Storage Core
    // TODO, protect to avoid races!
    // TODO fix!
    //TheRingBufferMutex.lock();
    //this->addElementByStamp(TheMeasurementToTheBuffer);

    // This is already safe
    this->addElement(TheTimeStamp, TheStateEstimationCore);

    //TheRingBufferMutex.unlock();


    // Add TimeStamp to the outdated elements list
    this->addOutdatedElement(TheTimeStamp);

#if _DEBUG_MSF_STORAGE
    {
        std::ostringstream logString;
        logString<<"MsfStorageCore::setMeasurement () ended TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
        this->log(logString.str());
    }
#endif


    // Set flag
    flag_new_measurement_=true;

    // End
    return 0;
}

int MsfStorageCore::setMeasurementList(const TimeStamp& TheTimeStamp, const std::list<std::shared_ptr<SensorMeasurementCore>>& TheListSensorMeasurement)
{
#if _DEBUG_MSF_STORAGE
    {
        std::ostringstream logString;
        logString<<"MsfStorageCore::setMeasurementList() TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
        this->log(logString.str());
    }
#endif

    std::shared_ptr<StateEstimationCore> TheStateEstimationCore;


    // Get the oldest time stamp in the buffer
    TimeStamp oldest_time_stamp;

    if(getOldestTimeStamp(oldest_time_stamp))
        return -1;

    // If the new element is older than the oldest time stamp in buffer, we discard the element
    if(TheTimeStamp<oldest_time_stamp)
        return 0;


    // This is already safe
    this->getElement(TheTimeStamp, TheStateEstimationCore);

    if(!TheStateEstimationCore)
    {
        TheStateEstimationCore=std::make_shared<StateEstimationCore>();
    }


    // Measurements
    if(!TheStateEstimationCore->sensor_measurement_component_)
        TheStateEstimationCore->sensor_measurement_component_=std::make_shared<SensorMeasurementComponent>();

    for(std::list<std::shared_ptr<SensorMeasurementCore>>::const_iterator itSensorMeasurements=TheListSensorMeasurement.begin();
        itSensorMeasurements!=TheListSensorMeasurement.end();
        ++itSensorMeasurements)
    {
        TheStateEstimationCore->sensor_measurement_component_->list_sensor_measurement_core_.push_back((*itSensorMeasurements));
    }

    // This is already safe
    this->addElement(TheTimeStamp, TheStateEstimationCore);


    // Add TimeStamp to the outdated elements list
    this->addOutdatedElement(TheTimeStamp);

#if _DEBUG_MSF_STORAGE
    {
        std::ostringstream logString;
        logString<<"MsfStorageCore::setMeasurement () ended TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
        this->log(logString.str());
    }
#endif

    // Set flag
    flag_new_measurement_=true;

    // End
    return 0;
}

int MsfStorageCore::setInputCommand(const TimeStamp &time_stamp, const std::shared_ptr<InputCommandCore> &input_command_core)
{
#if _DEBUG_MSF_STORAGE
    {
        std::ostringstream logString;
        logString<<"MsfStorageCore::setInputCommand() TS: sec="<<time_stamp.sec<<" s; nsec="<<time_stamp.nsec<<" ns"<<std::endl;
        this->log(logString.str());
    }
#endif

    std::shared_ptr<StateEstimationCore> TheStateEstimationCore;


    // Get the oldest time stamp in the buffer
    TimeStamp oldest_time_stamp;

    if(getOldestTimeStamp(oldest_time_stamp))
        return -1;

    // If the new element is older than the oldest time stamp in buffer, we discard the element
    if(time_stamp<oldest_time_stamp)
        return 0;


    // This is already safe
    this->getElement(time_stamp, TheStateEstimationCore);

    if(!TheStateEstimationCore)
    {
        TheStateEstimationCore=std::make_shared<StateEstimationCore>();
    }

    // Check
    if(!TheStateEstimationCore->input_command_component_)
    {
        TheStateEstimationCore->input_command_component_=std::make_shared<InputCommandComponent>();
    }

    // Avoid duplicates
    bool flag_input_command_found=false;


    for(std::list< std::shared_ptr<InputCommandCore> >::iterator itComm=TheStateEstimationCore->input_command_component_->list_input_command_core_.begin();
        itComm!=TheStateEstimationCore->input_command_component_->list_input_command_core_.end();
        ++itComm)
    {
        if((*itComm)->getInputCoreSharedPtr()==input_command_core->getInputCoreSharedPtr())
        {
            flag_input_command_found=true;
            // Found, replace
            *itComm=input_command_core;
            break;
        }

    }

    // Input
    if(!flag_input_command_found)
    {
        // Not found, add new one
        TheStateEstimationCore->input_command_component_->list_input_command_core_.push_back(input_command_core);
    }


    // This is already safe
    this->addElement(time_stamp, TheStateEstimationCore);

    // Add TimeStamp to the outdated elements list
    this->addOutdatedElement(time_stamp);

#if _DEBUG_MSF_STORAGE
    {
        std::ostringstream logString;
        logString<<"MsfStorageCore::setInputCommand() ended TS: sec="<<time_stamp.sec<<" s; nsec="<<time_stamp.nsec<<" ns"<<std::endl;
        this->log(logString.str());
    }
#endif


    return 0;
}

int MsfStorageCore::setInputCommandList(const TimeStamp& time_stamp, const std::list<std::shared_ptr<InputCommandCore> > &list_input_command_core)
{
#if _DEBUG_MSF_STORAGE
    {
        std::ostringstream logString;
        logString<<"MsfStorageCore::setInputCommandList() TS: sec="<<time_stamp.sec<<" s; nsec="<<time_stamp.nsec<<" ns"<<std::endl;
        this->log(logString.str());
    }
#endif

    std::shared_ptr<StateEstimationCore> TheStateEstimationCore;


    // Get the oldest time stamp in the buffer
    TimeStamp oldest_time_stamp;

    if(getOldestTimeStamp(oldest_time_stamp))
        return -1;

    // If the new element is older than the oldest time stamp in buffer, we discard the element
    if(time_stamp<oldest_time_stamp)
        return 0;


    // This is already safe
    this->getElement(time_stamp, TheStateEstimationCore);

    if(!TheStateEstimationCore)
    {
        TheStateEstimationCore=std::make_shared<StateEstimationCore>();
    }


    // Check
    if(!TheStateEstimationCore->input_command_component_)
    {
        TheStateEstimationCore->input_command_component_=std::make_shared<InputCommandComponent>();
    }

    // Inputs
    for(std::list<std::shared_ptr<InputCommandCore>>::const_iterator itInputCommand=list_input_command_core.begin();
        itInputCommand!=list_input_command_core.end();
        ++itInputCommand)
    {

        // Avoid duplicates
        bool flag_input_command_found=false;
        for(std::list< std::shared_ptr<InputCommandCore> >::iterator itComm=TheStateEstimationCore->input_command_component_->list_input_command_core_.begin();
            itComm!=TheStateEstimationCore->input_command_component_->list_input_command_core_.end();
            ++itComm)
        {
            if((*itComm)->getInputCoreSharedPtr()==(*itInputCommand)->getInputCoreSharedPtr())
            {
                flag_input_command_found=true;
                // Found, replace
                *itComm=(*itInputCommand);
                break;
            }

        }

        // Input
        if(!flag_input_command_found)
        {
            // Not found, add new one
            TheStateEstimationCore->input_command_component_->list_input_command_core_.push_back((*itInputCommand));
        }

    }

    // This is already safe
    this->addElement(time_stamp, TheStateEstimationCore);


    // Add TimeStamp to the outdated elements list
    this->addOutdatedElement(time_stamp);

#if _DEBUG_MSF_STORAGE
    {
        std::ostringstream logString;
        logString<<"MsfStorageCore::setInputCommandList() ended TS: sec="<<time_stamp.sec<<" s; nsec="<<time_stamp.nsec<<" ns"<<std::endl;
        this->log(logString.str());
    }
#endif


    return 0;
}

int MsfStorageCore::getLastElementWithStateEstimate(TimeStamp& TheTimeStamp, std::shared_ptr<StateEstimationCore>& PreviousState)
{
    StampedBufferObjectType< std::shared_ptr<StateEstimationCore> > BufferElement;

    TheRingBufferMutex.lock();
    for(std::list< StampedBufferObjectType< std::shared_ptr<StateEstimationCore> > >::iterator itElement=this->getBegin();
        itElement!=this->getEnd();
        ++itElement)
    {
        int errorGetElement=this->getElementI(BufferElement, itElement);
        if(errorGetElement)
            continue;
        if(BufferElement.object->hasState())
        {
            TheTimeStamp=BufferElement.timeStamp;
            PreviousState=BufferElement.object;
            //std::cout<<"found!"<<std::endl;
            break;
        }
    }
    TheRingBufferMutex.unlock();


    return 0;
}

int MsfStorageCore::getElementWithStateEstimateByStamp(const TimeStamp& ThePreviousTimeStamp, TimeStamp& TheTimeStamp, std::shared_ptr<StateEstimationCore>& PreviousState)
{
#if _DEBUG_MSF_STORAGE
    {
        std::ostringstream logString;
        logString<<"MsfStorageCore::getElementWithStateEstimateByStamp() TS: sec="<<ThePreviousTimeStamp.sec<<"; nsec="<<ThePreviousTimeStamp.nsec<<std::endl;
        this->log(logString.str());
    }
#endif

    // Reset time stamp
    PreviousState.reset();

    StampedBufferObjectType< std::shared_ptr<StateEstimationCore> > BufferElement;

    // Find
    TheRingBufferMutex.lock();
    for(std::list< StampedBufferObjectType< std::shared_ptr<StateEstimationCore> > >::iterator itElement=this->getBegin();
        itElement!=this->getEnd();
        ++itElement)
    {
        // Get the element
        int errorGetElement=this->getElementI(BufferElement, itElement);

        if(errorGetElement)
            continue;

#if _DEBUG_MSF_STORAGE
        {
            std::ostringstream logString;
            logString<<"MsfStorageCore::getElementWithStateEstimateByStamp() element is going to be checked TS: sec="<<BufferElement.timeStamp.sec<<" s; nsec="<<BufferElement.timeStamp.nsec<<" ns"<<std::endl;
            this->log(logString.str());
        }
#endif


        // Check the time stamp
        if(BufferElement.timeStamp>ThePreviousTimeStamp)
        {
            continue;
        }
        else
        {
            //logFile<<"found by time stamp!"<<std::endl;

            // Check if it has state
            if(BufferElement.object->hasState())
            {
#if _DEBUG_MSF_STORAGE
                {
                    std::ostringstream logString;
                    logString<<"MsfStorageCore::getElementWithStateEstimateByStamp() found pre assign TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
                    this->log(logString.str());
                }
#endif

                TheTimeStamp=BufferElement.timeStamp;
                PreviousState=BufferElement.object;
                //logFile<<"found with state!"<<std::endl;

#if _DEBUG_MSF_STORAGE
                {
                    std::ostringstream logString;
                    logString<<"MsfStorageCore::getElementWithStateEstimateByStamp() found post assign TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
                    this->log(logString.str());
                }
#endif

                break;
            }
        }
    }
    TheRingBufferMutex.unlock();

#if _DEBUG_MSF_STORAGE
    {
        std::ostringstream logString;
        logString<<"MsfStorageCore::getElementWithStateEstimateByStamp() ended"<<std::endl;
        this->log(logString.str());
    }
#endif

    if(!PreviousState)
        return -1;

    return 0;

}


int MsfStorageCore::getPreviousElementWithStateEstimateByStamp(const TimeStamp& ThePreviousTimeStamp, TimeStamp& TheTimeStamp, std::shared_ptr<StateEstimationCore>& PreviousState)
{
#if _DEBUG_MSF_STORAGE
    {
        std::ostringstream logString;
        logString<<"MsfStorageCore::getPreviousElementWithStateEstimateByStamp() TS: sec="<<ThePreviousTimeStamp.sec<<"; nsec="<<ThePreviousTimeStamp.nsec<<std::endl;
        this->log(logString.str());
    }
#endif

    // Reset time stamp
    PreviousState.reset();;

    StampedBufferObjectType< std::shared_ptr<StateEstimationCore> > BufferElement;

    // Find
    TheRingBufferMutex.lock();
    for(std::list< StampedBufferObjectType< std::shared_ptr<StateEstimationCore> > >::iterator itElement=this->getBegin();
        itElement!=this->getEnd();
        ++itElement)
    {
        // Get the element
        int errorGetElement=this->getElementI(BufferElement, itElement);

        if(errorGetElement)
            continue;

#if _DEBUG_MSF_STORAGE
        {
            std::ostringstream logString;
            logString<<"MsfStorageCore::getPreviousElementWithStateEstimateByStamp() element is going to be checked TS: sec="<<BufferElement.timeStamp.sec<<" s; nsec="<<BufferElement.timeStamp.nsec<<" ns"<<std::endl;
            this->log(logString.str());
        }
#endif


        // Check the time stamp
        if(BufferElement.timeStamp>=ThePreviousTimeStamp)
        {
            continue;
        }
        else
        {
            //logFile<<"found by time stamp!"<<std::endl;

            // Check if it has state
            if(BufferElement.object->hasState())
            {
#if _DEBUG_MSF_STORAGE
                {
                    std::ostringstream logString;
                    logString<<"MsfStorageCore::getPreviousElementWithStateEstimateByStamp() found pre assign TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
                    this->log(logString.str());
                }
#endif

                TheTimeStamp=BufferElement.timeStamp;
                PreviousState=BufferElement.object;
                //logFile<<"found with state!"<<std::endl;

#if _DEBUG_MSF_STORAGE
                {
                    std::ostringstream logString;
                    logString<<"MsfStorageCore::getPreviousElementWithStateEstimateByStamp() found post assign TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;
                    this->log(logString.str());
                }
#endif

                break;
            }
        }
    }
    TheRingBufferMutex.unlock();

#if _DEBUG_MSF_STORAGE
    {
        std::ostringstream logString;
        logString<<"MsfStorageCore::getPreviousElementWithStateEstimateByStamp() ended"<<std::endl;
        this->log(logString.str());
    }
#endif

    if(!PreviousState)
        return -1;

    return 0;
}




int MsfStorageCore::getElement(const TimeStamp &timeStamp, std::shared_ptr<StateEstimationCore> &TheElement)
{
    //StateEstimationCore BufferElement;

    TheRingBufferMutex.lock();
    int error=this->getElementByStamp(timeStamp, TheElement);
    TheRingBufferMutex.unlock();

    //TheElement=BufferElement.object;

    return error;
}

int MsfStorageCore::getNextTimeStamp(const TimeStamp& currentTimeStamp, TimeStamp& nextTimeStamp)
{

    //std::cout<<"MsfStorageCore::getNextTimeStamp()"<<std::endl;

    //StampedBufferObjectType< std::shared_ptr<StateEstimationCore> > BufferElement;
    TimeStamp bufferElementTimeStamp;

    nextTimeStamp=currentTimeStamp;

    // Find
    TheRingBufferMutex.lock();

//std::cout<<"TS: ";

    // Loop
    for(std::list< StampedBufferObjectType< std::shared_ptr<StateEstimationCore> > >::iterator itElement=this->getBegin();
        itElement!=this->getEnd();
        ++itElement)
    {
        // Get the element
        int errorGetElement=this->getElementITimeStamp(bufferElementTimeStamp, itElement);

        if(errorGetElement)
            continue;

        // Check the time stamp
        if(bufferElementTimeStamp>currentTimeStamp)
        {
            nextTimeStamp=bufferElementTimeStamp;

            //std::cout<<"s:"<<BufferElement.timeStamp.sec<<",ns:"<<BufferElement.timeStamp.nsec<<"; ";

            continue;
        }
        else
        {
            //std::cout<<"s:"<<BufferElement.timeStamp.sec<<",ns:"<<BufferElement.timeStamp.nsec<<"; ";
            break;
        }
    }


    //std::cout<<std::endl;


    TheRingBufferMutex.unlock();

    //logFile<<"MsfStorageCore::getNextTimeStamp() ended"<<std::endl;

    // Check if success
    if(nextTimeStamp<=currentTimeStamp)
        return 1;

    return 0;
}

int MsfStorageCore::getPreviousTimeStamp(const TimeStamp &currentTimeStamp, TimeStamp& previousTimeStamp)
{
    //StampedBufferObjectType< std::shared_ptr<StateEstimationCore> > BufferElement;
    TimeStamp bufferElementTimeStamp;

    previousTimeStamp=currentTimeStamp;

    // Find
    TheRingBufferMutex.lock();

//std::cout<<"TS: ";

    // Loop
    for(std::list< StampedBufferObjectType< std::shared_ptr<StateEstimationCore> > >::iterator itElement=this->getBegin();
        itElement!=this->getEnd();
        ++itElement)
    {
        // Get the element
        int errorGetElement=this->getElementITimeStamp(bufferElementTimeStamp, itElement);

        if(errorGetElement)
            continue;

        // Check the time stamp
        if(bufferElementTimeStamp<currentTimeStamp)
        {
            previousTimeStamp=bufferElementTimeStamp;

            //std::cout<<"s:"<<BufferElement.timeStamp.sec<<",ns:"<<BufferElement.timeStamp.nsec<<"; ";

            break;
        }
        else
        {
            //std::cout<<"s:"<<BufferElement.timeStamp.sec<<",ns:"<<BufferElement.timeStamp.nsec<<"; ";
            continue;
        }
    }


    //std::cout<<std::endl;


    TheRingBufferMutex.unlock();


    // Check if success
    if(previousTimeStamp>=currentTimeStamp)
        return -1;


    return 0;
}

int MsfStorageCore::getPreviousInputCommandByStampAndInputCore(const TimeStamp& time_stamp, const std::shared_ptr<InputCore>& input_core, std::shared_ptr<InputCommandCore>& input_command_core)
{
#if _DEBUG_MSF_STORAGE
        {
            std::ostringstream logString;
            logString<<"MsfStorageCore::getPreviousInputCommandByStampAndInputCore() finding TS: sec="<<time_stamp.sec<<" s; nsec="<<time_stamp.nsec<<" ns"<<std::endl;
            this->log(logString.str());

            this->displayRingBuffer();
        }
#endif

    StampedBufferObjectType< std::shared_ptr<StateEstimationCore> > BufferElement;

    // Find
    TheRingBufferMutex.lock();
    for(std::list< StampedBufferObjectType< std::shared_ptr<StateEstimationCore> > >::iterator itElement=this->getBegin();
        itElement!=this->getEnd();
        ++itElement)
    {
        // Get the element
        int errorGetElement=this->getElementI(BufferElement, itElement);

        if(errorGetElement)
            continue;

#if _DEBUG_MSF_STORAGE
        {
            std::ostringstream logString;
            logString<<"MsfStorageCore::getPreviousInputCommandByStampAndInputCore() element is going to be checked TS: sec="<<BufferElement.timeStamp.sec<<" s; nsec="<<BufferElement.timeStamp.nsec<<" ns"<<std::endl;
            this->log(logString.str());
        }
#endif


        // Check the time stamp
        if(BufferElement.timeStamp>time_stamp)
        {
            continue;
        }
        else
        {
            //logFile<<"found by time stamp!"<<std::endl;

            // Check if it has input commands
            if(BufferElement.object->hasInputCommand())
            {

                //TheTimeStamp=BufferElement.timeStamp;
                //PreviousState=BufferElement.object;


                // Check
                bool flag_input_command_found=false;
                for(std::list<std::shared_ptr<InputCommandCore>>::iterator itInputCommand=BufferElement.object->input_command_component_->list_input_command_core_.begin();
                    itInputCommand!=BufferElement.object->input_command_component_->list_input_command_core_.end();
                    ++itInputCommand)
                {
                    if((*itInputCommand)->getInputCoreSharedPtr() == input_core)
                    {
#if _DEBUG_MSF_STORAGE
                        {
                            std::ostringstream logString;
                            logString<<"MsfStorageCore::getPreviousInputCommandByStampAndInputCore() found pre assign TS: sec="<<time_stamp.sec<<" s; nsec="<<time_stamp.nsec<<" ns"<<std::endl;
                            this->log(logString.str());
                        }
#endif

                        flag_input_command_found=true;
                        input_command_core=(*itInputCommand);
                        break;
                    }
                }

                // Check and break
                if(flag_input_command_found)
                {

#if _DEBUG_MSF_STORAGE
                    {
                        std::ostringstream logString;
                        logString<<"MsfStorageCore::getPreviousInputCommandByStampAndInputCore() found post assign TS: sec="<<time_stamp.sec<<" s; nsec="<<time_stamp.nsec<<" ns"<<std::endl;
                        logString<<"MsfStorageCore::getPreviousInputCommandByStampAndInputCore() the assigned TS is: sec="<<BufferElement.timeStamp.sec<<" s; nsec="<<BufferElement.timeStamp.nsec<<" ns"<<std::endl;
                        this->log(logString.str());
                        this->displayStateEstimationElement(BufferElement.timeStamp, BufferElement.object);
                    }
#endif

                    break;
                }
            }
        }
    }
    TheRingBufferMutex.unlock();

#if _DEBUG_MSF_STORAGE
    {
        std::ostringstream logString;
        logString<<"MsfStorageCore::getPreviousInputCommandByStampAndInputCore() ended"<<std::endl;
        this->log(logString.str());
    }
#endif

    if(!input_command_core)
        return -1;

    return 0;
}

int MsfStorageCore::getOldestTimeStamp(TimeStamp& oldest_time_stamp)
{
    // Lock
    TheRingBufferMutex.lock();

    getOldestTimeStampInBuffer(oldest_time_stamp);

    // Unlock
    TheRingBufferMutex.unlock();

    // End
    return 0;
}

int MsfStorageCore::addElement(const TimeStamp &TheTimeStamp, const std::shared_ptr<StateEstimationCore>& TheStateEstimationCore)
{
#if _DEBUG_MSF_STORAGE
    {
        std::ostringstream logString;
        logString<<"MsfStorageCore::addElement()"<<std::endl;
        logString<<"Adding element to the buffer:"<<std::endl;
        this->log(logString.str());
    }
#endif

#if _DEBUG_MSF_STORAGE
    this->displayStateEstimationElement(TheTimeStamp, TheStateEstimationCore);
#endif


    // Check that nobody else is using the element in the buffer if it is going to be overwritten
    std::shared_ptr<StateEstimationCore> OldStateEstimationElement;


    this->getElement(TheTimeStamp, OldStateEstimationElement);


    while(OldStateEstimationElement.use_count()>2)
    {
        // Do nothing, sleep for a little
        std::this_thread::sleep_for( std::chrono::nanoseconds( 50 ) );
    }


    // Lock the buffer. OJO!! Posible fuente de errores. no 100% sincronizado!
    TheRingBufferMutex.lock();



    // New buffer element
    StampedBufferObjectType< std::shared_ptr<StateEstimationCore> > BufferElement;

    BufferElement.timeStamp=TheTimeStamp;
    BufferElement.object=TheStateEstimationCore;


    // Error flag
    int errorType;

#if _DEBUG_MSF_STORAGE
    {
        std::ostringstream logString;
        logString<<"MsfStorageCore::addElement() pre addElementByStamp"<<std::endl;
        this->log(logString.str());
    }
#endif



    // Add
    errorType=this->addElementByStamp(BufferElement);

    // Unlock
    TheRingBufferMutex.unlock();

#if _DEBUG_MSF_STORAGE
    {
        std::ostringstream logString;
        logString<<"MsfStorageCore::addElement() post addElementByStamp"<<std::endl;
        this->log(logString.str());
    }
#endif

    if(errorType)
    {

#if _DEBUG_MSF_STORAGE
        {
            std::ostringstream logString;
            logString<<"MsfStorageCore::addElement() error in addElementByStamp number: "<<errorType<<std::endl;
            this->log(logString.str());
        }
#endif

        return errorType;
    }


#if _DEBUG_MSF_STORAGE
    {
        std::ostringstream logString;
        logString<<"MsfStorageCore::addElement() ended"<<std::endl;
        this->log(logString.str());
    }
#endif

    return 0;
}


int MsfStorageCore::displayStateEstimationElement(const TimeStamp& TheTimeStamp, const std::shared_ptr<StateEstimationCore> &TheStateEstimationCore)
{

    std::ostringstream logString;
    logString<<"-TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;



    // Check
    if(!TheStateEstimationCore)
    {
        logString<<"error in MsfStorageCore::displayStateEstimationElement()"<<std::endl;
        return -1;
    }


    /////// State
    if(TheStateEstimationCore->hasState())
    {
        logString<<"\t";
        logString<<"+State:"<<std::endl;


        //// Robot
        // TODO
        logString<<"\t\t";
        logString<<"Robot ";

        // Robot State
        std::shared_ptr<RobotStateCore> current_robot_state=std::dynamic_pointer_cast<RobotStateCore>(TheStateEstimationCore->state_component_->TheRobotStateCore);

        switch(std::dynamic_pointer_cast<RobotCore>(TheStateEstimationCore->state_component_->TheRobotStateCore->getMsfElementCoreSharedPtr())->getRobotCoreType())
        {
            case RobotCoreTypes::undefined:
            {
                break;
            }

            case RobotCoreTypes::free_model:
            {
                // State
                logString<<"pos=["<<current_robot_state->getPositionRobotWrtWorld().transpose()<<"]' ";
                logString<<"lin_speed=["<<current_robot_state->getLinearSpeedRobotWrtWorld().transpose()<<"]' ";
                logString<<"lin_accel=["<<current_robot_state->getLinearAccelerationRobotWrtWorld().transpose()<<"]' ";
                logString<<"attit=["<<current_robot_state->getAttitudeRobotWrtWorld().transpose()<<"]' ";
                logString<<"ang_vel=["<<current_robot_state->getAngularVelocityRobotWrtWorld().transpose()<<"]' ";


                // Jacobian


                break;
            }

        }

        logString<<std::endl;




        //// Sensors
        for(std::list<std::shared_ptr<StateCore> >::iterator itSensorStateCore=TheStateEstimationCore->state_component_->TheListSensorStateCore.begin();
            itSensorStateCore!=TheStateEstimationCore->state_component_->TheListSensorStateCore.end();
            ++itSensorStateCore)
        {
            std::shared_ptr<SensorCore> SensorCorePtrAux=std::dynamic_pointer_cast<SensorCore>((*itSensorStateCore)->getMsfElementCoreSharedPtr());
            logString<<"\t\t";
            logString<<"Sensor id="<<SensorCorePtrAux->getSensorId();

            switch(SensorCorePtrAux->getSensorType())
            {
                case SensorTypes::undefined:
                {
                    break;
                }
                case SensorTypes::imu:
                {
                    logString<<" (IMU)";
                    std::shared_ptr<const ImuSensorCore> ImuSensorCorePtrAux=std::dynamic_pointer_cast< const ImuSensorCore >(SensorCorePtrAux);


                    std::shared_ptr<ImuSensorStateCore> sensorStatePtr=std::static_pointer_cast< ImuSensorStateCore >(*itSensorStateCore);

                    // State (Parameters)
                    //if(ImuSensorCorePtrAux->isEstimationPositionSensorWrtRobotEnabled())
                        logString<<" posi_wrt_robot=["<<sensorStatePtr->getPositionSensorWrtRobot().transpose()<<"]'";

                    //if(ImuSensorCorePtrAux->isEstimationAttitudeSensorWrtRobotEnabled())
                        logString<<" atti_wrt_robot=["<<sensorStatePtr->getAttitudeSensorWrtRobot().transpose()<<"]'";

                    //if(ImuSensorCorePtrAux->isEstimationBiasLinearAccelerationEnabled())
                        logString<<" est_bias_lin_acc=["<<sensorStatePtr->getBiasesLinearAcceleration().transpose()<<"]'";

                    //if(ImuSensorCorePtrAux->isEstimationBiasAngularVelocityEnabled())
                        logString<<" est_bias_ang_vel=["<<sensorStatePtr->getBiasesAngularVelocity().transpose()<<"]'";


                    // Jacobian



                    break;
                }

            }

            logString<<std::endl;
        }


        //// Map
        for(std::list<std::shared_ptr<StateCore> >::iterator itSensorStateCore=TheStateEstimationCore->state_component_->TheListMapElementStateCore.begin();
            itSensorStateCore!=TheStateEstimationCore->state_component_->TheListMapElementStateCore.end();
            ++itSensorStateCore)
        {
            //std::shared_ptr<SensorCore> SensorCorePtrAux=std::dynamic_pointer_cast<SensorCore>((*itSensorStateCore)->getMsfElementCoreSharedPtr());
            logString<<"\t\t";
            logString<<"Map Element";



            logString<<std::endl;
        }


        /////// Covariance of the state
//        logString<<"\t";
//        logString<<"+Covariance of the state:"<<std::endl;
//        logString<<"\t\t";
//        logString<<"size: "<<TheStateEstimationCore->covarianceMatrix.rows()<<" x "<<TheStateEstimationCore->covarianceMatrix.cols()<<std::endl;
//        logString<<TheStateEstimationCore->covarianceMatrix<<std::endl;




    }




    /////// Measurements
    if(TheStateEstimationCore->hasMeasurement())
    {
        for(std::list< std::shared_ptr<SensorMeasurementCore> >::iterator itMeas=TheStateEstimationCore->sensor_measurement_component_->list_sensor_measurement_core_.begin();
            itMeas!=TheStateEstimationCore->sensor_measurement_component_->list_sensor_measurement_core_.end();
            ++itMeas)
        {
            logString<<"\t";
            logString<<"+Meas:"<<std::endl;

            std::shared_ptr<SensorCore> SensorCorePtrAux=(*itMeas)->getSensorCoreSharedPtr();
            logString<<"\t\t";
            logString<<"Sensor id="<<SensorCorePtrAux->getSensorId();

            switch(SensorCorePtrAux->getSensorType())
            {
                case SensorTypes::undefined:
                {
                    break;
                }
                case SensorTypes::imu:
                {
                    std::shared_ptr<ImuSensorCore> TheImuSensorCore=std::dynamic_pointer_cast<ImuSensorCore>(SensorCorePtrAux);


                    logString<<" (IMU)";
                    //std::shared_ptr<SensorMeasurementCore> measurePtrAux=(*itMeas);
                    //std::shared_ptr<ImuSensorMeasurementCore> measurePtr=dynamic_cast< std::shared_ptr<ImuSensorMeasurementCore> >(measurePtrAux);
                    std::shared_ptr<ImuSensorMeasurementCore> measurePtr=std::static_pointer_cast< ImuSensorMeasurementCore >(*itMeas);
                    //std::shared_ptr<ImuSensorMeasurementCore> measurePtr=std::dynamic_pointer_cast< ImuSensorMeasurementCore >(*itMeas);
                    if(TheImuSensorCore->isMeasurementOrientationEnabled())
                        logString<<" orientation=["<<measurePtr->getOrientation().transpose()<<"]'";
                    if(TheImuSensorCore->isMeasurementAngularVelocityEnabled())
                        logString<<" angularVel=["<<measurePtr->getAngularVelocity().transpose()<<"]'";
                    if(TheImuSensorCore->isMeasurementLinearAccelerationEnabled())
                        logString<<" linearAcc=["<<measurePtr->getLinearAcceleration().transpose()<<"]'";
                    break;
                }
                case SensorTypes::coded_visual_marker_eye:
                {
                    logString<<" (Coded Visual Marker Eye)";
                    break;
                }
                case SensorTypes::absolute_pose:
                {
                    logString<<" (Absolute Pose)";
                    break;
                }
            }



            logString<<std::endl;
        }
    }


    /////// Input
    if(TheStateEstimationCore->hasInputCommand())
    {
        for(std::list< std::shared_ptr<InputCommandCore> >::iterator itInputCommand=TheStateEstimationCore->input_command_component_->list_input_command_core_.begin();
            itInputCommand!=TheStateEstimationCore->input_command_component_->list_input_command_core_.end();
            ++itInputCommand)
        {
            logString<<"\t";
            logString<<"+Input Comm:"<<std::endl;

            std::shared_ptr<InputCore> InputCorePtrAux=(*itInputCommand)->getInputCoreSharedPtr();
            logString<<"\t\t";
            logString<<"Input";

            switch(InputCorePtrAux->getInputType())
            {
                case InputTypes::undefined:
                {
                    break;
                }
                case InputTypes::imu:
                {
                    std::shared_ptr<ImuInputCore> TheImuInputCore=std::dynamic_pointer_cast<ImuInputCore>(InputCorePtrAux);

                    logString<<" (IMU)";
                    std::shared_ptr<ImuInputCommandCore> inputCommandPtr=std::static_pointer_cast<ImuInputCommandCore>(*itInputCommand);
                    if(TheImuInputCore->isInputCommandOrientationEnabled())
                        logString<<" orientation=["<<inputCommandPtr->getOrientation().transpose()<<"]'";
                    if(TheImuInputCore->isInputCommandAngularVelocityEnabled())
                        logString<<" angularVel=["<<inputCommandPtr->getAngularVelocity().transpose()<<"]'";
                    if(TheImuInputCore->isInputCommandLinearAccelerationEnabled())
                        logString<<" linearAcc=["<<inputCommandPtr->getLinearAcceleration().transpose()<<"]'";
                    break;
                }

            }



            logString<<std::endl;
        }
    }



    // Log it
    this->log(logString.str());


    return 0;
}

int MsfStorageCore::displayRingBuffer()
{
    TheRingBufferMutex.lock();

    // Display Buffer
    {
        std::ostringstream logString;
        logString<<" "<<std::endl;
        logString<<"Displaying buffer of "<<this->getSize()<<" elements:"<<std::endl;

        this->log(logString.str());
    }



    for(std::list< StampedBufferObjectType< std::shared_ptr<StateEstimationCore> > >::iterator it=this->TheElementsList.begin(); it!=this->TheElementsList.end(); ++it)
    {
        this->displayStateEstimationElement(it->timeStamp, it->object);
    }

    //logFile<<" "<<std::endl;

    TheRingBufferMutex.unlock();


    // Buffer Info
    //logFile<<"Number of elements in buffer (before purge): "<<this->getSize()<<std::endl;


    return 0;
}




int MsfStorageCore::purgeRingBuffer(int numElementsFrom)
{
#if _DEBUG_MSF_STORAGE
    {
        std::ostringstream logString;
        logString<<"MsfStorageCore::purgeRingBuffer() numElementsFrom: "<<numElementsFrom<<std::endl;
        this->log(logString.str());
    }
#endif


    // TODO
    // Delete Lasts elements of the buffer, to avoid it growing a lot
    if(numElementsFrom>=0)
    {
        TheRingBufferMutex.lock();
        this->purgeLastElementsFromI(numElementsFrom);
        TheRingBufferMutex.unlock();
    }
    else
    {
        TheRingBufferMutex.lock();
        this->purgeFull();
        TheRingBufferMutex.unlock();
    }

    //std::cout<<"Number of elements in buffer (after purge): "<<this->getSize()<<std::endl;

#if _DEBUG_MSF_STORAGE
    {
        std::ostringstream logString;
        logString<<"MsfStorageCore::purgeRingBuffer() ended"<<std::endl;
        this->log(logString.str());
    }
#endif


    // End
    return 0;
}


int MsfStorageCore::purgeElementRingBuffer(const TimeStamp& TheTimeStamp)
{

    TheRingBufferMutex.lock();
    int error=purgeElementByStamp(TheTimeStamp);

    TheRingBufferMutex.unlock();

    return error;
}




int MsfStorageCore::addOutdatedElement(const TimeStamp &TheTimeStamp)
{
    // Search for duplicates
    std::list<TimeStamp>::iterator duplicate=std::find(outdatedBufferElements.begin(), outdatedBufferElements.end(), TheTimeStamp);

    // It is duplicated, no need to add it
    if(duplicate!=outdatedBufferElements.end())
        return 0;

    // Add element to the list
    outdatedBufferElements.push_back(TheTimeStamp);

    // Notify to wake up
    outdatedBufferElementsConditionVariable.notify_all();

    // End
    return 0;
}

int MsfStorageCore::getOldestOutdatedElement(TimeStamp &TheOutdatedTimeStamp, bool sleep_if_empty)
{
    // Check the list size
    while(outdatedBufferElements.size()==0)
    {
        // No new outdated measurement
        if(flag_new_measurement_)
        {
            // Unset flag
            flag_new_measurement_=false;

            // Buffer is clean -> We are updated!
            // notify to wake up
            updated_buffer_condition_variable_.notify_all();

        }

        // Wait until a new element is pushed in the buffer
        if(sleep_if_empty)
            outdatedBufferElementsConditionVariable.wait(*outdatedBufferElementsLock);
        else
            break;

    }

#if _DEBUG_MSF_STORAGE
    // Display
    this->displayOutdatedBufferElements();
#endif

    // Find the oldest element
    std::list<TimeStamp>::iterator minElement=min_element(outdatedBufferElements.begin(),outdatedBufferElements.end());

    if(minElement==outdatedBufferElements.end())
        return 1;

    // Save the TimeStamp to return it
    TheOutdatedTimeStamp=(*minElement);
    // Remove element of the list
    outdatedBufferElements.erase(minElement);
    // End
    return 0;
}


int MsfStorageCore::displayOutdatedBufferElements()
{
    this->log(getDisplayOutdatedElements());

    return 0;
}

std::string MsfStorageCore::getDisplayOutdatedElements()
{
    std::ostringstream logString;

    logString<<"List of outdated elements: ";
    for(std::list<TimeStamp>::iterator itElement=outdatedBufferElements.begin();
        itElement!=outdatedBufferElements.end();
        ++itElement)
    {
        logString<<"TS: sec="<<itElement->sec<<" s; nsec="<<itElement->nsec<<" ns. ";

    }

    logString<<std::endl;


    return logString.str();
}


int MsfStorageCore::semaphoreBufferUpdated()
{
    // Wait until buffer is updated
    updated_buffer_condition_variable_.wait(*updated_buffer_lock_);

    return 0;
}


int MsfStorageCore::log(std::string logString)
{
    // Lock mutex
    TheLogFileMutex.lock();

    // Write in file
    logFile<<logString;

    // Unlock mutex
    TheLogFileMutex.unlock();

    return 0;
}
