
#include "msf_storage_core.h"


//#define _DEBUG_DISPLAY


MsfStorageCore::MsfStorageCore()
{
    // Mutex
    outdatedBufferElementsLock=new std::unique_lock<std::mutex>(outdatedBufferElementsMutex);


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
    logFile.close();
    return;
}

int MsfStorageCore::setMeasurement(const TimeStamp TheTimeStamp, const std::shared_ptr<SensorMeasurementCore> TheSensorMeasurement)
{
    std::cout<<"MsfStorageCore::setMeasurement TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;

    // Add measurement to the MSF Storage Core
    StampedBufferObjectType< std::shared_ptr<StateEstimationCore> > TheMeasurementToTheBuffer;

    // Stamp
    TheMeasurementToTheBuffer.timeStamp=TheTimeStamp;

    // Measure
    TheMeasurementToTheBuffer.object=std::make_shared<StateEstimationCore>();
    TheMeasurementToTheBuffer.object->TheListMeasurementCore.push_back(TheSensorMeasurement);
    //TheMeasurementToTheBuffer.object.flagHasMeasurement=true;



    // Add measurement to the MSF Storage Core
    // TODO, protect to avoid races!
    // TODO fix!
    TheRingBufferMutex.lock();
    this->addElementByStamp(TheMeasurementToTheBuffer);
    TheRingBufferMutex.unlock();


    // Add TimeStamp to the outdated elements list
    addOutdatedElement(TheTimeStamp);


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
        this->getElementI(BufferElement, itElement);
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



int MsfStorageCore::getElement(const TimeStamp timeStamp, std::shared_ptr<StateEstimationCore> &TheElement)
{
    //StateEstimationCore BufferElement;

    TheRingBufferMutex.lock();
    this->getElementByStamp(timeStamp, TheElement);
    TheRingBufferMutex.unlock();

    //TheElement=BufferElement.object;

    return 0;
}

int MsfStorageCore::getNextTimeStamp(const TimeStamp previousTimeStamp, TimeStamp& nextTimeStamp)
{
    TheRingBufferMutex.lock();
    int result=this->searchPreIStampByStamp(previousTimeStamp, nextTimeStamp);
    TheRingBufferMutex.unlock();

    return result;
}



int MsfStorageCore::addElement(const TimeStamp TheTimeStamp, const std::shared_ptr<StateEstimationCore> TheStateEstimationCore)
{
    StampedBufferObjectType< std::shared_ptr<StateEstimationCore> > BufferElement;

    BufferElement.timeStamp=TheTimeStamp;
    BufferElement.object=TheStateEstimationCore;

    TheRingBufferMutex.lock();
    this->addElementByStamp(BufferElement);
    TheRingBufferMutex.unlock();

    return 0;
}


int MsfStorageCore::displayRingBuffer()
{
    TheRingBufferMutex.lock();

    // Display Buffer
    logFile<<" "<<std::endl;
    logFile<<"Displaying buffer of "<<this->getSize()<<" elements:"<<std::endl;
    for(std::list< StampedBufferObjectType< std::shared_ptr<StateEstimationCore> > >::iterator it=this->TheElementsList.begin(); it!=this->TheElementsList.end(); ++it)
    {
        logFile<<"-TS="<<it->timeStamp.sec<<" s; "<<it->timeStamp.nsec<<" ns"<<std::endl;


        /////// State
        if(it->object->hasState())
        {
            logFile<<"\t";
            logFile<<"+State:"<<std::endl;


            //// Robot
            // TODO
            logFile<<"\t\t";
            logFile<<"Robot ";

            switch(it->object->TheRobotStateCore->getTheRobotCore()->getRobotType())
            {
                case RobotTypes::undefined:
                {
                    break;
                }

                case RobotTypes::free_model:
                {
                    std::shared_ptr<const FreeModelRobotCore> FreeModelRobotPtr=std::dynamic_pointer_cast< const FreeModelRobotCore >(it->object->TheRobotStateCore->getTheRobotCore());
                    std::shared_ptr<FreeModelRobotStateCore> FreeModelRobotStatePtr=std::static_pointer_cast< FreeModelRobotStateCore >(it->object->TheRobotStateCore);

                    // State
                    logFile<<"pos=["<<FreeModelRobotStatePtr->getPosition().transpose()<<"]' ";
                    logFile<<"lin_speed=["<<FreeModelRobotStatePtr->getLinearSpeed().transpose()<<"]' ";
                    logFile<<"lin_accel=["<<FreeModelRobotStatePtr->getLinearAcceleration().transpose()<<"]' ";
                    logFile<<"attit=["<<FreeModelRobotStatePtr->getAttitude().transpose()<<"]' ";
                    logFile<<"ang_vel=["<<FreeModelRobotStatePtr->getAngularVelocity().transpose()<<"]' ";


                    // Jacobian
#ifdef _DEBUG_DISPLAY
                    logFile<<std::endl;
                    logFile<<"Jacobian Robot Linear=["<<std::endl<<FreeModelRobotStatePtr->errorStateJacobian.linear<<"]";

                    logFile<<std::endl;
                    logFile<<"Jacobian Robot Angular=["<<std::endl<<FreeModelRobotStatePtr->errorStateJacobian.angular<<"]";
#endif

                    break;
                }

            }

            logFile<<std::endl;




            //// Sensors
            for(std::list<std::shared_ptr<SensorStateCore> >::iterator itSensorStateCore=it->object->TheListSensorStateCore.begin();
                itSensorStateCore!=it->object->TheListSensorStateCore.end();
                ++itSensorStateCore)
            {
                std::shared_ptr<const SensorCore> SensorCorePtrAux=(*itSensorStateCore)->getTheSensorCore();
                logFile<<"\t\t";
                logFile<<"Sensor id="<<SensorCorePtrAux->getSensorId();

                switch(SensorCorePtrAux->getSensorType())
                {
                    case SensorTypes::undefined:
                    {
                        break;
                    }
                    case SensorTypes::imu:
                    {
                        logFile<<" (IMU)";
                        std::shared_ptr<const ImuSensorCore> ImuSensorCorePtrAux=std::dynamic_pointer_cast< const ImuSensorCore >(SensorCorePtrAux);


                        std::shared_ptr<ImuSensorStateCore> sensorStatePtr=std::static_pointer_cast< ImuSensorStateCore >(*itSensorStateCore);

                        // State (Parameters)
                        //if(ImuSensorCorePtrAux->isEstimationPositionSensorWrtRobotEnabled())
                            logFile<<" posi_wrt_robot=["<<sensorStatePtr->getPositionSensorWrtRobot().transpose()<<"]'";

                        //if(ImuSensorCorePtrAux->isEstimationAttitudeSensorWrtRobotEnabled())
                            logFile<<" atti_wrt_robot=["<<sensorStatePtr->getAttitudeSensorWrtRobot().transpose()<<"]'";

                        //if(ImuSensorCorePtrAux->isEstimationBiasLinearAccelerationEnabled())
                            logFile<<" est_bias_lin_acc=["<<sensorStatePtr->getBiasesLinearAcceleration().transpose()<<"]'";

                        //if(ImuSensorCorePtrAux->isEstimationBiasAngularVelocityEnabled())
                            logFile<<" est_bias_ang_vel=["<<sensorStatePtr->getBiasesAngularVelocity().transpose()<<"]'";


                        // Jacobian
#ifdef _DEBUG_DISPLAY
                        logFile<<std::endl;
                        if(ImuSensorCorePtrAux->isEstimationPositionSensorWrtRobotEnabled())
                            logFile<<"Jacobian Posi wrt Robot=["<<std::endl<<sensorStatePtr->errorStateJacobian.positionSensorWrtRobot<<"]";

                        logFile<<std::endl;
                        if(ImuSensorCorePtrAux->isEstimationAttitudeSensorWrtRobotEnabled())
                            logFile<<"Jacobian Atti wrt Robot=["<<std::endl<<sensorStatePtr->errorStateJacobian.attitudeSensorWrtRobot<<"]";

                        logFile<<std::endl;
                        if(ImuSensorCorePtrAux->isEstimationBiasLinearAccelerationEnabled())
                            logFile<<"Jacobian Atti Linear Accele=["<<std::endl<<sensorStatePtr->errorStateJacobian.biasesLinearAcceleration<<"]";

                        logFile<<std::endl;
                        if(ImuSensorCorePtrAux->isEstimationBiasAngularVelocityEnabled())
                            logFile<<"Jacobian Bias Angular Veloc=["<<std::endl<<sensorStatePtr->errorStateJacobian.biasesAngularVelocity<<"]";
#endif


                        break;
                    }

                }

                logFile<<std::endl;
            }


            //// Map
            // TODO

        }


        /////// Measurements
        if(it->object->hasMeasurement())
        {
            for(std::list< std::shared_ptr<SensorMeasurementCore> >::iterator itMeas=it->object->TheListMeasurementCore.begin(); itMeas!=it->object->TheListMeasurementCore.end(); ++itMeas)
            {
                logFile<<"\t";
                logFile<<"+Meas:"<<std::endl;

                std::shared_ptr<const SensorCore> SensorCorePtrAux=(*itMeas)->getTheSensorCore().lock();
                logFile<<"\t\t";
                logFile<<"Sensor id="<<SensorCorePtrAux->getSensorId();

                switch(SensorCorePtrAux->getSensorType())
                {
                    case SensorTypes::undefined:
                    {
                        break;
                    }
                    case SensorTypes::imu:
                    {
                        logFile<<" (IMU)";
                        //std::shared_ptr<SensorMeasurementCore> measurePtrAux=(*itMeas);
                        //std::shared_ptr<ImuSensorMeasurementCore> measurePtr=dynamic_cast< std::shared_ptr<ImuSensorMeasurementCore> >(measurePtrAux);
                        std::shared_ptr<ImuSensorMeasurementCore> measurePtr=std::static_pointer_cast< ImuSensorMeasurementCore >(*itMeas);
                        //std::shared_ptr<ImuSensorMeasurementCore> measurePtr=std::dynamic_pointer_cast< ImuSensorMeasurementCore >(*itMeas);
                        if(measurePtr->isOrientationSet())
                            logFile<<" orientation=["<<measurePtr->getOrientation().transpose()<<"]'";
                        if(measurePtr->isAngularVelocitySet())
                            logFile<<" angularVel=["<<measurePtr->getAngularVelocity().transpose()<<"]'";
                        if(measurePtr->isLinearAccelerationSet())
                            logFile<<" linearAcc=["<<measurePtr->getLinearAcceleration().transpose()<<"]'";
                        break;
                    }

                }



                logFile<<std::endl;
            }
        }

        //logFile<<" "<<std::endl;
    }

    logFile<<" "<<std::endl;

    TheRingBufferMutex.unlock();


    // Buffer Info
    //logFile<<"Number of elements in buffer (before purge): "<<this->getSize()<<std::endl;


    return 0;
}




int MsfStorageCore::purgeRingBuffer(int numElementsFrom)
{
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
}




int MsfStorageCore::addOutdatedElement(TimeStamp TheTimeStamp)
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

int MsfStorageCore::getOldestOutdatedElement(TimeStamp &TheOutdatedTimeStamp)
{
    // Check the list size
    while(outdatedBufferElements.size()==0)
    {
        //cv.wait(lck);
        outdatedBufferElementsConditionVariable.wait(*outdatedBufferElementsLock);
    }

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
