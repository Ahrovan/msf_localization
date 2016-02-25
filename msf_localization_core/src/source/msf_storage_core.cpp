
#include "msf_storage_core.h"


//#define _DEBUG_DISPLAY


MsfStorageCore::MsfStorageCore()
{
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

int MsfStorageCore::setMeasurement(TimeStamp TheTimeStamp, std::shared_ptr<SensorMeasurementCore> TheSensorMeasurement)
{

    // Add measurement to the MSF Storage Core
    StampedBufferObjectType<StateEstimationCore> TheMeasurementToTheBuffer;

    // Stamp
    TheMeasurementToTheBuffer.timeStamp=TheTimeStamp;

    // Measure
    TheMeasurementToTheBuffer.object.TheListMeasurementCore.push_back(TheSensorMeasurement);
    //TheMeasurementToTheBuffer.object.flagHasMeasurement=true;



    // Add measurement to the MSF Storage Core
    // TODO, protect to avoid races!
    // TODO fix!
    TheRingBufferMutex.lock();
    this->addElementByStamp(TheMeasurementToTheBuffer);
    TheRingBufferMutex.unlock();



    return 0;
}


int MsfStorageCore::getLastElementWithStateEstimate(TimeStamp& TheTimeStamp, StateEstimationCore& PreviousState)
{
    StampedBufferObjectType<StateEstimationCore> BufferElement;

    TheRingBufferMutex.lock();
    for(std::list< StampedBufferObjectType<StateEstimationCore> >::iterator itElement=this->getBegin();
        itElement!=this->getEnd();
        ++itElement)
    {
        this->getElementI(BufferElement, itElement);
        if(BufferElement.object.hasState())
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


int MsfStorageCore::addElement(TimeStamp TheTimeStamp, StateEstimationCore TheStateEstimationCore)
{
    StampedBufferObjectType<StateEstimationCore> BufferElement;

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
    for(std::list< StampedBufferObjectType<StateEstimationCore> >::iterator it=this->TheElementsList.begin(); it!=this->TheElementsList.end(); ++it)
    {
        logFile<<"-TS="<<it->timeStamp.sec<<" s; "<<it->timeStamp.nsec<<" ns"<<std::endl;


        /////// State
        if(it->object.hasState())
        {
            logFile<<"\t";
            logFile<<"+State:"<<std::endl;


            //// Robot
            // TODO
            logFile<<"\t\t";
            logFile<<"Robot ";

            switch(it->object.TheRobotStateCore->getTheRobotCore()->getRobotType())
            {
                case RobotTypes::undefined:
                {
                    break;
                }

                case RobotTypes::free_model:
                {
                    std::shared_ptr<const FreeModelRobotCore> FreeModelRobotPtr=std::dynamic_pointer_cast< const FreeModelRobotCore >(it->object.TheRobotStateCore->getTheRobotCore());
                    std::shared_ptr<FreeModelRobotStateCore> FreeModelRobotStatePtr=std::static_pointer_cast< FreeModelRobotStateCore >(it->object.TheRobotStateCore);

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
            for(std::list<std::shared_ptr<SensorStateCore> >::iterator itSensorStateCore=it->object.TheListSensorStateCore.begin();
                itSensorStateCore!=it->object.TheListSensorStateCore.end();
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
                            logFile<<" est_bis_lin_acc=["<<sensorStatePtr->getBiasesLinearAcceleration().transpose()<<"]'";

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
        if(it->object.hasMeasurement())
        {
            for(std::list< std::shared_ptr<SensorMeasurementCore> >::iterator itMeas=it->object.TheListMeasurementCore.begin(); itMeas!=it->object.TheListMeasurementCore.end(); ++itMeas)
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
