
#include "msf_storage_core.h"




MsfStorageCore::MsfStorageCore()
{

    return;
}


MsfStorageCore::~MsfStorageCore()
{
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
    std::cout<<" "<<std::endl;
    std::cout<<"Displaying buffer of "<<this->getSize()<<" elements:"<<std::endl;
    for(std::list< StampedBufferObjectType<StateEstimationCore> >::iterator it=this->TheElementsList.begin(); it!=this->TheElementsList.end(); ++it)
    {
        std::cout<<"-TS="<<it->timeStamp.sec<<" s; "<<it->timeStamp.nsec<<" ns"<<std::endl;


        // State
        if(it->object.hasState())
        {
            std::cout<<"\t\t\t\t";
            std::cout<<"+State:"<<std::endl;
        }


        // Measurements
        if(it->object.hasMeasurement())
        {
            for(std::list< std::shared_ptr<SensorMeasurementCore> >::iterator itMeas=it->object.TheListMeasurementCore.begin(); itMeas!=it->object.TheListMeasurementCore.end(); ++itMeas)
            {
                std::cout<<"\t\t\t\t";
                std::cout<<"+Meas:"<<std::endl;

                std::shared_ptr<const SensorCore> SensorCorePtrAux=(*itMeas)->getTheSensorCore().lock();
                std::cout<<"\t\t\t\t\t";
                std::cout<<"Sensor id="<<SensorCorePtrAux->getSensorId();

                switch(SensorCorePtrAux->getSensorType())
                {
                    case SensorTypes::undefined:
                    {
                        break;
                    }
                    case SensorTypes::imu:
                    {
                        std::cout<<" (IMU)";
                        //std::shared_ptr<SensorMeasurementCore> measurePtrAux=(*itMeas);
                        //std::shared_ptr<ImuSensorMeasurementCore> measurePtr=dynamic_cast< std::shared_ptr<ImuSensorMeasurementCore> >(measurePtrAux);
                        std::shared_ptr<ImuSensorMeasurementCore> measurePtr=std::static_pointer_cast< ImuSensorMeasurementCore >(*itMeas);
                        //std::shared_ptr<ImuSensorMeasurementCore> measurePtr=std::dynamic_pointer_cast< ImuSensorMeasurementCore >(*itMeas);
                        if(measurePtr->isOrientationSet())
                            std::cout<<" orientation=["<<measurePtr->getOrientation().transpose()<<"]'";
                        if(measurePtr->isAngularVelocitySet())
                            std::cout<<" angularVel=["<<measurePtr->getAngularVelocity().transpose()<<"]'";
                        if(measurePtr->isLinearAccelerationSet())
                            std::cout<<" linearAcc=["<<measurePtr->getLinearAcceleration().transpose()<<"]'";
                        break;
                    }

                }



                std::cout<<std::endl;
            }
        }

        //std::cout<<" "<<std::endl;
    }

    std::cout<<" "<<std::endl;

    TheRingBufferMutex.unlock();


    // Buffer Info
    //std::cout<<"Number of elements in buffer (before purge): "<<this->getSize()<<std::endl;


    return 0;
}




int MsfStorageCore::purgeRingBuffer(int numElementsFrom)
{
    // TODO
    // Delete Lasts elements of the buffer, to avoid it growing a lot
    TheRingBufferMutex.lock();
    this->purgeLastElementsFromI(numElementsFrom);
    TheRingBufferMutex.unlock();

    //std::cout<<"Number of elements in buffer (after purge): "<<this->getSize()<<std::endl;
}
