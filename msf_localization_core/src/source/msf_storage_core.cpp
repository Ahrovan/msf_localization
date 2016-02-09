
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
    TheMeasurementToTheBuffer.object.flagHasMeasurement=true;



    // Add measurement to the MSF Storage Core
    // TODO, protect to avoid races!
    // TODO fix!
    this->addElementByStamp(TheMeasurementToTheBuffer);



    // Display Buffer
    std::cout<<" "<<std::endl;
    std::cout<<"Displaying buffer of "<<this->getSize()<<" elements:"<<std::endl;
    for(std::list< StampedBufferObjectType<StateEstimationCore> >::iterator it=this->TheElementsList.begin(); it!=this->TheElementsList.end(); ++it)
    {
        std::cout<<"-TS="<<it->timeStamp.sec<<" s; "<<it->timeStamp.nsec<<" ns"<<std::endl;
        if(it->object.flagHasMeasurement)
        {
            for(std::list< std::shared_ptr<SensorMeasurementCore> >::iterator itMeas=it->object.TheListMeasurementCore.begin(); itMeas!=it->object.TheListMeasurementCore.end(); ++itMeas)
            {
                std::cout<<" +Meas:";

                std::shared_ptr<const SensorCore> SensorCorePtrAux=(*itMeas)->getTheSensorCore().lock();
                std::cout<<" Sensor id="<<SensorCorePtrAux->getSensorId();

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
    }


    // Buffer Info
    std::cout<<"Number of elements in buffer (before purge): "<<this->getSize()<<std::endl;


    // TODO
    // Delete Lasts elements of the buffer, to avoid it growing a lot
    this->purgeLastElementsFromI(10);

    std::cout<<"Number of elements in buffer (after purge): "<<this->getSize()<<std::endl;






    return 0;
}






