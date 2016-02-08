
#include "state_estimation_core.h"





StateEstimationCore::StateEstimationCore() :
    flagHasRobotState(false),
    flagHasSensorState(false),
    flagHasMeasurement(false)
{
    return;
}

StateEstimationCore::~StateEstimationCore()
{
    // Be tidy

    // TheListSensorStateCore
//    for(std::list<SensorStateCore*>::iterator it=TheListSensorStateCore.begin(); it!=TheListSensorStateCore.end(); ++it)
//        delete *it;
    this->TheListSensorStateCore.clear();

    // TheListMeasurementCore
//    for(std::list<SensorMeasurementCore*>::iterator it=TheListMeasurementCore.begin(); it!=TheListMeasurementCore.end(); ++it)
//        delete *it;
    this->TheListMeasurementCore.clear();


    return;
}









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
    this->addElementByStamp(TheMeasurementToTheBuffer);

    std::cout<<"Number of elements in buffer: "<<this->getSize()<<std::endl;


    // TODO
    // Delete Lasts elements of the buffer, to avoid it growing a lot
    this->purgeLastElementsFromI(10);

    std::cout<<"Number of elements in buffer (after purge): "<<this->getSize()<<std::endl;






    return 0;
}






