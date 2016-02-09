
#ifndef _SENSOR_CORE_H
#define _SENSOR_CORE_H


//I/O stream
//std::cout
#include <iostream>

#include <memory>



#include "sensor_basics.h"


// For TimeStamp
//#include "stamped_ring_buffer.h"





class MsfStorageCore;


class SensorCore : public SensorBasics
{
public:
    SensorCore();
    virtual ~SensorCore();


    // Pointer to itself
protected:
public:
    std::weak_ptr<const SensorCore> SensorCorePtr;


    // Pointer to the MSF Storage Core
protected:
    std::weak_ptr<MsfStorageCore> TheMsfStorageCore;
public:
    int setTheMsfStorageCore(std::weak_ptr<MsfStorageCore> TheMsfStorageCore);







};







#endif
