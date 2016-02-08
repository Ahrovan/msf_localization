
#ifndef _SENSOR_CORE_H
#define _SENSOR_CORE_H


//I/O stream
//std::cout
#include <iostream>

#include <memory>



#include "sensor_basics.h"


#include "state_estimation_core.h"






class SensorCore : public SensorBasics
{
public:
    SensorCore();
    virtual ~SensorCore();


protected:
    std::shared_ptr<MsfStorageCore> TheMsfStorageCore;
public:
    int setTheMsfStorageCore(std::shared_ptr<MsfStorageCore> TheMsfStorageCore);







};







#endif
