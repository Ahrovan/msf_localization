
#ifndef _ROBOT_CORE_H
#define _ROBOT_CORE_H



//I/O stream
//std::cout
#include <iostream>


#include <memory>



enum class RobotTypes
{
    undefined=0,
    free_model=1
};




class MsfStorageCore;

class RobotCore
{

public:
    RobotCore();
    virtual ~RobotCore();


    // Dimension state
protected:
    unsigned int dimensionState;
public:
    unsigned int getDimensionState() const;
    int setDimensionState(unsigned int dimensionState);

    // Dimension error state
protected:
    unsigned int dimensionErrorState;
public:
    unsigned int getDimensionErrorState() const;
    int setDimensionErrorState(unsigned int dimensionErrorState);


    // Robot Core
protected:
    RobotTypes robotType;
public:
    int setRobotType(RobotTypes robotType);
    RobotTypes getRobotType() const;


    // Pointer to itself
protected:
//public:
    std::weak_ptr<const RobotCore> TheRobotCorePtr;
public:
    int setTheRobotCore(std::weak_ptr<const RobotCore> TheRobotCorePtr);
    std::shared_ptr<const RobotCore> getTheRobotCore() const;


    // Pointer to the MSF Storage Core
protected:
    std::weak_ptr<MsfStorageCore> TheMsfStorageCore;
public:
    int setTheMsfStorageCore(std::weak_ptr<MsfStorageCore> TheMsfStorageCore);


};









#endif
