#ifndef _MSF_ELEMENT_CORE_H
#define _MSF_ELEMENT_CORE_H




//I/O stream
//std::cout
#include <iostream>


//String
//std::string, std::getline()
#include <string>

//String stream
//std::istringstream
#include <sstream>

//File Stream
//std::ofstream, std::ifstream
#include <fstream>


// Memory
#include <memory>


// Mutex
#include <mutex>





#include "msf_localization_core/time_stamp.h"




enum class MsfElementCoreTypes
{
    undefined=0,
    input,
    sensor,
    robot,
    map,
    world
};




class MsfStorageCore;




class MsfElementCore
{

public:
    MsfElementCore();
    MsfElementCore(std::weak_ptr<MsfElementCore> msf_element_core_ptr, std::weak_ptr<MsfStorageCore> msf_storage_core_ptr);
    virtual ~MsfElementCore();

protected:
    int init();
    int destroy();


    // MsfElementCoreTypes
protected:
    MsfElementCoreTypes msf_element_core_type_;
public:
    int setMsfElementCoreType(MsfElementCoreTypes msf_element_core_type);
    MsfElementCoreTypes getMsfElementCoreType() const;



    // Pointer to itself
protected:
    std::weak_ptr<MsfElementCore> msf_element_core_ptr_;
public:
    int setMsfElementCorePtr(std::weak_ptr<MsfElementCore> msf_element_core_ptr);
    std::weak_ptr<MsfElementCore> getMsfElementCoreWeakPtr() const;
    std::shared_ptr<MsfElementCore> getMsfElementCoreSharedPtr() const;


    // Pointer to the MSF Storage Core
protected:
    std::weak_ptr<MsfStorageCore> msf_storage_core_ptr_;
public:
    int setMsfStorageCorePtr(std::weak_ptr<MsfStorageCore> msf_storage_core_ptr);
    std::weak_ptr<MsfStorageCore> getMsfStorageCoreWeakPtr() const;
    std::shared_ptr<MsfStorageCore> getMsfStorageCoreSharedPtr() const;


public:
    virtual bool isCorrect();



    //// Debug log
protected:
    std::string log_path_;
    std::ofstream log_file_;
    // mutex to protect the log file
protected:
    std::mutex log_file_mutex_;
public:
    int log(std::string log_string);

};








#endif
