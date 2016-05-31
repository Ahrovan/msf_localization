#ifndef _INPUT_CORE_H
#define _INPUT_CORE_H




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



#include <Eigen/Dense>
#include <Eigen/Sparse>


#include "msf_localization_core/time_stamp.h"

#include "msf_localization_core/msf_element_core.h"


enum class InputTypes
{
    undefined=0,
    imu=1,
    absolute_pose
};




//class InputCommandCore;


class InputCore : public MsfElementCore
{

public:
    InputCore();
    InputCore(MsfLocalizationCore* msf_localization_core_ptr);
    ~InputCore();

protected:
    int init();


    // InputTypes
protected:
    InputTypes input_type_;
public:
    int setInputType(InputTypes input_type);
    InputTypes getInputType() const;


    // Input Name
protected:
    std::string input_name_;
public:
    int setInputName(std::string input_name);
    std::string getInputName() const;

    // Input Id
protected:
    int id_;
public:
    void setInputId(int id);
    int getInputId() const;


protected:
    bool flag_input_enabled_;
public:
    bool isInputEnabled() const;
    int setInputEnabled(bool flag_input_enabled);


    // Dimension input
protected:
    unsigned int dimension_input_command_;
public:
    unsigned int getDimensionInputCommand() const;
    int setDimensionInputCommand(unsigned int dimension_input_command);

    // Dimension error input
protected:
    unsigned int dimension_error_input_command_;
public:
    unsigned int getDimensionErrorInputCommand() const;
    int setDimensionErrorInputCommand(unsigned int dimension_error_input_command);




    ///// Covariances Getters

    // Covariance Error Inputs: Qu
public:
    virtual Eigen::SparseMatrix<double> getCovarianceInputs(const TimeStamp deltaTimeStamp)=0;




    ///// Predict Step functions

    // None


    //// Update Step functions

    // None




};



#endif
