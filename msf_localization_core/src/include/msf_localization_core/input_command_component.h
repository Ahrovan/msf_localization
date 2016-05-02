
#ifndef _IMU_COMMAND_COMPONENT_H
#define _IMU_COMMAND_COMPONENT_H




//I/O stream
//std::cout
#include <iostream>

//String
//std::string, std::getline()
#include <string>



//Vector
//std::vector
#include <vector>

// List
#include <list>


#include <memory>


#include <Eigen/Dense>




/// Forward declarations

// Inputs
class InputCommandCore;





class InputCommandComponent
{
public:
    InputCommandComponent();
    ~InputCommandComponent();



    /// Input Commands

    // Check
public:
    bool hasInputCommand() const;


    // Dimension total of input and error input
public:
    int getDimensionInputCommand() const;
    int getDimensionErrorInputCommand() const;


    // Avaliable Inputs
public:
    std::list< std::shared_ptr<InputCommandCore> > TheListInputCommandCore;




};


#endif
