

#ifndef _STATE_COMPONENT_H
#define _STATE_COMPONENT_H


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

// States
class StateCore;



// Class that stores the information for every time instant
class StateComponent
{

public:
    StateComponent();
    ~StateComponent();



    // Check
public:
    bool checkState() const;
    bool hasState() const;

    // Dimension total of state and error state
public:
    int getDimensionState() const;
    int getDimensionErrorState() const;


    // Dimension total of parameters and error parameters
public:
    int getDimensionParameters() const;
    int getDimensionErrorParameters() const;


    // World (Global Parameters) State
public:
    std::shared_ptr<StateCore> TheGlobalParametersStateCore;

    // Robot State
public:
    std::shared_ptr<StateCore> TheRobotStateCore;

    // Inputs State
public:
    std::list< std::shared_ptr<StateCore> > TheListInputStateCore;
    int getNumberInputStates() const;

    // Sensors State
public:
    std::list< std::shared_ptr<StateCore> > TheListSensorStateCore;
    int getNumberSensorStates() const;

    // Map State
public:
    std::list< std::shared_ptr<StateCore> > TheListMapElementStateCore;
    int getNumberMapElementStates() const;



    // Covariances Matrixes
public:
    int prepareCovarianceInitErrorState();


    // Covariances Matrixes of the error state
public:
    std::shared_ptr<Eigen::MatrixXd> covarianceMatrix;



};






#endif
