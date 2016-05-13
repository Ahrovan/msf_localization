
#ifndef _STATE_CORE_H
#define _STATE_CORE_H


#include <Eigen/Dense>

#include "msf_localization_core/msf_element_core.h"



enum class StateCoreTypes
{
    undefined=0,
    input,
    sensor,
    robot,
    map,
    world
};



class StateCore
{
public:
    StateCore();
    StateCore(std::weak_ptr<MsfElementCore> msf_element_core_ptr);
    ~StateCore();


protected:
    int init();



    // Ptr to MsfElementCore
protected:
    // It is not the owner of this Pointer. it doesn't modify the pointer
    std::weak_ptr<MsfElementCore> msf_element_core_ptr_;
public:
    int setMsfElementCorePtr(std::weak_ptr<MsfElementCore> msf_element_core_ptr);
    std::shared_ptr<MsfElementCore> getMsfElementCoreSharedPtr() const;
    std::weak_ptr<MsfElementCore> getMsfElementCoreWeakPtr() const;


    // Type
protected:
    StateCoreTypes state_core_type_;
public:
    int setStateCoreType(StateCoreTypes state_core_type);
    StateCoreTypes getStateCoreType() const;


public:
    virtual bool isCorrect();



    //// Jacobians Prediction

    // Fx
protected:
    struct
    {
        Eigen::SparseMatrix<double> world;
        Eigen::SparseMatrix<double> robot;
        std::vector<Eigen::SparseMatrix<double> > inputs;
        std::vector<Eigen::SparseMatrix<double> > sensors;
        std::vector<Eigen::SparseMatrix<double> > map_elements;
    } jacobian_error_state_;
public:
    Eigen::SparseMatrix<double> getJacobianErrorStateWorld();
    Eigen::SparseMatrix<double> getJacobianErrorStateRobot();
    Eigen::SparseMatrix<double> getJacobianErrorStateInput(int input_number);
    Eigen::SparseMatrix<double> getJacobianErrorStateSensor(int sensor_number);
    Eigen::SparseMatrix<double> getJacobianErrorStateMapElement(int map_element_number);
public:
    int setJacobianErrorStateWorld(Eigen::SparseMatrix<double> jacobian_error_state);
    int setJacobianErrorStateRobot(Eigen::SparseMatrix<double> jacobian_error_state);
    int setJacobianErrorStateInput(Eigen::SparseMatrix<double> jacobian_error_state, int input_number);
    int setJacobianErrorStateSensor(Eigen::SparseMatrix<double> jacobian_error_state, int sensor_number);
    int setJacobianErrorStateMapElement(Eigen::SparseMatrix<double> jacobian_error_state, int map_element_number);
public:
    int setJacobianErrorStateInputsSize(int size_inputs);
    int setJacobianErrorStateSensorsSize(int size_sensors);
    int setJacobianErrorStateMapElementsSize(int size_map_elements);



    // Fp
protected:
public:
    Eigen::SparseMatrix<double> jacobian_error_parameters_;
public:
    Eigen::SparseMatrix<double> getJacobianErrorParameters();

public:
    struct
    {
        Eigen::SparseMatrix<double> world;
        Eigen::SparseMatrix<double> robot;
        std::vector<Eigen::SparseMatrix<double> > inputs;
        std::vector<Eigen::SparseMatrix<double> > sensors;
        std::vector<Eigen::SparseMatrix<double> > map_elements;
    } jacobian_error_parameters_t;



    // Fu
protected:
public:
    Eigen::SparseMatrix<double> jacobian_error_inputs_;
public:
    Eigen::SparseMatrix<double> getJacobianErrorInputs();


public:
    struct
    {
        std::vector<Eigen::SparseMatrix<double> > input_commands;
    } jacobian_error_input_commands_t;



    // Fn
protected:
public:
    Eigen::SparseMatrix<double> jacobian_error_state_noise_;
public:
    Eigen::SparseMatrix<double> getJacobianErrorStateNoise();


public:
    struct
    {
        Eigen::SparseMatrix<double> world;
        Eigen::SparseMatrix<double> robot;
        std::vector<Eigen::SparseMatrix<double> > inputs;
        std::vector<Eigen::SparseMatrix<double> > sensors;
        std::vector<Eigen::SparseMatrix<double> > map_elements;
    } jacobian_error_state_noise_t;




    //// Jacobian Error State Reset

public:
    Eigen::SparseMatrix<double> jacobian_error_state_reset_;



    //// Update State

public:
    virtual int updateStateFromIncrementErrorState(const Eigen::VectorXd& increment_error_state)=0;


};





#endif
