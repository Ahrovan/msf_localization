
#ifndef _STATE_CORE_H
#define _STATE_CORE_H


#include <memory>

#include <Eigen/Dense>
#include <Eigen/Sparse>


#include "time_stamp/time_stamp.h"



enum class StateCoreTypes
{
    undefined=0,
    input,
    sensor,
    robot,
    map,
    world
};



// Formward
class MsfElementCore;


class StateCore
{
public:
    StateCore();
    StateCore(const std::weak_ptr<MsfElementCore> msf_element_core_ptr);
    ~StateCore();


protected:
    int init();



    // Ptr to MsfElementCore
protected:
    // It is not the owner of this Pointer. it doesn't modify the pointer
    std::weak_ptr<MsfElementCore> msf_element_core_ptr_;
public:
    int setMsfElementCorePtr(const std::weak_ptr<MsfElementCore> msf_element_core_ptr);
    std::shared_ptr<MsfElementCore> getMsfElementCoreSharedPtr() const;
    std::weak_ptr<MsfElementCore> getMsfElementCoreWeakPtr() const;


    // Type
protected:
    StateCoreTypes state_core_type_;
public:
    int setStateCoreType(StateCoreTypes state_core_type);
    StateCoreTypes getStateCoreType() const;


public:
    virtual bool isCorrect() const;



    //// Jacobians Prediction

    /// Fx
protected:
public:
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
    int setJacobianErrorStateWorld(const Eigen::SparseMatrix<double>& jacobian_error_state);
    int setJacobianErrorStateRobot(const Eigen::SparseMatrix<double>& jacobian_error_state);
    int setJacobianErrorStateInput(const Eigen::SparseMatrix<double>& jacobian_error_state, int input_number);
    int setJacobianErrorStateSensor(const Eigen::SparseMatrix<double>& jacobian_error_state, int sensor_number);
    int setJacobianErrorStateMapElement(const Eigen::SparseMatrix<double>& jacobian_error_state, int map_element_number);
public:
    int setJacobianErrorStateInputsSize(int size_inputs);
    int setJacobianErrorStateSensorsSize(int size_sensors);
    int setJacobianErrorStateMapElementsSize(int size_map_elements);



    /// Fp
protected:
public:
    struct
    {
        Eigen::SparseMatrix<double> world;
        Eigen::SparseMatrix<double> robot;
        std::vector<Eigen::SparseMatrix<double> > inputs;
        std::vector<Eigen::SparseMatrix<double> > sensors;
        std::vector<Eigen::SparseMatrix<double> > map_elements;
    } jacobian_error_parameters_;
public:
    Eigen::SparseMatrix<double> getJacobianErrorParametersWorld();
    Eigen::SparseMatrix<double> getJacobianErrorParametersRobot();
    Eigen::SparseMatrix<double> getJacobianErrorParametersInput(int input_number);
    Eigen::SparseMatrix<double> getJacobianErrorParametersSensor(int sensor_number);
    Eigen::SparseMatrix<double> getJacobianErrorParametersMapElement(int map_element_number);



    /// Fu
protected:
public:
    struct
    {
        std::vector<Eigen::SparseMatrix<double> > input_commands;
    } jacobian_error_input_commands_;
public:
    Eigen::SparseMatrix<double> getJacobianErrorInputCommands(int input_number);



    /// Fn
protected:
public:
    Eigen::SparseMatrix<double> jacobian_error_state_noise_;
public:
    Eigen::SparseMatrix<double> getJacobianErrorStateNoise();




    //// Jacobian Error State Reset

public:
    Eigen::SparseMatrix<double> jacobian_error_state_reset_;



    //// Update State

public:
    virtual int updateStateFromIncrementErrorState(const Eigen::VectorXd& increment_error_state)=0;





    ///// Covariances getters


    // Covariance Sensor Error Parameters: Qp = Rp
public:
    virtual Eigen::SparseMatrix<double> getCovarianceParameters();


    // Covariance Noise Estimation: Qn
public:
    virtual Eigen::SparseMatrix<double> getCovarianceNoise(const TimeStamp& delta_time_stamp);

};





#endif
