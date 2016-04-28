
#ifndef _INPUT_COMMAND_CORE_H
#define _INPUT_COMMAND_CORE_H


#include <memory>


#include <Eigen/Dense>


#include "msf_localization_core/input_core.h"


enum class InputCommandTypes
{
    undefined=0,
    imu=1,
};


class InputCommandCore
{
public:
    InputCommandCore();
    InputCommandCore(std::weak_ptr<InputCore> input_core_ptr);
    ~InputCommandCore();

protected:
    int init();

protected:
    // It is not the owner of this Pointer. it doesn't modify the pointer
    std::weak_ptr<InputCore> input_core_ptr_;
public:
    int setInputCorePtr(std::weak_ptr<InputCore> input_core_ptr);
    std::weak_ptr<InputCore> getInputCoreWeakPtr() const;
    std::shared_ptr<InputCore> getInputCoreSharedPtr() const;


protected:
    InputCommandTypes input_command_type_;
public:
    int setInputCommandType(InputCommandTypes input_command_type);
    InputCommandTypes getInputCommandType() const;


/*
    /// Jacobians Measurement

public:
    // Jacobian Error Measurement - Error State
    struct
    {
        Eigen::MatrixXd jacobianMeasurementRobotErrorState;
        Eigen::MatrixXd jacobianMeasurementGlobalParametersErrorState;
        Eigen::MatrixXd jacobianMeasurementSensorErrorState;
        Eigen::MatrixXd jacobianMeasurementMapElementErrorState; // TODO
    } jacobianMeasurementErrorState;


    // Jacobian Error Measurement - Error Parameters
    struct
    {
        // TODO Robot Parameters
        Eigen::MatrixXd jacobianMeasurementGlobalParameters;
        Eigen::MatrixXd jacobianMeasurementSensorParameters;
        Eigen::MatrixXd jacobianMeasurementMapElementParameters;
    } jacobianMeasurementErrorParameters;



    // Jacobian Error Measurement - Sensor Noise
    struct
    {
        Eigen::MatrixXd jacobianMeasurementSensorNoise;

    } jacobianMeasurementSensorNoise;



*/


//    //// Debug log
//protected:
//    std::string logPath;
//    std::ofstream logFile;
//    // mutex to protect the log file
//protected:
//    std::mutex TheLogFileMutex;
//public:
//    int log(std::string logString);

};



#endif
