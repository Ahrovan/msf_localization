
#include "msf_localization_core/imu_input_core.h"

// Circular Dependency
#include "msf_localization_core/msf_storage_core.h"


ImuInputCore::ImuInputCore() :
    InputCore()
{
    init();

    return;
}

ImuInputCore::ImuInputCore(std::weak_ptr<MsfStorageCore> the_msf_storage_core) :
    InputCore(the_msf_storage_core)
{
    init();

    return;
}

ImuInputCore::~ImuInputCore()
{

    return;
}

int ImuInputCore::init()
{
    // type
    setInputType(InputTypes::imu);

    // name -> default
    setInputName("imu_input");

    // dimensions
    dimension_state_=0;
    dimension_error_state_=0;
    dimension_parameters_=6;
    dimension_error_parameters_=6;
    dimension_noise_=0;
    dimension_input_command_=0;
    dimension_error_input_command_=0;

    // flags

    // flags inputs
    flag_input_command_orientation_=false;
    flag_input_command_angular_velocity_=false;
    flag_input_command_linear_acceleration_=false;

    // flags state and parameters
    flag_estimation_position_input_wrt_robot_=false;
    flag_estimation_attitude_input_wrt_robot_=false;
    flag_estimation_bias_linear_acceleration_=false;
    flag_estimation_sensitivity_linear_acceleration_=false;
    flag_estimation_bias_angular_velocity_=false;
    flag_estimation_sensitivity_angular_velocity_=false;

    // noises

    // noises inputs
    noise_input_command_linear_acceleration_.setZero();
    noise_input_command_angular_velocity_.setZero();

    // noises state and parameters
    noise_position_input_wrt_robot_.setZero();
    noise_attitude_input_wrt_robot_.setZero();
    noise_bias_linear_acceleration_.setZero();
    noise_sensitivity_linear_acceleration_.setZero();
    noise_bias_angular_velocity_.setZero();
    noise_sensitivity_angular_velocity_.setZero();

    // noises estimation
    noise_estimation_bias_linear_acceleration_.setZero();
    noise_estimation_bias_angular_velocity_.setZero();

    // end
    return 0;
}

int ImuInputCore::readConfig(pugi::xml_node input, std::shared_ptr<ImuInputStateCore>& init_state_core)
{
    // Create a class for the SensorStateCore
    if(!init_state_core)
        init_state_core=std::make_shared<ImuInputStateCore>(this->getMsfElementCoreWeakPtr());


    // Auxiliar reading value
    std::string readingValue;


    // Name
    readingValue=input.child_value("name");
    this->setInputName(readingValue);


    //// Input configurations
    pugi::xml_node parameters=input.child("parameters");


    /// Pose of the input wrt robot
    pugi::xml_node pose_in_robot=parameters.child("pose_in_robot");

    // Position of the sensor wrt robot
    readingValue=pose_in_robot.child("position").child_value("enabled");
    if(std::stoi(readingValue))
        this->enableEstimationPositionInputWrtRobot();

    // Attitude of the sensor wrt robot
    readingValue=pose_in_robot.child("attitude").child_value("enabled");
    if(std::stoi(readingValue))
        this->enableEstimationAttitudeInputWrtRobot();


    /// Other Parameters

    // Angular Velocity
    pugi::xml_node param_angular_velocity = parameters.child("angular_velocity");

    // Angular Velocity Biases
    readingValue=param_angular_velocity.child("biases").child_value("enabled");
    if(std::stoi(readingValue))
    {
        this->enableEstimationBiasAngularVelocity();
    }

    // Angular Velocity Sensitivity
    readingValue=param_angular_velocity.child("sensitivity").child_value("enabled");
    if(std::stoi(readingValue))
    {
        this->enableEstimationSensitivityAngularVelocity();
    }


    // Linear Acceleration
    pugi::xml_node param_linear_acceleration = parameters.child("linear_acceleration");

    // Linear Acceleration Biases
    readingValue=param_linear_acceleration.child("biases").child_value("enabled");
    if(std::stoi(readingValue))
        this->enableEstimationBiasLinearAcceleration();

    // Linear Acceleration Sensitivity
    readingValue=param_linear_acceleration.child("sensitivity").child_value("enabled");
    if(std::stoi(readingValue))
        this->enableEstimationSensitivityLinearAcceleration();




    //// Input Commands
    pugi::xml_node commands = input.child("commands");


    /// Linear Acceleration
    pugi::xml_node command_linear_acceleration = commands.child("linear_acceleration");

    readingValue=command_linear_acceleration.child_value("enabled");
    if(std::stoi(readingValue))
        this->enableInputCommandLinearAcceleration();

    readingValue=command_linear_acceleration.child_value("var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoiseInputCommandLinearAcceleration(variance.asDiagonal());
    }


    /// Orientation
    pugi::xml_node command_orientation = commands.child("orientation");

    readingValue=command_orientation.child_value("enabled");
    if(std::stoi(readingValue))
        this->enableInputCommandOrientation();

    readingValue=command_orientation.child_value("var");
    // TODO


    /// Angular Velocity
    pugi::xml_node command_angular_velocity = commands.child("angular_velocity");

    readingValue=command_angular_velocity.child_value("enabled");
    if(std::stoi(readingValue))
        this->enableInputCommandAngularVelocity();

    readingValue=command_angular_velocity.child_value("var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoiseInputCommandAngularVelocity(variance.asDiagonal());
    }






    //// Init State

    /// Pose of the sensor wrt robot

    // Position of the sensor wrt robot
    readingValue=pose_in_robot.child("position").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d init_estimation;
        stm>>init_estimation[0]>>init_estimation[1]>>init_estimation[2];
        init_state_core->setPositionInputWrtRobot(init_estimation);
    }

    // Attitude of the sensor wrt robot
    readingValue=pose_in_robot.child("attitude").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector4d init_estimation;
        stm>>init_estimation[0]>>init_estimation[1]>>init_estimation[2]>>init_estimation[3];
        init_state_core->setAttitudeInputWrtRobot(init_estimation);
    }


    /// Parameters

    // Bias Angular Velocity
    readingValue=param_angular_velocity.child("biases").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d init_estimation;
        stm>>init_estimation[0]>>init_estimation[1]>>init_estimation[2];
        init_state_core->setBiasesAngularVelocity(init_estimation);
    }

    // Sensitivity Angular Velocity
    readingValue=param_angular_velocity.child("sensitivity").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Matrix3d init_estimation;
        init_estimation.setZero();
        stm>>init_estimation(0,0)>>init_estimation(0,1)>>init_estimation(0,2)
                >>init_estimation(1,0)>>init_estimation(1,1)>>init_estimation(1,2)
                >>init_estimation(2,0)>>init_estimation(2,1)>>init_estimation(2,2);
        init_state_core->setSensitivityAngularVelocity(init_estimation);
    }

    // Bias Linear Acceleration
    readingValue=param_linear_acceleration.child("biases").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d init_estimation;
        stm>>init_estimation[0]>>init_estimation[1]>>init_estimation[2];
        init_state_core->setBiasesLinearAcceleration(init_estimation);
    }

    // Scale Linear Acceleration
    readingValue=param_linear_acceleration.child("sensitivity").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Matrix3d init_estimation;
        init_estimation.setZero();
        stm>>init_estimation(0,0)>>init_estimation(0,1)>>init_estimation(0,2)
                >>init_estimation(1,0)>>init_estimation(1,1)>>init_estimation(1,2)
                >>init_estimation(2,0)>>init_estimation(2,1)>>init_estimation(2,2);
        init_state_core->setSensitivityLinearAcceleration(init_estimation);
    }




    //// Init Variances


    /// Pose of the sensor wrt robot

    // Position of the sensor wrt robot
    readingValue=pose_in_robot.child("position").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoisePositionInputWrtRobot(variance.asDiagonal());
    }


    // Attitude of the sensor wrt robot
    readingValue=pose_in_robot.child("attitude").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoiseAttitudeInputWrtRobot(variance.asDiagonal());
    }



    /// Other Parameters

    // Bias Linear Acceleration
    readingValue=param_linear_acceleration.child("biases").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoiseBiasLinearAcceleration(variance.asDiagonal());
    }

    // Sensitivity Linear Acceleration
    // TODO
    /*
    readingValue=param_linear_acceleration.child("sensitivity").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoiseSensitivityLinearAcceleration(variance.asDiagonal());
    }
    */

    // Bias Angular Velocity
    readingValue=param_angular_velocity.child("biases").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoiseBiasAngularVelocity(variance.asDiagonal());
    }

    // Sensitivity Angular Velocity
    // TODO
    /*
    readingValue=param_angular_velocity.child("sensitivity").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoiseSensitivityAngularVelocity(variance.asDiagonal());
    }
    */


    // Noises in the estimation (if enabled)

    // Bias Linear Acceleration
    if(this->isEstimationBiasLinearAccelerationEnabled())
    {
        readingValue=param_linear_acceleration.child("biases").child_value("noise");
        {
            std::istringstream stm(readingValue);
            Eigen::Vector3d variance;
            stm>>variance[0]>>variance[1]>>variance[2];
            this->setNoiseEstimationBiasLinearAcceleration(variance.asDiagonal());
        }
    }

    // Bias Angular Velocity
    if(this->isEstimationBiasAngularVelocityEnabled())
    {
        readingValue=param_angular_velocity.child("biases").child_value("noise");
        {
            std::istringstream stm(readingValue);
            Eigen::Vector3d variance;
            stm>>variance[0]>>variance[1]>>variance[2];
            this->setNoiseEstimationBiasAngularVelocity(variance.asDiagonal());
        }
    }


    // Prepare covariance matrix
    this->prepareCovarianceInitErrorState();



    /// Finish

    // End
    return 0;
}


bool ImuInputCore::isInputCommandOrientationEnabled() const
{
    return this->flag_input_command_orientation_;
}

int ImuInputCore::enableInputCommandOrientation()
{
    if(!flag_input_command_orientation_)
    {
        flag_input_command_orientation_=true;
        this->dimension_input_command_+=4;
        this->dimension_error_input_command_+=3;
    }
    return 0;
}

bool ImuInputCore::isInputCommandAngularVelocityEnabled() const
{
    return this->flag_input_command_angular_velocity_;
}

int ImuInputCore::enableInputCommandAngularVelocity()
{
    if(!flag_input_command_angular_velocity_)
    {
        flag_input_command_angular_velocity_=true;
        dimension_input_command_+=3;
        dimension_error_input_command_+=3;
    }
    return 0;
}

Eigen::Matrix3d ImuInputCore::getNoiseInputCommandAngularVelocity() const
{
    return this->noise_input_command_angular_velocity_;
}

int ImuInputCore::setNoiseInputCommandAngularVelocity(Eigen::Matrix3d noise_input_command_angular_velocity)
{
    this->noise_input_command_angular_velocity_=noise_input_command_angular_velocity;
    return 0;
}

bool ImuInputCore::isInputCommandLinearAccelerationEnabled() const
{
    return this->flag_input_command_linear_acceleration_;
}

int ImuInputCore::enableInputCommandLinearAcceleration()
{
    if(!flag_input_command_linear_acceleration_)
    {
        flag_input_command_linear_acceleration_=true;
        dimension_input_command_+=3;
        dimension_error_input_command_+=3;
    }
    return 0;
}

Eigen::Matrix3d ImuInputCore::getNoiseInputCommandLinearAcceleration() const
{
    return this->noise_input_command_linear_acceleration_;
}

int ImuInputCore::setNoiseInputCommandLinearAcceleration(Eigen::Matrix3d noise_input_command_linear_acceleration)
{
    this->noise_input_command_linear_acceleration_=noise_input_command_linear_acceleration;
    return 0;
}

int ImuInputCore::setInputCommand(const TimeStamp time_stamp, std::shared_ptr<ImuInputCommandCore> imu_input_command_core)
{
    if(!this->isInputEnabled())
        return 0;

    if(!this->isCorrect())
    {
        std::cout<<"ERROR"<<std::endl;
        return -1;
    }

    if(this->getMsfStorageCoreSharedPtr()->setInputCommand(time_stamp, imu_input_command_core))
        return 1;

    return 0;
}

bool ImuInputCore::isEstimationPositionInputWrtRobotEnabled() const
{
    return flag_estimation_position_input_wrt_robot_;
}

int ImuInputCore::enableEstimationPositionInputWrtRobot()
{
    if(!this->flag_estimation_position_input_wrt_robot_)
    {
        // Enable
        this->flag_estimation_position_input_wrt_robot_=true;
        // Update State Dimension
        this->dimension_state_+=3;
        // Update Error State Dimension
        this->dimension_error_state_+=3;
        //
        this->dimension_parameters_-=3;
        //
        this->dimension_error_parameters_-=3;
        //
        this->dimension_noise_+=0;
    }
    return 0;
}

int ImuInputCore::enableParameterPositionInputWrtRobot()
{
    if(this->flag_estimation_position_input_wrt_robot_)
    {
        // Enable
        this->flag_estimation_position_input_wrt_robot_=false;
        // Update State Dimension
        this->dimension_state_-=3;
        // Update Error State Dimension
        this->dimension_error_state_-=3;
        //
        this->dimension_parameters_+=3;
        //
        this->dimension_error_parameters_+=3;
        //
        this->dimension_noise_-=0;
    }
    return 0;
}

Eigen::Matrix3d ImuInputCore::getNoisePositionInputWrtRobot() const
{
    return this->noise_position_input_wrt_robot_;
}

int ImuInputCore::setNoisePositionInputWrtRobot(Eigen::Matrix3d noise_position_input_wrt_robot)
{
    this->noise_position_input_wrt_robot_=noise_position_input_wrt_robot;
    return 0;
}

bool ImuInputCore::isEstimationAttitudeInputWrtRobotEnabled() const
{
    return flag_estimation_attitude_input_wrt_robot_;
}

int ImuInputCore::enableEstimationAttitudeInputWrtRobot()
{
    if(!this->flag_estimation_attitude_input_wrt_robot_)
    {
        // Enable
        this->flag_estimation_attitude_input_wrt_robot_=true;
        // Update State Dimension
        this->dimension_state_+=4;
        // Update Error State Dimension
        this->dimension_error_state_+=3;
        //
        this->dimension_parameters_-=4;
        //
        this->dimension_error_parameters_-=3;
        //
        this->dimension_noise_+=0;
    }
    return 0;
}

int ImuInputCore::enableParameterAttitudeInputWrtRobot()
{
    if(this->flag_estimation_attitude_input_wrt_robot_)
    {
        // Enable
        this->flag_estimation_attitude_input_wrt_robot_=false;
        // Update State Dimension
        this->dimension_state_-=4;
        // Update Error State Dimension
        this->dimension_error_state_-=3;
        //
        this->dimension_parameters_+=4;
        //
        this->dimension_error_parameters_+=3;
        //
        this->dimension_noise_-=0;
    }
    return 0;
}

Eigen::Matrix3d ImuInputCore::getNoiseAttitudeInputWrtRobot() const
{
    return this->noise_attitude_input_wrt_robot_;
}

int ImuInputCore::setNoiseAttitudeInputWrtRobot(Eigen::Matrix3d noise_attitude_input_wrt_robot)
{
    this->noise_attitude_input_wrt_robot_=noise_attitude_input_wrt_robot;
    return 0;
}

bool ImuInputCore::isEstimationBiasLinearAccelerationEnabled() const
{
    return this->flag_estimation_bias_linear_acceleration_;
}

int ImuInputCore::enableEstimationBiasLinearAcceleration()
{
    if(!this->flag_estimation_bias_linear_acceleration_)
    {
        // Enable
        this->flag_estimation_bias_linear_acceleration_=true;
        // Update State Dimension
        this->dimension_state_+=3;
        // Update Error State Dimension
        this->dimension_error_state_+=3;
        //
        this->dimension_parameters_-=3;
        //
        this->dimension_error_parameters_-=3;
        //
        this->dimension_noise_+=3;
    }
    return 0;
}

int ImuInputCore::enableParameterBiasLinearAcceleration()
{
    if(this->flag_estimation_bias_linear_acceleration_)
    {
        // Enable
        this->flag_estimation_bias_linear_acceleration_=false;
        // Update State Dimension
        this->dimension_state_-=3;
        // Update Error State Dimension
        this->dimension_error_state_-=3;
        //
        this->dimension_parameters_+=3;
        //
        this->dimension_error_parameters_+=3;
        //
        this->dimension_noise_-=3;
    }
    return 0;
}

Eigen::Matrix3d ImuInputCore::getNoiseBiasLinearAcceleration() const
{
    return this->noise_bias_linear_acceleration_;
}

int ImuInputCore::setNoiseBiasLinearAcceleration(Eigen::Matrix3d noise_bias_linear_acceleration)
{
    this->noise_bias_linear_acceleration_=noise_bias_linear_acceleration;
    return 0;
}

Eigen::Matrix3d ImuInputCore::getNoiseEstimationBiasLinearAcceleration() const
{
    return this->noise_estimation_bias_linear_acceleration_;
}

int ImuInputCore::setNoiseEstimationBiasLinearAcceleration(Eigen::Matrix3d noise_estimation_bias_linear_acceleration)
{
    this->noise_estimation_bias_linear_acceleration_=noise_estimation_bias_linear_acceleration;
    return 0;
}

bool ImuInputCore::isEstimationBiasAngularVelocityEnabled() const
{
    return this->flag_estimation_bias_angular_velocity_;
}

int ImuInputCore::enableEstimationBiasAngularVelocity()
{
    if(!this->flag_estimation_bias_angular_velocity_)
    {
        // Enable
        this->flag_estimation_bias_angular_velocity_=true;
        // Update State Dimension
        this->dimension_state_+=3;
        // Update Error State Dimension
        this->dimension_error_state_+=3;
        //
        this->dimension_parameters_-=3;
        //
        this->dimension_error_parameters_-=3;
        //
        this->dimension_noise_+=3;
    }
    return 0;
}

int ImuInputCore::enableParameterBiasAngularVelocity()
{
    if(this->flag_estimation_bias_angular_velocity_)
    {
        // Enable
        this->flag_estimation_bias_angular_velocity_=false;
        // Update State Dimension
        this->dimension_state_-=3;
        // Update Error State Dimension
        this->dimension_error_state_-=3;
        //
        this->dimension_parameters_+=3;
        //
        this->dimension_error_parameters_+=3;
        //
        this->dimension_noise_-=3;
    }
    return 0;
}

Eigen::Matrix3d ImuInputCore::getNoiseBiasAngularVelocity() const
{
    return this->noise_bias_angular_velocity_;
}

int ImuInputCore::setNoiseBiasAngularVelocity(Eigen::Matrix3d noise_bias_angular_velocity)
{
    this->noise_bias_angular_velocity_=noise_bias_angular_velocity;
    return 0;
}

Eigen::Matrix3d ImuInputCore::getNoiseEstimationBiasAngularVelocity() const
{
    return this->noise_estimation_bias_angular_velocity_;
}

int ImuInputCore::setNoiseEstimationBiasAngularVelocity(Eigen::Matrix3d noise_estimation_bias_angular_velocity)
{
    this->noise_estimation_bias_angular_velocity_=noise_estimation_bias_angular_velocity;
    return 0;
}

bool ImuInputCore::isEstimationSensitivityLinearAccelerationEnabled() const
{
    return this->flag_estimation_sensitivity_linear_acceleration_;
}

int ImuInputCore::enableEstimationSensitivityLinearAcceleration()
{
    return -1; // TODO
    if(!this->flag_estimation_sensitivity_linear_acceleration_)
    {
        // Enable
        this->flag_estimation_sensitivity_linear_acceleration_=true;
        // Update State Dimension
        this->dimension_state_+=9;
        // Update Error State Dimension
        this->dimension_error_state_+=9;
        //
        this->dimension_parameters_-=9;
        //
        this->dimension_error_parameters_-=9;
        //
        this->dimension_noise_+=0;
    }
    return 0;
}

int ImuInputCore::enableParameterSensitivityLinearAcceleration()
{
    return -1; //TODO
    if(this->flag_estimation_sensitivity_linear_acceleration_)
    {
        // Enable
        this->flag_estimation_sensitivity_linear_acceleration_=false;
        // Update State Dimension
        this->dimension_state_-=9;
        // Update Error State Dimension
        this->dimension_error_state_-=9;
        //
        this->dimension_parameters_+=9;
        //
        this->dimension_error_parameters_+=9;
        //
        this->dimension_noise_-=0;
    }
    return 0;
}

Eigen::MatrixXd ImuInputCore::getNoiseSensitivityLinearAcceleration() const
{
    return this->noise_sensitivity_linear_acceleration_;
}

int ImuInputCore::setNoiseSensitivityLinearAcceleration(Eigen::Matrix3d noise_sensitivity_linear_acceleration)
{
    this->noise_sensitivity_linear_acceleration_=noise_sensitivity_linear_acceleration;
    return 0;
}

bool ImuInputCore::isEstimationSensitivityAngularVelocityEnabled() const
{
    return this->flag_estimation_sensitivity_angular_velocity_;
}

int ImuInputCore::enableEstimationSensitivityAngularVelocity()
{
    return -1; // TODO
    if(!this->flag_estimation_sensitivity_angular_velocity_)
    {
        // Enable
        this->flag_estimation_sensitivity_angular_velocity_=true;
        // Update State Dimension
        this->dimension_state_+=9;
        // Update Error State Dimension
        this->dimension_error_state_+=9;
        //
        this->dimension_parameters_-=9;
        //
        this->dimension_error_parameters_-=9;
        //
        this->dimension_noise_+=0;
    }
    return 0;
}

int ImuInputCore::enableParameterSensitivityAngularVelocity()
{
    return -1; // TODO
    if(this->flag_estimation_sensitivity_angular_velocity_)
    {
        // Enable
        this->flag_estimation_sensitivity_angular_velocity_=false;
        // Update State Dimension
        this->dimension_state_-=9;
        // Update Error State Dimension
        this->dimension_error_state_-=9;
        //
        this->dimension_parameters_+=9;
        //
        this->dimension_error_parameters_+=9;
        //
        this->dimension_noise_-=0;
    }
    return 0;
}

Eigen::Matrix3d ImuInputCore::getNoiseSensitivityAngularVelocity() const
{
    return this->noise_sensitivity_angular_velocity_;
}

int ImuInputCore::setNoiseSensitivityAngularVelocity(Eigen::Matrix3d noise_sensitivity_angular_velocity)
{
    this->noise_sensitivity_angular_velocity_=noise_sensitivity_angular_velocity;
    return 0;
}

int ImuInputCore::prepareCovarianceInitErrorState()
{
    int error=MsfElementCore::prepareCovarianceInitErrorState();

    if(error)
        return error;


    int point=0;
    if(this->isEstimationPositionInputWrtRobotEnabled())
    {
        this->InitErrorStateVariance.block<3,3>(point,point)=noise_position_input_wrt_robot_;
        point+=3;
    }
    if(this->isEstimationAttitudeInputWrtRobotEnabled())
    {
        this->InitErrorStateVariance.block<3,3>(point,point)=noise_attitude_input_wrt_robot_;
        point+=3;
    }
    if(this->isEstimationBiasLinearAccelerationEnabled())
    {
        this->InitErrorStateVariance.block<3,3>(point,point)=noise_bias_linear_acceleration_;
        point+=3;
    }
    if(this->isEstimationBiasAngularVelocityEnabled())
    {
        this->InitErrorStateVariance.block<3,3>(point,point)=noise_bias_angular_velocity_;
        point+=3;
    }
    // TODO
    // Add Sa
    // TODO
    // Add Sw

    // End
    return 0;
}
