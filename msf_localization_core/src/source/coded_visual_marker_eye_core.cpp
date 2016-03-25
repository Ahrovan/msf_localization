
#include "msf_localization_core/coded_visual_marker_eye_core.h"

// Circular Dependency
#include "msf_localization_core/msf_storage_core.h"


CodedVisualMarkerEyeCore::CodedVisualMarkerEyeCore() :
    SensorCore()
{
    //
    init();

    // End
    return;
}

CodedVisualMarkerEyeCore::CodedVisualMarkerEyeCore(std::weak_ptr<SensorCore> the_sensor_core, std::weak_ptr<MsfStorageCore> the_msf_storage_core) :
    SensorCore(the_sensor_core, the_msf_storage_core)
{
    //
    init();

    return;
}

CodedVisualMarkerEyeCore::~CodedVisualMarkerEyeCore()
{

    return;
}

int CodedVisualMarkerEyeCore::init()
{
    // Sensor Type
    setSensorType(SensorTypes::coded_visual_marker_eye);


    // Sensor name -> Default
    sensor_name_="coded_visual_marker_eye";

    // Flags measurement
    flag_measurement_position_=false;
    flag_measurement_attitude_=false;

    // Flags estimation -> By default are considered parameters
    // none

    // State -> Again just in case
    dimensionErrorState=0;
    dimensionState=0;

    // Parameters
    dimensionParameters+=0;
    dimensionErrorParameters+=0;

    // Dimension measurements -> Again just in case
    dimensionMeasurement=0;

    // Dimension noise -> Again just in case
    dimensionNoise=0;

    // Noises measurements
    noise_measurement_position_.setZero();
    noise_measurement_attitude_.setZero();

    // Noises parameters
    // None

    // Noises estimation
    // None

    return 0;
}

bool CodedVisualMarkerEyeCore::isMeasurementPositionEnabled() const
{
    return this->flag_measurement_position_;
}

int CodedVisualMarkerEyeCore::enableMeasurementPosition()
{
    if(!this->flag_measurement_position_)
    {
        this->flag_measurement_position_=true;
        this->dimensionMeasurement+=3;
    }
    return 0;
}

Eigen::Matrix3d CodedVisualMarkerEyeCore::getNoiseMeasurementPosition() const
{
    return this->noise_measurement_position_;
}

int CodedVisualMarkerEyeCore::setNoiseMeasurementPosition(Eigen::Matrix3d noise_measurement_position)
{
    this->noise_measurement_position_=noise_measurement_position;
    return 0;
}

bool CodedVisualMarkerEyeCore::isMeasurementAttitudeEnabled() const
{
    return this->flag_measurement_attitude_;
}

int CodedVisualMarkerEyeCore::enableMeasurementAttitude()
{
    if(!this->flag_measurement_attitude_)
    {
        this->flag_measurement_attitude_=true;
        this->dimensionMeasurement+=4;
    }
    return 0;
}

Eigen::Matrix3d CodedVisualMarkerEyeCore::getNoiseMeasurementAttitude() const
{
    return this->noise_measurement_attitude_;
}

int CodedVisualMarkerEyeCore::setNoiseMeasurementAttitude(Eigen::Matrix3d noise_measurement_attitude)
{
    this->noise_measurement_attitude_=noise_measurement_attitude;
    return 0;
}

int CodedVisualMarkerEyeCore::setMeasurement(const TimeStamp the_time_stamp, std::shared_ptr<CodedVisualMarkerMeasurementCore> the_visual_marker_measurement)
{
    if(!isSensorEnabled())
        return 0;

    if(this->getTheMsfStorageCore()->setMeasurement(the_time_stamp, the_visual_marker_measurement))
    {
        std::cout<<"CodedVisualMarkerEyeCore::setMeasurement() error"<<std::endl;
        return 1;
    }

    return 0;
}

Eigen::SparseMatrix<double> CodedVisualMarkerEyeCore::getCovarianceMeasurement()
{
    Eigen::SparseMatrix<double> covariances_matrix;

    covariances_matrix.resize(this->getDimensionMeasurement(), this->getDimensionMeasurement());
    //covariances_matrix.setZero();
    covariances_matrix.reserve(this->getDimensionMeasurement());

    std::vector<Eigen::Triplet<double> > tripletCovarianceMeasurement;

    unsigned int dimension=0;
    if(this->isMeasurementPositionEnabled())
    {
        //covariances_matrix.block<3,3>(dimension, dimension)=this->getNoiseMeasurementPosition();

        for(int i=0; i<3; i++)
            tripletCovarianceMeasurement.push_back(Eigen::Triplet<double>(dimension+i,dimension+i,noise_measurement_position_(i,i)));


        dimension+=3;
    }
    if(this->isMeasurementAttitudeEnabled())
    {
        //covariances_matrix.block<3,3>(dimension, dimension)=this->getNoiseMeasurementAttitude();

        for(int i=0; i<3; i++)
            tripletCovarianceMeasurement.push_back(Eigen::Triplet<double>(dimension+i,dimension+i,noise_measurement_attitude_(i,i)));


        dimension+=3;
    }

    covariances_matrix.setFromTriplets(tripletCovarianceMeasurement.begin(), tripletCovarianceMeasurement.end());

    return covariances_matrix;
}

Eigen::SparseMatrix<double> CodedVisualMarkerEyeCore::getCovarianceParameters()
{
    Eigen::SparseMatrix<double> covariances_matrix;
    covariances_matrix.resize(this->getDimensionErrorParameters(), this->getDimensionErrorParameters());
    //covariances_matrix.setZero();
    covariances_matrix.reserve(this->getDimensionErrorParameters());

    std::vector<Eigen::Triplet<double> > tripletCovarianceParameters;

    unsigned int dimension=0;
    if(!this->isEstimationPositionSensorWrtRobotEnabled())
    {
        //covariances_matrix.block<3,3>(dimension, dimension)=this->getNoisePositionSensorWrtRobot();

        for(int i=0; i<3; i++)
            tripletCovarianceParameters.push_back(Eigen::Triplet<double>(dimension+i,dimension+i,noisePositionSensorWrtRobot(i,i)));

        dimension+=3;
    }
    if(!this->isEstimationAttitudeSensorWrtRobotEnabled())
    {
        //covariances_matrix.block<3,3>(dimension, dimension)=this->getNoiseAttitudeSensorWrtRobot();

        for(int i=0; i<3; i++)
            tripletCovarianceParameters.push_back(Eigen::Triplet<double>(dimension+i,dimension+i,noiseAttitudeSensorWrtRobot(i,i)));

        dimension+=3;
    }

    covariances_matrix.setFromTriplets(tripletCovarianceParameters.begin(), tripletCovarianceParameters.end());


    return covariances_matrix;
}

int CodedVisualMarkerEyeCore::prepareInitErrorStateVariance()
{
    int error=SensorCore::prepareInitErrorStateVariance();

    if(error)
        return error;


    int point=0;
    if(this->isEstimationPositionSensorWrtRobotEnabled())
    {
        this->InitErrorStateVariance.block<3,3>(point,point)=this->getNoisePositionSensorWrtRobot();
        point+=3;
    }
    if(this->isEstimationAttitudeSensorWrtRobotEnabled())
    {
        this->InitErrorStateVariance.block<3,3>(point,point)=this->getNoiseAttitudeSensorWrtRobot();
        point+=3;
    }



    return 0;
}

Eigen::SparseMatrix<double> CodedVisualMarkerEyeCore::getCovarianceNoise(const TimeStamp deltaTimeStamp) const
{
    Eigen::SparseMatrix<double> covariance_noise;

    // Dimension noise
    int dimension_noise=getDimensionNoise();


    // Resize
    covariance_noise.resize(dimension_noise, dimension_noise);
    //covariance_noise.setZero();

    // Fill
    int dimension_noise_i=0;

    // Nothing


    // End
    return covariance_noise;
}

int CodedVisualMarkerEyeCore::predictState(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, const std::shared_ptr<SensorStateCore> pastStateI, std::shared_ptr<SensorStateCore>& predictedStateI)
{

    // Poly
    std::shared_ptr<CodedVisualMarkerEyeStateCore> pastState=std::static_pointer_cast<CodedVisualMarkerEyeStateCore>(pastStateI);
    std::shared_ptr<CodedVisualMarkerEyeStateCore> predictedState=std::static_pointer_cast<CodedVisualMarkerEyeStateCore>(predictedStateI);



    // Checks in the past state
    if(pastState->getTheSensorCoreWeak().expired())
    {
        return -5;
        std::cout<<"FreeModelRobotCore::predictState() error !pastState->getTheRobotCore()"<<std::endl;
    }


    // Create the predicted state if it doesn't exist
    if(!predictedState)
    {
        predictedState=std::make_shared<CodedVisualMarkerEyeStateCore>(pastState->getTheSensorCoreWeak());
    }

    // Set The robot core if it doesn't exist
    if(predictedState->getTheSensorCoreWeak().expired())
    {
        predictedState->setTheSensorCore(pastState->getTheSensorCoreWeak());
    }


    // Equations


    //Delta Time
    TimeStamp DeltaTime=currentTimeStamp-previousTimeStamp;
    double dt=DeltaTime.get_double();


    /// Position
    if(this->isEstimationPositionSensorWrtRobotEnabled())
    {
        // Estimation
        predictedState->positionSensorWrtRobot=pastState->positionSensorWrtRobot;
    }
    else
    {
        // Parameter
        predictedState->positionSensorWrtRobot=pastState->positionSensorWrtRobot;
    }


    /// Attitude
    if(this->isEstimationAttitudeSensorWrtRobotEnabled())
    {
        predictedState->attitudeSensorWrtRobot=pastState->attitudeSensorWrtRobot;
    }
    else
    {
        predictedState->attitudeSensorWrtRobot=pastState->attitudeSensorWrtRobot;
    }


    predictedStateI=predictedState;

    return 0;
}

// Jacobian
int CodedVisualMarkerEyeCore::predictStateErrorStateJacobians(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, std::shared_ptr<SensorStateCore> pastStateI, std::shared_ptr<SensorStateCore>& predictedStateI)
{

    // Poly
    std::shared_ptr<CodedVisualMarkerEyeStateCore> pastState=std::static_pointer_cast<CodedVisualMarkerEyeStateCore>(pastStateI);
    std::shared_ptr<CodedVisualMarkerEyeStateCore> predictedState=std::static_pointer_cast<CodedVisualMarkerEyeStateCore>(predictedStateI);


    // Create the predicted state if it doesn't exist
    if(!predictedState)
    {
        return 1;
    }


    //Delta Time
    TimeStamp DeltaTime=currentTimeStamp-previousTimeStamp;
    // delta time
    double dt=DeltaTime.get_double();


    ///// Jacobian Error State

    /// Jacobian of the error: Linear Part

    // posi / posi
    if(this->isEstimationPositionSensorWrtRobotEnabled())
    {
        predictedState->error_state_jacobian_.position_sensor_wrt_robot_.resize(3, 3);
        predictedState->error_state_jacobian_.position_sensor_wrt_robot_.setZero();

        predictedState->error_state_jacobian_.position_sensor_wrt_robot_.block<3,3>(0,0)=Eigen::MatrixXd::Identity(3,3);
    }


    /// Jacobian of the error -> Angular Part

    // att / att
    if(this->isEstimationAttitudeSensorWrtRobotEnabled())
    {
        predictedState->error_state_jacobian_.attitude_sensor_wrt_robot_.resize(3, 3);
        predictedState->error_state_jacobian_.attitude_sensor_wrt_robot_.setZero();

        predictedState->error_state_jacobian_.attitude_sensor_wrt_robot_.block<3,3>(0,0)=Eigen::MatrixXd::Identity(3,3);
    }


    //// Jacobian Error State Noise

    // Resize the jacobian
    predictedState->jacobian_error_state_noise_.resize(getDimensionErrorState(), getDimensionNoise());

    // Fill

    // Nothing to do



    //// Finish
    predictedStateI=predictedState;

    // End
    return 0;
}

int CodedVisualMarkerEyeCore::predictMeasurement(const TimeStamp theTimeStamp, std::shared_ptr<GlobalParametersStateCore> currentGlobalParametersState, const std::shared_ptr<RobotStateCore> currentRobotState, const std::shared_ptr<CodedVisualMarkerEyeStateCore> currentSensorState, std::shared_ptr<CodedVisualMarkerMeasurementCore>& predictedMeasurement)
{
    // TODO

    return 0;
}

int CodedVisualMarkerEyeCore::jacobiansMeasurements(const TimeStamp theTimeStamp, std::shared_ptr<GlobalParametersStateCore> currentGlobalParametersState, std::shared_ptr<RobotStateCore> currentRobotState, std::shared_ptr<CodedVisualMarkerEyeStateCore> currentSensorState, std::shared_ptr<CodedVisualMarkerMeasurementCore>& predictedMeasurement)
{
    // TODO


    // sensor core
    std::shared_ptr<const CodedVisualMarkerEyeCore> the_sensor_core=std::dynamic_pointer_cast<const CodedVisualMarkerEyeCore>(currentSensorState->getTheSensorCoreShared());


    // dimension of the measurement
    int dimension_measurement=the_sensor_core->getTheSensorCore()->getDimensionMeasurement();


    // Robot
    std::shared_ptr<FreeModelRobotStateCore> TheRobotStateCoreAux=std::static_pointer_cast<FreeModelRobotStateCore>(currentRobotState);
    std::shared_ptr<const FreeModelRobotCore> TheRobotCoreAux=std::static_pointer_cast<const FreeModelRobotCore>(TheRobotStateCoreAux->getTheRobotCore());


// TODO
/*
    /// Jacobians measurement - state

    /// Jacobian measurement - robot state
    switch(TheRobotStateCore->getTheRobotCore()->getRobotType())
    {
        case RobotTypes::free_model:
        {
            // Robot
            std::shared_ptr<FreeModelRobotStateCore> TheRobotStateCoreAux=std::static_pointer_cast<FreeModelRobotStateCore>(TheRobotStateCore);
            std::shared_ptr<const FreeModelRobotCore> TheRobotCoreAux=std::static_pointer_cast<const FreeModelRobotCore>(TheRobotStateCoreAux->getTheRobotCore());

            // dimension of robot state
            int dimensionRobotErrorState=TheRobotCoreAux->getDimensionErrorState();

            // Resize and init Jacobian
            predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementRobotErrorState.resize(dimensionMeasurement, dimensionRobotErrorState);
            predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementRobotErrorState.setZero();

            // Fill Jacobian
            {
                unsigned int dimension_measurement_i=0;

                // z_lin_acc
                if(this->isMeasurementLinearAccelerationEnabled())
                {
                    // z_lin_acc / posi
                    // Zeros


                    // z_lin_acc / lin_speed
                    // Zeros


                    // z_lin_acc / lin_acc
                    predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementRobotErrorState.block<3,3>(dimension_measurement_i, 6)=TheImuStateCore->getScaleLinearAcceleration().asDiagonal()*mat_diff_w_amp_wrt_w*mat_q_plus_attitude_world_wrt_sensor* mat_q_minus_attitude_sensor_wrt_world*mat_diff_w_amp_wrt_w.transpose();


                    // z_lin_acc / attit
                    predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementRobotErrorState.block<3,3>(dimension_measurement_i, 9)=TheImuStateCore->getScaleLinearAcceleration().asDiagonal()*mat_diff_w_amp_wrt_w*(
                        // d(ga_I)/d(q_r_w)
                        ( mat_q_plus_attitude_robot_wrt_sensor*mat_q_minus_attitude_sensor_wrt_robot*(mat_q_minus_cross_gravity_wrt_wolrd_and_attitude_robot_wrt_world*mat_diff_quat_inv_wrt_quat + mat_q_plus_attitude_world_wrt_robot*mat_q_plus_gravity_wrt_world) ) +
                        // d(a_real)/d(q_r_w)
                        ( mat_q_plus_attitude_robot_wrt_sensor*mat_q_minus_attitude_sensor_wrt_robot*(mat_q_minus_cross_acceleration_robot_wrt_world_and_attitude_robot_wrt_world* mat_diff_quat_inv_wrt_quat + mat_q_plus_attitude_world_wrt_robot* mat_q_plus_linear_acceleration_robot_wrt_world) ) +
                        // d(a_ficticious)/d(q_r_w)
                        ( mat_q_plus_attitude_robot_wrt_sensor*mat_q_minus_attitude_sensor_wrt_robot*(
                              // an
                              mat_diff_w_amp_wrt_w.transpose()*(-2*Quaternion::skewSymMat(angular_velocity_robot_wrt_world_in_robot.cross(TheImuStateCore->getPositionSensorWrtRobot())))*mat_diff_w_amp_wrt_w*(mat_q_minus_cross_angular_vel_robot_wrt_world_in_world_and_atti_robot_wrt_world*mat_diff_quat_inv_wrt_quat+mat_q_plus_attitude_world_wrt_robot*mat_q_plus_angular_velocity_robot_wrt_world_in_world)
                              +
                              // at
                              mat_diff_w_amp_wrt_w.transpose()*(-1*Quaternion::skewSymMat(TheImuStateCore->getPositionSensorWrtRobot()))*mat_diff_w_amp_wrt_w*(mat_q_minus_cross_angular_acceleration_robot_wrt_world_in_world_and_atti_robot_wrt_world*mat_diff_quat_inv_wrt_quat+mat_q_plus_attitude_world_wrt_robot*mat_q_plus_angular_acceleration_robot_wrt_world)
                              //Eigen::Matrix4d::Zero(4,4)
                              )
                        // others
                        ))*mat_q_plus_attitude_robot_wrt_world*0.5*mat_diff_error_quat_wrt_error_theta;


                    // z_lin_acc / ang_vel
                    predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementRobotErrorState.block<3,3>(dimension_measurement_i, 12)=TheImuStateCore->getScaleLinearAcceleration().asDiagonal()*mat_diff_w_amp_wrt_w*mat_q_plus_attitude_robot_wrt_sensor*mat_q_minus_attitude_sensor_wrt_robot*mat_diff_w_amp_wrt_w.transpose()* (-2*Quaternion::skewSymMat(angular_velocity_robot_wrt_world_in_robot.cross(TheImuStateCore->getPositionSensorWrtRobot()))) *mat_diff_w_amp_wrt_w*mat_q_plus_attitude_world_wrt_robot* mat_q_minus_attitude_robot_wrt_world*mat_diff_w_amp_wrt_w.transpose();


                    // z_lin_acc / ang_acc
                    predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementRobotErrorState.block<3,3>(dimension_measurement_i, 15)=TheImuStateCore->getScaleLinearAcceleration().asDiagonal()*mat_diff_w_amp_wrt_w*mat_q_plus_attitude_robot_wrt_sensor*mat_q_minus_attitude_sensor_wrt_robot*mat_diff_w_amp_wrt_w.transpose()*(-1*Quaternion::skewSymMat(TheImuStateCore->getPositionSensorWrtRobot()))*mat_diff_w_amp_wrt_w*mat_q_plus_attitude_world_wrt_robot*mat_q_minus_attitude_robot_wrt_world*mat_diff_w_amp_wrt_w.transpose();


                    dimension_measurement_i+=3;
                }

                // z_atti
                if(this->isMeasurementOrientationEnabled())
                {
                    // TODO
                    //predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementRobotErrorState;




                    dimension_measurement_i+=3;
                }

                // z_ang_vel
                if(this->isMeasurementAngularVelocityEnabled())
                {
                    // z_ang_vel / posi
                    // Zeros

                    // z_ang_vel / lin_speed
                    // Zeros

                    // z_ang_vel / lin_acc
                    // Zeros


                    // z_ang_vel / attit
                    predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementRobotErrorState.block<3,3>(dimension_measurement_i, 9)=TheImuStateCore->getScaleAngularVelocity().asDiagonal()*mat_diff_w_amp_wrt_w*( mat_q_minus_cross_angular_vel_robot_wrt_world_in_world_and_atti_imu_wrt_world*mat_diff_quat_inv_wrt_quat + mat_q_plus_attitude_world_wrt_sensor*mat_q_plus_angular_velocity_robot_wrt_world_in_world )*mat_q_minus_attitude_sensor_wrt_robot*mat_q_plus_attitude_robot_wrt_world*0.5*mat_diff_error_quat_wrt_error_theta;


                    // z_ang_vel / ang_vel
                    predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementRobotErrorState.block<3,3>(dimension_measurement_i, 12)=TheImuStateCore->getScaleAngularVelocity().asDiagonal()*mat_diff_w_amp_wrt_w* mat_q_plus_attitude_world_wrt_sensor*mat_q_minus_attitude_sensor_wrt_world *mat_diff_w_amp_wrt_w.transpose();


                    // z_ang_vel / ang_acc
                    // Zeros


                    dimension_measurement_i+=3;
                }

            }


            // end
            break;
        }
        default:
        {
            return -2;
        }

    }



    /// Jacobian measurement - sensor state & Jacobians measurement - sensor parameters

    // dimension of the sensor state
    int dimensionSensorErrorState=TheImuStateCore->getTheSensorCore()->getDimensionErrorState();

    // Resize and init Jacobian
    predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState.resize(dimensionMeasurement, dimensionSensorErrorState);
    predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState.setZero();


    // dimension of the sensor parameters
    int dimensionSensorParameters=TheImuStateCore->getTheSensorCore()->getDimensionErrorParameters();

    // Resize and init Jacobian
    predictedMeasurement->jacobianMeasurementSensorParameters.jacobianMeasurementSensorParameters.resize(dimensionMeasurement, dimensionSensorParameters);
    predictedMeasurement->jacobianMeasurementSensorParameters.jacobianMeasurementSensorParameters.setZero();



    // Fill Jacobian
    {
        unsigned int dimension_measurement_i=0;

        // z_lin_acc
        if(this->isMeasurementLinearAccelerationEnabled())
        {
            unsigned int dimension_error_state_sensor_i=0;
            unsigned int dimension_error_parameter_sensor_i=0;


            // Auxiliar

            // Angular vel
            Eigen::Vector3d angular_vel_robot_wrt_world_in_robot=Quaternion::cross_sandwich(Quaternion::inv(TheRobotStateCoreAux->getAttitude()), TheRobotStateCoreAux->getAngularVelocity() ,TheRobotStateCoreAux->getAttitude());
            // Angular acceler
            Eigen::Vector3d angular_acc_robot_wrt_world_in_robot=Quaternion::cross_sandwich(Quaternion::inv(TheRobotStateCoreAux->getAttitude()), TheRobotStateCoreAux->getAngularAcceleration() ,TheRobotStateCoreAux->getAttitude());

            // Ficticious Acc: Normal wrt robot
            Eigen::Vector3d normal_acceleration=angular_vel_robot_wrt_world_in_robot.cross(angular_vel_robot_wrt_world_in_robot.cross(TheImuStateCore->getPositionSensorWrtRobot()));
            // Ficticious Acc: Tang wrt robot
            Eigen::Vector3d tangencial_acceleration=angular_acc_robot_wrt_world_in_robot.cross(TheImuStateCore->getPositionSensorWrtRobot());

            // Ficticious Acc total wrt robot
            Eigen::Vector3d ficticious_acceleration=normal_acceleration+tangencial_acceleration;

            // Attitude sensor wrt world
            Eigen::Vector4d attitude_sensor_wrt_world=Quaternion::cross(TheRobotStateCoreAux->getAttitude(), TheImuStateCore->getAttitudeSensorWrtRobot());

            // Acceleracion in sensor
            Eigen::Vector3d accel_sensor_wrt_sensor=Quaternion::cross_sandwich(Quaternion::inv(attitude_sensor_wrt_world), TheRobotStateCoreAux->getLinearAcceleration(), attitude_sensor_wrt_world) + Quaternion::cross_sandwich(Quaternion::inv(TheImuStateCore->getAttitudeSensorWrtRobot()), ficticious_acceleration, TheImuStateCore->getAttitudeSensorWrtRobot());

            // Gravity in sensor
            Eigen::Vector3d gravity_sensor=Quaternion::cross_sandwich(Quaternion::inv(attitude_sensor_wrt_world), TheGlobalParametersStateCore->getGravity(), attitude_sensor_wrt_world);
            // Gravity in robot
            Eigen::Vector3d gravity_robot=Quaternion::cross_sandwich(Quaternion::inv(TheRobotStateCoreAux->getAttitude()), TheGlobalParametersStateCore->getGravity(), TheRobotStateCoreAux->getAttitude());

            // Linear acceleration in sensor wrt robot
            Eigen::Vector3d accel_robot_wrt_robot=Quaternion::cross_sandwich(Quaternion::inv(TheRobotStateCoreAux->getAttitude()), TheRobotStateCoreAux->getLinearAcceleration(), TheRobotStateCoreAux->getAttitude());

            // Acceleration total imu wrt robot
            Eigen::Vector3d acceleration_total_imu_wrt_robot=gravity_robot+accel_robot_wrt_robot+ficticious_acceleration;




            // z_lin_acc / posi_sen_wrt_robot
            if(the_imu_sensor_core->isEstimationPositionSensorWrtRobotEnabled())
            {
                predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState.block<3,3>(dimension_measurement_i, dimension_error_state_sensor_i)=TheImuStateCore->getScaleLinearAcceleration().asDiagonal()*mat_diff_w_amp_wrt_w*mat_q_plus_attitude_robot_wrt_sensor*mat_q_plus_attitude_sensor_wrt_robot*mat_diff_w_amp_wrt_w.transpose()*(Quaternion::skewSymMat( angular_velocity_robot_wrt_world_in_robot.cross(angular_velocity_robot_wrt_world_in_robot) )+Quaternion::skewSymMat( angular_acceleration_robot_wrt_world_in_robot ));
                dimension_error_state_sensor_i+=3;
            }
            else
            {
                predictedMeasurement->jacobianMeasurementSensorParameters.jacobianMeasurementSensorParameters.block<3,3>(dimension_measurement_i, dimension_error_parameter_sensor_i)=TheImuStateCore->getScaleLinearAcceleration().asDiagonal()*mat_diff_w_amp_wrt_w*mat_q_plus_attitude_robot_wrt_sensor*mat_q_plus_attitude_sensor_wrt_robot*mat_diff_w_amp_wrt_w.transpose()*(Quaternion::skewSymMat( angular_velocity_robot_wrt_world_in_robot.cross(angular_velocity_robot_wrt_world_in_robot) )+Quaternion::skewSymMat( angular_acceleration_robot_wrt_world_in_robot ));
                dimension_error_parameter_sensor_i+=3;
            }


            // z_lin_acc / atti_sen_wrt_robot
            if(the_imu_sensor_core->isEstimationAttitudeSensorWrtRobotEnabled())
            {
                predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState.block<3,3>(dimension_measurement_i, dimension_error_state_sensor_i)=TheImuStateCore->getScaleLinearAcceleration().asDiagonal()*mat_diff_w_amp_wrt_w*( Quaternion::quatMatMinus(Quaternion::cross_pure_gen(acceleration_total_imu_wrt_robot, TheImuStateCore->getAttitudeSensorWrtRobot()))*mat_diff_quat_inv_wrt_quat + mat_q_plus_attitude_robot_wrt_sensor*Quaternion::quatMatPlus(acceleration_total_imu_wrt_robot) )*mat_q_plus_attitude_sensor_wrt_robot*0.5*mat_diff_error_quat_wrt_error_theta;
                dimension_error_state_sensor_i+=3;
            }
            else
            {
                predictedMeasurement->jacobianMeasurementSensorParameters.jacobianMeasurementSensorParameters.block<3,3>(dimension_measurement_i, dimension_error_parameter_sensor_i)=TheImuStateCore->getScaleLinearAcceleration().asDiagonal()*mat_diff_w_amp_wrt_w*( Quaternion::quatMatMinus(Quaternion::cross_pure_gen(acceleration_total_imu_wrt_robot, TheImuStateCore->getAttitudeSensorWrtRobot()))*mat_diff_quat_inv_wrt_quat + mat_q_plus_attitude_robot_wrt_sensor*Quaternion::quatMatPlus(acceleration_total_imu_wrt_robot) )*mat_q_plus_attitude_sensor_wrt_robot*0.5*mat_diff_error_quat_wrt_error_theta;
                dimension_error_parameter_sensor_i+=3;
            }


            // z_lin_acc / ba
            if(the_imu_sensor_core->isEstimationBiasLinearAccelerationEnabled())
            {
                predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState.block<3,3>(dimension_measurement_i, dimension_error_state_sensor_i)=Eigen::Matrix3d::Identity(3,3);
                dimension_error_state_sensor_i+=3;
            }
            else
            {
                predictedMeasurement->jacobianMeasurementSensorParameters.jacobianMeasurementSensorParameters.block<3,3>(dimension_measurement_i, dimension_error_parameter_sensor_i)=Eigen::Matrix3d::Identity(3,3);
                dimension_error_parameter_sensor_i+=3;
            }


            // z_lin_acc / ka
            if(the_imu_sensor_core->isEstimationScaleLinearAccelerationEnabled())
            {
                predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState.block<3,3>(dimension_measurement_i, dimension_error_state_sensor_i)=(accel_sensor_wrt_sensor+gravity_sensor).asDiagonal();
                dimension_error_state_sensor_i+=3;
            }
            else
            {
                predictedMeasurement->jacobianMeasurementSensorParameters.jacobianMeasurementSensorParameters.block<3,3>(dimension_measurement_i, dimension_error_parameter_sensor_i)=(accel_sensor_wrt_sensor+gravity_sensor).asDiagonal();
                dimension_error_parameter_sensor_i+=3;
            }


            // z_lin_acc / bw
            if(the_imu_sensor_core->isEstimationBiasAngularVelocityEnabled())
            {
                // Zeros
                dimension_error_state_sensor_i+=3;
            }
            else
            {
                // Zeros
                dimension_error_parameter_sensor_i+=3;
            }


            // z_lin_acc / kw
            if(the_imu_sensor_core->isEstimationScaleAngularVelocityEnabled())
            {
                // Zeros
                dimension_error_state_sensor_i+=3;
            }
            else
            {
                // Zeros
                dimension_error_parameter_sensor_i+=3;
            }


            dimension_measurement_i+=3;
        }

        // z_atti
        if(this->isMeasurementOrientationEnabled())
        {

            unsigned int dimension_error_state_sensor_i=0;
            unsigned int dimension_error_parameter_sensor_i=0;


            // z_atti / posi_sen_wrt_robot
            if(the_imu_sensor_core->isEstimationPositionSensorWrtRobotEnabled())
            {
                // TODO
                //predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState.block<3,3>(dimension_measurement_i, dimension_error_state_sensor_i);
                dimension_error_state_sensor_i+=3;
            }
            else
            {
                // TODO
                //predictedMeasurement->jacobianMeasurementSensorParameters.jacobianMeasurementSensorParameters.block<3,3>(dimension_measurement_i, dimension_error_parameter_sensor_i);
                dimension_error_parameter_sensor_i+=3;
            }


            // z_atti / atti_sen_wrt_robot
            if(the_imu_sensor_core->isEstimationAttitudeSensorWrtRobotEnabled())
            {
                // TODO
                //predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState.block<3,3>(dimension_measurement_i, dimension_error_state_sensor_i);
                dimension_error_state_sensor_i+=3;
            }
            else
            {
                // TODO
                //predictedMeasurement->jacobianMeasurementSensorParameters.jacobianMeasurementSensorParameters.block<3,3>(dimension_measurement_i, dimension_error_parameter_sensor_i);
                dimension_error_parameter_sensor_i+=3;
            }


            // z_atti / ba
            if(the_imu_sensor_core->isEstimationBiasLinearAccelerationEnabled())
            {
                // Zeros
                dimension_error_state_sensor_i+=3;
            }
            else
            {
                // Zeros
                dimension_error_parameter_sensor_i+=3;
            }


            // z_atti / ka
            if(the_imu_sensor_core->isEstimationScaleLinearAccelerationEnabled())
            {
                // Zeros
                dimension_error_state_sensor_i+=3;
            }
            else
            {
                // Zeros
                dimension_error_parameter_sensor_i+=3;
            }


            // z_atti / bw
            if(the_imu_sensor_core->isEstimationBiasAngularVelocityEnabled())
            {
                // Zeros
                dimension_error_state_sensor_i+=3;
            }
            else
            {
                // Zeros
                dimension_error_parameter_sensor_i+=3;
            }


            // z_atti / kw
            if(the_imu_sensor_core->isEstimationScaleAngularVelocityEnabled())
            {
                // Zeros
                dimension_error_state_sensor_i+=3;
            }
            else
            {
                // Zeros
                dimension_error_parameter_sensor_i+=3;
            }


            dimension_measurement_i+=3;
        }

        // z_ang_vel
        if(this->isMeasurementAngularVelocityEnabled())
        {
            unsigned int dimension_error_state_sensor_i=0;
            unsigned int dimension_error_parameter_sensor_i=0;


            // z_ang_vel / posi_sen_wrt_robot
            if(the_imu_sensor_core->isEstimationPositionSensorWrtRobotEnabled())
            {
                // Zeros
                dimension_error_state_sensor_i+=3;
            }
            else
            {
                // Zeros
                dimension_error_parameter_sensor_i+=3;
            }


            // z_ang_vel / atti_sen_wrt_robot
            if(the_imu_sensor_core->isEstimationAttitudeSensorWrtRobotEnabled())
            {
                predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState.block<3,3>(dimension_measurement_i, dimension_error_state_sensor_i)=TheImuStateCore->getScaleAngularVelocity().asDiagonal()*mat_diff_w_amp_wrt_w* ( mat_q_minus_cross_angular_vel_robot_wrt_world_in_world_and_atti_imu_wrt_world*mat_diff_quat_inv_wrt_quat + mat_q_plus_attitude_world_wrt_sensor*mat_q_plus_angular_velocity_robot_wrt_world_in_world ) *mat_q_plus_attitude_robot_wrt_world*mat_q_plus_attitude_sensor_wrt_robot*0.5*mat_diff_error_quat_wrt_error_theta;
                dimension_error_state_sensor_i+=3;
            }
            else
            {
                predictedMeasurement->jacobianMeasurementSensorParameters.jacobianMeasurementSensorParameters.block<3,3>(dimension_measurement_i, dimension_error_parameter_sensor_i)=TheImuStateCore->getScaleAngularVelocity().asDiagonal()*mat_diff_w_amp_wrt_w* ( mat_q_minus_cross_angular_vel_robot_wrt_world_in_world_and_atti_imu_wrt_world*mat_diff_quat_inv_wrt_quat + mat_q_plus_attitude_world_wrt_sensor*mat_q_plus_angular_velocity_robot_wrt_world_in_world ) *mat_q_plus_attitude_robot_wrt_world*mat_q_plus_attitude_sensor_wrt_robot*0.5*mat_diff_error_quat_wrt_error_theta;
                dimension_error_parameter_sensor_i+=3;
            }


            // z_ang_vel / ba
            if(the_imu_sensor_core->isEstimationBiasLinearAccelerationEnabled())
            {
                // Zeros
                dimension_error_state_sensor_i+=3;
            }
            else
            {
                // Zeros
                dimension_error_parameter_sensor_i+=3;
            }


            // z_ang_vel / ka
            if(the_imu_sensor_core->isEstimationScaleLinearAccelerationEnabled())
            {
                // Zeros
                dimension_error_state_sensor_i+=3;
            }
            else
            {
                // Zeros
                dimension_error_parameter_sensor_i+=3;
            }


            // z_ang_vel / bw
            if(the_imu_sensor_core->isEstimationBiasAngularVelocityEnabled())
            {
                predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState.block<3,3>(dimension_measurement_i, dimension_error_state_sensor_i)=Eigen::Matrix3d::Identity(3,3);
                dimension_error_state_sensor_i+=3;
            }
            else
            {
                predictedMeasurement->jacobianMeasurementSensorParameters.jacobianMeasurementSensorParameters.block<3,3>(dimension_measurement_i, dimension_error_parameter_sensor_i)=Eigen::Matrix3d::Identity(3,3);
                dimension_error_parameter_sensor_i+=3;
            }


            // z_ang_vel / kw
            if(the_imu_sensor_core->isEstimationScaleAngularVelocityEnabled())
            {
                predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState.block<3,3>(dimension_measurement_i, dimension_error_state_sensor_i)=angular_velocity_imu_wrt_world_in_imu.asDiagonal();
                dimension_error_state_sensor_i+=3;
            }
            else
            {
                predictedMeasurement->jacobianMeasurementSensorParameters.jacobianMeasurementSensorParameters.block<3,3>(dimension_measurement_i, dimension_error_parameter_sensor_i)=angular_velocity_imu_wrt_world_in_imu.asDiagonal();
                dimension_error_parameter_sensor_i+=3;
            }


            dimension_measurement_i+=3;
        }

    }




    /// Jacobians measurement - global parameters

    // dimension of the global parameters
    int dimensionGlobalParameters=TheGlobalParametersStateCore->getTheGlobalParametersCore()->getDimensionErrorParameters();

    // Resize and init Jacobian
    predictedMeasurement->jacobianMeasurementGlobalParameters.jacobianMeasurementGlobalParameters.resize(dimensionMeasurement, dimensionGlobalParameters);
    predictedMeasurement->jacobianMeasurementGlobalParameters.jacobianMeasurementGlobalParameters.setZero();

    // Fill Jacobian
    {
        unsigned int dimension_measurement_i=0;

        // z_lin_acc
        if(this->isMeasurementLinearAccelerationEnabled())
        {
            // z_lin_acc / grav
            predictedMeasurement->jacobianMeasurementGlobalParameters.jacobianMeasurementGlobalParameters.block<3,3>(dimension_measurement_i,0)=TheImuStateCore->getScaleLinearAcceleration().asDiagonal()*mat_diff_w_amp_wrt_w*mat_q_plus_attitude_world_wrt_sensor* mat_q_minus_attitude_sensor_wrt_robot*mat_diff_w_amp_wrt_w.transpose();

            dimension_measurement_i+=3;
        }

        // z_atti
        if(this->isMeasurementOrientationEnabled())
        {
            // z_atti / grav
            // Zeros

            dimension_measurement_i+=3;
        }


    }
    */


    /// Jacobians measurement - sensor noise of the measurement

    // Resize and init Jacobian
    predictedMeasurement->jacobianMeasurementSensorNoise.jacobianMeasurementSensorNoise.resize(dimensionMeasurement, dimensionMeasurement);
    predictedMeasurement->jacobianMeasurementSensorNoise.jacobianMeasurementSensorNoise.setZero();

    // Fill Jacobian
    predictedMeasurement->jacobianMeasurementSensorNoise.jacobianMeasurementSensorNoise=Eigen::MatrixXd::Identity(dimensionMeasurement, dimensionMeasurement);




    return 0;
}
