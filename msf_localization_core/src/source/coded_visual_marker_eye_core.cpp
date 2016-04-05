
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
    dimensionErrorMeasurement=0;

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
        dimensionErrorMeasurement+=3;
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
        dimensionErrorMeasurement+=3;
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

/*
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
*/

int CodedVisualMarkerEyeCore::setMeasurementList(const TimeStamp the_time_stamp, std::list< std::shared_ptr<SensorMeasurementCore> > the_visual_marker_measurement_list)
{
    if(!isSensorEnabled())
        return 0;

    if(this->getTheMsfStorageCore()->setMeasurementList(the_time_stamp, the_visual_marker_measurement_list))
    {
        std::cout<<"CodedVisualMarkerEyeCore::setMeasurement() error"<<std::endl;
        return 1;
    }

    return 0;
}

Eigen::SparseMatrix<double> CodedVisualMarkerEyeCore::getCovarianceMeasurement()
{
    Eigen::SparseMatrix<double> covariances_matrix;

    covariances_matrix.resize(this->getDimensionErrorMeasurement(), this->getDimensionErrorMeasurement());
    //covariances_matrix.setZero();
    covariances_matrix.reserve(this->getDimensionErrorMeasurement());

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


int CodedVisualMarkerEyeCore::predictMeasurement(const TimeStamp theTimeStamp, const std::shared_ptr<GlobalParametersStateCore> TheGlobalParametersStateCore, const std::shared_ptr<RobotStateCore> currentRobotState, const std::shared_ptr<SensorStateCore> currentSensorStateI, const std::shared_ptr<MapElementStateCore> currentMapElementStateI, std::shared_ptr<CodedVisualMarkerMeasurementCore>& predictedMeasurement)
{
#if _DEBUG_SENSOR_CORE
    logFile<<"CodedVisualMarkerEyeCore::predictMeasurement() TS: sec="<<theTimeStamp.sec<<" s; nsec="<<theTimeStamp.nsec<<" ns"<<std::endl;
#endif

    // Check
    if(!this->getTheSensorCore())
    {
        std::cout<<"CodedVisualMarkerEyeCore::predictMeasurement() error 50"<<std::endl;
        return 50;
    }

    // Check global parameters
    if(!TheGlobalParametersStateCore)
        return 1;

    if(!TheGlobalParametersStateCore->getTheGlobalParametersCore())
        return 1;


    // Check sensor
    if(!currentSensorStateI)
    {
        std::cout<<"CodedVisualMarkerEyeCore::predictMeasurement() error 1"<<std::endl;
        return 1;
    }

    // Check sensor
    if(!currentSensorStateI->getTheSensorCore())
    {
        std::cout<<"CodedVisualMarkerEyeCore::predictMeasurement() error 1"<<std::endl;
        return 1;
    }

    // Cast
    std::shared_ptr<CodedVisualMarkerEyeStateCore> currentSensorState=std::dynamic_pointer_cast<CodedVisualMarkerEyeStateCore>(currentSensorStateI);

    // Robot check
    if(!currentRobotState)
    {
        std::cout<<"CodedVisualMarkerEyeCore::predictMeasurement() error 2"<<std::endl;
        return 2;
    }

    // Robot core check
    if(!currentRobotState->getTheRobotCore())
    {
        std::cout<<"CodedVisualMarkerEyeCore::predictMeasurement() error 3"<<std::endl;
        return 3;
    }


    // Map element
    if(!currentMapElementStateI)
    {
        std::cout<<"CodedVisualMarkerEyeCore::predictMeasurement() error 2"<<std::endl;
        return 4;
    }

    // Map element core
    if(!currentMapElementStateI->getTheMapElementCore())
    {
        std::cout<<"CodedVisualMarkerEyeCore::predictMeasurement() error 2"<<std::endl;
        return 5;
    }

    // Cast
    std::shared_ptr<CodedVisualMarkerLandmarkStateCore> currentMapElementState=std::dynamic_pointer_cast<CodedVisualMarkerLandmarkStateCore>(currentMapElementStateI);

    // Checks
    // TODO


    // Create pointer
    // TODO check if it must be done here
    if(!predictedMeasurement)
    {
        predictedMeasurement=std::make_shared<CodedVisualMarkerMeasurementCore>();
#if _DEBUG_SENSOR_CORE
        logFile<<"CodedVisualMarkerEyeCore::predictMeasurement() pointer created"<<std::endl;
#endif
    }


    // Set the sensor core -> Needed
    if(predictedMeasurement)
    {
        predictedMeasurement->setTheSensorCore(this->getTheSensorCore());
    }


    // id -> Needed
    std::shared_ptr<CodedVisualMarkerLandmarkCore> TheCodedVisualMarkerLandmarkCore=std::dynamic_pointer_cast<CodedVisualMarkerLandmarkCore>(currentMapElementState->getTheMapElementCore());
    predictedMeasurement->setVisualMarkerId(TheCodedVisualMarkerLandmarkCore->getId());



    // Prediction

    // Switch depending on robot used
    switch(currentRobotState->getTheRobotCore()->getRobotType())
    {
        // Free model robot
        case RobotTypes::free_model:
        {
            // Cast
            std::shared_ptr<FreeModelRobotStateCore> currentFreeModelRobotState=std::static_pointer_cast<FreeModelRobotStateCore>(currentRobotState);



            // Aux vars
            Eigen::Vector4d attitude_visual_marker_eye_wrt_world=
                    Quaternion::cross(currentFreeModelRobotState->getAttitude(), currentSensorState->getAttitudeSensorWrtRobot());


            // Position
            if(this->isMeasurementPositionEnabled())
            {
                // Aux Variable
                Eigen::Vector3d position_visual_marker_wrt_visual_marker_eye;
                position_visual_marker_wrt_visual_marker_eye.setZero();


                Eigen::Vector3d position_visual_marker_eye_wrt_world=
                        Quaternion::cross_sandwich(currentFreeModelRobotState->getAttitude(), currentSensorState->getPositionSensorWrtRobot(), Quaternion::inv(currentFreeModelRobotState->getAttitude()));


                // Equation
                position_visual_marker_wrt_visual_marker_eye=
                        Quaternion::cross_sandwich(Quaternion::inv(attitude_visual_marker_eye_wrt_world), currentMapElementState->getPosition()-position_visual_marker_eye_wrt_world-currentFreeModelRobotState->getPosition(), attitude_visual_marker_eye_wrt_world);

                // Set
                predictedMeasurement->setVisualMarkerPosition(position_visual_marker_wrt_visual_marker_eye);
            }



            // Attitude
            if(this->isMeasurementAttitudeEnabled())
            {
                // Aux Variable
                Eigen::Vector4d attitude_visual_marker_wrt_visual_marker_eye;
                attitude_visual_marker_wrt_visual_marker_eye.setZero();

                // Equation
                attitude_visual_marker_wrt_visual_marker_eye=
                        Quaternion::cross(Quaternion::inv(attitude_visual_marker_eye_wrt_world), currentMapElementState->getAttitude());


                // Set
                if(attitude_visual_marker_wrt_visual_marker_eye[0]<0)
                    predictedMeasurement->setVisualMarkerAttitude(-attitude_visual_marker_wrt_visual_marker_eye);
                else
                    predictedMeasurement->setVisualMarkerAttitude(attitude_visual_marker_wrt_visual_marker_eye);
            }

            break;
        }

        // Default
        default:
            return -1000;
    }


#if _DEBUG_SENSOR_CORE
    logFile<<"CodedVisualMarkerEyeCore::predictMeasurement() ended TS: sec="<<theTimeStamp.sec<<" s; nsec="<<theTimeStamp.nsec<<" ns"<<std::endl;
#endif


    // End
    return 0;
}



int CodedVisualMarkerEyeCore::jacobiansMeasurements(const TimeStamp theTimeStamp, const std::shared_ptr<GlobalParametersStateCore> currentGlobalParametersStateCore, const std::shared_ptr<RobotStateCore> currentRobotState, const std::shared_ptr<SensorStateCore> currentSensorState, const std::shared_ptr<MapElementStateCore> currentMapElementState, std::shared_ptr<SensorMeasurementCore> matchedMeasurement, std::shared_ptr<CodedVisualMarkerMeasurementCore>& predictedMeasurement)
{
    // TODO


    // sensor
    std::shared_ptr<CodedVisualMarkerEyeStateCore> the_sensor_state_core=std::dynamic_pointer_cast<CodedVisualMarkerEyeStateCore>(currentSensorState);
    std::shared_ptr<const CodedVisualMarkerEyeCore> the_sensor_core=std::dynamic_pointer_cast<const CodedVisualMarkerEyeCore>(the_sensor_state_core->getTheSensorCoreShared());


    // dimension of the measurement
    int dimension_error_measurement=the_sensor_core->getTheSensorCore()->getDimensionErrorMeasurement();


    // Robot
    std::shared_ptr<FreeModelRobotStateCore> TheRobotStateCoreAux=std::dynamic_pointer_cast<FreeModelRobotStateCore>(currentRobotState);
    std::shared_ptr<const FreeModelRobotCore> TheRobotCoreAux=std::dynamic_pointer_cast<const FreeModelRobotCore>(TheRobotStateCoreAux->getTheRobotCore());


    // Map element
    std::shared_ptr<CodedVisualMarkerLandmarkStateCore> TheMapElementStateCore=std::dynamic_pointer_cast<CodedVisualMarkerLandmarkStateCore>(currentMapElementState);
    std::shared_ptr<CodedVisualMarkerLandmarkCore> TheMapElementCore=std::dynamic_pointer_cast<CodedVisualMarkerLandmarkCore>(TheMapElementStateCore->getTheMapElementCore());


    // Matched measurement
    std::shared_ptr<CodedVisualMarkerMeasurementCore> TheMatchedMeasurementCore=std::dynamic_pointer_cast<CodedVisualMarkerMeasurementCore>(matchedMeasurement);


    // TODO


    // Auxiliar variables
    Eigen::Vector3d tran_inc_wrt_world=TheMapElementStateCore->getPosition()-TheRobotStateCoreAux->getPosition()-Quaternion::cross_sandwich(TheRobotStateCoreAux->getAttitude(), the_sensor_state_core->getPositionSensorWrtRobot(), Quaternion::inv(TheRobotStateCoreAux->getAttitude()));
    Eigen::Vector3d tran_inc2_wrt_world=TheMapElementStateCore->getPosition()-TheRobotStateCoreAux->getPosition();

    //Eigen::Vector4d att_visual_marker_wrt_visual_marker_eye=TheMatchedMeasurementCore->getVisualMarkerAttitude();
    Eigen::Vector4d att_visual_marker_wrt_visual_marker_eye=predictedMeasurement->getVisualMarkerAttitude();

    Eigen::Vector4d att_visual_marker_eye_wrt_world=Quaternion::cross(TheRobotStateCoreAux->getAttitude(), the_sensor_state_core->getAttitudeSensorWrtRobot());
    Eigen::Vector4d att_world_wrt_visual_marker_eye=Quaternion::inv(att_visual_marker_eye_wrt_world);

    Eigen::Matrix4d mat_q_plus_att_world_wrt_visual_marker_eye=Quaternion::quatMatPlus(att_world_wrt_visual_marker_eye);

    Eigen::Matrix4d mat_q_minus_att_visual_marker_eye_wrt_world=Quaternion::quatMatMinus(att_visual_marker_eye_wrt_world);

    Eigen::Matrix4d mat_q_plus_att_visual_marker_wrt_visual_marker_eye=Quaternion::quatMatPlus(att_visual_marker_wrt_visual_marker_eye);
    Eigen::Matrix4d inv_mat_q_plus_att_visual_marker_wrt_visual_marker_eye=mat_q_plus_att_visual_marker_wrt_visual_marker_eye.inverse();

    Eigen::Matrix4d mat_q_minus_att_visual_marker_wrt_world=Quaternion::quatMatMinus(TheMapElementStateCore->getAttitude());
    Eigen::Matrix4d mat_q_plus_att_visual_marker_wrt_world=Quaternion::quatMatPlus(TheMapElementStateCore->getAttitude());

    Eigen::Matrix4d mat_q_plus_att_robot_wrt_world=Quaternion::quatMatPlus(TheRobotStateCoreAux->getAttitude());

    Eigen::Matrix4d mat_q_minus_att_visual_marker_eye_wrt_robot=Quaternion::quatMatMinus(the_sensor_state_core->getAttitudeSensorWrtRobot());
    Eigen::Matrix4d mat_q_plus_att_visual_marker_eye_wrt_robot=Quaternion::quatMatPlus(the_sensor_state_core->getAttitudeSensorWrtRobot());

    Eigen::Matrix4d mat_q_plus_tran_inc2_wrt_world=Quaternion::quatMatPlus(tran_inc2_wrt_world);

    Eigen::Matrix4d mat_q_minus_tinc2aux_wrt_world=Quaternion::quatMatMinus(Quaternion::cross_pure_gen(tran_inc2_wrt_world, att_visual_marker_eye_wrt_world));

    Eigen::Matrix4d mat_q_minus_aux1=Quaternion::quatMatMinus(Quaternion::cross_pure_gen(the_sensor_state_core->getPositionSensorWrtRobot(),the_sensor_state_core->getAttitudeSensorWrtRobot()));

    Eigen::Matrix4d mat_q_plus_tra_visual_marker_eye_wrt_robot=Quaternion::quatMatPlus(the_sensor_state_core->getAttitudeSensorWrtRobot());

    Eigen::Matrix4d mat_q_plus_tran_inc_wrt_world=Quaternion::quatMatPlus(tran_inc_wrt_world);

    Eigen::Matrix4d mat_q_mins_aux2=Quaternion::quatMatMinus(Quaternion::cross_pure_gen(tran_inc_wrt_world, att_visual_marker_eye_wrt_world));

    Eigen::Matrix4d mat_diff_quat_inv_wrt_quat;
    mat_diff_quat_inv_wrt_quat<<1, 0, 0, 0,
                                0, -1, 0, 0,
                                0, 0, -1, 0,
                                0, 0, 0, -1;

    Eigen::MatrixXd mat_diff_error_quat_wrt_error_theta(4,3);
    mat_diff_error_quat_wrt_error_theta<<0, 0, 0,
                                        1, 0, 0,
                                        0, 1, 0,
                                        0, 0, 1;

    Eigen::MatrixXd mat_diff_vector_wrt_vector_amp(3,4);
    mat_diff_vector_wrt_vector_amp<<0, 1, 0, 0,
                                    0, 0, 1, 0,
                                    0, 0, 0, 1;



    //// Jacobian Measurement Error - Error State && Jacobian Measurement Error - Error Parameters

    /// Jacobian Measurement Error - Robot Error State

    // Dimension
    int dimension_robot_error_state=TheRobotCoreAux->getDimensionErrorState();

    // Resize and init
    predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementRobotErrorState.resize(dimension_error_measurement, dimension_robot_error_state);
    predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementRobotErrorState.setZero();

    // Fill
    {
        int dimension_error_measurement_i=0;
        if(this->isMeasurementPositionEnabled())
        {
            // pos
            predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementRobotErrorState.block<3,3>(dimension_error_measurement_i,0)=
                    -mat_diff_vector_wrt_vector_amp*mat_q_plus_att_world_wrt_visual_marker_eye*mat_q_minus_att_visual_marker_eye_wrt_world*mat_diff_vector_wrt_vector_amp.transpose();

            // lin_vel
            // zeros

            // lin_acc
            // zeros

            // attit
            predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementRobotErrorState.block<3,3>(dimension_error_measurement_i,9)=
                    mat_diff_vector_wrt_vector_amp*( mat_q_minus_tinc2aux_wrt_world*mat_diff_quat_inv_wrt_quat + mat_q_plus_att_world_wrt_visual_marker_eye*mat_q_plus_tran_inc2_wrt_world )*mat_q_minus_att_visual_marker_eye_wrt_robot*mat_q_plus_att_robot_wrt_world*0.5*mat_diff_error_quat_wrt_error_theta;

            // ang_vel
            // zeros

            // ang_acc
            // zeros

            dimension_error_measurement_i+=3;
        }

        if(this->isMeasurementAttitudeEnabled())
        {
            // pos
            // zeros

            // lin_vel
            // zeros

            // lin_acc
            // zeros

            // attit
            predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementRobotErrorState.block<3,3>(dimension_error_measurement_i,9)=
                    mat_diff_error_quat_wrt_error_theta.transpose()*inv_mat_q_plus_att_visual_marker_wrt_visual_marker_eye*mat_q_minus_att_visual_marker_wrt_world*mat_diff_quat_inv_wrt_quat*mat_q_minus_att_visual_marker_eye_wrt_robot*mat_q_plus_att_robot_wrt_world*mat_diff_error_quat_wrt_error_theta;

            // ang_vel
            // zeros

            // ang_acc
            // zeros


            dimension_error_measurement_i+=3;
        }
    }



    /// Jacobian Measurement Error - Global Parameters Error State & Error Parameters

    // Dimension
    int dimension_global_parameters_error_state=currentGlobalParametersStateCore->getTheGlobalParametersCore()->getDimensionErrorState();
    int dimension_global_parameters_error_parameters=currentGlobalParametersStateCore->getTheGlobalParametersCore()->getDimensionErrorParameters();

    // Resize and init
    predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementGlobalParametersErrorState.resize(dimension_error_measurement, dimension_global_parameters_error_state);
    predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementGlobalParametersErrorState.setZero();

    predictedMeasurement->jacobianMeasurementErrorParameters.jacobianMeasurementGlobalParameters.resize(dimension_error_measurement, dimension_global_parameters_error_parameters);
    predictedMeasurement->jacobianMeasurementErrorParameters.jacobianMeasurementGlobalParameters.setZero();

    // Fill
    // No dependency on global parameters -> Everything is set to zero



    /// Jacobian Measurement Error - Sensor Error State & Error Parameters

    // Dimension
    int dimension_sensor_error_state=the_sensor_core->getDimensionErrorState();
    int dimension_sensor_error_parameters=the_sensor_core->getDimensionErrorParameters();

    // Resize and init
    predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState.resize(dimension_error_measurement, dimension_sensor_error_state);
    predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState.setZero();

    predictedMeasurement->jacobianMeasurementErrorParameters.jacobianMeasurementSensorParameters.resize(dimension_error_measurement, dimension_sensor_error_parameters);
    predictedMeasurement->jacobianMeasurementErrorParameters.jacobianMeasurementSensorParameters.setZero();

    // Fill
    {
        int dimension_error_measurement_i=0;

        // pos
        if(this->isMeasurementPositionEnabled())
        {
            int dimension_sensor_error_parameters_i=0;
            int dimension_sensor_error_state_i=0;

            // pos
            if(the_sensor_core->isEstimationAttitudeSensorWrtRobotEnabled())
            {
                predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState.block<3, 3>(dimension_error_measurement_i, dimension_sensor_error_state_i)=
                        -mat_diff_vector_wrt_vector_amp*(mat_q_minus_aux1*mat_diff_quat_inv_wrt_quat + mat_q_plus_att_visual_marker_eye_wrt_robot*mat_q_plus_tra_visual_marker_eye_wrt_robot )*mat_diff_vector_wrt_vector_amp.transpose();
                dimension_sensor_error_state_i+=3;
            }
            else
            {
                predictedMeasurement->jacobianMeasurementErrorParameters.jacobianMeasurementSensorParameters.block<3,3>(dimension_error_measurement_i, dimension_sensor_error_parameters_i)=
                        -mat_diff_vector_wrt_vector_amp*(mat_q_minus_aux1*mat_diff_quat_inv_wrt_quat + mat_q_plus_att_visual_marker_eye_wrt_robot*mat_q_plus_tra_visual_marker_eye_wrt_robot )*mat_diff_vector_wrt_vector_amp.transpose();
                dimension_sensor_error_parameters_i+=3;
            }

            // att
            if(the_sensor_core->isEstimationAttitudeSensorWrtRobotEnabled())
            {
                predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState.block<3, 3>(dimension_error_measurement_i, dimension_sensor_error_state_i)=
                        mat_diff_vector_wrt_vector_amp*( mat_q_mins_aux2*mat_diff_quat_inv_wrt_quat + mat_q_plus_att_world_wrt_visual_marker_eye*mat_q_plus_tran_inc_wrt_world )*mat_q_plus_att_robot_wrt_world*mat_q_plus_att_visual_marker_eye_wrt_robot*0.5*mat_diff_error_quat_wrt_error_theta;
                dimension_sensor_error_state_i+=3;
            }
            else
            {
                predictedMeasurement->jacobianMeasurementErrorParameters.jacobianMeasurementSensorParameters.block<3,3>(dimension_error_measurement_i, dimension_sensor_error_parameters_i)=
                        mat_diff_vector_wrt_vector_amp*( mat_q_mins_aux2*mat_diff_quat_inv_wrt_quat + mat_q_plus_att_world_wrt_visual_marker_eye*mat_q_plus_tran_inc_wrt_world )*mat_q_plus_att_robot_wrt_world*mat_q_plus_att_visual_marker_eye_wrt_robot*0.5*mat_diff_error_quat_wrt_error_theta;
                dimension_sensor_error_parameters_i+=3;
            }

            dimension_error_measurement_i+=3;
        }

        // att
        if(this->isMeasurementAttitudeEnabled())
        {
            int dimension_sensor_error_parameters_i=0;
            int dimension_sensor_error_state_i=0;

            // pos
            if(the_sensor_core->isEstimationAttitudeSensorWrtRobotEnabled())
            {
                // Zeros
                dimension_sensor_error_state_i+=3;
            }
            else
            {
                // Zeros
                dimension_sensor_error_parameters_i+=3;
            }

            // att
            if(the_sensor_core->isEstimationAttitudeSensorWrtRobotEnabled())
            {
                predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementSensorErrorState.block<3,3>(dimension_error_measurement_i, dimension_sensor_error_state_i)=
                        mat_diff_error_quat_wrt_error_theta.transpose()*inv_mat_q_plus_att_visual_marker_wrt_visual_marker_eye*mat_q_minus_att_visual_marker_wrt_world*mat_diff_quat_inv_wrt_quat*mat_q_plus_att_robot_wrt_world*mat_q_plus_att_visual_marker_eye_wrt_robot*mat_diff_error_quat_wrt_error_theta;
                dimension_sensor_error_state_i+=3;
            }
            else
            {
                predictedMeasurement->jacobianMeasurementErrorParameters.jacobianMeasurementSensorParameters.block<3,3>(dimension_error_measurement_i, dimension_sensor_error_parameters_i)=
                        mat_diff_error_quat_wrt_error_theta.transpose()*inv_mat_q_plus_att_visual_marker_wrt_visual_marker_eye*mat_q_minus_att_visual_marker_wrt_world*mat_diff_quat_inv_wrt_quat*mat_q_plus_att_robot_wrt_world*mat_q_plus_att_visual_marker_eye_wrt_robot*mat_diff_error_quat_wrt_error_theta;
                dimension_sensor_error_parameters_i+=3;
            }

            dimension_error_measurement_i+=3;
        }
    }



    /// Jacobian Measurement Error - Map Element Error State & Error Parameters

    // Dimension
    int dimension_map_element_error_state=TheMapElementCore->getDimensionErrorState();
    int dimension_map_element_error_parameters=TheMapElementCore->getDimensionErrorParameters();

    // Resize and init
    predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementMapElementErrorState.resize(dimension_error_measurement, dimension_map_element_error_state);
    predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementMapElementErrorState.setZero();

    predictedMeasurement->jacobianMeasurementErrorParameters.jacobianMeasurementMapElementParameters.resize(dimension_error_measurement, dimension_map_element_error_parameters);
    predictedMeasurement->jacobianMeasurementErrorParameters.jacobianMeasurementMapElementParameters.setZero();


    // Fill
    {
        int dimension_error_measurement_i=0;

        // pos
        if(this->isMeasurementPositionEnabled())
        {
            int dimension_map_element_error_parameters_i=0;
            int dimension_map_element_error_state_i=0;

            // pos
            if(TheMapElementCore->isEstimationPositionVisualMarkerWrtWorldEnabled())
            {
                predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementMapElementErrorState.block<3,3>(dimension_error_measurement_i, dimension_map_element_error_state_i)=
                        mat_diff_vector_wrt_vector_amp*mat_q_plus_att_world_wrt_visual_marker_eye*mat_q_minus_att_visual_marker_eye_wrt_world  *mat_diff_vector_wrt_vector_amp.transpose();
                dimension_map_element_error_state_i+=3;
            }
            else
            {
                predictedMeasurement->jacobianMeasurementErrorParameters.jacobianMeasurementMapElementParameters.block<3,3>(dimension_error_measurement_i, dimension_map_element_error_parameters_i)=
                        mat_diff_vector_wrt_vector_amp*mat_q_plus_att_world_wrt_visual_marker_eye*mat_q_minus_att_visual_marker_eye_wrt_world  *mat_diff_vector_wrt_vector_amp.transpose();
                dimension_map_element_error_parameters_i+=3;
            }

            // att
            if(TheMapElementCore->isEstimationAttitudeVisualMarkerWrtWorldEnabled())
            {
                // Zeros
                dimension_map_element_error_state_i+=3;
            }
            else
            {
                // Zeros
                dimension_map_element_error_parameters_i+=3;
            }

            dimension_error_measurement_i+=3;
        }

        // att
        if(this->isMeasurementAttitudeEnabled())
        {
            int dimension_map_element_error_parameters_i=0;
            int dimension_map_element_error_state_i=0;

            // pos
            if(TheMapElementCore->isEstimationPositionVisualMarkerWrtWorldEnabled())
            {
                // Zeros
                dimension_map_element_error_state_i+=3;
            }
            else
            {
                // Zeros
                dimension_map_element_error_parameters_i+=3;
            }

            // att
            if(TheMapElementCore->isEstimationAttitudeVisualMarkerWrtWorldEnabled())
            {
                predictedMeasurement->jacobianMeasurementErrorState.jacobianMeasurementMapElementErrorState.block<3,3>(dimension_error_measurement_i, dimension_map_element_error_state_i)=
                        mat_diff_error_quat_wrt_error_theta.transpose()*inv_mat_q_plus_att_visual_marker_wrt_visual_marker_eye* mat_q_plus_att_world_wrt_visual_marker_eye *mat_q_plus_att_visual_marker_wrt_world*mat_diff_error_quat_wrt_error_theta;
                dimension_map_element_error_state_i+=3;
            }
            else
            {
                predictedMeasurement->jacobianMeasurementErrorParameters.jacobianMeasurementMapElementParameters.block<3,3>(dimension_error_measurement_i, dimension_map_element_error_parameters_i)=
                        mat_diff_error_quat_wrt_error_theta.transpose()*inv_mat_q_plus_att_visual_marker_wrt_visual_marker_eye* mat_q_plus_att_world_wrt_visual_marker_eye *mat_q_plus_att_visual_marker_wrt_world*mat_diff_error_quat_wrt_error_theta;
                dimension_map_element_error_parameters_i+=3;
            }

            dimension_error_measurement_i+=3;
        }
    }





    /// Jacobians error measurement - sensor noise of the measurement

    // Resize and init Jacobian
    predictedMeasurement->jacobianMeasurementSensorNoise.jacobianMeasurementSensorNoise.resize(dimensionErrorMeasurement, dimensionErrorMeasurement);
    predictedMeasurement->jacobianMeasurementSensorNoise.jacobianMeasurementSensorNoise.setZero();

    // Fill Jacobian -> Identity
    predictedMeasurement->jacobianMeasurementSensorNoise.jacobianMeasurementSensorNoise=Eigen::MatrixXd::Identity(dimensionErrorMeasurement, dimensionErrorMeasurement);




    return 0;
}


int CodedVisualMarkerEyeCore::mapMeasurement(const TimeStamp theTimeStamp, const std::shared_ptr<GlobalParametersStateCore> TheGlobalParametersStateCore, const std::shared_ptr<RobotStateCore> currentRobotState, const std::shared_ptr<SensorStateCore> currentSensorState, const std::shared_ptr<SensorMeasurementCore> matchedMeasurement, std::shared_ptr<MapElementCore>& newMapElementCore, std::shared_ptr<MapElementStateCore>& newMapElementState)
{
    // Checks

    // Global Parameters State
    if(!TheGlobalParametersStateCore)
        return -10;
    if(!TheGlobalParametersStateCore->getTheGlobalParametersCore())
        return -11;

    // Current Robot State
    if(!currentRobotState)
        return -20;
    if(!currentRobotState->getTheRobotCore())
        return -21;
    // Cast
    std::shared_ptr<FreeModelRobotStateCore> TheRobotState=std::dynamic_pointer_cast<FreeModelRobotStateCore>(currentRobotState);

    // Sensor State
    if(!currentSensorState)
        return 30;
    // Cast
    std::shared_ptr<CodedVisualMarkerEyeStateCore> TheSensorState=std::dynamic_pointer_cast<CodedVisualMarkerEyeStateCore>(currentSensorState);

    // Sensor Core
    if(!currentSensorState->getTheSensorCore())
        return 31;
    // Cast
    std::shared_ptr<const CodedVisualMarkerEyeCore> TheSensorCore=std::dynamic_pointer_cast<const CodedVisualMarkerEyeCore>(currentSensorState->getTheSensorCore());

    // Matched Measurement
    if(!matchedMeasurement)
        return -1;
    // Cast
    std::shared_ptr<CodedVisualMarkerMeasurementCore> TheCodedVisualMarkerMeasurement=std::dynamic_pointer_cast<CodedVisualMarkerMeasurementCore>(matchedMeasurement);


    // Create Map Element Core
    std::shared_ptr<CodedVisualMarkerLandmarkCore> TheCodeCodedVisualMarkerLandmarkCore;
    if(!TheCodeCodedVisualMarkerLandmarkCore)
        TheCodeCodedVisualMarkerLandmarkCore=std::make_shared<CodedVisualMarkerLandmarkCore>();
    else
        TheCodeCodedVisualMarkerLandmarkCore=std::dynamic_pointer_cast<CodedVisualMarkerLandmarkCore>(newMapElementCore);

    // Set id in the map element core
    if(TheCodeCodedVisualMarkerLandmarkCore->setId(TheCodedVisualMarkerMeasurement->getVisualMarkerId()))
        return 2;

    // Set name
    if(TheCodeCodedVisualMarkerLandmarkCore->setMapElementName("visual_marker_"+std::to_string(TheCodeCodedVisualMarkerLandmarkCore->getId())))
        return 3;


    // Set Default configurations
    // Enable Estimation Position if measurement pos is enabled
    // TODO Check!
    if(TheSensorCore->isMeasurementPositionEnabled())
        TheCodeCodedVisualMarkerLandmarkCore->enableEstimationPositionVisualMarkerWrtWorld();
    else
        TheCodeCodedVisualMarkerLandmarkCore->enableParameterPositionVisualMarkerWrtWorld();

    // Enable Estimation Attitude if measurement att is enabled
    // TODO Check!
    if(TheSensorCore->isMeasurementAttitudeEnabled())
        TheCodeCodedVisualMarkerLandmarkCore->enableEstimationAttitudeVisualMarkerWrtWorld();
    else
        TheCodeCodedVisualMarkerLandmarkCore->enableParameterAttitudeVisualMarkerWrtWorld();

    // Covariances not needed because are included in P.
    // TODO check!


    // Create Map Element State Core With Map Element Core
    std::shared_ptr<CodedVisualMarkerLandmarkStateCore> TheCodedVisualMarkerLandmarkStateCore;
    if(!TheCodedVisualMarkerLandmarkStateCore)
        TheCodedVisualMarkerLandmarkStateCore=std::make_shared<CodedVisualMarkerLandmarkStateCore>(TheCodeCodedVisualMarkerLandmarkCore);
    else
        TheCodedVisualMarkerLandmarkStateCore=std::dynamic_pointer_cast<CodedVisualMarkerLandmarkStateCore>(newMapElementState);


    /// State


    // Position
    // TODO Check measurements set!
    Eigen::Vector3d tran_visual_marker_wrt_robot=Quaternion::cross_sandwich(TheSensorState->getAttitudeSensorWrtRobot(), TheCodedVisualMarkerMeasurement->getVisualMarkerPosition(), Quaternion::inv(TheSensorState->getAttitudeSensorWrtRobot())) + TheSensorState->getPositionSensorWrtRobot();
    TheCodedVisualMarkerLandmarkStateCore->position_=Quaternion::cross_sandwich(TheRobotState->getAttitude(), tran_visual_marker_wrt_robot, Quaternion::inv(TheRobotState->getAttitude())) + TheRobotState->getPosition();


    // Attitude
    // TODO Check measurements set!
    TheCodedVisualMarkerLandmarkStateCore->attitude_=Quaternion::cross(TheRobotState->getAttitude(), TheSensorState->getAttitudeSensorWrtRobot(), TheCodedVisualMarkerMeasurement->getVisualMarkerAttitude());



    // Polymorph
    newMapElementCore=TheCodeCodedVisualMarkerLandmarkCore;
    newMapElementState=TheCodedVisualMarkerLandmarkStateCore;

    // End
    return 0;
}


int CodedVisualMarkerEyeCore::jacobiansMapMeasurement(const TimeStamp theTimeStamp, const std::shared_ptr<GlobalParametersStateCore> TheGlobalParametersStateCore, const std::shared_ptr<RobotStateCore> currentRobotState, const std::shared_ptr<SensorStateCore> currentSensorState, const std::shared_ptr<SensorMeasurementCore> matchedMeasurement, std::shared_ptr<MapElementStateCore>& newMapElementState)
{
    // Checks

    // Global Parameters State
    if(!TheGlobalParametersStateCore)
        return -10;
    if(!TheGlobalParametersStateCore->getTheGlobalParametersCore())
        return -11;

    // Current Robot State
    if(!currentRobotState)
        return -20;
    if(!currentRobotState->getTheRobotCore())
        return -21;
    // Cast
    std::shared_ptr<FreeModelRobotStateCore> TheRobotState=std::dynamic_pointer_cast<FreeModelRobotStateCore>(currentRobotState);

    // Sensor State
    if(!currentSensorState)
        return 30;
    // Cast
    std::shared_ptr<CodedVisualMarkerEyeStateCore> TheSensorState=std::dynamic_pointer_cast<CodedVisualMarkerEyeStateCore>(currentSensorState);

    // Sensor Core
    if(!currentSensorState->getTheSensorCore())
        return 31;
    // Cast
    std::shared_ptr<const CodedVisualMarkerEyeCore> TheSensorCore=std::dynamic_pointer_cast<const CodedVisualMarkerEyeCore>(currentSensorState->getTheSensorCore());

    // Matched Measurement
    if(!matchedMeasurement)
        return -1;
    // Cast
    std::shared_ptr<CodedVisualMarkerMeasurementCore> TheCodedVisualMarkerMeasurement=std::dynamic_pointer_cast<CodedVisualMarkerMeasurementCore>(matchedMeasurement);


    // Create Map Element State Core
    if(!newMapElementState)
        return -50;
    // Cast
    std::shared_ptr<CodedVisualMarkerLandmarkStateCore> TheCodedVisualMarkerLandmarkStateCore=std::dynamic_pointer_cast<CodedVisualMarkerLandmarkStateCore>(newMapElementState);


    // Create Map Element Core
    if(!newMapElementState->getTheMapElementCore())
        return -51;
    std::shared_ptr<CodedVisualMarkerLandmarkCore> TheCodeCodedVisualMarkerLandmarkCore=std::dynamic_pointer_cast<CodedVisualMarkerLandmarkCore>(newMapElementState->getTheMapElementCore());




    //// Jacobian

    int dimension_map_new_element_error_state_total=TheCodeCodedVisualMarkerLandmarkCore->getDimensionErrorState();



    // Aux vars
    Eigen::Vector3d tran_visual_marker_wrt_robot=Quaternion::cross_sandwich(TheSensorState->getAttitudeSensorWrtRobot(), TheCodedVisualMarkerMeasurement->getVisualMarkerPosition(), Quaternion::inv(TheSensorState->getAttitudeSensorWrtRobot()))+TheSensorState->getPositionSensorWrtRobot();

    Eigen::Vector4d att_visual_marker_eye_wrt_world=Quaternion::cross(TheRobotState->getAttitude(), TheSensorState->getAttitudeSensorWrtRobot());


    Eigen::Matrix4d mat_quat_plus_att_visual_marker_wrt_world=Quaternion::quatMatPlus(TheCodedVisualMarkerLandmarkStateCore->getAttitude());
    Eigen::Matrix4d mat_inv_quat_plus_att_visual_marker_wrt_world=mat_quat_plus_att_visual_marker_wrt_world.inverse();

    Eigen::Matrix4d mat_quat_minus_att_visual_marker_wrt_visual_marker_eye=Quaternion::quatMatMinus(TheCodedVisualMarkerMeasurement->getVisualMarkerAttitude());
    Eigen::Matrix4d mat_quat_plus_att_visual_marker_wrt_visual_marker_eye=Quaternion::quatMatPlus(TheCodedVisualMarkerMeasurement->getVisualMarkerAttitude());

    Eigen::Matrix4d mat_quat_minus_att_visual_marker_eye_wrt_robot=Quaternion::quatMatMinus(TheSensorState->getAttitudeSensorWrtRobot());
    Eigen::Matrix4d mat_quat_plus_att_visual_marker_eye_wrt_robot=Quaternion::quatMatPlus(TheSensorState->getAttitudeSensorWrtRobot());

    Eigen::Matrix4d mat_quat_minus_att_world_wrt_visual_marker_eye=Quaternion::quatMatMinus(Quaternion::inv(att_visual_marker_eye_wrt_world));

    Eigen::Matrix4d mat_quat_plus_att_robot_wrt_world=Quaternion::quatMatPlus(TheRobotState->getAttitude());

    Eigen::Matrix4d mat_quat_minus_att_world_wrt_robot=Quaternion::quatMatMinus(Quaternion::inv(TheRobotState->getAttitude()));

    Eigen::Matrix4d mat_quat_plus_att_visual_marker_eye_wrt_world=Quaternion::quatMatPlus(att_visual_marker_eye_wrt_world);

    Eigen::Matrix4d mat_quat_plus_tran_visual_marker_wrt_robot=Quaternion::quatMatPlus(tran_visual_marker_wrt_robot);

    Eigen::Matrix4d mat_quat_plus_tran_visual_marker_wrt_visual_marker_eye=Quaternion::quatMatPlus(TheCodedVisualMarkerMeasurement->getVisualMarkerPosition());

    Eigen::Matrix4d mat_quat_minus_aux1=Quaternion::quatMatMinus(Quaternion::cross_pure_gen(tran_visual_marker_wrt_robot, TheRobotState->getAttitude()));
    Eigen::Matrix4d mat_quat_minus_aux2=Quaternion::quatMatMinus(Quaternion::cross_pure_gen(TheCodedVisualMarkerMeasurement->getVisualMarkerPosition(), att_visual_marker_eye_wrt_world));


    Eigen::Matrix4d mat_diff_quat_inv_wrt_quat;
    mat_diff_quat_inv_wrt_quat<<1, 0, 0, 0,
                                0, -1, 0, 0,
                                0, 0, -1, 0,
                                0, 0, 0, -1;

    Eigen::MatrixXd mat_diff_error_quat_wrt_error_theta(4,3);
    mat_diff_error_quat_wrt_error_theta<<0, 0, 0,
                                        1, 0, 0,
                                        0, 1, 0,
                                        0, 0, 1;

    Eigen::MatrixXd mat_diff_vector_wrt_vector_amp(3,4);
    mat_diff_vector_wrt_vector_amp<<0, 1, 0, 0,
                                    0, 0, 1, 0,
                                    0, 0, 0, 1;




    //// Jacobian Mapping Error-State


    /// Robot
    int dimension_robot_error_state_total=TheRobotState->getTheRobotCore()->getDimensionErrorState();
    TheCodedVisualMarkerLandmarkStateCore->jacobian_mapping_error_state_.jacobian_mapping_robot_error_state_.resize(dimension_map_new_element_error_state_total, dimension_robot_error_state_total);
    TheCodedVisualMarkerLandmarkStateCore->jacobian_mapping_error_state_.jacobian_mapping_robot_error_state_.setZero();


    // tran landmark -> Enabled by default -> TODO Check

        // tran robot
        TheCodedVisualMarkerLandmarkStateCore->jacobian_mapping_error_state_.jacobian_mapping_robot_error_state_.block<3,3>(0,0)=
                Eigen::Matrix3d::Identity(3,3);

        // lin vel robot
        // Zeros

        // lin acc robot
        // Zeros

        // attitude robot
        TheCodedVisualMarkerLandmarkStateCore->jacobian_mapping_error_state_.jacobian_mapping_robot_error_state_.block<3,3>(0,9)=
                mat_diff_vector_wrt_vector_amp*(mat_quat_minus_aux1*mat_diff_quat_inv_wrt_quat + mat_quat_plus_att_robot_wrt_world*mat_quat_plus_tran_visual_marker_wrt_robot  )*mat_quat_plus_att_robot_wrt_world*0.5*mat_diff_error_quat_wrt_error_theta;

        // ang vel robot
        // Zeros

        // ang acc robot
        // Zeros


    // atti landmark -> Enabled by default -> TODO Check

        // tran robot
        // Zeros

        // lin vel robot
        // Zeros

        // lin acc robot
        // Zeros

        // attitude robot
        TheCodedVisualMarkerLandmarkStateCore->jacobian_mapping_error_state_.jacobian_mapping_robot_error_state_.block<3,3>(3,9)=
            mat_diff_error_quat_wrt_error_theta.transpose()*mat_inv_quat_plus_att_visual_marker_wrt_world*mat_quat_minus_att_visual_marker_wrt_visual_marker_eye*mat_quat_minus_att_visual_marker_eye_wrt_robot* mat_quat_plus_att_robot_wrt_world *mat_diff_error_quat_wrt_error_theta;

        // ang vel robot
        // Zeros

        // ang acc robot
        // Zeros




    /// Global Parameters
    int dimension_global_parameters_error_state_total=TheGlobalParametersStateCore->getTheGlobalParametersCore()->getDimensionErrorState();
    TheCodedVisualMarkerLandmarkStateCore->jacobian_mapping_error_state_.jacobian_mapping_global_parameters_error_state_.resize(dimension_map_new_element_error_state_total, dimension_global_parameters_error_state_total);
    TheCodedVisualMarkerLandmarkStateCore->jacobian_mapping_error_state_.jacobian_mapping_global_parameters_error_state_.setZero();


    // Zeros -> Do nothing



    /// Sensor
    int dimension_sensor_error_state_total=TheSensorState->getTheSensorCore()->getDimensionErrorState();
    TheCodedVisualMarkerLandmarkStateCore->jacobian_mapping_error_state_.jacobian_mapping_sensor_error_state_.resize(dimension_map_new_element_error_state_total, dimension_sensor_error_state_total);
    TheCodedVisualMarkerLandmarkStateCore->jacobian_mapping_error_state_.jacobian_mapping_sensor_error_state_.setZero();


    {
        int dimension_map_new_element_error_state_i=0;

        // tran landmark -> Enabled by default -> TODO Check
        {
            int dimension_error_state_i=0;

            // Posi sensor wrt robot
            if(TheSensorCore->isEstimationPositionSensorWrtRobotEnabled())
            {
                TheCodedVisualMarkerLandmarkStateCore->jacobian_mapping_error_state_.jacobian_mapping_sensor_error_state_.block<3,3>(dimension_map_new_element_error_state_i, dimension_error_state_i)=
                    mat_diff_vector_wrt_vector_amp*mat_quat_plus_att_robot_wrt_world* mat_quat_minus_att_world_wrt_robot *mat_diff_vector_wrt_vector_amp.transpose();
                dimension_error_state_i+=3;
            }

            // Atti sensor wrt robot
            if(TheSensorCore->isEstimationAttitudeSensorWrtRobotEnabled())
            {
                TheCodedVisualMarkerLandmarkStateCore->jacobian_mapping_error_state_.jacobian_mapping_sensor_error_state_.block<3,3>(dimension_map_new_element_error_state_i, dimension_error_state_i)=
                    mat_diff_vector_wrt_vector_amp*( mat_quat_minus_aux2 *mat_diff_quat_inv_wrt_quat + mat_quat_plus_att_visual_marker_eye_wrt_world* mat_quat_plus_tran_visual_marker_wrt_visual_marker_eye )*mat_quat_plus_att_robot_wrt_world*mat_quat_plus_att_visual_marker_eye_wrt_robot*0.5*mat_diff_error_quat_wrt_error_theta;
                dimension_error_state_i+=3;
            }
            dimension_map_new_element_error_state_i+=3;
        }


        // atti landmark -> Enabled by default -> TODO Check
        {
            int dimension_error_state_i=0;

            // Posi sensor wrt robot
            if(TheSensorCore->isEstimationPositionSensorWrtRobotEnabled())
            {
                // Zeros
                dimension_error_state_i+=3;
            }

            // Atti sensor wrt robot
            if(TheSensorCore->isEstimationAttitudeSensorWrtRobotEnabled())
            {
                TheCodedVisualMarkerLandmarkStateCore->jacobian_mapping_error_state_.jacobian_mapping_sensor_error_state_.block<3,3>(dimension_map_new_element_error_state_i, dimension_error_state_i)=
                    mat_diff_error_quat_wrt_error_theta.transpose()*mat_inv_quat_plus_att_visual_marker_wrt_world*mat_quat_plus_att_robot_wrt_world*mat_quat_minus_att_visual_marker_wrt_visual_marker_eye* mat_quat_plus_att_visual_marker_eye_wrt_robot *mat_diff_error_quat_wrt_error_theta;
                dimension_error_state_i+=3;
            }
            dimension_map_new_element_error_state_i+=3;
        }
    }




    //// Jacobian Mapping Error-State Noise

    int dimension_map_new_element_measurement=TheCodedVisualMarkerMeasurement->getTheSensorCore()->getDimensionErrorMeasurement();

    TheCodedVisualMarkerLandmarkStateCore->jacobian_mapping_error_state_noise_.resize(dimension_map_new_element_error_state_total, dimension_map_new_element_measurement);
    TheCodedVisualMarkerLandmarkStateCore->jacobian_mapping_error_state_noise_.setZero();


    {
        int dimension_map_new_element_error_state_i=0;

        // tran landmark -> Enabled by default -> TODO Check
        {
            int dimension_measurement_i=0;
            // tran meas
            if(TheSensorCore->isMeasurementPositionEnabled())
            {
                TheCodedVisualMarkerLandmarkStateCore->jacobian_mapping_error_state_noise_.block<3,3>(dimension_map_new_element_error_state_i, dimension_measurement_i)=
                     mat_diff_vector_wrt_vector_amp* mat_quat_plus_att_visual_marker_eye_wrt_world* mat_quat_minus_att_world_wrt_visual_marker_eye *mat_diff_vector_wrt_vector_amp.transpose();
                dimension_measurement_i+=3;
            }

            // Atti meas
            if(TheSensorCore->isMeasurementAttitudeEnabled())
            {
                // Zeros
                dimension_measurement_i+=3;
            }

            dimension_map_new_element_error_state_i+=3;
        }


        // atti landmark -> Enabled by default -> TODO Check
        {
            int dimension_measurement_i=0;

            // tran meas
            if(TheSensorCore->isMeasurementPositionEnabled())
            {
                // Zeros
                dimension_measurement_i+=3;
            }

            // Atti meas
            if(TheSensorCore->isMeasurementAttitudeEnabled())
            {
                TheCodedVisualMarkerLandmarkStateCore->jacobian_mapping_error_state_noise_.block<3,3>(dimension_map_new_element_error_state_i, dimension_measurement_i)=
                    mat_diff_error_quat_wrt_error_theta.transpose()*mat_inv_quat_plus_att_visual_marker_wrt_world*mat_quat_plus_att_visual_marker_eye_wrt_world*mat_quat_plus_att_visual_marker_wrt_visual_marker_eye  *mat_diff_error_quat_wrt_error_theta;
                dimension_measurement_i+=3;
            }

            dimension_map_new_element_error_state_i+=3;
        }
    }




    /// End
    newMapElementState=TheCodedVisualMarkerLandmarkStateCore;

    return 0;
}
