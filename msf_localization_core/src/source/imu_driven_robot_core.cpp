
#include "msf_localization_core/imu_driven_robot_core.h"


ImuDrivenRobotCore::ImuDrivenRobotCore() :
    RobotCore()
{
    init();

    return;
}

ImuDrivenRobotCore::ImuDrivenRobotCore(MsfLocalizationCore *msf_localization_core_ptr) :
    RobotCore(msf_localization_core_ptr)
{
    init();

    return;
}

ImuDrivenRobotCore::~ImuDrivenRobotCore()
{

}

int ImuDrivenRobotCore::init()
{
    // Dimensions
    dimension_state_=9+7;
    dimension_error_state_=9+6;

    dimension_parameters_=0;
    dimension_error_parameters_=0;

    dimension_noise_=0;


    // Noises
    this->noise_position_robot_wrt_world_.setZero();
    this->noise_linear_speed_robot_wrt_world_.setZero();
    this->noise_linear_acceleration_robot_wrt_world_.setZero();
    this->noise_attitude_robot_wrt_world_.setZero();
    this->noise_angular_velocity_robot_wrt_world_.setZero();


    // Type
    this->setRobotCoreType(RobotCoreTypes::imu_driven);


    // End
    return 0;
}

int ImuDrivenRobotCore::readConfig(const pugi::xml_node& robot, std::shared_ptr<ImuDrivenRobotStateCore>& robot_init_state)
{
    // Create a class for the RobotStateCore
    if(!robot_init_state)
        robot_init_state=std::make_shared<ImuDrivenRobotStateCore>(this->getMsfElementCoreWeakPtr());


    // Aux vars
    std::string readingValue;


    /// Inputs
    readingValue=robot.child_value("input_list");
    if(!readingValue.empty())
    {
        std::istringstream stm(readingValue);
        std::list<int> input_ids;
        int read_id;
        while(stm>>read_id)
        {
            input_ids.push_back(read_id);
        }
        this->setInputIds(input_ids);
    }


    /// Init State

    // Position
    readingValue=robot.child("position").child_value("init_estimation");
    if(!readingValue.empty())
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d position;
        stm>>position[0]>>position[1]>>position[2];
        robot_init_state->setPositionRobotWrtWorld(position);
    }

    // Linear Speed
    readingValue=robot.child("lin_speed").child_value("init_estimation");
    if(!readingValue.empty())
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d lin_speed;
        stm>>lin_speed[0]>>lin_speed[1]>>lin_speed[2];
        robot_init_state->setLinearSpeedRobotWrtWorld(lin_speed);
    }

    // Linear Acceleration
    readingValue=robot.child("lin_accel").child_value("init_estimation");
    if(!readingValue.empty())
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d lin_accel;
        stm>>lin_accel[0]>>lin_accel[1]>>lin_accel[2];
        robot_init_state->setLinearAccelerationRobotWrtWorld(lin_accel);
    }

    // Attitude
    readingValue=robot.child("attitude").child_value("init_estimation");
    if(!readingValue.empty())
    {
        std::istringstream stm(readingValue);
        Eigen::Vector4d attitude;
        stm>>attitude[0]>>attitude[1]>>attitude[2]>>attitude[3];
        robot_init_state->setAttitudeRobotWrtWorld(attitude);
    }

    // Angular Velocity
    readingValue=robot.child("ang_velocity").child_value("init_estimation");
    if(!readingValue.empty())
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d ang_velocity;
        stm>>ang_velocity[0]>>ang_velocity[1]>>ang_velocity[2];
        robot_init_state->setAngularVelocityRobotWrtWorld(ang_velocity);
    }



    /// State / Parameters Covariances

    // Position
    readingValue=robot.child("position").child_value("init_var");
    if(!readingValue.empty())
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoisePositionRobotWrtWorld(variance.asDiagonal());
    }

    // Linear Speed
    readingValue=robot.child("lin_speed").child_value("init_var");
    if(!readingValue.empty())
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoiseLinearSpeedRobotWrtWorld(variance.asDiagonal());
    }

    // Linear Acceleration
    readingValue=robot.child("lin_accel").child_value("init_var");
    if(!readingValue.empty())
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoiseLinearAccelerationRobotWrtWorld(variance.asDiagonal());
    }

    // Attitude
    readingValue=robot.child("attitude").child_value("init_var");
    if(!readingValue.empty())
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoiseAttitudeRobotWrtWorld(variance.asDiagonal());
    }

    // Angular Velocity
    readingValue=robot.child("ang_velocity").child_value("init_var");
    if(!readingValue.empty())
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoiseAngularVelocityRobotWrtWorld(variance.asDiagonal());
    }


    // Prepare Covariance of init error state
    this->prepareCovarianceInitErrorState();



    /// Noises Estimation

    // Nothing



    // End
    return 0;
}

int ImuDrivenRobotCore::setNoisePositionRobotWrtWorld(const Eigen::Matrix3d &noise_position_robot_wrt_world)
{
    this->noise_position_robot_wrt_world_=noise_position_robot_wrt_world;
    return 0;
}

Eigen::Matrix3d ImuDrivenRobotCore::getNoisePositionRobotWrtWorld()
{
    return this->noise_position_robot_wrt_world_;
}

int ImuDrivenRobotCore::setNoiseLinearSpeedRobotWrtWorld(const Eigen::Matrix3d &noise_linear_speed_robot_wrt_world)
{
    this->noise_linear_speed_robot_wrt_world_=noise_linear_speed_robot_wrt_world;
    return 0;
}

Eigen::Matrix3d ImuDrivenRobotCore::getNoiseLinearSpeedRobotWrtWorld()
{
    return this->noise_linear_speed_robot_wrt_world_;
}

int ImuDrivenRobotCore::setNoiseLinearAccelerationRobotWrtWorld(const Eigen::Matrix3d &noise_linear_acceleration_robot_wrt_world)
{
    this->noise_linear_acceleration_robot_wrt_world_=noise_linear_acceleration_robot_wrt_world;
    return 0;
}

Eigen::Matrix3d ImuDrivenRobotCore::getNoiseLinearAccelerationRobotWrtWorld()
{
    return this->noise_linear_acceleration_robot_wrt_world_;
}

int ImuDrivenRobotCore::setNoiseAttitudeRobotWrtWorld(const Eigen::Matrix3d &noise_attitude_robot_wrt_world)
{
    this->noise_attitude_robot_wrt_world_=noise_attitude_robot_wrt_world;
    return 0;
}

Eigen::Matrix3d ImuDrivenRobotCore::getNoiseAttitudeRobotWrtWorld()
{
    return this->noise_attitude_robot_wrt_world_;
}

int ImuDrivenRobotCore::setNoiseAngularVelocityRobotWrtWorld(const Eigen::Matrix3d &noise_angular_velocity_robot_wrt_world)
{
    this->noise_angular_velocity_robot_wrt_world_=noise_angular_velocity_robot_wrt_world;
    return 0;
}

Eigen::Matrix3d ImuDrivenRobotCore::getNoiseAngularVelocityRobotWrtWorld()
{
    return this->noise_angular_velocity_robot_wrt_world_;
}

Eigen::SparseMatrix<double> ImuDrivenRobotCore::getCovarianceParameters()
{
    // No parameters

    return Eigen::SparseMatrix<double>();
}

Eigen::SparseMatrix<double> ImuDrivenRobotCore::getCovarianceNoise(const TimeStamp deltaTimeStamp)
{
    Eigen::SparseMatrix<double> covariance_noise;

    // Resize
    covariance_noise.resize(dimension_noise_, dimension_noise_);
    //covariance_noise.setZero();
    covariance_noise.reserve(dimension_noise_);

    std::vector<Eigen::Triplet<double> > tripletList;

    //
    double dt=deltaTimeStamp.get_double();

    int dimension_noise_i=0;

    // Fill

    // Nothing


    covariance_noise.setFromTriplets(tripletList.begin(), tripletList.end());


    // End
    return covariance_noise;
}

int ImuDrivenRobotCore::prepareCovarianceInitErrorStateSpecific()
{

    this->covariance_init_error_state_.block<3,3>(0,0)=this->getNoisePositionRobotWrtWorld();
    this->covariance_init_error_state_.block<3,3>(3,3)=this->getNoiseLinearSpeedRobotWrtWorld();
    this->covariance_init_error_state_.block<3,3>(6,6)=this->getNoiseLinearAccelerationRobotWrtWorld();
    this->covariance_init_error_state_.block<3,3>(9,9)=this->getNoiseAttitudeRobotWrtWorld();
    this->covariance_init_error_state_.block<3,3>(12,12)=this->getNoiseAngularVelocityRobotWrtWorld();

    return 0;
}

int ImuDrivenRobotCore::predictState(//Time
                                     const TimeStamp &previousTimeStamp,
                                     const TimeStamp &currentTimeStamp,
                                     // Previous State
                                     const std::shared_ptr<StateComponent> &pastState,
                                     // Inputs
                                     const std::shared_ptr<InputCommandComponent> &inputCommand,
                                     // Predicted State
                                     std::shared_ptr<StateCore> &predictedState)
{
    // Checks

    // Past State
    if(!pastState)
        return -1;

    // TODO

    // Robot Predicted State
    std::shared_ptr<ImuDrivenRobotStateCore> predictedRobotState;
    if(!predictedState)
        predictedRobotState=std::make_shared<ImuDrivenRobotStateCore>(pastState->TheRobotStateCore->getMsfElementCoreWeakPtr());
    else
        predictedRobotState=std::dynamic_pointer_cast<ImuDrivenRobotStateCore>(predictedState);



    // Search for the command
    std::shared_ptr<ImuInputCommandCore> imu_input_command;
    // TODO Improve
    for(std::list< std::shared_ptr<InputCommandCore> >::iterator itInputCommand=inputCommand->list_input_command_core_.begin();
        itInputCommand != inputCommand->list_input_command_core_.end();
        ++itInputCommand)
    {
        if((*itInputCommand)->getInputCommandType()==InputCommandTypes::imu)
        {
            // Iterators cannot be dynamic cast!
            imu_input_command=std::static_pointer_cast<ImuInputCommandCore>(*itInputCommand);
            break;
        }
    }

    if(!imu_input_command)
        return -10;




    // Predict State
    int error_predict_state=predictStateSpecific(previousTimeStamp, currentTimeStamp,
                                                std::dynamic_pointer_cast<ImuDrivenRobotStateCore>(pastState->TheRobotStateCore),
                                                imu_input_command,
                                                predictedRobotState);

    // Check error
    if(error_predict_state)
        return error_predict_state;


    // Set predicted state
    predictedState=predictedRobotState;


    // End
    return 0;
}

int ImuDrivenRobotCore::predictStateSpecific(const TimeStamp& previousTimeStamp, const TimeStamp& currentTimeStamp,
                                             const std::shared_ptr<ImuDrivenRobotStateCore>& pastState,
                                             const std::shared_ptr<ImuInputCommandCore>& input,
                                             std::shared_ptr<ImuDrivenRobotStateCore>& predictedState)
{
    //std::cout<<"ImuDrivenRobotCore::predictStateSpecific"<<std::endl;

    // Polymorph
    //std::shared_ptr<ImuDrivenRobotStateCore> pastState=std::dynamic_pointer_cast<ImuDrivenRobotStateCore>(pastStateI);
    //std::shared_ptr<ImuDrivenRobotStateCore> predictedState=std::dynamic_pointer_cast<ImuDrivenRobotStateCore>(predictedStateI);

//std::cout<<"Input TS: sec="<<currentTimeStamp.sec<<" s; nsec="<<currentTimeStamp.nsec<<" ns."<<std::endl;
//std::cout<<"\t + a: "<<input->getLinearAcceleration().transpose()<<std::endl;
//std::cout<<"\t + w: "<<input->getAngularVelocity().transpose()<<std::endl;

    // Checks in the past state
    if(!pastState->isCorrect())
    {
        std::cout<<"ImuDrivenRobotCore::predictStateSpecific() error !pastState"<<std::endl;
        return -5;
    }


    // Create the predicted state if it doesn't exist
    if(!predictedState)
    {
        predictedState=std::make_shared<ImuDrivenRobotStateCore>(pastState->getMsfElementCoreWeakPtr());
    }





    // Equations


    //Delta Time
    TimeStamp DeltaTime=currentTimeStamp-previousTimeStamp;
    double dt=DeltaTime.get_double();


    /// Position
    predictedState->position_robot_wrt_world_=pastState->position_robot_wrt_world_+pastState->linear_speed_robot_wrt_world_*dt+0.5*pastState->linear_acceleration_robot_wrt_world_*dt*dt;


    /// Linear Speed
    predictedState->linear_speed_robot_wrt_world_=pastState->linear_speed_robot_wrt_world_+pastState->linear_acceleration_robot_wrt_world_*dt;


    /// Linear Acceleration
    predictedState->linear_acceleration_robot_wrt_world_=pastState->linear_acceleration_robot_wrt_world_;


    /// Attitude

    // deltaQw

    Eigen::Vector3d w_mean_dt=(pastState->angular_velocity_robot_wrt_world_)*dt;
    Eigen::Vector4d rotation_w_mean_dt_to_quat;
    rotation_w_mean_dt_to_quat=Quaternion::rotationVectorToQuaternion(w_mean_dt);
    rotation_w_mean_dt_to_quat=rotation_w_mean_dt_to_quat/rotation_w_mean_dt_to_quat.norm();

    //std::cout<<"w_mean_dt="<<w_mean_dt<<std::endl;
    //std::cout<<"rotation_w_mean_dt_to_quat="<<rotation_w_mean_dt_to_quat<<std::endl;


    // prediction
    //predictedState->attitude=Quaternion::cross(rotation_w_mean_dt_to_quat + pow(dt,2)/24*deltaQalpha, pastState->attitude);

    Eigen::Vector4d quat_perturbation=rotation_w_mean_dt_to_quat;

    //std::cout<<"quat_perturbation="<<quat_perturbation<<std::endl;

    Eigen::Vector4d predictedAttitude=Quaternion::cross(quat_perturbation, pastState->attitude_robot_wrt_world_);


    // Unit quaternion -> Very needed!

    //predictedState->attitude=predictedAttitude/predictedAttitude.norm();
//std::cout<<"predictedState->attitude="<<predictedState->attitude.transpose()<<std::endl;


    // Quaternion unit (very needed)
    predictedState->attitude_robot_wrt_world_=predictedAttitude/predictedAttitude.norm();



    /// Angular Velocity
    predictedState->angular_velocity_robot_wrt_world_=pastState->angular_velocity_robot_wrt_world_;

//std::cout<<"predictedState->angular_velocity="<<predictedState->angular_velocity<<std::endl;




    /// Finish
    //predictedStateI=predictedState;

    return 0;
}

int ImuDrivenRobotCore::predictErrorStateJacobian(//Time
                                                 const TimeStamp& previousTimeStamp, const TimeStamp& currentTimeStamp,
                                                 // Previous State
                                                 const std::shared_ptr<StateComponent>& pastState,
                                                  // Inputs
                                                  const std::shared_ptr<InputCommandComponent>& inputCommand,
                                                 // Predicted State
                                                 std::shared_ptr<StateCore> &predictedState)
{
    // Checks

    // Past State
    if(!pastState)
        return -1;

    // TODO


    // Predicted State
    if(!predictedState)
        return -1;

    // Robot Predicted State
    std::shared_ptr<ImuDrivenRobotStateCore> predictedRobotState=std::dynamic_pointer_cast<ImuDrivenRobotStateCore>(predictedState);



    // TODO


    // Search for the command
    std::shared_ptr<ImuInputCommandCore> imu_input_command;
    // TODO




    // Predict State
    int error_predict_state=predictErrorStateJacobianSpecific(previousTimeStamp, currentTimeStamp,
                                                            std::dynamic_pointer_cast<ImuDrivenRobotStateCore>(pastState->TheRobotStateCore),
                                                            imu_input_command,
                                                            predictedRobotState);

    // Check error
    if(error_predict_state)
        return error_predict_state;


    // Set predicted state
    predictedState=predictedRobotState;


    // End
    return 0;
}

int ImuDrivenRobotCore::predictErrorStateJacobianSpecific(const TimeStamp& previousTimeStamp, const TimeStamp& currentTimeStamp,
                                                   const std::shared_ptr<ImuDrivenRobotStateCore>& pastState,
                                                   const std::shared_ptr<ImuInputCommandCore>& input,
                                                   std::shared_ptr<ImuDrivenRobotStateCore>& predictedState)
{

    //std::cout<<"ImuDrivenRobotCore::predictErrorStateJacobians"<<std::endl;


    // Polymorph
    //std::shared_ptr<ImuDrivenRobotStateCore> pastState=std::static_pointer_cast<ImuDrivenRobotStateCore>(pastStateI);
    //std::shared_ptr<ImuDrivenRobotStateCore> predictedState=std::static_pointer_cast<ImuDrivenRobotStateCore>(predictedStateI);




    // Create the predicted state if it doesn't exist
    if(!predictedState)
    {
        std::cout<<"ImuDrivenRobotCore::predictErrorStateJacobians error en predictedState"<<std::endl;
        return 1;
    }


    //Delta Time
    TimeStamp DeltaTime=currentTimeStamp-previousTimeStamp;
    // delta time
    double dt=DeltaTime.get_double();


    ///// Jacobian Error State

    // Jacobian Size


    Eigen::SparseMatrix<double> jacobian_error_state;
    jacobian_error_state.resize(this->getDimensionErrorState(),this->getDimensionErrorState());
    jacobian_error_state.reserve(18+36);




    /// Jacobian of the error: Linear Part
    std::vector<Eigen::Triplet<double> > tripletListErrorJacobian;

    // posi / posi
    // Eigen::Matrix3d::Identity(3,3);
    for(int i=0; i<3; i++)
        tripletListErrorJacobian.push_back(Eigen::Triplet<double>(i,i,1));


    // posi / vel
    // Eigen::Matrix3d::Identity(3,3)*dt;
    for(int i=0; i<3; i++)
        tripletListErrorJacobian.push_back(Eigen::Triplet<double>(i,3+i,dt));


    // posi / acc
    // 0.5*Eigen::Matrix3d::Identity(3,3)*pow(dt,2);
    double dt2=pow(dt,2);
    for(int i=0; i<3; i++)
        tripletListErrorJacobian.push_back(Eigen::Triplet<double>(i,i+6,0.5*dt2));



    // vel / posi
    // zero

    // vel / vel
    // Eigen::Matrix3d::Identity(3,3);
    for(int i=0; i<3; i++)
        tripletListErrorJacobian.push_back(Eigen::Triplet<double>(3+i,3+i,1));


    // vel / acc
    // Eigen::Matrix3d::Identity(3,3)*dt;
    for(int i=0; i<3; i++)
        tripletListErrorJacobian.push_back(Eigen::Triplet<double>(3+i,6+i,dt));



    // acc / posi
    // zero

    // acc / vel
    // zero

    // acc / acc
    // Eigen::Matrix3d::Identity(3,3);
    for(int i=0; i<3; i++)
        tripletListErrorJacobian.push_back(Eigen::Triplet<double>(6+i,6+i,1));





    /// Jacobian of the error -> Angular Part


    // Auxiliar values
    Eigen::Vector3d w_mean_dt=(pastState->angular_velocity_robot_wrt_world_)*dt;
    Eigen::Vector4d quat_w_mean_dt=Quaternion::rotationVectorToQuaternion(w_mean_dt);

    // Auxiliar Matrixes
    Eigen::Matrix4d quat_mat_plus_quat_ref_k1=Quaternion::quatMatPlus(predictedState->attitude_robot_wrt_world_);
    Eigen::Matrix4d quat_mat_plus_quat_ref_k1_inv=quat_mat_plus_quat_ref_k1.inverse();
    Eigen::Matrix<double, 4, 3> mat_delta_q_delta_theta;//(4, 3);
    mat_delta_q_delta_theta.setZero();
    mat_delta_q_delta_theta(1,0)=1;
    mat_delta_q_delta_theta(2,1)=1;
    mat_delta_q_delta_theta(3,2)=1;
    Eigen::Matrix4d quat_mat_plus_quat_ref_k=Quaternion::quatMatPlus(pastState->attitude_robot_wrt_world_);
    Eigen::Matrix4d quat_mat_minus_quat_ref_k=Quaternion::quatMatMinus(pastState->attitude_robot_wrt_world_);
    Eigen::Matrix<double, 4, 3> mat_jacobian_w_mean_dt_to_quat=Quaternion::jacobianRotationVectorToQuaternion(w_mean_dt);



    //std::cout<<"w_mean_dt="<<w_mean_dt<<std::endl;
    //std::cout<<"predictedState->angular_velocity="<<predictedState->angular_velocity<<std::endl;

    //std::cout<<"quat_w_mean_dt="<<quat_w_mean_dt<<std::endl;
    //std::cout<<"quat_for_second_order_correction="<<quat_for_second_order_correction<<std::endl;


    // att / att [9 non-zero]
    Eigen::Matrix3d jacobianAttAtt=
            mat_delta_q_delta_theta.transpose()*quat_mat_plus_quat_ref_k1_inv*(Quaternion::quatMatPlus(quat_w_mean_dt))*quat_mat_plus_quat_ref_k*mat_delta_q_delta_theta;
    for(int i=0; i<3; i++)
        for(int j=0; j<3; j++)
            tripletListErrorJacobian.push_back(Eigen::Triplet<double>(9+i,9+j,jacobianAttAtt(i,j)));



    // att / ang_vel [9 non-zero]
    Eigen::Matrix3d jacobianAttAngVel=
            2*mat_delta_q_delta_theta.transpose()*quat_mat_plus_quat_ref_k1_inv*quat_mat_minus_quat_ref_k*(mat_jacobian_w_mean_dt_to_quat*dt);
    for(int i=0; i<3; i++)
        for(int j=0; j<3; j++)
            tripletListErrorJacobian.push_back(Eigen::Triplet<double>(9+i,9+3+j,jacobianAttAngVel(i,j)));




    // ang_vel / att
    // zero

    // ang_vel / ang_vel [3 non-zero]
    // Eigen::Matrix3d::Identity(3,3);
    for(int i=0; i<3; i++)
        tripletListErrorJacobian.push_back(Eigen::Triplet<double>(9+3+i,9+3+i,1));





    jacobian_error_state.setFromTriplets(tripletListErrorJacobian.begin(), tripletListErrorJacobian.end());


    predictedState->setJacobianErrorStateRobot(jacobian_error_state);



    ///// Jacobian Error State Noise

    // Jacobian Size


    predictedState->jacobian_error_state_noise_.resize(dimension_error_state_, dimension_noise_);
    predictedState->jacobian_error_state_noise_.reserve(dimension_noise_);


    // Fill
    std::vector<Eigen::Triplet<double> > tripletListNoiseJacobian;

    int dimension_state_i=0;
    int dimension_noise_i=0;



    // Do nothing



    predictedState->jacobian_error_state_noise_.setFromTriplets(tripletListNoiseJacobian.begin(), tripletListNoiseJacobian.end());



#if _DEBUG_ROBOT_CORE
    {
        std::ostringstream logString;
        logString<<"ImuDrivenRobotCore::predictErrorStateJacobians() for TS: sec="<<currentTimeStamp.sec<<" s; nsec="<<currentTimeStamp.nsec<<" ns"<<std::endl;
        logString<<"Jacobian Error State"<<std::endl;
        logString<<Eigen::MatrixXd(predictedState->jacobian_error_state_)<<std::endl;
        logString<<"Jacobian Error State Noise"<<std::endl;
        logString<<Eigen::MatrixXd(predictedState->jacobian_error_state_noise_)<<std::endl;
        this->log(logString.str());
    }
#endif


    // Finish
    //predictedStateI=predictedState;

    return 0;
}

int ImuDrivenRobotCore::resetErrorStateJacobian(// Time
                                                const TimeStamp& current_time_stamp,
                                                // Increment Error State
                                                const Eigen::VectorXd& increment_error_state,
                                                // Current State
                                                std::shared_ptr<StateCore>& current_state
                                                )
{
    // Checks
    if(!current_state)
        return -1;


    // Resize Jacobian
    current_state->jacobian_error_state_reset_.resize(this->dimension_error_state_, this->dimension_error_state_);

    // Fill
    std::vector< Eigen::Triplet<double> > triplets_jacobian_error_reset;

    int dimension_error_state_i=0;

    // Position
    {
        for(int i=0; i<3; i++)
            triplets_jacobian_error_reset.push_back(Eigen::Triplet<double>(dimension_error_state_i+i, dimension_error_state_i+i, 1.0));

        dimension_error_state_i+=3;
    }

    // Lin speed
    {
        for(int i=0; i<3; i++)
            triplets_jacobian_error_reset.push_back(Eigen::Triplet<double>(dimension_error_state_i+i, dimension_error_state_i+i, 1.0));

        dimension_error_state_i+=3;
    }

    // Lin acceleration
    {
        for(int i=0; i<3; i++)
            triplets_jacobian_error_reset.push_back(Eigen::Triplet<double>(dimension_error_state_i+i, dimension_error_state_i+i, 1.0));

        dimension_error_state_i+=3;
    }

    // Attitude
    {
        // Error Reset Matrixes
        Eigen::Matrix3d G_update_theta_robot=Eigen::Matrix3d::Identity(3,3);


        // Ojo, signo cambiado por la definicion de incrementError!
        G_update_theta_robot-=Quaternion::skewSymMat(0.5*increment_error_state.block<3,1>(dimension_error_state_i,0));

        // Triplets
        BlockMatrix::insertVectorEigenTripletFromEigenDense(triplets_jacobian_error_reset, G_update_theta_robot, dimension_error_state_i, dimension_error_state_i);

        dimension_error_state_i+=3;
    }

    // Angular Veloc
    {
        for(int i=0; i<3; i++)
            triplets_jacobian_error_reset.push_back(Eigen::Triplet<double>(dimension_error_state_i+i, dimension_error_state_i+i, 1.0));

        dimension_error_state_i+=3;
    }


    current_state->jacobian_error_state_reset_.setFromTriplets(triplets_jacobian_error_reset.begin(), triplets_jacobian_error_reset.end());

    // End
    return 0;
}
