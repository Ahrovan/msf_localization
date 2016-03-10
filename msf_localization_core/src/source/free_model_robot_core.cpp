
#include "msf_localization_core/free_model_robot_core.h"


FreeModelRobotCore::FreeModelRobotCore():
    RobotCore()
{
    dimensionState=9+10;
    dimensionErrorState=9+9;

    dimensionParameters=0;
    dimensionErrorParameters=0;

    noiseLinearAcceleration.setZero();
    noiseAngularVelocity.setZero(); // Not used anymore
    noiseAngularAcceleration.setZero();

    // TODO -> This is not the best moment to do this! The state might change!
    InitErrorStateVariance.resize(dimensionErrorState, dimensionErrorState);
    InitErrorStateVariance.setZero();

    return;
}

FreeModelRobotCore::~FreeModelRobotCore()
{
    return;
}


Eigen::Matrix3d FreeModelRobotCore::getNoiseLinearAcceleration() const
{
    return this->noiseLinearAcceleration;
}

int FreeModelRobotCore::setNoiseLinearAcceleration(Eigen::Matrix3d noiseLinearAcceleration)
{
    this->noiseLinearAcceleration=noiseLinearAcceleration;
    return 0;
}

Eigen::Matrix3d FreeModelRobotCore::getNoiseAngularVelocity() const
{
    return this->noiseAngularVelocity;
}

int FreeModelRobotCore::setNoiseAngularVelocity(Eigen::Matrix3d noiseAngularVelocity)
{
    this->noiseAngularVelocity=noiseAngularVelocity;
    return 0;
}


Eigen::Matrix3d FreeModelRobotCore::getNoiseAngularAcceleration() const
{
    return this->noiseAngularAcceleration;
}

int FreeModelRobotCore::setNoiseAngularAcceleration(Eigen::Matrix3d noiseAngularAcceleration)
{
    this->noiseAngularAcceleration=noiseAngularAcceleration;
    return 0;
}

int FreeModelRobotCore::setInitErrorStateVariancePosition(Eigen::Vector3d initVariance)
{
    this->InitErrorStateVariance.block<3,3>(0,0)=initVariance.asDiagonal();
    return 0;
}

int FreeModelRobotCore::setInitErrorStateVarianceLinearSpeed(Eigen::Vector3d initVariance)
{
    this->InitErrorStateVariance.block<3,3>(3,3)=initVariance.asDiagonal();
    return 0;
}

int FreeModelRobotCore::setInitErrorStateVarianceLinearAcceleration(Eigen::Vector3d initVariance)
{
    this->InitErrorStateVariance.block<3,3>(6,6)=initVariance.asDiagonal();
    return 0;
}

int FreeModelRobotCore::setInitErrorStateVarianceAttitude(Eigen::Vector3d initVariance)
{
    this->InitErrorStateVariance.block<3,3>(9,9)=initVariance.asDiagonal();
    return 0;
}

int FreeModelRobotCore::setInitErrorStateVarianceAngularVelocity(Eigen::Vector3d initVariance)
{
    this->InitErrorStateVariance.block<3,3>(12,12)=initVariance.asDiagonal();
    return 0;
}

int FreeModelRobotCore::setInitErrorStateVarianceAngularAcceleration(Eigen::Vector3d initVariance)
{
    this->InitErrorStateVariance.block<3,3>(15,15)=initVariance.asDiagonal();
    return 0;
}

int FreeModelRobotCore::predictState(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, const std::shared_ptr<FreeModelRobotStateCore> pastState, std::shared_ptr<FreeModelRobotStateCore>& predictedState)
{
    //std::cout<<"FreeModelRobotCore::predictState"<<std::endl;

    // Checks in the past state
    if(!pastState->getTheRobotCore())
    {
        return -5;
        std::cout<<"FreeModelRobotCore::predictState() error !pastState->getTheRobotCore()"<<std::endl;
    }


    // Create the predicted state if it doesn't exist
    if(!predictedState)
    {
        predictedState=std::make_shared<FreeModelRobotStateCore>();
    }

    // Set The robot core if it doesn't exist
    if(!predictedState->getTheRobotCore())
    {
        predictedState->setTheRobotCore(pastState->getTheRobotCore());
    }


    // Equations


    //Delta Time
    TimeStamp DeltaTime=currentTimeStamp-previousTimeStamp;
    double dt=DeltaTime.get_double();


    /// Position
    predictedState->position=pastState->position+pastState->linear_speed*dt;


    /// Linear Speed
    predictedState->linear_speed=pastState->linear_speed+pastState->linear_acceleration*dt;


    /// Linear Acceleration
    predictedState->linear_acceleration=pastState->linear_acceleration;


    /// Attitude

    // deltaQw

    Eigen::Vector3d w_mean_dt=(pastState->angular_velocity+0.5*pastState->angular_acceleration*dt)*dt;
    Eigen::Vector4d rotation_w_mean_dt_to_quat;
    rotation_w_mean_dt_to_quat=Quaternion::rotationVectorToQuaternion(w_mean_dt);
    rotation_w_mean_dt_to_quat=rotation_w_mean_dt_to_quat/rotation_w_mean_dt_to_quat.norm();

    //std::cout<<"w_mean_dt="<<w_mean_dt<<std::endl;
    //std::cout<<"rotation_w_mean_dt_to_quat="<<rotation_w_mean_dt_to_quat<<std::endl;


    // deltaQalpha
    Eigen::Vector3d alphaContribution;
    alphaContribution=pastState->angular_velocity;
    alphaContribution=alphaContribution.cross(pastState->angular_velocity+pastState->angular_acceleration*dt);

    Eigen::Vector4d deltaQalpha;
    deltaQalpha[0]=0;
    deltaQalpha.block<3,1>(1,0)=alphaContribution;
    // Unit quaternion -> No needed, NO
    //deltaQalpha=deltaQalpha/deltaQalpha.norm();

    //std::cout<<"deltaQalpha="<<deltaQalpha<<std::endl;

    // prediction
    //predictedState->attitude=Quaternion::cross(rotation_w_mean_dt_to_quat + pow(dt,2)/24*deltaQalpha, pastState->attitude);

    Eigen::Vector4d quat_perturbation=rotation_w_mean_dt_to_quat + pow(dt,2)/24*deltaQalpha;

    //std::cout<<"quat_perturbation="<<quat_perturbation<<std::endl;

    predictedState->attitude=Quaternion::cross(quat_perturbation, pastState->attitude);


    // Unit quaternion -> Very needed!
    predictedState->attitude=predictedState->attitude/predictedState->attitude.norm();



    /// Angular Velocity
    predictedState->angular_velocity=pastState->angular_velocity+pastState->angular_acceleration*dt;

//std::cout<<"predictedState->angular_velocity="<<predictedState->angular_velocity<<std::endl;


    /// Angular Acceleration
    predictedState->angular_acceleration=pastState->angular_acceleration;
//std::cout<<"predictedState->angular_acceleration="<<predictedState->angular_acceleration<<std::endl;


    return 0;
}

// Jacobian
int FreeModelRobotCore::predictStateErrorStateJacobians(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, std::shared_ptr<FreeModelRobotStateCore> pastState, std::shared_ptr<FreeModelRobotStateCore>& predictedState)
{
    //std::cout<<"FreeModelRobotCore::predictStateErrorStateJacobians"<<std::endl;

    // Create the predicted state if it doesn't exist
    if(!predictedState)
    {
        return 1;
    }


    //Delta Time
    TimeStamp DeltaTime=currentTimeStamp-previousTimeStamp;
    // delta time
    double dt=DeltaTime.get_double();


    // Jacobian
    // Jacobian Size
    predictedState->errorStateJacobian.linear.resize(9, 9);
    predictedState->errorStateJacobian.linear.setZero();


    predictedState->errorStateJacobian.angular.resize(9, 9);
    predictedState->errorStateJacobian.angular.setZero();



    /// Jacobian of the error: Linear Part


    // posi / posi
    predictedState->errorStateJacobian.linear.block<3,3>(0,0)=Eigen::MatrixXd::Identity(3,3);

    // posi / vel
    predictedState->errorStateJacobian.linear.block<3,3>(0,3)=Eigen::MatrixXd::Identity(3,3)*dt;

    // posi / acc
    predictedState->errorStateJacobian.linear.block<3,3>(0,6)=0.5*Eigen::MatrixXd::Identity(3,3)*pow(dt,2);



    // vel / posi
    // zero

    // vel / vel
    predictedState->errorStateJacobian.linear.block<3,3>(3,3)=Eigen::MatrixXd::Identity(3,3);

    // vel / acc
    predictedState->errorStateJacobian.linear.block<3,3>(3,6)=Eigen::MatrixXd::Identity(3,3)*dt;



    // acc / posi
    // zero

    // acc / vel
    // zero

    // acc / acc
    predictedState->errorStateJacobian.linear.block<3,3>(6,6)=Eigen::MatrixXd::Identity(3,3);



    /// Jacobian of the error -> Angular Part


    // Auxiliar values
    Eigen::Vector3d w_mean_dt=(pastState->angular_velocity+0.5*pastState->angular_acceleration*dt)*dt;
    Eigen::Vector4d quat_w_mean_dt=Quaternion::rotationVectorToQuaternion(w_mean_dt);
    Eigen::Vector4d quat_for_second_order_correction;
    quat_for_second_order_correction.setZero();
    quat_for_second_order_correction.block<3,1>(1,0)=pastState->angular_velocity.cross(predictedState->angular_velocity);

    // Auxiliar Matrixes
    Eigen::Matrix4d quat_mat_plus_quat_ref_k1=Quaternion::quatMatPlus(predictedState->attitude);
    Eigen::Matrix4d quat_mat_plus_quat_ref_k1_inv=quat_mat_plus_quat_ref_k1.inverse();
    Eigen::MatrixXd mat_delta_q_delta_theta(4, 3);
    mat_delta_q_delta_theta.setZero();
    mat_delta_q_delta_theta(1,0)=1;
    mat_delta_q_delta_theta(2,1)=1;
    mat_delta_q_delta_theta(3,2)=1;
    Eigen::Matrix4d quat_mat_plus_quat_ref_k=Quaternion::quatMatPlus(pastState->attitude);
    Eigen::Matrix4d quat_mat_minus_quat_ref_k=Quaternion::quatMatMinus(pastState->attitude);
    Eigen::MatrixXd mat_jacobian_w_mean_dt_to_quat=Quaternion::jacobianRotationVectorToQuaternion(w_mean_dt);
    Eigen::Matrix3d skew_mat_alpha_dt=Quaternion::skewSymMat(pastState->angular_acceleration*dt);
    Eigen::MatrixXd mat_aux_j2(4,3);
    mat_aux_j2.setZero();
    mat_aux_j2.block<3,3>(1,0)=-skew_mat_alpha_dt;
    Eigen::MatrixXd mat_aux_j3(4,3);
    mat_aux_j3.setZero();
    mat_aux_j3.block<3,3>(1,0)=skew_mat_alpha_dt*dt;


    //std::cout<<"w_mean_dt="<<w_mean_dt<<std::endl;
    //std::cout<<"predictedState->angular_velocity="<<predictedState->angular_velocity<<std::endl;

    //std::cout<<"quat_w_mean_dt="<<quat_w_mean_dt<<std::endl;
    //std::cout<<"quat_for_second_order_correction="<<quat_for_second_order_correction<<std::endl;


    // att / att
    predictedState->errorStateJacobian.angular.block<3,3>(0,0)=mat_delta_q_delta_theta.transpose()*quat_mat_plus_quat_ref_k1_inv*(Quaternion::quatMatPlus(quat_w_mean_dt)+pow(dt,2)/24*Quaternion::quatMatPlus(quat_for_second_order_correction))*quat_mat_plus_quat_ref_k*mat_delta_q_delta_theta;


    // att / ang_vel
    // TODO fix
    predictedState->errorStateJacobian.angular.block<3,3>(0,3)=2*mat_delta_q_delta_theta.transpose()*quat_mat_plus_quat_ref_k1_inv*quat_mat_minus_quat_ref_k*(mat_jacobian_w_mean_dt_to_quat*dt + pow(dt,2)/24*mat_aux_j2);


    // att / ang_accel
    // TODO
    predictedState->errorStateJacobian.angular.block<3,3>(0,6)=2*mat_delta_q_delta_theta.transpose()*quat_mat_plus_quat_ref_k1_inv*quat_mat_minus_quat_ref_k*(mat_jacobian_w_mean_dt_to_quat*0.5*pow(dt,2) + pow(dt,2)/24*mat_aux_j3);



    // ang_vel / att
    // zero

    // ang_vel / ang_vel
    predictedState->errorStateJacobian.angular.block<3,3>(3,3)=Eigen::MatrixXd::Identity(3,3);

    // ang_vel / ang_acc
    predictedState->errorStateJacobian.angular.block<3,3>(3,6)=Eigen::MatrixXd::Identity(3,3)*dt;



    // ang_acc / att
    // zero

    // ang_acc / ang_vel
    // zero

    // ang_acc / ang_acc
    predictedState->errorStateJacobian.angular.block<3,3>(6,6)=Eigen::MatrixXd::Identity(3,3);




#if _DEBUG_ROBOT_CORE
    {
        std::ostringstream logString;
        logString<<"FreeModelRobotCore::predictStateErrorStateJacobians() for TS: sec="<<currentTimeStamp.sec<<" s; nsec="<<currentTimeStamp.nsec<<" ns"<<std::endl;
        logString<<"Jacobian Linear"<<std::endl;
        logString<<predictedState->errorStateJacobian.linear<<std::endl;
        logString<<"Jacobian Angular"<<std::endl;
        logString<<predictedState->errorStateJacobian.angular<<std::endl;
        this->log(logString.str());
    }
#endif




    return 0;
}

