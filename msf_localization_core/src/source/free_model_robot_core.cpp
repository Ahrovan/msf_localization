
#include "msf_localization_core/free_model_robot_core.h"


FreeModelRobotCore::FreeModelRobotCore():
    RobotCore()
{
    dimensionState=9+10;
    dimensionErrorState=9+9;

    dimensionParameters=0;
    dimensionErrorParameters=0;

    dimensionNoise=9+9;

    noisePosition.setZero();
    noiseLinearSpeed.setZero();
    noiseLinearAcceleration.setZero();
    noiseAngularVelocity.setZero();
    noiseAttitude.setZero();
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


Eigen::Matrix3d FreeModelRobotCore::getNoisePosition() const
{
    return this->noisePosition;
}

int FreeModelRobotCore::setNoisePosition(Eigen::Matrix3d noisePosition)
{
    this->noisePosition=noisePosition;
    return 0;
}

Eigen::Matrix3d FreeModelRobotCore::getNoiseLinearSpeed() const
{
    return this->noiseLinearSpeed;
}

int FreeModelRobotCore::setNoiseLinearSpeed(Eigen::Matrix3d noiseLinearSpeed)
{
    this->noiseLinearSpeed=noiseLinearSpeed;
    return 0;
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

Eigen::Matrix3d FreeModelRobotCore::getNoiseAttitude() const
{
    return this->noiseAttitude;
}

int FreeModelRobotCore::setNoiseAttitude(Eigen::Matrix3d noiseAttitude)
{
    this->noiseAttitude=noiseAttitude;
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

Eigen::SparseMatrix<double> FreeModelRobotCore::getCovarianceNoise(const TimeStamp deltaTimeStamp)
{
    Eigen::SparseMatrix<double> covariance_noise;

    // Resize
    covariance_noise.resize(dimensionNoise, dimensionNoise);
    //covariance_noise.setZero();
    covariance_noise.reserve(dimensionNoise);

    std::vector<Eigen::Triplet<double> > tripletList;

    //
    double dt=deltaTimeStamp.get_double();

    int dimension_noise_i=0;

    // Fill

    // Position
    for(int i=0; i<3; i++)
        tripletList.push_back(Eigen::Triplet<double>(dimension_noise_i+i,dimension_noise_i+i,this->noisePosition(i,i)*dt));
    dimension_noise_i+=3;

    // Linear Speed
    for(int i=0; i<3; i++)
        tripletList.push_back(Eigen::Triplet<double>(dimension_noise_i+i,dimension_noise_i+i,this->noiseLinearSpeed(i,i)*dt));
    dimension_noise_i+=3;

    // Linear Acceleration
    for(int i=0; i<3; i++)
        tripletList.push_back(Eigen::Triplet<double>(dimension_noise_i+i,dimension_noise_i+i,this->noiseLinearAcceleration(i,i)*dt));
    dimension_noise_i+=3;



    // Attitude
    for(int i=0; i<3; i++)
        tripletList.push_back(Eigen::Triplet<double>(dimension_noise_i+i,dimension_noise_i+i,this->noiseAttitude(i,i)*dt));
    dimension_noise_i+=3;

    // Angular Velocity
    for(int i=0; i<3; i++)
        tripletList.push_back(Eigen::Triplet<double>(dimension_noise_i+i,dimension_noise_i+i,this->noiseAngularVelocity(i,i)*dt));
    dimension_noise_i+=3;

    // Angular Acceleration
    for(int i=0; i<3; i++)
        tripletList.push_back(Eigen::Triplet<double>(dimension_noise_i+i,dimension_noise_i+i,this->noiseAngularAcceleration(i,i)*dt));
    dimension_noise_i+=3;


    covariance_noise.setFromTriplets(tripletList.begin(), tripletList.end());


//    {
//        std::ostringstream logString;
//        logString<<"covariance_noise"<<std::endl;
//        logString<<covariance_noise<<std::endl;
//        this->log(logString.str());
//    }


    // End
    return covariance_noise;
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

int FreeModelRobotCore::predictState(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, const std::shared_ptr<RobotStateCore> pastStateI, std::shared_ptr<RobotStateCore>& predictedStateI)
{
    //std::cout<<"FreeModelRobotCore::predictState"<<std::endl;

    // Polymorph
    std::shared_ptr<FreeModelRobotStateCore> pastState=std::static_pointer_cast<FreeModelRobotStateCore>(pastStateI);
    std::shared_ptr<FreeModelRobotStateCore> predictedState=std::static_pointer_cast<FreeModelRobotStateCore>(predictedStateI);



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

    Eigen::Vector4d predictedAttitude=Quaternion::cross(quat_perturbation, pastState->attitude);


    // Unit quaternion -> Very needed!

    predictedState->attitude=predictedAttitude/predictedAttitude.norm();
//std::cout<<"predictedState->attitude="<<predictedState->attitude.transpose()<<std::endl;


    // Quaternion -> Real part always positive
    if(predictedState->attitude[0]<0)
        predictedState->attitude=-predictedState->attitude;



    /// Angular Velocity
    predictedState->angular_velocity=pastState->angular_velocity+pastState->angular_acceleration*dt;

//std::cout<<"predictedState->angular_velocity="<<predictedState->angular_velocity<<std::endl;


    /// Angular Acceleration
    predictedState->angular_acceleration=pastState->angular_acceleration;
//std::cout<<"predictedState->angular_acceleration="<<predictedState->angular_acceleration.transpose()<<std::endl;


    // Finish
    predictedStateI=predictedState;

    return 0;
}

// Jacobian
int FreeModelRobotCore::predictStateErrorStateJacobians(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, std::shared_ptr<RobotStateCore> pastStateI, std::shared_ptr<RobotStateCore>& predictedStateI)
{
    //std::cout<<"FreeModelRobotCore::predictStateErrorStateJacobians"<<std::endl;


    // Polymorph
    std::shared_ptr<FreeModelRobotStateCore> pastState=std::static_pointer_cast<FreeModelRobotStateCore>(pastStateI);
    std::shared_ptr<FreeModelRobotStateCore> predictedState=std::static_pointer_cast<FreeModelRobotStateCore>(predictedStateI);




    // Create the predicted state if it doesn't exist
    if(!predictedState)
    {
        std::cout<<"FreeModelRobotCore::predictStateErrorStateJacobians error en predictedState"<<std::endl;
        return 1;
    }


    //Delta Time
    TimeStamp DeltaTime=currentTimeStamp-previousTimeStamp;
    // delta time
    double dt=DeltaTime.get_double();


    ///// Jacobian Error State

    // Jacobian Size

    predictedState->errorStateJacobian.resize(18,18);
    predictedState->errorStateJacobian.reserve(18+36);

//    predictedState->errorStateJacobian.linear.resize(9, 9);
//    predictedState->errorStateJacobian.linear.reserve(18);
//    //predictedState->errorStateJacobian.linear.setZero();


//    predictedState->errorStateJacobian.angular.resize(9, 9);
//    predictedState->errorStateJacobian.linear.reserve(36);
//    //predictedState->errorStateJacobian.angular.setZero();



    /// Jacobian of the error: Linear Part
    std::vector<Eigen::Triplet<double> > tripletListErrorJacobian;

    // posi / posi  
    //predictedState->errorStateJacobian.linear.block<3,3>(0,0)=Eigen::MatrixXd::Identity(3,3);
    for(int i=0; i<3; i++)
        tripletListErrorJacobian.push_back(Eigen::Triplet<double>(i,i,1));


    // posi / vel
    //predictedState->errorStateJacobian.linear.block<3,3>(0,3)=Eigen::MatrixXd::Identity(3,3)*dt;
    for(int i=0; i<3; i++)
        tripletListErrorJacobian.push_back(Eigen::Triplet<double>(i,3+i,dt));


    // posi / acc
    //predictedState->errorStateJacobian.linear.block<3,3>(0,6)=0.5*Eigen::MatrixXd::Identity(3,3)*pow(dt,2);
    double dt2=pow(dt,2);
    for(int i=0; i<3; i++)
        tripletListErrorJacobian.push_back(Eigen::Triplet<double>(i,i+6,dt2));



    // vel / posi
    // zero

    // vel / vel
    //predictedState->errorStateJacobian.linear.block<3,3>(3,3)=Eigen::MatrixXd::Identity(3,3);
    for(int i=0; i<3; i++)
        tripletListErrorJacobian.push_back(Eigen::Triplet<double>(3+i,3+i,1));


    // vel / acc
    //predictedState->errorStateJacobian.linear.block<3,3>(3,6)=Eigen::MatrixXd::Identity(3,3)*dt;
    for(int i=0; i<3; i++)
        tripletListErrorJacobian.push_back(Eigen::Triplet<double>(3+i,6+i,dt));



    // acc / posi
    // zero

    // acc / vel
    // zero

    // acc / acc
    //predictedState->errorStateJacobian.linear.block<3,3>(6,6)=Eigen::MatrixXd::Identity(3,3);
    for(int i=0; i<3; i++)
        tripletListErrorJacobian.push_back(Eigen::Triplet<double>(6+i,6+i,1));





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


    // att / att [9 non-zero]
    Eigen::Matrix3d jacobianAttAtt=mat_delta_q_delta_theta.transpose()*quat_mat_plus_quat_ref_k1_inv*(Quaternion::quatMatPlus(quat_w_mean_dt)+pow(dt,2)/24*Quaternion::quatMatPlus(quat_for_second_order_correction))*quat_mat_plus_quat_ref_k*mat_delta_q_delta_theta;
    //predictedState->errorStateJacobian.angular.block<3,3>(0,0)=jacobianAttAtt;
    //predictedState->errorStateJacobian.angular=jacobianAttAtt.sparseView();
    for(int i=0; i<3; i++)
        for(int j=0; j<3; j++)
            tripletListErrorJacobian.push_back(Eigen::Triplet<double>(9+i,9+j,jacobianAttAtt(i,j)));



    // att / ang_vel [9 non-zero]
    // TODO fix
    Eigen::Matrix3d jacobianAttAngVel=2*mat_delta_q_delta_theta.transpose()*quat_mat_plus_quat_ref_k1_inv*quat_mat_minus_quat_ref_k*(mat_jacobian_w_mean_dt_to_quat*dt + pow(dt,2)/24*mat_aux_j2);
    //predictedState->errorStateJacobian.angular.block<3,3>(0,3)=jacobianAttAngVel;
    //predictedState->errorStateJacobian.angular=jacobianAttAngVel.sparseView();
    for(int i=0; i<3; i++)
        for(int j=0; j<3; j++)
            tripletListErrorJacobian.push_back(Eigen::Triplet<double>(9+i,9+3+j,jacobianAttAngVel(i,j)));


    // att / ang_accel [9 non-zero]
    // TODO
    Eigen::Matrix3d jacobianAttAngAcc=2*mat_delta_q_delta_theta.transpose()*quat_mat_plus_quat_ref_k1_inv*quat_mat_minus_quat_ref_k*(mat_jacobian_w_mean_dt_to_quat*0.5*pow(dt,2) + pow(dt,2)/24*mat_aux_j3);
    //predictedState->errorStateJacobian.angular.block<3,3>(0,6)=jacobianAttAngVel;
    //predictedState->errorStateJacobian.angular=jacobianAttAngAcc.sparseView();
    for(int i=0; i<3; i++)
        for(int j=0; j<3; j++)
            tripletListErrorJacobian.push_back(Eigen::Triplet<double>(9+i,9+6+j,jacobianAttAngAcc(i,j)));



    // ang_vel / att
    // zero

    // ang_vel / ang_vel [3 non-zero]
    //predictedState->errorStateJacobian.angular.block<3,3>(3,3)=Eigen::MatrixXd::Identity(3,3);
    for(int i=0; i<3; i++)
        tripletListErrorJacobian.push_back(Eigen::Triplet<double>(9+3+i,9+3+i,1));



    // ang_vel / ang_acc [3 non-zero]
    //predictedState->errorStateJacobian.angular.block<3,3>(3,6)=Eigen::MatrixXd::Identity(3,3)*dt;
    for(int i=0; i<3; i++)
        tripletListErrorJacobian.push_back(Eigen::Triplet<double>(9+3+i,9+6+i,dt));


    // ang_acc / att
    // zero

    // ang_acc / ang_vel
    // zero

    // ang_acc / ang_acc [3 non-zero]
    //predictedState->errorStateJacobian.angular.block<3,3>(6,6)=Eigen::MatrixXd::Identity(3,3);
    for(int i=0; i<3; i++)
        tripletListErrorJacobian.push_back(Eigen::Triplet<double>(9+6+i,9+6+i,1));



    predictedState->errorStateJacobian.setFromTriplets(tripletListErrorJacobian.begin(), tripletListErrorJacobian.end());




    ///// Jacobian Error State Noise

    // Jacobian Size


    predictedState->errorStateNoiseJacobian.resize(dimensionErrorState, dimensionNoise);
    predictedState->errorStateNoiseJacobian.reserve(dimensionNoise);


    std::vector<Eigen::Triplet<double> > tripletListNoiseJacobian;

    int dimension_state_i=0;
    int dimension_noise_i=0;


    // Fill

    // Position
    for(int i=0; i<3; i++)
        tripletListNoiseJacobian.push_back(Eigen::Triplet<double>(dimension_state_i+i,dimension_noise_i+i,1));
    dimension_state_i+=3;
    dimension_noise_i+=3;

    // Linear Velocity
    for(int i=0; i<3; i++)
        tripletListNoiseJacobian.push_back(Eigen::Triplet<double>(dimension_state_i+i,dimension_noise_i+i,1));
    dimension_state_i+=3;
    dimension_noise_i+=3;

    // Linear Acceleration
    //jacobian_error_state_noise.block<3,3>(6,0)=Eigen::MatrixXd::Identity(3,3);
    for(int i=0; i<3; i++)
        tripletListNoiseJacobian.push_back(Eigen::Triplet<double>(dimension_state_i+i,dimension_noise_i+i,1));
    dimension_state_i+=3;
    dimension_noise_i+=3;


    // Attitude
    for(int i=0; i<3; i++)
        tripletListNoiseJacobian.push_back(Eigen::Triplet<double>(dimension_state_i+i,dimension_noise_i+i,1));
    dimension_state_i+=3;
    dimension_noise_i+=3;

    // Angular Velocity
    for(int i=0; i<3; i++)
        tripletListNoiseJacobian.push_back(Eigen::Triplet<double>(dimension_state_i+i,dimension_noise_i+i,1));
    dimension_state_i+=3;
    dimension_noise_i+=3;

    // Angular Acceleration
    //jacobian_error_state_noise.block<3,3>(15,3)=Eigen::MatrixXd::Identity(3,3);
    for(int i=0; i<3; i++)
        tripletListNoiseJacobian.push_back(Eigen::Triplet<double>(dimension_state_i+i,dimension_noise_i+i,1));
    dimension_state_i+=3;
    dimension_noise_i+=3;



    predictedState->errorStateNoiseJacobian.setFromTriplets(tripletListNoiseJacobian.begin(), tripletListNoiseJacobian.end());



#if _DEBUG_ROBOT_CORE
    {
        std::ostringstream logString;
        logString<<"FreeModelRobotCore::predictStateErrorStateJacobians() for TS: sec="<<currentTimeStamp.sec<<" s; nsec="<<currentTimeStamp.nsec<<" ns"<<std::endl;
        logString<<"Jacobian Error State"<<std::endl;
        logString<<Eigen::MatrixXd(predictedState->errorStateJacobian)<<std::endl;
        logString<<"Jacobian Error State Noise"<<std::endl;
        logString<<Eigen::MatrixXd(predictedState->errorStateNoiseJacobian)<<std::endl;
        this->log(logString.str());
    }
#endif


    // Finish
    predictedStateI=predictedState;



    return 0;
}

