
#include "msf_localization_core/free_model_robot_core.h"


FreeModelRobotCore::FreeModelRobotCore():
    RobotCore()
{
    init();

    return;
}

FreeModelRobotCore::FreeModelRobotCore(std::weak_ptr<MsfStorageCore> msf_storage_core_ptr):
    RobotCore(msf_storage_core_ptr)
{
    init();

    return;
}

FreeModelRobotCore::~FreeModelRobotCore()
{
    return;
}

int FreeModelRobotCore::init()
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


    // Type
    this->setRobotCoreType(RobotCoreTypes::free_model);


    return 0;
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

int FreeModelRobotCore::prepareCovarianceInitErrorState()
{
    int error=MsfElementCore::prepareCovarianceInitErrorState();

    if(error)
        return error;

    // Do nothing else. The rest is done when setting!

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

int FreeModelRobotCore::predictState(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, const std::shared_ptr<RobotStateCore> pastStateI, std::shared_ptr<RobotStateCore>& predictedStateI)
{
    //std::cout<<"FreeModelRobotCore::predictState"<<std::endl;

    // Polymorph
    std::shared_ptr<FreeModelRobotStateCore> pastState=std::static_pointer_cast<FreeModelRobotStateCore>(pastStateI);
    std::shared_ptr<FreeModelRobotStateCore> predictedState=std::static_pointer_cast<FreeModelRobotStateCore>(predictedStateI);



    // Checks in the past state
    if(!pastState->isCorrect())
    {
        std::cout<<"FreeModelRobotCore::predictState() error !pastState->getTheRobotCore()"<<std::endl;
        return -5;
    }


    // Create the predicted state if it doesn't exist
    if(!predictedState)
    {
        predictedState=std::make_shared<FreeModelRobotStateCore>(pastState->getMsfElementCoreWeakPtr());
    }

//    // Set The robot core if it doesn't exist
//    if(!predictedState->getTheRobotCore())
//    {
//        predictedState->setTheRobotCore(pastState->getTheRobotCore());
//    }



    // Equations


    //Delta Time
    TimeStamp DeltaTime=currentTimeStamp-previousTimeStamp;
    double dt=DeltaTime.get_double();


    /// Position
    predictedState->position=pastState->position+pastState->linear_speed*dt+0.5*pastState->linear_acceleration*dt*dt;


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

    //predictedState->attitude=predictedAttitude/predictedAttitude.norm();
//std::cout<<"predictedState->attitude="<<predictedState->attitude.transpose()<<std::endl;


    // Quaternion unit (very needed) + Real part always positive
//    if(predictedAttitude[0]<0)
//    {
//        predictedState->attitude=-predictedAttitude/predictedAttitude.norm();
//        std::cout<<"FreeModelRobotCore::predictState() quaternion!!"<<std::endl;
//    }
//    else
        predictedState->attitude=predictedAttitude/predictedAttitude.norm();



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
        tripletListErrorJacobian.push_back(Eigen::Triplet<double>(i,i+6,0.5*dt2));



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
    Eigen::Matrix3d jacobianAttAtt=
            mat_delta_q_delta_theta.transpose()*quat_mat_plus_quat_ref_k1_inv*(Quaternion::quatMatPlus(quat_w_mean_dt)+pow(dt,2)/24*Quaternion::quatMatPlus(quat_for_second_order_correction))*quat_mat_plus_quat_ref_k*mat_delta_q_delta_theta;
    //predictedState->errorStateJacobian.angular.block<3,3>(0,0)=jacobianAttAtt;
    //predictedState->errorStateJacobian.angular=jacobianAttAtt.sparseView();
    for(int i=0; i<3; i++)
        for(int j=0; j<3; j++)
            tripletListErrorJacobian.push_back(Eigen::Triplet<double>(9+i,9+j,jacobianAttAtt(i,j)));



    // att / ang_vel [9 non-zero]
    Eigen::Matrix3d jacobianAttAngVel=
            2*mat_delta_q_delta_theta.transpose()*quat_mat_plus_quat_ref_k1_inv*quat_mat_minus_quat_ref_k*(mat_jacobian_w_mean_dt_to_quat*dt + pow(dt,2)/24*mat_aux_j2);
    //predictedState->errorStateJacobian.angular.block<3,3>(0,3)=jacobianAttAngVel;
    //predictedState->errorStateJacobian.angular=jacobianAttAngVel.sparseView();
    for(int i=0; i<3; i++)
        for(int j=0; j<3; j++)
            tripletListErrorJacobian.push_back(Eigen::Triplet<double>(9+i,9+3+j,jacobianAttAngVel(i,j)));


    // att / ang_accel [9 non-zero]
    Eigen::Matrix3d jacobianAttAngAcc=
            2*mat_delta_q_delta_theta.transpose()*quat_mat_plus_quat_ref_k1_inv*quat_mat_minus_quat_ref_k*(mat_jacobian_w_mean_dt_to_quat*0.5*pow(dt,2) + pow(dt,2)/24*mat_aux_j3);
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


int FreeModelRobotCore::readConfig(pugi::xml_node robot, std::shared_ptr<FreeModelRobotStateCore>& RobotInitStateCore)
{
    // Map Element Core Pointer
    //std::shared_ptr<FreeModelRobotCore> TheRobotCoreCore(this);
    //std::shared_ptr<FreeModelRobotCore> TheRobotCoreCore=std::dynamic_pointer_cast<FreeModelRobotCore>(this->getMsfElementCoreSharedPtr());

    // Set pointer to the RobotCore
    //this->setTheRobotCore(TheRobotCoreCore);

    // Set robot type
    //this->setRobotType(RobotTypes::free_model);

    // Set the access to the Storage core
    //this->setTheMsfStorageCore(TheMsfStorageCore);

    // Create a class for the RobotStateCore
    if(!RobotInitStateCore)
        RobotInitStateCore=std::make_shared<FreeModelRobotStateCore>(this->getMsfElementCoreWeakPtr());
    // Set pointer to the SensorCore
    //RobotInitStateCore->setTheRobotCore(TheRobotCoreCore);


    // Aux vars
    std::string readingValue;


    // Name
    readingValue=robot.child_value("name");
    this->setRobotName(readingValue);


    /// Init State

    // Position
    readingValue=robot.child("position").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d position;
        stm>>position[0]>>position[1]>>position[2];
        RobotInitStateCore->setPosition(position);
    }

    // Linear Speed
    readingValue=robot.child("lin_speed").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d lin_speed;
        stm>>lin_speed[0]>>lin_speed[1]>>lin_speed[2];
        RobotInitStateCore->setLinearSpeed(lin_speed);
    }

    // Linear Acceleration
    readingValue=robot.child("lin_accel").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d lin_accel;
        stm>>lin_accel[0]>>lin_accel[1]>>lin_accel[2];
        RobotInitStateCore->setLinearAcceleration(lin_accel);
    }

    // Attitude
    readingValue=robot.child("attitude").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector4d attitude;
        stm>>attitude[0]>>attitude[1]>>attitude[2]>>attitude[3];
        RobotInitStateCore->setAttitude(attitude);
    }

    // Angular Velocity
    readingValue=robot.child("ang_velocity").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d ang_velocity;
        stm>>ang_velocity[0]>>ang_velocity[1]>>ang_velocity[2];
        RobotInitStateCore->setAngularVelocity(ang_velocity);
    }

    // Angular Acceleration
    readingValue=robot.child("ang_accel").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d auxVec;
        stm>>auxVec[0]>>auxVec[1]>>auxVec[2];
        RobotInitStateCore->setAngularAcceleration(auxVec);
    }


    /// Init Variances

    // Prepare Covariance of init error state -> After preparing, the parameters are read!
    this->prepareCovarianceInitErrorState();

    // Position
    readingValue=robot.child("position").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setInitErrorStateVariancePosition(variance);
    }

    // Linear Speed
    readingValue=robot.child("lin_speed").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setInitErrorStateVarianceLinearSpeed(variance);
    }

    // Linear acceleration
    readingValue=robot.child("lin_accel").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setInitErrorStateVarianceLinearAcceleration(variance);
    }

    // Attitude
    readingValue=robot.child("attitude").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setInitErrorStateVarianceAttitude(variance);
    }

    // Angular velocity
    readingValue=robot.child("ang_velocity").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setInitErrorStateVarianceAngularVelocity(variance);
    }

    // Angular acceleration
    readingValue=robot.child("ang_accel").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setInitErrorStateVarianceAngularAcceleration(variance);
    }



    // Noises Estimation

    // Linear acceleration
    readingValue=robot.child("position").child_value("noise");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoisePosition(variance.asDiagonal());
    }

    // Linear velocity
    readingValue=robot.child("lin_speed").child_value("noise");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoiseLinearSpeed(variance.asDiagonal());
    }

    // Linear acceleration
    readingValue=robot.child("lin_accel").child_value("noise");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoiseLinearAcceleration(variance.asDiagonal());
    }

    // Attitude
    readingValue=robot.child("attitude").child_value("noise");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoiseAttitude(variance.asDiagonal());
    }

    // Angular velocity
    readingValue=robot.child("ang_velocity").child_value("noise");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoiseAngularVelocity(variance.asDiagonal());
    }

    // Angular acceleration
    readingValue=robot.child("ang_accel").child_value("noise");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoiseAngularAcceleration(variance.asDiagonal());
    }


    // End
    return 0;
}

