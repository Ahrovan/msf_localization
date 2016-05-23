
#include "msf_localization_core/free_model_robot_core.h"


FreeModelRobotCore::FreeModelRobotCore() :
    RobotCore()
{
    init();

    return;
}

FreeModelRobotCore::FreeModelRobotCore(const std::weak_ptr<MsfStorageCore> msf_storage_core_ptr) :
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
    dimension_state_=9+10;
    dimension_error_state_=9+9;

    dimension_parameters_=0;
    dimension_error_parameters_=0;

    dimension_noise_=9+9;

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

int FreeModelRobotCore::readConfig(const pugi::xml_node &robot, std::shared_ptr<FreeModelRobotStateCore>& RobotInitStateCore)
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


//    // Name
//    readingValue=robot.child_value("name");
//    this->setRobotName(readingValue);


    /// Init State

    // Position
    readingValue=robot.child("position").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d position;
        stm>>position[0]>>position[1]>>position[2];
        RobotInitStateCore->setPositionRobotWrtWorld(position);
    }

    // Linear Speed
    readingValue=robot.child("lin_speed").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d lin_speed;
        stm>>lin_speed[0]>>lin_speed[1]>>lin_speed[2];
        RobotInitStateCore->setLinearSpeedRobotWrtWorld(lin_speed);
    }

    // Linear Acceleration
    readingValue=robot.child("lin_accel").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d lin_accel;
        stm>>lin_accel[0]>>lin_accel[1]>>lin_accel[2];
        RobotInitStateCore->setLinearAccelerationRobotWrtWorld(lin_accel);
    }

    // Attitude
    readingValue=robot.child("attitude").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector4d attitude;
        stm>>attitude[0]>>attitude[1]>>attitude[2]>>attitude[3];
        RobotInitStateCore->setAttitudeRobotWrtWorld(attitude);
    }

    // Angular Velocity
    readingValue=robot.child("ang_velocity").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d ang_velocity;
        stm>>ang_velocity[0]>>ang_velocity[1]>>ang_velocity[2];
        RobotInitStateCore->setAngularVelocityRobotWrtWorld(ang_velocity);
    }

    // Angular Acceleration
    readingValue=robot.child("ang_accel").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d auxVec;
        stm>>auxVec[0]>>auxVec[1]>>auxVec[2];
        RobotInitStateCore->setAngularAccelerationRobotWrtWorld(auxVec);
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


Eigen::Matrix3d FreeModelRobotCore::getNoisePosition() const
{
    return this->noisePosition;
}

int FreeModelRobotCore::setNoisePosition(const Eigen::Matrix3d &noisePosition)
{
    this->noisePosition=noisePosition;
    return 0;
}

Eigen::Matrix3d FreeModelRobotCore::getNoiseLinearSpeed() const
{
    return this->noiseLinearSpeed;
}

int FreeModelRobotCore::setNoiseLinearSpeed(const Eigen::Matrix3d &noiseLinearSpeed)
{
    this->noiseLinearSpeed=noiseLinearSpeed;
    return 0;
}

Eigen::Matrix3d FreeModelRobotCore::getNoiseLinearAcceleration() const
{
    return this->noiseLinearAcceleration;
}

int FreeModelRobotCore::setNoiseLinearAcceleration(const Eigen::Matrix3d& noiseLinearAcceleration)
{
    this->noiseLinearAcceleration=noiseLinearAcceleration;
    return 0;
}

Eigen::Matrix3d FreeModelRobotCore::getNoiseAttitude() const
{
    return this->noiseAttitude;
}

int FreeModelRobotCore::setNoiseAttitude(const Eigen::Matrix3d &noiseAttitude)
{
    this->noiseAttitude=noiseAttitude;
    return 0;
}

Eigen::Matrix3d FreeModelRobotCore::getNoiseAngularVelocity() const
{
    return this->noiseAngularVelocity;
}

int FreeModelRobotCore::setNoiseAngularVelocity(const Eigen::Matrix3d& noiseAngularVelocity)
{
    this->noiseAngularVelocity=noiseAngularVelocity;
    return 0;
}


Eigen::Matrix3d FreeModelRobotCore::getNoiseAngularAcceleration() const
{
    return this->noiseAngularAcceleration;
}

int FreeModelRobotCore::setNoiseAngularAcceleration(const Eigen::Matrix3d& noiseAngularAcceleration)
{
    this->noiseAngularAcceleration=noiseAngularAcceleration;
    return 0;
}

Eigen::SparseMatrix<double> FreeModelRobotCore::getCovarianceParameters()
{
    // No parameters

    return Eigen::SparseMatrix<double>();
}

Eigen::SparseMatrix<double> FreeModelRobotCore::getCovarianceNoise(const TimeStamp deltaTimeStamp)
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

int FreeModelRobotCore::prepareCovarianceInitErrorStateSpecific()
{

    // Do nothing else. The rest is done when setting!

    return 0;
}

int FreeModelRobotCore::setInitErrorStateVariancePosition(const Eigen::Vector3d &initVariance)
{
    this->covariance_init_error_state_.block<3,3>(0,0)=initVariance.asDiagonal();
    return 0;
}

int FreeModelRobotCore::setInitErrorStateVarianceLinearSpeed(const Eigen::Vector3d& initVariance)
{
    this->covariance_init_error_state_.block<3,3>(3,3)=initVariance.asDiagonal();
    return 0;
}

int FreeModelRobotCore::setInitErrorStateVarianceLinearAcceleration(const Eigen::Vector3d &initVariance)
{
    this->covariance_init_error_state_.block<3,3>(6,6)=initVariance.asDiagonal();
    return 0;
}

int FreeModelRobotCore::setInitErrorStateVarianceAttitude(const Eigen::Vector3d& initVariance)
{
    this->covariance_init_error_state_.block<3,3>(9,9)=initVariance.asDiagonal();
    return 0;
}

int FreeModelRobotCore::setInitErrorStateVarianceAngularVelocity(const Eigen::Vector3d &initVariance)
{
    this->covariance_init_error_state_.block<3,3>(12,12)=initVariance.asDiagonal();
    return 0;
}

int FreeModelRobotCore::setInitErrorStateVarianceAngularAcceleration(const Eigen::Vector3d& initVariance)
{
    this->covariance_init_error_state_.block<3,3>(15,15)=initVariance.asDiagonal();
    return 0;
}

int FreeModelRobotCore::predictState(//Time
                                     const TimeStamp &previousTimeStamp,
                                     const TimeStamp &currentTimeStamp,
                                     // Previous State
                                     const std::shared_ptr<StateEstimationCore> &pastState,
                                     // Inputs
                                     const std::shared_ptr<InputCommandComponent> &inputCommand,
                                     // Predicted State
                                     std::shared_ptr<StateCore> &predictedState)
{
    // Checks

    // Past State
    if(!pastState)
        return -1;



    // Robot Predicted State
    FreeModelRobotStateCore* predictedRobotState(nullptr);
    if(!predictedState)
    {
        predictedRobotState=new FreeModelRobotStateCore;
        predictedRobotState->setMsfElementCorePtr(pastState->TheRobotStateCore->getMsfElementCoreWeakPtr());
        predictedState=std::shared_ptr<StateCore>(predictedRobotState);
    }
    else
        predictedRobotState=dynamic_cast<FreeModelRobotStateCore*>(predictedState.get());


    // Predict State
    int error_predict_state=predictStateSpecific(previousTimeStamp, currentTimeStamp,
                                         dynamic_cast<FreeModelRobotStateCore*>(pastState->TheRobotStateCore.get()),
                                         predictedRobotState);

    // Check error
    if(error_predict_state)
        return error_predict_state;


    // End
    return 0;
}

int FreeModelRobotCore::predictStateSpecific(const TimeStamp &previousTimeStamp, const TimeStamp &currentTimeStamp,
                                             const FreeModelRobotStateCore *pastState,
                                             FreeModelRobotStateCore *&predictedState)
{

    // Checks in the past state
    if(!pastState->isCorrect())
    {
        std::cout<<"FreeModelRobotCore::predictState() error"<<std::endl;
        return -5;
    }


    // Create the predicted state if it doesn't exist
    if(!predictedState)
    {
        predictedState= new FreeModelRobotStateCore;
        predictedState->setMsfElementCorePtr(pastState->getMsfElementCoreWeakPtr());
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

    Eigen::Vector3d w_mean_dt=(pastState->angular_velocity_robot_wrt_world_+0.5*pastState->angular_acceleration_robot_wrt_world_*dt)*dt;
    Eigen::Vector4d rotation_w_mean_dt_to_quat;
    rotation_w_mean_dt_to_quat=Quaternion::rotationVectorToQuaternion(w_mean_dt);
    rotation_w_mean_dt_to_quat=rotation_w_mean_dt_to_quat/rotation_w_mean_dt_to_quat.norm();

    //std::cout<<"w_mean_dt="<<w_mean_dt<<std::endl;
    //std::cout<<"rotation_w_mean_dt_to_quat="<<rotation_w_mean_dt_to_quat<<std::endl;


    // deltaQalpha
    Eigen::Vector3d alphaContribution;
    alphaContribution=pastState->angular_velocity_robot_wrt_world_;
    alphaContribution=alphaContribution.cross(pastState->angular_velocity_robot_wrt_world_+pastState->angular_acceleration_robot_wrt_world_*dt);

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

    Eigen::Vector4d predictedAttitude=Quaternion::cross(quat_perturbation, pastState->attitude_robot_wrt_world_);


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
        predictedState->attitude_robot_wrt_world_=predictedAttitude/predictedAttitude.norm();



    /// Angular Velocity
    predictedState->angular_velocity_robot_wrt_world_=pastState->angular_velocity_robot_wrt_world_+pastState->angular_acceleration_robot_wrt_world_*dt;

//std::cout<<"predictedState->angular_velocity="<<predictedState->angular_velocity<<std::endl;


    /// Angular Acceleration
    predictedState->angular_acceleration_robot_wrt_world_=pastState->angular_acceleration_robot_wrt_world_;
//std::cout<<"predictedState->angular_acceleration="<<predictedState->angular_acceleration.transpose()<<std::endl;


    // Finish
    //predictedStateI=predictedState;

    return 0;
}

// Jacobian
int FreeModelRobotCore::predictErrorStateJacobian(//Time
                                                 const TimeStamp& previousTimeStamp, const TimeStamp& currentTimeStamp,
                                                 // Previous State
                                                 const std::shared_ptr<StateEstimationCore>& past_state,
                                                  // Inputs
                                                  const std::shared_ptr<InputCommandComponent>& input_command,
                                                 // Predicted State
                                                 std::shared_ptr<StateCore>& predicted_state)
{
    // Checks

    // Past State
    if(!past_state)
        return -1;

    // TODO

    // Predicted State
    if(!predicted_state)
        return -1;


    //// Init Jacobians
    int error_init_jacobians=predictErrorStateJacobianInit(// Past State
                                                           past_state,
                                                           // Input commands
                                                           input_command,
                                                           // Predicted State
                                                           predicted_state);

    if(error_init_jacobians)
        return error_init_jacobians;


    /// Robot Predicted State
    FreeModelRobotStateCore* predicted_robot_state=dynamic_cast<FreeModelRobotStateCore*>(predicted_state.get());


    /// Get iterators to fill jacobians

    // Fx & Fp
    // Robot
    // Nothing to do


    // Fu
    // Nothing


    // Fn
    // Nothing to do


    /// Predict State Jacobians
    int error_predict_state_jacobians=predictErrorStateJacobianSpecific(previousTimeStamp, currentTimeStamp,
                                                                        dynamic_cast<FreeModelRobotStateCore*>(past_state->TheRobotStateCore.get()),
                                                                        predicted_robot_state,
                                                                        // Jacobians Error State: Fx, Fp
                                                                        predicted_robot_state->jacobian_error_state_.robot,
                                                                        predicted_robot_state->jacobian_error_parameters_.robot,
                                                                        // Jacobian Error Noise: Fn
                                                                        predicted_robot_state->jacobian_error_state_noise_
                                                                        );

    // Check error
    if(error_predict_state_jacobians)
        return error_predict_state_jacobians;


    // End
    return 0;
}

int FreeModelRobotCore::predictErrorStateJacobianSpecific(const TimeStamp& previousTimeStamp, const TimeStamp& currentTimeStamp,
                                                          const FreeModelRobotStateCore *pastState,
                                                          const FreeModelRobotStateCore *predictedState,
                                                          // Jacobians Error State: Fx, Fp
                                                          // Robot
                                                          Eigen::SparseMatrix<double>& jacobian_error_state_wrt_robot_error_state,
                                                          Eigen::SparseMatrix<double>& jacobian_error_state_wrt_robot_error_parameters,
                                                          // Jacobians Noise: Fn
                                                          Eigen::SparseMatrix<double>& jacobian_error_state_wrt_noise
                                                          )
{

    // Create the predicted state if it doesn't exist
    if(!predictedState)
    {
        std::cout<<"FreeModelRobotCore::predictErrorStateJacobians error en predictedState"<<std::endl;
        return 1;
    }


    //Delta Time
    TimeStamp DeltaTime=currentTimeStamp-previousTimeStamp;
    // delta time
    double dt=DeltaTime.get_double();




    ///// Jacobian Error State - Error State: Fx & Jacobian Error State - Error Parameters: Fp

    /// World
    {
        // Nothing to do
    }

    /// Robot
    {
        // Resize and init
        jacobian_error_state_wrt_robot_error_state.resize(dimension_error_state_, dimension_error_state_);
        jacobian_error_state_wrt_robot_error_parameters.resize(dimension_error_state_, dimension_error_parameters_);

        std::vector<Eigen::Triplet<double>> triplet_list_jacobian_error_state_wrt_error_state;
        std::vector<Eigen::Triplet<double>> triplet_list_jacobian_error_state_wrt_error_parameters;


        // Fill

        int dimension_error_state_i=0;
        int dimension_error_parameters_i=0;


        // posi
        {
            int dimension_error_state_j=0;

            // posi / posi
            {
                // Eigen::MatrixXd::Identity(3,3);
                for(int i=0; i<3; i++)
                    triplet_list_jacobian_error_state_wrt_error_state.push_back(Eigen::Triplet<double>(dimension_error_state_i+i,dimension_error_state_j+i,1));
                dimension_error_state_j+=3;
            }


            // posi / vel
            {
                // Eigen::MatrixXd::Identity(3,3)*dt;
                for(int i=0; i<3; i++)
                    triplet_list_jacobian_error_state_wrt_error_state.push_back(Eigen::Triplet<double>(dimension_error_state_i+i,dimension_error_state_j+i,dt));
                dimension_error_state_j+=3;
            }


            // posi / acc
            {
                // 0.5*Eigen::MatrixXd::Identity(3,3)*pow(dt,2);
                double dt2=pow(dt,2);
                for(int i=0; i<3; i++)
                    triplet_list_jacobian_error_state_wrt_error_state.push_back(Eigen::Triplet<double>(dimension_error_state_i+i,dimension_error_state_j+i,0.5*dt2));
                dimension_error_state_j+=3;
            }


            dimension_error_state_i+=3;
        }


        // lin vel
        {
            int dimension_error_state_j=0;

            // vel / posi
            {
                // zero
                dimension_error_state_j+=3;
            }

            // vel / vel
            {
                // Eigen::MatrixXd::Identity(3,3);
                for(int i=0; i<3; i++)
                    triplet_list_jacobian_error_state_wrt_error_state.push_back(Eigen::Triplet<double>(dimension_error_state_i+i,dimension_error_state_j+i,1));
                dimension_error_state_j+=3;
            }


            // vel / acc
            {
                // Eigen::MatrixXd::Identity(3,3)*dt;
                for(int i=0; i<3; i++)
                    triplet_list_jacobian_error_state_wrt_error_state.push_back(Eigen::Triplet<double>(dimension_error_state_i+i,dimension_error_state_j+i,dt));
                dimension_error_state_j+=3;
            }


            dimension_error_state_i+=3;
        }


        // lin accel
        {
            int dimension_error_state_j=0;

            // acc / posi
            {
                // zero
                dimension_error_state_j+=3;
            }

            // acc / vel
            {
                // zero
                dimension_error_state_j+=3;
            }

            // acc / acc
            {
                // Eigen::MatrixXd::Identity(3,3);
                for(int i=0; i<3; i++)
                    triplet_list_jacobian_error_state_wrt_error_state.push_back(Eigen::Triplet<double>(dimension_error_state_i+i,dimension_error_state_j+i,1));
                dimension_error_state_j+=3;
            }

            dimension_error_state_i+=3;
        }



        // Jacobian of the error -> Angular Part


        // Auxiliar values
        Eigen::Vector3d w_mean_dt=(pastState->angular_velocity_robot_wrt_world_+0.5*pastState->angular_acceleration_robot_wrt_world_*dt)*dt;
        Eigen::Vector4d quat_w_mean_dt=Quaternion::rotationVectorToQuaternion(w_mean_dt);
        Eigen::Vector4d quat_for_second_order_correction;
        quat_for_second_order_correction.setZero();
        quat_for_second_order_correction.block<3,1>(1,0)=pastState->angular_velocity_robot_wrt_world_.cross(predictedState->angular_velocity_robot_wrt_world_);

        // Auxiliar Matrixes
        Eigen::Matrix4d quat_mat_plus_quat_ref_k1=Quaternion::quatMatPlus(predictedState->attitude_robot_wrt_world_);
        Eigen::Matrix4d quat_mat_plus_quat_ref_k1_inv=quat_mat_plus_quat_ref_k1.inverse();
        Eigen::MatrixXd mat_delta_q_delta_theta(4, 3);
        mat_delta_q_delta_theta.setZero();
        mat_delta_q_delta_theta(1,0)=1;
        mat_delta_q_delta_theta(2,1)=1;
        mat_delta_q_delta_theta(3,2)=1;
        Eigen::Matrix4d quat_mat_plus_quat_ref_k=Quaternion::quatMatPlus(pastState->attitude_robot_wrt_world_);
        Eigen::Matrix4d quat_mat_minus_quat_ref_k=Quaternion::quatMatMinus(pastState->attitude_robot_wrt_world_);
        Eigen::MatrixXd mat_jacobian_w_mean_dt_to_quat=Quaternion::jacobianRotationVectorToQuaternion(w_mean_dt);
        Eigen::Matrix3d skew_mat_alpha_dt=Quaternion::skewSymMat(pastState->angular_acceleration_robot_wrt_world_*dt);
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


        // attitude
        {
            int dimension_error_state_j=0;

            // att / linear
            {
                // zero
                dimension_error_state_j+=3+3+3;
            }

            // att / att [9 non-zero]
            {
                Eigen::Matrix3d jacobianAttAtt=
                        mat_delta_q_delta_theta.transpose()*quat_mat_plus_quat_ref_k1_inv*(Quaternion::quatMatPlus(quat_w_mean_dt)+pow(dt,2)/24*Quaternion::quatMatPlus(quat_for_second_order_correction))*quat_mat_plus_quat_ref_k*mat_delta_q_delta_theta;
                BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_state_wrt_error_state, jacobianAttAtt, dimension_error_state_i, dimension_error_state_j);
                dimension_error_state_j+=3;
            }



            // att / ang_vel [9 non-zero]
            {
                Eigen::Matrix3d jacobianAttAngVel=
                        2*mat_delta_q_delta_theta.transpose()*quat_mat_plus_quat_ref_k1_inv*quat_mat_minus_quat_ref_k*(mat_jacobian_w_mean_dt_to_quat*dt + pow(dt,2)/24*mat_aux_j2);
                BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_state_wrt_error_state, jacobianAttAngVel, dimension_error_state_i, dimension_error_state_j);
                dimension_error_state_j+=3;
            }


            // att / ang_accel [9 non-zero]
            {
                Eigen::Matrix3d jacobianAttAngAcc=
                        2*mat_delta_q_delta_theta.transpose()*quat_mat_plus_quat_ref_k1_inv*quat_mat_minus_quat_ref_k*(mat_jacobian_w_mean_dt_to_quat*0.5*pow(dt,2) + pow(dt,2)/24*mat_aux_j3);
                BlockMatrix::insertVectorEigenTripletFromEigenDense(triplet_list_jacobian_error_state_wrt_error_state, jacobianAttAngAcc, dimension_error_state_i, dimension_error_state_j);
                dimension_error_state_j+=3;
            }

            dimension_error_state_i+=3;
        }


        // angular velocity
        {
            int dimension_error_state_j=0;

            // ang_vel / linear
            {
                // zero
                dimension_error_state_j+=3+3+3;
            }

            // ang_vel / att
            {
                // zero
                dimension_error_state_j+=3;
            }

            // ang_vel / ang_vel [3 non-zero]
            {
                // Eigen::MatrixXd::Identity(3,3);
                for(int i=0; i<3; i++)
                    triplet_list_jacobian_error_state_wrt_error_state.push_back(Eigen::Triplet<double>(dimension_error_state_i+i,dimension_error_state_j+i,1));
                dimension_error_state_j+=3;
            }



            // ang_vel / ang_acc [3 non-zero]
            {
                // Eigen::MatrixXd::Identity(3,3)*dt;
                for(int i=0; i<3; i++)
                    triplet_list_jacobian_error_state_wrt_error_state.push_back(Eigen::Triplet<double>(dimension_error_state_i+i,dimension_error_state_j+i,dt));
                dimension_error_state_j+=3;
            }

            dimension_error_state_i+=3;
        }


        // angular acceleration
        {
            int dimension_error_state_j=0;

            // ang acc / linear
            {
                // zero
                dimension_error_state_j+=3+3+3;
            }

            // ang_acc / att
            {
                // zero
                dimension_error_state_j+=3;
            }

            // ang_acc / ang_vel
            {
                // zero
                dimension_error_state_j+=3;
            }

            // ang_acc / ang_acc [3 non-zero]
            {
                // Eigen::MatrixXd::Identity(3,3);
                for(int i=0; i<3; i++)
                    triplet_list_jacobian_error_state_wrt_error_state.push_back(Eigen::Triplet<double>(dimension_error_state_i+i,dimension_error_state_j+i,1));
                dimension_error_state_j+=3;
            }

            dimension_error_state_i+=3;
        }


        // Set From Triplets
        jacobian_error_state_wrt_robot_error_state.setFromTriplets(triplet_list_jacobian_error_state_wrt_error_state.begin(), triplet_list_jacobian_error_state_wrt_error_state.end());
        jacobian_error_state_wrt_robot_error_parameters.setFromTriplets(triplet_list_jacobian_error_state_wrt_error_parameters.begin(), triplet_list_jacobian_error_state_wrt_error_parameters.end());

    }

    /// Inputs
    {
        // Nothing to do
    }

    /// Sensors
    {
        // Nothing to do
    }

    /// Map Elements
    {
        // Nothing to do
    }



    //// Jacobian Error State - Error Input

    {
        // Nothing to do
    }


    //// Jacobian Error State - Noise Estimation: Fn

    {
        // Resize and init
        jacobian_error_state_wrt_noise.resize(dimension_error_state_, dimension_noise_);


        std::vector<Eigen::Triplet<double> > tripletListNoiseJacobian;

        int dimension_state_i=0;
        int dimension_noise_i=0;


        // Fill

        // Position
        {
            for(int i=0; i<3; i++)
                tripletListNoiseJacobian.push_back(Eigen::Triplet<double>(dimension_state_i+i,dimension_noise_i+i,1));
            dimension_state_i+=3;
            dimension_noise_i+=3;
        }

        // Linear Velocity
        {
            for(int i=0; i<3; i++)
                tripletListNoiseJacobian.push_back(Eigen::Triplet<double>(dimension_state_i+i,dimension_noise_i+i,1));
            dimension_state_i+=3;
            dimension_noise_i+=3;
        }

        // Linear Acceleration
        {
            //jacobian_error_state_noise.block<3,3>(6,0)=Eigen::MatrixXd::Identity(3,3);
            for(int i=0; i<3; i++)
                tripletListNoiseJacobian.push_back(Eigen::Triplet<double>(dimension_state_i+i,dimension_noise_i+i,1));
            dimension_state_i+=3;
            dimension_noise_i+=3;
        }

        // Attitude
        {
            for(int i=0; i<3; i++)
                tripletListNoiseJacobian.push_back(Eigen::Triplet<double>(dimension_state_i+i,dimension_noise_i+i,1));
            dimension_state_i+=3;
            dimension_noise_i+=3;
        }

        // Angular Velocity
        {
            for(int i=0; i<3; i++)
                tripletListNoiseJacobian.push_back(Eigen::Triplet<double>(dimension_state_i+i,dimension_noise_i+i,1));
            dimension_state_i+=3;
            dimension_noise_i+=3;
        }

        // Angular Acceleration
        {
            //jacobian_error_state_noise.block<3,3>(15,3)=Eigen::MatrixXd::Identity(3,3);
            for(int i=0; i<3; i++)
                tripletListNoiseJacobian.push_back(Eigen::Triplet<double>(dimension_state_i+i,dimension_noise_i+i,1));
            dimension_state_i+=3;
            dimension_noise_i+=3;
        }


        // Set From Triplets
        jacobian_error_state_wrt_noise.setFromTriplets(tripletListNoiseJacobian.begin(), tripletListNoiseJacobian.end());

    }


    // End
    return 0;
}

int FreeModelRobotCore::resetErrorStateJacobian(// Time
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

    // Angular Acc
    {
        for(int i=0; i<3; i++)
            triplets_jacobian_error_reset.push_back(Eigen::Triplet<double>(dimension_error_state_i+i, dimension_error_state_i+i, 1.0));

        dimension_error_state_i+=3;
    }


    current_state->jacobian_error_state_reset_.setFromTriplets(triplets_jacobian_error_reset.begin(), triplets_jacobian_error_reset.end());

    // End
    return 0;
}


