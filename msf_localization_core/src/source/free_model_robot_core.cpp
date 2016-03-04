
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
    Eigen::Vector4d deltaQw;
    Eigen::Vector3d wContribution=pastState->angular_velocity+0.5*pastState->angular_acceleration*dt;
    if(wContribution.norm() < 1e-3)
    {
        // Fill
        deltaQw[0]=1;
        deltaQw.block<3,1>(1,0)=wContribution*dt;
        // Unit quaternion
        deltaQw=deltaQw/deltaQw.norm();
    }
    else
    {
        // Fill
        deltaQw[0]=cos(wContribution.norm()*dt/2);
        deltaQw.block<3,1>(1,0)=wContribution/wContribution.norm()*sin(wContribution.norm()*dt/2);
        // Unit quaternion -> Not needed, just in case! ?
        deltaQw=deltaQw/deltaQw.norm();
    }

    // deltaQalpha
    Eigen::Vector4d deltaQalpha;
    Eigen::Vector3d alphaContribution;
    alphaContribution=pastState->angular_velocity;
    alphaContribution=alphaContribution.cross(pastState->angular_velocity+pastState->angular_acceleration*dt);

    deltaQalpha[0]=0;
    deltaQalpha.block<3,1>(1,0)=alphaContribution;
    // Unit quaternion -> No needed, NO
    //deltaQalpha=deltaQalpha/deltaQalpha.norm();

    // prediction
    predictedState->attitude=Quaternion::cross(pastState->attitude,deltaQw)+pow(dt,2)/24*Quaternion::cross(pastState->attitude,deltaQalpha);

    // Unit quaternion -> Very needed!
    predictedState->attitude=predictedState->attitude/predictedState->attitude.norm();



    /// Angular Velocity
    predictedState->angular_velocity=pastState->angular_velocity+pastState->angular_acceleration*dt;




    /// Angular Acceleration
    predictedState->angular_acceleration=pastState->angular_acceleration;



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



    // References
    // Attitude k
    double qrj1=pastState->attitude[0];
    double qrj2=pastState->attitude[1];
    double qrj3=pastState->attitude[2];
    double qrj4=pastState->attitude[3];
    // Attitude k+1
    double qrk1=predictedState->attitude[0];
    double qrk2=predictedState->attitude[1];
    double qrk3=predictedState->attitude[2];
    double qrk4=predictedState->attitude[3];
    // Angular velocity k
    double wr1=pastState->angular_velocity[0];
    double wr2=pastState->angular_velocity[1];
    double wr3=pastState->angular_velocity[2];


    // att / att
    // TODO fix
    predictedState->errorStateJacobian.angular.block<3,3>(0,0)<<
        qrj1*qrk1 + qrj2*qrk2 + qrj3*qrk3 + qrj4*qrk4 + dt*wr1*((qrj1*qrk2)/2 - (qrj2*qrk1)/2 - (qrj3*qrk4)/2 + (qrj4*qrk3)/2) - dt*wr2*((qrj1*qrk3)/2 - (qrj3*qrk1)/2 + (qrj2*qrk4)/2 - (qrj4*qrk2)/2) - dt*wr3*((qrj1*qrk4)/2 - (qrj2*qrk3)/2 + (qrj3*qrk2)/2 - (qrj4*qrk1)/2),        qrj1*qrk4 - qrj2*qrk3 + qrj3*qrk2 - qrj4*qrk1 + dt*wr1*((qrj1*qrk3)/2 - (qrj3*qrk1)/2 + (qrj2*qrk4)/2 - (qrj4*qrk2)/2) + dt*wr2*((qrj1*qrk2)/2 - (qrj2*qrk1)/2 - (qrj3*qrk4)/2 + (qrj4*qrk3)/2) + dt*wr3*((qrj1*qrk1)/2 + (qrj2*qrk2)/2 + (qrj3*qrk3)/2 + (qrj4*qrk4)/2),      qrj3*qrk1 - qrj1*qrk3 - qrj2*qrk4 + qrj4*qrk2 + dt*wr1*((qrj1*qrk4)/2 - (qrj2*qrk3)/2 + (qrj3*qrk2)/2 - (qrj4*qrk1)/2) - dt*wr2*((qrj1*qrk1)/2 + (qrj2*qrk2)/2 + (qrj3*qrk3)/2 + (qrj4*qrk4)/2) + dt*wr3*((qrj1*qrk2)/2 - (qrj2*qrk1)/2 - (qrj3*qrk4)/2 + (qrj4*qrk3)/2),
        qrj2*qrk3 - qrj1*qrk4 - qrj3*qrk2 + qrj4*qrk1 + dt*wr1*((qrj1*qrk3)/2 - (qrj3*qrk1)/2 + (qrj2*qrk4)/2 - (qrj4*qrk2)/2) + dt*wr2*((qrj1*qrk2)/2 - (qrj2*qrk1)/2 - (qrj3*qrk4)/2 + (qrj4*qrk3)/2) - dt*wr3*((qrj1*qrk1)/2 + (qrj2*qrk2)/2 + (qrj3*qrk3)/2 + (qrj4*qrk4)/2),        qrj1*qrk1 + qrj2*qrk2 + qrj3*qrk3 + qrj4*qrk4 - dt*wr1*((qrj1*qrk2)/2 - (qrj2*qrk1)/2 - (qrj3*qrk4)/2 + (qrj4*qrk3)/2) + dt*wr2*((qrj1*qrk3)/2 - (qrj3*qrk1)/2 + (qrj2*qrk4)/2 - (qrj4*qrk2)/2) - dt*wr3*((qrj1*qrk4)/2 - (qrj2*qrk3)/2 + (qrj3*qrk2)/2 - (qrj4*qrk1)/2),      qrj1*qrk2 - qrj2*qrk1 - qrj3*qrk4 + qrj4*qrk3 + dt*wr1*((qrj1*qrk1)/2 + (qrj2*qrk2)/2 + (qrj3*qrk3)/2 + (qrj4*qrk4)/2) + dt*wr2*((qrj1*qrk4)/2 - (qrj2*qrk3)/2 + (qrj3*qrk2)/2 - (qrj4*qrk1)/2) + dt*wr3*((qrj1*qrk3)/2 - (qrj3*qrk1)/2 + (qrj2*qrk4)/2 - (qrj4*qrk2)/2),
        qrj1*qrk3 - qrj3*qrk1 + qrj2*qrk4 - qrj4*qrk2 + dt*wr1*((qrj1*qrk4)/2 - (qrj2*qrk3)/2 + (qrj3*qrk2)/2 - (qrj4*qrk1)/2) + dt*wr2*((qrj1*qrk1)/2 + (qrj2*qrk2)/2 + (qrj3*qrk3)/2 + (qrj4*qrk4)/2) + dt*wr3*((qrj1*qrk2)/2 - (qrj2*qrk1)/2 - (qrj3*qrk4)/2 + (qrj4*qrk3)/2),        qrj2*qrk1 - qrj1*qrk2 + qrj3*qrk4 - qrj4*qrk3 - dt*wr1*((qrj1*qrk1)/2 + (qrj2*qrk2)/2 + (qrj3*qrk3)/2 + (qrj4*qrk4)/2) + dt*wr2*((qrj1*qrk4)/2 - (qrj2*qrk3)/2 + (qrj3*qrk2)/2 - (qrj4*qrk1)/2) + dt*wr3*((qrj1*qrk3)/2 - (qrj3*qrk1)/2 + (qrj2*qrk4)/2 - (qrj4*qrk2)/2),      qrj1*qrk1 + qrj2*qrk2 + qrj3*qrk3 + qrj4*qrk4 - dt*wr1*((qrj1*qrk2)/2 - (qrj2*qrk1)/2 - (qrj3*qrk4)/2 + (qrj4*qrk3)/2) - dt*wr2*((qrj1*qrk3)/2 - (qrj3*qrk1)/2 + (qrj2*qrk4)/2 - (qrj4*qrk2)/2) + dt*wr3*((qrj1*qrk4)/2 - (qrj2*qrk3)/2 + (qrj3*qrk2)/2 - (qrj4*qrk1)/2);


    // att / ang_vel
    // TODO fix
    predictedState->errorStateJacobian.angular.block<3,3>(0,3)<<
         dt*(qrj1*qrk1 + qrj2*qrk2 + qrj3*qrk3 + qrj4*qrk4),     dt*(qrj1*qrk4 - qrj2*qrk3 + qrj3*qrk2 - qrj4*qrk1),     -dt*(qrj1*qrk3 - qrj3*qrk1 + qrj2*qrk4 - qrj4*qrk2),
         -dt*(qrj1*qrk4 - qrj2*qrk3 + qrj3*qrk2 - qrj4*qrk1),    dt*(qrj1*qrk1 + qrj2*qrk2 + qrj3*qrk3 + qrj4*qrk4),    dt*(qrj1*qrk2 - qrj2*qrk1 - qrj3*qrk4 + qrj4*qrk3),
         dt*(qrj1*qrk3 - qrj3*qrk1 + qrj2*qrk4 - qrj4*qrk2),     -dt*(qrj1*qrk2 - qrj2*qrk1 - qrj3*qrk4 + qrj4*qrk3),     dt*(qrj1*qrk1 + qrj2*qrk2 + qrj3*qrk3 + qrj4*qrk4);

    // att / ang_accel
    // TODO
    //predictedState->errorStateJacobian.angular.block<3,3>(0,6);



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




    return 0;
}

