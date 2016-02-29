
#include "free_model_robot_core.h"


FreeModelRobotCore::FreeModelRobotCore()
{
    dimensionState=9+7;
    dimensionErrorState=9+6;

    noiseLinearAcceleration.setZero();
    noiseAngularVelocity.setZero();

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


    // Position
    predictedState->position=pastState->position+pastState->linear_speed*DeltaTime.get_double();

    // Linear Speed
    predictedState->linear_speed=pastState->linear_speed+pastState->linear_acceleration*DeltaTime.get_double();

    // Linear Acceleration
    predictedState->linear_acceleration=pastState->linear_acceleration;


    // Attitude
    Eigen::Vector4d deltaQw;
    if(pastState->angular_velocity.norm() < 1e-3)
    {
        // Fill
        deltaQw[0]=1;
        deltaQw.block<3,1>(1,0)=pastState->angular_velocity*DeltaTime.get_double();
        // Unit quaternion
        deltaQw=deltaQw/deltaQw.norm();
    }
    else
    {
        // Fill
        deltaQw[0]=cos(pastState->angular_velocity.norm()*DeltaTime.get_double()/2);
        deltaQw.block<3,1>(1,0)=pastState->angular_velocity/pastState->angular_velocity.norm()*sin(pastState->angular_velocity.norm()*DeltaTime.get_double()/2);
        // Unit quaternion -> Not needed!
        deltaQw=deltaQw/deltaQw.norm();
    }

    predictedState->attitude=Quaternion::cross(pastState->attitude,deltaQw);

    // Unit quaternion -> Not needed, Just in case
    predictedState->attitude=predictedState->attitude/predictedState->attitude.norm();


    // Angular Velocity
    predictedState->angular_velocity=pastState->angular_velocity;



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


    // Jacobian
    // Jacobian Size
    predictedState->errorStateJacobian.linear.resize(9, 9);
    predictedState->errorStateJacobian.linear.setZero();


    predictedState->errorStateJacobian.angular.resize(6, 6);
    predictedState->errorStateJacobian.angular.setZero();



    // Linear Part


    // posi / posi
    predictedState->errorStateJacobian.linear.block<3,3>(0,0)=Eigen::MatrixXd::Identity(3,3);

    // posi / vel
    predictedState->errorStateJacobian.linear.block<3,3>(0,3)=Eigen::MatrixXd::Identity(3,3)*DeltaTime.get_double();

    // pos i/ acc
    // zero


    // vel / posi
    // zero

    // vel / vel
    predictedState->errorStateJacobian.linear.block<3,3>(3,3)=Eigen::MatrixXd::Identity(3,3);

    // vel / acc
    predictedState->errorStateJacobian.linear.block<3,3>(3,6)=Eigen::MatrixXd::Identity(3,3)*DeltaTime.get_double();




    // acc / acc
    predictedState->errorStateJacobian.linear.block<3,3>(6,6)=Eigen::MatrixXd::Identity(3,3);



    // Angular Part

    // delta time
    double dt=DeltaTime.get_double();

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
    predictedState->errorStateJacobian.angular.block<3,3>(0,0)<<
        qrj1*qrk1 + qrj2*qrk2 + qrj3*qrk3 + qrj4*qrk4 + dt*wr1*((qrj1*qrk2)/2 - (qrj2*qrk1)/2 - (qrj3*qrk4)/2 + (qrj4*qrk3)/2) - dt*wr2*((qrj1*qrk3)/2 - (qrj3*qrk1)/2 + (qrj2*qrk4)/2 - (qrj4*qrk2)/2) - dt*wr3*((qrj1*qrk4)/2 - (qrj2*qrk3)/2 + (qrj3*qrk2)/2 - (qrj4*qrk1)/2),        qrj1*qrk4 - qrj2*qrk3 + qrj3*qrk2 - qrj4*qrk1 + dt*wr1*((qrj1*qrk3)/2 - (qrj3*qrk1)/2 + (qrj2*qrk4)/2 - (qrj4*qrk2)/2) + dt*wr2*((qrj1*qrk2)/2 - (qrj2*qrk1)/2 - (qrj3*qrk4)/2 + (qrj4*qrk3)/2) + dt*wr3*((qrj1*qrk1)/2 + (qrj2*qrk2)/2 + (qrj3*qrk3)/2 + (qrj4*qrk4)/2),      qrj3*qrk1 - qrj1*qrk3 - qrj2*qrk4 + qrj4*qrk2 + dt*wr1*((qrj1*qrk4)/2 - (qrj2*qrk3)/2 + (qrj3*qrk2)/2 - (qrj4*qrk1)/2) - dt*wr2*((qrj1*qrk1)/2 + (qrj2*qrk2)/2 + (qrj3*qrk3)/2 + (qrj4*qrk4)/2) + dt*wr3*((qrj1*qrk2)/2 - (qrj2*qrk1)/2 - (qrj3*qrk4)/2 + (qrj4*qrk3)/2),
        qrj2*qrk3 - qrj1*qrk4 - qrj3*qrk2 + qrj4*qrk1 + dt*wr1*((qrj1*qrk3)/2 - (qrj3*qrk1)/2 + (qrj2*qrk4)/2 - (qrj4*qrk2)/2) + dt*wr2*((qrj1*qrk2)/2 - (qrj2*qrk1)/2 - (qrj3*qrk4)/2 + (qrj4*qrk3)/2) - dt*wr3*((qrj1*qrk1)/2 + (qrj2*qrk2)/2 + (qrj3*qrk3)/2 + (qrj4*qrk4)/2),        qrj1*qrk1 + qrj2*qrk2 + qrj3*qrk3 + qrj4*qrk4 - dt*wr1*((qrj1*qrk2)/2 - (qrj2*qrk1)/2 - (qrj3*qrk4)/2 + (qrj4*qrk3)/2) + dt*wr2*((qrj1*qrk3)/2 - (qrj3*qrk1)/2 + (qrj2*qrk4)/2 - (qrj4*qrk2)/2) - dt*wr3*((qrj1*qrk4)/2 - (qrj2*qrk3)/2 + (qrj3*qrk2)/2 - (qrj4*qrk1)/2),      qrj1*qrk2 - qrj2*qrk1 - qrj3*qrk4 + qrj4*qrk3 + dt*wr1*((qrj1*qrk1)/2 + (qrj2*qrk2)/2 + (qrj3*qrk3)/2 + (qrj4*qrk4)/2) + dt*wr2*((qrj1*qrk4)/2 - (qrj2*qrk3)/2 + (qrj3*qrk2)/2 - (qrj4*qrk1)/2) + dt*wr3*((qrj1*qrk3)/2 - (qrj3*qrk1)/2 + (qrj2*qrk4)/2 - (qrj4*qrk2)/2),
        qrj1*qrk3 - qrj3*qrk1 + qrj2*qrk4 - qrj4*qrk2 + dt*wr1*((qrj1*qrk4)/2 - (qrj2*qrk3)/2 + (qrj3*qrk2)/2 - (qrj4*qrk1)/2) + dt*wr2*((qrj1*qrk1)/2 + (qrj2*qrk2)/2 + (qrj3*qrk3)/2 + (qrj4*qrk4)/2) + dt*wr3*((qrj1*qrk2)/2 - (qrj2*qrk1)/2 - (qrj3*qrk4)/2 + (qrj4*qrk3)/2),        qrj2*qrk1 - qrj1*qrk2 + qrj3*qrk4 - qrj4*qrk3 - dt*wr1*((qrj1*qrk1)/2 + (qrj2*qrk2)/2 + (qrj3*qrk3)/2 + (qrj4*qrk4)/2) + dt*wr2*((qrj1*qrk4)/2 - (qrj2*qrk3)/2 + (qrj3*qrk2)/2 - (qrj4*qrk1)/2) + dt*wr3*((qrj1*qrk3)/2 - (qrj3*qrk1)/2 + (qrj2*qrk4)/2 - (qrj4*qrk2)/2),      qrj1*qrk1 + qrj2*qrk2 + qrj3*qrk3 + qrj4*qrk4 - dt*wr1*((qrj1*qrk2)/2 - (qrj2*qrk1)/2 - (qrj3*qrk4)/2 + (qrj4*qrk3)/2) - dt*wr2*((qrj1*qrk3)/2 - (qrj3*qrk1)/2 + (qrj2*qrk4)/2 - (qrj4*qrk2)/2) + dt*wr3*((qrj1*qrk4)/2 - (qrj2*qrk3)/2 + (qrj3*qrk2)/2 - (qrj4*qrk1)/2);


    // att / ang_vel
    predictedState->errorStateJacobian.angular.block<3,3>(0,3)<<
         dt*(qrj1*qrk1 + qrj2*qrk2 + qrj3*qrk3 + qrj4*qrk4),     dt*(qrj1*qrk4 - qrj2*qrk3 + qrj3*qrk2 - qrj4*qrk1),     -dt*(qrj1*qrk3 - qrj3*qrk1 + qrj2*qrk4 - qrj4*qrk2),
         -dt*(qrj1*qrk4 - qrj2*qrk3 + qrj3*qrk2 - qrj4*qrk1),    dt*(qrj1*qrk1 + qrj2*qrk2 + qrj3*qrk3 + qrj4*qrk4),    dt*(qrj1*qrk2 - qrj2*qrk1 - qrj3*qrk4 + qrj4*qrk3),
         dt*(qrj1*qrk3 - qrj3*qrk1 + qrj2*qrk4 - qrj4*qrk2),     -dt*(qrj1*qrk2 - qrj2*qrk1 - qrj3*qrk4 + qrj4*qrk3),     dt*(qrj1*qrk1 + qrj2*qrk2 + qrj3*qrk3 + qrj4*qrk4);


    // ang_vel / ang_vel
    predictedState->errorStateJacobian.angular.block<3,3>(3,3)=Eigen::MatrixXd::Identity(3,3);




    return 0;
}

