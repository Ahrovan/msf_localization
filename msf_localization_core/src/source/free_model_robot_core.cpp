
#include "free_model_robot_core.h"


FreeModelRobotCore::FreeModelRobotCore()
{
    return;
}

FreeModelRobotCore::~FreeModelRobotCore()
{
    return;
}


int FreeModelRobotCore::predictState(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, const std::shared_ptr<FreeModelRobotStateCore> pastState, std::shared_ptr<FreeModelRobotStateCore>& predictedState)
{
    std::cout<<"FreeModelRobotCore::predictState"<<std::endl;

    // Create the predicted state if it doen't exists
    if(!predictedState)
    {
        predictedState=std::make_shared<FreeModelRobotStateCore>();
    }

    // Set The core
    predictedState->setTheRobotCore(pastState->getTheRobotCore());


    // Equations
    //*predictedState=*pastState;

    //Delta Time
    TimeStamp DeltaTime=currentTimeStamp-previousTimeStamp;


    // Position
    predictedState->position=pastState->position+pastState->linear_speed*DeltaTime.get_double();

    // Linear Speed
    predictedState->linear_speed=pastState->linear_speed+pastState->linear_acceleration*DeltaTime.get_double();

    // Linear Acceleration
    predictedState->linear_acceleration=pastState->linear_acceleration;


    // Attitude
    predictedState->attitude=pastState->attitude;

    // Angular Velocity
    predictedState->angular_velocity=pastState->angular_velocity;




    return 0;
}

// Jacobian
int FreeModelRobotCore::predictStateJacobians(TimeStamp theTimeStamp, std::shared_ptr<FreeModelRobotStateCore> currentState)
{

    return 0;
}

