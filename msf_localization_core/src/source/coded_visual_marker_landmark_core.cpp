
#include "msf_localization_core/coded_visual_marker_landmark_core.h"

CodedVisualMarkerLandmarkCore::CodedVisualMarkerLandmarkCore():
    MapElementCore()
{
    dimensionState=3+4;
    dimensionErrorState=3+3;

    dimensionParameters=0;
    dimensionErrorParameters=0;


    // TODO -> This is not the best moment to do this! The state might change!
    InitErrorStateVariance.resize(dimensionErrorState, dimensionErrorState);
    InitErrorStateVariance.setZero();

    return;
}

CodedVisualMarkerLandmarkCore::~CodedVisualMarkerLandmarkCore()
{
    return;
}

int CodedVisualMarkerLandmarkCore::setInitErrorStateVariancePosition(Eigen::Vector3d initVariance)
{
    this->InitErrorStateVariance.block<3,3>(0,0)=initVariance.asDiagonal();
    return 0;
}

int CodedVisualMarkerLandmarkCore::setInitErrorStateVarianceAttitude(Eigen::Vector3d initVariance)
{
    this->InitErrorStateVariance.block<3,3>(3,3)=initVariance.asDiagonal();
    return 0;
}

int CodedVisualMarkerLandmarkCore::predictState(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, const std::shared_ptr<CodedVisualMarkerLandmarkStateCore> pastState, std::shared_ptr<CodedVisualMarkerLandmarkStateCore>& predictedState)
{
    // Checks in the past state
    if(!pastState->getTheMapElementCore())
    {
        return -5;
        std::cout<<"FreeModelRobotCore::predictState() error !pastState->getTheRobotCore()"<<std::endl;
    }


    // Create the predicted state if it doesn't exist
    if(!predictedState)
    {
        predictedState=std::make_shared<CodedVisualMarkerLandmarkStateCore>(pastState->getTheMapElementCoreWeak());
    }

    // Set The robot core if it doesn't exist
    if(predictedState->getTheMapElementCoreWeak().expired())
    {
        predictedState->setTheMapElementCore(pastState->getTheMapElementCoreWeak());
    }


    // Equations


    //Delta Time
    TimeStamp DeltaTime=currentTimeStamp-previousTimeStamp;
    double dt=DeltaTime.get_double();


    /// Position
    predictedState->position_=pastState->position_;


    /// Attitude
    predictedState->attitude_=pastState->attitude_;


    return 0;
}

// Jacobian
int CodedVisualMarkerLandmarkCore::predictStateErrorStateJacobians(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, std::shared_ptr<CodedVisualMarkerLandmarkStateCore> pastState, std::shared_ptr<CodedVisualMarkerLandmarkStateCore>& predictedState)
{

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
    predictedState->error_state_jacobian_.linear.resize(3, 3);
    predictedState->error_state_jacobian_.linear.setZero();


    predictedState->error_state_jacobian_.angular.resize(3, 3);
    predictedState->error_state_jacobian_.angular.setZero();



    /// Jacobian of the error: Linear Part

    // posi / posi
    predictedState->error_state_jacobian_.linear.block<3,3>(0,0)=Eigen::MatrixXd::Identity(3,3);


    /// Jacobian of the error -> Angular Part

    // att / att
    predictedState->error_state_jacobian_.angular.block<3,3>(0,0)=Eigen::MatrixXd::Identity(3,3);





    return 0;
}

