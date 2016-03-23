

#include "msf_localization_core/coded_visual_marker_eye_state_core.h"

#include "msf_localization_core/coded_visual_marker_eye_core.h"


CodedVisualMarkerEyeStateCore::CodedVisualMarkerEyeStateCore()
{
    // Error State Jacobian: Init to zero
    errorStateJacobian.positionSensorWrtRobot.setZero();
    errorStateJacobian.attitudeSensorWrtRobot.setZero();

    return;
}

CodedVisualMarkerEyeStateCore::~CodedVisualMarkerEyeStateCore()
{

    return;
}

int CodedVisualMarkerEyeStateCore::updateStateFromIncrementErrorState(Eigen::VectorXd increment_error_state)
{

    unsigned int dimension=0;

    std::shared_ptr<const CodedVisualMarkerEyeCore> TheCodedVisualMarkerEyeCore=std::dynamic_pointer_cast<const CodedVisualMarkerEyeCore>(this->getTheSensorCoreShared());

    if(TheCodedVisualMarkerEyeCore->isEstimationPositionSensorWrtRobotEnabled())
    {
        this->positionSensorWrtRobot+=increment_error_state.block<3,1>(dimension, 0);
        dimension+=3;
    }
    if(TheCodedVisualMarkerEyeCore->isEstimationAttitudeSensorWrtRobotEnabled())
    {
        Eigen::Vector4d DeltaQuat;
        DeltaQuat[0]=1;
        DeltaQuat.block<3,1>(1,0)=0.5*increment_error_state.block<3,1>(dimension,0);
        DeltaQuat=DeltaQuat/DeltaQuat.norm();

        this->attitudeSensorWrtRobot=Quaternion::cross(this->attitudeSensorWrtRobot, DeltaQuat);
        dimension+=3;
    }


    return 0;
}
