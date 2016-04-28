

#include "msf_localization_core/coded_visual_marker_eye_state_core.h"

#include "msf_localization_core/coded_visual_marker_eye_core.h"


CodedVisualMarkerEyeStateCore::CodedVisualMarkerEyeStateCore() :
    SensorStateCore()
{
    init();

    return;
}

CodedVisualMarkerEyeStateCore::CodedVisualMarkerEyeStateCore(std::weak_ptr<MsfElementCore> msf_element_core_ptr) :
    SensorStateCore(msf_element_core_ptr)
{
    init();

    return;
}

CodedVisualMarkerEyeStateCore::~CodedVisualMarkerEyeStateCore()
{

    return;
}

int CodedVisualMarkerEyeStateCore::init()
{
    this->setSensorStateCoreType(SensorStateCoreTypes::coded_visual_maker_eye);

    // Error State Jacobian: Init to zero
    error_state_jacobian_.position_sensor_wrt_robot_.setZero();
    error_state_jacobian_.attitude_sensor_wrt_robot_.setZero();

    return 0;
}

/*
Eigen::MatrixXd CodedVisualMarkerEyeStateCore::getJacobianErrorState()
{
    Eigen::MatrixXd jacobian_error_state;

    std::shared_ptr<CodedVisualMarkerEyeCore> the_sensor_core=std::dynamic_pointer_cast<CodedVisualMarkerEyeCore>(this->getMsfElementCoreSharedPtr());

    // Resize the jacobian
    int dimension_error_state=the_sensor_core->getDimensionErrorState();

    jacobian_error_state.resize(dimension_error_state, dimension_error_state);
    jacobian_error_state.setZero();


    // Fill
    int dimension_error_state_i=0;

    // Position sensor wrt robot
    if(the_sensor_core->isEstimationPositionSensorWrtRobotEnabled())
    {
        // Update jacobian
        jacobian_error_state.block<3,3>(dimension_error_state_i, dimension_error_state_i)=error_state_jacobian_.position_sensor_wrt_robot_;

        // Update dimension for next
        dimension_error_state_i+=3;
    }

    // Attitude sensor wrt robot
    if(the_sensor_core->isEstimationAttitudeSensorWrtRobotEnabled())
    {
        // Update jacobian
        jacobian_error_state.block<3,3>(dimension_error_state_i, dimension_error_state_i)=error_state_jacobian_.attitude_sensor_wrt_robot_;

        // Update dimension for next
        dimension_error_state_i+=3;
    }


    // End
    return jacobian_error_state;
}
*/


int CodedVisualMarkerEyeStateCore::updateStateFromIncrementErrorState(Eigen::VectorXd increment_error_state)
{

    unsigned int dimension=0;

    std::shared_ptr<CodedVisualMarkerEyeCore> TheCodedVisualMarkerEyeCore=std::dynamic_pointer_cast<CodedVisualMarkerEyeCore>(this->getMsfElementCoreSharedPtr());

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
