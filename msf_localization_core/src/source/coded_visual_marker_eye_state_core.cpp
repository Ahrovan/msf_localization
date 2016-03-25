

#include "msf_localization_core/coded_visual_marker_eye_state_core.h"

#include "msf_localization_core/coded_visual_marker_eye_core.h"


CodedVisualMarkerEyeStateCore::CodedVisualMarkerEyeStateCore() :
    SensorStateCore()
{
    init();

    return;
}

CodedVisualMarkerEyeStateCore::CodedVisualMarkerEyeStateCore(std::weak_ptr<const SensorCore> the_sensor_core_ptr) :
    SensorStateCore(the_sensor_core_ptr)
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
    // Error State Jacobian: Init to zero
    error_state_jacobian_.position_sensor_wrt_robot_.setZero();
    error_state_jacobian_.attitude_sensor_wrt_robot_.setZero();

    return 0;
}


Eigen::MatrixXd CodedVisualMarkerEyeStateCore::getJacobianErrorState()
{
    Eigen::MatrixXd jacobian_error_state;

    std::shared_ptr<const CodedVisualMarkerEyeCore> the_sensor_core=std::dynamic_pointer_cast<const CodedVisualMarkerEyeCore>(this->getTheSensorCoreShared());

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

Eigen::SparseMatrix<double> CodedVisualMarkerEyeStateCore::getJacobianErrorStateNoise()
{
    return this->jacobian_error_state_noise_;

    /*
    Eigen::MatrixXd jacobian_error_state_noise;

    std::shared_ptr<const CodedVisualMarkerEyeCore> the_sensor_core=std::dynamic_pointer_cast<const CodedVisualMarkerEyeCore>(this->getTheSensorCoreShared());

    // Resize the jacobian
    int dimension_error_state=the_sensor_core->getDimensionErrorState();
    int dimension_noise=the_sensor_core->getDimensionNoise();

    jacobian_error_state_noise.resize(dimension_error_state, dimension_noise);
    jacobian_error_state_noise.setZero();


    // Fill
    int dimension_noise_i=0;


    // Nothing to do


    // End
    return jacobian_error_state_noise;
    */
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
