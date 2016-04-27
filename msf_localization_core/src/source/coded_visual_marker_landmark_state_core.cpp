
#include "msf_localization_core/coded_visual_marker_landmark_core.h"


CodedVisualMarkerLandmarkStateCore::CodedVisualMarkerLandmarkStateCore() :
    MapElementStateCore()
{
    init();

    return;
}

CodedVisualMarkerLandmarkStateCore::CodedVisualMarkerLandmarkStateCore(std::weak_ptr<MsfElementCore> msf_element_core_ptr) :
    MapElementStateCore(msf_element_core_ptr)
{
    init();

    return;
}

int CodedVisualMarkerLandmarkStateCore::init()
{

    return 0;
}

CodedVisualMarkerLandmarkStateCore::~CodedVisualMarkerLandmarkStateCore()
{
    return;
}

Eigen::Vector3d CodedVisualMarkerLandmarkStateCore::getPosition() const
{
    return this->position_;
}

int CodedVisualMarkerLandmarkStateCore::setPosition(Eigen::Vector3d position)
{
    this->position_=position;
    return 0;
}

Eigen::Vector4d CodedVisualMarkerLandmarkStateCore::getAttitude() const
{
    return this->attitude_;
}

int CodedVisualMarkerLandmarkStateCore::setAttitude(Eigen::Vector4d attitude)
{
    this->attitude_=attitude;
    return 0;
}

Eigen::SparseMatrix<double> CodedVisualMarkerLandmarkStateCore::getJacobianErrorState()
{
    return this->jacobian_error_state_;
}

Eigen::SparseMatrix<double> CodedVisualMarkerLandmarkStateCore::getJacobianErrorStateNoise()
{
    return this->jacobian_error_state_noise_;
}

Eigen::MatrixXd CodedVisualMarkerLandmarkStateCore::getJacobianMappingRobotErrorState()
{
    return this->jacobian_mapping_error_state_.jacobian_mapping_robot_error_state_;
}

Eigen::MatrixXd CodedVisualMarkerLandmarkStateCore::getJacobianMappingGlobalParametersErrorState()
{
    return this->jacobian_mapping_error_state_.jacobian_mapping_global_parameters_error_state_;
}

Eigen::MatrixXd CodedVisualMarkerLandmarkStateCore::getJacobianMappingSensorErrorState()
{
    return this->jacobian_mapping_error_state_.jacobian_mapping_sensor_error_state_;
}

Eigen::MatrixXd CodedVisualMarkerLandmarkStateCore::getJacobianMappingErrorStateNoise()
{
    return this->jacobian_mapping_error_state_noise_;
}

int CodedVisualMarkerLandmarkStateCore::updateStateFromIncrementErrorState(Eigen::VectorXd increment_error_state)
{

    position_+=increment_error_state.block<3,1>(0,0);


    Eigen::Vector4d DeltaQuat, DeltaQuatAux;
    double NormDeltaQuatAux;
    DeltaQuatAux[0]=1;
    DeltaQuatAux.block<3,1>(1,0)=0.5*increment_error_state.block<3,1>(3,0);
    NormDeltaQuatAux=DeltaQuatAux.norm();
    DeltaQuat=DeltaQuatAux/NormDeltaQuatAux;

    Eigen::Vector4d attitude_aux=Quaternion::cross(attitude_, DeltaQuat);

//    if(attitude_aux[0]<0)
//    {
//        attitude_=-attitude_aux;
//        std::cout<<"CodedVisualMarkerLandmarkStateCore::updateStateFromIncrementErrorState() quaternion!!"<<std::endl;
//    }
//    else
        attitude_=attitude_aux;


    return 0;
}
