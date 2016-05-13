
#include "msf_localization_core/mocap_sensor_state_core.h"

#include "msf_localization_core/mocap_sensor_core.h"


MocapSensorStateCore::MocapSensorStateCore() :
    SensorStateCore()
{
    init();

    return;
}

MocapSensorStateCore::MocapSensorStateCore(std::weak_ptr<MsfElementCore> msf_element_core_ptr) :
    SensorStateCore(msf_element_core_ptr)
{
    init();

    return;
}

MocapSensorStateCore::~MocapSensorStateCore()
{

    return;
}

int MocapSensorStateCore::init()
{
    this->setSensorStateCoreType(SensorStateCoreTypes::mocap);

    // Error State Jacobian: Init to zero
    error_state_jacobian_.position_sensor_wrt_robot_.setZero();
    error_state_jacobian_.attitude_sensor_wrt_robot_.setZero();

    return 0;
}


int MocapSensorStateCore::updateStateFromIncrementErrorState(const Eigen::VectorXd &increment_error_state)
{

    unsigned int dimension=0;

    std::shared_ptr<MocapSensorCore> sensor_core=std::dynamic_pointer_cast<MocapSensorCore>(this->getMsfElementCoreSharedPtr());

    if(sensor_core->isEstimationPositionSensorWrtRobotEnabled())
    {
        this->positionSensorWrtRobot+=increment_error_state.block<3,1>(dimension, 0);
        dimension+=3;
    }
    if(sensor_core->isEstimationAttitudeSensorWrtRobotEnabled())
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
