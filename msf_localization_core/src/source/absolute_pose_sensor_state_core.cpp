
#include "msf_localization_core/absolute_pose_sensor_state_core.h"

#include "msf_localization_core/absolute_pose_sensor_core.h"


AbsolutePoseSensorStateCore::AbsolutePoseSensorStateCore() :
    SensorStateCore()
{
    init();

    return;
}

AbsolutePoseSensorStateCore::AbsolutePoseSensorStateCore(std::weak_ptr<MsfElementCore> msf_element_core_ptr) :
    SensorStateCore(msf_element_core_ptr)
{
    init();

    return;
}

AbsolutePoseSensorStateCore::~AbsolutePoseSensorStateCore()
{

    return;
}

int AbsolutePoseSensorStateCore::init()
{
    this->setSensorStateCoreType(SensorStateCoreTypes::mocap);

    return 0;
}


int AbsolutePoseSensorStateCore::updateStateFromIncrementErrorState(const Eigen::VectorXd &increment_error_state)
{

    unsigned int dimension=0;

    std::shared_ptr<AbsolutePoseSensorCore> sensor_core=std::dynamic_pointer_cast<AbsolutePoseSensorCore>(this->getMsfElementCoreSharedPtr());

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
