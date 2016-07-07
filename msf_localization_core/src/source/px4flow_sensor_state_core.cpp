
#include "msf_localization_core/px4flow_sensor_state_core.h"


#include "msf_localization_core/px4flow_sensor_core.h"


Px4FlowSensorStateCore::Px4FlowSensorStateCore() :
    SensorStateCore()
{
    init();

    return;
}

Px4FlowSensorStateCore::Px4FlowSensorStateCore(const std::weak_ptr<MsfElementCore> msf_element_core_ptr) :
    SensorStateCore(msf_element_core_ptr)
{
    init();

    return;
}

Px4FlowSensorStateCore::~Px4FlowSensorStateCore()
{
    return;
}

int Px4FlowSensorStateCore::init()
{
    this->setSensorStateCoreType(SensorStateCoreTypes::px4flow);

    return 0;
}

/*
Eigen::Vector3d Px4FlowSensorStateCore::getBiasesAngularVelocity() const
{
    return this->biasesAngularVelocity;
}

int Px4FlowSensorStateCore::setBiasesAngularVelocity(const Eigen::Vector3d &biasesAngularVelocity)
{
    this->biasesAngularVelocity=biasesAngularVelocity;
    return 0;
}
*/


int Px4FlowSensorStateCore::updateStateFromIncrementErrorState(const Eigen::VectorXd &increment_error_state)
{
    unsigned int dimension=0;

    std::shared_ptr<Px4FlowSensorCore> px4flow_sensor_core=std::dynamic_pointer_cast<Px4FlowSensorCore>(this->getMsfElementCoreSharedPtr());

    if(px4flow_sensor_core->isEstimationPositionSensorWrtRobotEnabled())
    {
        this->positionSensorWrtRobot+=increment_error_state.block<3,1>(dimension, 0);
        dimension+=3;
    }
    if(px4flow_sensor_core->isEstimationAttitudeSensorWrtRobotEnabled())
    {
        Eigen::Vector4d DeltaQuat, DeltaQuatAux;
        double normDeltaQuatAux;
        DeltaQuatAux[0]=1;
        DeltaQuatAux.block<3,1>(1,0)=0.5*increment_error_state.block<3,1>(dimension,0);
        normDeltaQuatAux=DeltaQuatAux.norm();
        DeltaQuat=DeltaQuatAux/normDeltaQuatAux;

        Eigen::Vector4d attitudeSensorWrtRobot=Quaternion::cross(this->attitudeSensorWrtRobot, DeltaQuat);

        this->attitudeSensorWrtRobot=attitudeSensorWrtRobot;

        dimension+=3;
    }
    /*
    if(px4flow_sensor_core->isEstimationBiasLinearAccelerationEnabled())
    {
        this->biasesLinearAcceleration+=increment_error_state.block<3,1>(dimension, 0);

        //std::cout<<"biasesLinearAcceleration="<<this->biasesLinearAcceleration.transpose()<<std::endl;

        dimension+=3;
    }
    */


    return 0;
}
