
#include "msf_localization_core/imu_sensor_state_core.h"


#include "msf_localization_core/imu_sensor_core.h"


ImuSensorStateCore::ImuSensorStateCore() :
    SensorStateCore()
{
    init();

    return;
}

ImuSensorStateCore::ImuSensorStateCore(std::weak_ptr<const SensorCore> TheSensorCorePtr) :
    SensorStateCore(TheSensorCorePtr)
{
    init();

    return;
}

ImuSensorStateCore::~ImuSensorStateCore()
{
    return;
}

int ImuSensorStateCore::init()
{
    // Error State Jacobian: Init to zero
    errorStateJacobian.positionSensorWrtRobot.setZero();
    errorStateJacobian.attitudeSensorWrtRobot.setZero();
    errorStateJacobian.biasesLinearAcceleration.setZero();
    errorStateJacobian.biasesAngularVelocity.setZero();

    return 0;
}


Eigen::Vector3d ImuSensorStateCore::getBiasesAngularVelocity() const
{
    return this->biasesAngularVelocity;
}

int ImuSensorStateCore::setBiasesAngularVelocity(Eigen::Vector3d biasesAngularVelocity)
{
    this->biasesAngularVelocity=biasesAngularVelocity;
    return 0;
}

Eigen::Vector3d ImuSensorStateCore::getScaleAngularVelocity() const
{
    return this->scaleAngularVelocity;
}

int ImuSensorStateCore::setScaleAngularVelocity(Eigen::Vector3d scaleAngularVelocity)
{
    this->scaleAngularVelocity=scaleAngularVelocity;
    return 0;
}


Eigen::Vector3d ImuSensorStateCore::getBiasesLinearAcceleration() const
{
    return this->biasesLinearAcceleration;
}

int ImuSensorStateCore::setBiasesLinearAcceleration(Eigen::Vector3d biasesLinearAcceleration)
{
    this->biasesLinearAcceleration=biasesLinearAcceleration;
    return 0;
}

Eigen::Vector3d ImuSensorStateCore::getScaleLinearAcceleration() const
{
    return this->scaleLinearAcceleration;
}

int ImuSensorStateCore::setScaleLinearAcceleration(Eigen::Vector3d scaleLinearAcceleration)
{
    this->scaleLinearAcceleration=scaleLinearAcceleration;
    return 0;
}

Eigen::MatrixXd ImuSensorStateCore::getJacobianErrorState()
{
    Eigen::MatrixXd jacobian_error_state;

    std::shared_ptr<const ImuSensorCore> the_imu_sensor_core=std::dynamic_pointer_cast<const ImuSensorCore>(this->getTheSensorCoreShared());

    // Resize the jacobian
    int dimension_error_state=the_imu_sensor_core->getDimensionErrorState();

    jacobian_error_state.resize(dimension_error_state, dimension_error_state);
    jacobian_error_state.setZero();


    // Fill
    int dimension_error_state_i=0;

    // Position sensor wrt robot
    if(the_imu_sensor_core->isEstimationPositionSensorWrtRobotEnabled())
    {
        // Update jacobian
        jacobian_error_state.block<3,3>(dimension_error_state_i, dimension_error_state_i)=errorStateJacobian.positionSensorWrtRobot;

        // Update dimension for next
        dimension_error_state_i+=3;
    }

    // Attitude sensor wrt robot
    if(the_imu_sensor_core->isEstimationAttitudeSensorWrtRobotEnabled())
    {
        // Update jacobian
        jacobian_error_state.block<3,3>(dimension_error_state_i, dimension_error_state_i)=errorStateJacobian.attitudeSensorWrtRobot;

        // Update dimension for next
        dimension_error_state_i+=3;
    }

    // bias linear acceleration
    if(the_imu_sensor_core->isEstimationBiasLinearAccelerationEnabled())
    {
        // Update jacobian
        jacobian_error_state.block<3,3>(dimension_error_state_i, dimension_error_state_i)=errorStateJacobian.biasesLinearAcceleration;

        // Update dimension for next
        dimension_error_state_i+=3;
    }

    // Ka
    // TODO

    // bias angular velocity
    if(the_imu_sensor_core->isEstimationBiasAngularVelocityEnabled())
    {
        // Update jacobian
        jacobian_error_state.block<3,3>(dimension_error_state_i, dimension_error_state_i)=errorStateJacobian.biasesAngularVelocity;

        // Update dimension for next
        dimension_error_state_i+=3;
    }

    // Kw
    // TODO


    // End
    return jacobian_error_state;
}


Eigen::SparseMatrix<double> ImuSensorStateCore::getJacobianErrorStateNoise()
{
    return jacobianErrorStateNoise;

    /*

    Eigen::MatrixXd jacobian_error_state_noise;

    std::shared_ptr<const ImuSensorCore> the_imu_sensor_core=std::dynamic_pointer_cast<const ImuSensorCore>(this->getTheSensorCoreShared());

    // Resize the jacobian
    int dimension_error_state=the_imu_sensor_core->getDimensionErrorState();
    int dimension_noise=the_imu_sensor_core->getDimensionNoise();

    jacobian_error_state_noise.resize(dimension_error_state, dimension_noise);
    jacobian_error_state_noise.setZero();


    // Fill
    int dimension_noise_i=0;


    // bias linear acceleration
    if(the_imu_sensor_core->isEstimationBiasLinearAccelerationEnabled())
    {
        int dimension_error_state_i=0;

        if(the_imu_sensor_core->isEstimationPositionSensorWrtRobotEnabled())
            dimension_error_state_i+=3;

        if(the_imu_sensor_core->isEstimationAttitudeSensorWrtRobotEnabled())
            dimension_error_state_i+=3;


        // Update jacobian
        jacobian_error_state_noise.block<3,3>(dimension_error_state_i, dimension_noise_i)=Eigen::MatrixXd::Identity(3,3);

        // Update dimension for next
        dimension_noise_i+=3;
    }

    // bias angular velocity
    if(the_imu_sensor_core->isEstimationBiasAngularVelocityEnabled())
    {
        int dimension_error_state_i=0;

        if(the_imu_sensor_core->isEstimationPositionSensorWrtRobotEnabled())
            dimension_error_state_i+=3;

        if(the_imu_sensor_core->isEstimationAttitudeSensorWrtRobotEnabled())
            dimension_error_state_i+=3;

        if(the_imu_sensor_core->isEstimationBiasLinearAccelerationEnabled())
            dimension_error_state_i+=3;

        if(the_imu_sensor_core->isEstimationScaleLinearAccelerationEnabled())
            dimension_error_state_i+=3;

        // Update jacobian
        jacobian_error_state_noise.block<3,3>(dimension_error_state_i, dimension_noise_i)=Eigen::MatrixXd::Identity(3,3);

        // Update dimension for next
        dimension_noise_i+=3;
    }


    // End
    return jacobian_error_state_noise;

    */

}

int ImuSensorStateCore::updateStateFromIncrementErrorState(Eigen::VectorXd increment_error_state)
{
    unsigned int dimension=0;

    std::shared_ptr<const ImuSensorCore> TheImuSensorCore=std::dynamic_pointer_cast<const ImuSensorCore>(this->getTheSensorCore());

    if(TheImuSensorCore->isEstimationPositionSensorWrtRobotEnabled())
    {
        this->positionSensorWrtRobot+=increment_error_state.block<3,1>(dimension, 0);
        dimension+=3;
    }
    if(TheImuSensorCore->isEstimationAttitudeSensorWrtRobotEnabled())
    {
        Eigen::Vector4d DeltaQuat, DeltaQuatAux;
        double normDeltaQuatAux;
        DeltaQuatAux[0]=1;
        DeltaQuatAux.block<3,1>(1,0)=0.5*increment_error_state.block<3,1>(dimension,0);
        normDeltaQuatAux=DeltaQuatAux.norm();
        DeltaQuat=DeltaQuatAux/normDeltaQuatAux;

        Eigen::Vector4d attitudeSensorWrtRobot=Quaternion::cross(this->attitudeSensorWrtRobot, DeltaQuat);

//        if(attitudeSensorWrtRobot[0]<0)
//        {
//            this->attitudeSensorWrtRobot=-attitudeSensorWrtRobot;
//            std::cout<<"ImuSensorStateCore::updateStateFromIncrementErrorState() quaternion!!"<<std::endl;
//        }
//        else
            this->attitudeSensorWrtRobot=attitudeSensorWrtRobot;

        dimension+=3;
    }
    if(TheImuSensorCore->isEstimationBiasLinearAccelerationEnabled())
    {
        this->biasesLinearAcceleration+=increment_error_state.block<3,1>(dimension, 0);

        std::cout<<"biasesLinearAcceleration="<<this->biasesLinearAcceleration.transpose()<<std::endl;

        dimension+=3;
    }
    if(TheImuSensorCore->isEstimationScaleLinearAccelerationEnabled())
    {
        this->scaleLinearAcceleration+=increment_error_state.block<3,1>(dimension, 0);
        dimension+=3;
    }
    if(TheImuSensorCore->isEstimationBiasAngularVelocityEnabled())
    {
        this->biasesAngularVelocity+=increment_error_state.block<3,1>(dimension, 0);

        std::cout<<"biasesAngularVelocity="<<this->biasesAngularVelocity.transpose()<<std::endl;

        dimension+=3;
    }
    if(TheImuSensorCore->isEstimationScaleAngularVelocityEnabled())
    {
        this->scaleAngularVelocity+=increment_error_state.block<3,1>(dimension, 0);
        dimension+=3;
    }


    return 0;
}
