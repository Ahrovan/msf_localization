
#include "msf_localization_core/imu_driven_robot_core.h"


ImuDrivenRobotCore::ImuDrivenRobotCore() :
    RobotCore()
{
    init();

    return;
}

ImuDrivenRobotCore::ImuDrivenRobotCore(std::weak_ptr<MsfStorageCore> msf_storage_core_ptr) :
    RobotCore(msf_storage_core_ptr)
{
    init();

    return;
}

ImuDrivenRobotCore::~ImuDrivenRobotCore()
{

}

int ImuDrivenRobotCore::init()
{
    // Dimensions
    dimension_state_=6+4;
    dimension_error_state_=6+3;

    dimension_parameters_=0;
    dimension_error_parameters_=0;

    dimension_noise_=0;


    // Noises
    this->noise_position_robot_wrt_world_.setZero();
    this->noise_linear_speed_robot_wrt_world_.setZero();
    this->noise_attitude_robot_wrt_world_.setZero();


    // Type
    this->setRobotCoreType(RobotCoreTypes::imu_driven);


    // End
    return 0;
}

int ImuDrivenRobotCore::readConfig(pugi::xml_node robot, std::shared_ptr<ImuDrivenRobotStateCore>& robot_init_state)
{
    // Create a class for the RobotStateCore
    if(!robot_init_state)
        robot_init_state=std::make_shared<ImuDrivenRobotStateCore>(this->getMsfElementCoreWeakPtr());


    // Aux vars
    std::string readingValue;



    /// Init State

    // Position
    readingValue=robot.child("position").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d position;
        stm>>position[0]>>position[1]>>position[2];
        robot_init_state->setPositionRobotWrtWorld(position);
    }

    // Linear Speed
    readingValue=robot.child("lin_speed").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d lin_speed;
        stm>>lin_speed[0]>>lin_speed[1]>>lin_speed[2];
        robot_init_state->setLinearSpeedRobotWrtWorld(lin_speed);
    }

    // Attitude
    readingValue=robot.child("attitude").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector4d attitude;
        stm>>attitude[0]>>attitude[1]>>attitude[2]>>attitude[3];
        robot_init_state->setAttitudeRobotWrtWorld(attitude);
    }



    /// State / Parameters Covariances

    // Position
    readingValue=robot.child("position").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoisePositionRobotWrtWorld(variance.asDiagonal());
    }

    // Linear Speed
    readingValue=robot.child("lin_speed").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoiseLinearSpeedRobotWrtWorld(variance.asDiagonal());
    }

    // Attitude
    readingValue=robot.child("attitude").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setNoiseAttitudeRobotWrtWorld(variance.asDiagonal());
    }


    // Prepare Covariance of init error state
    this->prepareCovarianceInitErrorState();



    /// Noises Estimation

    // Nothing



    // End
    return 0;
}

int ImuDrivenRobotCore::setNoisePositionRobotWrtWorld(Eigen::Matrix3d noise_position_robot_wrt_world)
{
    this->noise_position_robot_wrt_world_=noise_position_robot_wrt_world;
    return 0;
}

Eigen::Matrix3d ImuDrivenRobotCore::getNoisePositionRobotWrtWorld()
{
    return this->noise_position_robot_wrt_world_;
}

int ImuDrivenRobotCore::setNoiseLinearSpeedRobotWrtWorld(Eigen::Matrix3d noise_linear_speed_robot_wrt_world)
{
    this->noise_linear_speed_robot_wrt_world_=noise_linear_speed_robot_wrt_world;
    return 0;
}

Eigen::Matrix3d ImuDrivenRobotCore::getNoiseLinearSpeedRobotWrtWorld()
{
    return this->noise_linear_speed_robot_wrt_world_;
}

int ImuDrivenRobotCore::setNoiseAttitudeRobotWrtWorld(Eigen::Matrix3d noise_attitude_robot_wrt_world)
{
    this->noise_attitude_robot_wrt_world_=noise_attitude_robot_wrt_world;
    return 0;
}

Eigen::Matrix3d ImuDrivenRobotCore::getNoiseAttitudeRobotWrtWorld()
{
    return this->noise_attitude_robot_wrt_world_;
}

Eigen::SparseMatrix<double> ImuDrivenRobotCore::getCovarianceNoise(const TimeStamp deltaTimeStamp)
{
    Eigen::SparseMatrix<double> covariance_noise;

    // Resize
    covariance_noise.resize(dimension_noise_, dimension_noise_);
    //covariance_noise.setZero();
    covariance_noise.reserve(dimension_noise_);

    std::vector<Eigen::Triplet<double> > tripletList;

    //
    double dt=deltaTimeStamp.get_double();

    int dimension_noise_i=0;

    // Fill

    // Nothing


    covariance_noise.setFromTriplets(tripletList.begin(), tripletList.end());


    // End
    return covariance_noise;
}

int ImuDrivenRobotCore::prepareCovarianceInitErrorState()
{
    int error=MsfElementCore::prepareCovarianceInitErrorState();

    if(error)
        return error;

    this->covariance_init_error_state_.block<3,3>(0,0)=this->getNoisePositionRobotWrtWorld();
    this->covariance_init_error_state_.block<3,3>(3,3)=this->getNoiseLinearSpeedRobotWrtWorld();
    this->covariance_init_error_state_.block<3,3>(6,6)=this->getNoiseAttitudeRobotWrtWorld();

    return 0;
}

int ImuDrivenRobotCore::predictState(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, const std::shared_ptr<RobotStateCore> pastState, std::shared_ptr<RobotStateCore>& predictedState)
{

    return 0;
}

int ImuDrivenRobotCore::predictStateErrorStateJacobians(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, std::shared_ptr<RobotStateCore> pastState, std::shared_ptr<RobotStateCore>& predictedState)
{

    return 0;
}
