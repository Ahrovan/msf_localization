
#include "msf_localization_core/absolute_pose_input_core.h"


AbsolutePoseInputCore::AbsolutePoseInputCore()
{

}

AbsolutePoseInputCore::AbsolutePoseInputCore(std::weak_ptr<MsfStorageCore> the_msf_storage_core)
{

}

AbsolutePoseInputCore::~AbsolutePoseInputCore()
{

}

int AbsolutePoseInputCore::init()
{

}

int AbsolutePoseInputCore::readConfig(pugi::xml_node input, std::shared_ptr<AbsolutePoseInputStateCore> &init_state_core)
{

}

bool AbsolutePoseInputCore::isInputCommandPositionRobotWrtWorldEnabled() const
{

}

int AbsolutePoseInputCore::enableInputCommandPositionRobotWrtWorld()
{

}

Eigen::Matrix3d AbsolutePoseInputCore::getNoiseInputCommandPositionRobotWrtWorld() const
{

}

int AbsolutePoseInputCore::setNoiseInputCommandPositionRobotWrtWorld(const Eigen::Matrix3d& noise_input_command_position_robot_wrt_world)
{

}

bool AbsolutePoseInputCore::isInputCommandAttitudeRobotWrtWorldEnabled() const
{

}

int AbsolutePoseInputCore::enableInputCommandAttitudeRobotWrtWorld()
{

}

Eigen::Matrix3d AbsolutePoseInputCore::getNoiseInputCommandAttitudeRobotWrtWorld() const
{

}

int AbsolutePoseInputCore::setNoiseInputCommandAttitudeRobotWrtWorld(Eigen::Matrix3d noise_input_command_attitude_robot_wrt_world)
{

}

int AbsolutePoseInputCore::setInputCommand(const TimeStamp time_stamp, std::shared_ptr<AbsolutePoseInputCommandCore> input_command_core)
{

}

int AbsolutePoseInputCore::prepareCovarianceInitErrorStateSpecific()
{

}

Eigen::SparseMatrix<double> AbsolutePoseInputCore::getCovarianceInputs(const TimeStamp deltaTimeStamp)
{

}

Eigen::SparseMatrix<double> AbsolutePoseInputCore::getCovarianceParameters()
{

}

Eigen::SparseMatrix<double> AbsolutePoseInputCore::getCovarianceNoise(const TimeStamp deltaTimeStamp)
{

}

int AbsolutePoseInputCore::predictState(//Time
                                         const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp,
                                         // Previous State
                                         const std::shared_ptr<StateEstimationCore> pastState,
                                         // Inputs
                                         const std::shared_ptr<InputCommandComponent> inputCommand,
                                         // Predicted State
                                         std::shared_ptr<StateCore>& predictedState)
{

}

int AbsolutePoseInputCore::predictErrorStateJacobian(//Time
                                                     const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp,
                                                     // Previous State
                                                     const std::shared_ptr<StateEstimationCore> pastState,
                                                      // Inputs
                                                      const std::shared_ptr<InputCommandComponent> inputCommand,
                                                     // Predicted State
                                                     std::shared_ptr<StateCore>& predictedState)
{

}

int AbsolutePoseInputCore::resetErrorStateJacobian(// Time
                                                    const TimeStamp& current_time_stamp,
                                                    // Increment Error State
                                                    const Eigen::VectorXd& increment_error_state,
                                                    // Current State
                                                    std::shared_ptr<StateCore>& current_state
                                                    )
{

}
