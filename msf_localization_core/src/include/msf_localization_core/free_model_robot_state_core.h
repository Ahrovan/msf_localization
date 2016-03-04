
#ifndef _FREE_MODEL_ROBOT_STATE_CORE_H
#define _FREE_MODEL_ROBOT_STATE_CORE_H



#include <Eigen/Dense>
#include <Eigen/Sparse>


#include "robot_state_core.h"





class FreeModelRobotStateCore : public RobotStateCore
{
public:
    FreeModelRobotStateCore();
    ~FreeModelRobotStateCore();


    // State: xR=[pos (3/3), lin_speed (3/3), lin_accel (3/3), attit (4/3), ang_vel (3/3), ang_acc (3/3)]'

protected:
public:
    Eigen::Vector3d position;
public:
    Eigen::Vector3d getPosition() const;
    int setPosition(Eigen::Vector3d position);



protected:
public:
    Eigen::Vector3d linear_speed;
public:
    Eigen::Vector3d getLinearSpeed() const;
    int setLinearSpeed(Eigen::Vector3d linear_speed);



protected:
public:
    Eigen::Vector3d linear_acceleration;
public:
    Eigen::Vector3d getLinearAcceleration() const;
    int setLinearAcceleration(Eigen::Vector3d linear_acceleration);



protected:
public:
    Eigen::Vector4d attitude;
public:
    Eigen::Vector4d getAttitude() const;
    int setAttitude(Eigen::Vector4d attitude);



protected:
public:
    Eigen::Vector3d angular_velocity;
public:
    Eigen::Vector3d getAngularVelocity() const;
    int setAngularVelocity(Eigen::Vector3d angular_velocity);



protected:
public:
    Eigen::Vector3d angular_acceleration;
public:
    Eigen::Vector3d getAngularAcceleration() const;
    int setAngularAcceleration(Eigen::Vector3d angular_acceleration);




    // Error State Jacobians (18 x 18)
public:
    struct
    {
        // Fx_robot linear (9 x 9)
        Eigen::MatrixXd linear;

        // Fx_robot angular (9 x 9)
        Eigen::MatrixXd angular;
    } errorStateJacobian;



};





#endif
