#ifndef _QUATERNION_ALGEBRA_H
#define _QUATERNION_ALGEBRA_H


#include <Eigen/Dense>

#include <cmath>

namespace Quaternion
{
    // Typedef
    typedef Eigen::Vector4d Quaternion;
    typedef Eigen::Vector3d PureQuaternion;


    // Conjugate
    Quaternion conj(const Quaternion& q);

    // Inverse
    Eigen::Vector4d inv(const Eigen::Vector4d &q);

    // Cross with two arguments
    Eigen::Vector4d cross(const Eigen::Vector4d& q1, const Eigen::Vector4d& q2);
    Eigen::Vector4d cross_gen_pure(const Eigen::Vector4d& q1, const Eigen::Vector3d& q2);
    Eigen::Vector4d cross_pure_gen(const Eigen::Vector3d& q1, const Eigen::Vector4d& q2);
    Eigen::Vector4d cross_pure_pure(const Eigen::Vector3d& q1, const Eigen::Vector3d& q2);

    // Cross with n arguments for general quaternions
    template <typename ...Tail>
    Eigen::Vector4d cross(const Eigen::Vector4d& head, Tail... tail)
    {
        return cross(head, cross(tail...));
    }

    // Sandwich product
     Eigen::Vector3d cross_sandwich(const Eigen::Vector4d& q1, const Eigen::Vector3d& q2, const Eigen::Vector4d& q3);

    // Quaternion Matrixes
    // Q+
    Eigen::Matrix4d quatMatPlus(const Eigen::Vector4d& q);
    Eigen::Matrix4d quatMatPlus(const Eigen::Vector3d& q);
    // Q-
    Eigen::Matrix4d quatMatMinus(const Eigen::Vector4d& q);
    Eigen::Matrix4d quatMatMinus(const Eigen::Vector3d& q);


    // Rotation vector to quaternion
    Eigen::Vector4d rotationVectorToQuaternion(const Eigen::Vector3d& v_rot);

    Eigen::MatrixXd jacobianRotationVectorToQuaternion(const Eigen::Vector3d& v_rot);


    // Skew-Symmetric Matrix: https://en.wikipedia.org/wiki/Skew-symmetric_matrix
    Eigen::Matrix3d skewSymMat(const Eigen::Vector3d& w);

}






#endif
