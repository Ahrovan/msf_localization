#ifndef _QUATERNION_ALGEBRA_H
#define _QUATERNION_ALGEBRA_H


#include <Eigen/Dense>

#include <cmath>

class Quaternion
{
public:
    // Conjugate
    static Eigen::Vector4d conj(const Eigen::Vector4d q);

    // Inverse
    static Eigen::Vector4d inv(const Eigen::Vector4d q);

    // Cross with two arguments
    static Eigen::Vector4d cross(const Eigen::Vector4d q1, const Eigen::Vector4d q2);

    // Cross with n arguments
    template <typename ...Tail>
    static Eigen::Vector4d cross(const Eigen::Vector4d head, Tail... tail)
    {
        return cross(head, cross(tail...));
    }

    // Quaternion Matrixes
    // Q+
    static Eigen::Matrix4d quatMatPlus(const Eigen::Vector4d q);
    // Q-
    static Eigen::Matrix4d quatMatMinus(const Eigen::Vector4d q);


    // Rotation vector to quaternion
    static Eigen::Vector4d rotationVectorToQuaternion(const Eigen::Vector3d v_rot);

    static Eigen::MatrixXd jacobianRotationVectorToQuaternion(const Eigen::Vector3d v_rot);


    // Skew-Symmetric Matrix: https://en.wikipedia.org/wiki/Skew-symmetric_matrix
    static Eigen::Matrix3d skewSymMat(const Eigen::Vector3d w);

};






#endif
