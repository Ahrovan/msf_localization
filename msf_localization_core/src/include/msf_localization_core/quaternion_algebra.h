#ifndef _QUATERNION_ALGEBRA_H
#define _QUATERNION_ALGEBRA_H


#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <cmath>

namespace Quaternion
{
    // Typedef
    typedef Eigen::Vector4d Quaternion;
    typedef Eigen::Vector3d PureQuaternion;


    // Class for Usefull Jacobians related to Quaternions
    extern class Jacobians
    {
        public:
            Jacobians();

        public:
            // Jacobian error quaternion wrt error theta
            Eigen::Matrix<double, 4, 3> mat_diff_error_quat_wrt_error_theta_dense;
            Eigen::SparseMatrix<double> mat_diff_error_quat_wrt_error_theta_sparse;

            // Jacobian error theta wrt error quaternion
            Eigen::Matrix<double, 3, 4> mat_diff_error_theta_wrt_error_quat_dense;
            Eigen::SparseMatrix<double> mat_diff_error_theta_wrt_error_quat_sparse;

            //
            Eigen::Matrix<double, 4, 4> mat_diff_quat_inv_wrt_quat_dense;
            Eigen::SparseMatrix<double> mat_diff_quat_inv_wrt_quat_sparse;

            //
            Eigen::Matrix<double, 3, 4> mat_diff_vector_wrt_vector_amp_dense;
            Eigen::SparseMatrix<double> mat_diff_vector_wrt_vector_amp_sparse;

            //
            Eigen::Matrix<double, 4, 3> mat_diff_vector_amp_wrt_vector_dense;
            Eigen::SparseMatrix<double> mat_diff_vector_amp_wrt_vector_sparse;


    } jacobians;


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

    Eigen::Matrix<double, 4, 3> jacobianRotationVectorToQuaternion(const Eigen::Vector3d& v_rot);


    // Skew-Symmetric Matrix: https://en.wikipedia.org/wiki/Skew-symmetric_matrix
    Eigen::Matrix3d skewSymMat(const Eigen::Vector3d& w);

}






#endif
