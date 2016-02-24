#ifndef _QUATERNION_ALGEBRA_H
#define _QUATERNION_ALGEBRA_H


#include <Eigen/Dense>



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
        return cross(tail...);
    }


};






#endif
