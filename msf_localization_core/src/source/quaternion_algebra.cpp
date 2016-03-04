
#include "msf_localization_core/quaternion_algebra.h"



Eigen::Vector4d Quaternion::conj(const Eigen::Vector4d q)
{
    Eigen::Vector4d qr;

    qr[0]=q[0];
    qr.block<3,1>(1,0)=-q.block<3,1>(1,0);

    return qr;
}


Eigen::Vector4d Quaternion::inv(const Eigen::Vector4d q)
{
    Eigen::Vector4d qr;


    qr=Quaternion::conj(q)/q.norm();


    return qr;
}



Eigen::Vector4d Quaternion::cross(const Eigen::Vector4d q1, const Eigen::Vector4d q2)
{
    Eigen::Vector4d qres;


    // Quat 1
    double pr=q1[0];
    Eigen::Vector3d pv=q1.block<3,1>(1,0);

    // Quat 2
    double qr=q2[0];
    Eigen::Vector3d qv=q2.block<3,1>(1,0);

    // Result
    qres[0]=pr*qr-pv.transpose()*qv;
    qres.block<3,1>(1,0)=pr*qv+qr*pv+pv.cross(qv);


    return qres;
}
