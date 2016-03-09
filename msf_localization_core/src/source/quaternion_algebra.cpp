
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


// Q+
Eigen::Matrix4d Quaternion::quatMatPlus(const Eigen::Vector4d q)
{
    Eigen::Matrix4d QuatMat=q[0]*Eigen::Matrix4d::Identity(4,4);

    QuatMat.block<1,3>(0,1)+=-(q.block<3,1>(1,0)).transpose();
    QuatMat.block<3,3>(1,1)+=skewSymMat(q.block<3,1>(1,0));
    QuatMat.block<3,1>(1,0)+=(q.block<3,1>(1,0));

    return QuatMat;
}

// Q-
Eigen::Matrix4d Quaternion::quatMatMinus(const Eigen::Vector4d q)
{
    Eigen::Matrix4d QuatMat=q[0]*Eigen::Matrix4d::Identity(4,4);

    QuatMat.block<1,3>(0,1)+=-(q.block<3,1>(1,0)).transpose();
    QuatMat.block<3,3>(1,1)+=-skewSymMat(q.block<3,1>(1,0));
    QuatMat.block<3,1>(1,0)+=(q.block<3,1>(1,0));

    return QuatMat;
}


Eigen::Vector4d Quaternion::rotationVectorToQuaternion(const Eigen::Vector3d v_rot)
{
    Eigen::Vector4d quat;

    double norm_v=v_rot.norm();

    if(norm_v<1e-3)
    {
        quat[0]=1;
        quat.block<3,1>(1,0)=0.5*v_rot;
    }
    else
    {
        quat[0]=cos(norm_v/2);
        quat.block<3,1>(1,0)=v_rot/norm_v*sin(norm_v/2);
    }

    return quat;
}


Eigen::MatrixXd Quaternion::jacobianRotationVectorToQuaternion(const Eigen::Vector3d v_rot)
{
    Eigen::MatrixXd jacobian_matrix(4,3);
    jacobian_matrix.setZero();


    double norm_v=v_rot.norm();

    if(norm_v<1e-3)
    {
        jacobian_matrix(1,0)=0.5;
        jacobian_matrix(2,1)=0.5;
        jacobian_matrix(3,2)=0.5;
    }
    else
    {
        jacobian_matrix.block<1,3>(0,0)=-v_rot.transpose()/(2*norm_v)*sin(norm_v/2);
        jacobian_matrix.block<3,3>(1,0)=1/(pow(norm_v,3))*(Eigen::Matrix3d::Identity()*pow(norm_v,2)-v_rot*v_rot.transpose())*sin(norm_v/2)+0.5*v_rot*v_rot.transpose()/(pow(norm_v,2))*cos(norm_v/2);
    }


    return jacobian_matrix;
}


Eigen::Matrix3d Quaternion::skewSymMat(const Eigen::Vector3d w)
{
    Eigen::Matrix3d skewSymMat;
    skewSymMat<<0, -w[2], w[1],
                w[2], 0, -w[0],
                -w[1], w[0], 0;

    return skewSymMat;
}
