
#include "msf_localization_core/quaternion_algebra.h"



namespace Quaternion
{

Jacobians::Jacobians()
{
    std::vector<Eigen::Triplet<double> > triplets;

    // Jacobian error quaternion wrt error theta
    mat_diff_error_quat_wrt_error_theta_dense<<0, 0, 0,
                                            0.5, 0, 0,
                                            0, 0.5, 0,
                                            0, 0, 0.5;

    mat_diff_error_quat_wrt_error_theta_sparse.resize(4,3);
    triplets.clear();
    triplets.push_back(Eigen::Triplet<double>(1, 0, 0.5));
    triplets.push_back(Eigen::Triplet<double>(2, 1, 0.5));
    triplets.push_back(Eigen::Triplet<double>(3, 2, 0.5));
    mat_diff_error_quat_wrt_error_theta_sparse.setFromTriplets(triplets.begin(), triplets.end());



    // Jacobian error theta wrt error quaternion
    mat_diff_error_theta_wrt_error_quat_dense<<0, 2, 0, 0,
                                                0, 0, 2, 0,
                                                0, 0, 0, 2;

    mat_diff_error_theta_wrt_error_quat_sparse.resize(3, 4);
    triplets.clear();
    triplets.push_back(Eigen::Triplet<double>(0, 1, 2));
    triplets.push_back(Eigen::Triplet<double>(1, 2, 2));
    triplets.push_back(Eigen::Triplet<double>(2, 3, 2));
    mat_diff_error_theta_wrt_error_quat_sparse.setFromTriplets(triplets.begin(), triplets.end());



    mat_diff_quat_inv_wrt_quat_dense<<1, 0, 0, 0,
                                        0, -1, 0, 0,
                                        0, 0, -1, 0,
                                        0, 0, 0, -1;

    mat_diff_quat_inv_wrt_quat_sparse.resize(4, 4);
    triplets.clear();
    triplets.push_back(Eigen::Triplet<double>(0, 0, 1));
    triplets.push_back(Eigen::Triplet<double>(1, 1, -1));
    triplets.push_back(Eigen::Triplet<double>(2, 2, -1));
    triplets.push_back(Eigen::Triplet<double>(3, 3, -1));
    mat_diff_quat_inv_wrt_quat_sparse.setFromTriplets(triplets.begin(), triplets.end());



    mat_diff_vector_wrt_vector_amp_dense<<0, 1, 0, 0,
                                            0, 0, 1, 0,
                                            0, 0, 0, 1;

    mat_diff_vector_wrt_vector_amp_sparse.resize(3, 4);
    triplets.clear();
    triplets.push_back(Eigen::Triplet<double>(0, 1, 1));
    triplets.push_back(Eigen::Triplet<double>(1, 2, 1));
    triplets.push_back(Eigen::Triplet<double>(2, 3, 1));
    mat_diff_vector_wrt_vector_amp_sparse.setFromTriplets(triplets.begin(), triplets.end());


    mat_diff_vector_amp_wrt_vector_dense<<0, 0, 0,
                                            1, 0, 0,
                                            0, 1, 0,
                                            0, 0, 1;

    mat_diff_vector_amp_wrt_vector_sparse.resize(4, 3);
    triplets.clear();
    triplets.push_back(Eigen::Triplet<double>(1, 0, 1));
    triplets.push_back(Eigen::Triplet<double>(2, 1, 1));
    triplets.push_back(Eigen::Triplet<double>(3, 2, 1));
    mat_diff_vector_amp_wrt_vector_sparse.setFromTriplets(triplets.begin(), triplets.end());


    // End
    return;
}


Jacobians jacobians;



Quaternion conj(const Quaternion &q)
{
    Eigen::Vector4d qr;

    qr[0]=q[0];
    qr.block<3,1>(1,0)=-q.block<3,1>(1,0);

    return qr;
}


Eigen::Vector4d inv(const Eigen::Vector4d& q)
{
    Eigen::Vector4d qr;


    qr=conj(q)/q.norm();


    return qr;
}



Eigen::Vector4d cross(const Eigen::Vector4d& q1, const Eigen::Vector4d& q2)
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

Eigen::Vector4d cross_gen_pure(const Eigen::Vector4d &q1, const Eigen::Vector3d &q2)
{
    Eigen::Vector4d qres;

    // Quat 1
    double pr=q1[0];
    Eigen::Vector3d pv=q1.block<3,1>(1,0);

    // Quat 2
    // qr=0
    // qv=q2


    // Result
    qres[0]=-pv.transpose()*q2;
    qres.block<3,1>(1,0)=pr*q2+pv.cross(q2);


    return qres;
}

Eigen::Vector4d cross_pure_gen(const Eigen::Vector3d& q1, const Eigen::Vector4d& q2)
{
    Eigen::Vector4d qres;


    // Quat 1
    // pr=0
    // pv=q1

    // Quat 2
    double qr=q2[0];
    Eigen::Vector3d qv=q2.block<3,1>(1,0);

    // Result
    qres[0]=-q1.transpose()*qv;
    qres.block<3,1>(1,0)=qr*q1+q1.cross(qv);


    return qres;
}

Eigen::Vector4d cross_pure_pure(const Eigen::Vector3d& q1, const Eigen::Vector3d& q2)
{
    Eigen::Vector4d qres;


    // Quat 1
    //pr=0;
    //pv=q1;

    // Quat 2
    //qr=0;
    //qv=q2;

    // Result
    qres[0]=-q1.transpose()*q2;
    qres.block<3,1>(1,0)=q1.cross(q2);


    return qres;

}


Eigen::Vector3d cross_sandwich(const Eigen::Vector4d &q1, const Eigen::Vector3d &q2, const Eigen::Vector4d &q3)
{
    Eigen::Vector4d qres;

    qres = cross(q1, cross_pure_gen(q2, q3));

    return qres.block<3,1>(1,0);
}


// Q+
Eigen::Matrix4d quatMatPlus(const Eigen::Vector4d &q)
{
    Eigen::Matrix4d QuatMat=q[0]*Eigen::Matrix4d::Identity(4,4);

    QuatMat.block<1,3>(0,1)+=-(q.block<3,1>(1,0)).transpose();
    QuatMat.block<3,3>(1,1)+=skewSymMat(q.block<3,1>(1,0));
    QuatMat.block<3,1>(1,0)+=(q.block<3,1>(1,0));

    return QuatMat;
}

Eigen::Matrix4d quatMatPlus(const Eigen::Vector3d& q)
{
//    Eigen::Vector4d quat;
//    quat[0]=0;
//    quat.block<3,1>(1,0)=q;

//    return quatMatPlus(quat);

    Eigen::Matrix4d QuatMat;
    QuatMat.setZero();

    QuatMat.block<1,3>(0,1)=-q.transpose();
    QuatMat.block<3,3>(1,1)=skewSymMat(q);
    QuatMat.block<3,1>(1,0)=q;

    return QuatMat;
}


// Q-
Eigen::Matrix4d quatMatMinus(const Eigen::Vector4d& q)
{
    Eigen::Matrix4d QuatMat=q[0]*Eigen::Matrix4d::Identity(4,4);

    QuatMat.block<1,3>(0,1)+=-(q.block<3,1>(1,0)).transpose();
    QuatMat.block<3,3>(1,1)+=-skewSymMat(q.block<3,1>(1,0));
    QuatMat.block<3,1>(1,0)+=(q.block<3,1>(1,0));

    return QuatMat;
}

Eigen::Matrix4d quatMatMinus(const Eigen::Vector3d& q)
{
//    Eigen::Vector4d quat;
//    quat[0]=0;
//    quat.block<3,1>(1,0)=q;

//    return quatMatMinus(quat);

    Eigen::Matrix4d QuatMat;
    QuatMat.setZero();

    QuatMat.block<1,3>(0,1)=-q.transpose();
    QuatMat.block<3,3>(1,1)=-skewSymMat(q);
    QuatMat.block<3,1>(1,0)=q;

    return QuatMat;
}


Eigen::Vector4d rotationVectorToQuaternion(const Eigen::Vector3d& v_rot)
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


Eigen::Matrix<double, 4, 3> jacobianRotationVectorToQuaternion(const Eigen::Vector3d& v_rot)
{
    Eigen::Matrix<double, 4, 3> jacobian_matrix;//(4,3);
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


Eigen::Matrix3d skewSymMat(const Eigen::Vector3d& w)
{
    Eigen::Matrix3d skewSymMat;
    skewSymMat<<0, -w[2], w[1],
                w[2], 0, -w[0],
                -w[1], w[0], 0;

    return skewSymMat;
}



}
