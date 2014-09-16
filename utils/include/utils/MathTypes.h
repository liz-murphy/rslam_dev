// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#ifndef _MATH_TYPES_H_
#define _MATH_TYPES_H_

#include <vector>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <sophus/se3.hpp>
#include <common/scalar.h>
#include <utils/AndroidHelpers.h>

namespace Eigen {

template<typename EigenT>
using aligned_vector = std::vector<EigenT, Eigen::aligned_allocator<EigenT> >;

#define USING_VECTOR_ARRAY(size)                                        \
  using Vector##size##tArray = aligned_vector<Matrix<Scalar, size, 1> >

USING_VECTOR_ARRAY(2);
USING_VECTOR_ARRAY(3);
USING_VECTOR_ARRAY(4);
USING_VECTOR_ARRAY(5);
USING_VECTOR_ARRAY(6);

#undef USING_VECTOR_ARRAY

typedef Matrix<Scalar,2,1> Vector2t;
typedef Matrix<Scalar,3,1> Vector3t;
typedef Matrix<Scalar,4,1> Vector4t;
typedef Matrix<Scalar,5,1> Vector5t;
typedef Matrix<Scalar,6,1> Vector6t;
typedef Matrix<Scalar,7,1> Vector7t;
typedef Matrix<Scalar,8,1> Vector8t;
typedef Matrix<Scalar,Eigen::Dynamic,1> VectorXt;
typedef Matrix<Scalar,2,3> Matrix2x3t;
typedef Matrix<Scalar,2,4> Matrix2x4t;
typedef Matrix<Scalar,2,6> Matrix2x6t;
typedef Matrix<Scalar,3,2> Matrix3x2t;
typedef Matrix<Scalar,3,4> Matrix3x4t;
typedef Matrix<Scalar,4,3> Matrix4x3t;
typedef Matrix<Scalar,2,2> Matrix2t;
typedef Matrix<Scalar,3,3> Matrix3t;
typedef Matrix<Scalar,4,4> Matrix4t;
typedef Matrix<Scalar,6,6> Matrix6t;
typedef Matrix<Scalar,7,7> Matrix7t;
typedef Matrix<Scalar,8,8> Matrix8t;
typedef Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic> MatrixXt;
}

///////////////////////////////////////////////////////////////////////////////
namespace Sophus
{

typedef Sophus::SO3Group<Scalar> SO3t;
typedef Sophus::SE3Group<Scalar> SE3t;

//template<typename Scalar=double>
inline Eigen::Vector4t MultHomogeneous(
    const Sophus::SE3t& lhs,
    const Eigen::Vector4t& rhs )
{
  Eigen::Vector4t out;
  out.head<3>() =
      lhs.so3() * (Eigen::Vector3t)rhs.head<3>() + lhs.translation()*rhs[3];
  out[3] = rhs[3];
  return out;
}
}

///////////////////////////////////////////////////////////////////////////////
inline double powi(const double x, const int y)
{
    if(y == 0){
        return 1.0;
    }else if(y < 0 ){
        return 1.0/powi(x,-y);
    }else{
        double ret = x;
        for(int ii = 1; ii <  y ; ii++){
            ret *= x;
        }
        return ret;
    }
}

///////////////////////////////////////////////////////////////////////////////
/// \brief ConvertSE3covarianceToEulerXYZ
///        Computes the 6x6 covariance: covEuler = J^T * covSE3 * J
///        where J is the jacobian of the function that transforms from
///        SE3 to EulerXYZ
/// \param pose      [ q0 q1 q2 w | x y z ]
/// \param covSE3    7x7 covariance matrix
/// \param covEuler  6x6 covariance matrix
///
inline void ConvertSE3covarianceToEulerXYZ(
    const Sophus::SE3t    &pose,
    const Eigen::Matrix7t &covSE3,
    Eigen::Matrix6t       &covEulerXYZ)
{
    // compute J
    const double q0 = pose.data()[0];
    const double q1 = pose.data()[1];
    const double q2 = pose.data()[2];
    const double w  = pose.data()[3];

    const double q0sq = q0*q0;
    const double q1sq = q1*q1;
    const double q2sq = q2*q2;

    const double factor1 = (2*q0sq + 2*q1sq - 1);
    const double factor2 = (powi(2*q1*q2 + 2*q0*w,2)/powi(factor1,2) + 1);

    Eigen::Matrix<Scalar,6,7> J;
    J.setZero();
    J(0,0) = -((2*w)/factor1 - (4*q0*(2*q1*q2 + 2*q0*w))/powi(factor1,2))/factor2;
    J(0,1) = -((2*q2)/factor1 - (4*q1*(2*q1*q2 + 2*q0*w))/powi(factor1,2))/factor2;
    J(0,2) = -(2*q1)/(factor2*factor1);
    J(0,3) = -(2*q0)/(factor2*factor1);

    const double factor4 = sqrt(1 - powi(2*q0*q2 - 2*q1*w,2));
    J(1,0) = -(2*q2)/factor4;
    J(1,1) = (2*w)/factor4;
    J(1,2) = -(2*q0)/factor4;
    J(1,3) = (2*q1)/factor4;

    const double factor5 = (2*q1sq + 2*q2sq - 1);
    const double factor6 = (2*q0*q1 + 2*q2*w);
    J(2,0) = -(2*q1)/((powi(factor6,2)/powi(factor5,2) + 1)*factor5);
    J(2,1) = -((2*q0)/factor5 - (4*q1*factor6)/powi(factor5,2))/(powi(factor6,2)/powi(factor5,2) + 1);
    J(2,2) = -((2*w)/factor5 - (4*q2*factor6)/powi(factor5,2))/(powi(factor6,2)/powi(factor5,2) + 1);
    J(2,3) = -((2*w)/factor5 - (4*q2*factor6)/powi(factor5,2))/(powi(factor6,2)/powi(factor5,2) + 1);

    J.block<3,3>(3,4) = Eigen::Matrix3t::Identity();

    covEulerXYZ = J*covSE3*J.transpose();
}



#endif
