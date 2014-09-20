// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#pragma once

#include <random>
#include <utils/MathTypes.h>
#include <common_front_end/FundamentalMatrix.h>

///////////////////////////////////////////////////////////////////////////////
Eigen::Vector4t Get3DPoint(
                   Eigen::Matrix<Scalar,3,4>& P0,
                   Eigen::Matrix<Scalar,3,4>& P1,
                   Eigen::Vector2t&           x0,
                   Eigen::Vector2t&           x1
                  )
{
     Eigen::Matrix<Scalar,6,4> A; A.setZero();
     A.block<1,4>(0,0) = x0(0) * P0.block<1,4>(2,0) - P0.block<1,4>(0,0);
     A.block<1,4>(1,0) = x0(1) * P0.block<1,4>(2,0) - P0.block<1,4>(1,0);
     A.block<1,4>(2,0) = x0(0) * P0.block<1,4>(1,0) - x0(1)*P0.block<1,4>(0,0);
     A.block<1,4>(3,0) = x1(0) * P1.block<1,4>(2,0) - P1.block<1,4>(0,0);
     A.block<1,4>(4,0) = x1(1) * P1.block<1,4>(2,0) - P1.block<1,4>(1,0);
     A.block<1,4>(5,0) = x1(0) * P1.block<1,4>(1,0) - x1(1)*P1.block<1,4>(0,0);

     Eigen::JacobiSVD< Eigen::Matrix<Scalar,6,4> > svd( A,
                                                        Eigen::ComputeFullU |
                                                        Eigen::ComputeFullV);
     Eigen::Matrix4t V = svd.matrixV();
     Eigen::Vector4t X = V.block<4,1>(0,3);

     Eigen::Matrix<Scalar,2,4> depthTest;
     depthTest.block<1,4>(0,0) = P0.block<1,4>(2,0);
     depthTest.block<1,4>(1,0) = P1.block<1,4>(2,0);

     Eigen::Vector2t s = depthTest * X;

     if (s(0) < 0 || s(1) < 0) {
         X = -X;
         if( s(0) > 0 || s(1) > 0){
             std::cout << "warning: Inconsistent orientation of point match"
                       << std::endl;
         }
     }

     X(0) = X(0)/X(3);
     X(1) = X(1)/X(3);
     X(2) = X(2)/X(3);
     X(3) = 1.0;

     return X;
}


///////////////////////////////////////////////////////////////////////////////
void ComputeCameraPoseFromEssentialMatrix(
    const std::vector<Eigen::Vector2t> &src_pts,
    const std::vector<Eigen::Vector2t> &dst_pts,
    const Eigen::Matrix3t              &K,
    std::mt19937                       *rng,  // Determinism in the RNG
    Sophus::SE3t                       &Tab )
{

  // Compute Essential Matrix
  Eigen::Matrix3t F;
  std::vector<unsigned int> inliers;
  ComputeFundamentalMatrix(src_pts, dst_pts, rng, F, inliers);
  Eigen::Matrix3t E = K.transpose() * F * K;

  // Compute principal components using svd.
  Eigen::JacobiSVD<Eigen::Matrix3t> svd( E,
                                         Eigen::ComputeFullU |
                                         Eigen::ComputeFullV);
  Eigen::Matrix3t U = -svd.matrixU();
  Eigen::Matrix3t V = -svd.matrixV();

  // There are four possible solutions, find the correct one.
  // (where a triangulated point is in front of both cameras )
  Eigen::Matrix4t T[4];
  Eigen::Matrix3t W; W << 0, -1, 0, 1, 0, 0, 0, 0, 1;
  Eigen::Matrix3t UWVt  = U * W * V.transpose();
  Eigen::Matrix3t UWtVt = U * W.transpose() * V.transpose();
  Eigen::Vector3t t = U.block<3,1>(0,2);

  // Make sure we have a right-hand system.
  if( UWVt.determinant() < 0 ){
    UWVt(0,0) = -UWVt(0,0);
    UWVt(1,1) = -UWVt(1,1);
    UWVt(2,2) = -UWVt(2,2);
    UWVt(0,2) = -UWVt(0,2);
    UWVt(2,0) = -UWVt(2,0);
    UWtVt(0,0) = -UWtVt(0,0);
    UWtVt(1,1) = -UWtVt(1,1);
    UWtVt(2,2) = -UWtVt(2,2);
    UWtVt(0,2) = -UWtVt(0,2);
    UWtVt(2,0) = -UWtVt(2,0);
    t(2) = -t(2);
  }

  T[0] = Eigen::Matrix4t::Identity();
  T[0].block<3,1>(0,3) = t;
  T[0].block<3,3>(0,0) = UWVt;
  T[1] = Eigen::Matrix4t::Identity();
  T[1].block<3,1>(0,3) = -t;
  T[1].block<3,3>(0,0) = UWVt;
  T[2] = Eigen::Matrix4t::Identity();
  T[2].block<3,1>(0,3) = t;
  T[2].block<3,3>(0,0) = UWtVt;
  T[3] = Eigen::Matrix4t::Identity();
  T[3].block<3,1>(0,3) = -t;
  T[3].block<3,3>(0,0) = UWtVt;

  // Init camera matrices.
  Eigen::Matrix<Scalar,3,4> P0 = Eigen::Matrix<Scalar,3,4>::Zero();
  Eigen::Matrix<Scalar,3,4> P1 = Eigen::Matrix<Scalar,3,4>::Zero();

  P0.block<3,3>(0,0) = K;

  // Find a correspondence that satisfies the epipolar constrain.
  int validCamera = -1;

  Eigen::Vector2t x0 = src_pts[inliers[0]];
  Eigen::Vector2t x1 = dst_pts[inliers[0]];

  // Now triangulate using all cameras.
  for (unsigned int jj=0; jj < 4; ++jj) {
    P1 = K * T[jj].block<3,4>(0,0);
    Eigen::Vector4t X0 = Get3DPoint(P0, P1, x0, x1);
    Eigen::Vector4t X1 = T[jj]*X0;
    // Check if point is in front of both cameras.
    if( X0(2) > 0 && X1(2) > 0 ){
       validCamera = jj;
       break;
    }
  }

  if(validCamera < 0){
    std::cerr << "FATAL ERROR: invalid set of cameras!" << std::endl;
    return;
  }

  P1 = K * T[validCamera].block<3,4>(0,0);

  // TODO: compute essential matrix using RANSAC and 5 point method described in:
  // H. Stewenius, C. Engels, and D. Nister.
  // Recent developments on direct relative orientation.
  // ISPRS Journal of Photogrammetry and Remote Sensing, 60:284-294, June 2006.

  // Init permutation matrix
  Eigen::Matrix4t pose = Eigen::Matrix4t::Identity();
  Eigen::Matrix3t M;
  M <<  0.0,0.0,1.0,1.0,0.0,0.0,0.0,1.0,0.0 ;
  Eigen::Matrix4t posecv = T[validCamera].inverse();
  pose.block<3,3>(0,0) = M * posecv.block<3,3>(0,0) * M.transpose();
  pose.block<3,1>(0,3) = M * posecv.block<3,1>(0,3);

  Tab = Sophus::SE3t(pose);

}
