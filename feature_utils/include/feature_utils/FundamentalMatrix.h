// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#pragma once

#include <iostream>
#include <random>
#include <vector>
#include <miniglog/logging.h>
#include <utils/MathTypes.h>


///////////////////////////////////////////////////////////////////////////////
/// Computes the Sampson distance for two corresponding points under the
/// Fundamental matrix F
inline
Scalar SampsonDistance( const Eigen::Matrix3t& F,
                        const Eigen::Vector3t& x0,
                        const Eigen::Vector3t& x1)
{
  Eigen::Vector3t Fx0 = F*x0;
  Eigen::Vector3t Ftx1 = F.transpose()*x1;

  Scalar n = x1.transpose() * F * x0;
  Scalar d = Fx0(0)*Fx0(0)+Fx0(1)*Fx0(1) + Ftx1(0)*Ftx1(0)+Ftx1(1)*Ftx1(1);
  return (n*n)/d;
}

///////////////////////////////////////////////////////////////////////////////
inline
bool NormalizedEightPointAlgorithm(
    const std::vector<Eigen::Vector2t>& x1,
    const std::vector<Eigen::Vector2t>& x2,
    Eigen::Matrix3t& F
    )
{
  const unsigned int num_points = x1.size();

  if (num_points < 8) {
    std::cerr << "8-Point Alg: Not enough input correspondences" << std::endl;
    return false;
  }

  if (num_points != x2.size()) {
    std::cerr << "8-Point Alg: Correspondences don't match" << std::endl;
    return false;
  }

  // Normalize data such that mean is 0 and std is sqrt(2)
  Eigen::Vector2t mean1, mean2;
  mean1.setZero();
  mean2.setZero();
  for (unsigned int ii=0; ii < num_points; ++ii) {
    mean1 += x1[ii];
    mean2 += x2[ii];
  }
  mean1 /= x1.size();
  mean2 /= x2.size();

  std::vector<Eigen::Vector2t> nx1 = x1;
  std::vector<Eigen::Vector2t> nx2 = x2;

  Scalar norm1 = 0;
  Scalar norm2 = 0;
  for (unsigned int ii=0; ii < num_points; ++ii) {
    nx1[ii] -= mean1;
    nx2[ii] -= mean2;
    norm1 += nx1[ii].norm();
    norm2 += nx2[ii].norm();
  }

  Scalar scale1 = std::sqrt(2)*num_points/norm1;
  Scalar scale2 = std::sqrt(2)*num_points/norm2;

  for (unsigned int ii=0; ii < num_points; ++ii) {
    nx1[ii] *= scale1;
    nx2[ii] *= scale2;
  }

  // Generate transformations.
  Eigen::Matrix3t T1;
  Eigen::Matrix3t T2;
  T1.setZero();
  T2.setZero();
  T1.diagonal() << scale1, scale1, 1;
  T2.diagonal() << scale2, scale2, 1;
  T1.block<2,1>(0,2) = -scale1*mean1;
  T2.block<2,1>(0,2) = -scale2*mean2;

  // Compute fundamental matrix.
  Eigen::MatrixXt A(num_points,9);
  for (unsigned int ii=0; ii < num_points; ++ii) {
    A.block<1,9>(ii,0) << nx2[ii](0)*nx1[ii](0),
                          nx2[ii](0)*nx1[ii](1),
                          nx2[ii](0),
                          nx2[ii](1)*nx1[ii](0),
                          nx2[ii](1)*nx1[ii](1),
                          nx2[ii](1),
                          nx1[ii](0),
                          nx1[ii](1),
                          1;
  }

  Eigen::JacobiSVD<Eigen::MatrixXt> svd_A(A,
                                        Eigen::ComputeFullU |
                                        Eigen::ComputeFullV);

  Eigen::Matrix<Scalar,9,9> V = svd_A.matrixV();
  Eigen::Matrix<Scalar,9,1> p = V.block<9,1>(0,8);
  p /= p(8);

  F << p(0),p(1),p(2),
       p(3),p(4),p(5),
       p(6),p(7),p(8);

  // Enforce rank 2.
  Eigen::JacobiSVD<Eigen::Matrix3t> svd_F(F,
                                        Eigen::ComputeFullU |
                                        Eigen::ComputeFullV);
  Eigen::Matrix3t V_F = svd_F.matrixV();
  Eigen::Vector3t S_F = svd_F.singularValues();
  Eigen::Matrix3t U_F = svd_F.matrixU();
  Eigen::Matrix3t nS;
  nS.setZero();
  nS.diagonal() << S_F(0), S_F(1), 0;

  F = U_F*nS*V_F.transpose();

  // Denormalization.
  F = T2.transpose()*F*T1;
  F /= F(2,2);

  return true;
}

///////////////////////////////////////////////////////////////////////////////
inline
void GetRandIndices(unsigned int min_idx,
                    unsigned int max_idx,
                    unsigned int num_idx,
                    std::mt19937* rng,
                    unsigned int* indices) {
  CHECK_NOTNULL(rng);
  // There must be more numbers available than indices required
  CHECK_GT(max_idx - min_idx, num_idx)
      << "Max, min, #: " << max_idx << ", " << min_idx << ", " << num_idx;

  unsigned int count = 0;
  std::uniform_int_distribution<unsigned int> n_dist(min_idx, max_idx);
  do {
    unsigned int n = n_dist(*rng);
    if (count == 0) {
      indices[count++] = n;
    } else {
      bool is_new = true;
      for (unsigned int ii = 0; ii < count; ++ii) {
        if ( n == indices[ii] ){
          is_new = false;
          break;
        }
      }
      if (is_new) {
        indices[count++] = n;
      }
    }

  } while(count < num_idx);
}

///////////////////////////////////////////////////////////////////////////////
inline
unsigned int ComputeInliers(const std::vector<Eigen::Vector2t>& src_pts,
                            const std::vector<Eigen::Vector2t>& dst_pts,
                            const Eigen::Matrix3t& F,
                            const Scalar eps,
                            std::vector<unsigned int>& inliers )
{
  Eigen::Vector3t x0;
  Eigen::Vector3t x1;
  for (unsigned int ii=0; ii<src_pts.size(); ++ii){
    x0 << src_pts[ii], 1;
    x1 << dst_pts[ii], 1;
    Scalar dist = SampsonDistance(F, x0, x1);
    if( dist < eps ) {
      inliers.push_back(ii);
    }
  }
  return inliers.size();
}

///////////////////////////////////////////////////////////////////////////////
inline
void ComputeFundamentalMatrix( const std::vector<Eigen::Vector2t>& src_pts,
                               const std::vector<Eigen::Vector2t>& dst_pts,
                               std::mt19937* rng,
                               Eigen::Matrix3t& F,
                               std::vector<unsigned int>& inliers) {
  CHECK_NOTNULL(rng);

  // Ransac
  unsigned int max_trials = 100;
  unsigned int trial_count = 0;
  unsigned int min_set_size = 8;
  unsigned int num_points = src_pts.size();
  unsigned int best_num_inliers = 0;
  unsigned int rand_idx[8];
  Scalar inlier_threshold = 1.0;
  Scalar eps = 1.0e-10;
  Scalar prob = 0.99;

  while( trial_count < max_trials ) {

    // Select random minimum set
    GetRandIndices(0, num_points, min_set_size, rng, rand_idx);

    std::vector<Eigen::Vector2t> x1;
    std::vector<Eigen::Vector2t> x2;

    for (unsigned int ii=0; ii<min_set_size; ++ii) {
      x1.push_back(src_pts[rand_idx[ii]]);
      x2.push_back(dst_pts[rand_idx[ii]]);
    }

    // Compute Fundamental matrix
    Eigen::Matrix3t min_set_F;
    NormalizedEightPointAlgorithm(x1, x2, min_set_F);

    // Compute num inliers.
    std::vector<unsigned int> min_set_inliers;
    unsigned int num_inliers = ComputeInliers(src_pts,
                                              dst_pts,
                                              min_set_F,
                                              inlier_threshold,
                                              min_set_inliers);

    // Adjust max number of trials
    if( num_inliers > best_num_inliers ) {
      inliers = min_set_inliers;
      best_num_inliers = num_inliers;

      Scalar frac_inliers = ((Scalar)best_num_inliers)/num_points;
      Scalar prob_no_outliers = 1 - (frac_inliers*frac_inliers*frac_inliers);
      if (prob_no_outliers < eps) {
        prob_no_outliers = eps; // Avoid division by -Inf
      }
      if (prob_no_outliers > (1-eps)) {
        prob_no_outliers = (1-eps); // Avoid division by 0
      }
      max_trials = log( 1-prob ) / log( prob_no_outliers );
    }

    trial_count++;
  }

  // Recompute F using all inliers
  std::vector<Eigen::Vector2t> x1;
  std::vector<Eigen::Vector2t> x2;
  for (unsigned int ii=0; ii<inliers.size(); ++ii) {
    x1.push_back(src_pts[inliers[ii]]);
    x2.push_back(dst_pts[inliers[ii]]);
  }
  NormalizedEightPointAlgorithm(x1, x2, F);

}
