// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#pragma once

#include <stdio.h>
#include <vector>

#include <opencv2/features2d/features2d.hpp>

#include <calibu/Calibu.h>

#include <non_free/Quartic.h>

#include <utils/MathTypes.h>


#define SQUARE(x) ((x)*(x))

///////////////////////////////////////////////////////////////////////////////
void HarrisScore(
    const unsigned char*         image,
    const unsigned int           image_width,
    const unsigned int           image_height,
    std::vector< cv::KeyPoint >& points,
    double                       k = 0.04);


/////////////////////////////////////////////////////////////////////////////
void BuildJetMap(std::vector<unsigned char>& color_map,
                 unsigned int length);


//////////////////////////////////////////////////////////////////////////////
inline int GNRansacModelFit(
    const Eigen::Vector2t z0,
    const Eigen::Vector2t z1,
    const Eigen::Vector2t z2,
    const Eigen::Vector4t x0,
    const Eigen::Vector4t x1,
    const Eigen::Vector4t x2,
    const calibu::CameraModelGeneric<Scalar>& camera,
    std::vector<Sophus::SE3t>& vPoses
    )
{
  return 0;
}

//////////////////////////////////////////////////////////////////////////////
inline int SolveThreePointPose(
    const Eigen::Vector2t z0,
    const Eigen::Vector2t z1,
    const Eigen::Vector2t z2,
    const Eigen::Vector3t x0,
    const Eigen::Vector3t x1,
    const Eigen::Vector3t x2,
    const calibu::CameraModelGeneric<Scalar>& camera,
    //        const Eigen::Matrix3d Kinv,
    std::vector<Sophus::SE3t>& vPoses
    )
{
  /*
    CHECK_MATRIX(z0);
    CHECK_MATRIX(z1);
    CHECK_MATRIX(z2);
    CHECK_MATRIX(x0);
    CHECK_MATRIX(x1);
    CHECK_MATRIX(x2);
    CHECK_MATRIX(Kinv);
    */

  vPoses.clear();

  double ab_sq = (x1 - x0).squaredNorm();
  double ac_sq = (x2 - x0).squaredNorm();
  double bc_sq = (x2 - x1).squaredNorm();
  //    Eigen::Vector3t za = Kinv*Eigen::Vector3t( z0[0], z0[1], 1.0 );
  //    Eigen::Vector3t zb = Kinv*Eigen::Vector3t( z1[0], z1[1], 1.0 );
  //    Eigen::Vector3t zc = Kinv*Eigen::Vector3t( z2[0], z2[1], 1.0 );
  Eigen::Vector3t za = camera.Unproject(z0);
  Eigen::Vector3t zb = camera.Unproject(z1);
  Eigen::Vector3t zc = camera.Unproject(z2);

  za.normalize();
  zb.normalize();
  zc.normalize();

  double cos_ab = za.dot( zb );
  double cos_ac = za.dot( zc );
  double cos_bc = zb.dot( zc );

  double K1 = bc_sq / ac_sq;
  double K2 = bc_sq / ab_sq;

  double p[5];
  {
    double K12 = K1*K2;
    double K12_1_2 = K12 - K1 - K2;
    double K2_1_K1_cab = K2*(1-K1)*cos_ab;

    p[4] = (SQUARE(K12_1_2)
            - 4*K12*cos_bc*cos_bc);
    p[3] = (4*(K12_1_2)*K2_1_K1_cab
            + 4*K1*cos_bc*((K12+K2-K1)*cos_ac
                           + 2*K2*cos_ab*cos_bc));
    p[2] = (SQUARE(2*K2_1_K1_cab) +
            2*(K12 + K1 - K2)*K12_1_2
            + 4*K1*((K1-K2)*cos_bc*cos_bc
                    + (1-K2)*K1*cos_ac*cos_ac
                    - 2*K2*(1 + K1) *cos_ab*cos_ac*cos_bc));
    p[1] = (4*(K12 + K1 - K2)*K2_1_K1_cab
            + 4*K1*((K12 - K1 + K2)*cos_ac*cos_bc
                    + 2*K12*cos_ab*cos_ac*cos_ac));
    p[0] = SQUARE(K12 + K1 - K2) - 4*K12*K1*cos_ac*cos_ac;
  }

  //    printf( "p: %f %f %f %f %f.\n", p[4], p[3], p[2], p[1], p[0] );

  double roots[4];
  double inv_p4 = 1.0 / p[4];
  for (int i=0; i<4; ++i) {
    p[i] *= inv_p4;
  }
  int nr = FindQuarticRealRoots( p[3], p[2], p[1], p[0], roots );

  //    printf( "%d  roots: %f %f %f %f.\n", nr, roots[3], roots[2], roots[1], roots[0] );

  int count = 0;
  for (int i=0; i<nr; ++i) {
    double x = roots[i];
    if (x <= 0) {
      continue;
    }
    for (int j=0; j<3; ++j) {
      x = NewtonQuartic( p[3], p[2], p[1], p[0], x );
    }
    double xx = x*x;

    double a_den = xx - 2*x*cos_ab + 1;
    double a = sqrt(ab_sq / a_den);
    double b = a*x;

    double M = 1 - K1;
    double P = 2*(K1*cos_ac - x*cos_bc);
    double Q = xx - K1;

    double P1 = -2*x*cos_bc;
    double Q1 = xx - K2*a_den;

    double den = M*Q1 - Q;
    if (den > -1e-12 && den < 1e-12) {
      //            assert(0);
      //            cerr << "skipped" << endl;
      continue;
    }

    double y = (P1*Q - P*Q1) / den;
    double c = a * y;

    Eigen::Vector3t y0 = a*za;
    Eigen::Vector3t y1 = b*zb;
    Eigen::Vector3t y2 = c*zc;

    Eigen::Matrix4t T;

    //        ThreePointAbsoluteOrientation( x0, x1, x2, xi0, xi1, xi2, T );
    {
      Eigen::Matrix3t D, D1;

      D.col(0) = x1 - x0;
      D.col(1) = x2 - x0;

      D.col(2) = D.col(1).cross( D.col(0) );

      D1.col(0) = y1 - y0;
      D1.col(1) = y2 - y0;

      D1.col(2) = D1.col(1).cross( D1.col(0) );

      Eigen::Matrix3t R = (D1*D.inverse()).transpose();
      T.block<3,3>(0,0) = R;
      T.block<3,1>(0,3) = x0 - R*y0;
      T.row(3) = Eigen::Vector4t( 0.0, 0.0, 0.0, 1.0 );
    }
    vPoses.push_back( Sophus::SE3t(T) );

    ++count;
  }
  return count;
}

/////////////////////////////////////////////////////////////////////////////
//  If p1 is on the plane, then points p on plane will obey
//      dot( p1-p, n) == 0.
//  We also know p is on our line:
//      p = slope*d+base
//  so
//      dot( p1-(slope*d+base), n) == 0
//      dot( p1-base-slope*d, n) == 0
//  will be true when the line hits the plane.
//  Solve for d: dot(p1-base,n) = d*dot(slope,n)
//      d = dot(p1-base,n) / dot(slope,n)
//  And return p = slope*d+base
inline Eigen::Vector3t LinePlaneIntersect(
    const Eigen::Vector3t& PlanePoint,
    const Eigen::Vector3t& PlaneNormal,
    const Eigen::Vector3t& LineSlope,
    const Eigen::Vector3t& LineBase // Should != PointPlane
    )
{
  assert( PlanePoint != LineBase );
  const double d =  (PlanePoint-LineBase).dot(PlaneNormal) / LineSlope.dot(PlaneNormal);
  return LineSlope * d + LineBase;
}

/////////////////////////////////////////////////////////////////////////////
///
/// \brief _IntersectRayWithFrustumSide
/// \param[in] PlaneNormal,normal of frustum side we're intrested in
/// \param[in] LineSlope ray slope
/// \param[in] LineBase point on the ray
/// \param[in] rSearchCam Camera defining the frustum
/// \param[out] p found 3d point
/// \return true if there is a valid intersection, including points at infinity
///
inline bool _IntersectRayWithFrustumSide(
      const Eigen::Vector3t& PlaneNormal,
      const Eigen::Vector3t& LineSlope,
      const Eigen::Vector3t& LineBase,
      const calibu::CameraModelGeneric<Scalar>& rSearchCam,
      Eigen::Vector3t& p)
{


  p = LinePlaneIntersect(Eigen::Vector3t::Zero(),
                         PlaneNormal, LineSlope, LineBase);
  if (p[2] < 0) {
    p = LineBase + LineSlope*1e6;
  }

  //    printf( "p: [%.4f,%.4f,%.4f]\n", p[0], p[1], p[2] );
  //    fflush(stdout);

  // ok, now is point between two lines (tl-c) and (bl-c)?
  // we can check this by testing where it projects to:
  const Eigen::Vector2t pix = rSearchCam.Project( p );

  //    printf( "pix: [%f, %2f]\n", pix[0], pix[1] ); fflush(stdout);

  const float ffx = rSearchCam.Width()*0.2;
  const float ffy = rSearchCam.Height()*0.2;
  if (pix[0] >= -ffx && pix[0] < (rSearchCam.Width()+ffx)
      && pix[1] >= -ffy && pix[1] < (rSearchCam.Height()+ffy)) {
    return true;
  }
  return false;
}

/////////////////////////////////////////////////////////////////////////////
inline bool IntersectRayWithFrustum(
    const Eigen::Vector3t& LineSlope,
    const Eigen::Vector3t& LineBase,
    const calibu::CameraModelGeneric<Scalar>& rSearchCam,
    double& rho1,
    double& rho2
    )
{
  // compute points that define the search camera frustum TODO pre-compute these normals
  const double w = rSearchCam.Width();
  const double h = rSearchCam.Height();
  // use pixel 1 to keep the frustum slightly smaller...
  Eigen::Vector3t tl = rSearchCam.Unproject(Eigen::Vector2t(double(  1),double(1  )));
  Eigen::Vector3t tr = rSearchCam.Unproject(Eigen::Vector2t(double(w-2),double(1  )));
  Eigen::Vector3t br = rSearchCam.Unproject(Eigen::Vector2t(double(w-2),double(h-2)));
  Eigen::Vector3t bl = rSearchCam.Unproject(Eigen::Vector2t(double(  1),double(h-2)));

  Eigen::Vector3t p, n;
  rho1 = DBL_MAX;
  rho2 = DBL_MAX;
  std::vector<double> vInverseDepths;
  n =  (tl.cross(bl)).normalized();// / (tl.cross(bl).norm());
  if (_IntersectRayWithFrustumSide( n, LineSlope, LineBase, rSearchCam, p)) {
    //            printf( "Ray=[%.2f,%.2f,%.2f]' b=[%.2f,%.2f,%.2f]' intersects left side of frustum n=[%.2f,%.2f,%.2f]' at [%.2f,%.2f,%.2f]\n",
    //                    LineSlope[0], LineSlope[1], LineSlope[2], LineBase[0], LineBase[1], LineBase[2], n[0], n[1], n[2], p[0], p[1], p[2] ); fflush(stdout);
    // If z < 0, then the line intesects plane behind the camera
    // which means that the point at infinity is visible in the image
    const double rho = p[2] < 0 ? 0 : 1.0 / ( p(2)-LineBase(2) );
    vInverseDepths.push_back(rho);
  }
  n =  tr.cross(tl).normalized();
  if (_IntersectRayWithFrustumSide( n, LineSlope, LineBase, rSearchCam, p )) {
    //            printf( "Ray=[%.2f,%.2f,%.2f]' b=[%.2f,%.2f,%.2f]' intersects top side of frustum n=[%.2f,%.2f,%.2f]' at [%.2f,%.2f,%.2f]\n",
    //                    LineSlope[0], LineSlope[1], LineSlope[2], LineBase[0], LineBase[1], LineBase[2], n[0], n[1], n[2], p[0], p[1], p[2] ); fflush(stdout);
    const double rho = p[2] < 0 ? 0 : 1.0 / ( p(2)-LineBase(2) );
    vInverseDepths.push_back(rho);
  }
  n = br.cross(tr).normalized();
  if (_IntersectRayWithFrustumSide( n, LineSlope, LineBase, rSearchCam, p )) {
    //            printf( "Ray=[%.2f,%.2f,%.2f]' b=[%.2f,%.2f,%.2f]' intersects right side of frustum n=[%.2f,%.2f,%.2f]' at [%.2f,%.2f,%.2f]\n",
    //                    LineSlope[0], LineSlope[1], LineSlope[2], LineBase[0], LineBase[1], LineBase[2], n[0], n[1], n[2], p[0], p[1], p[2] ); fflush(stdout);
    const double rho = p[2] < 0 ? 0 : 1.0 / ( p(2)-LineBase(2) );
    vInverseDepths.push_back(rho);
  }
  n = bl.cross(br).normalized();
  if (_IntersectRayWithFrustumSide( n, LineSlope, LineBase, rSearchCam, p )) {
    //            printf( "Ray=[%.2f,%.2f,%.2f]' b=[%.2f,%.2f,%.2f]' intersects bottom side of frustum n=[%.2f,%.2f,%.2f]' at [%.2f,%.2f,%.2f]\n",
    //                    LineSlope[0], LineSlope[1], LineSlope[2], LineBase[0], LineBase[1], LineBase[2], n[0], n[1], n[2], p[0], p[1], p[2] ); fflush(stdout);
    const double rho = p[2] < 0 ? 0 : 1.0 / ( p(2)-LineBase(2) );
    vInverseDepths.push_back(rho);
  }

  if (vInverseDepths.size() >= 2) {
    // sort inverse depths
    std::sort( vInverseDepths.begin(), vInverseDepths.end() );
    rho1 = vInverseDepths.front();
    rho2 = vInverseDepths.back();
    return !vInverseDepths.empty();
  } else {
    // we need at least two intersections to draw the epipolar line, so if there are less, we can disregard this match
    return false;
  }
}

/////////////////////////////////////////////////////////////////////////////
inline Eigen::Vector2tArray GetEpiPolarLine(
    const calibu::CameraModelGeneric<Scalar>& rRefCam,
    const calibu::CameraModelGeneric<Scalar>& rSearchCam,
    const Sophus::SE3t& Tsr,
    const Eigen::Vector2t& pt
    )
{
  Eigen::Vector2tArray vPts;
  // Orientation of ref in search cam.
  const Eigen::Vector3t ray = Tsr.so3()*rRefCam.Unproject(pt);
  // Base of ref in search cam.
  Eigen::Vector3t base = Tsr.matrix3x4().col(3);

  double rho1, rho2;
  if (IntersectRayWithFrustum( ray, base, rSearchCam, rho1, rho2 )) {
    double rhostep = (rho2-rho1)/10.0;
    for (double rho = rho1+1e-6; rho <= rho2+rhostep; rho+=rhostep) {
      const Eigen::Vector2t pf = rSearchCam.Project( ray*(1.0/rho)+base );
      if (pf[0] >= 0 && pf[0] < rSearchCam.Width() &&
          pf[1] >= 0 && pf[1] < rSearchCam.Height()) {
        vPts.push_back(pf);
      }
    }
  }
  return vPts;
}

/////////////////////////////////////////////////////////////////////////////
inline Eigen::Vector3t RayRayIntersect(
    const Eigen::Vector3t& rRefRay,
    const Eigen::Vector3t& rRefBase,
    const Eigen::Vector3t& rSearchRay
    )
{
  Eigen::Matrix3x2t A;
  A.col(0) = rSearchRay;
  A.col(1) = -rRefRay;
  Eigen::Vector2t res = (A.transpose()*A).ldlt().solve(A.transpose()*rRefBase);
  const Eigen::Vector3t p0 = rSearchRay*res[0]; // search ray base is zero
  const Eigen::Vector3t p1 = rRefBase + rRefRay*res[1];
  return (p0+p1)/2;
}

////////////////////////////////////////////////////////////////////////////
/// Draw one frustum in the view of the other
Eigen::Vector4t GetFrustrumBoundingBox(
    const calibu::CameraRigT<Scalar>& rig,
    const unsigned int uRefCamera,
    const unsigned int uDstCamera);

////////////////////////////////////////////////////////////////////////////
Scalar ComputeAngle(Eigen::Vector3t c1, Eigen::Vector3t c2, Eigen::Vector3t x);

// code to move into calibu at some point
namespace calibu
{
  /////////////////////////////////////////////////////////////////////////////
  inline int SmallestFrustum( const calibu::CameraRig& rRig )
  {
    int idx = 0;
    float smallest = FLT_MAX;
    for (size_t ii = 0; ii< rRig.cameras.size(); ii++) {
      const calibu::CameraModelGeneric<Scalar>& cam = rRig.cameras[ii].camera;
      Eigen::Matrix<Scalar,Eigen::Dynamic,1> params = cam.GenericParams();
      float hFOV = 180.0*2.0*atan2( cam.Width()/2, params[0] )/M_PI;
      float vFOV = 180.0*2.0*atan2( cam.Height()/2, params[1] )/M_PI;
      float fov = (hFOV+vFOV)/2.0;
      if (fov < smallest) {
        smallest = fov;
        idx = ii;
      }
    }
    return idx;
  }

  inline int LargestFrustum( const calibu::CameraRig& rRig )
  {
    int idx = 0;
    float largest = 0;
    for (size_t ii = 0; ii< rRig.cameras.size(); ii++) {
      const calibu::CameraModelGeneric<Scalar>& cam = rRig.cameras[ii].camera;
      Eigen::Matrix<Scalar,Eigen::Dynamic,1> params = cam.GenericParams();
      float hFOV = 180.0*2.0*atan2( cam.Width()/2, params[0] )/M_PI;
      float vFOV = 180.0*2.0*atan2( cam.Height()/2, params[1] )/M_PI;
      float fov = (hFOV+vFOV)/2.0;
      if (fov > largest) {
        largest = fov;
        idx = ii;
      }
    }
    return idx;
  }
} // calibu namespace
