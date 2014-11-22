
#include <feature_utils/Triangulation.h>
#include <utils/PrintMessage.h>


////////////////////////////////////////////////////////////////////////////////////////////
///  Triangulate a 3D point from multi-view measurements
bool Compute3dPnt(
    const Eigen::Vector2tArray &vPixels, ///< Input: vector of pixel measuements (2 for stereo)
    const unsigned int                 uRefCam, ///< Input: reference camera
    const calibu::CameraRigT<Scalar>   &rRig,    ///< Input: camera intrinsics and sensor poses
    Eigen::Vector4t                    &rX       ///< Output: homogeneous 3d landmark
    )
{
  // TODO -- it should be easy to setup an over-determined
  // system of equations to solve
  // and init from more than 2 cameras.

  // TODO -- this is a specialization for two cameras, fix this
  const unsigned int uSearchCam = uRefCam == 0 ? 1 : 0;

  // get the rays for each point
  const calibu::CameraModelAndTransformT<Scalar>& refCam =
      rRig.cameras[uRefCam];
  const calibu::CameraModelAndTransformT<Scalar>& searchCam =
      rRig.cameras[uSearchCam];

  const Sophus::SE3t T_ab = refCam.T_wc.inverse() * searchCam.T_wc;
  const Eigen::Vector3t ray0 = refCam.camera.Unproject(vPixels[uRefCam]);
  // bring the 2nd ray into the frame of the 1st camera
  const Eigen::Vector3t ray1 =
      T_ab.so3()*searchCam.camera.Unproject(vPixels[uSearchCam]);

  Eigen::Vector3t   rhs =  T_ab.translation();
  Eigen::Matrix3x2t A;
  A.col(0) = ray0;
  A.col(1) = -ray1;
  Eigen::Vector2t res = (A.transpose()*A).ldlt().solve(A.transpose()*rhs);
  const Eigen::Vector3t p0 = ray0*res[0];
  const Eigen::Vector3t p1 =  T_ab.translation() + ray1*res[1];
  rX.head(3) = ray0.normalized() * ((p0+p1)/2).norm();
  rX[3] = 1.0;

  return true;
}

///////////////////////////////////////////////////////////////////////////////
/// Given (possible) multi-view measurements in rFeature, see if we can
/// triangulate the 3d landmark position.
void TriangulateIfPossible(
    const calibu::CameraRigT<Scalar> &rRig,       //< Input:
    const unsigned int               uRefCamera, //< Input:
    Eigen::Vector2tArray     &vPixels,    //< Input:
    Eigen::Vector4t                  &Xr          //< Output: 3D point in robotic convention
    )
{
  // count good measuements, as flagged by a -1
  int nSeen = 0;
  for( size_t ii = 0; ii < rRig.cameras.size(); ii++ ){
    if( vPixels[ii][0] != -1 ){
      nSeen++;
    }
  }

  // not enough to triangulate, initialize at some "reasonable" depth instead
  if( nSeen < 2 ){
    // "reasonable" == 5 TODO compute this from the scene.... or figure
    //out if it matters to the BA engine.
    Xr.head<3>() =
        rRig.cameras[uRefCamera].camera.Unproject(
          vPixels[uRefCamera]).normalized() * 5.0;
    Xr[3] = 1;
    PrintMessage(1,"Initializing landmark from single view \n");
  }
  else{
    // Xr will be in computer vision coordinates (in reference camera frame)
    Compute3dPnt(vPixels, uRefCamera, rRig, Xr);
  }

  PrintMessage( 1, "Triangulating [%.2f, %.2f] [%.2f, %.2f]  == "
                "[%.2f, %.2f, %.2f]\n",
                vPixels[0][0], vPixels[0][1], vPixels[1][0],
      vPixels[1][1], Xr[0], Xr[1], Xr[2] ); fflush(stdout);

  // transform back into the vehicle frame, where the landmark is stored
  Xr = Sophus::MultHomogeneous( rRig.cameras[uRefCamera].T_wc ,Xr );
}

///////////////////////////////////////////////////////////////////////////////
/// Given (possible) multi-view measurements in rFeature, see if we can
/// triangulate the 3d landmark position.  This one works on a vector of features.
void TriangulateIfPossible(
    const calibu::CameraRigT<Scalar> &rRig,                //< Input:
    const unsigned int               uRefCamera,          //< Input:
    std::vector<Feature*>            &vMatchedFeatures,    //< Input:
    Eigen::Vector4t                  &Xr)                  //< Output: 3D point in robotic convention
{
  if( rRig.cameras.size() != vMatchedFeatures.size() ){
    assert(0);
    return;  // er, not possible.
  }
  // Prep a vector for triangulation; -1 is bad.
  Eigen::Vector2tArray vPixels( rRig.cameras.size() );
  for( size_t ii = 0; ii < rRig.cameras.size(); ii++ ){
    if( vMatchedFeatures[ii] ){
      vPixels[ii][0] = vMatchedFeatures[ii]->x;
      vPixels[ii][1] = vMatchedFeatures[ii]->y;
    }else{
      vPixels[ii][0] = -1;
      vPixels[ii][1] = -1;
    }
  }
  TriangulateIfPossible( rRig, uRefCamera, vPixels, Xr );
}
