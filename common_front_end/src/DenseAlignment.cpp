
#include <common_front_end/DenseAlignment.h>

// derivative of transfer function (warping function) wrt to rotation
Eigen::Matrix2x3t dTransfer_dRba(
    const calibu::CameraModelGeneric<Scalar> &cam,
    const Sophus::SE3t                       &t_ba,
    const Eigen::Vector2t                    &pa
    )
{
  Eigen::Matrix2x3t dWdR;
  const Eigen::Vector3t ray = cam.Unproject( pa );
  const Eigen::Matrix2x4t dP = cam.dTransfer3D_dP( t_ba, ray, 0 );
  for (unsigned int ii=0; ii<3; ++ii) {
    dWdR.block<2,1>(0,ii) =
        dP.topLeftCorner<2,3>() * Sophus::SO3t::generator(ii) * ray;
  }
  return dWdR;

  //    Matrix2x6t dWdR;
  //    const Vector4t ray4;
  //    ray4 << ray[0], ray[1], ray[2], 1;
  //    const Matrix2x4t dP = rCam.dTransfer3D_dP( Tba, ray, 1.0/depth(pa) );
  //    for( unsigned int ii=0; ii<6; ++ii ){
  //        dWdR.template block<2,1>(0,ii) = dP.template topLeftCorner<2,3>() *
  //        Sophus::SE3Group<Scalar>::generator(ii)*ray4;
  //    }
  //    return dWdR;

}
