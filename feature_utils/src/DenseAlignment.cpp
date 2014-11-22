#include <feature_utils/DenseAlignment.h>

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
}
