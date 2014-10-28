#pragma once

#include <common/config.h>
#include <Eigen/Eigen>
#include <calibu/cam/CameraModel.h>
#include <utils/MathTypes.h>

#include <iomanip>
#include <ros/ros.h>
// #define IGNORE_PATCH_WARPING 1
template<unsigned int PatchSize>
class PatchHomography
{

public:
  enum HomographyState
  {
    eValid,
    eOutsideFOV
  };


  PatchHomography()
  {
    H = Eigen::Matrix3t::Identity();
    Hscale = 1;
    State = eValid;
  }

  PatchHomography( const PatchHomography& rhs )
  {
    if( this == &rhs ){
      return;
    }
    this->H = rhs.H;
    this->Hscale = rhs.Hscale;
    this->State = rhs.State;
  }

  /// construct a warping homography from a 3D patch that projects to tl,tr,br,bl
  PatchHomography(const Eigen::Vector2t& tl,
                  const Eigen::Vector2t& tr,
                  const Eigen::Vector2t& br,
                  const Eigen::Vector2t& bl)
  {
    // here dst = [tl,tr,br,bl], and src is the four corners of the canonical
    // patch

    // H = [ a b c ;
    //       d e f ;
    //       g h i ];
    //
    // generally, dst = Hsd*src (where src is [u;v;q], dst is [x;y;w], and Hsd
    // is the homography mapping from src image to dst image), hence assuming
    // w=q and i = 1,
    //
    //  x = (a*u' + b*v' + c)/(g*u' + h*v' + 1)
    //  y = (d*u' + e*v' + f)/(g*u' + h*v' + 1)
    //
    //  and then for each point we can get the following 2 equations in the 8
    //  unknowns a-h,
    //
    //  xk = a*uk + b*vk + c*1 + d*0  + e*0  + f*0 - g*uk*xk + h*vk*xk
    //  yk = a*0  + b*0  + c*0 + d*uk + e*vk + f*1 - g*uk*yk + h*vk*yk
    //
    // for each of the k points.  Thus with 4 points we get 8 equations and can
    // solve for the 8 unknowns.
    //
    // The best place I've seen this procedure explained is pages 17-21 of Paul
    // Heckbert's MS thesis, "Fundamentals of Texture Mapping and ImageWarping".

    // build A matrix
    Eigen::Matrix8t A;

    Eigen::Matrix2x4t src;
    Eigen::Matrix2x4t dst;

    dst.col(0) = tl;
    dst.col(1) = tr;
    dst.col(2) = br;
    dst.col(3) = bl;

    src.col(0) = Eigen::Vector2t(           0,           0 );
    src.col(1) = Eigen::Vector2t( PatchSize-1,           0 );
    src.col(2) = Eigen::Vector2t( PatchSize-1, PatchSize-1 );
    src.col(3) = Eigen::Vector2t(           0, PatchSize-1 );

    //    A(1:4,1:8) = [ u, v, ones(npts,1), zeros(npts,3), -u.*x, -v.*x ];
    A.block<4,1>(0,0) = src.row(0); // A(1:4,1) = u
    A.block<4,1>(0,1) = src.row(1); // A(1:4,2) = v
    A.block<4,1>(0,2) = Eigen::Vector4t::Ones();
    A.block<4,3>(0,3) = Eigen::Matrix4x3t::Zero();
    A.block<4,1>(0,6) = -src.row(0).cwiseProduct( dst.row(0) );  // -u.*x
    A.block<4,1>(0,7) = -src.row(1).cwiseProduct( dst.row(0) );  // -v.*x

    //    A(5:8,1:8) = [ zeros(npts,3), u, v, ones(npts,1), -u.*y, -v.*y ];
    A.block<4,3>(4,0) = Eigen::Matrix4x3t::Zero(); // A(5:8,1:3) = v
    A.block<4,1>(4,3) = src.row(0); // A(5:8,4) = u
    A.block<4,1>(4,4) = src.row(1); // A(5:8,5) = v
    A.block<4,1>(4,5) = Eigen::Vector4t::Ones(); // A(5:8,6) = v
    // A(5:8,7) =  -u.*y
    A.block<4,1>(4,6) = -src.row(0).cwiseProduct( dst.row(1) );
    // A(5:8,8) = -v.*y
    A.block<4,1>(4,7) = -src.row(1).cwiseProduct( dst.row(1) );

    // and RHS vector
    Eigen::Vector8t b;
    b.head<4>() = dst.row(0);
    b.tail<4>() = dst.row(1);

    // and compute solution
    Eigen::Vector8t res = A.inverse()*b;
    //                Eigen::Matrix<Scalar,8,1> res = A.ldlt().solve(b);

    H.block<1,3>(0,0) = res.head<3>();
    H.block<1,3>(1,0) = res.segment<3>(3);
    H.block<1,2>(2,0) = res.tail<2>();
    H(2,2) = 1;
    Hscale = PatchHomography<CANONICAL_PATCH_SIZE>::GetQuadrilateralScale(
          tl,tr,bl,br);
    State = eValid;
  }


  PatchHomography(
      const Scalar& x,
      const Scalar& y,
      const Scalar& scale
      )
  {

    // we subtract 1 from patch size, as it is specified in whole pixels,
    // whereas we are interested in inter-center pixel distance
    //(which is patchsize - 2*half pixels)
    H << scale,     0, x - scale*(PatchSize-1)/2.0,
        0, scale, y - scale*(PatchSize-1)/2.0,
        0,     0,                           1;
    Hscale = scale;
    State = eValid;
  }


  //        PatchHomography( const Eigen::Matrix<Scalar,3,3>& rhs )
  //        {
  //            H = rhs.template topLeftCorner<2,3>();
  //            Hscale = H.template topLeftCorner<2,2>().determinant();
  //        }


  /// \brief PatchHomography: construct a scaled and rotated patch
  ///  homography based on the input top-left, top-right, and center
  ///  pixels.
  /// \param tl top left pixel
  /// \param tr top right pixel
  /// \param c center pixel
  PatchHomography(
      const Eigen::Vector2t& tl,
      const Eigen::Vector2t& tr,
      const Eigen::Vector2t& c
      )
  {
    const float hypotenuse = (tr-tl).norm(); // hypotenuse in new image
    // since patch since is in pixels and hypotenuse is measured between the
    // centers of pixels, we must add 1 (or two half pixels) to the hypotenuse
    // in order for scale to work
    const float s = hypotenuse / (PatchSize-1);
    const float rise = tr[1]-tl[1];
    const float run = tr[0]-tl[0];
    const float cth = run/hypotenuse;
    const float sth = rise/hypotenuse;
    //            const float cth = 1;
    //            const float sth = 0;
    const float tx = c[0]-hypotenuse/2.0;
    const float ty = c[1]-hypotenuse/2.0;
    this->H(0,0) = s*cth;
    this->H(0,1) = -s*sth;
    this->H(0,2) = tx;
    this->H(1,0) = s*sth;
    this->H(1,1) = s*cth;
    this->H(1,2) = ty;
    this->H(2,0) = 0;
    this->H(2,1) = 0;
    this->H(2,2) = 1;
    this->Hscale = 1.0;
    State = eValid;
  }


  Scalar scale() const
  {
    return Hscale;
  }

  void SetScale(Scalar s)
  {
    Hscale = s;
  }


  HomographyState GetState() const { return State; }
  void SetState(const HomographyState state) { State = state; }


  void GetSamplingHomographyAndLevel(
      const Scalar       fLevelFactor,       ///< Input:
      const unsigned int uMaxSamplingLevel,  ///< Input:
      PatchHomography&   LevelHomography,    ///< Output:
      unsigned int&      uSamplingLevel      ///< Output:
      ) const
  {

    // use scale change to decide which octave to sample from
    if( Hscale < 1.0 ){
      ROS_WARN("PatchHomography::GetSamplingHomographyAndLevel. hscale = %f < 1.0", Hscale);
      uSamplingLevel = 0;
    }else{
      uSamplingLevel = round(log(1.0/Hscale)/log(fLevelFactor));
    }

    if( uSamplingLevel >= uMaxSamplingLevel ){
      // printf("WARNING: Scale of feature [%d] not supported, looking in:
      //%d\n",nSamplingLevel,nMaxSamplingLevel-1); fflush(stdout);
      uSamplingLevel = uMaxSamplingLevel-1;
    }
    // grow/shrink H to sample in appropriate octave
    Scalar sd = 1.0;
    for( size_t ii = 0; ii < uSamplingLevel; ii++ ){
      sd *= fLevelFactor;
    }
    // translation component is left,top of patch
    // TODO isn't it really the projective center?
    LevelHomography.H = this->H;
    LevelHomography.H.template topLeftCorner<2,2>() *= sd;
    LevelHomography.H(0,2) = (LevelHomography.H(0,2)+0.5)*sd - 0.5;
    LevelHomography.H(1,2) = (LevelHomography.H(1,2)+0.5)*sd - 0.5;
    LevelHomography.Hscale = Hscale * sd;

    // std::cout << "Input scale is " << Hscale  <<  " output scale is " <<
    // LevelHomography.Hscale << " with sampling level " << nSamplingLevel <<
    // std::endl;
  }


  void SetTranslation( const Eigen::Vector2t& t )
  {
    SetTranslation( t[0], t[1] );
  }


  void SetTranslation( const Scalar& x, const Scalar& y )
  {
    H(0,2) = x - Hscale*(PatchSize-1)/2.0;
    H(1,2) = y - Hscale*(PatchSize-1)/2.0;
  }


  Eigen::Vector2t TopLeftPixel() const
  {
    const Eigen::Vector3t pix =
        H * Eigen::Vector3t( 0.0, 0.0, 1.0 );
    return pix.template head<2>()/pix[2];
  }


  Eigen::Vector2t TopRightPixel() const
  {
    const Eigen::Vector3t pix =
        H * Eigen::Vector3t( PatchSize-1, 0.0, 1.0 );
    return pix.template head<2>()/pix[2];
  }


  Eigen::Vector2t BottomRightPixel() const
  {
    const Eigen::Vector3t pix =
        H * Eigen::Vector3t( PatchSize-1, PatchSize-1, 1.0 );
    return pix.template head<2>()/pix[2];
  }


  Eigen::Vector2t BottomLeftPixel() const
  {
    const Eigen::Vector3t pix =
        H * Eigen::Vector3t( 0.0, PatchSize-1, 1.0 );
    return pix.template head<2>()/pix[2];
  }


  Eigen::Vector2t CenterPixel() const
  {
    const Eigen::Vector3t pix =
        H * Eigen::Vector3t( (PatchSize-1)/2.0, (PatchSize-1)/2.0, 1.0 );
    return pix.template head<2>()/pix[2];
  }


  Eigen::Matrix3t matrix() const
  {
    return H;
  }

  void SetMatrix(const Eigen::Matrix3t& h)
  {
    H = h;
  }

  /////////
  Eigen::Vector2t operator*(const Eigen::Vector3t& rhs) const
  {
    Eigen::Vector3t lhs;
    lhs = H*rhs;
    return lhs.template head<2>() / lhs[2];
  }

  /////////
  static inline Scalar GetQuadrilateralScale(const Eigen::Vector2t& tl,
                                             const Eigen::Vector2t& tr,
                                             const Eigen::Vector2t& bl,
                                             const Eigen::Vector2t& br)
  {
    const Eigen::Vector2t ac = (tl-br);
    const Eigen::Vector2t bd = (tr-bl);
    return std::sqrt(0.5 * std::abs(ac[0] * bd[1] - ac[1] * bd[0])) /
        (CANONICAL_PATCH_SIZE - 1);
  }

  /////////
  static inline PatchHomography<PatchSize>
  Transfer3DPatchHomography(const Eigen::Vector3t& tl3d_w,
                            const Eigen::Vector3t& tr3d_w,
                            const Eigen::Vector3t& bl3d_w,
                            const Eigen::Vector3t& br3d_w,
                            const Eigen::Vector3t& /* c3d_w */,
                            const calibu::CameraModelGeneric<Scalar>& cam,
                            const Sophus::SE3Group<Scalar>&   Tsw)
  {
    bool bInValidTransfer = false;

    // transform to sensor frame to project:
    const Eigen::Vector3t tl3d_sp = Tsw * tl3d_w;
    const Eigen::Vector3t tr3d_sp = Tsw * tr3d_w;
    const Eigen::Vector3t br3d_sp = Tsw * br3d_w;
    const Eigen::Vector3t bl3d_sp = Tsw * bl3d_w;

    // calculate the dot product of all points and make sure nothing
    // is near the edge of the screen
    const Eigen::Vector3t axis(0,0,1.0);
    Scalar angle = std::max(
        {std::abs(std::acos(tr3d_sp.normalized().dot(axis))),
         std::abs(std::acos(br3d_sp.normalized().dot(axis))),
         std::abs(std::acos(bl3d_sp.normalized().dot(axis))),
         std::abs(std::acos(tl3d_sp.normalized().dot(axis)))});

    // This makes sure that features stay within a certain angle of the camera,
    // to avoid hitting the 180 degree contour and rolling over.
    if (angle > 85.0/180.0 * M_PI) {
      bInValidTransfer = true;
    }

    // and project:
    const Eigen::Vector2t tl = cam.Project(tl3d_sp); // top left
    const Eigen::Vector2t tr = cam.Project(tr3d_sp); // top right
    const Eigen::Vector2t br = cam.Project(br3d_sp); // top right
    const Eigen::Vector2t bl = cam.Project(bl3d_sp); // top right

    PatchHomography<CANONICAL_PATCH_SIZE> H;

    //#if 1
#ifndef IGNORE_PATCH_WARPING

    //            std::cout << "tl: " << tl.transpose() << std::endl;
    //            std::cout << "tr: " << tr.transpose() << std::endl;
    //            std::cout << "br: " << br.transpose() << std::endl;
    //            std::cout << "bl: " << bl.transpose() << std::endl;
    H = PatchHomography<CANONICAL_PATCH_SIZE>(tl,tr,br,bl);
#else
    const Eigen::Vector3t c3d_sp = Tsw * bl3d_w;
    const Eigen::Vector2t c = cam.Project( c3d_sp ); // center
    H =  PatchHomography<PatchSize>( c[0], c[1],
        PatchHomography<PatchSize>::GetQuadrilateralScale(tl,tr,bl,br) );
#endif
    H.SetState( bInValidTransfer ?
                  PatchHomography<CANONICAL_PATCH_SIZE>::eOutsideFOV :
                  PatchHomography<CANONICAL_PATCH_SIZE>::eValid );
    return H;

  }


  //////////
  /// construct a warping homography from the nxn patch in the ref image
  /// to a transformed sampling patch in the search image -- this is what allows
  /// matching between massivley warped images.
  PatchHomography<PatchSize> TransferPatchHomography(
      const calibu::CameraModelGeneric<Scalar>&  rSearchCam, //< Input: search camera model we transfer into
      const calibu::CameraModelGeneric<Scalar>&  rRefCam, //< Input: search camera model we transfer into
      const Sophus::SE3Group<Scalar>&            Tsr,        //< Input: ref cam to search cam transform
      const Scalar                               rho         //< Input: inverse depth used for transfer
      ) const
  {
    // figure out homography using only the top left, top right,
    // and center pixels.

    const Eigen::Vector2t tl_ref = this->TopLeftPixel();
    const Eigen::Vector2t tr_ref = this->TopRightPixel();
    const Eigen::Vector2t bl_ref = this->BottomLeftPixel();
    const Eigen::Vector2t br_ref = this->BottomRightPixel();
    const Eigen::Vector2t tl_search = rSearchCam.Transfer(
          rRefCam, Tsr, tl_ref, rho );
    const Eigen::Vector2t tr_search = rSearchCam.Transfer(
          rRefCam, Tsr, tr_ref, rho );
    const Eigen::Vector2t br_search = rSearchCam.Transfer(
          rRefCam, Tsr, br_ref, rho );
    const Eigen::Vector2t bl_search = rSearchCam.Transfer(
          rRefCam, Tsr, bl_ref, rho );

    // std::cout << "Tranferring from tl=" << tl_ref.transpose() <<
    // " ,tr=" << tr_ref.transpose() << " to tls=" << tl_search.transpose() <<
    // " trs=" << tr_search.transpose() << std::endl;
#ifndef IGNORE_PATCH_WARPING
    return PatchHomography<PatchSize>(tl_search, tr_search, br_search,
                                      bl_search);
#else
    const Eigen::Vector2t c_ref = this->CenterPixel();
    const Eigen::Vector2t c_search = rSearchCam.Transfer(
          rRefCam, Tsr, c_ref, rho );
    return PatchHomography<CANONICAL_PATCH_SIZE>(c_search[0], c_search[1],
        PatchHomography<CANONICAL_PATCH_SIZE>::GetQuadrilateralScale(
        tl_search,tr_search,bl_search,br_search));
#endif


  }

  //////////
  /// construct a warping homography from the nxn patch in the ref image
  /// to a transformed sampling patch in the search image -- this is what allows
  /// matching between massivley warped images.
  PatchHomography<PatchSize> TransferPatchSimilarityHomography(
      const Eigen::Vector2t&                    rSearchPix, //< Input: center of new homography
      const calibu::CameraModelGeneric<Scalar>& rSearchCam, //< Input: search camera model we transfer into
      const calibu::CameraModelGeneric<Scalar>& rRefCam,    //< Input: reference camera model
      const Sophus::SE3Group<Scalar>&           Tsr,        //< Input: ref cam to search cam transform
      const Scalar                              rho         //< Input: inverse depth used for transfer
      ) const
  {
    // figure out homography using only the top left, top right, and center pixels.
    const Eigen::Vector2t tl_ref = this->TopLeftPixel();
    const Eigen::Vector2t tr_ref = this->TopRightPixel();
    const Eigen::Vector2t tl_search = rSearchCam.Transfer(
          rRefCam, Tsr, tl_ref, rho);
    const Eigen::Vector2t tr_search = rSearchCam.Transfer(
          rRefCam, Tsr, tr_ref, rho);
    // std::cout << "Tranferring from cs=" << rSearchPix << ", tl=" <<
    // tl_ref.transpose() << " ,tr=" << tr_ref.transpose() << " to tls=" <<
    // tl_search.transpose() << " trs=" << tr_search.transpose() << std::endl;
    return PatchHomography<PatchSize>(
          tl_search, tr_search, rSearchPix);
  }

private:
  HomographyState  State;
  Scalar           Hscale;
  Eigen::Matrix3t  H;
};
