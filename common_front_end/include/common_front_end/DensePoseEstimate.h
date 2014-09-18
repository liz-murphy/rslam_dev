#pragma once

#include <opencv2/opencv.hpp>

#ifdef USE_TBB
//#undef USE_TBB
#endif

#ifdef USE_TBB
#include <tbb/tbb.h>
#endif

#include <sophus/se3.hpp>
#include <calibu/cam/CameraRig.h>

#include <Utils/ESM.h>
#include <Utils/PatchUtils.h>
#include <Utils/PoseHelpers.h>
#include <Utils/MathTypes.h>
#include <Utils/PrintMessage.h>
#include <HAL/Utils/TicToc.h>


#ifdef USE_TBB

/*
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
class Gradients {
public:

  /////////////////////////////////////////////////////////////////////////////
  Gradients(
      unsigned char*  image,
      size_t          image_width,
      size_t          image_height,
      float*          grad_x,
      float*          grad_y
      )
    : image_(image),
      image_width_(image_width),
      image_height_(image_height),
      grad_x_(grad_x),
      grad_y_(grad_y)
  {}

  /////////////////////////////////////////////////////////////////////////////
  void operator()( const tbb::blocked_range<size_t>& r ) const
  {
    float* grad_x = grad_x_;
    float* grad_y = grad_y_;
    for(size_t ii=r.begin(); ii!=r.end(); ++ii) {

//      float* row = image_ + ii;
//      float* bottom_row = row + image_width_;

      // first pixel
      if(ii == 0) {
//        grad_x_[0] = image_[1] - image_[0];
//        grad_y_[0] = bottom_row[0] - image_[0];
//        continue;
      }

      // first row
      if(ii < image_width_) {

      }

    }
  }

///
/////////////////////////////////////////////////////////////////////////////
private:
  unsigned char*  image_;
  size_t          image_width_;
  size_t          image_height_;
  float*          grad_x_;
  float*          grad_y_;
};
*/


/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
class PoseRefine {
public:

  /////////////////////////////////////////////////////////////////////////////
  PoseRefine(
      const cv::Mat&            LiveGrey,
      const cv::Mat&            RefGrey,
      const cv::Mat&            RefDepth,
      const Eigen::Matrix3t&    Klg,
      const Eigen::Matrix3t&    Krg,
      const Eigen::Matrix3t&    Krd,
      const Eigen::Matrix4t&    Tgd,
      const Eigen::Matrix4t&    Tlr,
      const Eigen::Matrix3x4t&  KlgTlr,
      float                     NormParam,
      bool                      DiscardSaturated,
      float                     MinDepth,
      float                     MaxDepth
      )
    : error(0),
      num_obs(0),
      live_grey_(LiveGrey),
      ref_grey_(RefGrey),
      ref_depth_(RefDepth),
      Klg_(Klg),
      Krg_(Krg),
      Krd_(Krd),
      Tgd_(Tgd),
      Tlr_(Tlr),
      KlgTlr_(KlgTlr),
      norm_param_(NormParam),
      discard_saturated_(DiscardSaturated),
      min_depth_(MinDepth),
      max_depth_(MaxDepth)
  {
    LHS.setZero();
    RHS.setZero();
  }

  /////////////////////////////////////////////////////////////////////////////
  PoseRefine( PoseRefine& x, tbb::split )
    : error(0),
      num_obs(0),
      live_grey_(x.live_grey_),
      ref_grey_(x.ref_grey_),
      ref_depth_(x.ref_depth_),
      Klg_(x.Klg_),
      Krg_(x.Krg_),
      Krd_(x.Krd_),
      Tgd_(x.Tgd_),
      Tlr_(x.Tlr_),
      KlgTlr_(x.KlgTlr_),
      norm_param_(x.norm_param_),
      discard_saturated_(x.discard_saturated_),
      min_depth_(x.min_depth_),
      max_depth_(x.max_depth_)
  {
    LHS.setZero();
    RHS.setZero();
  }

  /////////////////////////////////////////////////////////////////////////////
  void operator()( const tbb::blocked_range<size_t>& r )
  {
    // local pointer for optimization apparently
    for(size_t ii=r.begin(); ii!=r.end(); ++ii) {

      const unsigned int u = ii%ref_depth_.cols;
      const unsigned int v = ii/ref_depth_.cols;

      // 2d point in reference depth camera
      Eigen::Vector2t pr_d;
      pr_d << u, v;

      // get depth
      const float fDepth = ref_depth_.at<float>(v, u);

      if( fDepth <= min_depth_ || fDepth >= max_depth_ ) {
        continue;
      }

      // 3d point in reference depth camera
      Eigen::Vector4t hPr_d;
      hPr_d(0) = fDepth * (pr_d(0) - Krd_(0,2)) / Krd_(0,0);
      hPr_d(1) = fDepth * (pr_d(1) - Krd_(1,2)) / Krd_(1,1);
      hPr_d(2) = fDepth;
      hPr_d(3) = 1;

      // 3d point in reference grey camera (homogenized)
      const Eigen::Vector4t hPr_g = Tgd_ * hPr_d;

      // project to reference grey camera's image coordinate
      Eigen::Vector2t pr_g;
      pr_g(0) = (hPr_g(0) * Krg_(0,0) / hPr_g(2)) + Krg_(0,2);
      pr_g(1) = (hPr_g(1) * Krg_(1,1) / hPr_g(2)) + Krg_(1,2);

      // check if point is out of bounds
      if(pr_g(0) < 2 || pr_g(0) >= ref_grey_.cols-3
         || pr_g(1) < 2 || pr_g(1) >= ref_grey_.rows-3) {
        continue;
      }

      // homogenized 3d point in live grey camera
      const Eigen::Vector4t hPl_g = Tlr_ * hPr_g;

      // project to live grey camera's image coordinate
      Eigen::Vector2t pl_g;
      pl_g(0) = (hPl_g(0) * Klg_(0,0) / hPl_g(2)) + Klg_(0,2);
      pl_g(1) = (hPl_g(1) * Klg_(1,1) / hPl_g(2)) + Klg_(1,2);

      // check if point is out of bounds
      if(pl_g(0) < 2 || pl_g(0) >= live_grey_.cols-3
         || pl_g(1) < 2 || pl_g(1) >= live_grey_.rows-3) {
        continue;
      }

      // get intensities
      const float Il = interp(pl_g(0), pl_g(1), live_grey_.data,
                              live_grey_.cols, live_grey_.rows);
      const float Ir = interp(pr_g(0), pr_g(1), ref_grey_.data,
                              ref_grey_.cols, ref_grey_.rows);

      // discard under/over-saturated pixels
      if( discard_saturated_ ) {
        if( Il == 0 || Il == 255 || Ir == 0 || Ir == 255 ) {
          continue;
        }
      }

      // calculate error
      const double y = Il - Ir;

      // image derivative
      const float Il_xr = interp( pl_g(0)+1, pl_g(1), live_grey_.data, live_grey_.cols, live_grey_.rows );
      const float Il_xl = interp( pl_g(0)-1, pl_g(1), live_grey_.data, live_grey_.cols, live_grey_.rows );
      const float Il_yu = interp( pl_g(0), pl_g(1)-1, live_grey_.data, live_grey_.cols, live_grey_.rows );
      const float Il_yd = interp( pl_g(0), pl_g(1)+1, live_grey_.data, live_grey_.cols, live_grey_.rows );
//      Scalar dx = interpf(pl_g(0), pl_g(1), (float*)GradX.data,
//                          LiveGreyImg.cols, LiveGreyImg.rows);
//      Scalar dy = interpf(pl_g(0), pl_g(1), (float* )GradY.data,
//                          LiveGreyImg.cols, LiveGreyImg.rows);

      Eigen::Matrix<Scalar,1,2> dIl;
      dIl << (Il_xr - Il_xl)/2.0, (Il_yd - Il_yu)/2.0;
//      dIl << dx, dy;


      // projection & dehomogenization derivative
      Eigen::Vector3t KlPl = Klg_ * hPl_g.head(3);

      Eigen::Matrix2x3t dPl;
      dPl  << 1.0/KlPl(2), 0, -KlPl(0)/(KlPl(2)*KlPl(2)),
          0, 1.0/KlPl(2), -KlPl(1)/(KlPl(2)*KlPl(2));



      const Eigen::Vector4t dIl_dPl_KlgTlr = dIl * dPl * KlgTlr_;

      // J = dIl_dPl_KlgTlr * gen_i * Pr
      Eigen::Vector6t J;
      J << dIl_dPl_KlgTlr(0),
          dIl_dPl_KlgTlr(1),
          dIl_dPl_KlgTlr(2),
          -dIl_dPl_KlgTlr(1)*hPr_g(2) + dIl_dPl_KlgTlr(2)*hPr_g(1),
          +dIl_dPl_KlgTlr(0)*hPr_g(2) - dIl_dPl_KlgTlr(2)*hPr_g(0),
          -dIl_dPl_KlgTlr(0)*hPr_g(1) + dIl_dPl_KlgTlr(1)*hPr_g(0);

      // add robust norm here
//      const Scalar w = 1.0;
      const Scalar w = _NormTukey(y, norm_param_);
//      const Scalar w = _NormL1(y, NormC);

      LHS += J * J.transpose() * w;
      RHS += J * y * w;
      error += y * y;
      num_obs++;
    }
  }

  /////////////////////////////////////////////////////////////////////////////
  void join( const PoseRefine& other )
  {
    LHS += other.LHS;
    RHS += other.RHS;
    error += other.error;
    num_obs += other.num_obs;
  }

private:
  /////////////////////////////////////////////////////////////////////////////
  inline Scalar _NormTukey( Scalar r, Scalar c )
  {
    const Scalar absr = fabs(r);
    const Scalar roc = r / c;
    const Scalar omroc2 = 1.0f - roc*roc;
    return (absr <= c ) ? omroc2*omroc2 : 0.0f;
  }

  /////////////////////////////////////////////////////////////////////////////
  inline Scalar _NormL1( Scalar r, Scalar /*c*/ )
  {
    const Scalar absr = fabs(r);
    return (absr == 0 ) ? 1.0f : 1.0f / absr;
  }

///
/////////////////////////////////////////////////////////////////////////////
public:
  Eigen::Matrix6t     LHS;
  Eigen::Vector6t     RHS;
  double              error;
  unsigned int        num_obs;

private:
  cv::Mat             live_grey_;
  cv::Mat             ref_grey_;
  cv::Mat             ref_depth_;
  Eigen::Matrix3t     Klg_;
  Eigen::Matrix3t     Krg_;
  Eigen::Matrix3t     Krd_;
  Eigen::Matrix4t     Tgd_;
  Eigen::Matrix4t     Tlr_;
  Eigen::Matrix3x4t   KlgTlr_;
  float               norm_param_;
  bool                discard_saturated_;
  float               min_depth_;
  float               max_depth_;
};
#endif



/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
class DensePoseEstimate
{

public:
  /////////////////////////////////////////////////////////////////////////////
  DensePoseEstimate()
    : PYRAMID_LEVELS(5)
#ifdef USE_TBB
    , tbb_scheduler_(tbb::task_scheduler_init::deferred)
#endif
  {}

  /////////////////////////////////////////////////////////////////////////////
  ~DensePoseEstimate()
  {
#ifdef USE_TBB
    tbb_scheduler_.terminate();
#endif
  }

  /////////////////////////////////////////////////////////////////////////////
  void Init()
  {
#ifdef USE_TBB
    tbb_scheduler_.initialize();
#endif
  }

  /////////////////////////////////////////////////////////////////////////////
  void SetParams(
      const calibu::CameraModelGeneric<Scalar>&   LiveGreyCM,
      const calibu::CameraModelGeneric<Scalar>&   RefGreyCM,
      const calibu::CameraModelGeneric<Scalar>&   RefDepthCM,
      const Sophus::SE3t&                         Tgd
      )
  {
    // store scaled camera models (to avoid recomputing)
    for( size_t ii = 0; ii < PYRAMID_LEVELS; ++ii ) {
      live_grey_cam_model_.push_back(_ScaleCM(LiveGreyCM, ii));
      ref_grey_cam_model_.push_back(_ScaleCM(RefGreyCM, ii));
      ref_depth_cam_model_.push_back(_ScaleCM(RefDepthCM, ii));
    }

    // copy reference camera's depth-grey transform
    Tgd_ = Tgd;
    std::cout << "Tgd: " << Tgd.log().transpose() << std::endl;
  }


  /////////////////////////////////////////////////////////////////////////////
  void SetKeyframe(
      const cv::Mat&                              RefGrey,
      const cv::Mat&                              RefDepth
      )
  {
    // build pyramids
    cv::buildPyramid(RefGrey, ref_grey_pyramid_, PYRAMID_LEVELS);
    cv::buildPyramid(RefDepth, ref_depth_pyramid_, PYRAMID_LEVELS);
  }

  /////////////////////////////////////////////////////////////////////////////
  Scalar Estimate(
      const cv::Mat&              LiveGrey,   //< Input: live image
      Sophus::SE3Group<Scalar>&   Trl         //< Input/Output: transform between grey cameras (input is hint)
      )
  {
    // options
    const Scalar    NormC = 8;
    const bool      DiscardSaturated = true;
    const float     MinDepth = 0.01;
    const float     MaxDepth = 100.0;

    // set pyramid max-iterations and full estimate mask
    std::vector<bool>           vFullEstimate = {1, 1, 1, 1, 0};
    std::vector<unsigned int>   vMaxIteration = {1, 2, 3, 4, 5};

    // aux variables
    Eigen::Matrix6t LHS;
    Eigen::Vector6t RHS;
    Scalar          SqError;
    Scalar          NumObs;
    Scalar          LastError = FLT_MAX;

    // build live pyramid
    cv::buildPyramid(LiveGrey, live_grey_pyramid_, PYRAMID_LEVELS);

    // iterate through pyramid levels
    for(int nPyrLvl = PYRAMID_LEVELS-1; nPyrLvl >= 0; nPyrLvl--) {

      const cv::Mat& LiveGreyImg = live_grey_pyramid_[nPyrLvl];
      const cv::Mat& RefGreyImg = ref_grey_pyramid_[nPyrLvl];
      const cv::Mat& RefDepthImg = ref_depth_pyramid_[nPyrLvl];

      // TODO(jmf) Pass live camera model (in case they are different)
      //            const calibu::CameraModelGeneric<Scalar>& LiveGreyCM = m_LiveGreyCM[nPyrLvl];
      const calibu::CameraModelGeneric<Scalar>& LiveGreyCM
                                                  = ref_grey_cam_model_[nPyrLvl];
      const calibu::CameraModelGeneric<Scalar>& RefGreyCM
                                                  = ref_grey_cam_model_[nPyrLvl];
      const calibu::CameraModelGeneric<Scalar>& RefDepthCM
                                                  = ref_depth_cam_model_[nPyrLvl];

#ifndef USE_TBB
      // pre-calculate gradients so we don't do it each iteration
      cv::Mat GradX(LiveGreyImg.rows, LiveGreyImg.cols, CV_32FC1);
      cv::Mat GradY(LiveGreyImg.rows, LiveGreyImg.cols, CV_32FC1);
      Gradient(LiveGreyImg.data, LiveGreyImg.cols, LiveGreyImg.rows,
               LiveGreyImg.cols, (float*)GradX.data, (float*)GradY.data);
#endif

      // reset error
      LastError = FLT_MAX;

      // set pyramid norm parameter
      const Scalar    NormCPyr = NormC * (nPyrLvl + 1);

      for(unsigned int nIters = 0; nIters < vMaxIteration[nPyrLvl]; ++nIters) {

        // reset
        LHS.setZero();
        RHS.setZero();

        // reset error
        NumObs = 0;
        SqError = 0;

        // inverse transform
        const Sophus::SE3t Tlr = Trl.inverse();

        const Eigen::Matrix3t Klg = LiveGreyCM.K();
        const Eigen::Matrix3t Krg = RefGreyCM.K();
        const Eigen::Matrix3t Krd = RefDepthCM.K();

        const Eigen::Matrix3x4t KlgTlr = Klg * Tlr.matrix3x4();

#ifdef USE_TBB
        // launch TBB
        PoseRefine pose_ref(LiveGreyImg, RefGreyImg, RefDepthImg,
                   Klg, Krg, Krd, Tgd_.matrix(), Tlr.matrix(), KlgTlr,
                   NormCPyr, DiscardSaturated, MinDepth, MaxDepth);

        tbb::parallel_reduce(tbb::blocked_range<size_t>(0, RefDepthImg.cols*RefDepthImg.rows,10000), pose_ref);

        LHS = pose_ref.LHS;
        RHS = pose_ref.RHS;
        SqError = pose_ref.error;
        NumObs = pose_ref.num_obs;
#else
        // iterate through depth map
        for(int row = 0; row < RefDepthImg.rows; ++row) {
          for(int col = 0; col < RefDepthImg.cols; ++col) {

            // 2d point in reference depth camera
            Eigen::Vector2t pr_d;
            pr_d << col, row;

            // get depth
            const float fDepth = RefDepthImg.at<float>(row, col);

            if( fDepth <= MinDepth || fDepth >= MaxDepth ) {
              continue;
            }

            // 3d point in reference depth camera
//            const Eigen::Vector3t Pr_d = RefDepthCM.Unproject(pr_d);
            Eigen::Vector4t hPr_d;
//            hPr_d << Pr_d * fDepth, 1;
            hPr_d(0) = fDepth * (pr_d(0) - Krd(0,2)) / Krd(0,0);
            hPr_d(1) = fDepth * (pr_d(1) - Krd(1,2)) / Krd(1,1);
            hPr_d(2) = fDepth;
            hPr_d(3) = 1;

            // 3d point in reference grey camera (homogenized)
            const Eigen::Vector4t hPr_g = Tgd_.matrix() * hPr_d;

            // project to reference grey camera's image coordinate
//            const Eigen::Vector2t pr_g = RefGreyCM.Project( hPr_g.head(3) );
            Eigen::Vector2t pr_g;
            pr_g(0) = (hPr_g(0) * Krg(0,0) / hPr_g(2)) + Krg(0,2);
            pr_g(1) = (hPr_g(1) * Krg(1,1) / hPr_g(2)) + Krg(1,2);

            // check if point is out of bounds
            if(pr_g(0) < 2 || pr_g(0) >= RefGreyImg.cols-3
               || pr_g(1) < 2 || pr_g(1) >= RefGreyImg.rows-3) {
              continue;
            }

            // homogenized 3d point in live grey camera
            const Eigen::Vector4t hPl_g = Tlr.matrix() * hPr_g;

            // project to live grey camera's image coordinate
//            const Eigen::Vector2t pl_g = LiveGreyCM.Project( hPl_g.head(3) );
            Eigen::Vector2t pl_g;
            pl_g(0) = (hPl_g(0) * Klg(0,0) / hPl_g(2)) + Klg(0,2);
            pl_g(1) = (hPl_g(1) * Klg(1,1) / hPl_g(2)) + Klg(1,2);

            // check if point is out of bounds
            if(pl_g(0) < 2 || pl_g(0) >= LiveGreyImg.cols-3
               || pl_g(1) < 2 || pl_g(1) >= LiveGreyImg.rows-3) {
              continue;
            }

            // get intensities
            const float Il = interp(pl_g(0), pl_g(1), LiveGreyImg.data,
                                    LiveGreyImg.cols, LiveGreyImg.rows);
            const float Ir = interp(pr_g(0), pr_g(1), RefGreyImg.data,
                                    RefGreyImg.cols, RefGreyImg.rows);

            // discard under/over-saturated pixels
            if( DiscardSaturated ) {
              if( Il == 0 || Il == 255 || Ir == 0 || Ir == 255 ) {
                continue;
              }
            }

            // calculate error
            const Scalar y = Il - Ir;


            // image derivative
            const float Il_xr = interp(pl_g(0)+1, pl_g(1), LiveGreyImg.data, LiveGreyImg.cols, LiveGreyImg.rows);
            const float Il_xl = interp(pl_g(0)-1, pl_g(1), LiveGreyImg.data, LiveGreyImg.cols, LiveGreyImg.rows);
            const float Il_yu = interp(pl_g(0), pl_g(1)-1, LiveGreyImg.data, LiveGreyImg.cols, LiveGreyImg.rows);
            const float Il_yd = interp(pl_g(0), pl_g(1)+1, LiveGreyImg.data, LiveGreyImg.cols, LiveGreyImg.rows);
//            Scalar dx = interpf(pl_g(0), pl_g(1), (float*)GradX.data,
//                                LiveGreyImg.cols, LiveGreyImg.rows);
//            Scalar dy = interpf(pl_g(0), pl_g(1), (float* )GradY.data,
//                                LiveGreyImg.cols, LiveGreyImg.rows);

            Eigen::Matrix<Scalar,1,2> dIl;
            dIl << (Il_xr - Il_xl)/2.0, (Il_yd - Il_yu)/2.0;
//            dIl << dx, dy;
//            printf("%f %f\n",dIlt(0)-dIl(0),dIlt(1)-dIl(1));


            // projection & dehomogenization derivative
            //                        Vector3t KlPl = LiveGreyCM.K() * hPl_g.head(3);
            Eigen::Vector3t KlPl = Klg * hPl_g.head(3);

            Eigen::Matrix2x3t dPl;
            dPl  << 1.0/KlPl(2), 0, -KlPl(0)/(KlPl(2)*KlPl(2)),
                0, 1.0/KlPl(2), -KlPl(1)/(KlPl(2)*KlPl(2));



            const Eigen::Vector4t dIl_dPl_KlgTlr = dIl * dPl * KlgTlr;

            // J = dIl_dPl_KlgTlr * gen_i * Pr
            Eigen::Vector6t J;
            J << dIl_dPl_KlgTlr(0),
                dIl_dPl_KlgTlr(1),
                dIl_dPl_KlgTlr(2),
                -dIl_dPl_KlgTlr(1)*hPr_g(2) + dIl_dPl_KlgTlr(2)*hPr_g(1),
                +dIl_dPl_KlgTlr(0)*hPr_g(2) - dIl_dPl_KlgTlr(2)*hPr_g(0),
                -dIl_dPl_KlgTlr(0)*hPr_g(1) + dIl_dPl_KlgTlr(1)*hPr_g(0);

            // add robust norm here
//            const Scalar w = 1.0;
            const Scalar w = _NormTukey(y, NormCPyr);
//            const Scalar w = _NormL1(y, NormCPyr);

            LHS += J * J.transpose() * w;
            RHS += J * y * w;
            SqError += y * y;
            NumObs++;

          }
        }
#endif
//        std::cout << "LHS: " << std::endl << LHS << std::endl;
//        std::cout << "RHS: " << RHS.transpose() << std::endl;
//        std::cout << "ERROR: " << SqError << std::endl;
//        std::cout << "NUM OBS: " << NumObs << std::endl;


        // solution
        Eigen::Vector6t X;

        // check if we are solving only for rotation, or full estimate
        if( vFullEstimate[nPyrLvl] ) {
          // decompose matrix
          Eigen::FullPivLU< Eigen::Matrix<Scalar,6,6> >    lu_JTJ(LHS);

          // check degenerate system
          if( lu_JTJ.rank() < 6 ) {
            PrintMessage(0, "warning(@L%d I%d) LS trashed. Rank deficient!\n",
                         nPyrLvl+1, nIters+1);
          }

          X = - (lu_JTJ.solve(RHS));
        } else {
          // extract rotation information only
          Eigen::Matrix3t                           rLHS = LHS.block<3,3>(3,3);
          Eigen::Vector3t                           rRHS = RHS.tail(3);
          Eigen::FullPivLU< Eigen::Matrix<Scalar,3,3> > lu_JTJ(rLHS);

          // check degenerate system
          if( lu_JTJ.rank() < 3 ) {
            PrintMessage(0, "warning(@L%d I%d) LS trashed. Rank deficient!\n",
                         nPyrLvl+1, nIters+1);
          }

          Eigen::Vector3t rX;
          rX = - (lu_JTJ.solve(rRHS));

          // pack solution
          X.setZero();
          X.tail(3) = rX;
        }


        // get RMSE
        const Scalar NewError = sqrt(SqError / NumObs);

        if(NewError < LastError) {
          // update error
          LastError = NewError;

          // update Trl
          Trl = (Tlr * Sophus::SE3Group<Scalar>::exp(X)).inverse();

          if( X.norm() < 1e-5 ) {
            PrintMessage(0, "notice(@L%d I%d) Update is too small. Breaking early!\n",
                         nPyrLvl+1, nIters+1);
            break;
          }
        } else {
          PrintMessage(0, "notice(@L%d I%d) Error is increasing. Breaking early!\n",
                       nPyrLvl+1, nIters+1);
          break;
        }
      }
    }

    return LastError;
  }


///
/////////////////////////////////////////////////////////////////////////////
private:

  /////////////////////////////////////////////////////////////////////////////
  inline Scalar _NormTukey( Scalar r, Scalar c )
  {
    const Scalar absr = fabs(r);
    const Scalar roc = r / c;
    const Scalar omroc2 = 1.0f - roc*roc;
    return (absr <= c ) ? omroc2*omroc2 : 0.0f;
  }

  /////////////////////////////////////////////////////////////////////////////
  inline Scalar _NormL1( Scalar r, Scalar /*c*/ )
  {
    const Scalar absr = fabs(r);
    return (absr == 0 ) ? 1.0f : 1.0f / absr;
  }


  /////////////////////////////////////////////////////////////////////////////
  inline calibu::CameraModelGeneric<Scalar> _ScaleCM(
      calibu::CameraModelGeneric<Scalar>  CamModel,
      unsigned int                        nLevel
      )
  {
    const float scale = 1.0f / (1 << nLevel);
    return CamModel.Scaled( scale );
  }


  /////
  /////////////////////////////////////////////////////////////////////////////
public:
  const unsigned int                              PYRAMID_LEVELS;

private:
#ifdef USE_TBB
  tbb::task_scheduler_init                        tbb_scheduler_;
#endif

  std::vector< cv::Mat >                          live_grey_pyramid_;
  std::vector< cv::Mat >                          ref_grey_pyramid_;
  std::vector< cv::Mat >                          ref_depth_pyramid_;
  std::vector<calibu::CameraModelGeneric<Scalar>> live_grey_cam_model_;
  std::vector<calibu::CameraModelGeneric<Scalar>> ref_grey_cam_model_;
  std::vector<calibu::CameraModelGeneric<Scalar>> ref_depth_cam_model_;
  Sophus::SE3t                                    Tgd_;          // reference camera's grey-depth transform
};
