#ifndef DENSEALIGNMENT_H
#define DENSEALIGNMENT_H

#include <calibu/cam/CameraRig.h>
#include <opencv2/opencv.hpp>

#include <utils/MathTypes.h>
#include <utils/PatchUtils.h>
#include <utils/PoseHelpers.h> // T2Cart
#include <utils/ESM.h>

// TODO: this is terribly unoptimized --

// derivative of transfer function (warping function) wrt to rotation
Eigen::Matrix2x3t dTransfer_dRba(const calibu::CameraModelGeneric<Scalar> &cam,
    const Sophus::SE3t &t_ba,
    const Eigen::Vector2t &pa
    );


class DenseAlignment {
 public:
  DenseAlignment() : threshold_(0.0), size_pow_2_(0) {}

  void Init(
      const calibu::CameraModelGeneric<Scalar> &cam,
      const unsigned int                       nThumbWidth,
      const double                             thresh
      )
  {
    threshold_ = thresh;
    float fScale = float(nThumbWidth) / cam.Width();
    size_pow_2_ = log( 1.0/fScale) / log(2);
    cameras_.resize(size_pow_2_);
    cur_thumbs_.resize(size_pow_2_);
    prev_thumbs_.resize(size_pow_2_);

    cameras_[0] = cam.Scaled( 0.5 );
    for (int ii = 1; ii < size_pow_2_; ++ii) {
      cameras_[ii] = cameras_[ii-1].Scaled( 0.5 );
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  void SetRefImage(
      const cv::Mat& ref_image
      )
  {
    cv::pyrDown(ref_image, prev_thumbs_[0],
                cv::Size(ref_image.cols/2, ref_image.rows/2));
    for (int ii = 1; ii < size_pow_2_; ++ii) {
      cv::pyrDown(prev_thumbs_[ii-1], prev_thumbs_[ii],
          cv::Size(prev_thumbs_[ii-1].cols/2, prev_thumbs_[ii-1].rows/2));
    }

    if ((int)grad_x_.size() < ref_image.rows*ref_image.cols) {
      grad_x_.resize( ref_image.rows*ref_image.cols );
      grad_y_.resize( ref_image.rows*ref_image.cols );
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  Scalar Align(
      const cv::Mat &ref_image,
      const cv::Mat &cur_image,
      const calibu::CameraModelGeneric<Scalar> &cam,
      Sophus::SE3Group<Scalar> &T_ba                    ///< Output:
      )
  {
    Gradient(cur_image.data, cur_image.cols,
             cur_image.rows, cur_image.cols,
             &grad_x_[0], &grad_y_[0] );


    int num_votes;
    Scalar sq_error;
    Scalar sq_prev_error = 1e9;
    const int max_iter = 10;

    for (int n = 0; n < max_iter; ++n) {
      sq_error = 0;
      Eigen::Matrix3t A = Eigen::Matrix3t::Zero();
      Eigen::Vector3t b = Eigen::Vector3t::Zero();
      num_votes = 0;
      for (int ii = 0; ii < ref_image.rows; ++ii) {
        for (int jj = 0; jj < ref_image.cols; ++jj) {
          Eigen::Vector2t pa = Eigen::Vector2t(jj, ii);
          Eigen::Vector2t pb = cam.Transfer(T_ba, pa, 0);
          if (pb(0) < 0 || pb(0) >= cur_image.cols-2 ||
              pb(1) < 0 || pb(1) >= cur_image.rows-2) {
            continue;
          }
          num_votes++;

          const float ia = ref_image.at<unsigned char>(ii,jj);
          const float ib = interp(pb(0), pb(1), cur_image.data,
                                  cur_image.cols, cur_image.rows);
          Scalar error = ia - ib;
          // Scalar w = dError > 1.345*dStdDev ? 1.345*dStdDev/dError : 1; // huber kernel
          Scalar w = 1;
          Scalar dx = interpf(pb(0), pb(1), &grad_x_[0],
                              cur_image.cols, cur_image.rows);
          Scalar dy = interpf(pb(0), pb(1), &grad_y_[0],
                              cur_image.cols, cur_image.rows);

          const Eigen::Matrix<Scalar,1,2> dIdW(dx, dy);
          const Eigen::Matrix2x3t dWdR = dTransfer_dRba(cam, T_ba, pa);
          const Eigen::Vector3t J = -dIdW*dWdR;
          A += J*J.transpose()*w;
          b += J*w*error;
          sq_error += error*w*error;
        }
      }

      const Eigen::Vector3t update = -A.llt().solve(b);

      if (sq_error < sq_prev_error) {
        sq_prev_error = sq_error;
        T_ba = T_ba*Sophus::SE3t::exp( (Eigen::Vector6t()<<0, 0, 0, update[0],
                               update[1], update[2]).finished() );
        //                Tba = Tba*SE3t::exp( (Vector6t()<<0, 0, 0, 0, dUpdate[1], 0 ).finished() );
        //                std::cout << n << " Error: "<< dSqError << ": update = " << dUpdate.transpose() << ", res = " << Tba.so3().log().transpose()*180/M_PI << std::endl;
        if( update.norm() < 1e-5 ){
          break;
        }
      }
      else{
        //                std::cout << n << " Error increasing -- finshing." << std::endl;
        break;
      }
    }
    return sqrt( sq_error/num_votes );
  }

  //////////////////////////////////////////////////////////////////////////////
  ///< Output: rotation between views
  bool Align(
      const cv::Mat& cur_image,  ///< Input: rotated view of scene
      Sophus::SE3Group<Scalar>& Tba_est ///< Input/Output: transform from a to b
      )
  {
    Sophus::SE3Group<Scalar> Tba_save = Tba_est;
    // add some error to fix
    // Tba_est =
    //   Tba_est*SE3t::exp((Vector6t()<<0, 0, 0, 0, M_PI*30.0/180.0, 0 ).finished());

    cv::pyrDown(cur_image, cur_thumbs_[0],
                cv::Size( cur_image.cols/2, cur_image.rows/2 ) );
    for (int ii = 1; ii < size_pow_2_; ++ii) {
      cv::pyrDown( cur_thumbs_[ii-1], cur_thumbs_[ii],
                   cv::Size( cur_thumbs_[ii-1].cols/2,
                             cur_thumbs_[ii-1].rows/2));
    }

    Scalar error = 0;
    for (int ii = size_pow_2_-1; ii >= size_pow_2_-3; --ii) {
      error = Align(prev_thumbs_[ii], cur_thumbs_[ii], cameras_[ii], Tba_est);
    }

#if 0
    // compute warped image
    int n = 1;
    cv::Mat tmp = m_PrevThumbs[n].clone();
    for( int r = 0; r < m_PrevThumbs[n].rows; r++ ){
      for( int c = 0; c < m_PrevThumbs[n].cols; c++ ){
        Vector2t p = m_Cams[n].Transfer( Tba_est, Vector2t(c,r), 0 );
        if( p(0) < 0 || p(0) >= m_CurThumbs[n].cols-2 ||
            p(1) < 0 || p(1) >= m_CurThumbs[n].rows-2 ){
          tmp.at<uchar>(r,c) = 0;
          continue;
        }
        tmp.at<uchar>(r,c) =
            (unsigned char)interp(p[0],p[1], m_CurThumbs[n].data,
                                  m_PrevThumbs[n].cols, m_PrevThumbs[n].rows);
      }
    }

    cv::imshow( "W", tmp );
    cv::imshow( "Ref", m_PrevThumbs[n] );
    cv::imshow( "Ref-Cur(W)", m_PrevThumbs[n]-tmp );

#endif

    // copy cur thumbs
    for (int ii = 1; ii < size_pow_2_; ++ii) {
      prev_thumbs_[ii] = cur_thumbs_[ii].clone();
    }

    if (error > threshold_) {
      Tba_est = Tba_save;
      return false;
    }

    // thing is in vision frame, gah
    Tba_est =
        calibu::ToCoordinateConvention<Scalar>(Tba_est, calibu::RdfRobotics);
    Tba_est = Tba_est.inverse();
    return true;
  }

private:

  double                                            threshold_;
  int                                               size_pow_2_;
  std::vector<cv::Mat>                              cur_thumbs_;
  std::vector<cv::Mat>                              prev_thumbs_;
  std::vector< calibu::CameraModelGeneric<Scalar> > cameras_;
  std::vector<float>                                grad_x_;
  std::vector<float>                                grad_y_;
};



#endif // DENSEALIGNMENT_H
