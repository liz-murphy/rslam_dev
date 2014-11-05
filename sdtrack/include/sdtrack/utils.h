#pragma once
#include <stdint.h>
#include <Eigen/Eigen>

#ifdef REAL_TYPE
typedef REAL_TYPE Scalar;
#else // REAL_TYPE
typedef double Scalar;
#endif // REAL_TYPE

namespace Sophus
{

typedef Sophus::SO3Group<Scalar> SO3t;
typedef Sophus::SE3Group<Scalar> SE3t;
}

/*namespace Eigen
{
  template<typename EigenT>
  using aligned_vector = std::vector<EigenT, Eigen::aligned_allocator<EigenT> >;

  #define USING_VECTOR_ARRAY(size)                                        \
    using Vector##size##tArray = aligned_vector<Matrix<Scalar, size, 1> >

  USING_VECTOR_ARRAY(2);
  USING_VECTOR_ARRAY(3);
  USING_VECTOR_ARRAY(4);
  USING_VECTOR_ARRAY(5);
  USING_VECTOR_ARRAY(6);
  typedef Matrix<Scalar, Eigen::Dynamic, 1> VectorXt;
  typedef Matrix<Scalar, 2, 1> Vector2t;
  typedef Matrix<Scalar, 3, 1> Vector3t;
  typedef Matrix<Scalar, 4, 1> Vector4t;
  typedef Matrix<Scalar, 6, 1> Vector6t;
  typedef Matrix<Scalar, 4, 4> Matrix4t;
}*/
namespace sdtrack
{
  static Eigen::IOFormat kCleanFmt(4, 0, ", ", ";\n", "", "");
  static Eigen::IOFormat kLongFmt(Eigen::FullPrecision,
                                  0, ", ", ";\n", "", "");
  static Eigen::IOFormat kLongCsvFmt(Eigen::FullPrecision,
                                     0, ", ", "\n", "", "");

  //template<typename Scalar=double>
/*  inline Eigen::Matrix<Scalar, 4, 1> MultHomogeneous(
      const Sophus::SE3Group<Scalar>& lhs,
      const Eigen::Matrix<Scalar, 4, 1>& rhs )
  {
    Eigen::Matrix<Scalar, 4, 1> out;
    out.head<3>() =
        lhs.so3() * (Eigen::Matrix<Scalar, 3, 1>)rhs.head<3>() +
        lhs.translation()*rhs[3];
    out[3] = rhs[3];
    return out;
  }
*/

  template<typename Scalar = double>
  inline Scalar powi(const Scalar x, const int y) {
    if (y == 0) {
      return 1.0;
    } else if (y < 0) {
      return 1.0 / powi(x, -y);
    } else if (y == 0) {
      return 1.0;
    } else {
      Scalar ret = x;
      for (int ii = 1; ii < y; ii++) {
        ret *= x;
      }
      return ret;
    }
  }

  inline int hsv2rgb(const Eigen::Vector3d& hsv, Eigen::Vector3d& rgb) {
    /*
      * Purpose:
      * Convert HSV values to RGB values
      * All values are in the range [0.0 .. 1.0]
      */
    float S, H, V, F, M, N, K;
    int   I;

    S = hsv[1];  /* Saturation */
    H = hsv[0];  /* Hue */
    V = hsv[2];  /* value or brightness */

    if ( S == 0.0 ) {
      /*
         * Achromatic case, set level of grey
         */
      rgb[0] = V;
      rgb[1] = V;
      rgb[2] = V;
    } else {
      /*
         * Determine levels of primary colours.
         */
      if (H >= 1.0) {
        H = 0.0;
      } else {
        H = H * 6;
      } /* end if */
      I = (int) H;   /* should be in the range 0..5 */
      F = H - I;     /* fractional part */

      M = V * (1 - S);
      N = V * (1 - S * F);
      K = V * (1 - S * (1 - F));

      if (I == 0) { rgb[0] = V; rgb[1] = K; rgb[2] = M; }
      if (I == 1) { rgb[0] = N; rgb[1] = V; rgb[2] = M; }
      if (I == 2) { rgb[0] = M; rgb[1] = V; rgb[2] = K; }
      if (I == 3) { rgb[0] = M; rgb[1] = N; rgb[2] = V; }
      if (I == 4) { rgb[0] = K; rgb[1] = M; rgb[2] = V; }
      if (I == 5) { rgb[0] = V; rgb[1] = M; rgb[2] = N; }
    } /* end if */

    return 0;
  } /* end function hsv2rgb */

  inline double Interpolate(
      double x,						//< Input: X coordinate
      double y,						//< Input: Y coordinate
      const unsigned char* pImage,	//< Input: Pointer to Image
      const unsigned int uImageWidth,	//< Input: Image width
      const unsigned int uImageHeight	//< Input: Image height
      )
  {
        if( !(x >= 0 && y >= 0 && x <= uImageWidth - 1 &&
              y <= uImageHeight - 1) ){
          std::cerr << "\t!!BAD: " << x << ", " << y << " w: " << uImageWidth <<
                       " h " << uImageHeight << std::endl;
        }
    //    x = std::max(std::min(x,(double)uImageWidth-2.0),2.0);
    //    y = std::max(std::min(y,(double)uImageHeight-2.0),2.0);

    const int   px = (int) x;  /* top-left corner */
    const int   py = (int) y;
    const double ax = x - px;
    const double ay = y - py;
    const double ax1 = 1.0 - ax;
    const double ay1 = 1.0 - ay;

    const unsigned char* p0 = pImage + (uImageWidth*py) + px;
    //    const unsigned char& p1 = p0[0];
    //    const unsigned char& p2 = p0[1];
    //    const unsigned char& p3 = p0[ImageWidth];
    //    const unsigned char& p4 = p0[ImageWidth+1];
    //return ( ax1*(ay1*p1+ay*p3) + ax*(ay1*p2 + ay*p4) );

    double p1 = (double)p0[0];
    double p2 = (double)p0[1];
    double p3 = (double)p0[uImageWidth];
    double p4 = (double)p0[uImageWidth+1];
    p1 *= ay1;
    p2 *= ay1;
    p3 *= ay;
    p4 *= ay;
    p1 += p3;
    p2 += p4;
    p1 *= ax1;
    p2 *= ax;

    return p1 + p2;

    // return ( ax1*(ay1*p0[0]+ay*p0[ImageWidth]) + ax*(ay1*p0[1] + ay*p0[ImageWidth-1]) );
  }

  inline void ComputeHessian(const unsigned char* image,
                             const unsigned int   image_width,
                             const unsigned int   ,//uImageHeight,
                             const int            col,
                             const int            row,
                             const unsigned int   patch_width,
                             const unsigned int   patch_height,
                             double*              hessian) {
    size_t start_row = row - (patch_height - 1)/2;
    size_t end_row   = row + (patch_height - 1)/2;
    size_t start_col = col - (patch_width - 1)/2;
    size_t end_col   = col + (patch_width - 1)/2;

    std::vector<int> int_hessian(4, 0);
    for (size_t ii=start_row; ii <= end_row; ++ii) {
      const unsigned char* upper_row = &image[(ii-1)*image_width + start_col];
      const unsigned char* lower_row = &image[(ii+1)*image_width + start_col];

      const unsigned char* left_col  = &image[ii*image_width + start_col];
      const unsigned char* right_col = &image[ii*image_width + start_col + 2];

      for (size_t jj = start_col; jj <= end_col; ++jj) {
        // Calculate the image gradient (do not divide by two yet,
        // divide by 4 at the end)
        int row_val = *lower_row++ - *upper_row++;
        int col_val = *right_col++ - *left_col++;

        // Calculate JpJ (Symmetric matrix is stored as a lower
        // triangular matrix)
        int_hessian[0] += col_val * col_val;
        int_hessian[1] += row_val * col_val;
        int_hessian[3] += row_val * row_val;
      }
    }

    hessian[0] = static_cast<double>(int_hessian[0]) / 4.0;
    hessian[3] = static_cast<double>(int_hessian[3]) / 4.0;
    hessian[1] = static_cast<double>(int_hessian[1]) / 4.0;
    hessian[2] = hessian[1];
  }

  inline float ComputeScore(
      const unsigned char* image,         //< Input: Image data pointer
      uint32_t   image_width,    //< Input:
      uint32_t   image_height,   //< Input:
      int        row,           //< Input: Feature row position
      int        col,           //< Input: Feature col position
      uint32_t   patch_dim,
      double     k,              //< Input: Harris constant
      double&    l1,
      double&    l2
      )
  {

    double dHessian[4];
    ComputeHessian( image, image_width, image_height, col, row,
                    patch_dim, patch_dim, &dHessian[0] );

    double det   = dHessian[0]*dHessian[3] - dHessian[1]*dHessian[1];
    double trace = dHessian[0] + dHessian[3];
    l1 = trace / 2 + sqrt((trace * trace) / 4 -det);
    l2 = trace / 2 - sqrt((trace * trace) / 4 -det);
    // return det - k * trace * trace;
    return std::min(l1, l2);
  }

  ///////////////////////////////////////////////////////////////////////////////
  inline void HarrisScore(const unsigned char*         image,
                          uint32_t           image_width,
                          uint32_t           image_height,
                          uint32_t           patch_dim,
                          std::vector< cv::KeyPoint >& points,
                          double                       k = 0.04)
  {

    std::vector< cv::KeyPoint >::iterator it;
    const uint32_t max_x = image_width - patch_dim;
    const uint32_t max_y = image_height - patch_dim;

    for (it = points.begin(); it != points.end(); ++it) {
      if (it->pt.x < patch_dim || it->pt.x > max_x ||
          it->pt.y < patch_dim || it->pt.y > max_y) {
        continue;
      }
      double l1, l2;
      it->response = ComputeScore(image, image_width, image_height,
                                  it->pt.y, it->pt.x,
                                  patch_dim, k, l1, l2);
      it->angle = std::max(l1, l2);
    }
  }

  ////////////////////////////////////////////////////////////////////////////////
  inline double ScorePatchesNCC(const std::vector<double>& patch_a,
                        const std::vector<double>& patch_b,
                        const unsigned int   patch_width,
                        const unsigned int   patch_height)
  {
    double mean_a  = 0.0;
    double mean_b  = 0.0;

    unsigned int size = patch_width*patch_height;

    for (unsigned int ii = 0; ii < size; ++ii) {
      mean_a += patch_a[ii];
      mean_b += patch_b[ii];
    }

    mean_a /= size;
    mean_b /= size;

    double denA = 0.0;
    double denB = 0.0;
    double num = 0.0;

    for (unsigned int ii = 0; ii < size; ++ii) {
      double fA = patch_a[ii] - mean_a;
      double fB = patch_b[ii] - mean_b;
      num  += fA*fB;
      denA += fA*fA;
      denB += fB*fB;
    }

    double den = sqrt(denA*denB);

    return num/den;
  }
}
