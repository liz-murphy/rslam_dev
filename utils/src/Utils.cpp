// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#include <utils/Utils.h>

////////////////////////////////////////////////////////////////////////////
/// Draw one frustum in the view of the other
Eigen::Vector4t GetFrustrumBoundingBox(
    const calibu::CameraRigT<Scalar>& rig,
    const unsigned int uRefCamera,
    const unsigned int uDstCamera
    )
{

  // check the camera indices
  assert( uRefCamera < rig.cameras.size() );
  assert( uDstCamera < rig.cameras.size() );

  Eigen::Vector4t bounds; // [top left bottom right]
  const calibu::CameraModelAndTransformT<Scalar>& rRefCam = rig.cameras[uRefCamera];
  const calibu::CameraModelAndTransformT<Scalar>& rDstCam = rig.cameras[uDstCamera];
  const Scalar w = rRefCam.camera.Width();
  const Scalar h = rRefCam.camera.Height();
  // if the camera is the same return image size
  if (uRefCamera == uDstCamera) {
    bounds[0] = 0;
    bounds[1] = 0;
    bounds[2] = h;
    bounds[3] = w;
    return bounds;
  }

  // find the fov of the ref camera in the dst view
  const float nSteps = 10;

  Sophus::SO3t Rdr = rDstCam.T_wc.so3().inverse() * rRefCam.T_wc.so3();

  std::vector<Eigen::Vector2t> vPixels;
  for (Scalar x = 0; x < w-1; x+= w/nSteps) {
    Eigen::Vector2t p =
        rDstCam.camera.Project(Rdr * rRefCam.camera.Unproject(Eigen::Vector2t(x,0)));
    vPixels.push_back( p );
  }
  for (Scalar y = 0; y < h; y+= h/nSteps) {

    Eigen::Vector2t p = rDstCam.camera.Project( Rdr * rRefCam.camera.Unproject(Eigen::Vector2t(w,y)) );
    vPixels.push_back( p );
  }
  for (Scalar x = w; x > 0; x-= w/nSteps) {
    Eigen::Vector2t p = rDstCam.camera.Project( Rdr * rRefCam.camera.Unproject(Eigen::Vector2t(x,h)) );
    vPixels.push_back( p );
  }
  for (Scalar y = h; y > 0; y-= h/nSteps) {
    Eigen::Vector2t p = rDstCam.camera.Project( Rdr * rRefCam.camera.Unproject(Eigen::Vector2t(0,y)) );
    vPixels.push_back( p );
  }

  // bounds are ordered as t l b r
  bounds[0] = h/2;
  bounds[1] = w/2;
  bounds[2] = h/2;
  bounds[3] = w/2;

  for (const Eigen::Vector2t& pixel: vPixels) {
    bounds[0] = std::min(bounds[0],pixel[1]);   // top bound
    bounds[1] = std::min(bounds[1],pixel[0]);   // left bound
    bounds[2] = std::max(bounds[2],pixel[1]);   // bottm bound
    bounds[3] = std::max(bounds[3],pixel[0]);   // right bound
  }
  return bounds;
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
        const unsigned int   image_width,    //< Input:
        const unsigned int   image_height,   //< Input:
        const int            row,           //< Input: Feature row position
        const int            col,           //< Input: Feature col position
        double               k              //< Input: Harris constant
        )
{

    double dHessian[4];
    ComputeHessian( image, image_width, image_height, col, row, 3, 3, &dHessian[0] );

    double det   = dHessian[0]*dHessian[3] - dHessian[1]*dHessian[1];
    double trace = dHessian[0] + dHessian[3];
    return det - k * trace * trace;

    /*
    double A,B,C;

    int sigma = 1;

    int     rowNum, colNum;
    int     trow,brow,crow;
    int     tc,bc,cc;
    int     tl,tr,bl,br,cl,cr;
    double  Ix, Iy;
    double  dScale;

    //explicitly initializing the matrix
    A = 0.0;  B = 0.0; C = 0.0;

    dScale = 1.0/(8.0*255.0);

    //For each neighbor, compute the contribution to A
    for(int ii=-sigma; ii<sigma; ii++ ) {
        for(int jj=-sigma; jj<sigma; jj++ ) {
            rowNum = (int)nRow + ii;
            colNum = (int)nCol + jj;

            // Skip if the point is not within the image bounds.
            // The -1 term comes from the fact that the gradient is not
            // computed for the last row and column
            if( rowNum <= 0 || rowNum >= (int)nImageHeight - 2 || colNum <= 0 || colNum >= (int)nImageWidth - 2 ){
                return;// 0.0;//continue;
            }

            //compute the gradients at the required point
            trow = (rowNum-1)*nImageWidth;
            brow = (rowNum+1)*nImageWidth;
            crow = (rowNum)  *nImageWidth;
            tc   = trow + colNum;
            bc   = brow + colNum;
            cc   = crow + colNum;
            tl   = tc - 1;
            tr   = tc + 1;
            bl   = bc - 1;
            br   = bc + 1;
            cl   = cc - 1;
            cr   = cc + 1;

            Ix = (double)(pImage[tr]-pImage[tl] + 2*(pImage[cr]-pImage[cl]) + pImage[br] - pImage[bl])*dScale;
            Iy = (double)(pImage[br]-pImage[tr] + 2*(pImage[bc]-pImage[tc]) + pImage[bl] - pImage[tl])*dScale;

            //update the entries for A
            A += Ix * Ix;
            B += Iy * Iy;
            C += Ix * Iy;
        }
    }

    //Now the matrix A has been computed. Just compute and return the score.
    double det   = ( A * B ) - ( C * C ) ;
    double trace = ( A + B ) ;
    fScore       = det - dK * trace * trace;
     */

}

///////////////////////////////////////////////////////////////////////////////
void HarrisScore( const unsigned char*         image,
                  const unsigned int           image_width,
                  const unsigned int           image_height,
                  std::vector< cv::KeyPoint >& points,
                  double                       k)
{

    std::vector< cv::KeyPoint >::iterator itPoint;

    for (itPoint = points.begin(); itPoint != points.end(); ++itPoint) {

        //        if(itPoint->pt.y < 1 || itPoint->pt.y > uImageHeight-2 ||
        //           itPoint->pt.x < 1 || itPoint->pt.x > uImageWidth-2 ){ continue; }

        itPoint->response = ComputeScore(image, image_width, image_height,
                                         itPoint->pt.y, itPoint->pt.x, k);

    }

}

/////////////////////////////////////////////////////////////////////////////
void BuildJetMap(
        std::vector<unsigned char>& color_map,
        unsigned int length
        )
{
    color_map.resize( length*3 );

    int sectlength = length / 5;
    const unsigned int sect1 = (1 * length) / 5;
    const unsigned int sect2 = (2 * length) / 5;
    const unsigned int sect3 = (3 * length) / 5;
    const unsigned int sect4 = (4 * length) / 5;

    //printf("%d %d %d %d %d %d\n",sect1,sect2,sect3,sect4,length,sectlength);

    if (sectlength < 1) sectlength = 1;

    for (unsigned int ii = 0; ii < length; ++ii) {
        if (ii < sect1) {
            /* red is zero */
            color_map[ii*3 + 0] = 0;
            /* green is zero */
            color_map[ii*3 + 1] = 0;
            /* blue is rising */
            color_map[ii*3 + 2] = (128* ii) / sectlength + 127;
        } else if (ii < sect2) {
            /* red is zero */
            color_map[ii*3 + 0] = 0;
            /* green is rising */
            color_map[ii*3 + 1] = (255 * (ii - sect1)) / sectlength;
            /* blue is flat */
            color_map[ii*3 + 2] = 255;
        } else if (ii < sect3) {
            /* red is rising */
            color_map[ii*3 + 0] = (255 * (ii - sect2)) / sectlength;
            /* green is flat */
            color_map[ii*3 + 1] = 255;
            /* blue is falling */
            color_map[ii*3 + 2] = 255 - (255 * (ii - sect2)) / sectlength;
        } else if (ii < sect4) {
            /* red is flat */
            color_map[ii*3 + 0] = 255;
            /* green is falling */
            color_map[ii*3 + 1] = 255 - (255 * (ii - sect3)) / sectlength;
            /* blue is zero */
            color_map[ii*3 + 2] = 0;
        } else {
            /* red is falling */
            color_map[ii*3 + 0] = 255 - (128 * (ii - sect4)) / sectlength;
            /* green is zero */
            color_map[ii*3 + 1] = 0;
            /* blue is zero */
            color_map[ii*3 + 2] = 0;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////
Scalar ComputeAngle(Eigen::Vector3t c1, Eigen::Vector3t c2, Eigen::Vector3t x)
{
  Eigen::Vector3t v1 = c1 - x;
  Eigen::Vector3t v2 = c2 - x;
  return acos(v1.dot(v2) / (v1.norm() *  v2.norm())) * 180 / M_PI;
}
