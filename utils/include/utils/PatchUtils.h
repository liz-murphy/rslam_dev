// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.


#pragma once

#include <iostream>
#include <assert.h>
#include <Eigen/Core>
#include <utils/MathTypes.h>

#ifdef __ARM_NEON__
#   include <arm_neon.h>
#endif



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline float interp(
        float x,						//< Input: X coordinate
        float y,						//< Input: Y coordinate
        const unsigned char* pImage,	//< Input: Pointer to Image
        const unsigned int uImageWidth,	//< Input: Image width
        const unsigned int uImageHeight	//< Input: Image height
        )
{
    if( !(x >= 0 && y >= 0 && x <= uImageWidth - 2 && y <= uImageHeight - 2) ){
        std::cout << "Bad: " << x << ", " << y << std::endl;
    }
    x = std::max(std::min(x,(float)uImageWidth-2.0f),2.0f);
    y = std::max(std::min(y,(float)uImageHeight-2.0f),2.0f);

    const int   px = (int) x;  /* top-left corner */
    const int   py = (int) y;
    const float ax = x - px;
    const float ay = y - py;
    const float ax1 = 1.0f - ax;
    const float ay1 = 1.0f - ay;

    const unsigned char* p0 = pImage + (uImageWidth*py) + px;
//    const unsigned char& p1 = p0[0];
//    const unsigned char& p2 = p0[1];
//    const unsigned char& p3 = p0[ImageWidth];
//    const unsigned char& p4 = p0[ImageWidth+1];
    //return ( ax1*(ay1*p1+ay*p3) + ax*(ay1*p2 + ay*p4) );

    float p1 = (float)p0[0];
    float p2 = (float)p0[1];
    float p3 = (float)p0[uImageWidth];
    float p4 = (float)p0[uImageWidth+1];
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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline float interpf(
        float x,						//< Input: X coordinate
        float y,						//< Input: Y coordinate
        const float* pImage,	        //< Input: Pointer to Image
        int ImageWidth,					//< Input: Image width
        int ImageHeight					//< Input: Image height
        )
{
    if( !(x >= 0 && y >= 0 && x <= ImageWidth - 2 && y <= ImageHeight - 2) ){
        std::cout << "Bad." << std::endl;
    }
    x = std::max(std::min(x,(float)ImageWidth-2.0f),2.0f);
    y = std::max(std::min(y,(float)ImageHeight-2.0f),2.0f);
    const int   px = (int) x;
    const int   py = (int) y;
    const float ax = x - px;
    const float ay = y - py;
    const float ax1 = 1.0f - ax;
    const float ay1 = 1.0f - ay;

    const float* p0 = pImage + (ImageWidth*py) + px;
    const float& p1 = p0[0];
    const float& p2 = p0[1];
    const float& p3 = p0[ImageWidth];
    const float& p4 = p0[ImageWidth+1];

    return ( ax1*(ay1*p1+ay*p3) + ax*(ay1*p2 + ay*p4) );
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool LoadInterpolatedPatch(
        const unsigned char* pImage,       //< Input:
        const unsigned int   nImageWidth,  //< Input:
        const unsigned int   nImageHeight, //< Input:
        const float          nPatchPosX,   //< Input:
        const float          nPatchPosY,   //< Input:
        unsigned char*       pPatch,       //< Output:
        const unsigned int   nPatchWidth,
        const unsigned int   nPatchHeight
        );

//bool LoadWarpedPatch(
//        const unsigned char*    pImage,       //< Input:
//        const unsigned int      nImageWidth,  //< Input:
//        const unsigned int      nImageHeight, //< Input:
//        const Eigen::Matrix3f&  H,            //< Input: warping homography
//        const unsigned int      nPatchWidth,
//        const unsigned int      nPatchHeight,
//        unsigned char*          pPatch       //< Output:
//        );


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// checks if a quadrilateral is convex
template<typename Scalar=double>
bool IsConvexQ(
        const Eigen::Matrix<Scalar,2,1>& tl,
        const Eigen::Matrix<Scalar,2,1>& tr,
        const Eigen::Matrix<Scalar,2,1>& br,
        const Eigen::Matrix<Scalar,2,1>& bl
        )
{
    const Eigen::Matrix<Scalar,2,1> p12 = tr - tl;
    const Eigen::Matrix<Scalar,2,1> p23 = br - tr;
    const Eigen::Matrix<Scalar,2,1> p34 = bl - br;
    const Eigen::Matrix<Scalar,2,1> p41 = tl - bl;

    // compute determinants
    const Scalar d1 = p12(0)*p23(1) - p12(1)*p23(0);
    const Scalar d2 = p23(0)*p34(1) - p23(1)*p34(0);
    const Scalar d3 = p34(0)*p41(1) - p34(1)*p41(0);
    const Scalar d4 = p41(0)*p12(1) - p41(1)*p12(0);

    if( (d1 <= 0 && d2 <= 0 && d3 <= 0 && d4 <= 0 ) ||
        (d1 >= 0 && d2 >= 0 && d3 >= 0 && d4 >= 0 ) ){
        return true;
    }else{
        return false;
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<unsigned int PATCHSIZE>
bool LoadWarpedPatch(
        const unsigned char*              pImage,       //< Input:
        const unsigned int                uImageWidth,  //< Input:
        const unsigned int                uImageHeight, //< Input:
        const Eigen::Matrix3t&  H,            //< Input: warping homography
        unsigned char*                    pPatch       //< Output:
        )
{

  Eigen::Matrix<Scalar,3,4> canonicalCorners;
    canonicalCorners << 0.0, PATCHSIZE-1, PATCHSIZE-1,         0.0,
                         0.0,         0.0, PATCHSIZE-1, PATCHSIZE-1,
                         1.0,         1.0,         1.0,         1.0;
    Eigen::Matrix<Scalar,3,4> pc = H * canonicalCorners;
    const Scalar n0 = 1.0/pc(2,0); pc(0,0) *= n0; pc(1,0) *= n0;
    const Scalar n1 = 1.0/pc(2,1); pc(0,1) *= n1; pc(1,1) *= n1;
    const Scalar n2 = 1.0/pc(2,2); pc(0,2) *= n2; pc(1,2) *= n2;
    const Scalar n3 = 1.0/pc(2,3); pc(0,3) *= n3; pc(1,3) *= n3;
    const Scalar m = 2;                  // margin
    const Scalar w  = uImageWidth - m;
    const Scalar h  = uImageHeight - m;

    // check that we have a convex set
    if( !IsConvexQ<Scalar>( pc.template block<2,1>(0,0),
                            pc.template block<2,1>(0,1),
                            pc.template block<2,1>(0,2),
                            pc.template block<2,1>(0,3) ) ){
        std::cout << "WARNING: non-convex quadrilateral generated by homography -> skipping" << std::endl;
        std::cout << "  tl: [" << pc(0,0) << " " << pc(1,0) << "]" <<
                     "  tr: [" << pc(0,1) << " " << pc(1,1) << "]" <<
                     "  br: [" << pc(0,2) << " " << pc(1,2) << "]" <<
                     "  bl: [" << pc(0,3) << " " << pc(1,3) << "]" << std::endl;
        std::cout << "H: " << std::endl << H << std::endl;
        return false;
    }

    if( pc(0,0) >= m && pc(0,0) <= w && pc(1,0) >= m && pc(1,0) <= h &&
        pc(0,1) >= m && pc(0,1) <= w && pc(1,1) >= m && pc(1,1) <= h &&
        pc(0,2) >= m && pc(0,2) <= w && pc(1,2) >= m && pc(1,2) <= h &&
        pc(0,3) >= m && pc(0,3) <= w && pc(1,3) >= m && pc(1,3) <= h )
    {
        Eigen::Matrix<Scalar,3,1> cp; cp << 0.0,0.0,1.0;
        Eigen::Matrix<Scalar,3,1> p;
        Scalar x,y,n;
        for( unsigned int uRow = 0; uRow < PATCHSIZE; ++uRow ){
            for( unsigned int uCol = 0; uCol < PATCHSIZE; ++uCol ){
                cp[0] = uCol; cp[1] = uRow;
                p = H*cp;
                n = 1.0/p[2];
                x = p[0]*n;
                y = p[1]*n;

                int px = (int)x;  /* top-left corner */
                int py = (int)y;
                Scalar ax = x - px;
                Scalar ay = y - py;
                Scalar ax1 = 1.0f - ax;
                Scalar ay1 = 1.0f - ay;

                const unsigned char* p0 = pImage + (uImageWidth*py) + px;

                Scalar p1 = (Scalar)p0[0];
                Scalar p2 = (Scalar)p0[1];
                p0 += uImageWidth;
                Scalar p3 = (Scalar)p0[0];
                Scalar p4 = (Scalar)p0[1];
                p1 *= ay1;
                p2 *= ay1;
                p3 *= ay;
                p4 *= ay;
                p1 += p3;
                p2 += p4;
                p1 *= ax1;
                p2 *= ax;

                *(pPatch++) = p1 + p2;
            }
        }

        return true;
    }else{
        return false;
    }

}


/////Compute homography
//inline Eigen::Matrix3d ComputeHomography(
//        const Eigen::Matrix<double,2,4>& src,
//        const Eigen::Matrix<double,2,4>& dst
//        );
