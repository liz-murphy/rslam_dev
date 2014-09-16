// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.


#include <Eigen/Eigen>
#include <utils/PatchUtils.h>
#include <assert.h>
#include <stdio.h>
//#include <mmintrin.h>
//#include <smmintrin.h>

/*
static const __m128 CONST_1111 = _mm_set1_ps(1);

inline __m128 _computeWeightsSSE( float x, float y )
{
     // compute weights                                        // ----> least significant bit
    __m128 ssx       = _mm_set_ss(x);                         // ssx  = [ 0 0 0 x]
    __m128 ssy       = _mm_set_ss(y);                         // ssy  = [ 0 0 0 y ]
    __m128 psXY      = _mm_unpacklo_ps(ssx, ssy);             // psXY = [ 0 0 y x ] 
    __m128 psXYfloor = _mm_floor_ps(psXY);                    // floor SSE4 instruction
    __m128 psXYfrac  = _mm_sub_ps(psXY, psXYfloor);           // psXYfrac  = [ 0 0 y-ry x-rx] 
    __m128 psXYfrac1 = _mm_sub_ps(CONST_1111, psXYfrac);      // psXYFrac1 = [ 0 0 1-y-ry, 1-x-rx]
    __m128 w_x       = _mm_unpacklo_ps(psXYfrac1, psXYfrac);  // wx = [ (y-ry) (1-y-ry) (x-rx) (1-x-rx) ]
           w_x       = _mm_movelh_ps(w_x, w_x);               // wx = [ (x-rx) (1-x-rx) (x-rx) (1-x-rx) ]
    __m128 w_y       = _mm_shuffle_ps(psXYfrac1, psXYfrac, _MM_SHUFFLE(1, 1, 1, 1)); 
                                                              // wy = [ (y-ry) (y-ry) (1-y-ry) (1-y-ry) ]
    return _mm_mul_ps(w_x, w_y);                              // multiplication of corresponting 4 floats in wx and wy
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline float _interpSSE(
        float x,					//< Input: X coordinate
        float y,					//< Input: Y coordinate
        const unsigned char* data,	//< Input: Pointer to Image
        int stride				    //< Input: Image width
        )
{
    
    __m128 weights = _computeWeightsSSE(x,y);
    
    // Load the data (2 pixels in one load)
    const unsigned char* p0 = data + (int)x + (int)y * stride; 
    __m128 ssp1  = _mm_set_ss( (float)p0[0] );
    __m128 ssp2  = _mm_set_ss( (float)p0[1] );
    __m128 ssp3  = _mm_set_ss( (float)p0[stride] );
    __m128 ssp4  = _mm_set_ss( (float)p0[stride+1] );
    __m128 ssp12 = _mm_unpacklo_ps( ssp1, ssp2 );
    __m128 ssp34 = _mm_unpacklo_ps( ssp3, ssp4 );
    __m128 pixels = _mm_movelh_ps( ssp12, ssp34 );
    
    // multiply each pixel with its weight
    const int mask = 0xf1;  // 11110001
    __m128 dp = _mm_dp_ps( weights, pixels, mask );
    float result;
    _mm_store_ss( &result, dp);
    
    return result;
}
 */

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool LoadInterpolatedPatch(
        const unsigned char* pImage,       //< Input:
        const unsigned int   nImageWidth,  //< Input:
        const unsigned int   nImageHeight, //< Input:
        const float          fPatchPosX,   //< Input:
        const float          fPatchPosY,   //< Input:
        unsigned char*       pPatch,       //< Output:
        const unsigned int   nPatchWidth,
        const unsigned int   nPatchHeight
        )
{
//    assert(nPatchWidth % 2 == 1);
//    assert(nPatchHeight % 2 == 1);
    
    const int px    = (int)fPatchPosX;
    const int py    = (int)fPatchPosY;
    const int radx  = nPatchWidth/2;
    const int rady  = nPatchHeight/2;

    // check patch boundries -- adjusted to cover ESM's +/- 1 pixel bigger patch
    if( ( ( px - radx - 1 ) < 0) || ( ( px + radx + 1) > (int)nImageWidth) || 
        ( ( py - rady - 1 ) < 0) || ( ( py + rady + 1) > (int)nImageHeight) ) {
        return false;
    }

    int index = 0;
    const float fx0  = fPatchPosX - px;
    const float fy0  = fPatchPosY - py;
    const float fx1  = 1.0f - fx0;
    const float fy1  = 1.0f - fy0;
    const float wx[] = { fx1, fx0, fx1, fx0 };
    const float wy[] = { fy1, fy1, fy0, fy0 };
    const float w[]  = { wx[0]*wy[0], wx[1]*wy[1], wx[2]*wy[2], wx[3]*wy[3] };
    
    for( int nRow = py - rady; nRow <= py + rady; nRow++ ) {
        for( int nCol = px - radx; nCol <= px + radx; nCol++ ) {
            
            const unsigned char* p0 = pImage + (nImageWidth*nRow) + nCol;
            const unsigned char& p1 = p0[0];
            const unsigned char& p2 = p0[1];
            const unsigned char& p3 = p0[nImageWidth];
            const unsigned char& p4 = p0[nImageWidth+1];
            // add 0.5 for a cheap round
            
            pPatch[ index++ ] = (unsigned char)( w[0]*p1 + w[1]*p2 + w[2]*p3 + w[3]*p4 + 0.5f);
        }
    }
    return true;
}

