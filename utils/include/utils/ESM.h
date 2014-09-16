// This file was extracted from the CTrack Library.
//
// Copyright (C) 2010 Christopher Mei (christopher.m.mei@gmail.com)
// All rights reserved.
//
// Redistribution and use in source and binary forms are permitted provided
// that the above copyright notice and this paragraph are duplicated in all
// such forms and that any documentation, advertising materials, and other
// materials related to such distribution and use acknowledge that the software
// was developed by the author.  The name of the author may not be used to
// endorse or promote products derived from this software without specific
// prior written permission.  THIS SOFTWARE IS PROVIDED ``AS IS'' AND WITHOUT
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.

#pragma once

#include <utils/CTrackHomography.h>
#include <utils/PatchUtils.h>

#define ESM_TRANSL       0
#define ESM_TRANSL_ILLUM 1
#define ESM_SE2          2
#define ESM_SE2_ILLUM    3
#define ESM_AFFINE       4
#define ESM_AFFINE_ILLUM 5
#define ESM_HOMOG        6
#define ESM_HOMOG_ILLUM  7

// Differentiation with respect to
// the Lie generators
#define JH1(x,y,dx,dy) dx
#define JH2(x,y,dx,dy) dy
#define JH3(x,y,dx,dy) (dx*y - dy*x)
#define JH4(x,y,dx,dy) (dx*x + dy*y)
#define JH5(x,y,dx,dy) (dx*x - dy*y)
#define JH6(x,y,dx,dy) dx*y
#define JH7(x,y,dx,dy) (-dx*x*x - dy*y*x) // do not factorise (the compiler does it better!)
#define JH8(x,y,dx,dy) (-dx*x*y - dy*y*y)

// Homography Jacobians for ESM
#define JK1  JH1(xf,yf,dx,dy)
#define JK2  JH2(xf,yf,dx,dy)
#define JK3  JH3(xf,yf,dx,dy)
#define JK4  JH4(xf,yf,dx,dy)
#define JK5  JH5(xf,yf,dx,dy)
#define JK6  JH6(xf,yf,dx,dy)
#define JK7  JH7(xf,yf,dx,dy)
#define JK8  JH8(xf,yf,dx,dy)

inline int CDOF_T( const int DOF ) { return DOF*(DOF+1)/2; }

/// This class can be used optionally to handle memory buffers
/// used for tracking to avoid repeated allocation/deallocation.
class TrackBuffer {
 public:
  TrackBuffer( int nPatchWidth, int nPatchHeight, int nPatchWidthStep, bool bBlur = false ) :
      m_nPatchWidth( nPatchWidth ), m_nPatchHeight( nPatchHeight ) {
    m_pRefPatchGradX = new float[ nPatchWidth*nPatchHeight ];
    m_pRefPatchGradY = new float[ nPatchWidth*nPatchHeight ];

    m_pCurPatch      = new unsigned char[ nPatchWidthStep*nPatchHeight ];
    m_pCurPatchGradX = new float[ nPatchWidth*nPatchHeight ];
    m_pCurPatchGradY = new float[ nPatchWidth*nPatchHeight ];

    if( bBlur ) {
      m_pRefPatchWTGradX = new float[ nPatchWidth*nPatchHeight ];
      m_pRefPatchWTGradY = new float[ nPatchWidth*nPatchHeight ];
    }
    else {
      m_pRefPatchWTGradX = NULL;
      m_pRefPatchWTGradY = NULL;
    }
  }
  ~TrackBuffer() {
    delete[] m_pRefPatchGradX;
    delete[] m_pRefPatchGradY;
    delete[] m_pCurPatch;
    delete[] m_pCurPatchGradX;
    delete[] m_pCurPatchGradY;
    if( m_pRefPatchWTGradX != NULL ) {
      delete[] m_pRefPatchWTGradX;
    }
    if( m_pRefPatchWTGradY != NULL ) {
      delete[] m_pRefPatchWTGradY;
    }
  }
  float* RefPatchGradX() { return m_pRefPatchGradX; }
  float* RefPatchGradY() { return m_pRefPatchGradY; }
  float* RefPatchWTGradX() { return m_pRefPatchWTGradX; }
  float* RefPatchWTGradY() { return m_pRefPatchWTGradY; }
  unsigned char* CurPatch() { return m_pCurPatch; }
  float* CurPatchGradX() { return m_pCurPatchGradX; }
  float* CurPatchGradY() { return m_pCurPatchGradY; }
  void AllocateBlurBuffers() {
    if( m_pRefPatchWTGradX != NULL ) {
      delete[] m_pRefPatchWTGradX;
    }
    m_pRefPatchWTGradX = new float[ m_nPatchWidth*m_nPatchHeight ];
    if( m_pRefPatchWTGradY != NULL ) {
      delete[] m_pRefPatchWTGradY;
    }
    m_pRefPatchWTGradY = new float[ m_nPatchWidth*m_nPatchHeight ];
  }
 private:
  TrackBuffer( const TrackBuffer& );
  TrackBuffer& operator=( const TrackBuffer& );
 private:
  unsigned char* m_pCurPatch;
  int m_nPatchWidth;
  int m_nPatchHeight;
  float* m_pRefPatchGradX;
  float* m_pRefPatchGradY;
  float* m_pRefPatchWTGradX;
  float* m_pRefPatchWTGradY;
  float* m_pCurPatchGradX;
  float* m_pCurPatchGradY;
};

class TrackingSettings {
 public:
  /// Default constructor: should provide sensible settings
  TrackingSettings( const double dPatchBegX, const double dPatchBegY,
                    const int nPatchWidth, const int nPatchHeight,
                    const int nPatchWidthStep, bool bRegularise = false ) :
      m_dPatchBegX( dPatchBegX ), m_dPatchBegY( dPatchBegY ),
      m_bRegularise( bRegularise ),
      m_nMaxNumIterations( 30 ),
      m_TrackBuffer( nPatchWidth, nPatchHeight, nPatchWidthStep ) { }

  double BegX() { return m_dPatchBegX; }
  double BegY() { return m_dPatchBegY; }
  bool Regularise() { return m_bRegularise; }
  void MaxNumIterations( int nMaxNumIterations ) { m_nMaxNumIterations = nMaxNumIterations; }
  int MaxNumIterations() { return m_nMaxNumIterations; }
  TrackBuffer* GetTrackBuffer() { return &m_TrackBuffer; }

 private:
  double      m_dPatchBegX;
  double      m_dPatchBegY;
  bool        m_bRegularise;
  int         m_nMaxNumIterations;
  TrackBuffer m_TrackBuffer;
};

class TrackingResults {
 public:
  TrackingResults() :
      m_dAlpha( 1 ), m_dAlphaPrev( -1 ), m_dBeta( 0 ), m_dBetaPrev( 1000 ), m_dLambda( 0 ), m_dLambdaPrev( -1 ) {}

  // The tracking code uses the default copy constructor (make explicit if required).
  CTrackHomography GetHomography() { return m_H; }
  CTrackHomography GetPrevHomography() { return m_HPrev; }
  double GetAlpha() { return m_dAlpha; }
  double GetAlphaPrev() { return m_dAlphaPrev; }
  double GetBeta() { return m_dBeta; }
  double GetBetaPrev() { return m_dBetaPrev; }
  double GetBlurMagn() { return m_dLambda; }
  double GetBlurMagnPrev() { return m_dLambdaPrev; }

  void SetHomography( const CTrackHomography& mH ) { m_H = mH; }
  void SetPrevHomography( const CTrackHomography& mHPrev ) { m_HPrev = mHPrev; }
  void SetAlpha( const double& dAlpha ) { m_dAlpha = dAlpha; }
  void SetBeta( const double& dBeta ) { m_dBeta = dBeta; }
  void SetBlurMagn( const double& dLambda ) { m_dLambda = dLambda; }

  /// Copies all current values to previous values (homography, alpha, ...)
  /// AND sets the current homography to the identity
  void CopyCurToPrevIgnoringH() {
    m_dAlphaPrev  = m_dAlpha;
    m_dBetaPrev   = m_dBeta;
    m_dLambdaPrev = m_dLambda;
  }

  void id() { m_dAlpha = 1; m_dBeta = 0; m_dLambda = 0; m_H.id(); m_HPrev.id();}
  friend std::ostream& operator<< (std::ostream& o, TrackingResults const& trackingResults) {
    o << "Alpha: " << trackingResults.m_dAlpha << ", Beta: " << trackingResults.m_dBeta << ", Lambda: " << trackingResults.m_dLambda;
    return o;
  }
 private:
  double m_dAlpha, m_dAlphaPrev;
  double m_dBeta, m_dBetaPrev;
  double m_dLambda, m_dLambdaPrev;
  CTrackHomography m_H;
  CTrackHomography m_HPrev;
};

class CTrackTrackingStats {
 public:
  CTrackTrackingStats() {
    m_dFinalRMS = 0.;
    m_nNumIter  = 0;
  }
  void RMS( double dRMS ) {
    m_dFinalRMS = dRMS;
  }
  double RMS() {
    return m_dFinalRMS;
  }
  void NumIter( int nNumIter ) {
    m_nNumIter = nNumIter;
  }
 private:
  double m_dFinalRMS;
  int    m_nNumIter;
};

class ImageHolder {
 public:

  ImageHolder(const unsigned char* pImageData,
              const int nWidth,  const int nHeight,
              const int nWidthStep) :
      pImageData( pImageData ), nWidth( nWidth ), nHeight( nHeight ),
      nWidthStep( nWidthStep ) {}

  inline void print( const char* sMsg ) {
    std::cout << sMsg << std::endl;
    for( int ii=0; ii<nWidth; ii++ ) {
      for( int jj=0; jj<nHeight; jj++ ) {
        std::cout << pImageData[ii+jj*nWidthStep] << " ";
      }
      std::cout << std::endl;
    }
  }

  const unsigned char* pImageData;
  const int nWidth;
  const int nHeight;
  const int nWidthStep;

  friend void copy( const ImageHolder* pIn, ImageHolder** pOut ) {
    unsigned char* pData = (unsigned char*)malloc( pIn->nWidthStep * pIn->nHeight );
    memcpy( pData, pIn->pImageData, pIn->nWidthStep * pIn->nHeight );
    *pOut = new ImageHolder( pData, pIn->nWidth, pIn->nHeight, pIn->nWidthStep  );
  }
 private:
  ImageHolder( const ImageHolder& );
  ImageHolder& operator=( const ImageHolder& );
};

inline void BilinearInterpolation(
    const int            nWM,
    const int            nHM,
    const int            nImageWidthStep,
    const unsigned char* pImage,
    const float          fXc,
    const float          fYc,
    unsigned char*       pWarpedPatch
                                  );

inline void Gradient(
    const unsigned char* pImage,    ///< Input:
    const int      nImageWidth,     ///< Input:
    const int      nImageHeight,    ///< Input:
    const int      nImageWidthStep,
    float*         pGradX,          ///< Output:
    float*         pGradY           ///< Output:
                     )
{
  const int nImageWidthM1  = nImageWidth - 1;
  const int nImageHeightM1 = nImageHeight - 1;

  const unsigned char* pRow,*pBottomRow,*pTopRow;

  pRow         = pImage;
  pBottomRow   = pImage + nImageWidthStep;
  float* pRowX = pGradX;
  float* pRowY = pGradY;

  // Work on the first row
  pRowX[0] = pRow[1] - pRow[0];
  pRowY[0] = pBottomRow[0] - pRow[0];
  for( int nCol = 1; nCol < nImageWidthM1; ++nCol ) {
    pRowX[nCol] = (pRow[nCol+1] - pRow[nCol-1])/2;
    pRowY[nCol] = pBottomRow[nCol] - pRow[nCol];
  }
  pRowX[nImageWidthM1] = pRow[nImageWidthM1] - pRow[nImageWidthM1-1];
  pRowY[nImageWidthM1] = pBottomRow[nImageWidthM1] - pRow[nImageWidthM1];

  pRow       = pImage + nImageWidthStep;
  pBottomRow = pImage + 2*nImageWidthStep;
  pTopRow    = pImage;
  pRowX      = pGradX + nImageWidth;
  pRowY      = pGradY + nImageWidth;

  // Work from the second to the "last-1" row
  for( int nRow = 1; nRow < nImageHeightM1; ++nRow ) {
    // First column
    *pRowX++ = pRow[1] - pRow[0];
    *pRowY++ = (pBottomRow[0] - pTopRow[0])/2;

    for( int nCol = 1; nCol < nImageWidthM1; ++nCol ) {
      *pRowX++ = (pRow[nCol+1] - pRow[nCol-1])/2;
      *pRowY++ = (pBottomRow[nCol] - pTopRow[nCol])/2;
    }

    // Last column
    *pRowX++ = pRow[nImageWidthM1] - pRow[nImageWidthM1-1];
    *pRowY++ = (pBottomRow[nImageWidthM1] - pTopRow[nImageWidthM1])/2;

    // Move to next rows
    pRow       += nImageWidthStep;
    pBottomRow += nImageWidthStep;
    pTopRow    += nImageWidthStep;
  }

  // Last row
  pTopRow  = pImage + ( nImageHeightM1 - 1 ) * nImageWidthStep;
  pRow     = pImage + nImageHeightM1 * nImageWidthStep;
  pRowX    = pGradX + nImageHeightM1 * nImageWidth;
  pRowY    = pGradY + nImageHeightM1 * nImageWidth;
  pRowX[0] = pRow[1] - pRow[0];
  pRowY[0] = pRow[0] - pTopRow[0];

  for( int nCol = 1; nCol < nImageWidthM1; ++nCol ) {
    pRowX[nCol] = (pRow[nCol+1] - pRow[nCol-1])/2;
    pRowY[nCol] = pRow[nCol] - pTopRow[nCol];
  }
  pRowX[nImageWidthM1] = pRow[nImageWidthM1] - pRow[nImageWidthM1-1];
  pRowY[nImageWidthM1] = pRow[nImageWidthM1] - pTopRow[nImageWidthM1];
}

void WarpTranslation(
    const unsigned char* pImage,          ///< Input: image input
    const int            nImageWidth,     ///< Input: image width
    const int            nImageHeight,    ///< Input: image height
    const int            nImageWidthStep,
    float                dfX,             ///< Input: offset in X
    float                dfY,             ///< Input: offset in Y
    unsigned char*       pWarpedPatch,    ///< Input/Output: memory allocated by the user that will be used to store the result of the warping
    const int            nPatchWidth,     ///< Input: warped patch width
    const int            nPatchHeight,    ///< Input: warped patch height
    const int            nPatchWidthStep
                     );

void WarpNoIllumination(
    const unsigned char* pCurImage,
    const int            nImageWidth,
    const int            nImageHeight,
    const int            nImageWidthStep,
    unsigned char*       pCurPatch,
    const int            nPatchWidth,
    const int            nPatchHeight,
    const int            nPatchWidthStep,
    const double         dBegX,
    const double         dBegY,
    TrackingResults*     pTrackingResults
                        );

bool UpdateResult(
    TrackingResults* pTrackingResults,
    double*          JtJ,
    double*          JtE,
    double*          dNormUpdate
                  );

bool RSLAM_UpdateResult(
    TrackingResults* pTrackingResults,
    double*          JtJ,
    double*          JtE,
    double*          dNormUpdate
                        );

// aux for ESMJacobian
template <int DOF, int ROW, int COL>
class AddOuterProd
{
 public:
  inline static void call( double** JtJ, const double* JTmp ) {
    *(*JtJ)++ += JTmp[DOF-COL]*JTmp[DOF-ROW];
    AddOuterProd<DOF,ROW, COL - 1 >::call( JtJ, JTmp );
  }
};

template <int DOF, int ROW>
class AddOuterProd<DOF,ROW,ROW>
{
 public:
  inline static void call( double** JtJ, const double* JTmp ) {
    *(*JtJ)++ += JTmp[DOF-ROW]*JTmp[DOF-ROW];
    AddOuterProd<DOF,ROW-1, DOF >::call( JtJ, JTmp );
  }
};

template <int DOF>
class AddOuterProd<DOF,1,1>
{
 public:
  inline static void call( double** JtJ, const double* JTmp  ) {
    *(*JtJ) += JTmp[DOF-1]*JTmp[DOF-1];
  }
};

// aux for ESMJacobian
inline void AddJtE( const int nDOF, double* JtE, double* JTmp, const double dE )
{
  for( int nRow = 0; nRow < nDOF; nRow++ ) {
    *JtE++ += *JTmp++ * dE;
  }
}

void ESMJacobian(
    const unsigned char* pRefPatch,
    float*         pRefPatchGradX,
    float*         pRefPatchGradY,
    unsigned char* pCurPatch,
    float*         pCurPatchGradX,
    float*         pCurPatchGradY,
    const int      nPatchWidth,
    const int      nPatchHeight,
    const int      nPatchWidthStep,
    const int      nDOF,
    double*        JtJ, ///< Output: triangular part of J'*J, of size DOF_TRANSL_T
    double*        JtE, ///< Output: J'*(current_pixel_value - referenc_pixel_value), of size DOF_TRANSL
    double*        dRMS ///< Output: root mean square error (useful for checking decreasing error)
                 );

bool SSDUpdateSL3Motion(
    unsigned char*       pRefPatch,
    float*               pRefPatchGradX,
    float*               pRefPatchGradY,
    float*               /*pRefPatchWTGradX*/,
    float*               /*pRefPatchWTGradY*/,
    const int            nPatchWidth,
    const int            nPatchHeight,
    const int            nPatchWidthStep,
    const unsigned char* pCurImage,
    const int            nImageWidth,
    const int            nImageHeight,
    const int            nImageWidthStep,
    const double         dBegX,
    const double         dBegY,
    const bool           /*bRegularise*/,
    const int            nDOF,
    unsigned char*       pCurPatch,
    float*               pCurPatchGradX,
    float*               pCurPatchGradY,
    TrackingResults*     pTrackingResults,
    double*              dRMS,
    double*              dNormUpdate
                        );

void TrackPlaneHomography(
    TrackingSettings&  trackingSettings, ///< Input
    const ImageHolder& refPatchHolder,   ///< Input
    const ImageHolder& curImageHolder,   ///< Input
    const int          nDOF,             ///< Input
    CTrackTrackingStats*     pTrackingStats,   ///< Output:
    TrackingResults*   pTrackingResults  ///< Output:
                          );

inline float _ScorePatchesSAD(
    const unsigned char* pPatchA,       //< Input:
    const unsigned char* pPatchB,       //< Input:
    const unsigned int   nPatchWidth,     //< Input:
    const unsigned int   nPatchHeight     //< Input:
                              )
{
  unsigned int nScore = 0;
  for( unsigned int p = 0; p < nPatchWidth*nPatchHeight; p++ ){
    nScore += abs(pPatchA[p] - pPatchB[p]);
  }
  return (float)nScore/(nPatchWidth*nPatchHeight);
}

// SAD with mean subtraction
inline float _ScorePatchesMeanSAD(
    const unsigned char* pPatchA,        //< Input:
    const unsigned char* pPatchB,        //< Input:
    const unsigned int   nPatchWidth,
    const unsigned int   nPatchHeight
                                  )
{

  int nSize  = (nPatchWidth*nPatchHeight);
  int nSAD   = 0;
  int nMean  = 0;

  for( int p = 0; p < nSize; p++){
    nMean += (pPatchA[p] - pPatchB[p]);
  }

  nMean /= nSize;

  for( int p = 0; p < nSize; p++ ){
    nSAD += abs( pPatchA[p] - pPatchB[p] - nMean );
  }

  return (float)nSAD/nSize;
}


/// This version is called by RefineSubPixelXY and calls
/// RSLAM_SSDUpdateSL3Motion, which uses our LoadWarpedPatch which can
/// handle full homographies.
template <typename Scalar, unsigned int PatchSize>
inline bool RSLAM_SSDUpdateSL3Motion(
    const unsigned char*       pRefPatch,
    float*               pRefPatchGradX,
    float*               pRefPatchGradY,
    float*               /*pRefPatchWTGradX*/,
    float*               /*pRefPatchWTGradY*/,
    const unsigned char* pCurImage,
    const int            nImageWidth,
    const int            nImageHeight,
    const int            ,//nImageWidthStep,
    const double         ,//dBegX,
    const double         ,//dBegY, in homography now in trackinresults
    const bool           /*bRegularise*/,
    const int            nDOF,
    unsigned char*       pCurPatch,
    float*               pCurPatchGradX,
    float*               pCurPatchGradY,
    TrackingResults*     pTrackingResults,
    double*              dRMS,
    double*              dNormUpdate
                                     )
{

  const Eigen::Matrix<Scalar,3,3> H = pTrackingResults->GetHomography().m_mH.cast<Scalar>();
  LoadWarpedPatch<PatchSize>( pCurImage, nImageWidth, nImageHeight, H, pCurPatch );

  // float fScore = _ScorePatchesSAD( pRefPatch, pCurPatch, PatchSize, PatchSize );
  // printf("ESM pre score %f\n", fScore ); fflush(stdout);

  // Compute gradient of warped patch
  Gradient( pCurPatch,
            PatchSize, PatchSize, PatchSize,
            pCurPatchGradX, pCurPatchGradY );

  double JtJ[ CDOF_T( nDOF ) ];
  double JtE[ nDOF ];

  // Compute the Jacobians
  ESMJacobian( pRefPatch, pRefPatchGradX, pRefPatchGradY,
               pCurPatch, pCurPatchGradX, pCurPatchGradY,
               PatchSize, PatchSize, PatchSize,
               nDOF, JtJ, JtE, dRMS );

  // Compute and apply update
  bool bSuccess = RSLAM_UpdateResult( pTrackingResults,
                                      JtJ, JtE, dNormUpdate );

  // CTrackHomography newH = pTrackingResults->GetHomography();
  // printf( "RMS: %.2f, Update %.2f, %.2f\n", *dRMS, newH.m_mH(0,2), newH.m_mH(1,2) ); fflush(stdout);

  return bSuccess;
}

template <typename Scalar, unsigned int PATCHSIZE>
inline void RSLAM_TrackPlaneHomography(
    TrackingSettings&     trackingSettings, ///< Input
    const ImageHolder&    refPatchHolder,   ///< Input
    const ImageHolder&    curImageHolder,   ///< Input
    const int             nDOF,             ///< Input
    CTrackTrackingStats*  pTrackingStats,   ///< Output:
    TrackingResults*      pTrackingResults  ///< Output:
                                       ) {
  // Copy current values to previous values
  // The effect is a bit subtle when looking at the regulariser AND pyramid levels,
  // it should not have a very big effect though...
  pTrackingResults->CopyCurToPrevIgnoringH();

  const int nImageWidth     = curImageHolder.nWidth;
  const int nImageHeight    = curImageHolder.nHeight;
  const int nImageWidthStep = curImageHolder.nWidthStep;

  float* pRefPatchGradX    = trackingSettings.GetTrackBuffer()->RefPatchGradX();
  float* pRefPatchGradY    = trackingSettings.GetTrackBuffer()->RefPatchGradY();
  float* pRefPatchWTGradX  = trackingSettings.GetTrackBuffer()->RefPatchWTGradX();
  float* pRefPatchWTGradY  = trackingSettings.GetTrackBuffer()->RefPatchWTGradY();
  unsigned char* pCurPatch = trackingSettings.GetTrackBuffer()->CurPatch();
  float* pCurPatchGradX    = trackingSettings.GetTrackBuffer()->CurPatchGradX();
  float* pCurPatchGradY    = trackingSettings.GetTrackBuffer()->CurPatchGradY();

  // Compute gradient of reference patch
  Gradient( refPatchHolder.pImageData,
            PATCHSIZE, PATCHSIZE, PATCHSIZE,
            pRefPatchGradX, pRefPatchGradY );

  double dPrevRMS = 0.;
  double dBestRMS = 0.;
  double dNormUpdate = 0.;

  const int nMaxIter = trackingSettings.MaxNumIterations();
  double dMinNormUpdate = 0.1;
  int nNumIter = 0;

  TrackingResults trackingResultsBest = *pTrackingResults;
  TrackingResults trackingResultsPrev = *pTrackingResults;
  TrackingResults trackingResultsCur  = *pTrackingResults;

  bool bSuccess = RSLAM_SSDUpdateSL3Motion<Scalar,PATCHSIZE>(
      refPatchHolder.pImageData,
      pRefPatchGradX, pRefPatchGradY,
      pRefPatchWTGradX, pRefPatchWTGradY,
      curImageHolder.pImageData,
      nImageWidth, nImageHeight, nImageWidthStep,
      trackingSettings.BegX(),
      trackingSettings.BegY(),
      trackingSettings.Regularise(),
      nDOF,
      pCurPatch,
      pCurPatchGradX, pCurPatchGradY,
      &trackingResultsCur,
      &dPrevRMS, &dNormUpdate);

  if( !bSuccess ) {
    return;
  }

  dBestRMS = dPrevRMS;
  trackingResultsBest = trackingResultsPrev;
  trackingResultsPrev = trackingResultsCur;

  for( nNumIter=0; nNumIter < nMaxIter; nNumIter++ ) {

    bSuccess = RSLAM_SSDUpdateSL3Motion<Scalar,PATCHSIZE>(
        refPatchHolder.pImageData,
        pRefPatchGradX, pRefPatchGradY,
        pRefPatchWTGradX, pRefPatchWTGradY,
        curImageHolder.pImageData,
        nImageWidth, nImageHeight, nImageWidthStep,
        trackingSettings.BegX(),
        trackingSettings.BegY(),
        trackingSettings.Regularise(),
        nDOF,
        pCurPatch,
        pCurPatchGradX, pCurPatchGradY,
        &trackingResultsCur,
        &dPrevRMS, &dNormUpdate
                                                          );

    // problem computing SL3
    if( !bSuccess ) {
      break;
    }

    // root mean square error increasing!
    if( dPrevRMS > dBestRMS ) {
      break;
    }

    // error decreasing too slowly!
    if( dNormUpdate < dMinNormUpdate ) {
      if( dPrevRMS < dBestRMS ) {
        dBestRMS = dPrevRMS;
        trackingResultsBest = trackingResultsPrev;
      }
      break;
    }

    if( dPrevRMS < dBestRMS ) {
      dBestRMS = dPrevRMS;
      trackingResultsBest = trackingResultsPrev;
    }
    trackingResultsPrev = trackingResultsCur;
  }

  trackingResultsCur = trackingResultsBest;
  RSLAM_SSDUpdateSL3Motion<Scalar,PATCHSIZE>(
      refPatchHolder.pImageData,
      pRefPatchGradX, pRefPatchGradY,
      pRefPatchWTGradX, pRefPatchWTGradY,
      curImageHolder.pImageData,
      nImageWidth, nImageHeight, nImageWidthStep,
      trackingSettings.BegX(),
      trackingSettings.BegY(),
      trackingSettings.Regularise(),
      nDOF,
      pCurPatch,
      pCurPatchGradX, pCurPatchGradY,
      // Warning: this will be
      // updated (use something different than trackingResultsBest
      &trackingResultsCur,
      &dBestRMS, &dNormUpdate
                                             );

  pTrackingStats->RMS( dBestRMS );
  pTrackingStats->NumIter( nNumIter );
  *pTrackingResults = trackingResultsBest;
}
