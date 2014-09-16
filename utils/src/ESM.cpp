#include <utils/ESM.h>

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

#include <utils/PatchUtils.h> // Will move shortly...

void BilinearInterpolation(
    const int            nWM,
    const int            nHM,
    const int            nImageWidthStep,
    const unsigned char* pImage,
    const float          fXc,
    const float          fYc,
    unsigned char*       pWarpedPatch
                           )
{
  // Truncates the values
  const int nFloorXc = (int)fXc;
  const int nFloorYc = (int)fYc;

  if( nFloorXc >= 0 && nFloorYc >= 0 && nFloorXc < nWM && nFloorYc < nHM ) {

    // Main case: points inside the image
    int nCoord = nFloorXc + nFloorYc*nImageWidthStep;

    float fV00 = pImage[ nCoord ];
    float fV10 = pImage[ nCoord + 1  ];
    nCoord += nImageWidthStep;
    float fV01 = pImage[ nCoord ];
    float fV11 = pImage[ nCoord + 1 ];

    *pWarpedPatch = (char) ( fV00 +
                             (fXc-nFloorXc)*(fV10-fV00)+
                             (fYc-nFloorYc)*(fV01-fV00)-
                             (fXc-nFloorXc)*(fYc-nFloorYc)*(fV01-fV00+fV10-fV11) );
  }
  else {
    *pWarpedPatch     = 0;
  }
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
                     )
{
  const int nPatchPadding = nPatchWidthStep - nPatchWidth;
  const int nWM = nImageWidth - 1;
  const int nHM = nImageHeight - 1;

  // fColRef,nRowRef: values taken in the reference image
  // fXc,fYc: values taken in the current image

  float fYc = dfY;
  for( int nRowRef=0; nRowRef<nPatchHeight; nRowRef++, fYc++ ) {
    float fColRef = dfX;
    for( int nColRef=0; nColRef < nPatchWidth; nColRef++, fColRef++ ) {
      BilinearInterpolation( nWM, nHM,  nImageWidthStep,
                             pImage, fColRef, fYc,
                             pWarpedPatch
                             );

      pWarpedPatch++;
    }
    for( int kk = 0; kk < nPatchPadding; kk++ ) {
      pWarpedPatch++;
    }
  }
}

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
                        )
{
  CTrackHomography HFull;
  HFull.Set( 0, 2, dBegX );
  HFull.Set( 1, 2, dBegY );
  HFull.mult( pTrackingResults->GetHomography() );

  WarpTranslation( pCurImage,
                   nImageWidth, nImageHeight, nImageWidthStep,
                   HFull.Get(0,2),
                   HFull.Get(1,2),
                   pCurPatch,
                   nPatchWidth, nPatchHeight, nPatchWidthStep );
}

bool UpdateResult(
    TrackingResults* pTrackingResults,
    double*          JtJ,
    double*          JtE,
    double*          dNormUpdate
                  )
{
  CTrackHomography HEst = pTrackingResults->GetHomography();
  bool bSuccess = HEst.R2Update( JtJ, JtE, dNormUpdate );
  if( bSuccess ) {
    pTrackingResults->SetHomography( HEst );
  }
  return bSuccess;
}

bool RSLAM_UpdateResult(
    TrackingResults* pTrackingResults,
    double*          JtJ,
    double*          JtE,
    double*          dNormUpdate
                        )
{
  CTrackHomography HEst = pTrackingResults->GetHomography();
  bool bSuccess = HEst.RSLAM_R2Update( JtJ, JtE, dNormUpdate );
  if( bSuccess ) {
    pTrackingResults->SetHomography( HEst );
  }
  return bSuccess;
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
                 )
{
  //const int DOF = 2;
  const int nPadding = nPatchWidthStep - nPatchWidth;

  double dx,dy,xf,yf,dE;
  double JTmp[ nDOF ];
  double* JtJ_store = JtJ;
  memset( JtJ, 0, CDOF_T( nDOF )*sizeof( double ) );
  memset( JtE, 0, nDOF*sizeof( double ) );

  *dRMS = 0;

  const unsigned char* pRefMeanPatch = pRefPatch;
  unsigned char* pCurMeanPatch = pCurPatch;
  double dMean = 0;
  yf = 0.;
  for( int y=0; y<nPatchHeight; y++, yf++ ) {
    xf = 0.;
    for( int x=0; x<nPatchWidth; x++, xf++ ) {
      dMean += (double)*pCurMeanPatch++ - *pRefMeanPatch++;
    }
  }
  dMean /= (nPatchHeight*nPatchWidth);

  yf = 0.;
  for( int y=0; y<nPatchHeight; y++, yf++ ) {
    xf = 0.;
    for( int x=0; x<nPatchWidth; x++, xf++ ) {
      // We do not divide by 2 yet...
      // multiply by 2 at the end
      dx = (double)*pCurPatchGradX++ + *pRefPatchGradX++;
      dy = (double)*pCurPatchGradY++ + *pRefPatchGradY++;

      if( std::isnan( dx ) || std::isnan(dy) ) {
        pCurPatch++; pRefPatch++;
        continue;
      }

      // Compute the row 'y' of the Jacobian
      JTmp[0] = JK1;
      if(nDOF == 2){
        JTmp[1] = JK2;
      }

      // Calculate JtJ (The symmetric matrix is stored as a lower triangular matrix)
      if(nDOF == 1){
        AddOuterProd<1,1,1>::call( &JtJ, JTmp );
      }else{
        AddOuterProd<2,2,2>::call( &JtJ, JTmp );
      }

      dE = ((double)*pCurPatch++ - *pRefPatch++) - dMean;

      if( std::isnan(dE) ) {
        //cout << "******" << endl;
        //fflush(stdout);
        // This is in fact only useful
        // to help the compiler (I'm not sure why)
        // We shouldn't arrive here, the Jacobian will
        // be incorrect if we do...
        continue;
      }

      AddJtE( nDOF, JtE, JTmp, dE );

      *dRMS += dE*dE;
      JtJ = JtJ_store;
    }
    for( int ii=0; ii < nPadding; ii++ ) {
      pCurPatch++;
      pRefPatch++;
    }
  }
  for( int ii=0; ii<CDOF_T( nDOF ); ii++ ) {
    JtJ[ii] /= 4;
  }
  for( int ii=0; ii<nDOF; ii++ ) {
    JtE[ii] /= 2;
  }
  *dRMS = sqrt(*dRMS/(nPatchWidth*nPatchHeight));
}

bool SSDUpdateSL3Motion(
    const unsigned char*       pRefPatch,
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
                        )
{
  WarpNoIllumination( pCurImage,
                      nImageWidth, nImageHeight, nImageWidthStep,
                      pCurPatch,
                      nPatchWidth, nPatchHeight, nPatchWidthStep,
                      dBegX, dBegY, pTrackingResults);

  // Compute gradient of warped patch
  Gradient( pCurPatch,
            nPatchWidth, nPatchHeight, nPatchWidthStep,
            pCurPatchGradX, pCurPatchGradY );

  double JtJ[ CDOF_T( nDOF ) ];
  double JtE[ nDOF ];

  // Compute the Jacobians
  ESMJacobian
      ( pRefPatch, pRefPatchGradX, pRefPatchGradY,
        pCurPatch, pCurPatchGradX, pCurPatchGradY,
        nPatchWidth, nPatchHeight, nPatchWidthStep,
        nDOF, JtJ, JtE, dRMS );

  // Compute and apply update
  bool bSuccess = UpdateResult( pTrackingResults,
                                JtJ, JtE, dNormUpdate );

  return bSuccess;
}

void TrackPlaneHomography(
    TrackingSettings&  trackingSettings, ///< Input
    const ImageHolder& refPatchHolder,   ///< Input
    const ImageHolder& curImageHolder,   ///< Input
    const int          nDOF,             ///< Input
    CTrackTrackingStats*     pTrackingStats,   ///< Output:
    TrackingResults*   pTrackingResults  ///< Output:
                          )
{
  // Copy current values to previous values
  // The effect is a bit subtle when looking at the regulariser AND pyramid levels,
  // it should not have a very big effect though...
  pTrackingResults->CopyCurToPrevIgnoringH();

  const int nPatchWidth     = refPatchHolder.nWidth;
  const int nPatchHeight    = refPatchHolder.nHeight;
  const int nPatchWidthStep = refPatchHolder.nWidthStep;
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
            nPatchWidth, nPatchHeight, nPatchWidthStep,
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

  bool bSuccess = SSDUpdateSL3Motion
      (
          refPatchHolder.pImageData,
          pRefPatchGradX, pRefPatchGradY,
          pRefPatchWTGradX, pRefPatchWTGradY,
          nPatchWidth, nPatchHeight, nPatchWidthStep,
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

  if( !bSuccess ) {
    return;
  }

  dBestRMS = dPrevRMS;
  trackingResultsBest = trackingResultsPrev;
  trackingResultsPrev = trackingResultsCur;

  for( nNumIter=0; nNumIter < nMaxIter; nNumIter++ ) {

    bSuccess = SSDUpdateSL3Motion
        (
            refPatchHolder.pImageData,
            pRefPatchGradX, pRefPatchGradY,
            pRefPatchWTGradX, pRefPatchWTGradY,
            nPatchWidth, nPatchHeight, nPatchWidthStep,
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
  SSDUpdateSL3Motion
      ( refPatchHolder.pImageData,
        pRefPatchGradX, pRefPatchGradY,
        pRefPatchWTGradX, pRefPatchWTGradY,
        nPatchWidth, nPatchHeight, nPatchWidthStep,
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

//double C_ESM(
//        const unsigned char* pPatch,	   //< Input: Patch data
//        const unsigned int   nPatchSize,   //< Input: patch size
//        unsigned char*       pImageData,
//        unsigned int         nWidth,
//        unsigned int         nHeight,
//        unsigned int         uScale,       //< Input: Pyramid level
//        int                  nDOF,         //< Input: Degrees of Freedom
//        float&               fCol,         //< Input/Output:
//        float&               fRow          //< Input/Output:
//)
//{
//    if(nDOF < 1 || nDOF > 2 ){
//        std::cout << "WARNING, C_ESM: degrees of freedom not supported, reverting to DOF=2" << std::endl;
//        nDOF = 2;
//    }
//     // convert col-row to coordinates in corresponding the pyramid level
//    int nFactor = 1 << uScale;
//    float fU    = (fCol + 0.5)/nFactor - 0.5;
//    float fV    = (fRow + 0.5)/nFactor - 0.5;

//    //printf("esm: patch size %d  row: %f col: %f\n", nPatchSize,Col,Row);
//    int nPatchWidth = nPatchSize;
//    int nPatchHeight = nPatchSize;

//    TrackingStats    trackingStats;
//    TrackingResults  trackingResults;
//    TrackingSettings trackingSettings(
//                           fU,
//                           fV,
//                           nPatchWidth, nPatchHeight, nPatchWidth );

//    trackingSettings.MaxNumIterations( 10 );

//    const ImageHolder refPatchHolder( (unsigned char*)pPatch,
//                                        nPatchWidth,
//                                        nPatchHeight,
//                                        nPatchWidth );

//    const ImageHolder curImageHolder( pImageData, nWidth, nHeight, nWidth );

//    TrackPlaneHomography( trackingSettings,
//                          refPatchHolder,
//                          curImageHolder,
//                          nDOF,
//                          &trackingStats,
//                          &trackingResults );

//    double dCol = trackingResults.GetHomography().Get(0,2);
//    double dRow = trackingResults.GetHomography().Get(1,2);

//    fU += (float)dCol;
//    fV += (float)dRow;

//    // transform back to base level coordinates
//    fCol = ( fU + 0.5 )*nFactor - 0.5;
//    fRow = ( fV + 0.5 )*nFactor - 0.5;

//    return trackingStats.RMS();
//}
