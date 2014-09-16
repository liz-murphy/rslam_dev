// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#include <utils/ImageProcessing.h>

/////////////////////////////////////////////////////////////////////////////////////////
ImageProcessing::ImageProcessing() {}

/////////////////////////////////////////////////////////////////////////////////////////
ImageProcessing::~ImageProcessing() {}

/////////////////////////////////////////////////////////////////////////////////////////
// Adjusts the mean and std deviation of the first image to match the second.
// The mean and std of the images are computed from the give ROIs.
// If the ROIs are empty the images' dims are used.
void ImageProcessing::DoStereoBrightnessCorrection(
        cv::Mat& img1,
        cv::Mat& img2,
        int* roi1, // [top left bottom right]
        int* roi2  // [top left bottom right]
        )
{

    unsigned char *pData1     = img1.data;
    unsigned char *pData2     = img2.data;
    const int     nSampleStep = 1;
    const int     nImageSize  = img1.cols*img1.rows;
    int           nSamples    = 0;

    // compute mean
   float fMean1  = 0.0;
   float fMean2  = 0.0;
   float fMean12 = 0.0;
   float fMean22 = 0.0;

    if(roi1){
        for( int row=0; row < img1.rows; row+=nSampleStep){
            pData1 = &img1.data[row*img1.cols];
            for( int col=0; col < img1.cols; col+=nSampleStep, pData1+=nSampleStep){
                if( row >= roi1[0] && row < roi1[2] && col >= roi1[1] && col < roi1[3] ){
                    fMean1  += (*pData1);
                    fMean12 += (*pData1) * (*pData1);
                    nSamples++;
                }
            }
        }
    }else{
        for(int ii=0; ii<nImageSize; ii+=nSampleStep, pData1+=nSampleStep) {
            fMean1  += (*pData1);
            fMean12 += (*pData1) * (*pData1);
            nSamples++;
        }
    }

    fMean1  /= nSamples;
    fMean12 /= nSamples;
    nSamples = 0;

    if(roi2){
        for( int row=0; row < img2.rows; row+=nSampleStep){
            pData2 = &img2.data[row*img2.cols];
            for( int col=0; col < img2.cols; col+=nSampleStep, pData2+=nSampleStep){
                if( row >= roi2[0] && row < roi2[2] && col >= roi2[1] && col < roi2[3] ){
                    fMean2  += (*pData2);
                    fMean22 += (*pData2) * (*pData2);
                    nSamples++;
                }
            }
        }
    }else{
        for(int ii=0; ii<nImageSize; ii+=nSampleStep, pData2+=nSampleStep) {
            fMean2  += (*pData2);
            fMean22 += (*pData2) * (*pData2);
            nSamples++;
        }
    }

    fMean2  /= nSamples;
    fMean22 /= nSamples;

    // compute std
    float fStd1 = sqrt(fMean12 - fMean1*fMean1);
    float fStd2 = sqrt(fMean22 - fMean2*fMean2);

    // mean diff;
    //float mdiff = mean1 - mean2;
    // std factor
    float fRatio = fStd2/fStd1;
    // reset pointer
    pData1 = img1.data;

    int nMean1 = (int)fMean1;
    int nMean2 = (int)fMean2;

    for(int ii=0; ii < nImageSize; ++ii) {

        float tmp = (float)( pData1[ii] - nMean1 )*fRatio + nMean2;
        if(tmp < 0)  tmp = 0;
        if(tmp > 255) tmp = 255;
        pData1[ii] = (unsigned char)tmp;
    }
}
