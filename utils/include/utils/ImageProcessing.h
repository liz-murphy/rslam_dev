// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#ifndef  _IMAGEPROCESSING_H
#define _IMAGEPROCESSING_H

#include <opencv2/imgproc/imgproc.hpp>

class ImageProcessing {
public:
    ImageProcessing();
    virtual ~ImageProcessing();
    
    void DoStereoBrightnessCorrection(cv::Mat& img0,
                                      cv::Mat& img1,
                                      int *roi1=nullptr,
                                      int *roi2=nullptr);
    

};

#endif	/* IMAGEPROCESSING_H */

