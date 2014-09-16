// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#ifndef _LOG_INFO_H_
#define _LOG_INFO_H_

struct StereoLogInfo
{
    std::string   m_sLeftFile;
    std::string   m_sRightFile;
    std::string   m_sLogDirFile;
    std::string   m_sLeftCameraModel;
    std::string   m_sRightCameraModel;
    int           m_nStartFrame;

    inline std::ostream& operator<<( std::ostream& Stream, StereoLogInfo& rhs )
    {
//        Stream << " ";
        return Stream;
    }


};

#endif

