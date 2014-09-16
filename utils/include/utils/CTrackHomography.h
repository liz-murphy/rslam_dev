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

#ifndef _HOMOGRAPHY_H
#define _HOMOGRAPHY_H

#include <iostream>
#include <utility>
#include <vector>
#include <Eigen/Eigen>


#define DOF_TRANSL         2
#define DOF_TRANSL_ILLUM   4
#define DOF_SE2            3
#define DOF_SE2_ILLUM      5
#define DOF_AFFINE         6
#define DOF_AFFINE_ILLUM   8
#define DOF_HOMOG          8
#define DOF_HOMOG_ILLUM   10

#define DOF_HOMOG_BLUR_MAGN    9


const double dTOL = 1e-6;

////////////////////////////////////////////////////////////////////////////
float randf();

////////////////////////////////////////////////////////////////////////////
/// Compute random number from Gausssian distribution, returns two
/// values at once (Box-Muller transform)
void rand_gaussf( float fMean, float fStd, float& y1, float& y2 );

////////////////////////////////////////////////////////////////////////////
/// Compute random number from Gausssian distribution, returns two
/// values at once (Box-Muller transform)
void rand_gaussf( float& y1, float& y2 );

////////////////////////////////////////////////////////////////////////////
bool SolveNormalEquations( const int nDOF, double* JtJ, double* JtE,
                            Eigen::VectorXd& vESMUpdate );

////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXd MakeFullSymmetricMatrix( const int nDOF, double* JtJ );

////////////////////////////////////////////////////////////////////////////
/// Given vESMUpdate, this function will compute
/// sum_i(vESMUpdate(i)*G_i) with {G_i,i=1..8} a basis (generators) of sl3.
Eigen::Matrix3d MakeSumGenerators( const Eigen::VectorXd& vESMUpdate );

////////////////////////////////////////////////////////////////////////////
Eigen::Matrix3d cexpm_series( const Eigen::Matrix3d& mM );

////////////////////////////////////////////////////////////////////////////
// Cheap expm computation (not exact...)
Eigen::Matrix3d cexpm( const Eigen::Matrix3d& mM );

////////////////////////////////////////////////////////////////////////////
/// This function computes the SL3 matrix from a vector of sl3.
/// i.e. exp( sum_i( vESMUpdate(i)*G_i )
/// If vESMUpdate does not have 8 values, only the
/// vESMUpdate.size() first generators will be used.
Eigen::Matrix3d ComputeSL3Update( const Eigen::VectorXd& vESMUpdate );

////////////////////////////////////////////////////////////////////////////
class CTrackHomography;
CTrackHomography GenerateScaled( double dScale );

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
class CTrackHomography {
public:
    ////////////////////////////////////////////////////////////////////////
    CTrackHomography() {
        m_mH = Eigen::Matrix3d::Identity();
#ifndef EIGEN_DEFAULT_TO_ROW_MAJOR
        m_mHt = Eigen::Matrix3d::Identity();
#endif
    }

    ////////////////////////////////////////////////////////////////////////
    CTrackHomography( const Eigen::Matrix3d& mH ) {
        m_mH = mH;
    }

    ////////////////////////////////////////////////////////////////////////
    CTrackHomography( std::vector<double>& vLieValues ) {
        Eigen::Map<Eigen::VectorXd> vESMUpdate( &vLieValues[0], vLieValues.size() );
        m_mH = ComputeSL3Update( vESMUpdate );
    }

    ////////////////////////////////////////////////////////////////////////
    ~CTrackHomography() {
    }

    ////////////////////////////////////////////////////////////////////////////
    /// Writes a homography to a flow
    inline friend std::ostream& operator<< ( std::ostream& stream, 
                                        const CTrackHomography& H ) {
        stream << H.m_mH << std::endl;
        return stream;
    }

    ////////////////////////////////////////////////////////////////////////////
    /// Reads a homography from a flow
    inline friend std::istream& operator>> ( std::istream& stream, 
                                        CTrackHomography& H ) {
        double h1, h2, h3, h4, h5, h6, h7, h8, h9;
        h1 = h2 = h3 = h4 = h5 = h6 = h7 = h8 = h9 = 0.0;
        stream >> h1 >> h2 >> h3 >> h4 >> h5 >> h6 >> h7>> h8 >> h9;
        H.m_mH << h1, h2, h3, h4, h5, h6, h7, h8, h9;
        return stream;
    }

    ////////////////////////////////////////////////////////////////////////
    inline void id() {
        m_mH = Eigen::Matrix3d::Identity();
    }

    ////////////////////////////////////////////////////////////////////////
    inline void zero() {
        m_mH = Eigen::Matrix3d::Zero();
    }

    ////////////////////////////////////////////////////////////////////////
    inline void mult( const CTrackHomography& H ) {
        m_mH *= H.m_mH;
    }

    ////////////////////////////////////////////////////////////////////////
    inline void mult( const double dScalar ) {
        m_mH *= dScalar;
    }

    ////////////////////////////////////////////////////////////////////////
    /// Axis-angle update
    inline void rot( const double d1, const double d2, const double d3 ) {
        Eigen::Matrix<double,3,1> vESMUpdate;
        vESMUpdate[0] = d1;
        vESMUpdate[1] = d2;
        vESMUpdate[2] = d3;
        m_mH *= ComputeSL3Update( vESMUpdate );
    }

    ////////////////////////////////////////////////////////////////////////
    inline void rot( const double dAngle ) {
        Eigen::Matrix<double,3,1> vESMUpdate;
        vESMUpdate[0] = 0;
        vESMUpdate[1] = 0;
        vESMUpdate[2] = dAngle/180.*3.14;
        m_mH *= ComputeSL3Update( vESMUpdate );
    }

    ////////////////////////////////////////////////////////////////////////
    /// Replace homography H with:  H(cx/2,cy/2)*H*H(-cx/2,-cy/2)
    inline void center( const double dCx, const double dCy ) {
        Eigen::Matrix3d mHC = Eigen::Matrix3d::Identity();
        mHC( 0, 2 ) = dCx/2;
        mHC( 1, 2 ) = dCy/2;
        Eigen::Matrix3d mHCInv = Eigen::Matrix3d::Identity();
        mHCInv( 0, 2 ) = -dCx/2;
        mHCInv( 1, 2 ) = -dCy/2;

        m_mH = mHC * m_mH * mHCInv;
    }

    ////////////////////////////////////////////////////////////////////////
    /// Generate a random matrix
    /// WARNING: if the generators are changes, this function has to be changed
    inline void rand( const double dMeanTx, const double dStdTx, ///< Input: translation in x
                const double dMeanTy, const double dStdTy, ///< Input: translation in y
                const double dMeanRz, const double dStdRz, ///< Input: inplane rotation (rotation around z)
                const double dMeanS,  const double dStdS,  ///< Input: exp-scale
                const double dMeanA,  const double dStdA,  ///< Input: exp-aspect ratio
                const double dMeanSh, const double dStdSh, ///< Input: shear
                const double dMeanRy, const double dStdRy, ///< Input: part of out-of-plane rotation around y
                const double dMeanRx, const double dStdRx  ///< Input: part of out-of-plane rotation around y
                ) {
        Eigen::Matrix<double,8,1> vESMUpdate;

        float dTmp, dSample;
        // Draw samples and fill SL3 update vector (i.e. lie algebra term)
        rand_gaussf( dMeanTx, dStdTx, dSample, dTmp );
        vESMUpdate[0] = dSample;
        rand_gaussf( dMeanTy, dStdTy, dSample, dTmp );
        vESMUpdate[1] = dSample;
        rand_gaussf( dMeanRz, dStdRz, dSample, dTmp );
        vESMUpdate[2] = dSample;
        rand_gaussf( dMeanS, dStdS, dSample, dTmp );
        vESMUpdate[3] = dSample;
        rand_gaussf( dMeanA, dStdA, dSample, dTmp );
        vESMUpdate[4] = dSample;
        rand_gaussf( dMeanSh, dStdSh, dSample, dTmp );
        vESMUpdate[5] = dSample;
        rand_gaussf( dMeanRy, dStdRy, dSample, dTmp );
        vESMUpdate[6] = dSample;            
        rand_gaussf( dMeanRx, dStdRx, dSample, dTmp );
        vESMUpdate[7] = dSample;            

        id();
        m_mH *= ComputeSL3Update( vESMUpdate );
    }

    ////////////////////////////////////////////////////////////////////////
    /// Matrix will become inv(Hs)*H*Hs with Hs a scaling matrix
    /// defined by dScale.
    inline CTrackHomography scale( double dScale ) {
        Eigen::Matrix3d MScaleDown = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d MScaleUp = Eigen::Matrix3d::Identity();
        MScaleDown(0,0) = 1./dScale;
        MScaleDown(1,1) = 1./dScale;
        MScaleUp(0,0)   = dScale;
        MScaleUp(1,1)   = dScale;
        m_mH = MScaleDown*m_mH*MScaleUp;
        return *this; // return a copy
    }

    ////////////////////////////////////////////////////////////////////////
    void inv();

    ////////////////////////////////////////////////////////////////////////
    void expm();

    ////////////////////////////////////////////////////////////////////////
    void logm();

    ////////////////////////////////////////////////////////////////////////
    template<class T1, class T2>
    inline  std::pair<T2,T2> warp( const std::pair<T1,T1>& pIn ) const {
        Eigen::Vector3d vIn( pIn.first, pIn.second, 1 );
        Eigen::Vector3d vOut = m_mH*vIn;
        return std::pair<T2,T2>( vOut(0)/vOut(2), 
                                    vOut(1)/vOut(2) );
    }

    ////////////////////////////////////////////////////////////////////////
    template<class T1, class T2>
    inline void warp_polymult( const std::vector<std::pair<T1,T1> >& vPolyIn,
                        std::vector<std::pair<T2,T2> >& vPolyOut
                        ) const {
        vPolyOut.clear();
        for( size_t ii=0; ii<vPolyIn.size(); ii++ ) {
            Eigen::Vector3d vIn( vPolyIn[ii].first, vPolyIn[ii].second, 1 );
            Eigen::Vector3d vOut = m_mH*vIn;
            vPolyOut.push_back( std::pair<T2,T2>( vOut(0)/vOut(2), 
                                                    vOut(1)/vOut(2) ) );
            //std::cout << vPolyOut[ii].first << " " << vPolyOut[ii].second << std::endl;
        }
    }

    ////////////////////////////////////////////////////////////////////////
    inline void warp_polymult_exp(
        const std::vector<std::pair<double,double> > &vPolyIn,
        std::vector<std::pair<double,double> >       &vPolyOut
                        ) const {
        vPolyOut.clear();
        for( size_t ii=0; ii<vPolyIn.size(); ii++ ) {
            Eigen::Vector3d vIn( vPolyIn[ii].first, vPolyIn[ii].second, 1.0);
            Eigen::Vector3d vOut = cexpm( m_mH )*vIn;
            vPolyOut.push_back( std::pair<double,double>( vOut(0)/vOut(2), 
                                                          vOut(1)/vOut(2) ) );
            //std::cout << vPolyOut[ii].first << " " << vPolyOut[ii].second << std::endl;
        }
    }

    ////////////////////////////////////////////////////////////////////////
    inline void Set( const int nX, const int nY, const double dValue ) {
        m_mH( nX,  nY ) = dValue;
    }

    ////////////////////////////////////////////////////////////////////////
    template<class T>
    inline    void set( T* mH ) {
        for( size_t ii=0; ii<3; ii++ ) {
            for( size_t jj=0; jj<3; jj++ ) {
                m_mH( ii, jj ) = mH[ 3*ii + jj ];
            }
        }
    }

    ////////////////////////////////////////////////////////////////////////
    double Get( const int nX, const int nY ) const {
        return m_mH( nX,  nY );
    }

    ////////////////////////////////////////////////////////////////////////
    /// Returns a row-major representation of the matrix
    const double* GetRowMajorPtr() {
        //#if EIGEN_DEFAULT_TO_ROW_MAJOR
        if( Eigen::Matrix3d::Base::IsRowMajor ) {
            return const_cast<double*>( m_mH.data() );
        }
        else {
            m_mHt = m_mH.transpose();
            return const_cast<double*>( m_mHt.data() );
        }
    }

    ////////////////////////////////////////////////////////////////////////
    inline void print() const {
        std::cout << m_mH << std::endl;
    }

    ////////////////////////////////////////////////////////////////////////
    inline void print( const char * sMsg ) const {
        std::cout << sMsg << std::endl << m_mH << std::endl;
    }

    ////////////////////////////////////////////////////////////////////////
    bool R2Update( double* JtJ,         ///< Input
                    double* JtE,        ///< Input
                    double* dNormUpdate ///< Output
                    ) {

        double dDet = JtJ[0]*JtJ[2] - JtJ[1]*JtJ[1];

        if( dDet>0 && JtJ[0]>=0 && JtJ[2]>=0 ) {
            dDet = -1/dDet;

            double dUpdateX = ( JtJ[2]*JtE[0] - JtJ[1]*JtE[1])*dDet;
            double dUpdateY = (-JtJ[1]*JtE[0] + JtJ[0]*JtE[1])*dDet;

            *dNormUpdate = sqrt( dUpdateX*dUpdateX + dUpdateY*dUpdateY );

            m_mH(0,2) += dUpdateX;
            m_mH(1,2) += dUpdateY;
            return true;
        } 
        else {
            return false;
        }
    }

    ////////////////////////////////////////////////////////////////////////
    bool RSLAM_R2Update(
            double* JtJ,         ///< Input
            double* JtE,        ///< Input
            double* dNormUpdate ///< Output
                    ) {

        double dDet = JtJ[0]*JtJ[2] - JtJ[1]*JtJ[1];

        if( dDet > 0.0 && JtJ[0] >= 0.0 && JtJ[2] >= 0.0 ) {
            dDet = -1.0/dDet;

            double dUpdateX = ( JtJ[2]*JtE[0] - JtJ[1]*JtE[1])*dDet;
            double dUpdateY = (-JtJ[1]*JtE[0] + JtJ[0]*JtE[1])*dDet;

            *dNormUpdate = sqrt( dUpdateX*dUpdateX + dUpdateY*dUpdateY );

            Eigen::Vector2d dUpdate(dUpdateX,dUpdateY);
            Eigen::Matrix3d Hupdate = Eigen::Matrix3d::Identity();
            Hupdate.block<2,1>(0,2) = dUpdate;
            m_mH = m_mH * Hupdate;

            return true;
        } else {
            return false;
        }
    }

    ////////////////////////////////////////////////////////////////////////
    bool IsPureTranslation() {
        return std::abs( m_mH(0,0)-1) < dTOL &&
            std::abs( m_mH(1,1)-1) < dTOL &&
            std::abs( m_mH(2,2)-1) < dTOL &&
            std::abs( m_mH(0,1) ) < dTOL &&
            std::abs( m_mH(1,0) ) < dTOL &&
            std::abs( m_mH(2,0) ) < dTOL &&
            std::abs( m_mH(2,1) ) < dTOL;
    }


    ////////////////////////////////////////////////////////////////////////
    Eigen::Matrix3d m_mH;
private:
//#ifndef EIGEN_DEFAULT_TO_ROW_MAJOR
    Eigen::Matrix3d m_mHt;
//#endif
};

#endif
