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


#include <utils/CTrackHomography.h>
#include <stdio.h>
#include <assert.h>
#include <Eigen/Cholesky>
#include <Eigen/LU>

inline double log2(double x)
{
    return (log(x) / log(2.0));
}

////////////////////////////////////////////////////////////////////////////////
bool csqrtm(
       const Eigen::MatrixXd& M, 
       Eigen::MatrixXd& sqrtM, 
       double dMaxErr
     )
{
    const int MAX_ITER_ROOT = 1000;
    
    typedef double T;
    int nN = M.rows();
    assert( nN == M.rows() && nN == M.cols() );
    assert( nN == sqrtM.rows() && nN == sqrtM.cols() );

    T norm_Res;

    int iter = 0;
    Eigen::MatrixXd Zk( nN, nN );
    Eigen::MatrixXd V( nN, nN );
    Eigen::MatrixXd W( nN, nN );
    Eigen::MatrixXd Res( nN, nN );
    Eigen::MatrixXd I( nN, nN );
    Eigen::MatrixXd I3( nN, nN );
    I.setIdentity();
    I3.setIdentity();
    I3 *= 3.;
    Zk.setIdentity();

    // Initialise values
    T scale = M.lpNorm<Eigen::Infinity>();
    sqrtM = M/scale;
    Zk /= scale;
    Res = sqrtM*sqrtM-M;
    norm_Res = Res.lpNorm<Eigen::Infinity>();

    while((iter++<MAX_ITER_ROOT)&&(norm_Res>dMaxErr)) {
        //M = (I*3+Zk*sqrtM)*(I+(Zk*sqrtM)*3).inverse();
        W = Zk*sqrtM;
        V = (I3+W)*(I+W*3).inverse();
        sqrtM *= V;
        Zk = V*Zk;
  
        Res = sqrtM*sqrtM-M;
        norm_Res = Res.lpNorm<Eigen::Infinity>();
    }

    if(norm_Res>dMaxErr) {
        std::cout << "Error: " << norm_Res << std::endl;
        std::cout << "Iter: " << iter << std::endl;
    } 

    if(norm_Res>dMaxErr)
        return false;

    return true;
}

////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXd csqrtm( const Eigen::MatrixXd& M )
{
    Eigen::MatrixXd sqrtM( M.rows(), M.cols() );
    csqrtm( M, sqrtM, 1e-10 );
    return sqrtM;
}

////////////////////////////////////////////////////////////////////////////////
bool crootm( 
        const Eigen::MatrixXd& M,
        Eigen::MatrixXd& rootM,
        double dMaxErr,
        int nP = 2
      )
{
    const int MAX_ITER_ROOT = 1000;

    typedef double T;
    assert( nP > 0 );

    if( nP == 1 ) {
        rootM = M;
        return true;
    }

    double dLog2p = 
#if !defined(_WIN32) && !defined(_WIN64)
        // no such thing in windows...
        log2( nP );
#else
    log( nP )/log((T)2);
#endif
    
    if(ceil(dLog2p)!=dLog2p) {
        std::cout << "Matrix square currently only works with " <<
            "powers of 2 because it's simpler to calculate the power of a matrix..." << std::endl;
        assert(ceil(dLog2p)==dLog2p);
    }

    int nN = M.rows();
    assert( nN == M.rows() && nN == M.cols() );
    assert( nN == rootM.rows() && nN == rootM.cols() );

    bool bEven = fmod( nP, 2 )==0;

    Eigen::MatrixXd Yk, V, Xk;
    Eigen::MatrixXd sqrtM = Eigen::MatrixXd::Identity( nN, nN );
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity( nN, nN );

    // Initialise values
    T invscale;
    T invp = (T)(1./nP);
    /*
     * If even start by calculating the
     * square root for better stability
     */
    if( bEven ) {
        if( !csqrtm( M, sqrtM, dMaxErr ) )
            return false;

        if( nP == 2 ) {
            rootM = sqrtM;
            return true;
        }      

        nP /= 2;
        invp *= 2;
        dLog2p--;

        //invscale = 1/Yk.operatorInfNorm();
        invscale = 1/sqrtM.norm();

        Yk = sqrtM*pow( invscale, nP ); 
        rootM = I*invscale;
        Xk = rootM;
    } else {
        invscale = 1/M.lpNorm<Eigen::Infinity>();
        Yk = M*pow( invscale, nP ); 
        rootM = I*invscale;
        Xk = rootM;
    }
  
    double dIncrVal = 1;
    int nIter = 0;

    while((nIter++<MAX_ITER_ROOT)&&(dIncrVal>1e-14)) {
        V = ( (nP+1)*I-Yk )*invp;
        Xk = Xk*V;

        // Calculate V^2^log2(p);
        // This should be changed to enable
        // roots that are not powers of 2
        for( int ii=0; ii<dLog2p; ii++ )
            V *= V;

        Yk = V*Yk;
  
        // Only stop iterations when the value does
        // not change. This is not a good stopping 
        // policy as we could be far from the correct
        // value and not detect it... FIX ME - HOW?
        // (calculating the product is expensive)
        dIncrVal = (Xk-rootM).lpNorm<Eigen::Infinity>();
        rootM = Xk;
    }

    if( bEven ) {
        // Calculate rootM^(p-1)= rootM^p*rootM^{-1}
        V = rootM;

        // Calculate V^2^log2(p);
        // This should be changed to enable
        // roots that are not powers of 2
        for( int ii=0; ii<dLog2p; ii++ )
            V *= V;

        V *= rootM.inverse();

        rootM = sqrtM*V;
    }

    // Expensive final test...
    V = rootM;
    if( bEven ) {
        for( int ii=0; ii<dLog2p+1; ii++ )
            V *= V;
    } else {
        for( int ii=0; ii<dLog2p; ii++ )
            V *= V;
    }

    dIncrVal = (M-V).lpNorm<Eigen::Infinity>();

    if( dIncrVal>dMaxErr ) {
        std::cout << "Error: " << dIncrVal << std::endl;
        std::cout << "Iter: " << nIter << std::endl;
    }

    return dIncrVal < dMaxErr;
}

////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXd crootm( const Eigen::MatrixXd& M, int nP = 2)
{
    Eigen::MatrixXd rootM( M.rows(), M.cols() );
    crootm( M, rootM, 1e-10, nP );
    return rootM;
}


////////////////////////////////////////////////////////////////////////////////
bool clogm( const Eigen::MatrixXd& M, 
           Eigen::MatrixXd& logM,
           double dMaxErr
         )
{
    const int MAX_ITER_LOG = 10000;
    
    typedef double T;
    int nN = M.rows();
    assert( nN == M.rows() && nN == M.cols() );
    assert( nN == logM.rows() && nN == logM.cols() );

    Eigen::MatrixXd dM = Eigen::MatrixXd::Identity( nN, nN );
    Eigen::MatrixXd K;

    T k = (T)ceil(log(log((double) 2.*M.trace()/nN))/log(2.0));
    T two_k = 1;
    if( k < 0 ) {
        two_k = 1;        
        K = M;
    }
    else {
        T two_k = (T) (1 << (int)k);
        
        // Calculate pth root of X
        K = crootm( M, two_k );
    }

    int nIter = 0;

    // Check the norm
    while( ( nIter++ < MAX_ITER_LOG ) && ( K.lpNorm<Eigen::Infinity>()>nN ) ) {
        K = crootm( K );
        k++;
        two_k *= 2.;
    }

    if( K.lpNorm<Eigen::Infinity>() > nN ) {
        std::cerr << "Problem scaling matrix before logm, norm stays big!!!" << std::endl;
        assert( K.lpNorm<Eigen::Infinity>() <= nN );
    }

    dM.setIdentity();
    dM = K-dM;
    K = dM;

    logM = K;

    nIter = 0;
    T norm_Res = 1e10;

    T val = 1;
    T denom = 1;

    // log(X)=log(I+K) = K-K^2/2+K^3/3 ...
    while( ( nIter++ < MAX_ITER_LOG ) && ( norm_Res>dMaxErr ) ) {
        dM *= K;
        denom++;
        val = -val;

        logM += val*dM/denom;

        norm_Res = dM.lpNorm<Eigen::Infinity>()/denom;
    }

    if( norm_Res > dMaxErr ) {
        std::cout << "WARNING: matrix log unsuccessful residual error: " << norm_Res << std::endl;
        std::cout << "Iter: " << nIter << std::endl;
    } 

    if( norm_Res > dMaxErr )
        return false;

    logM = two_k*logM;

    return true;
}

////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXd clogm( const Eigen::MatrixXd& M, bool* pSuccess = NULL )
{
    Eigen::MatrixXd logM( M.rows(), M.cols() );
  
    bool bRetVal = clogm( M, logM, 1e-10 );
 
    if( pSuccess != NULL ) {
        *pSuccess = bRetVal;
    }

    return logM;
}

////////////////////////////////////////////////////////////////////////////
bool SolveNormalEquations( const int nDOF, double* JtJ, double* JtE,
                            Eigen::VectorXd& vESMUpdate ) {
    Eigen::MatrixXd mJtJ = MakeFullSymmetricMatrix( nDOF, JtJ );
    //std::cout << "mJtJ: " << mJtJ << std::endl;
    Eigen::Map<Eigen::VectorXd> vJtE( JtE, nDOF );
    //std::cout << "mJtE: " << vJtE << std::endl;
    vJtE = -vJtE;
    //bool bSuccess = 
    //    mJtJ.ldlt().solve( vJtE, &vESMUpdate ); 
    bool bSuccess = true;
    vESMUpdate = mJtJ.ldlt().solve( vJtE );

    vJtE = -vJtE;
    return bSuccess;
}

////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXd MakeFullSymmetricMatrix( const int nDOF, double* JtJ ) {
    Eigen::MatrixXd mJtJ( nDOF, nDOF );
    double* pJtJ = NULL;
    // Fill the lower triangular part
    pJtJ = JtJ;
    for( int ii=0; ii<nDOF; ii++ ) {
        for( int jj=0; jj<=ii; jj++ ) {
            mJtJ( ii, jj ) = *pJtJ++;
        }
    }
    // Fill the upper triangular part
    pJtJ = JtJ;
    for( int jj=0; jj<nDOF; jj++ ) {
        for( int ii=0; ii<=jj; ii++ ) {
            mJtJ( ii, jj ) = *pJtJ++;
        }
    }
    return mJtJ;
}

////////////////////////////////////////////////////////////////////////////
Eigen::Matrix3d cexpm_series( const Eigen::Matrix3d& mM ) {
    Eigen::Matrix3d mExpM = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d mTerm = mM;
    const double dTol = 1e-10;
    const double dMaxIter = 100.;
    for( double ii=2.; ii < dMaxIter; ii++ ) {
        mExpM += mTerm;
        mTerm = (mTerm*mM)/ii;
        if( mTerm.lpNorm<Eigen::Infinity>() < dTol ) {
            mExpM += mTerm;
            break;
        }
    }
    return mExpM;
}

////////////////////////////////////////////////////////////////////////////
Eigen::Matrix3d cexpm( const Eigen::Matrix3d& mM ) {
    // Scale the matrix by 2^n for better numerical stability
    const double dScale = log2( mM.lpNorm<Eigen::Infinity>() ); // divide by two as we later take the sqrt
    int nPow = std::max(0,(int)ceil(dScale));
    Eigen::Matrix3d mExpM = cexpm_series( mM/(1<<nPow) );
    for( int ii=0; ii<nPow; ii++ ){
        mExpM = mExpM*mExpM;
    }
    return mExpM;
}

////////////////////////////////////////////////////////////////////////////
Eigen::Matrix3d ComputeSL3Update( const Eigen::VectorXd& vESMUpdate ) {
    Eigen::VectorXd vFullESMUpdate( DOF_HOMOG );
    for( int ii=0; ii<vESMUpdate.size(); ii++ ) {
        vFullESMUpdate[ii] = vESMUpdate[ii];
    }
    for( int ii=vESMUpdate.size(); ii<DOF_HOMOG; ii++ ) {
        vFullESMUpdate[ii] = 0;
    }
    Eigen::Matrix3d mGUpdate = MakeSumGenerators( vFullESMUpdate );
    return cexpm( mGUpdate );
}

////////////////////////////////////////////////////////////////////////////
Eigen::Matrix3d MakeSumGenerators( const Eigen::VectorXd& vESMUpdate ) {
    //std::cout << "vESMUpdate: " << std::endl << vESMUpdate << std::endl;
    Eigen::Matrix3d mGUpdate;
    mGUpdate << 
        vESMUpdate[3]/3+vESMUpdate[4], vESMUpdate[2]+vESMUpdate[5], vESMUpdate[0], 
        -vESMUpdate[2], vESMUpdate[3]/3-vESMUpdate[4], vESMUpdate[1], 
        vESMUpdate[6], vESMUpdate[7], -2*vESMUpdate[3]/3;
    //std::cout << "mGUpdate: " << std::endl << mGUpdate << std::endl;
    return mGUpdate;
}

////////////////////////////////////////////////////////////////////////////
CTrackHomography GenerateScaled( double dScale ) {
    CTrackHomography HScaled;
    HScaled.Set( 0, 0, dScale );
    HScaled.Set( 1, 1, dScale );
#if 0
    HScaled.Set( 0, 2, 0.5 );
    HScaled.Set( 1, 2, 0.5 );
#endif
    return HScaled;
}

////////////////////////////////////////////////////////////////////////////
float randf() {
    return (float)rand()/RAND_MAX;
}

////////////////////////////////////////////////////////////////////////////
void rand_gaussf( float fMean, float fStd, float& y1, float& y2 ) {
    rand_gaussf( y1, y2 );
    y1 = fMean + y1*fStd;
    y2 = fMean + y2*fStd;
}

////////////////////////////////////////////////////////////////////////////
void rand_gaussf( float& y1, float& y2 ) {
    float x1, x2, w;
    do {
        x1 = 2.0 * randf() - 1.0;
        x2 = 2.0 * randf() - 1.0;
        w = x1 * x1 + x2 * x2;
    } while ( w >= 1.0 );

    w = sqrt( (-2.0 * log( w ) ) / w );
    y1 = x1 * w;
    y2 = x2 * w;        
}

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
void CTrackHomography::inv() {
    m_mH = m_mH.inverse().eval();
}

////////////////////////////////////////////////////////////////////////////
void CTrackHomography::expm() {
        m_mH = cexpm( m_mH );
}

////////////////////////////////////////////////////////////////////////
void CTrackHomography::logm() {
    m_mH = clogm( m_mH );
}







