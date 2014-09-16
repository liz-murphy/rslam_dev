// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.


#pragma once

#include <Eigen/Core>
#include <Common/scalar.h>

namespace rslam
{

inline Eigen::Matrix<Scalar,4,4> Cart2Tf(
        Scalar x,
        Scalar y,
        Scalar z,
        Scalar r,
        Scalar p,
        Scalar q
        )
{
    Eigen::Matrix<Scalar,4,4> T;
    // psi = roll, th = pitch, phi = yaw
    float cq, cp, cr, sq, sp, sr;
    cr = cos( r );
    cp = cos( p );
    cq = cos( q );

    sr = sin( r );
    sp = sin( p );
    sq = sin( q );

    T(0,0) = cp*cq;
    T(0,1) = -cr*sq+sr*sp*cq;
    T(0,2) = sr*sq+cr*sp*cq;

    T(1,0) = cp*sq;
    T(1,1) = cr*cq+sr*sp*sq;
    T(1,2) = -sr*cq+cr*sp*sq;

    T(2,0) = -sp;
    T(2,1) = sr*cp;
    T(2,2) = cr*cp;

    T(0,3) = x;
    T(1,3) = y;
    T(2,3) = z;
    T.row(3) = Eigen::Matrix<Scalar,1,4>( 0.0, 0.0, 0.0, 1.0 );
    return T;
}

inline Eigen::Matrix<Scalar,4,4> Cart2T(
        Scalar x,
        Scalar y,
        Scalar z,
        Scalar r,
        Scalar p,
        Scalar q
        )
{
    Eigen::Matrix<Scalar,4,4> T;
    // psi = roll, th = pitch, phi = yaw
    Scalar cq, cp, cr, sq, sp, sr;
    cr = cos( r );
    cp = cos( p );
    cq = cos( q );

    sr = sin( r );
    sp = sin( p );
    sq = sin( q );

    T(0,0) = cp*cq;
    T(0,1) = -cr*sq+sr*sp*cq;
    T(0,2) = sr*sq+cr*sp*cq;

    T(1,0) = cp*sq;
    T(1,1) = cr*cq+sr*sp*sq;
    T(1,2) = -sr*cq+cr*sp*sq;

    T(2,0) = -sp;
    T(2,1) = sr*cp;
    T(2,2) = cr*cp;

    T(0,3) = x;
    T(1,3) = y;
    T(2,3) = z;
    T.row(3) = Eigen::Matrix<Scalar,1,4>( 0.0, 0.0, 0.0, 1.0 );
    return T;
}

inline Eigen::Matrix<Scalar,4,4> Cart2T( Eigen::Matrix<Scalar,6,1> x)
{
    return Cart2T( x(0), x(1), x(2), x(3), x(4), x(5) );
}

inline Eigen::Matrix<Scalar,3,1> R2Cart(
        const Eigen::Matrix<Scalar,3,3>& R
        )
{
    Eigen::Matrix<Scalar,3,1> rpq;
    // roll
    rpq[0] = atan2( R(2,1), R(2,2) );

    // pitch
    Scalar det = -R(2,0) * R(2,0) + 1.0;
    if (det <= 0) {
        if (R(2,0) > 0){
            rpq[1] = -M_PI / 2.0;
        }
        else{
            rpq[1] = M_PI / 2.0;
        }
    }
    else{
        rpq[1] = -asin(R(2,0));
    }

    // yaw
    rpq[2] = atan2(R(1,0), R(0,0));

    return rpq;
}

inline Eigen::Matrix<Scalar,6,1> T2Cart(
        const Eigen::Matrix<Scalar,4,4>& T
        )
{
    Eigen::Matrix<Scalar,6,1> Cart;
    Eigen::Matrix<Scalar,3,1> rpq = R2Cart(T.block<3,3>(0,0));
    Cart[0] = T(0,3);
    Cart[1] = T(1,3);
    Cart[2] = T(2,3);
    Cart[3] = rpq[0];
    Cart[4] = rpq[1];
    Cart[5] = rpq[2];

    return Cart;
}


inline Eigen::Matrix3d Cart2R(
        const Scalar& r,
        const Scalar& p,
        const Scalar& q
        )
{
    Eigen::Matrix3d R;
    // psi = roll, th = pitch, phi = yaw
    Scalar cq, cp, cr, sq, sp, sr;
    cr = cos( r );
    cp = cos( p );
    cq = cos( q );

    sr = sin( r );
    sp = sin( p );
    sq = sin( q );

    R(0,0) = cp*cq;
    R(0,1) = -cr*sq+sr*sp*cq;
    R(0,2) = sr*sq+cr*sp*cq;

    R(1,0) = cp*sq;
    R(1,1) = cr*cq+sr*sp*sq;
    R(1,2) = -sr*cq+cr*sp*sq;

    R(2,0) = -sp;
    R(2,1) = sr*cp;
    R(2,2) = cr*cp;
    return R;
}
}
