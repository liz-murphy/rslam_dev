// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.
#if 0

#ifndef COSTFUNCTIONS_H
#define	COSTFUNCTIONS_H

#include <sophus/se3.hpp>

 // define cost function
struct ReprojectionError
{
        ReprojectionError( const double& observation_x,        // image measurement
                           const double& observation_y,        // image measurement
                           const Eigen::Vector3d& dXa,         // landmark position in the previous reference frame
                           const Sophus::SE3d& dTsb,           //  vehicle to sensor transformation
                           const Eigen::Matrix3d& dK,          // camera intrinsic parameters
                           const double dWeight  )             // measurement weight
                           : m_dObservationX(observation_x),
                             m_dObservationY(observation_y),
                             m_dXa(dXa),
                             m_dTsb(dTsb),
                             m_dK(dK),
                             m_dWeight(dWeight) { }


        template <typename T>
        bool operator()(  const T* const _tTba,        //  transform from previous
                          T* residuals) const
        {

        //project the position of the landmark into the current coordinate system
        const Eigen::Map<const Sophus::SE3Group<T> > Tba(_tTba);   
        // get sensor to datum coordinates transform
        const Sophus::SE3Group<T> Tsa =  m_dTsb.cast<T>() * Tba; 
        //get the landmark in the sensor coordinates
        const Eigen::Matrix<T,3,1> Xs = Tsa * m_dXa.cast<T>();
        //transform to computer vision frame
//        const Eigen::Matrix<T,3,1> Xs_cv(Xs(1),Xs(2),Xs(0));

        const Eigen::Matrix<T,2,1> proj_uv(
            ((T)(m_dK(0,0))*Xs[1] + (T)(m_dK(0,1))*Xs[2] + T(m_dK(0,2))*Xs[0]) / (T)(Xs[0]),
            ((T)(m_dK(1,1))*Xs[2] + (T)(m_dK(1,2))*Xs[0]) / (T)(Xs[0])
        );

//        Eigen::Matrix<T,3,1> proj_uv = m_dK.cast<T>() * Xs_cv;
//        proj_uv(0) /= proj_uv(2);
//        proj_uv(1) /= proj_uv(2);
        // calculate residual error
        residuals[0] = T(m_dWeight)*( proj_uv[0] - T(m_dObservationX) );
        residuals[1] = T(m_dWeight)*( proj_uv[1] - T(m_dObservationY) );
        return true;
    }

    const double  m_dObservationX;
    const double  m_dObservationY;
    const Eigen::Vector3d m_dXa;
    const Sophus::SE3d    m_dTsb;
    const Eigen::Matrix3d m_dK;
    const double m_dWeight;
};

struct ReprojectionErrorC
{
        ReprojectionErrorC( const double& observation_x, // image measurement
                            const double& observation_y, // image measurement
                            const double* dXa,     // landmark position in the previous reference frame
                            const double* dTsb, //  vehicle to sensor transformation 
                            const double* dK,    // camera intrinsic parameters
                            const double dWeight  )         // measurement weight
                            : m_dObservationX(observation_x),
                              m_dObservationY(observation_y),
                              m_dXa(dXa),
                              m_dTsb(dTsb),
                              m_dK(dK),
                              m_dWeight(dWeight) { }


        template <typename T>
        bool operator()(  const T* const _tTba,        //  transform from previous
                                  T* residuals) const
        {
       
        //now do the projection into image space
       
        const T tx  = _tTba[0];
        const T ty  = _tTba[1];
        const T tz  = _tTba[2];
        const T qx = T(2.0) * _tTba[3];
        const T qy = T(2.0) * _tTba[4];
        const T qz = T(2.0) * _tTba[5];    
        const T qxx = qx * _tTba[3];
        const T qyy = qy * _tTba[4];
        const T qzz = qz  * _tTba[5];
        const T qxw = qx * _tTba[6];
        const T qyw = qy * _tTba[6];
        const T qzw = qz * _tTba[6];
        const T qxy = qy * _tTba[4];
        const T qxz = qz * _tTba[5];
        const T qyz = qy * _tTba[5];

        // compute rotation  matrix
        T R[3][3]; 
        R[0][0] = T(1.0) - qyy - qzz; 
        R[0][1] = qxy - qzw; 
        R[0][2] = qxz + qyw;
        R[1][0] = qxy + qzw;            
        R[1][1] = T(1.0) - qxx - qzz;  
        R[1][2] = qyz - qxw;
        R[2][0] = qxz - qyw;
        R[2][1] = qyz + qxw;
        R[2][2] = T(1.0) - qxx - qyy;
        
        
        T Tsa[3][4];
        Tsa[0][0] = T(m_dTsb[0])*R[0][0] + T(m_dTsb[4])*R[1][0] + T(m_dTsb[8])*R[2][0];
        Tsa[0][1] = T(m_dTsb[0])*R[0][1] + T(m_dTsb[4])*R[1][1] + T(m_dTsb[8])*R[2][1];
        Tsa[0][2] = T(m_dTsb[0])*R[0][2] + T(m_dTsb[4])*R[1][2] + T(m_dTsb[8])*R[2][2];
        Tsa[0][3] = T(m_dTsb[0])*tx + T(m_dTsb[4])*ty + T(m_dTsb[8])*tz + T(m_dTsb[12]);
        
        Tsa[1][0] = T(m_dTsb[1])*R[0][0] + T(m_dTsb[5])*R[1][0] + T(m_dTsb[9])*R[2][0];
        Tsa[1][1] = T(m_dTsb[1])*R[0][1] + T(m_dTsb[5])*R[1][1] + T(m_dTsb[9])*R[2][1];
        Tsa[1][2] = T(m_dTsb[1])*R[0][2] + T(m_dTsb[5])*R[1][2] + T(m_dTsb[9])*R[2][2];
        Tsa[1][3] = T(m_dTsb[1])*tx + T(m_dTsb[5])*ty + T(m_dTsb[9])*tz + T(m_dTsb[13]);

        Tsa[2][0] = T(m_dTsb[2])*R[0][0] + T(m_dTsb[6])*R[1][0] + T(m_dTsb[10])*R[2][0];
        Tsa[2][1] = T(m_dTsb[2])*R[0][1] + T(m_dTsb[6])*R[1][1] + T(m_dTsb[10])*R[2][1];
        Tsa[2][2] = T(m_dTsb[2])*R[0][2] + T(m_dTsb[6])*R[1][2] + T(m_dTsb[10])*R[2][2];
        Tsa[2][3] = T(m_dTsb[2])*tx + T(m_dTsb[6])*ty + T(m_dTsb[10])*tz + T(m_dTsb[14]);

        T Xs[3];
        Xs[0] = Tsa[0][0]*T(m_dXa[0]) + Tsa[0][1]*T(m_dXa[1]) + Tsa[0][2]*T(m_dXa[2]) + Tsa[0][3]; 
        Xs[1] = Tsa[1][0]*T(m_dXa[0]) + Tsa[1][1]*T(m_dXa[1]) + Tsa[1][2]*T(m_dXa[2]) + Tsa[1][3];
        Xs[2] = Tsa[2][0]*T(m_dXa[0]) + Tsa[2][1]*T(m_dXa[1]) + Tsa[2][2]*T(m_dXa[2]) + Tsa[2][3];
        
        T proj_uv[2];
        proj_uv[0] = T(m_dK[0])*Xs[1] + T(m_dK[3])*Xs[2] + T(m_dK[6])*Xs[0]; 
        proj_uv[1] = T(m_dK[4])*Xs[2] + T(m_dK[7])*Xs[0] ;
        
        proj_uv[0] /= Xs[0] ;
        proj_uv[1] /= Xs[0] ;
        // calculate residual error
        residuals[0] = T(m_dWeight)*( proj_uv[0] - T(m_dObservationX) );
        residuals[1] = T(m_dWeight)*( proj_uv[1] - T(m_dObservationY) );
        return true;
    }

    const double  m_dObservationX;
    const double  m_dObservationY;
    const double* m_dXa;
    const double* m_dTsb;
    const double* m_dK;
    const double m_dWeight;
};

#endif	/* COSTFUNCTIONS_H */

#endif
