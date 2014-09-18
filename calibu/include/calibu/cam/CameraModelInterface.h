/* 
   This file is part of the Calibu Project.
   https://github.com/gwu-robotics/Calibu

   Copyright (C) 2013 George Washington University
                      Steven Lovegrove,
                      Gabe Sibley 

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
 */

#pragma once

#include <calibu/Platform.h>
#include <calibu/cam/CameraUtils.h>

#include <sophus/se3.hpp>

namespace calibu
{

///////////////////////////////////////////////////////////////////////////////
// Interface for polymorphic camera class
template<typename Scalar>
class CameraModelInterfaceT
{   
public:
    typedef Eigen::Matrix<Scalar,2,1> Vector2t;
    typedef Eigen::Matrix<Scalar,3,1> Vector3t;
    typedef Eigen::Matrix<Scalar,Eigen::Dynamic,1> VectorXt;
    typedef Eigen::Matrix<Scalar,3,3> Matrix3t;
    typedef Sophus::SE3Group<Scalar> SE3t;

    ////////////////////////////////////////////////////////////////////////////
    // Virtual member functions
    virtual ~CameraModelInterfaceT(){}

    ////////////////////////////////////////////////////////////////////////////
    virtual size_t Width() const = 0;
 
    ////////////////////////////////////////////////////////////////////////////
    virtual size_t Height() const = 0;

    ////////////////////////////////////////////////////////////////////////////
    virtual VectorXt GenericParams() const = 0;

    ////////////////////////////////////////////////////////////////////////////
    virtual void SetGenericParams(const VectorXt& params) = 0;

    ////////////////////////////////////////////////////////////////////////////
    virtual size_t NumParams() const = 0;
    
    ////////////////////////////////////////////////////////////////////////////
    virtual const Scalar* data() const = 0;

    ////////////////////////////////////////////////////////////////////////////
    virtual Scalar* data() = 0;
 
    ////////////////////////////////////////////////////////////////////////////
    virtual void SetImageDimensions( 
            size_t nWidth,  //< Input:
            size_t nHeight  //< Input:
            ) = 0;
 
    ////////////////////////////////////////////////////////////////////////////
    /// Report camera model version number.
    virtual int Version() const = 0;

    ////////////////////////////////////////////////////////////////////////////
    /// Set the camera veriona nuber.
    virtual void SetVersion( int nVersion ) = 0;

    ////////////////////////////////////////////////////////////////////////////
    /// Report camera model 
    virtual std::string Type() const = 0;

    ////////////////////////////////////////////////////////////////////////////
    /// Set the camera model name. e.g., "Left"
    virtual std::string Name() const = 0;

    ////////////////////////////////////////////////////////////////////////////
    /// Set the camera model name. e.g., "Left"
    virtual void SetName( const std::string& sName ) = 0;
    
    ////////////////////////////////////////////////////////////////////////////
    /// Set the camera serial number.
    virtual long int SerialNumber() const = 0;

    ////////////////////////////////////////////////////////////////////////////
    /// Set the camera serial number.
    virtual void SetSerialNumber( const long int nSerialNo ) = 0;

    ////////////////////////////////////////////////////////////////////////////
    /// Set the camera index (for multi-camera rigs).
    virtual int Index() const = 0;

    ////////////////////////////////////////////////////////////////////////////
    /// Set the camera index (for multi-camera rigs).
    virtual void SetIndex( const int nIndex ) = 0;

    ////////////////////////////////////////////////////////////////////////////
    /// Return 3x3 RDF matrix, describing the coordinate-frame convention.
    /// Row0: Right vector in camera frame of reference
    /// Row1: Down vector in camera frame of reference
    /// Row2: Forward vector in camera frame of reference
    virtual Matrix3t RDF() const = 0;
    
    ////////////////////////////////////////////////////////////////////////////
    /// Set the 3x3 RDF matrix, describing the coordinate-frame convention.
    /// Row0: Right vector in camera frame of reference
    /// Row1: Down vector in camera frame of reference
    /// Row2: Forward vector in camera frame of reference
    virtual void SetRDF( const Matrix3t& RDF ) = 0;

    ////////////////////////////////////////////////////////////////////////////
    virtual void PrintInfo() const = 0;
    
    ////////////////////////////////////////////////////////////////////////////
    // Return closest linear approximation of model as intrinsic matrix 'K'
    // Please avoid this function if possible - it may be deprecated in future.
    virtual Matrix3t K() const = 0;
 
    ////////////////////////////////////////////////////////////////////////////
    // Return closest linear approximation of model as inverse intrinsic matrix 'K^-1'
    // Please avoid this function if possible - it may be deprecated in future.
    virtual Matrix3t Kinv() const = 0;
 
    ////////////////////////////////////////////////////////////////////////////
    // Project point in 3d camera coordinates to image coordinates
    virtual Vector2t Project(
            const Vector3t& P      //< Input:
            ) const = 0;
 
    ////////////////////////////////////////////////////////////////////////////
    // Create 3D camera coordinates ray from image space coordinates
    virtual Vector3t Unproject(
            const Vector2t& p      //< Input:
            ) const = 0;
 
    ////////////////////////////////////////////////////////////////////////////
    /// Transfer point correspondence with known inv. depth to secondary camera frame.
    //  Points at infinity are supported (rho = 0)
    //  rhoPa = unproject(unmap(pa)).
    virtual Vector2t Transfer3D(
            const SE3t& T_ba,      //< Input:
            const Vector3t& rhoPa, //< Input:
            const Scalar rho       //< Input:
            ) const = 0;
    
    ////////////////////////////////////////////////////////////////////////////
    // Transfer point correspondence with known inv. depth to secondary camera frame.
    // Points at infinity are supported (rho = 0)
    // rhoPa = unproject(unmap(pa))
    virtual Vector2t Transfer3D(
            const SE3t& T_ba,      //< Input:
            const Vector3t& rhoPa, //< Input:
            const Scalar rho,      //< Input:
            bool& in_front         //< Output:
            ) const = 0;
 
    ////////////////////////////////////////////////////////////////////////////
    // Transfer point correspondence with known inv. depth to secondary camera frame.
    // Points at infinity are supported (rho = 0)
    virtual Vector2t Transfer(
            const SE3t& T_ba,   //< Input:
            const Vector2t& pa, //< Input:
            const Scalar rho    //< Input:
            ) const = 0;
 
    ////////////////////////////////////////////////////////////////////////////
    // Transfer point correspondence with known inv. depth to secondary camera frame.
    // Points at infinity are supported (rho = 0)
    virtual Vector2t Transfer(
            const SE3t& T_ba,   //< Input:
            const Vector2t& pa, //< Input:
            const Scalar rho,   //< Input:
            bool& in_front      //< Output:
            ) const = 0;
    
    ////////////////////////////////////////////////////////////////////////////
    // Transfer point correspondence with known inv. depth to secondary camera frame.
    // Points at infinity are supported (rho = 0)
    virtual Vector2t Transfer(
            const CameraModelInterfaceT<Scalar>& cam_a,
            const SE3t& T_ba,   //< Input:
            const Vector2t& pa, //< Input:
            const Scalar rho    //< Input:
            ) const = 0;
    
    ////////////////////////////////////////////////////////////////////////////
    // Transfer point correspondence with known inv. depth to secondary camera frame.
    // Points at infinity are supported (rho = 0)
    virtual Vector2t Transfer(
            const CameraModelInterfaceT<Scalar>& cam_a,            
            const SE3t& T_ba,   //< Input:
            const Vector2t& pa, //< Input:
            const Scalar rho,   //< Input:
            bool& in_front      //< Output:
            ) const = 0;
    
    ////////////////////////////////////////////////////////////////////////////
    // Return Jacobian of Project \in RR^2 wrt P \in RR^3.
    virtual Eigen::Matrix<Scalar,2,3> dProject_dP(const Vector3t& P) const = 0;

    ////////////////////////////////////////////////////////////////////////////
    virtual Eigen::Matrix<Scalar,2,Eigen::Dynamic> dMap_dParams(const Eigen::Matrix<Scalar,3,1>& p,
                                                                const Eigen::Matrix<Scalar,Eigen::Dynamic,1>& params) const = 0;

    ////////////////////////////////////////////////////////////////////////////
    // Return Jacobian of Transfer3D \in RR^2 wrt homogeneous P \in RR^4.
    virtual Eigen::Matrix<Scalar,2,4> dTransfer3D_dP(
            const SE3t& T_ba,   //< Input:
            const Eigen::Matrix<Scalar,3,1>& rhoPa, //< Input:
            const Scalar rho                        //< Input:
            ) const  = 0;
    
    ////////////////////////////////////////////////////////////////////////////
    // Scale 
    virtual void Scale( Scalar scale) = 0;
    
};

typedef CameraModelInterfaceT<double> CameraModelInterface;

}
