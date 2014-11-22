#ifndef TRIANGULATION_H
#define TRIANGULATION_H

#include <calibu/cam/CameraRig.h>
#include <feature_utils/Feature.h>
#include <utils/MathTypes.h>


///////////////////////////////////////////////////////////////////////////////
void TriangulateIfPossible(
        const calibu::CameraRigT<Scalar> &rRig,       //< Input:
        const unsigned int               uRefCamera, //< Input:
        Eigen::Vector2tArray     &vPixels,    //< Input:
        Eigen::Vector4t                  &Xr          //< Output: 3D point in robotic convention
        );


///////////////////////////////////////////////////////////////////////////////
bool Compute3dPnt(
        const Eigen::Vector2tArray &vPixels, ///< Input: vector of pixel measuements (2 for stereo)
        const unsigned int                 uRefCam,  ///< Input: reference camera
        const calibu::CameraRigT<Scalar>   &rig,     ///< Input: camera intrinsics and sensor poses
        Eigen::Vector4t                    &X        ///< Output: homogeneous 3d landmark
        );

///////////////////////////////////////////////////////////////////////////////
/// Given (possible) multi-view measurements in rFeature, see if we can
/// triangulate the 3d landmark position.
void TriangulateIfPossible(
        const calibu::CameraRigT<Scalar> &rRig,              //< Input:
        const unsigned int               uRefCamera,        //< Input:
        std::vector<Feature*>            &vMatchedFeatures,  //< Input:
        Eigen::Vector4t                  &Xr                 //< Output: 3D point in robotic convention
        );



#endif
