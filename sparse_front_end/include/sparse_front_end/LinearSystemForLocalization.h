// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

//   Specialized LinearSystem for solving the front-end localization equations

#ifndef _LINEAR_SYSTEM_FOR_LOCALIZATION_H_
#define _LINEAR_SYSTEM_FOR_LOCALIZATION_H_

#include <vector>
#include <SparseFrontEnd/LinearSystem.h>
#include <Utils/MathTypes.h>
#include <Eigen/Cholesky>
#include <sophus/se3.hpp>
#include <calibu/Calibu.h>
#include <Utils/MathTypes.h>
template<typename Scalar=double>
class LinearSystemForLocalization : public LinearSystem
{
    typedef Eigen::Matrix<Scalar,2,1> Vector2t;
    typedef Eigen::Matrix<Scalar,3,1> Vector3t;
    typedef Eigen::Matrix<Scalar,4,1> Vector4t;
    typedef Eigen::Matrix<Scalar,6,1> Vector6t;
    typedef Eigen::Matrix<Scalar,4,4> Matrix4t;
    typedef Eigen::Matrix<Scalar,6,6> Matrix6t;

    typedef Sophus::SE3Group<Scalar> SE3t;
    public:

        void Init(
                const calibu::CameraRigT<Scalar>             rig,
                const SE3t&                 Twv   ///< Input: initial guess of pose
                 );

        void AddConstraint(
                   const Vector4t& X,    ///< Input: Landmark position
                   const Vector2t& z,    ///< Input: Image measurement
                   const unsigned int     uCam  ///< Input: Measurement's camera
                  );

        double Build();
        double Solve();
        void   Update();
        void   UndoUpdate();
        void   Reset();

        Sophus::SE3Group<Scalar> GetPose();

    private:

        ///////////////////////////////////////////////////////////////////////////////
        void _SE3ProjectionJacobian(const Vector4t& X,      ///< Input: Landmark
                 const SE3t&    Twv,    ///< Input: Current vehicle pose estimate
                 const calibu::CameraModelAndTransformT<Scalar>& cameraAndPose,      ///< Input: Camera intrinsics
                 Vector2t&       p,      ///< Output: Pixel projection
                 Eigen::Matrix<Scalar,2,6>& J   ///< Output: 2x6 Jacobian
               );

        ///////////////////////////////////////////////////////////////////////////////
        inline Scalar _Huber(const Scalar dResidual, const Scalar dSigma );

        ///////////////////////////////////////////////////////////////////////////////
        inline Scalar _CalcSigma( );

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
        calibu::CameraRigT<Scalar>    m_Rig;
        SE3t                          m_Twv;       ///< Vehicle frame to world frame.
        Eigen::Vector4tArray<Scalar>  m_v3DPts;    ///< Homogeneous coords.
        Eigen::Vector2tArray<Scalar>  m_vImagePts; ///< Image Measurements.
        std::vector<unsigned int>     m_vCameras;  ///< Camera indices.
        Matrix6t                      m_LHS;       ///< Left hand side of the linear system.
        Vector6t                      m_RHS;       ///< Right hand side of the linear system.
        Vector6t                      m_DeltaX;    ///< New state estimate delta.
};

#endif
