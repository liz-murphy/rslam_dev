// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.


#include <SparseFrontEnd/LinearSystemForLocalization.h>
#include <Utils/Utils.h>
#include <float.h>

///////////////////////////////////////////////////////////////////////////////
template <typename Scalar>
void LinearSystemForLocalization<Scalar>::Init(
        const calibu::CameraRigT<Scalar>             rig,
        const SE3t&                 Twv   ///< Input: initial guess of pose
       )
{
    Reset(); //erases previous m_v3DPts and m_v3DPts
    m_Rig   = rig;
    m_Twv   = Twv;
}

///////////////////////////////////////////////////////////////////////////////
template <typename Scalar>
void LinearSystemForLocalization<Scalar>::AddConstraint(
                const Vector4t &X,    ///< Input: Landmark position
                const Vector2t &z,    ///< Input: Image measurement
                const unsigned int    uCam  ///< Input: Measurement's camera
               )
{
    m_v3DPts.push_back( X );
    m_vImagePts.push_back( z );
    m_vCameras.push_back( uCam );
}

///////////////////////////////////////////////////////////////////////////////
template <typename Scalar>
double LinearSystemForLocalization<Scalar>::Build()
{

    m_LHS.setZero();
    m_RHS.setZero();

    Eigen::Matrix<Scalar,2,2> W;

    Scalar dTotalError = 0;
    Scalar dSigma = _CalcSigma();

     for( size_t ii = 0; ii < m_v3DPts.size(); ii++ ){

         const Vector4t&  X = m_v3DPts[ii];
         const Vector2t&  z = m_vImagePts[ii];
         const unsigned int c = m_vCameras[ii];
         Vector2t   p;
         Eigen::Matrix<Scalar,2,6> J;

         // Get jacbobian and landmark projection
         _SE3ProjectionJacobian( X, m_Twv, m_Rig.cameras[c], p, J );

         Vector2t residual = p - z;
         Scalar dWeight = _Huber( residual.norm(), dSigma );  // apply Huber weight
         W.setIdentity();
         W *= dWeight;
         m_LHS += J.transpose()*W*J;
         m_RHS += J.transpose()*W*residual;
         dTotalError +=  dWeight*residual.squaredNorm();

    }

    return dTotalError/m_v3DPts.size();
}

///////////////////////////////////////////////////////////////////////////////
template <typename Scalar>
double LinearSystemForLocalization<Scalar>::Solve()
{
    // see http://eigen.tuxfamily.org/api/TutorialLinearAlgebra.html
    m_DeltaX = m_LHS.ldlt().solve(m_RHS);

    //double dRelativeError = (m_LHS*m_DeltaX - m_RHS).norm() / m_RHS.norm(); // norm() is L2 norm
    return m_DeltaX.norm();
}

///////////////////////////////////////////////////////////////////////////////
template <typename Scalar>
void LinearSystemForLocalization<Scalar>::Update()
{
    m_Twv *= SE3t::exp( m_DeltaX );
}

///////////////////////////////////////////////////////////////////////////////
template <typename Scalar>
void LinearSystemForLocalization<Scalar>::UndoUpdate()
{
    m_Twv *= SE3t::exp( -m_DeltaX );
}

///////////////////////////////////////////////////////////////////////////////
template <typename Scalar>
void LinearSystemForLocalization<Scalar>::Reset()
{
    //	erases previous m_v3DPts and m_vImagePts
    m_vImagePts.clear();
    m_v3DPts.clear();
    m_vCameras.clear();
}

///////////////////////////////////////////////////////////////////////////////
template <typename Scalar>
Sophus::SE3Group<Scalar> LinearSystemForLocalization<Scalar>::GetPose()
{
    return m_Twv;
}

///////////////////////////////////////////////////////////////////////////////
template <typename Scalar>
void LinearSystemForLocalization<Scalar>::_SE3ProjectionJacobian(
                 const Vector4t& X,      ///< Input: Landmark
                 const SE3t&    Twv,    ///< Input: Current vehicle pose estimate
//                 const SE3t&    Tvs,    ///< Input: Sensor to vehicle
//                 const Eigen::Matrix3d& K,      ///< Input: Camera intrinsics
                 const calibu::CameraModelAndTransformT<Scalar>& cameraAndPose,
                 Vector2t&       p,      ///< Output: Pixel projection
                 Eigen::Matrix<Scalar,2,6>& J   ///< Output: 2x6 Jacobian
               )
{
    Eigen::Matrix<Scalar,3,4> M;
    const SE3t Tsv = cameraAndPose.T_wc.inverse();
    const SE3t Tvw = Twv.inverse();
    const SE3t Tsw = Tsv*Tvw;

    // compute landmark in the sensor frame
    p = cameraAndPose.camera.Transfer3D(Tsw,X.head(3),X[3]);

    //compute jacobian
    Eigen::Matrix<Scalar,2,4> dP = cameraAndPose.camera.dTransfer3D_dP(Tsw,X.template head(3),X[3]);
    for(unsigned int ii=0; ii<6; ++ii){
        J.template block<2,1>(0,ii) =  dP * Sophus::SE3Group<Scalar>::generator(ii) *  X;
    }

//    Eigen::Matrix<double,2,6> J_fd;
//    double dEps = 1e-6;
//    for(int ii = 0; ii < 6 ; ii++) {
//        Vector6t delta;
//        delta.setZero();
//        delta[ii] = dEps;
//        const Vector2t pPlus = cameraAndPose.camera.Transfer3D(Tsw*SE3t::exp(delta),X.head(3),X[3]);
//        delta[ii] = -dEps;
//        const Vector2t pMinus = cameraAndPose.camera.Transfer3D(Tsw*SE3t::exp(delta),X.head(3),X[3]);
//        J_fd.col(ii) = (pPlus-pMinus)/(2*dEps);
//    }
//    std::cout << "J:" << J << std::endl;
//    std::cout << "J_fd:" << J_fd << std::endl;
}

///////////////////////////////////////////////////////////////////////////////
// This is not strictly the Huber method because we weight by the std. dev.
template <typename Scalar>
Scalar LinearSystemForLocalization<Scalar>::_Huber( const Scalar dResidual, const Scalar dSigma )
{
    // See "Parameter Estimation Techniques: A Tutorial with Application to Conic
    // Fitting" by Zhengyou Zhang. PP 26 defines this magic number:
    const Scalar c = 1.2107*dSigma;

    if( dResidual > c ){ // should weight
        return c/dResidual;
    }else{
        return 1.0;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////
// use median to compute measurement std. dev.
template <typename Scalar>
Scalar LinearSystemForLocalization<Scalar>::_CalcSigma()
{
    Scalar dSigma = 1e-6; // minimum sigma allowed -- slight hack
    std::vector<Scalar> vErrors;
    vErrors.resize(m_v3DPts.size());
    SE3t Twv = m_Twv;

    for( size_t ii = 0; ii < m_v3DPts.size(); ++ii ) {

        const Vector4t& X   = m_v3DPts[ii];
        const Vector2t& z   = m_vImagePts[ii];
        const unsigned int     c   = m_vCameras[ii];
        const SE3t     Tws = Twv * m_Rig.cameras[c].T_wc;
        const Vector4t Xs_homog = Sophus::MultHomogeneous(Tws.inverse(),X);
        const Vector2t p  = m_Rig.cameras[c].camera.Project(Xs_homog.head(3));

        const Vector2t residual = p - z;
        vErrors[ii] = residual.squaredNorm();
     }
    // now get median as our estimate of sigma
    if( !vErrors.empty() ){
        std::sort( vErrors.begin(), vErrors.end() );
        //dSigma += 1.4826 * sqrt(vErrors[ vErrors.size()/2 ]);
        dSigma += sqrt(vErrors[ vErrors.size()/2 ]);
    }

    return dSigma;
}

// Specialization, commented out for speed
template class LinearSystemForLocalization<REAL_TYPE>;
