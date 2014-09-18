#pragma once

#include <opencv2/opencv.hpp>

#ifdef __APPLE__
#include <OpenCL/opencl.h>
#else
#include <CL/cl.h>
#endif

#ifndef CL_VERSION_1_1
#error  "This program only runs for OpenCL v1.1!"
#endif

#include <sophus/se3.hpp>
#include <calibu/cam/CameraRig.h>

#include <Utils/ESM.h>
#include <Utils/PatchUtils.h>
#include <Utils/PoseHelpers.h>
#include <Utils/MathTypes.h>
#include <Utils/PrintMessage.h>


class DensePoseEstimateCL
{

public:
  /////////////////////////////////////////////////////////////////////////////
  DensePoseEstimateCL()
    : PYRAMID_LEVELS(5)
  {
    // http://www.khronos.org/registry/cl/sdk/1.1/docs/man/xhtml/clGetPlatformIDs.html
    cl_uint platformIdCount = 0;
    clGetPlatformIDs(0, nullptr, &platformIdCount);

    if(platformIdCount == 0) {
      PrintMessage(0, "No OpenCL platform found!\n");
    } else {
      PrintMessage(0, "Found %d platform(s).\n", platformIdCount);
    }

    std::vector<cl_platform_id> platformIds(platformIdCount);
    clGetPlatformIDs(platformIdCount, platformIds.data(), nullptr);

    for (cl_uint i = 0; i < platformIdCount; ++i) {
      PrintMessage(0, "\t (%d) : %s\n", i+1,
                   _GetPlatformName(platformIds[i]).c_str());
    }

    // http://www.khronos.org/registry/cl/sdk/1.1/docs/man/xhtml/clGetDeviceIDs.html
    cl_uint deviceIdCount = 0;
    clGetDeviceIDs(platformIds[0], CL_DEVICE_TYPE_ALL, 0, nullptr,
                   &deviceIdCount);

    if (deviceIdCount == 0) {
      PrintMessage(0, "No OpenCL devices found!\n");
    } else {
      PrintMessage(0, "Found %d device(s).\n", deviceIdCount);
    }

    std::vector<cl_device_id> deviceIds(deviceIdCount);
    clGetDeviceIDs(platformIds[0], CL_DEVICE_TYPE_ALL, deviceIdCount,
                   deviceIds.data(), nullptr);

    for (cl_uint i = 0; i < deviceIdCount; ++i) {
      PrintMessage(0, "\t (%d) : %s supports %s\n", i+1,
                   _GetDeviceName(deviceIds[i]).c_str(),
                   _GetDeviceVersion(deviceIds[i]).c_str());
    }

    // http://www.khronos.org/registry/cl/sdk/1.1/docs/man/xhtml/clCreateContext.html
    const cl_context_properties contextProperties[] = {
      CL_CONTEXT_PLATFORM,
      reinterpret_cast<cl_context_properties>(platformIds[0]), 0, 0
    };

#ifdef ANDROID
    cl_uint selectedDeviceId = 0;
#else
    cl_uint selectedDeviceId = 1;
#endif
    PrintMessage(0, "Device Selected: #%d\n", selectedDeviceId+1);

    cl_int error = CL_SUCCESS;

    // print some info
    /*
        cl_uint MaxDims;
        error = clGetDeviceInfo(deviceIds[selectedDeviceId], CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS, sizeof(cl_uint), &MaxDims, nullptr);
        _CheckError("Getting Max Dimensions", error);
        std::cout << "Max workgroup dimensions for this device is: " << MaxDims << std::endl;

        size_t MaxWorkItemSize[MaxDims];
        error = clGetDeviceInfo(deviceIds[selectedDeviceId], CL_DEVICE_MAX_WORK_ITEM_SIZES, sizeof(MaxWorkItemSize), &MaxWorkItemSize, nullptr);
        _CheckError("Getting Max Work Item Size", error);
        std::cout << "Max work item sizes for this device are: " << MaxWorkItemSize[0] << " " << MaxWorkItemSize[1] << " " << MaxWorkItemSize[2] << " " << std::endl;

        cl_uint MaxComputeUnits;
        error = clGetDeviceInfo(deviceIds[selectedDeviceId], CL_DEVICE_MAX_COMPUTE_UNITS, sizeof(cl_uint), &MaxComputeUnits, nullptr);
        _CheckError("Getting Max Compute Units", error);
        std::cout << "Max compute units for this device is: " << MaxComputeUnits << std::endl;

        error = clGetDeviceInfo(deviceIds[selectedDeviceId], CL_DEVICE_MAX_WORK_GROUP_SIZE, sizeof(size_t), &m_MaxWorkgroupSize, nullptr);
        _CheckError("Getting Max Workgroup Size", error);
        std::cout << "Max workgroup size for this device is: " << m_MaxWorkgroupSize << std::endl;
        */

    //        m_ClContext = clCreateContext(contextProperties, deviceIdCount, deviceIds.data(), nullptr, nullptr, &error);
    m_ClContext = clCreateContext(contextProperties, 1,
                                  &deviceIds[selectedDeviceId], &pfn_notify,
                                  nullptr, &error);
    _CheckError("Creating Context", error);
    PrintMessage(0, "OpenCL context created.\n");

    // create queue
    // http://www.khronos.org/registry/cl/sdk/1.1/docs/man/xhtml/clCreateCommandQueue.html
    m_ClQueue = clCreateCommandQueue(m_ClContext, deviceIds[selectedDeviceId],
                                     0, &error);
    _CheckError("Queue", error);
    PrintMessage(0, "OpenCL command queue created.\n");

#ifdef ANDROID
    m_ClProgram = _LoadProgramFromFile("/sdcard/kernel.cl", m_ClContext);
#else
    m_ClProgram = _LoadProgramFromFile(
      "/Users/jmf/Code/RSLAM/rslam/CommonFrontEnd/DensePoseEstimateKernel.cl", m_ClContext);
#endif

    error = clBuildProgram(m_ClProgram, 1, &deviceIds[selectedDeviceId],
                           nullptr, nullptr, nullptr);
    _CheckError("Building Program", error);

    cl_build_status build_status;
    error = clGetProgramBuildInfo(m_ClProgram, deviceIds[selectedDeviceId],
                                  CL_PROGRAM_BUILD_STATUS,
                                  sizeof(cl_build_status),
                                  &build_status, nullptr);
    _CheckError("Getting Program Build Info", error);
    if( build_status == CL_BUILD_ERROR ) {
      PrintMessage(0, "Fatal error encountered when building kernel.\n");
      size_t log_size;
      char log[1024];
      error = clGetProgramBuildInfo(m_ClProgram, deviceIds[selectedDeviceId],
                                    CL_PROGRAM_BUILD_LOG, 1024, log, &log_size);
      _CheckError("Getting Program Build Info", error);
      PrintMessage(0,"Log (%d) : %s\n", log_size, log);
      exit(EXIT_FAILURE);
    }

    // http://www.khronos.org/registry/cl/sdk/1.1/docs/man/xhtml/clCreateKernel.html
    m_ClKernel = clCreateKernel(m_ClProgram, "DensePoseRefinement", &error);
    _CheckError("Creating Kernel", error);


    error = clGetKernelWorkGroupInfo(m_ClKernel, deviceIds[selectedDeviceId],
                                     CL_KERNEL_WORK_GROUP_SIZE,
                                     sizeof(size_t), &m_WorkgroupSize, nullptr);
    _CheckError("Getting Workgroup Size", error);
    PrintMessage(0, "Preferred workgroup size for this device: %d\n",
                 m_WorkgroupSize);

    // TODO(jmf)
    // we might have to change how this is calculated since the limitation is
    // not actually the workgroup size but the amount of memory required for the
    // reduction. Must be powers of 2 given the way the reduction works.
#ifdef ANDROID
//    m_WorkgroupSize = 32;
    m_WorkgroupSize = 16;
#else
    m_WorkgroupSize = 128;
//    m_WorkgroupSize = 1;
#endif
  }

  /////////////////////////////////////////////////////////////////////////////
  ~DensePoseEstimateCL()
  {
    // TODO(jmf) release everything!!
    clReleaseMemObject(m_clTgd);
    clReleaseCommandQueue(m_ClQueue);
    clReleaseKernel(m_ClKernel);
    clReleaseProgram(m_ClProgram);
    clReleaseContext(m_ClContext);
  }

  /////////////////////////////////////////////////////////////////////////////
  void Init(
      unsigned int            nImageHeight,
      unsigned int            nImageWidth
      )
  {
    cl_int error = CL_SUCCESS;

    m_nImageWidth = nImageWidth;
    m_nImageHeight = nImageHeight;

    // calculate block sizes
    size_t local_size = _GCD(m_nImageWidth*m_nImageHeight, m_WorkgroupSize);
    size_t MaxNumWorkgroups = (nImageHeight*nImageWidth)/local_size;

    // prepare solution buffer
    m_Tmp.resize(21);
    m_hLHS.resize(21*MaxNumWorkgroups);
    m_hRHS.resize(6*MaxNumWorkgroups);
    m_hError.resize(2*MaxNumWorkgroups);

    m_clLHS = clCreateBuffer(m_ClContext, CL_MEM_READ_WRITE|CL_MEM_ALLOC_HOST_PTR,
                           sizeof(float)*21*MaxNumWorkgroups, nullptr, &error);
    _CheckError("Allocating LHS", error);
    m_clRHS = clCreateBuffer(m_ClContext, CL_MEM_READ_WRITE|CL_MEM_ALLOC_HOST_PTR,
                           sizeof(float)*6*MaxNumWorkgroups, nullptr, &error);
    _CheckError("Allocating RHS", error);
    m_clError = clCreateBuffer(m_ClContext, CL_MEM_READ_WRITE|CL_MEM_ALLOC_HOST_PTR,
                             sizeof(float)*2*MaxNumWorkgroups, nullptr, &error);
    _CheckError("Allocating Error", error);

    m_clLiveGreyPyr.resize(PYRAMID_LEVELS);
    m_clRefGreyPyr.resize(PYRAMID_LEVELS);
    m_clRefDepthPyr.resize(PYRAMID_LEVELS);

    // allocate image memory
    for( size_t ii = 0; ii < PYRAMID_LEVELS; ++ii ) {

      const unsigned int nLvlWidth = nImageWidth >> ii;
      const unsigned int nLvlHeight = nImageHeight >> ii;

      m_clLiveGreyPyr[ii] = clCreateBuffer(m_ClContext, CL_MEM_READ_ONLY|CL_MEM_ALLOC_HOST_PTR,
                                         nLvlWidth*nLvlHeight, nullptr, &error);
      _CheckError("Allocate GPU Grey Pyramid", error);
      m_clRefGreyPyr[ii] = clCreateBuffer(m_ClContext, CL_MEM_READ_ONLY|CL_MEM_ALLOC_HOST_PTR,
                                        nLvlWidth*nLvlHeight, nullptr, &error);
      _CheckError("Allocate GPU Grey Pyramid", error);
      m_clRefDepthPyr[ii] = clCreateBuffer(m_ClContext, CL_MEM_READ_ONLY|CL_MEM_ALLOC_HOST_PTR,
                                         sizeof(float)*nLvlWidth*nLvlHeight,
                                         nullptr, &error);
      _CheckError("Allocate GPU Depth Pyramid", error);
    }

    m_clLiveGreyCM.resize(PYRAMID_LEVELS);
    m_clRefGreyCM.resize(PYRAMID_LEVELS);
    m_clRefDepthCM.resize(PYRAMID_LEVELS);

    // allocate intrinsics memory
    for( size_t ii = 0; ii < PYRAMID_LEVELS; ++ii ) {
      m_clLiveGreyCM[ii] = clCreateBuffer(m_ClContext, CL_MEM_READ_ONLY|CL_MEM_ALLOC_HOST_PTR,
                                        sizeof(float)*4, nullptr, &error);
      _CheckError("Allocate GPU Grey K Pyramid", error);
      m_clRefGreyCM[ii] = clCreateBuffer(m_ClContext, CL_MEM_READ_ONLY|CL_MEM_ALLOC_HOST_PTR,
                                       sizeof(float)*4, nullptr, &error);
      _CheckError("Allocate GPU Grey K Pyramid", error);
      m_clRefDepthCM[ii] = clCreateBuffer(m_ClContext, CL_MEM_READ_ONLY|CL_MEM_ALLOC_HOST_PTR,
                                        sizeof(float)*4, nullptr, &error);
      _CheckError("Allocate GPU Depth K Pyramid", error);
    }

    // allocate transform
    m_clTgd = clCreateBuffer(m_ClContext, CL_MEM_READ_ONLY, sizeof(float)*12,
                           nullptr, &error);
    _CheckError("Allocate GPU Tgd", error);
    m_clTlr = clCreateBuffer(m_ClContext, CL_MEM_READ_ONLY,
                           sizeof(float)*12, nullptr, &error);
    _CheckError("Allocate GPU Tlr", error);
  }


  /////////////////////////////////////////////////////////////////////////////
  void SetParams(
      const calibu::CameraModelGeneric<Scalar>&   LiveGreyCM,
      const calibu::CameraModelGeneric<Scalar>&   RefGreyCM,
      const calibu::CameraModelGeneric<Scalar>&   RefDepthCM,
      const Sophus::SE3t&                         Tgd
      )
  {
    cl_int error = CL_SUCCESS;

    // store scaled camera models (to avoid recomputing)
    std::vector<calibu::CameraModelGeneric<Scalar>> vLiveGreyCM;
    std::vector<calibu::CameraModelGeneric<Scalar>> vRefGreyCM;
    std::vector<calibu::CameraModelGeneric<Scalar>> vRefDepthCM;

    PrintMessage(0, "+++++ Building Camera Model Pyramid.\n");
    for( size_t ii = 0; ii < PYRAMID_LEVELS; ++ii ) {
      vLiveGreyCM.push_back( _ScaleCM( LiveGreyCM, ii ) );
      vRefGreyCM.push_back( _ScaleCM( RefGreyCM, ii ) );
      vRefDepthCM.push_back( _ScaleCM( RefDepthCM, ii ) );
    }
    PrintMessage(0, "----- Building Camera Model Pyramid.\n");

    PrintMessage(0, "+++++ Copying Camera Model Pyramid to GPU.\n");
    float K[4];
    for( size_t ii = 0; ii < PYRAMID_LEVELS; ++ii ) {
      K[0] = vLiveGreyCM[ii].K()(0,0);
      K[1] = vLiveGreyCM[ii].K()(1,1);
      K[2] = vLiveGreyCM[ii].K()(0,2);
      K[3] = vLiveGreyCM[ii].K()(1,2);
      error = clEnqueueWriteBuffer(m_ClQueue, m_clLiveGreyCM[ii], CL_TRUE, 0,
                                   sizeof(float)*4, K, 0, nullptr, nullptr);
      _CheckError("Writing Grey K Pyramid", error);

      K[0] = vRefGreyCM[ii].K()(0,0);
      K[1] = vRefGreyCM[ii].K()(1,1);
      K[2] = vRefGreyCM[ii].K()(0,2);
      K[3] = vRefGreyCM[ii].K()(1,2);
      error = clEnqueueWriteBuffer(m_ClQueue, m_clRefGreyCM[ii], CL_TRUE, 0,
                                   sizeof(float)*4, K, 0, nullptr, nullptr);
      _CheckError("Writing Grey K Pyramid", error);

      K[0] = vRefDepthCM[ii].K()(0,0);
      K[1] = vRefDepthCM[ii].K()(1,1);
      K[2] = vRefDepthCM[ii].K()(0,2);
      K[3] = vRefDepthCM[ii].K()(1,2);
      error = clEnqueueWriteBuffer(m_ClQueue, m_clRefDepthCM[ii], CL_TRUE, 0,
                                   sizeof(float)*4, K, 0, nullptr, nullptr);
      _CheckError("Writing Depth K Pyramid", error);
    }
    PrintMessage(0, "----- Copying Camera Model Pyramid to GPU.\n");

    // copy reference camera's depth-grey transform
    // TODO(jmf) pass matrix directly but make sure it is float no double
    float T[12];
    T[0] = Tgd.matrix()(0,0);
    T[1] = Tgd.matrix()(0,1);
    T[2] = Tgd.matrix()(0,2);
    T[3] = Tgd.matrix()(0,3);
    T[4] = Tgd.matrix()(1,0);
    T[5] = Tgd.matrix()(1,1);
    T[6] = Tgd.matrix()(1,2);
    T[7] = Tgd.matrix()(1,3);
    T[8] = Tgd.matrix()(2,0);
    T[9] = Tgd.matrix()(2,1);
    T[10] = Tgd.matrix()(2,2);
    T[11] = Tgd.matrix()(2,3);
    error = clEnqueueWriteBuffer(m_ClQueue, m_clTgd, CL_TRUE, 0,
                                 sizeof(float)*12, T, 0, nullptr, nullptr);
    _CheckError("Writing Tgd", error);
    std::cout << "Tgd: " << Tgd.log().transpose() << std::endl;
  }


  /////////////////////////////////////////////////////////////////////////////
  void SetKeyframe(
      const cv::Mat&                              RefGrey,
      const cv::Mat&                              RefDepth
      )
  {
    cl_int error = CL_SUCCESS;

    // TODO(jmf) blur-decimate on the GPU directly to GPU memory
    PrintMessage(0, "+++++ Building CV Pyramids.\n");
    cv::buildPyramid( RefGrey, m_RefGreyPyr, PYRAMID_LEVELS );
    cv::buildPyramid( RefDepth, m_RefDepthPyr, PYRAMID_LEVELS );
    PrintMessage(0, "----- Building CV Pyramids.\n");

    PrintMessage(0, "+++++ Copying images to GPU memory.\n");
    for( size_t ii = 0; ii < PYRAMID_LEVELS; ++ii ) {

      const unsigned int nLvlWidth = m_RefGreyPyr[ii].cols;
      const unsigned int nLvlHeight = m_RefGreyPyr[ii].rows;

      error = clEnqueueWriteBuffer(m_ClQueue, m_clRefGreyPyr[ii], CL_TRUE, 0,
                                   nLvlWidth*nLvlHeight, m_RefGreyPyr[ii].data,
                                   0, nullptr, nullptr);
      _CheckError("Writing Ref Grey Pyramid", error);
      error = clEnqueueWriteBuffer(m_ClQueue, m_clRefDepthPyr[ii], CL_TRUE, 0,
                                   sizeof(float)*nLvlWidth*nLvlHeight,
                                   m_RefDepthPyr[ii].data, 0, nullptr, nullptr);
      _CheckError("Writing Ref Depth Pyramid", error);
    }
    PrintMessage(0, "----- Copying images to GPU memory.\n");
  }


  /////////////////////////////////////////////////////////////////////////////
  Scalar Estimate(
      const cv::Mat&              LiveGrey,   //< Input: live image
      Sophus::SE3Group<Scalar>&   Trl         //< Input/Output: transform between grey cameras (input is hint)
      )
  {
    cl_int error = CL_SUCCESS;

    // TODO(jmf) blur-decimate on the GPU directly to GPU memory
    cv::buildPyramid( LiveGrey, m_LiveGreyPyr, PYRAMID_LEVELS );

    for( size_t ii = 0; ii < PYRAMID_LEVELS; ++ii ) {

      const unsigned int nLvlWidth = m_LiveGreyPyr[ii].cols;
      const unsigned int nLvlHeight = m_LiveGreyPyr[ii].rows;

      error = clEnqueueWriteBuffer(m_ClQueue, m_clLiveGreyPyr[ii], CL_TRUE, 0,
                                   nLvlWidth*nLvlHeight, m_LiveGreyPyr[ii].data,
                                   0, nullptr, nullptr);
      _CheckError("Writing Live Grey Pyramid", error);
    }


    // options
    const float    NormC = 8;

    // set pyramid max-iterations and full estimate mask
    std::vector<bool>           vFullEstimate = {1, 1, 1, 1, 0};
    std::vector<unsigned int>   vMaxIteration = {1, 2, 3, 4, 5};

    // aux variables
    Eigen::Matrix6t LHS;
    Eigen::Vector6t RHS;
    float           Error[2];
    float           LastError = FLT_MAX;
    float T[12];


    // iterate through pyramid levels
    for( int nPyrLvl = PYRAMID_LEVELS-1; nPyrLvl >= 0; nPyrLvl-- ) {

      const unsigned int ImageWidth = m_nImageWidth >> nPyrLvl;

      const cl_mem& LiveGreyImg = m_clLiveGreyPyr[nPyrLvl];
      const cl_mem& RefGreyImg = m_clRefGreyPyr[nPyrLvl];
      const cl_mem& RefDepthImg = m_clRefDepthPyr[nPyrLvl];

      // TODO(jmf) Pass live camera model (in case they are different)
      const cl_mem& LiveGreyCM = m_clLiveGreyCM[nPyrLvl];
      const cl_mem& RefGreyCM = m_clRefGreyCM[nPyrLvl];
      const cl_mem& RefDepthCM = m_clRefDepthCM[nPyrLvl];

      // reset error
      LastError = FLT_MAX;

      // set pyramid norm parameter
      const float    NormCPyr = NormC * (nPyrLvl + 1);

      for(unsigned int nIters = 0; nIters < vMaxIteration[nPyrLvl]; ++nIters) {

        // inverse transform
        const Sophus::SE3t Tlr = Trl.inverse();

        // TODO(jmf) pass matrix directly but make sure it is float no double
        T[0] = Tlr.matrix()(0,0);
        T[1] = Tlr.matrix()(0,1);
        T[2] = Tlr.matrix()(0,2);
        T[3] = Tlr.matrix()(0,3);
        T[4] = Tlr.matrix()(1,0);
        T[5] = Tlr.matrix()(1,1);
        T[6] = Tlr.matrix()(1,2);
        T[7] = Tlr.matrix()(1,3);
        T[8] = Tlr.matrix()(2,0);
        T[9] = Tlr.matrix()(2,1);
        T[10] = Tlr.matrix()(2,2);
        T[11] = Tlr.matrix()(2,3);
        error = clEnqueueWriteBuffer(m_ClQueue, m_clTlr, CL_TRUE, 0,
                                     sizeof(float)*12, T, 0, nullptr, nullptr);
        _CheckError("Writing Tlr", error);


        ///--------------------

        // calculate block sizes
        size_t global_size = (m_nImageWidth >> nPyrLvl) *
                             (m_nImageHeight >> nPyrLvl);
        size_t local_size = _GCD(global_size, m_WorkgroupSize);
        size_t NumWorkgroups = global_size/local_size;
        //        PrintMessage(0, "Global Size: %d\n", global_size);
        //        PrintMessage(0, "Block Size: %d\n", local_size);
        //        PrintMessage(0, "Num of Workgroups: %d\n", NumWorkgroups);


        // setup the kernel arguments
        error = clSetKernelArg(m_ClKernel, 0, sizeof(cl_mem), &LiveGreyImg);
        _CheckError("Setting Argument 0", error);
        error = clSetKernelArg(m_ClKernel, 1, sizeof(cl_mem), &RefGreyImg);
        _CheckError("Setting Argument 1", error);
        error = clSetKernelArg(m_ClKernel, 2, sizeof(cl_mem), &RefDepthImg);
        _CheckError("Setting Argument 2", error);
        error = clSetKernelArg(m_ClKernel, 3, sizeof(unsigned int), &ImageWidth);
        _CheckError("Setting Argument 3", error);
        error = clSetKernelArg(m_ClKernel, 4, sizeof(cl_mem), &LiveGreyCM);
        _CheckError("Setting Argument 4", error);
        error = clSetKernelArg(m_ClKernel, 5, sizeof(cl_mem), &RefGreyCM);
        _CheckError("Setting Argument 5", error);
        error = clSetKernelArg(m_ClKernel, 6, sizeof(cl_mem), &RefDepthCM);
        _CheckError("Setting Argument 6", error);
        error = clSetKernelArg(m_ClKernel, 7, sizeof(cl_mem), &m_clTgd);
        _CheckError("Setting Argument 7", error);
        error = clSetKernelArg(m_ClKernel, 8, sizeof(cl_mem), &m_clTlr);
        _CheckError("Setting Argument 8", error);
        error = clSetKernelArg(m_ClKernel, 9, sizeof(float), &NormCPyr);
        _CheckError("Setting Argument 9", error);
        error = clSetKernelArg(m_ClKernel, 10, local_size*sizeof(float)*29,
                               nullptr);
        _CheckError("Setting Argument 10", error);
        error = clSetKernelArg(m_ClKernel, 11, sizeof(cl_mem), &m_clLHS);
        _CheckError("Setting Argument 11", error);
        error = clSetKernelArg(m_ClKernel, 12, sizeof(cl_mem), &m_clRHS);
        _CheckError("Setting Argument 12", error);
        error = clSetKernelArg(m_ClKernel, 13, sizeof(cl_mem), &m_clError);
        _CheckError("Setting Argument 13", error);


        // run the processing
        // http://www.khronos.org/registry/cl/sdk/1.1/docs/man/xhtml/clEnqueueNDRangeKernel.html
        error = clEnqueueNDRangeKernel(m_ClQueue, m_ClKernel, 1, nullptr,
                                       &global_size, &local_size, 0, nullptr,
                                       nullptr);
        _CheckError("Launching Kernel", error);

        // get results back
        error = clEnqueueReadBuffer(m_ClQueue, m_clLHS, CL_TRUE, 0,
                                    sizeof(float)*21*NumWorkgroups,
                                    m_hLHS.data(), 0, nullptr, nullptr);
        _CheckError("LHS Read", error);
        error = clEnqueueReadBuffer(m_ClQueue, m_clRHS, CL_TRUE, 0,
                                    sizeof(float)*6*NumWorkgroups,
                                    m_hRHS.data(), 0, nullptr, nullptr);
        _CheckError("RHS Read", error);
        error = clEnqueueReadBuffer(m_ClQueue, m_clError, CL_TRUE, 0,
                                    sizeof(float)*2*NumWorkgroups,
                                    m_hError.data(), 0, nullptr, nullptr);
        _CheckError("Error Read", error);

        // reset & populate our matrices
        std::fill(m_Tmp.begin(), m_Tmp.end(), 0);
        LHS.setZero();
        RHS.setZero();
        Error[0] = Error[1] = 0;

        for( size_t ii = 0; ii < NumWorkgroups; ++ii ) {
          for( size_t jj = 0; jj < 21; ++jj ) {
            m_Tmp[jj] += m_hLHS[jj+(ii*21)];
          }
        }

        // populate our matrices
        LHS << m_Tmp[0], m_Tmp[1],  m_Tmp[2],  m_Tmp[3],  m_Tmp[4],  m_Tmp[5],
            m_Tmp[1], m_Tmp[6],  m_Tmp[7],  m_Tmp[8],  m_Tmp[9],  m_Tmp[10],
            m_Tmp[2], m_Tmp[7],  m_Tmp[11], m_Tmp[12], m_Tmp[13], m_Tmp[14],
            m_Tmp[3], m_Tmp[8],  m_Tmp[12], m_Tmp[15], m_Tmp[16], m_Tmp[17],
            m_Tmp[4], m_Tmp[9],  m_Tmp[13], m_Tmp[16], m_Tmp[18], m_Tmp[19],
            m_Tmp[5], m_Tmp[10], m_Tmp[14], m_Tmp[17], m_Tmp[19], m_Tmp[20];

        for( size_t ii = 0; ii < NumWorkgroups; ++ii ) {
          RHS[0] += m_hRHS[ii*6];
          RHS[1] += m_hRHS[1+(ii*6)];
          RHS[2] += m_hRHS[2+(ii*6)];
          RHS[3] += m_hRHS[3+(ii*6)];
          RHS[4] += m_hRHS[4+(ii*6)];
          RHS[5] += m_hRHS[5+(ii*6)];
          Error[0] += m_hError[ii*2];
          Error[1] += m_hError[1+(ii*2)];
        }


        //        std::cout << "LHS: " << std::endl << LHS << std::endl;
        //        std::cout << "RHS: " << RHS.transpose() << std::endl;
        //        std::cout << "NUM OBS: " << Error[1] << std::endl;


        ///--------------------


        // solution
        Eigen::Vector6t X;

        // check if we are solving only for rotation, or full estimate
        if( vFullEstimate[nPyrLvl] ) {
          // decompose matrix
          Eigen::FullPivLU< Eigen::Matrix<Scalar,6,6> >    lu_JTJ(LHS);

          // check degenerate system
          if( lu_JTJ.rank() < 6 ) {
            PrintMessage(0, "warning(@L%d I%d) LS trashed. Rank deficient!\n",
                         nPyrLvl+1, nIters+1);
          }

          X = - (lu_JTJ.solve(RHS));
        } else {
          // extract rotation information only
          Eigen::Matrix3t                           rLHS = LHS.block<3,3>(3,3);
          Eigen::Vector3t                           rRHS = RHS.tail(3);
          Eigen::FullPivLU< Eigen::Matrix<Scalar,3,3> > lu_JTJ(rLHS);

          // check degenerate system
          if( lu_JTJ.rank() < 3 ) {
            PrintMessage(0, "warning(@L%d I%d) LS trashed. Rank deficient!\n",
                         nPyrLvl+1, nIters+1);
          }

          Eigen::Vector3t rX;
          rX = - (lu_JTJ.solve(rRHS));

          // pack solution
          X.setZero();
          X.tail(3) = rX;
        }


        // get RMSE
        const float NewError = sqrt( Error[0] / Error[1] );

        if( NewError < LastError ) {
          // update error
          LastError = NewError;

          // update Trl
          Trl = (Tlr * Sophus::SE3Group<Scalar>::exp(X)).inverse();

          if( X.norm() < 1e-5 ) {
            PrintMessage(0, "notice(@L%d I%d) Update is too small. Breaking early!\n",
                         nPyrLvl+1, nIters+1);
            break;
          }
        } else {
          PrintMessage(0, "notice(@L%d I%d) Error is increasing. Breaking early!\n",
                       nPyrLvl+1, nIters+1);
          break;
        }
      }
    }

    return LastError;
  }

private:

  /////////////////////////////////////////////////////////////////////////////
  void _CheckError( const char* label, cl_int error )
  {
    if(error != CL_SUCCESS) {
      PrintMessage(0, "OpenCL Error: Call failed at '%s' with error %d.\n",
                   label, error);
      std::exit(EXIT_FAILURE);
    }
  }

  /////////////////////////////////////////////////////////////////////////////
  std::string _GetPlatformName(cl_platform_id id)
  {
    size_t size = 0;
    clGetPlatformInfo (id, CL_PLATFORM_NAME, 0, nullptr, &size);

    std::string result;
    result.resize (size);
    clGetPlatformInfo (id, CL_PLATFORM_NAME, size,
                       const_cast<char*>(result.data()), nullptr);

    return result;
  }

  /////////////////////////////////////////////////////////////////////////////
  std::string _GetDeviceName(cl_device_id id)
  {
    size_t size = 0;
    clGetDeviceInfo(id, CL_DEVICE_NAME, 0, nullptr, &size);

    std::string result;
    result.resize(size);
    clGetDeviceInfo (id, CL_DEVICE_NAME, size,
                     const_cast<char*>(result.data()), nullptr);

    return result;
  }


  /////////////////////////////////////////////////////////////////////////////
  std::string _GetDeviceVersion(cl_device_id id)
  {
    size_t size = 0;
    clGetDeviceInfo(id, CL_DEVICE_VERSION, 0, nullptr, &size);

    std::string result;
    result.resize(size);
    clGetDeviceInfo (id, CL_DEVICE_VERSION, size,
                     const_cast<char*>(result.data()), nullptr);

    return result;
  }


  /////////////////////////////////////////////////////////////////////////////
  cl_program _LoadProgramFromFile(const char* filename, cl_context context)
  {
    std::ifstream in(filename);

    if( in.is_open() == false ) {
      std::cerr << "Error opening kernel file." << std::endl;
      return nullptr;
    }

    std::string source((std::istreambuf_iterator<char>(in)),
                       std::istreambuf_iterator<char>());

    // http://www.khronos.org/registry/cl/sdk/1.1/docs/man/xhtml/clCreateProgramWithSource.html
    size_t lengths[1] = { source.size() };
    const char* sources[1] = { source.data() };

    cl_int error = 0;
    cl_program program = clCreateProgramWithSource( context, 1, sources,
                                                    lengths, &error );
    _CheckError("Creating Program", error);

    return program;
  }


  /////////////////////////////////////////////////////////////////////////////
  cl_program _LoadProgramFromString(const char* code, cl_context context)
  {
    std::string source(code);

    // http://www.khronos.org/registry/cl/sdk/1.1/docs/man/xhtml/clCreateProgramWithSource.html
    size_t lengths[1] = { source.size() };
    const char* sources[1] = { source.data() };

    cl_int error = 0;
    cl_program program = clCreateProgramWithSource( context, 1, sources,
                                                    lengths, &error );
    _CheckError("Creating Program", error);

    return program;
  }


  /////////////////////////////////////////////////////////////////////////////
  Scalar _NormTukey( Scalar r, Scalar c )
  {
    const Scalar absr = fabs(r);
    const Scalar roc = r / c;
    const Scalar omroc2 = 1.0f - roc*roc;
    return (absr <= c ) ? omroc2*omroc2 : 0.0f;
  }

  /////////////////////////////////////////////////////////////////////////////
  Scalar _NormL1( Scalar r, Scalar /*c*/ )
  {
    const Scalar absr = fabs(r);
    return (absr == 0 ) ? 1.0f : 1.0f / absr;
  }


  /////////////////////////////////////////////////////////////////////////////
  calibu::CameraModelGeneric<Scalar> _ScaleCM(
      calibu::CameraModelGeneric<Scalar>  CamModel,
      unsigned int                        nLevel
      )
  {
    const float scale = 1.0f / (1 << nLevel);
    return CamModel.Scaled( scale );
  }


  /////////////////////////////////////////////////////////////////////////////
  inline unsigned int _GCD(unsigned int a, unsigned int b)
  {
    const unsigned int amodb = a%b;
    return amodb ? _GCD(b, amodb) : b;
  }

  /////////////////////////////////////////////////////////////////////////////
  static void CL_CALLBACK pfn_notify(
      const char *errinfo,
      const void *private_info,
      size_t cb,
      void *user_data)
  {
    PrintMessage(0, "CL Notify: %s\n", errinfo);
  }


  /////
  /////////////////////////////////////////////////////////////////////////////
public:
  unsigned int                PYRAMID_LEVELS;

private:
  cl_context                  m_ClContext;
  cl_program                  m_ClProgram;
  cl_kernel                   m_ClKernel;
  cl_command_queue            m_ClQueue;

  size_t                      m_WorkgroupSize;

  size_t                      m_nImageWidth;
  size_t                      m_nImageHeight;

  std::vector< cl_mem >       m_clLiveGreyPyr;
  std::vector< cl_mem >       m_clRefGreyPyr;
  std::vector< cl_mem >       m_clRefDepthPyr;
  std::vector< cv::Mat >                        m_LiveGreyPyr;
  std::vector< cv::Mat >                        m_RefGreyPyr;
  std::vector< cv::Mat >                        m_RefDepthPyr;

  std::vector< cl_mem >       m_clLiveGreyCM;
  std::vector< cl_mem >       m_clRefGreyCM;
  std::vector< cl_mem >       m_clRefDepthCM;
  cl_mem                      m_clTgd;      // reference camera's grey-depth transform
  cl_mem                      m_clTlr;

  cl_mem                      m_clLHS;
  cl_mem                      m_clRHS;
  cl_mem                      m_clError;
  std::vector<float>          m_Tmp;
  std::vector<float>          m_hLHS;
  std::vector<float>          m_hRHS;
  std::vector<float>          m_hError;
};
