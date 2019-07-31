// ====================================================================== 
// \title  MVDFSImpl.cpp
// \author mereweth
// \brief  cpp file for MVDFS component implementation class
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged.
//
// ======================================================================


#include <SnapdragonFlight/MVDFS/MVDFSComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

#include <Os/File.hpp>

//#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
#define DEBUG_PRINT(x,...)

namespace SnapdragonFlight {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction 
  // ----------------------------------------------------------------------

  MVDFSComponentImpl ::
#if FW_OBJECT_NAMES == 1
    MVDFSComponentImpl(
        const char *const compName
    ) :
      MVDFSComponentBase(compName)
#else
    MVDFSComponentImpl(void)
#endif
#ifdef BUILD_SDFLIGHT
      ,m_mvDFSPtr(NULL)
      ,m_depthCameraIntrinsics()
      ,m_camCfg()
#endif
      ,m_invDepth(NULL)
      ,m_pointCloud(NULL)
      ,m_initialized(false)
      ,m_activated(false)
      ,m_imagesProcessed(0u)
      ,m_imagesProcessedLast(0u)
  {

  }

  void MVDFSComponentImpl ::
    init(
        const NATIVE_INT_TYPE queueDepth,
        const NATIVE_INT_TYPE instance
    ) 
  {
    MVDFSComponentBase::init(queueDepth, instance);
  }

  MVDFSComponentImpl ::
    ~MVDFSComponentImpl(void)
  {

  }

  void MVDFSComponentImpl ::
    preamble(void)
  {
      initHelper();
      m_invDepth = new float[640 * 480];
      m_pointCloud = new float[640 * 480 * 3];
      if ((NULL == m_invDepth) ||
          (NULL == m_pointCloud)) {
          // TODO(mereweth) - EVR
          m_initialized = false;
      }
  }

  void MVDFSComponentImpl ::
    initHelper(void)
  {
#ifdef BUILD_SDFLIGHT
      memset(&m_camCfg, 0, sizeof(m_camCfg)); // important!
      m_camCfg.camera[0].pixelWidth = 640;
      m_camCfg.camera[0].pixelHeight = 480;
      // NOTE(mereweth) - saves the copy to separate joined rows
      m_camCfg.camera[0].memoryStride = 1280;
      m_camCfg.camera[1].pixelWidth = 640;
      m_camCfg.camera[1].pixelHeight = 480;
      // NOTE(mereweth) - saves the copy to separate joined rows
      m_camCfg.camera[1].memoryStride = 1280;

      m_camCfg.camera[0].principalPoint[0] = 327.25028370506277;
      m_camCfg.camera[0].principalPoint[1] = 240.52747319394973;
      m_camCfg.camera[0].focalLength[0] = 434.0397908576731;
      m_camCfg.camera[0].focalLength[1] = 433.8185119766046;
      m_camCfg.camera[0].uvOffset = 0;
      m_camCfg.camera[0].distortionModel = 4;
      m_camCfg.camera[0].distortion[0] = 0.010214217031416842;
      m_camCfg.camera[0].distortion[1] = -0.04651010371555157;
      m_camCfg.camera[0].distortion[2] = -0.0017014074768636185;
      m_camCfg.camera[0].distortion[3] = 0.0013863895250307325;

      m_camCfg.camera[1].principalPoint[0] = 320.34588249858376;
      m_camCfg.camera[1].principalPoint[1] = 244.4353351783047;
      m_camCfg.camera[1].focalLength[0] = 436.7596303746942;
      m_camCfg.camera[1].focalLength[1] = 436.4226057245768;
      m_camCfg.camera[1].uvOffset = 0;
      m_camCfg.camera[1].distortionModel = 4;
      m_camCfg.camera[1].distortion[0] = 0.0014070626511820305;
      m_camCfg.camera[1].distortion[1] = -0.03208074310964145;
      m_camCfg.camera[1].distortion[2] = -0.0021471073082785313;
      m_camCfg.camera[1].distortion[3] = 0.0008696973644813725;
      
      m_camCfg.translation[0] = -0.08057393152744473;
      m_camCfg.translation[1] = 0.0028380369090344037;
      m_camCfg.translation[2] = -0.00025007902285991877;
      // scaled axis-angle
      m_camCfg.rotation[0] = -0.0022546;
      m_camCfg.rotation[1] = -0.0086814;
      m_camCfg.rotation[2] = -0.0749294;

      this->m_mvDFSPtr = 
        mvDFS_Initialize(&m_camCfg,
                         MVDFS_MODE_ALG1_GPU,
                         false); //using 10bit input
      if (NULL == m_mvDFSPtr) {
          // TODO(mereweth) - EVR
          return;
      }

      memset(&m_depthCameraIntrinsics, 0, sizeof(m_depthCameraIntrinsics));
      mvDFS_GetDepthCameraConfiguration(m_mvDFSPtr, &m_depthCameraIntrinsics);

      m_initialized = true;
#endif //BUILD_SDFLIGHT
  }

  void MVDFSComponentImpl ::
    finalizer(void)
  {
      m_initialized = false;
#ifdef BUILD_SDFLIGHT
      if (NULL != m_mvDFSPtr) {
          mvDFS_Deinitialize(m_mvDFSPtr);
      }
#endif //BUILD_SDFLIGHT
      
      if (NULL != m_invDepth) {
          delete[] m_invDepth;
      }
      
      if (NULL != m_pointCloud) {
          delete[] m_pointCloud;
      }
  }
  
  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void MVDFSComponentImpl ::
    ImageIn_handler(
        const NATIVE_INT_TYPE portNum,
        ROS::sensor_msgs::Image &Image
    )
  {
      Fw::Buffer data = Image.getdata();
      if (m_initialized && m_activated) {
          this->m_imagesProcessed++;
#ifdef BUILD_SDFLIGHT
          mvDFS_GetDepths(m_mvDFSPtr,
                          (const U8*) (data.getdata()),
                          (const U8*) (data.getdata() + 640),
                          0, nullptr, //numMasks, masks
                          0, 32, //min, max disparity
                          NULL, // don't need raw disparity values yet
                          m_invDepth);

          // generate point cloud and send
          const U32 width = m_camCfg.camera[0].pixelWidth;
          const float fx = m_depthCameraIntrinsics.focalLength[0];
          const float fy = m_depthCameraIntrinsics.focalLength[1];
          const float cx = m_depthCameraIntrinsics.principalPoint[0];
          const float cy = m_depthCameraIntrinsics.principalPoint[1];
          U32 idx = 0u;
          U32 numPoints = 0u;
          if (this->isConnected_PointCloud_OutputPort(0)) {
              using namespace ROS::sensor_msgs;
              for (int i = 0; i < m_camCfg.camera[0].pixelHeight; i++) {
                  for (int j = 0; j < width; j++) {
                      const float d = m_invDepth[i * width + j];
                      if (d < 1e-3) {
                          continue;
                      }
                      const float z = 1.0 / d;
                      const float x = z * (j - cx) / fx;
                      const float y = z * (i - cy) / fy;
                      memcpy(&m_pointCloud[idx++], (const void*) &x, sizeof(x));
                      memcpy(&m_pointCloud[idx++], (const void*) &y, sizeof(y));
                      memcpy(&m_pointCloud[idx++], (const void*) &z, sizeof(z));

                      numPoints++;
                  }
              }
              // NOTE(mereweth) - REQUIRES SYNC PORT on other end - true for HLRosIface
              // TODO(mereweth) - get cycling buffers ?
              Fw::Buffer data(0, 0, (U64) m_pointCloud, numPoints * 3 * sizeof(F32));
              PointField fields[3];
              // NOTE(Mereweth) - have to keep pointfield type in sync manually
              fields[0] = PointField("x", 0, 7, 1);
              fields[1] = PointField("y", sizeof(F32), 7, 1);
              fields[2] = PointField("z", 2 * sizeof(F32), 7, 1);
              PointCloud2 pc2 = PointCloud2(Image.getheader(),
                                            1, numPoints,
                                            fields, FW_NUM_ARRAY_ELEMENTS(fields), FW_NUM_ARRAY_ELEMENTS(fields),
                                            0, // little endian -> is_bigendian
                                            3 * sizeof(F32), // 3x F32
                                            numPoints * 3 * sizeof(F32), // unorganized pointcloud
                                            data,
                                            1 /* is_dense */);

              PointCloud_out(0, pc2);
          }
#endif //BUILD_SDFLIGHT
      }
      
      ImageBufferReturn_out(0, data);
  }

  void MVDFSComponentImpl ::
    sched_handler(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
      NATIVE_UINT_TYPE imageCountDiff = m_imagesProcessed - m_imagesProcessedLast;
      this->tlmWrite_MVDFS_ImageCount(this->m_imagesProcessed);
      this->tlmWrite_MVDFS_ImageRate(imageCountDiff);
      m_imagesProcessedLast = m_imagesProcessed;
  }

  void MVDFSComponentImpl ::
    pingIn_handler(
        const NATIVE_INT_TYPE portNum,
        U32 key
    )
  {
      this->pingOut_out(0, key);
  }




  // TODO(kedelson) --- start2


  void MVDFSComponentImpl ::
      parameterUpdated(FwPrmIdType id)
    {
        DEBUG_PRINT("prm %d updated\n", id);
        Fw::ParamValid valid;

        switch (id) {

            case PARAMID_DFSALGORITHM:
            {
                U8 temp = paramGet_dfsAlgorithm(valid);
                if ((Fw::PARAM_VALID != valid) && (Fw::PARAM_DEFAULT != valid)) {
                    // TODO(mereweth) - issue EVR
                    return;
                }
                if (m_initialized) {
                    m_captureParamsLock.lock();
                }
                m_dfsAlgorithm = temp;
                if (m_initialized) {
                    m_captureParamsLock.unLock();
                }
            }
                break;



            case PARAMID_CAM_0_PIXWIDTH:
            {
                U32 temp = paramGet_cam_0_pixWidth(valid);
                if ((Fw::PARAM_VALID != valid) && (Fw::PARAM_DEFAULT != valid)) {
                    // TODO(mereweth) - issue EVR
                    return;
                }
                if (m_initialized) {
                    m_captureParamsLock.lock();
                }
                m_cam_0_pixWidth = temp;
                if (m_initialized) {
                    m_captureParamsLock.unLock();
                }
            }
                break;

            case PARAMID_CAM_0_PIXHEIGHT:
            {
                U32 temp = paramGet_cam_0_pixHeight(valid);
                if ((Fw::PARAM_VALID != valid) && (Fw::PARAM_DEFAULT != valid)) {
                    // TODO(mereweth) - issue EVR
                    return;
                }
                if (m_initialized) {
                    m_captureParamsLock.lock();
                }
                m_cam_0_pixHeight = temp;
                if (m_initialized) {
                    m_captureParamsLock.unLock();
                }
            }
                break;

            case PARAMID_CAM_0_MEMORYSTRIDE:
            {
                U32 temp = paramGet_cam_0_memoryStride(valid);
                if ((Fw::PARAM_VALID != valid) && (Fw::PARAM_DEFAULT != valid)) {
                    // TODO(mereweth) - issue EVR
                    return;
                }
                if (m_initialized) {
                    m_captureParamsLock.lock();
                }
                m_cam_0_memoryStride = temp;
                if (m_initialized) {
                    m_captureParamsLock.unLock();
                }
            }
                break;

            case PARAMID_CAM_0_PRINCIPALPOINT_0:
            {
                F32 temp = paramGet_cam_0_principalPoint_0(valid);
                if ((Fw::PARAM_VALID != valid) && (Fw::PARAM_DEFAULT != valid)) {
                    // TODO(mereweth) - issue EVR
                    return;
                }
                if (m_initialized) {
                    m_captureParamsLock.lock();
                }
                m_cam_0_principalPoint_0 = temp;
                if (m_initialized) {
                    m_captureParamsLock.unLock();
                }
            }
                break;

            case PARAMID_CAM_0_PRINCIPALPOINT_1:
            {
                F32 temp = paramGet_cam_0_principalPoint_1(valid);
                if ((Fw::PARAM_VALID != valid) && (Fw::PARAM_DEFAULT != valid)) {
                    // TODO(mereweth) - issue EVR
                    return;
                }
                if (m_initialized) {
                    m_captureParamsLock.lock();
                }
                m_cam_0_principalPoint_1 = temp;
                if (m_initialized) {
                    m_captureParamsLock.unLock();
                }
            }
                break;

            case PARAMID_CAM_0_FOCALLENGTH_0:
            {
                F32 temp = paramGet_cam_0_focalLength_0(valid);
                if ((Fw::PARAM_VALID != valid) && (Fw::PARAM_DEFAULT != valid)) {
                    // TODO(mereweth) - issue EVR
                    return;
                }
                if (m_initialized) {
                    m_captureParamsLock.lock();
                }
                m_cam_0_focalLength_0 = temp;
                if (m_initialized) {
                    m_captureParamsLock.unLock();
                }
            }
                break;

            case PARAMID_CAM_0_FOCALLENGTH_1:
            {
                F32 temp = paramGet_cam_0_focalLength_1(valid);
                if ((Fw::PARAM_VALID != valid) && (Fw::PARAM_DEFAULT != valid)) {
                    // TODO(mereweth) - issue EVR
                    return;
                }
                if (m_initialized) {
                    m_captureParamsLock.lock();
                }
                m_cam_0_focalLength_1 = temp;
                if (m_initialized) {
                    m_captureParamsLock.unLock();
                }
            }
                break;

            case PARAMID_CAM_0_UVOFFSET:
            {
                U8 temp = paramGet_cam_0_uvOffset(valid);
                if ((Fw::PARAM_VALID != valid) && (Fw::PARAM_DEFAULT != valid)) {
                    // TODO(mereweth) - issue EVR
                    return;
                }
                if (m_initialized) {
                    m_captureParamsLock.lock();
                }
                m_cam_0_uvOffset = temp;
                if (m_initialized) {
                    m_captureParamsLock.unLock();
                }
            }
                break;

            case PARAMID_CAM_0_DISTORTIONMODEL:
            {
                U8 temp = paramGet_cam_0_distortionModel(valid);
                if ((Fw::PARAM_VALID != valid) && (Fw::PARAM_DEFAULT != valid)) {
                    // TODO(mereweth) - issue EVR
                    return;
                }
                if (m_initialized) {
                    m_captureParamsLock.lock();
                }
                m_cam_0_distortionModel = temp;
                if (m_initialized) {
                    m_captureParamsLock.unLock();
                }
            }
                break;

            case PARAMID_CAM_0_DISTORT_0:
            {
                F32 temp = paramGet_cam_0_distort_0(valid);
                if ((Fw::PARAM_VALID != valid) && (Fw::PARAM_DEFAULT != valid)) {
                    // TODO(mereweth) - issue EVR
                    return;
                }
                if (m_initialized) {
                    m_captureParamsLock.lock();
                }
                m_cam_0_distort_0 = temp;
                if (m_initialized) {
                    m_captureParamsLock.unLock();
                }
            }
                break;

            case PARAMID_CAM_0_DISTORT_1:
            {
                F32 temp = paramGet_cam_0_distort_1(valid);
                if ((Fw::PARAM_VALID != valid) && (Fw::PARAM_DEFAULT != valid)) {
                    // TODO(mereweth) - issue EVR
                    return;
                }
                if (m_initialized) {
                    m_captureParamsLock.lock();
                }
                m_cam_0_distort_1 = temp;
                if (m_initialized) {
                    m_captureParamsLock.unLock();
                }
            }
                break;

            case PARAMID_CAM_0_DISTORT_2:
            {
                F32 temp = paramGet_cam_0_distort_2(valid);
                if ((Fw::PARAM_VALID != valid) && (Fw::PARAM_DEFAULT != valid)) {
                    // TODO(mereweth) - issue EVR
                    return;
                }
                if (m_initialized) {
                    m_captureParamsLock.lock();
                }
                m_cam_0_distort_2 = temp;
                if (m_initialized) {
                    m_captureParamsLock.unLock();
                }
            }
                break;

            case PARAMID_CAM_0_DISTORT_3:
            {
                F32 temp = paramGet_cam_0_distort_3(valid);
                if ((Fw::PARAM_VALID != valid) && (Fw::PARAM_DEFAULT != valid)) {
                    // TODO(mereweth) - issue EVR
                    return;
                }
                if (m_initialized) {
                    m_captureParamsLock.lock();
                }
                m_cam_0_distort_3 = temp;
                if (m_initialized) {
                    m_captureParamsLock.unLock();
                }
            }
                break;

            // CAMERA 1 -----------------------------------------

            case PARAMID_CAM_1_PIXWIDTH:
            {
                U32 temp = paramGet_cam_1_pixWidth(valid);
                if ((Fw::PARAM_VALID != valid) && (Fw::PARAM_DEFAULT != valid)) {
                    // TODO(mereweth) - issue EVR
                    return;
                }
                if (m_initialized) {
                    m_captureParamsLock.lock();
                }
                m_cam_1_pixWidth = temp;
                if (m_initialized) {
                    m_captureParamsLock.unLock();
                }
            }
                break;

            case PARAMID_CAM_1_PIXHEIGHT:
            {
                U32 temp = paramGet_cam_1_pixHeight(valid);
                if ((Fw::PARAM_VALID != valid) && (Fw::PARAM_DEFAULT != valid)) {
                    // TODO(mereweth) - issue EVR
                    return;
                }
                if (m_initialized) {
                    m_captureParamsLock.lock();
                }
                m_cam_1_pixHeight = temp;
                if (m_initialized) {
                    m_captureParamsLock.unLock();
                }
            }
                break;

            case PARAMID_CAM_1_MEMORYSTRIDE:
            {
                U32 temp = paramGet_cam_1_memoryStride(valid);
                if ((Fw::PARAM_VALID != valid) && (Fw::PARAM_DEFAULT != valid)) {
                    // TODO(mereweth) - issue EVR
                    return;
                }
                if (m_initialized) {
                    m_captureParamsLock.lock();
                }
                m_cam_1_memoryStride = temp;
                if (m_initialized) {
                    m_captureParamsLock.unLock();
                }
            }
                break;

            case PARAMID_CAM_1_PRINCIPALPOINT_0:
            {
                F32 temp = paramGet_cam_1_principalPoint_0(valid);
                if ((Fw::PARAM_VALID != valid) && (Fw::PARAM_DEFAULT != valid)) {
                    // TODO(mereweth) - issue EVR
                    return;
                }
                if (m_initialized) {
                    m_captureParamsLock.lock();
                }
                m_cam_1_principalPoint_0 = temp;
                if (m_initialized) {
                    m_captureParamsLock.unLock();
                }
            }
                break;

            case PARAMID_CAM_1_PRINCIPALPOINT_1:
            {
                F32 temp = paramGet_cam_1_principalPoint_1(valid);
                if ((Fw::PARAM_VALID != valid) && (Fw::PARAM_DEFAULT != valid)) {
                    // TODO(mereweth) - issue EVR
                    return;
                }
                if (m_initialized) {
                    m_captureParamsLock.lock();
                }
                m_cam_1_principalPoint_1 = temp;
                if (m_initialized) {
                    m_captureParamsLock.unLock();
                }
            }
                break;

            case PARAMID_CAM_1_FOCALLENGTH_0:
            {
                F32 temp = paramGet_cam_1_focalLength_0(valid);
                if ((Fw::PARAM_VALID != valid) && (Fw::PARAM_DEFAULT != valid)) {
                    // TODO(mereweth) - issue EVR
                    return;
                }
                if (m_initialized) {
                    m_captureParamsLock.lock();
                }
                m_cam_1_focalLength_0 = temp;
                if (m_initialized) {
                    m_captureParamsLock.unLock();
                }
            }
                break;

            case PARAMID_CAM_1_FOCALLENGTH_1:
            {
                F32 temp = paramGet_cam_1_focalLength_1(valid);
                if ((Fw::PARAM_VALID != valid) && (Fw::PARAM_DEFAULT != valid)) {
                    // TODO(mereweth) - issue EVR
                    return;
                }
                if (m_initialized) {
                    m_captureParamsLock.lock();
                }
                m_cam_1_focalLength_1 = temp;
                if (m_initialized) {
                    m_captureParamsLock.unLock();
                }
            }
                break;

            case PARAMID_CAM_1_UVOFFSET:
            {
                U8 temp = paramGet_cam_1_uvOffset(valid);
                if ((Fw::PARAM_VALID != valid) && (Fw::PARAM_DEFAULT != valid)) {
                    // TODO(mereweth) - issue EVR
                    return;
                }
                if (m_initialized) {
                    m_captureParamsLock.lock();
                }
                m_cam_1_uvOffset = temp;
                if (m_initialized) {
                    m_captureParamsLock.unLock();
                }
            }
                break;

            case PARAMID_CAM_1_DISTORTIONMODEL:
            {
                U8 temp = paramGet_cam_1_distortionModel(valid);
                if ((Fw::PARAM_VALID != valid) && (Fw::PARAM_DEFAULT != valid)) {
                    // TODO(mereweth) - issue EVR
                    return;
                }
                if (m_initialized) {
                    m_captureParamsLock.lock();
                }
                m_cam_1_distortionModel = temp;
                if (m_initialized) {
                    m_captureParamsLock.unLock();
                }
            }
                break;

            case PARAMID_CAM_1_DISTORT_0:
            {
                F32 temp = paramGet_cam_1_distort_0(valid);
                if ((Fw::PARAM_VALID != valid) && (Fw::PARAM_DEFAULT != valid)) {
                    // TODO(mereweth) - issue EVR
                    return;
                }
                if (m_initialized) {
                    m_captureParamsLock.lock();
                }
                m_cam_1_distort_0 = temp;
                if (m_initialized) {
                    m_captureParamsLock.unLock();
                }
            }
                break;

            case PARAMID_CAM_1_DISTORT_1:
            {
                F32 temp = paramGet_cam_1_distort_1(valid);
                if ((Fw::PARAM_VALID != valid) && (Fw::PARAM_DEFAULT != valid)) {
                    // TODO(mereweth) - issue EVR
                    return;
                }
                if (m_initialized) {
                    m_captureParamsLock.lock();
                }
                m_cam_1_distort_1 = temp;
                if (m_initialized) {
                    m_captureParamsLock.unLock();
                }
            }
                break;

            case PARAMID_CAM_1_DISTORT_2:
            {
                F32 temp = paramGet_cam_1_distort_2(valid);
                if ((Fw::PARAM_VALID != valid) && (Fw::PARAM_DEFAULT != valid)) {
                    // TODO(mereweth) - issue EVR
                    return;
                }
                if (m_initialized) {
                    m_captureParamsLock.lock();
                }
                m_cam_1_distort_2 = temp;
                if (m_initialized) {
                    m_captureParamsLock.unLock();
                }
            }
                break;

            case PARAMID_CAM_1_DISTORT_3:
            {
                F32 temp = paramGet_cam_1_distort_3(valid);
                if ((Fw::PARAM_VALID != valid) && (Fw::PARAM_DEFAULT != valid)) {
                    // TODO(mereweth) - issue EVR
                    return;
                }
                if (m_initialized) {
                    m_captureParamsLock.lock();
                }
                m_cam_1_distort_3 = temp;
                if (m_initialized) {
                    m_captureParamsLock.unLock();
                }
            }
                break;


            case PARAMID_TRANS_0:
            {
                F32 temp = paramGet_trans_0(valid);
                if ((Fw::PARAM_VALID != valid) && (Fw::PARAM_DEFAULT != valid)) {
                    // TODO(mereweth) - issue EVR
                    return;
                }
                if (m_initialized) {
                    m_captureParamsLock.lock();
                }
                m_trans_0 = temp;
                if (m_initialized) {
                    m_captureParamsLock.unLock();
                }
            }
                break;

            case PARAMID_TRANS_1:
            {
                F32 temp = paramGet_trans_1(valid);
                if ((Fw::PARAM_VALID != valid) && (Fw::PARAM_DEFAULT != valid)) {
                    // TODO(mereweth) - issue EVR
                    return;
                }
                if (m_initialized) {
                    m_captureParamsLock.lock();
                }
                m_trans_1 = temp;
                if (m_initialized) {
                    m_captureParamsLock.unLock();
                }
            }
                break;

            case PARAMID_TRANS_2:
            {
                F32 temp = paramGet_trans_2(valid);
                if ((Fw::PARAM_VALID != valid) && (Fw::PARAM_DEFAULT != valid)) {
                    // TODO(mereweth) - issue EVR
                    return;
                }
                if (m_initialized) {
                    m_captureParamsLock.lock();
                }
                m_trans_2 = temp;
                if (m_initialized) {
                    m_captureParamsLock.unLock();
                }
            }
                break;

            case PARAMID_ROT_0:
            {
                F32 temp = paramGet_rot_0(valid);
                if ((Fw::PARAM_VALID != valid) && (Fw::PARAM_DEFAULT != valid)) {
                    // TODO(mereweth) - issue EVR
                    return;
                }
                if (m_initialized) {
                    m_captureParamsLock.lock();
                }
                m_rot_0 = temp;
                if (m_initialized) {
                    m_captureParamsLock.unLock();
                }
            }
                break;

            case PARAMID_ROT_1:
            {
                F32 temp = paramGet_rot_1(valid);
                if ((Fw::PARAM_VALID != valid) && (Fw::PARAM_DEFAULT != valid)) {
                    // TODO(mereweth) - issue EVR
                    return;
                }
                if (m_initialized) {
                    m_captureParamsLock.lock();
                }
                m_rot_1 = temp;
                if (m_initialized) {
                    m_captureParamsLock.unLock();
                }
            }
                break;

            case PARAMID_ROT_2:
            {
                F32 temp = paramGet_rot_2(valid);
                if ((Fw::PARAM_VALID != valid) && (Fw::PARAM_DEFAULT != valid)) {
                    // TODO(mereweth) - issue EVR
                    return;
                }
                if (m_initialized) {
                    m_captureParamsLock.lock();
                }
                m_rot_2 = temp;
                if (m_initialized) {
                    m_captureParamsLock.unLock();
                }
            }
                break;


  
    void MVCamComponentImpl ::
      parametersLoaded()
    {
        for (U32 i = 0; i < __MAX_PARAMID; i++) {
            parameterUpdated(i);
        }
    }


  // TODO(kedelson) --- end2







  // ----------------------------------------------------------------------
  // Command handler implementations
  // ----------------------------------------------------------------------

  void MVDFSComponentImpl ::
    MVDFS_Reinit_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq
    )
  {
#ifdef BUILD_SDFLIGHT
      if (NULL != m_mvDFSPtr) {
          mvDFS_Deinitialize(m_mvDFSPtr);
      }
#endif //BUILD_SDFLIGHT
      m_initialized = false;
      initHelper();
      if (m_initialized) {
          this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_OK);
      }
      else {
          this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_EXECUTION_ERROR);
      }
  }

  void MVDFSComponentImpl ::
    MVDFS_Activate_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq,
        bool enable
    )
  {
      if (enable) {
          if (m_initialized) {
              m_activated = true;
              this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_OK);
          }
          else {
              this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_EXECUTION_ERROR);
          }
      }
      else {
          m_activated = false;
    	  this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_OK);
      }
  }

} // end namespace SnapdragonFlight
