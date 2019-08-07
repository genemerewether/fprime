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
      ,m_dfsAlgorithm(MVDFS_MODE_ALG1_GPU)
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
    
#ifdef BUILD_SDFLIGHT
    memset(&m_camCfg, 0, sizeof(m_camCfg)); // important!
#endif
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
      this->m_mvDFSPtr = 
        mvDFS_Initialize(&m_camCfg,
                         m_dfsAlgorithm,
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

  void MVDFSComponentImpl ::
      parameterUpdated(FwPrmIdType id)
    {
        DEBUG_PRINT("prm %d updated\n", id);
        Fw::ParamValid valid;

        switch (id) {
            // default params
          
            case PARAMID_MVDFS_DFSALGORITHM:
            {
                U8 temp = paramGet_MVDFS_dfsAlgorithm(valid);
                if ((Fw::PARAM_VALID == valid) ||
                    (Fw::PARAM_DEFAULT == valid)) {
#ifdef BUILD_SDFLIGHT
                  m_dfsAlgorithm = (MVDFS_MODE) temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
                
            // defaults camera 0
                
            case PARAMID_MVDFS_CAM_0_PIXELWIDTH:
            {
                U32 temp = paramGet_MVDFS_cam_0_pixelWidth(valid);
                if ((Fw::PARAM_VALID == valid) ||
                    (Fw::PARAM_DEFAULT == valid)) {
#ifdef BUILD_SDFLIGHT
                    m_camCfg.camera[0].pixelWidth = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVDFS_CAM_0_PIXELHEIGHT:
            {
                U32 temp = paramGet_MVDFS_cam_0_pixelHeight(valid);
                if ((Fw::PARAM_VALID == valid) ||
                    (Fw::PARAM_DEFAULT == valid)) {
#ifdef BUILD_SDFLIGHT
                    m_camCfg.camera[0].pixelHeight = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVDFS_CAM_0_MEMORYSTRIDE:
            {
                U32 temp = paramGet_MVDFS_cam_0_memoryStride(valid);
                if ((Fw::PARAM_VALID == valid) ||
                    (Fw::PARAM_DEFAULT == valid)) {
#ifdef BUILD_SDFLIGHT
                    m_camCfg.camera[0].memoryStride = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVDFS_CAM_0_UVOFFSET:
            {
                U32 temp = paramGet_MVDFS_cam_0_uvOffset(valid);
                if ((Fw::PARAM_VALID == valid) ||
                    (Fw::PARAM_DEFAULT == valid)) {
#ifdef BUILD_SDFLIGHT
                    m_camCfg.camera[0].uvOffset = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVDFS_CAM_0_DISTORTIONMODEL:
            {
                U32 temp = paramGet_MVDFS_cam_0_distortionModel(valid);
                if ((Fw::PARAM_VALID == valid) ||
                    (Fw::PARAM_DEFAULT == valid)) {
#ifdef BUILD_SDFLIGHT
                    m_camCfg.camera[0].distortionModel = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;

            // defaults camera 1

            case PARAMID_MVDFS_CAM_1_PIXELWIDTH:
            {
                U32 temp = paramGet_MVDFS_cam_1_pixelWidth(valid);
                if ((Fw::PARAM_VALID == valid) ||
                    (Fw::PARAM_DEFAULT == valid)) {
#ifdef BUILD_SDFLIGHT
                    m_camCfg.camera[1].pixelWidth = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVDFS_CAM_1_PIXELHEIGHT:
            {
                U32 temp = paramGet_MVDFS_cam_1_pixelHeight(valid);
                if ((Fw::PARAM_VALID == valid) ||
                    (Fw::PARAM_DEFAULT == valid)) {
#ifdef BUILD_SDFLIGHT
                    m_camCfg.camera[1].pixelHeight = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVDFS_CAM_1_MEMORYSTRIDE:
            {
                U32 temp = paramGet_MVDFS_cam_1_memoryStride(valid);
                if ((Fw::PARAM_VALID == valid) ||
                    (Fw::PARAM_DEFAULT == valid)) {
#ifdef BUILD_SDFLIGHT
                    m_camCfg.camera[1].memoryStride = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVDFS_CAM_1_UVOFFSET:
            {
                U32 temp = paramGet_MVDFS_cam_1_uvOffset(valid);
                if ((Fw::PARAM_VALID == valid) ||
                    (Fw::PARAM_DEFAULT == valid)) {
#ifdef BUILD_SDFLIGHT
                    m_camCfg.camera[1].uvOffset = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVDFS_CAM_1_DISTORTIONMODEL:
            {
                U8 temp = paramGet_MVDFS_cam_1_distortionModel(valid);
                if ((Fw::PARAM_VALID == valid) ||
                    (Fw::PARAM_DEFAULT == valid)) {
#ifdef BUILD_SDFLIGHT
                    m_camCfg.camera[1].distortionModel = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;

            // intrinsics camera 0
                
            case PARAMID_MVDFS_CAM_0_PRINCIPALPOINT_U:
            {
                F64 temp = paramGet_MVDFS_cam_0_principalPoint_u(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    m_camCfg.camera[0].principalPoint[0] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVDFS_CAM_0_PRINCIPALPOINT_V:
            {
                F64 temp = paramGet_MVDFS_cam_0_principalPoint_v(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    m_camCfg.camera[0].principalPoint[1] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVDFS_CAM_0_FOCALLENGTH_U:
            {
                F64 temp = paramGet_MVDFS_cam_0_focalLength_u(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    m_camCfg.camera[0].focalLength[0] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVDFS_CAM_0_FOCALLENGTH_V:
            {
                F64 temp = paramGet_MVDFS_cam_0_focalLength_v(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    m_camCfg.camera[0].focalLength[1] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
                
            case PARAMID_MVDFS_CAM_0_DISTORTION_0:
            {
                F64 temp = paramGet_MVDFS_cam_0_distortion_0(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    m_camCfg.camera[0].distortion[0] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVDFS_CAM_0_DISTORTION_1:
            {
                F64 temp = paramGet_MVDFS_cam_0_distortion_1(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    m_camCfg.camera[0].distortion[1] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVDFS_CAM_0_DISTORTION_2:
            {
                F64 temp = paramGet_MVDFS_cam_0_distortion_2(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    m_camCfg.camera[0].distortion[2] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVDFS_CAM_0_DISTORTION_3:
            {
                F64 temp = paramGet_MVDFS_cam_0_distortion_3(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    m_camCfg.camera[0].distortion[3] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;

            // intrinsics camera 1
                
            case PARAMID_MVDFS_CAM_1_PRINCIPALPOINT_U:
            {
                F64 temp = paramGet_MVDFS_cam_1_principalPoint_u(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    m_camCfg.camera[1].principalPoint[0] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVDFS_CAM_1_PRINCIPALPOINT_V:
            {
                F64 temp = paramGet_MVDFS_cam_1_principalPoint_v(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    m_camCfg.camera[1].principalPoint[1] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVDFS_CAM_1_FOCALLENGTH_U:
            {
                F64 temp = paramGet_MVDFS_cam_1_focalLength_u(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    m_camCfg.camera[1].focalLength[0] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVDFS_CAM_1_FOCALLENGTH_V:
            {
                F64 temp = paramGet_MVDFS_cam_1_focalLength_v(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    m_camCfg.camera[1].focalLength[1] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
                
            case PARAMID_MVDFS_CAM_1_DISTORTION_0:
            {
                F64 temp = paramGet_MVDFS_cam_1_distortion_0(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    m_camCfg.camera[1].distortion[0] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVDFS_CAM_1_DISTORTION_1:
            {
                F64 temp = paramGet_MVDFS_cam_1_distortion_1(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    m_camCfg.camera[1].distortion[1] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVDFS_CAM_1_DISTORTION_2:
            {
                F64 temp = paramGet_MVDFS_cam_1_distortion_2(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    m_camCfg.camera[1].distortion[2] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVDFS_CAM_1_DISTORTION_3:
            {
                F64 temp = paramGet_MVDFS_cam_1_distortion_3(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    m_camCfg.camera[1].distortion[3] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;

            // extrinsics
                
            case PARAMID_MVDFS_TRANS_X:
            {
                F32 temp = paramGet_MVDFS_trans_x(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    m_camCfg.translation[0] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVDFS_TRANS_Y:
            {
                F32 temp = paramGet_MVDFS_trans_y(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    m_camCfg.translation[1] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVDFS_TRANS_Z:
            {
                F32 temp = paramGet_MVDFS_trans_z(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    m_camCfg.translation[2] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVDFS_ROT_X:
            {
                F32 temp = paramGet_MVDFS_rot_x(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    m_camCfg.rotation[0] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVDFS_ROT_Y:
            {
                F32 temp = paramGet_MVDFS_rot_y(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    m_camCfg.rotation[1] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVDFS_ROT_Z:
            {
                F32 temp = paramGet_MVDFS_rot_z(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    m_camCfg.rotation[2] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
        }
    }

    void MVDFSComponentImpl ::
      parametersLoaded()
    {
        for (U32 i = 0; i < __MAX_PARAMID; i++) {
            parameterUpdated(i);
        }
    }
    
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
