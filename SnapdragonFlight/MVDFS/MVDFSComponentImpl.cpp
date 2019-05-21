// ====================================================================== 
// \title  MVDFSImpl.cpp
// \author mereweth
// \brief  cpp file for MVDFS component implementation class
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged. Any commercial use must be negotiated with the Office
// of Technology Transfer at the California Institute of Technology.
// 
// This software may be subject to U.S. export control laws and
// regulations.  By accepting this document, the user agrees to comply
// with all U.S. export laws and regulations.  User has the
// responsibility to obtain export licenses, or other export authority
// as may be required before exporting such information to foreign
// countries or providing access to foreign persons.
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
    MVDFSImpl(void)
#endif
#ifdef BUILD_SDFLIGHT
      ,m_mvDFSPtr(NULL)
#endif
      ,m_initialized(false)
      ,m_activated(false)
      ,m_imagesProcessed(0u)
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
  }


  void MVDFSComponentImpl ::
    initHelper(void)
  {
#ifdef BUILD_SDFLIGHT
      mvStereoConfiguration camCfg;
      memset(&camCfg, 0, sizeof(camCfg)); // important!
      camCfg.camera[0].pixelWidth = 640;
      camCfg.camera[0].pixelHeight = 480;
      // NOTE(mereweth) - saves the copy to separate joined rows
      camCfg.camera[0].memoryStride = 1280;
      camCfg.camera[1].pixelWidth = 640;
      camCfg.camera[1].pixelHeight = 480;
      // NOTE(mereweth) - saves the copy to separate joined rows
      camCfg.camera[1].memoryStride = 1280;

      camCfg.camera[0].principalPoint[0] = 320.0;
      camCfg.camera[0].principalPoint[1] = 240.0;
      camCfg.camera[0].focalLength[0] = 275.0;
      camCfg.camera[0].focalLength[1] = 275.0;
      camCfg.camera[0].uvOffset = 0;
      camCfg.camera[0].distortionModel = 10;
      camCfg.camera[0].distortion[0] = .003908;
      camCfg.camera[0].distortion[1] = -0.009574;
      camCfg.camera[0].distortion[2] = 0.010173;
      camCfg.camera[0].distortion[3] = -0.003329;

      camCfg.camera[1].principalPoint[0] = 320.0;
      camCfg.camera[1].principalPoint[1] = 240.0;
      camCfg.camera[1].focalLength[0] = 275.0;
      camCfg.camera[1].focalLength[1] = 275.0;
      camCfg.camera[1].uvOffset = 0;
      camCfg.camera[1].distortionModel = 10;
      camCfg.camera[1].distortion[0] = .003908;
      camCfg.camera[1].distortion[1] = -0.009574;
      camCfg.camera[1].distortion[2] = 0.010173;
      camCfg.camera[1].distortion[3] = -0.003329;
      
      camCfg.translation[0] = 0.0;
      camCfg.translation[1] = 0.0;
      camCfg.translation[2] = 0.0;
      // scaled axis-angle
      camCfg.rotation[0] = 0.0;
      camCfg.rotation[1] = 0.0;
      camCfg.rotation[2] = 0.0;

      this->m_mvDFSPtr = 
        mvDFS_Initialize(&camCfg,
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
                          NULL); // TODO(mereweth) - get inverse depth and convert to point cloud
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
      this->tlmWrite_MVDFS_ImageCount(this->m_imagesProcessed);
  }

  void MVDFSComponentImpl ::
    pingIn_handler(
        const NATIVE_INT_TYPE portNum,
        U32 key
    )
  {
      this->pingOut_out(0, key);
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
