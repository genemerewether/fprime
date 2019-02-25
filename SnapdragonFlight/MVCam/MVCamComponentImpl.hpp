// ======================================================================
// \title  MVCamComponentImpl.hpp
// \author tcanham
// \author mereweth
// \brief  hpp file for MVCam component implementation class
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

#ifndef MVCAM_MVCAMCOMPONENTIMPL_HPP
#define MVCAM_MVCAMCOMPONENTIMPL_HPP

#include <SnapdragonFlight/MVCam/MVCamComponentAc.hpp>
#include <SnapdragonFlight/MVCam/MVCamComponentImplCfg.hpp>

#include <Fw/Types/MemAllocator.hpp>
#include <Fw/Types/BasicTypes.hpp>
#include <Os/Mutex.hpp>

#include <Fw/Types/SerialBuffer.hpp>
#include <Os/LocklessQueue.hpp>

#include <time.h>

#ifdef BUILD_SDFLIGHT
// camera API Headers for the Snapdragon flight.
// Silence the camera warnings we can't fix
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreorder"
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#include "camera.h"
#include "camera_parameters.h"
#pragma GCC diagnostic pop

// Machine Vision SDK headers for Snapdragon flight.
#include "mv.h"
#include "mvCPA.h"
#endif //BUILD_SDFLIGHT

namespace SnapdragonFlight {

  class MVCamComponentImpl :
    public MVCamComponentBase
#ifdef BUILD_SDFLIGHT
    , private camera::ICameraListener
#endif //BUILD_SDFLIGHT
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object MVCam
      //!
      MVCamComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object MVCam
      //!
      void init(
          const NATIVE_INT_TYPE queueDepth, /*!< The queue depth*/
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object MVCam
      //!
      ~MVCamComponentImpl(void);

      // ----------------------------------------------------------------------
      // Public methods
      // ----------------------------------------------------------------------

      //! Give the class its memory. Should be called after constructor
      //! and init, but before task is spawned.
      void allocateBuffers(
          NATIVE_INT_TYPE identifier,
          Fw::MemAllocator& allocator,
          NATIVE_UINT_TYPE numHPImages, //!< The maximum number of high priority images
          NATIVE_UINT_TYPE numLPImages //!< The maximum number of low priority images
      );

      void deallocateBuffers(Fw::MemAllocator& allocator);

    PRIVATE:
      void parameterUpdated(FwPrmIdType id /*!< The parameter ID*/);
    
      void parametersLoaded();

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for Sched
      //!
      void Sched_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          NATIVE_UINT_TYPE context /*!< The call order*/
      );

      //! Handler implementation for WaypointRecv
      //!
      void WaypointRecv_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U32 waypoint /*!< Waypoint reporting*/
      );

      //! Handler implementation for GncBufferReturn
      //!
      void GncBufferReturn_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &fwBuffer
      );

      //! Handler implementation for ImageBufferIn
      //!
      void ImageBufferIn_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &fwBuffer
      );

      //!  \brief component ping handler
      //!
      //!  The ping handler responds to messages to verify that the task
      //!  is still executing. Will call output ping port
      //!
      //!  \param portNum the number of the incoming port.
      //!  \param key the key value that is returned with the ping response
      void pingIn_handler(NATIVE_INT_TYPE portNum, U32 key);

    PRIVATE:

      // ----------------------------------------------------------------------
      // Command handler implementations
      // ----------------------------------------------------------------------

      //! Implementation for MVCAM_SET_LOGGING command handler
      //! Turn on logging images to a file
      void MVCAM_SET_LOGGING_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq, /*!< The command sequence number*/
          LogMode Mode
      );

      //! Implementation for MVCAM_ADD_WAYPOINT command handler
      //! Add a waypoint
      void MVCAM_ADD_WAYPOINT_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq, /*!< The command sequence number*/
          U32 waypoint,
          WayptDestination destination,
          U8 quality /*!< Compression quality; between 1 and 100, inclusive*/
      );

      //! Implementation for MVCAM_TAKE_IMG command handler
      //! Take a still image
      void MVCAM_TAKE_IMG_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq, /*!< The command sequence number*/
          TakeImgDestination destination,
          U8 quality /*!< Compression quality; between 1 and 100, inclusive*/
      );

      //! Implementation for MVCAM_ACTIVATE command handler
      //! Turn imaging on or off
      void MVCAM_ACTIVATE_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq, /*!< The command sequence number*/
          MVCamActive Mode
      );

      //! Implementation for MVCAM_WAYPOINT_TEST command handler
      //! Manually trigger a waypoint image like port call would
      void MVCAM_WAYPOINT_TEST_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq, /*!< The command sequence number*/
          U32 index
      );

      //! Implementation for MVCAM_TEST_IMG command handler
      //! Send a still image test pattern
      void MVCAM_TEST_IMG_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq, /*!< The command sequence number*/
          TestImgDestination destination,
          U8 quality /*!< Compression quality; between 1 and 100, inclusive; greater than 100 is passthrough*/
      );

      //! Implementation for MVCAM_FLIGHT_MODE command handler
      //! Toggle frame notifications to GNC
      void MVCAM_FLIGHT_MODE_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq, /*!< The command sequence number*/
          MVCamFlightMode mode
      );

      //! Implementation for MVCAM_IMG_TLM_MODE command handler
      //! Toggle image telemetry
      void MVCAM_IMG_TLM_MODE_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq, /*!< The command sequence number*/
          MVCamImgTlmMode mode
      );

      //! Implementation for MVCAM_SET_LOG_SKIP command handler
      //!
      void MVCAM_SET_LOG_SKIP_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq, /*!< The command sequence number*/
          U32 skipCount
      );

    PRIVATE:

      // ----------------------------------------------------------------------
      // Declarations of methods inherited from base class
      // ----------------------------------------------------------------------

      //! Override preamble function to activate MVCam cycling
      //!
      void preamble(void);

      //! Override finalizer function to deactivate MVCam cycling
      //!
      void finalizer(void);

#ifdef BUILD_SDFLIGHT
      //! This function is called when an error is generated by the camera driver
      //!
      virtual void onError(void);

      //! This function is called when the camera driver generates a preview frame
      //!
      virtual void onPreviewFrame(camera::ICameraFrame *frame);
#endif //BUILD_SDFLIGHT

    PRIVATE:

      // ----------------------------------------------------------------------
      // Private methods
      // ----------------------------------------------------------------------

      //! Starts preview images cycling from the camera
      //!
      NATIVE_INT_TYPE activate(void);

      //! Stops preview images cycling from the camera
      //!
      NATIVE_INT_TYPE deactivate(void);

      //! Set callback type
      //!
      NATIVE_INT_TYPE setCallbackType(CallbackType type);

      const char* ISOToString(PostProcISO iso);

    PRIVATE:

      // ----------------------------------------------------------------------
      // Private variables
      // ----------------------------------------------------------------------

      enum PortOutType {
          MVCAM_OUT_MIN = 0,
          MVCAM_GNC_OUT = MVCAM_OUT_MIN,
          MVCAM_UNPROC_OUT = 1,
          MVCAM_PROC_OUT = 2,
          MVCAM_IMG_TLM_OUT = 3,
          MVCAM_OUT_MAX = 4
      };

      struct WaypointSet {
          U32 index;
          WayptDestination type;
          U8 quality;
          bool used;
      } m_waypointSet[MVCAM_MAX_NUM_WAYPOINTS];

      /*COMPILE_TIME_ASSERT(static_cast<NATIVE_INT_TYPE>(NUM_IMAGEBUFFERIN_INPUT_PORTS)
                          == static_cast<NATIVE_INT_TYPE>(NUM_UNPROCSEND_OUTPUT_PORTS)
                             + static_cast<NATIVE_INT_TYPE>(NUM_PROCSEND_OUTPUT_PORTS)
                             + static_cast<NATIVE_INT_TYPE>(NUM_IMAGETLMSEND_OUTPUT_PORTS),
                          NCAM_INOUT_BUFF_EQUAL);
      COMPILE_TIME_ASSERT(static_cast<NATIVE_INT_TYPE>(NCAM_OUT_MAX) ==
                          static_cast<NATIVE_INT_TYPE>(NUM_IMAGEBUFFERIN_INPUT_PORTS)
                          + static_cast<NATIVE_INT_TYPE>(NUM_GNCBUFFERRETURN_INPUT_PORTS),
                          NCAM_NUM_SEND_PORT_ENUMS);
      COMPILE_TIME_ASSERT(NCAM_IMG_MAX_NUM_BUFFERS >= NCAM_IMG_HP_BUFFER_POOL_SIZE,
                          NCAM_HP_BUFF_ENOUGH);
      COMPILE_TIME_ASSERT(NCAM_IMG_MAX_NUM_BUFFERS >= NCAM_IMG_LP_BUFFER_POOL_SIZE,
                          NCAM_LP_BUFF_ENOUGH);*/

      enum BufferPriority {
          MVCAM_HP_BUFFER = 0,
          MVCAM_LP_BUFFER = 1,
          MVCAM_PRIO_BUFFER_MAX = 2
      };

      struct BufferSet {
          Fw::Buffer readBuffer; //!< buffers for image reads
          bool available; //!< is buffer available?
      } m_buffSet[MVCAM_PRIO_BUFFER_MAX][MVCAM_IMG_MAX_NUM_BUFFERS];

      BYTE* m_buffMemLP;
      BYTE* m_buffMemHP;

      //! The allocator ID
      NATIVE_INT_TYPE m_allocatorId;

      U32 m_numGncBufferReturnQueueErrs;

      BYTE m_gncBufferReturnBuf[Fw::Buffer::SERIALIZED_SIZE];
      Fw::SerialBuffer m_gncBufferReturnSerialBuffObj;
      Os::LocklessQueue m_gncBufferReturnQueue;

#ifdef BUILD_SDFLIGHT
      //! Handle to Snapdragon camera
      //!
      camera::ICameraDevice* m_cameraPtr;

      //! Snapdragon camera parameters
      //!
      camera::CameraParams m_params;

      //! Camera autoexposure object pointer
      //!
      mvCPA* m_mvCPAPtr;

      //! Camera autoexposure parameters
      //!
      mvCPA_Configuration m_mvCPAConfig;
#endif //BUILD_SDFLIGHT

      //! Camera Manager ready for commands
      //!
      bool m_initialized;

      //! Camera Manager activated and images cycling
      //!
      bool m_activated;

      //! Number of images acquired
      //!
      NATIVE_UINT_TYPE m_imagesAcquired;

      //! Number of images acquired last sched call
      //!
      NATIVE_UINT_TYPE m_imagesAcquiredLast;

      //! Number of images acquired this time
      //!
      NATIVE_UINT_TYPE m_lpImagesAcquiredThisTime;

      //! Skip counter for low priority image frames
      //!
      NATIVE_UINT_TYPE m_lpImageSkipCount;

      //! Number of low priority image frames to skip
      //!
      NATIVE_UINT_TYPE m_lpImageSkip;

      //! Send the next camera frame to the uncompressed output port
      //!
      bool m_saveNextFrameUnproc;

      //! Send the next camera frame to the compressed output port
      //!
      bool m_saveNextFrameProc;

      //! Compression quality for next frame
      //!
      U8 m_compQuality;

      //! Ultimate destination of next processed frame
      //!
      Svc::FrameDestType m_camFrameDest;

      //! Stores logging state of MVCam
      //!
      LogMode m_logMode;

      //! Number of camera preview frames to skip before delivering image
      //!
      NATIVE_UINT_TYPE m_frameSkip;

      //! Number of camera preview frames to skip before delivering image
      //!
      NATIVE_UINT_TYPE m_frameCounter;

      //! 8-bit or 10-bit callback
      //!
      CallbackType m_callbackType;

      //! Fixed- or auto-exposure mode
      //!
      ExposureMode m_exposureMode;

      //! Exposure setpoint before roundoff, for telemetry
      //!
      float m_exposureTarget;

      //! Gain setpoint before roundoff, for telemetry
      //!
      float m_gainTarget;

      //! Exposure setpoint after roundoff, for telemetry
      //!
      U16 m_exposureTargetTrunc;

      //! Gain setpoint after roundoff, for telemetry
      //!
      U8 m_gainTargetTrunc;

      U16 m_exposureChangeThreshold;
      U8 m_gainChangeThreshold;
      U16 m_exposureMin;
      U16 m_exposureMax;
      U8 m_gainMin;
      U8 m_gainMax;

      U8 m_brightness;
      U8 m_contrast;
      U8 m_sharpness;
      PostProcISO m_iso;

      //! Mutex lock to protect save and cycle parameters
      //!
      Os::Mutex m_captureParamsLock;

      //! Mutex lock to protect image buffer set
      //!
      Os::Mutex m_bufferLock;

      //! Component was already activated when got image request
      //!
      bool m_wasActivated;

      //! Flag for checking if we were waiting on image
      //!
      bool m_waitingImage;

      //! Timeout while waiting on image (number of sched calls)
      //!
      U32 m_imageTimeout;

      //! True if the image capture was triggered by a command; false if waypoint triggered
      //!
      U32 m_cmdTriggered;

      //! Command opcode associated with waiting on image
      //!
      FwOpcodeType m_opCode;

      //! Command sequence associated with waiting on image
      //!
      U32 m_cmdSeq;

      //! Whether we are streaming image telemetry
      //!
      MVCamImgTlmMode m_imgTlmMode;

      //! Toggle frame notifications and images to GNC
      //!
      MVCamFlightMode m_flightMode;

      //! Max frame-to-frame change for exposure setpoint
      //!
      U32 m_maxExposureDelta;

      //! range for scaling exposure
      //!
      U32 m_exposureScale;

      //! range Max value for scaling gain
      //!
      U32 m_gainScale;

      U32 m_numNoHPBuffers;
      U32 m_numNoLPBuffers;
      U32 m_numLoRate;
      U32 m_numMaxExposureDelta;

      bool m_firstSched;

#ifndef BUILD_DARWIN
      NATIVE_UINT_TYPE m_printCounter;
      struct timeval m_tvLast;
#endif //BUILD_DARWIN
  };

} // end namespace SnapdragonFlight

#endif // #ifndef MVCAM_MVCAMCOMPONENTIMPL_HPP
