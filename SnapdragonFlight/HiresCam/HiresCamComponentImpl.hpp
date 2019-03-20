// ======================================================================
// \title  HiresCamComponentImpl.hpp
// \author tcanham
// \author mereweth
// \brief  hpp file for HiresCam component implementation class
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

#ifndef HIRESCAM_HIRESCAMCOMPONENTIMPL_HPP
#define HIRESCAM_HIRESCAMCOMPONENTIMPL_HPP

#include <SnapdragonFlight/HiresCam/HiresCamComponentAc.hpp>
#include <SnapdragonFlight/HiresCam/HiresCamComponentImplCfg.hpp>

#include <Fw/Types/MemAllocator.hpp>
#include <Fw/Types/BasicTypes.hpp>
#include <Os/Mutex.hpp>

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
#endif //BUILD_SDFLIGHT

namespace SnapdragonFlight {

  class HiresCamComponentImpl :
    public HiresCamComponentBase
#ifdef BUILD_SDFLIGHT
    , private camera::ICameraListener
#endif //BUILD_SDFLIGHT
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object HiresCam
      //!
      HiresCamComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object HiresCam
      //!
      void init(
          const NATIVE_INT_TYPE queueDepth, /*!< The queue depth*/
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      int spawnChild(); //!< forks another process

      //! Destroy object HiresCam
      //!
      ~HiresCamComponentImpl(void);

      // ----------------------------------------------------------------------
      // Public methods
      // ----------------------------------------------------------------------

      //! Give the class its memory. Should be called after constructor
      //! and init, but before task is spawned.
      void allocateBuffers(
          NATIVE_INT_TYPE identifier,
          Fw::MemAllocator& allocator,
          // TODO(mereweth) - reduce size of buffers
          NATIVE_UINT_TYPE numBuffers //!< The maximum number of buffers
      );

      void deallocateBuffers(Fw::MemAllocator& allocator);

    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //!  \brief component ping handler
      //!
      //!  The ping handler responds to messages to verify that the task
      //!  is still executing. Will call output ping port
      //!
      //!  \param portNum the number of the incoming port.
      //!  \param key the key value that is returned with the ping response
      void pingIn_handler(NATIVE_INT_TYPE portNum, U32 key);

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

      //! Handler implementation for BufferReturn
      //!
      void BufferReturn_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &fwBuffer
      );

    PRIVATE:

      // ----------------------------------------------------------------------
      // Command handler implementations
      // ----------------------------------------------------------------------

      //! Implementation for HIRESCAM_SET_LOGGING command handler
      //! Turn on logging images to a file
      void HIRESCAM_SET_LOGGING_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq, /*!< The command sequence number*/
          LogMode Mode
      );

      //! Implementation for HIRESCAM_ADD_WAYPOINT command handler
      //! Add a waypoint
      void HIRESCAM_ADD_WAYPOINT_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq, /*!< The command sequence number*/
          U32 waypoint,
          WayptDestination destination,
          WayptAction action,
          WayptCaptureType type,
          U8 quality /*!< Compression quality; between 1 and 100, inclusive; greater than 100 is passthrough*/
      );

      //! Implementation for HIRESCAM_TAKE_IMG command handler
      //! Take a still image
      void HIRESCAM_TAKE_IMG_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq, /*!< The command sequence number*/
          TakeImgDestination destination,
          ImgType type,
          U8 quality /*!< Compression quality; between 1 and 100, inclusive; greater than 100 is passthrough*/
      );

      //! Implementation for HIRESCAM_TAKE_VIDEO command handler
      //! Take a video
      void HIRESCAM_TAKE_VIDEO_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq, /*!< The command sequence number*/
          VidType setting,
          U32 frames
      );

      //! Implementation for HIRESCAM_WAYPOINT_TEST command handler
      //! Manually trigger a waypoint image like port call would
      void HIRESCAM_WAYPOINT_TEST_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq, /*!< The command sequence number*/
          U32 index
      );

      //! Implementation for HIRESCAM_SET_VID_TIMEOUTS command handler
      //! Set frame count and elapsed time timeouts for video
      void HIRESCAM_SET_VID_TIMEOUTS_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq, /*!< The command sequence number*/
          U32 frames,
          U32 sched
      );

      //! Implementation for HIRESCAM_TEST_IMG command handler
      //! Send a still image test pattern
      void HIRESCAM_TEST_IMG_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq, /*!< The command sequence number*/
          TestImgDestination destination,
          TestImgType type,
          U8 quality /*!< Compression quality; between 1 and 100, inclusive; greater than 100 is passthrough*/
      );

      //! Implementation for HIRESCAM_SET_VID_SKIP command handler
      //! Set frame skip count for video, e.g. 2 delivers every 3rd frame
      void HIRESCAM_SET_VID_SKIP_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq, /*!< The command sequence number*/
          U32 skip
      );

      //! Implementation for HIRESCAM_ACTIVATE command handler
      //! Turn imaging on or off
      void HIRESCAM_ACTIVATE_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq, /*!< The command sequence number*/
          HiresCamActive Mode
      );


    PRIVATE:

      // ----------------------------------------------------------------------
      // Declarations of methods inherited from base class
      // ----------------------------------------------------------------------

      //! Override preamble function to activate HiresCam cycling
      //!
      void preamble(void);

      //! Override finalizer function to deactivate HiresCam cycling
      //!
      void finalizer(void);

#ifdef BUILD_SDFLIGHT
      //! This function is called when an error is generated by the camera driver
      //!
      virtual void onError(void);

      //! This function is called when the camera driver generates a preview frame
      //!
      virtual void onPreviewFrame(camera::ICameraFrame *frame);


      //! This function is called when the camera driver generates a preview frame
      //!
      virtual void onPictureFrame(camera::ICameraFrame *frame);


      //! This function is called when the camera driver generates a preview frame
      //!
      virtual void onVideoFrame(camera::ICameraFrame *frame);
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

    PRIVATE:

      // ----------------------------------------------------------------------
      // Private variables
      // ----------------------------------------------------------------------

      enum PortOutType {
          HIRESCAM_OUT_MIN = 0,
          HIRESCAM_UNPROC_OUT = HIRESCAM_OUT_MIN,
          HIRESCAM_PROC_OUT = 1,
          HIRESCAM_OUT_MAX = 2
      };

      struct WaypointSet {
          U32 index;
          WayptDestination dest;
          WayptAction action;
          ImgType stillType;
          VidType vidType;
          U8 quality;
          bool used;
      } m_waypointSet[HIRESCAM_MAX_NUM_WAYPOINTS];

      /*COMPILE_TIME_ASSERT(static_cast<NATIVE_INT_TYPE>(NUM_BUFFERRETURN_INPUT_PORTS)
                          == static_cast<NATIVE_INT_TYPE>(NUM_UNPROCSEND_OUTPUT_PORTS)
                             + static_cast<NATIVE_INT_TYPE>(NUM_PROCSEND_OUTPUT_PORTS),
                          HIRESCAM_INOUT_BUFF_EQUAL);
      COMPILE_TIME_ASSERT(static_cast<NATIVE_INT_TYPE>(HIRESCAM_OUT_MAX) ==
                          static_cast<NATIVE_INT_TYPE>(NUM_BUFFERRETURN_INPUT_PORTS),
                          HIRESCAM_NUM_SEND_PORT_ENUMS);
      COMPILE_TIME_ASSERT(HIRESCAM_MAX_NUM_BUFFERS >= HIRESCAM_IMG_HP_BUFFER_POOL_SIZE,
                          HIRESCAM_HP_BUFF_ENOUGH);
      COMPILE_TIME_ASSERT(HIRESCAM_MAX_NUM_BUFFERS >= HIRESCAM_IMG_LP_BUFFER_POOL_SIZE,
                          HIRESCAM_LP_BUFF_ENOUGH);*/

      struct BufferSet {
          Fw::Buffer readBuffer; //!< buffers for image reads
          bool available; //!< is buffer available?
      } m_buffSet[1/*NUM_BUFFERSEND_OUTPUT_PORTS*/][HIRESCAM_MAX_NUM_BUFFERS];

      BYTE* m_buffMem;

      //! The allocator ID
      NATIVE_INT_TYPE m_allocatorId;

#ifdef BUILD_SDFLIGHT
      //! Handle to Snapdragon camera
      //!
      camera::ICameraDevice* m_cameraPtr;

      //! Snapdragon camera parameters
      //!
      camera::CameraParams m_params;
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

      //! Number of video frames acquired overall
      //!
      NATIVE_UINT_TYPE m_framesAcquired;

      //! Number of video frames acquired overall - from last loop
      //!
      NATIVE_UINT_TYPE m_framesAcquiredLast;

      //! Number of video frames acquired this time
      //!
      NATIVE_UINT_TYPE m_framesAcquiredThisTime;

      //! Skip counter for video frames
      //!
      NATIVE_UINT_TYPE m_frameSkipCount;

      //! Number of video frames to skip
      //!
      NATIVE_UINT_TYPE m_frameSkip;

      //! Number of frames acquired at which to stop recording
      //!
      NATIVE_UINT_TYPE m_framesStopRecording;

      //! Waypoint-triggered video number of frames to stop recording at
      //!
      NATIVE_UINT_TYPE m_framesStopRecordingWaypt;

      //! Waypoint-triggered video number of sched calls to stop recording at
      //!
      NATIVE_UINT_TYPE m_numSchedStopRecordingWaypt;

      //! Send the next camera frame to the uncompressed output port
      //!
      bool m_saveNextFrameUnproc;

      //! Send the next camera frame to the compressed output port
      //!
      bool m_saveNextFrameProc;

      //! Image type for current capture
      //!
      ImgType m_stillType;

      //! Video type for current capture
      //!
      VidType m_vidType;

      //! Compression quality for next frame
      //!
      U8 m_compQuality;

      //! Ultimate destination of next processed frame
      //!
      Svc::FrameDestType m_camFrameDest;

      //! Stores logging state of HiresCam
      //!
      LogMode m_logMode;

      //! Mutex lock to protect save and cycle parameters
      //!
      Os::Mutex m_captureParamsLock;

      //! Mutex lock to protect image buffer set
      //!
      Os::Mutex m_bufferLock;

      //! Keep track of what type of work we are doing
      //!
      enum HiresMode {
          HIRES_IDLE,
          HIRES_PRE_WAIT_IMAGE,
          HIRES_WAIT_IMAGE,
          HIRES_GOT_IMAGE,
          HIRES_PRE_WAIT_FIRST_VIDEO,
          HIRES_WAIT_FIRST_VIDEO,
          HIRES_WAIT_LAST_VIDEO,
          HIRES_GOT_LAST_VIDEO,
          HIRES_DEACTIVATE_WAIT,
          HIRES_DISCARD_IMAGE
      } m_currentMode;

      bool m_wasActivated;

      //! Timeout while waiting on API callback (number of sched calls)
      //!
      U32 m_callbackTimeout;

      //! True if the image capture was triggered by a command; false if waypoint triggered
      //!
      bool m_cmdTriggered;

      //! True if we have a pending error for when we reply to a command
      //!
      bool m_cmdErrorOnDeactivate;

      //! Command opcode associated with waiting on callback
      //!
      FwOpcodeType m_opCode;

      //! Command sequence associated with waiting on callback
      //!
      U32 m_cmdSeq;

#ifndef BUILD_DARWIN
      NATIVE_UINT_TYPE m_printCounter;
      struct timeval m_tvLast;
#endif //BUILD_DARWIN
  };

} // end namespace SnapdragonFlight

#endif // #ifndef HIRESCAM_HIRESCAMCOMPONENTIMPL_HPP
