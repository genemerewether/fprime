// ======================================================================
// \title  MVCamComponentImplSdFlight.cpp
// \author tcanham
// \author mereweth
// \brief  cpp file for MVCam component implementation class on Snapdragon
//
// \copyright
// Copyright 2009-2017, by the California Institute of Technology.
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


#include <SnapdragonFlight/MVCam/MVCamComponentImpl.hpp>
#include <SnapdragonFlight/MVCam/MVCamComponentImplCfg.hpp>

#include <Svc/ActiveFileLogger/ActiveFileLoggerPacket.hpp>
#include <Svc/ActiveFileLogger/ActiveFileLoggerStreams.hpp>

#include <Fw/Types/BasicTypes.hpp>
#include <Fw/Types/Assert.hpp>
#include <Os/Mutex.hpp>

#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <stdlib.h>
#include <errno.h>

#include <stdio.h> // TODO(mereweth@jpl.nasa.gov) - remove the debug prints
#include <sys/time.h>

// Snapdragon Flight libcamera
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

//#define DEBUG_MODE

#ifdef DEBUG_MODE
#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
#else
#define DEBUG_PRINT(x,...)
#endif //#ifdef DEBUG_MODE

#define MVCAM_TRUNC 1 // truncate 10bit -> 8bit
#define MVCAM_SCALE 0 // linearly scale 10bit -> 8bit with rounding

#define MVCAM_TEST_NAV_AOI 0

namespace SnapdragonFlight {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  MVCamComponentImpl ::
    ~MVCamComponentImpl(void)
  {
    m_initialized = false;
  }

  // ----------------------------------------------------------------------
  // Private method implementations
  // ----------------------------------------------------------------------

  void MVCamComponentImpl ::
    preamble(void)
  {
      if (m_initialized) {
          DEBUG_PRINT("Already initialized in preamble\n");
          this->log_WARNING_HI_MVCAM_AlreadyInitializedError();
          return;
      }

      NATIVE_INT_TYPE cameraID;
      NATIVE_INT_TYPE stat;

      NATIVE_INT_TYPE numCameras = camera::getNumberOfCameras();
      if (numCameras < 1) {
          DEBUG_PRINT("Not enough cameras in preamble\n");
          this->log_WARNING_HI_MVCAM_CameraError(MVCAM_NO_CAMERA);
          return;
      }

#ifndef SOC_8096
      bool found = false;

      for (NATIVE_INT_TYPE i = 0; i < numCameras; i++) {
          camera::CameraInfo cameraInfo;
          stat = camera::getCameraInfo(i, cameraInfo);
          if (stat) {
              DEBUG_PRINT("Failed to get camera info %d, stat %d in preamble\n", i, stat);
              this->log_WARNING_HI_MVCAM_CameraError(MVCAM_NO_CAMERA);
              return;
          }

          if (cameraInfo.func == static_cast<NATIVE_INT_TYPE>(MVCAM_CAMERA_TYPE))
          {
              cameraID = i;
              found = true;
          }
      }

      if(found != true) {
          DEBUG_PRINT("Failed to find camera in preamble\n");
          this->log_WARNING_HI_MVCAM_CameraError(MVCAM_NO_CAMERA);
          return;
      }
#else
        cameraID = static_cast<NATIVE_INT_TYPE>(MVCAM_CAMERA_TYPE);
#endif

      stat = camera::ICameraDevice::createInstance(cameraID, &m_cameraPtr);
      if ((stat != 0) ||
      (m_cameraPtr == NULL)) {
          DEBUG_PRINT("Failed to create camera instance in preamble\n");
          this->log_WARNING_HI_MVCAM_CameraError(MVCAM_NO_CAMERA);
          return;
      }

      stat = m_params.init(m_cameraPtr);
      if (stat) {
          camera::ICameraDevice::deleteInstance(&m_cameraPtr);
          DEBUG_PRINT("Failed to init params in preamble\n");
          this->log_WARNING_HI_MVCAM_CameraError(MVCAM_PARAM_ERR);
          return;
      }

      const camera::ImageSize imageSize = camera::ImageSize(
          static_cast<NATIVE_UINT_TYPE>(MVCAM_IMAGE_WIDTH),
          static_cast<NATIVE_UINT_TYPE>(MVCAM_IMAGE_HEIGHT));
      m_params.setPictureSize(imageSize);

      stat = setCallbackType(m_callbackType);
      if (stat) {
          camera::ICameraDevice::deleteInstance(&m_cameraPtr);
          DEBUG_PRINT("Failed to set callback type in preamble\n");
          this->log_WARNING_HI_MVCAM_CameraError(MVCAM_PARAM_ERR);
          return;
      }

      // Except for Sharpness, these are the default values
      m_params.setBrightness(m_brightness);
      m_params.setContrast(m_contrast);
      m_params.setISO(ISOToString(m_iso));
      //params_.setWhiteBalance("auto"); // no significant difference
      m_params.setSharpness(m_sharpness);

      // TODO(mereweth@jpl.nasa.gov) - this seems to have no effect
      const camera::Range fpsRange = camera::Range(30000, 30000, 0);
      m_params.setPreviewFpsRange(fpsRange);

      /* Set how many times faster than preview should video frames be delivered
      * We do not activate video callbacks anyway
      */
      m_params.setVideoFPS(camera::VIDEO_FPS_1X);

      stat = m_params.commit();
      if (stat) {
          camera::ICameraDevice::deleteInstance(&m_cameraPtr);
          DEBUG_PRINT("Failed to commit params in preamble\n");
          this->log_WARNING_HI_MVCAM_CameraError(MVCAM_PARAM_ERR);
          return;
      }

      // rest of parameters set in parametersUpdated call
      m_mvCPAConfig.width  = static_cast<NATIVE_UINT_TYPE>(MVCAM_AOI_WIDTH);
      m_mvCPAConfig.height = static_cast<NATIVE_UINT_TYPE>(MVCAM_AOI_HEIGHT);

      m_mvCPAPtr = mvCPA_Initialize(&m_mvCPAConfig);
      if (m_mvCPAPtr == NULL) {
          DEBUG_PRINT("MV CPA init error\n");
          this->log_WARNING_HI_MVCAM_CameraError(MVCAM_CPA_ERR);
          return;
      }

      DEBUG_PRINT("Initialized MVCam with raw size %s\n", m_params.get("raw-size").c_str());

      /*FW_ASSERT(m_cameraPtr != NULL);
      m_cameraPtr->addListener(this);*/

      m_initialized = true;

#ifdef ALWAYS_CYCLE
      (void) activate();
#endif //ALWAYS_CYCLE
  }

  void MVCamComponentImpl ::
    finalizer(void)
  {
      if (m_activated) {
          DEBUG_PRINT("Deactivating MVCam in finalizer\n");
          deactivate();
      }

      if(m_cameraPtr != NULL) {
          /*FW_ASSERT(m_cameraPtr);
          m_cameraPtr->removeListener(this);*/
          //delete the camera ptr
          camera::ICameraDevice::deleteInstance(&m_cameraPtr);
          m_cameraPtr = NULL;
      }


      if(m_mvCPAPtr != NULL) {
          DEBUG_PRINT("Deinitializing MV SDK\n");
          mvCPA_Deinitialize(m_mvCPAPtr);
          m_mvCPAPtr = NULL;
      }

      // TODO(mereweth@jpl.nasa.gov) - when are we guaranteed not to get more callbacks?
      m_initialized = false;
  }

  NATIVE_INT_TYPE MVCamComponentImpl ::
    activate(void)
  {
      NATIVE_INT_TYPE stat;

      // we never call our private methods at incorrect times
      FW_ASSERT(m_initialized);
      FW_ASSERT(!m_activated);

      // clear throttles
      this->log_WARNING_HI_MVCAM_NoHPBuffers_ThrottleClear();
      this->log_ACTIVITY_HI_MVCAM_NoLPBuffers_ThrottleClear();
      this->log_WARNING_HI_MVCAM_LoRate_ThrottleClear();
      this->log_WARNING_HI_MVCAM_MaxExposureDelta_ThrottleClear();

      this->m_firstSched = true;

      // add this as a listener.
      FW_ASSERT(m_cameraPtr != NULL);
      m_cameraPtr->addListener(this);
      DEBUG_PRINT("\nIn activate, before startPreview");
      stat = m_cameraPtr->startPreview();
      DEBUG_PRINT("\nIn activate, after startPreview");
      if (stat) {
          m_cameraPtr->removeListener(this);
          m_cameraPtr->stopPreview();
          this->log_WARNING_HI_MVCAM_CameraError(MVCAM_UNKNOWN_ERR);
          return stat;
      }

#ifdef DEBUG_MODE
      struct timeval tv;
      gettimeofday(&tv,NULL);
      DEBUG_PRINT("\nActivating MVCamComponentImpl at time %f\n",
                  tv.tv_sec + tv.tv_usec / 1000000.0);
#endif

      m_captureParamsLock.lock();
      // reset the counter; use the next frame that the callback returns
      m_frameCounter = m_frameSkip;
      m_activated = true;
      m_captureParamsLock.unLock();

      this->log_ACTIVITY_HI_MVCAM_Activated();
      return 0;
  }

  NATIVE_INT_TYPE MVCamComponentImpl ::
    deactivate(void)
  {
      // we never call our private methods at incorrect times
      FW_ASSERT(m_initialized);
      FW_ASSERT(m_activated);

      // prevent further callbacks from being triggered
      FW_ASSERT(m_cameraPtr != NULL);
      m_cameraPtr->removeListener(this);

      m_activated = false;
      /* TODO(mereweth@jpl.nasa.gov) - stopPreview must be called as soon as
       * no callbacks are active, and before any more start
       */

      //TODO(mereweth@jpl.nasa.gov) - do we need to wait for deactivation?
      /*NATIVE_UINT_TYPE skipUS = 1000 * 1000 * (m_frameSkip + 1) / 30.0;
      usleep(skipUS);*/
      FW_ASSERT(m_cameraPtr != NULL);
      m_cameraPtr->stopPreview();

#ifdef DEBUG_MODE
      struct timeval tv;
      gettimeofday(&tv,NULL);
      DEBUG_PRINT("\nDeactivating MVCamComponentImpl at time %f\n",
      tv.tv_sec + tv.tv_usec / 1000000.0);
#endif

      this->log_ACTIVITY_HI_MVCAM_Deactivated();

      return 0;
  }

  NATIVE_INT_TYPE MVCamComponentImpl ::
    setCallbackType(CallbackType type)
  {
      switch (type) {
          case CALLBACK_8BIT:
              /* NOTE(mereweth@jpl.nasa.gov) - we only save MVCAM_IMAGE_SIZE bytes;
              * The numbers here indicate the size of the buffer we receive, which will
              * have extra, empty channels
              */
              m_params.set("preview-format", "yuv420sp"); //462848
              //m_params.set("preview-format", "nv12-venus"); //475136

              // even though we do not use takePicture callback, NEED to set this
              m_params.set("picture-format", "jpeg");
              break;
          case CALLBACK_10BIT:
              /* With these settings, raw image is 640 * 480 * 10 / 8 + 1024
              * 10-bit raw, and 1024 bits of footer to align to 4K bytes
              */

              m_params.set("preview-format", "bayer-rggb");
              // even though we do not use takePicture callback, NEED to set this
              m_params.set("picture-format", "bayer-mipi-10gbrg");
              m_params.set("raw-size", "640x480");
              break;
          default:
              DEBUG_PRINT("Unhandled camera callback type %d\n", type);
              FW_ASSERT(0, type);
              return -1;
      }
      return m_params.commit();
  }

  // ----------------------------------------------------------------------
  // ICameraListener interface method implementations
  // ----------------------------------------------------------------------

  void MVCamComponentImpl ::
    onError(void)
  {
      this->log_WARNING_HI_MVCAM_CameraError(MVCAM_UNKNOWN_ERR);
  }

  void MVCamComponentImpl ::
    onPreviewFrame(camera::ICameraFrame *frame)
  {
      if (!m_activated) {
          this->log_WARNING_LO_MVCAM_ExtraPreviewCallbackError();
          DEBUG_PRINT("\nonPreviewFrame called while MVCam (de)activating\n");
          return;
      }

      NATIVE_INT_TYPE stat;

      NATIVE_UINT_TYPE callbackSize = 0u;
      switch (m_callbackType) {
          case CALLBACK_8BIT:
              callbackSize = static_cast<NATIVE_UINT_TYPE>(MVCAM_8BIT_CALLBACK_SIZE);
              break;
          case CALLBACK_10BIT:
              callbackSize = static_cast<NATIVE_UINT_TYPE>(MVCAM_10BIT_CALLBACK_SIZE);
              break;
          default:
              DEBUG_PRINT("Unhandled camera callback type %d\n", m_callbackType);
              FW_ASSERT(0, m_callbackType);
              return;
      }

      if ((frame == NULL)              ||
          (frame->size < callbackSize) ||
          (frame->data == NULL))        {
          this->log_WARNING_HI_MVCAM_CameraError(MVCAM_IMG_EMPTY);
          DEBUG_PRINT("Empty image in onPreviewFrame callback");
          return;
      }

      Fw::Time currTime = this->getTime();

#ifdef DEBUG_MODE
      struct timeval tv;
      gettimeofday(&tv,NULL);
      struct timeval tvStart = tv; // for elapsed time calculation at end of callback
      DEBUG_PRINT("\nPreview callback %d at time %f; size %u",
                  m_printCounter,
                  tv.tv_sec + tv.tv_usec / 1000000.0, frame->size);
      if (m_printCounter != 0) {
          struct timeval diff;
          timersub(&tv, &m_tvLast, &diff);
          DEBUG_PRINT(";\nelapsed time since last frame %f\n",
                      diff.tv_sec + diff.tv_usec / 1000000.0);
      }
#endif //#ifdef DEBUG_MODE

      // correct for exposure time - report middle of frame
      // NOTE(mereweth) - assumes that capture params from last callback have
      // taken effect; may be more delay than that
#ifdef SOC_8074
      uint64_t timestamp_ns = frame->timeStamp
        - m_exposureTargetTrunc * MVCAM_DEFAULT_ROW_HALF_PERIOD_NS;
#else
      uint64_t timestamp_ns = frame->timeStamp
        + m_params.getFrameExposureTime(frame) / 2;
#endif
      
      struct timespec monotonic_now;
      clock_gettime(CLOCK_MONOTONIC, &monotonic_now);

      struct timespec realtime_now;
      clock_gettime(CLOCK_REALTIME, &realtime_now);

      // TODO(mereweth) - update this when image struct changes
      F64 timestamp_realtime = (F64) (/*frame->timeStamp*/ timestamp_ns + realtime_now.tv_nsec - monotonic_now.tv_nsec) * 1.0e-9
            + (F64) realtime_now.tv_sec - (F64) monotonic_now.tv_sec;

      DEBUG_PRINT("\ncorrection %.9f, timestamp_realtime (s) %.9f\n",
                  0.0/*timestamp_ns*/, timestamp_realtime);

#ifdef DEBUG_MODE
      gettimeofday(&tv,NULL);
      DEBUG_PRINT("\nTaking m_captureParamsLock in preview callback %d at time %f\n",
                  m_printCounter, tv.tv_sec + tv.tv_usec / 1000000.0);
#endif //#ifdef DEBUG_MODE

      m_captureParamsLock.lock();

      // pull HP buffers of the lockless queue
      bool exit_loop = false;
      while (!exit_loop) {
          const NATIVE_INT_TYPE bufSize = Fw::Buffer::SERIALIZED_SIZE;
          BYTE buf[bufSize];
          NATIVE_INT_TYPE size;
          Os::Queue::QueueStatus queueStat;
          queueStat = m_gncBufferReturnQueue.Receive(buf, bufSize, size);

          switch (queueStat) {
              case Os::Queue::QUEUE_OK: {
                  Fw::SerialBuffer gncBufferReturnDeser(buf, bufSize);
                  Fw::SerializeStatus serStat;
                  // "fill" the SerialBuffer
                  serStat = gncBufferReturnDeser.setBuffLen(static_cast<NATIVE_INT_TYPE>(size));
                  Fw::Buffer gncBufferReturnData;
                  serStat = gncBufferReturnDeser.deserialize(gncBufferReturnData);
                  FW_ASSERT(Fw::FW_SERIALIZE_OK == serStat, serStat);

                  // NOTE(mereweth) always high-priority type; low-priority type is async handler
                  const NATIVE_UINT_TYPE buffType = MVCAM_HP_BUFFER;
                  bool found = false;
                  // search for open entry
                  for (NATIVE_UINT_TYPE entry = 0; entry < MVCAM_IMG_MAX_NUM_BUFFERS; entry++) {
                      // Look for slots to fill. "Available" is from
                      // the perspective of the driver thread looking for
                      // buffers to fill, so add the buffer and make it available.
                      if (not this->m_buffSet[buffType][entry].available) {
                          FW_ASSERT(buffType < FW_NUM_ARRAY_ELEMENTS(this->m_buffSet), buffType);
                          FW_ASSERT(entry < FW_NUM_ARRAY_ELEMENTS(this->m_buffSet[buffType]), entry);
                          this->m_buffSet[buffType][entry].readBuffer = gncBufferReturnData;
                          this->m_buffSet[buffType][entry].available = true;
                          found = true;
                          break;
                      }
                  }
                  FW_ASSERT(found,gncBufferReturnData.getbufferID(),gncBufferReturnData.getmanagerID());
                  continue;
              }
              case Os::Queue::QUEUE_NO_MORE_MSGS: {
                  exit_loop = true;
                  break;
              }
              default: {
                  FW_ASSERT(0, queueStat);
                  return;
              }
          }
      }

      if (m_imagesAcquired % (MVCAM_PARAMS_APPLY_SKIP+1) == 0) {
          float cpaExposure = -1.0f;
          float cpaGain = -1.0f;

          NATIVE_UINT_TYPE tempExposureTrunc = 0u;
          NATIVE_UINT_TYPE tempGainTrunc = 0u;

          FW_ASSERT(m_mvCPAPtr != NULL, 0);
          DEBUG_PRINT("Running autoexposure and setting params\n");
          mvCPA_AddFrame(m_mvCPAPtr,
                         static_cast<uint8_t *>(frame->data)
                         + static_cast<NATIVE_UINT_TYPE>(MVCAM_AOI_START_ROW * MVCAM_IMAGE_WIDTH
                             * ((CALLBACK_10BIT == m_callbackType) ? 10 / 8 : 1)
                             + MVCAM_AOI_START_COL),
                         static_cast<NATIVE_UINT_TYPE>(MVCAM_AOI_STRIDE));

          mvCPA_GetValues(m_mvCPAPtr, &cpaExposure, &cpaGain);
          cpaExposure = m_exposureMin + cpaExposure * m_exposureScale;
          cpaGain     = m_gainMin + cpaGain * m_gainScale;

          /* MV autoexposure should never return negative values, and our
           * calculation should never scale them to become negative
           */
          if ((cpaExposure < m_exposureMin)  ||
              (cpaExposure > m_exposureMax) ||
              (cpaGain < m_gainMin)          ||
              (cpaGain > m_gainMax))         {

              this->log_WARNING_HI_MVCAM_CameraError(MVCAM_CPA_ERR);
              //TODO(mereweth) - set floating point telemetry here to see what went wrong?
          }
          else {
              tempExposureTrunc = round(cpaExposure);
              tempGainTrunc     = round(cpaGain);

              if (m_exposureMode != EXPOSURE_FIXED) {
                  if (tempExposureTrunc > m_exposureTargetTrunc + m_maxExposureDelta) {
                      if (m_flightMode == MVCAM_MODE_FLIGHT) {
                          ++m_numMaxExposureDelta;
                          this->log_WARNING_HI_MVCAM_MaxExposureDelta(m_exposureTargetTrunc,
                                                                     tempExposureTrunc);

                          tempExposureTrunc = m_exposureTargetTrunc + m_maxExposureDelta;
                      }
                  }
                  else if (tempExposureTrunc + m_maxExposureDelta < m_exposureTargetTrunc) {
                      if (m_flightMode == MVCAM_MODE_FLIGHT) {
                          ++m_numMaxExposureDelta;
                          this->log_WARNING_HI_MVCAM_MaxExposureDelta(m_exposureTargetTrunc,
                                                                     tempExposureTrunc);

                          tempExposureTrunc = m_exposureTargetTrunc - m_maxExposureDelta;
                      }
                  }
              }

              if ((abs(tempExposureTrunc - m_exposureTargetTrunc) >=
                   m_exposureChangeThreshold) ||
                  (abs(tempGainTrunc - m_gainTargetTrunc) >=
                   m_gainChangeThreshold)) {

                  DEBUG_PRINT("Exposure diff %d\n", abs(tempExposureTrunc - m_exposureTargetTrunc));
                  if (m_exposureMode != EXPOSURE_FIXED) {
                      m_params.setManualExposure(static_cast<NATIVE_INT_TYPE>(tempExposureTrunc));
                      m_params.setManualGain(static_cast<NATIVE_INT_TYPE>(tempGainTrunc));
                      stat = m_params.commit();
                      if (stat) {
                          //DEBUG_PRINT("Unable to commit exposure and gain params in onPreviewFrame\n");
                          this->log_WARNING_HI_MVCAM_CameraError(MVCAM_PARAM_ERR);
                      }
                      else {
                          m_exposureTargetTrunc = tempExposureTrunc;
                          m_gainTargetTrunc     = tempGainTrunc;
                          m_exposureTarget      = cpaExposure;
                          m_gainTarget          = cpaGain;
                      }
                  }
              }
          }
      }

      // Truncate (when using 10bit callback) before AOI test and image send on ports
      if (m_callbackType == CALLBACK_10BIT) {
          NATIVE_UINT_TYPE src = 0;
          for (NATIVE_UINT_TYPE dst = 0; dst < MVCAM_IMAGE_SIZE; dst+=4) {
#if MVCAM_TRUNC
              frame->data[dst]   = frame->data[src];
              frame->data[dst+1] = frame->data[src+1];
              frame->data[dst+2] = frame->data[src+2];
              frame->data[dst+3] = frame->data[src+3];
#elif MVCAM_SCALE
              NATIVE_UINT_TYPE pixel = 0;
              // TODO(mereweth@jpl.nasa.gov) - truncate or round here?
              pixel = (frame->data[src] << 2) + ((frame->data[src+4] & 0xc0) >> 6);
              frame->data[dst] = static_cast<NATIVE_UINT_TYPE>(round(255.0 * pixel / 1023.0));

              pixel = (frame->data[src+1] << 2) + ((frame->data[src+4] & 0x30) >> 4);
              frame->data[dst+1] = static_cast<NATIVE_UINT_TYPE>(round(255.0 * pixel / 1023.0));

              pixel = (frame->data[src+2] << 2) + ((frame->data[src+4] & 0x0c) >> 2);
              frame->data[dst+2] = static_cast<NATIVE_UINT_TYPE>(round(255.0 * pixel / 1023.0));

              pixel = (frame->data[src+3] << 2) + (frame->data[src+4] & 0x03);
              frame->data[dst+3] = static_cast<NATIVE_UINT_TYPE>(round(255.0 * pixel / 1023.0));
#endif // TRUNC

              src += 5;
          }
      }

#if MVCAM_TEST_NAV_AOI
      {
          FILE* filp = fopen("/home/linaro/TEST_NAV_AOI.raw", "wb");
          if (filp == NULL) {
              printf("Could not create AOI test file\n");
          }
          else {
              uint8_t *ptr = static_cast<uint8_t *>(frame->data)
                  + static_cast<NATIVE_UINT_TYPE>(MVCAM_AOI_START_ROW * MVCAM_IMAGE_WIDTH
                      * ((CALLBACK_10BIT == m_callbackType) ? 10 / 8 : 1)
                      + MVCAM_AOI_START_COL);
              for (int i = 0; i < MVCAM_AOI_HEIGHT; i++) {
                  fwrite((const void*)ptr,
                         1,
                         static_cast<NATIVE_UINT_TYPE>(MVCAM_AOI_WIDTH),
                         filp);
                  ptr += static_cast<NATIVE_UINT_TYPE>(MVCAM_AOI_STRIDE);
              }

              fclose(filp);
          }

          filp = NULL;
      }
#endif //MVCAM_TEST_NAV_AOI

      // m_frameSkip controls
      if (m_frameCounter >= m_frameSkip) {
          // NOTE(mereweth) - skip count is incremented after this for block
          for(PortOutType i = MVCAM_OUT_MIN; i < MVCAM_OUT_MAX; i = (PortOutType) ((int) i+1)) {
              // high-priority type for GNC; else low-priority type
              const NATIVE_UINT_TYPE buffType = (i == MVCAM_GNC_OUT) ? 0 : 1;
              // Either: high priority always-send
              // Or, we are logging, or saving just next frame
              if (((i == MVCAM_GNC_OUT) &&
                   isConnected_GncBufferSend_OutputPort(0) &&
                   (m_flightMode == MVCAM_MODE_FLIGHT))                    ||

                  ((i == MVCAM_UNPROC_OUT) && (m_logMode != LOGGING_OFF) &&
		   isConnected_UnprocSend_OutputPort(0) &&
                   (((m_lpImageSkipCount >= m_lpImageSkip) &&
                     ((m_logMode == LOGGING_ALL) || (m_logMode == LOGGING_UNPROC)))
                     ||
                    m_saveNextFrameUnproc))                               ||

                  ((i == MVCAM_PROC_OUT) && (m_logMode != LOGGING_OFF) &&
		   isConnected_ProcSend_OutputPort(0) &&
                   (((m_lpImageSkipCount >= m_lpImageSkip) &&
                     ((m_logMode == LOGGING_ALL) || (m_logMode == LOGGING_PROC)))
                     ||
                    m_saveNextFrameProc))                                 ||

                  ((i == MVCAM_IMG_TLM_OUT) &&
                   isConnected_ImageTlmSend_OutputPort(0) &&
                   (m_imgTlmMode == MVCAM_IMG_TLM_ON) &&
                   (m_saveNextFrameProc   ||
                    m_saveNextFrameUnproc ||
                    (0 == m_imagesAcquired %
                     static_cast<NATIVE_UINT_TYPE>(MVCAM_IMG_TLM_SKIP))))) {

                  if (buffType == 1) {
                      DEBUG_PRINT("\nMVCAM sending LP buffer; m_logMode: %d; m_saveNextFrameUnproc: %d; m_saveNextFrameProc: %d\n",
                                  m_logMode, m_saveNextFrameUnproc, m_saveNextFrameProc);
                  }

                  // find open buffer
                  Fw::Buffer buff;
                  bool entryFound = false;
                  NATIVE_UINT_TYPE entry;

#ifdef DEBUG_MODE
                  gettimeofday(&tv,NULL);
                  DEBUG_PRINT("\nTaking m_bufferLock in preview callback %d at time %f",
                              m_printCounter, tv.tv_sec + tv.tv_usec / 1000000.0);
#endif //#ifdef DEBUG_MODE

                  this->m_bufferLock.lock();
                  for (entry = 0; entry < MVCAM_IMG_MAX_NUM_BUFFERS; entry++) {
                      FW_ASSERT(buffType < FW_NUM_ARRAY_ELEMENTS(this->m_buffSet), buffType);
                      FW_ASSERT(static_cast<unsigned long>(entry) < FW_NUM_ARRAY_ELEMENTS(this->m_buffSet[0]), entry);
                      if (this->m_buffSet[buffType][entry].available) {
                          this->m_buffSet[buffType][entry].available = false;
                          buff = this->m_buffSet[buffType][entry].readBuffer;
                          entryFound = true;
                          break;
                      }
                  }

                  if (entryFound) {
                      BYTE* ptr = (BYTE*)buff.getdata();
                      memcpy((void*)ptr, frame->data,
                             static_cast<size_t>(MVCAM_IMAGE_SIZE));

                      U32 tempSec = static_cast<U32>(timestamp_realtime);
                      U32 tempUsec = static_cast<U32>((timestamp_realtime
                                                       - tempSec)
                                                      * 1000.0f * 1000.0f);
                      Fw::Time hwTime(TB_WORKSTATION_TIME,
                                      tempSec,
                                      tempUsec);
                      switch (i) {
                          case MVCAM_GNC_OUT:
			  {
                              DEBUG_PRINT("\nMVCam Sending image out on GNC image port; i = %d\n", i);
                              ROS::sensor_msgs::Image image;
                              image.setdata(buff);
                              image.setheight(MVCAM_IMAGE_HEIGHT);
                              image.setwidth(MVCAM_IMAGE_WIDTH);
                              image.setstep(MVCAM_IMAGE_WIDTH);
                              image.setheader(ROS::std_msgs::Header(m_imagesAcquired,
                                                                    hwTime, 2/*"optic"*/));
                              GncBufferSend_out(0, image);
			  }
                              break;
                          case MVCAM_UNPROC_OUT:
                          case MVCAM_PROC_OUT:
                          {
                              const U32 offsets[3] = {0u, 0u, 0u};
                              const U32 strides[3] = {MVCAM_IMAGE_WIDTH, 0u, 0u};
                              Svc::CameraFrame camFrame;
                              camFrame.settype(Svc::CAMFRAME_STILL);
                              camFrame.setformat(Svc::CAMFMT_IMG_MVCAM_GRAY);
                              camFrame.setdestination(m_camFrameDest);
                              camFrame.setquality(m_compQuality);
                              camFrame.settimestamp(this->getTime());
                              camFrame.sethwTimestamp(hwTime);
                              camFrame.setsize(MVCAM_IMAGE_SIZE);
                              camFrame.setwidth(MVCAM_IMAGE_WIDTH);
                              camFrame.setheight(MVCAM_IMAGE_HEIGHT);
                              camFrame.setoffsets(&offsets[0], 3);
                              camFrame.setstrides(&strides[0], 3);
                              camFrame.setseq(m_imagesAcquired);
                              camFrame.setexposure_target(m_exposureTarget);
                              camFrame.setgain_target(m_gainTarget);
                              camFrame.setexposure(m_exposureTargetTrunc);
                              camFrame.setgain(m_gainTargetTrunc);
                              camFrame.setframe(buff);
                              DEBUG_PRINT("Before serialize of CameraFrame into Buffer\n");
                              Fw::ExternalSerializeBuffer extBuf(ptr + MVCAM_IMAGE_SIZE,
                                                                 buff.getsize() - MVCAM_IMAGE_SIZE);
                              Fw::SerializeStatus serStat = camFrame.serialize(extBuf);

                              if (i == MVCAM_PROC_OUT) {
                                  DEBUG_PRINT("\nMVCam Sending image out on image port as CameraFrame; i = %d\n", i);
                                  if (serStat != Fw::FW_SERIALIZE_OK) {
                                      this->m_buffSet[buffType][entry].available = true;
                                  }
                                  else {
                                      Fw::Buffer outBuff(0, 0, (U64) (ptr + MVCAM_IMAGE_SIZE),
                                                         Svc::CameraFrame::SERIALIZED_SIZE);
                                      this->ProcSend_out(0, outBuff);
                                  }
                              }
                              else {
                                  DEBUG_PRINT("\nMVCam Sending image out on unproc port as Fw::Buffer; i = %d\n", i);
                                  this->UnprocSend_out(0, buff);
                              }
                          }
                              break;
                          case MVCAM_IMG_TLM_OUT:
                              DEBUG_PRINT("\nMVCam Sending image out on tlm port as Fw::Buffer; i = %d\n", i);
                              this->ImageTlmSend_out(0, buff);
                              break;
                          default:
                              DEBUG_PRINT("Unknown ncam port send in callback\n", i);
                              FW_ASSERT(0, i);
                      }
                  }
                  else {
                      if (buffType == 0) { // high-priority
                          this->log_WARNING_HI_MVCAM_NoHPBuffers(0);
                          this->tlmWrite_MVCAM_NumNoHPBuffers(++m_numNoHPBuffers);
                      }
                      else { // low-priority
                          this->log_ACTIVITY_HI_MVCAM_NoLPBuffers(i - 1);
                          this->tlmWrite_MVCAM_NumNoLPBuffers(++m_numNoLPBuffers);
                      }
                      DEBUG_PRINT("\nMVCAM no buffers for port %d\n", i);
                  }
                  this->m_bufferLock.unLock();

#ifdef DEBUG_MODE
                  gettimeofday(&tv,NULL);
                  DEBUG_PRINT("\nReleasing m_bufferLock in preview callback %d at time %f",
                              m_printCounter, tv.tv_sec + tv.tv_usec / 1000000.0);
#endif //#ifdef DEBUG_MODE
              }
          }

          if (m_lpImageSkipCount < m_lpImageSkip) {
              DEBUG_PRINT("\nonPreviewFrame skipping LP %u < %u\n", m_lpImageSkipCount, m_lpImageSkip);
              m_lpImageSkipCount++;
          }
          else {
              m_lpImageSkipCount = 0;
          }

          DEBUG_PRINT(";\nkeeping this frame %u of %u\n", m_frameCounter + 1, m_frameSkip + 1);
          m_frameCounter = 0;

          // this is locked by m_captureParamsLock
          m_saveNextFrameProc = false;
          m_saveNextFrameUnproc = false;

          if (m_waitingImage) {
              if (m_cmdTriggered) {
                  m_cmdTriggered = false;
                  this->cmdResponse_out(m_opCode,m_cmdSeq,Fw::COMMAND_OK);
              }
              // TODO(mereweth) - add argument for which waypoint?
              this->log_ACTIVITY_HI_MVCAM_TookImg();
              m_waitingImage = false;
          }
      }
      else { // if !(m_frameCounter >= m_frameSkip)
          DEBUG_PRINT(";\nframe %u of %u\n", m_frameCounter + 1, m_frameSkip + 1);
          m_frameCounter++;
      }

      // log image metadata
      Svc::ActiveFileLoggerPacket fileBuff;
      if (this->isConnected_FileLogger_OutputPort(0)) {
          Fw::SerializeStatus stat;
          Fw::Time afterExecTime = this->getTime();
          fileBuff.resetSer();
          stat = fileBuff.serialize((U8)AFL_MVCAM_CALLBACK);
          FW_ASSERT(Fw::FW_SERIALIZE_OK == stat, stat);
          // serialize time values
          stat = fileBuff.serialize(currTime.getSeconds());
          FW_ASSERT(Fw::FW_SERIALIZE_OK == stat, stat);
          stat = fileBuff.serialize(currTime.getUSeconds());
          FW_ASSERT(Fw::FW_SERIALIZE_OK == stat, stat);
          stat = fileBuff.serialize(afterExecTime.getSeconds());
          FW_ASSERT(Fw::FW_SERIALIZE_OK == stat, stat);
          stat = fileBuff.serialize(afterExecTime.getUSeconds());
          FW_ASSERT(Fw::FW_SERIALIZE_OK == stat, stat);
          stat = fileBuff.serialize(timestamp_realtime);
          FW_ASSERT(Fw::FW_SERIALIZE_OK == stat, stat);

          this->FileLogger_out(0,fileBuff);
      }

      m_imagesAcquired++;

      m_captureParamsLock.unLock();

#ifdef DEBUG_MODE
      gettimeofday(&tv,NULL);
      DEBUG_PRINT("\nReleasing m_captureParamsLock in preview callback %d at time %f",
                  m_printCounter, tv.tv_sec + tv.tv_usec / 1000000.0);

      gettimeofday(&tv,NULL);
      struct timeval diff;
      timersub(&tv, &tvStart, &diff);
      DEBUG_PRINT("\nPreview callback %d done with elapsed time %f\n", m_printCounter,
                  diff.tv_sec + diff.tv_usec / 1000000.0);

      m_tvLast = tv;
      m_printCounter++;
#endif //#ifdef DEBUG_MODE
  }

} // end namespace SnapdragonFlight
