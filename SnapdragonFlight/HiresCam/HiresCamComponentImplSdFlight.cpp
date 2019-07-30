// ======================================================================
// \title  HiresCamComponentImplSdFlight.cpp
// \author tcanham
// \author mereweth
// \brief  cpp file for HiresCam component implementation class on Snapdragon
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


#include <SnapdragonFlight/HiresCam/HiresCamComponentImpl.hpp>
#include <SnapdragonFlight/HiresCam/HiresCamComponentImplCfg.hpp>

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

//#define DEBUG_MODE

#ifdef DEBUG_MODE
#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
#else
#define DEBUG_PRINT(x,...)
#endif //#ifdef DEBUG_MODE

namespace SnapdragonFlight {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

    HiresCamComponentImpl ::
      ~HiresCamComponentImpl(void)
    {
        m_initialized = false;
    }

  // ----------------------------------------------------------------------
  // Private method implementations
  // ----------------------------------------------------------------------

    void HiresCamComponentImpl ::
      preamble(void)
    {
        if (m_initialized) {
            DEBUG_PRINT("Already initialized in preamble\n");
            this->log_WARNING_HI_HIRESCAM_AlreadyInitializedError();
            return;
        }

        NATIVE_INT_TYPE cameraID;
        NATIVE_INT_TYPE stat = 0;

        NATIVE_INT_TYPE numCameras = camera::getNumberOfCameras();
        if (numCameras < 1) {
            this->log_WARNING_HI_HIRESCAM_CameraError(HIRESCAM_NO_CAMERA);
            return;
        }

#ifndef SOC_8096
        bool found = false;

        for (NATIVE_INT_TYPE i = 0; i < numCameras; i++) {
            camera::CameraInfo cameraInfo;
            stat = camera::getCameraInfo(i, cameraInfo);
            if (stat) {
                this->log_WARNING_HI_HIRESCAM_CameraError(HIRESCAM_NO_CAMERA);
                return;
            }

            if (cameraInfo.func == static_cast<NATIVE_INT_TYPE>(HIRESCAM_CAMERA_TYPE))
            {
                cameraID = i;
                found = true;
            }
        }

        if (found != true) {
            this->log_WARNING_HI_HIRESCAM_CameraError(HIRESCAM_NO_CAMERA);
            return;
        }
#else
        cameraID = static_cast<NATIVE_INT_TYPE>(HIRESCAM_CAMERA_TYPE);
#endif

        stat = camera::ICameraDevice::createInstance(cameraID, &m_cameraPtr);
        if (stat || (m_cameraPtr == NULL)) {
            this->log_WARNING_HI_HIRESCAM_CameraError(HIRESCAM_NO_CAMERA);
            return;
        }

        stat = m_params.init(m_cameraPtr);
        if (stat) {
            camera::ICameraDevice::deleteInstance(&m_cameraPtr);
            this->log_WARNING_HI_HIRESCAM_CameraError(HIRESCAM_PARAM_ERR);
            return;
        }

        /*m_params.set("dis", "eis_2_0");
          m_params.setFocusMode("manual");*/
        m_cameraPtr->sendFaceDetectCommand(false);

#ifndef SOC_8096
        camera::ImageSize imageSize = camera::ImageSize(320, 240);
        m_params.setPreviewSize(imageSize);
#endif

        stat = m_params.commit();
        if (stat) {
            this->log_WARNING_HI_HIRESCAM_CameraError(HIRESCAM_PARAM_ERR);
        }

        // TODO(mereweth) - should we test-apply the params here?

        DEBUG_PRINT("Initialized HiresCam with raw size %s\n", m_params.get("raw-size").c_str());

        /*FW_ASSERT(m_cameraPtr != NULL);
        m_cameraPtr->addListener(this);*/

        m_initialized = true;
    }

    void HiresCamComponentImpl ::
      finalizer(void)
    {
        if (m_activated) {
            DEBUG_PRINT("Deactivating HiresCam in finalizer\n");
            (void) deactivate();
        }

        if (m_cameraPtr != NULL) {
            /*FW_ASSERT(m_cameraPtr);
            m_cameraPtr->removeListener(this);*/
            //delete the camera ptr
            camera::ICameraDevice::deleteInstance(&m_cameraPtr);
            m_cameraPtr = NULL;
        }

        // TODO(mereweth@jpl.nasa.gov) - when are we guaranteed not to get more callbacks?
        m_initialized = false;
    }

    NATIVE_INT_TYPE HiresCamComponentImpl ::
      activate(void)
    {
        NATIVE_INT_TYPE stat;

        // we never call our private methods at incorrect times
        FW_ASSERT(m_initialized);
        FW_ASSERT(!m_activated);

        // add this as a listener.
        FW_ASSERT(m_cameraPtr != NULL);
        m_cameraPtr->addListener(this);
        DEBUG_PRINT("\nIn activate, before startPreview");
        stat = m_cameraPtr->startPreview();
        DEBUG_PRINT("\nIn activate, after startPreview");
        if (stat) {
            m_cameraPtr->removeListener(this);
            m_cameraPtr->stopPreview();
            this->log_WARNING_HI_HIRESCAM_CameraError(HIRESCAM_UNKNOWN_ERR);
            return stat;
        }

#ifdef DEBUG_MODE
        struct timeval tv;
        gettimeofday(&tv,NULL);
        DEBUG_PRINT("\nActivating HiresCamComponentImpl at time %f\n",
                    tv.tv_sec + tv.tv_usec / 1000000.0);
#endif

        //m_captureParamsLock.lock();
        m_activated = true;
        //m_captureParamsLock.unLock();

        this->log_ACTIVITY_HI_HIRESCAM_Activated();
        return 0;
    }

    NATIVE_INT_TYPE HiresCamComponentImpl ::
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
        DEBUG_PRINT("\nDeactivating HiresCamComponentImpl at time %f\n",
        tv.tv_sec + tv.tv_usec / 1000000.0);
#endif

        this->log_ACTIVITY_HI_HIRESCAM_Deactivated();

        return 0;
    }

  // ----------------------------------------------------------------------
  // ICameraListener interface method implementations
  // ----------------------------------------------------------------------

    void HiresCamComponentImpl ::
      onError(void)
    {
        this->log_WARNING_HI_HIRESCAM_CameraError(HIRESCAM_UNKNOWN_ERR);
    }

    void HiresCamComponentImpl ::
#ifdef SOC_8096 // TODO(mereweth) - figure out video for 8096
      onPreviewFrame(camera::ICameraFrame *frame)
#else
      onVideoFrame(camera::ICameraFrame *frame)
#endif
    {
        if (!m_activated                                   ||
            !((m_currentMode == HIRES_WAIT_FIRST_VIDEO) ||
              (m_currentMode == HIRES_WAIT_LAST_VIDEO)  ||
              (m_currentMode == HIRES_GOT_LAST_VIDEO)))       {
#ifndef SOC_8096
            this->log_WARNING_LO_HIRESCAM_ExtraCallbackError(HIRESCAM_EXTRA_VIDEO);
#endif
            DEBUG_PRINT("\nonVideoFrame called while HiresCam (de)activating; mode %d\n",
                        m_currentMode);
            return;
        }

        if ((frame == NULL) ||
            (frame->data == NULL)) {
            this->log_WARNING_HI_HIRESCAM_CameraError(HIRESCAM_VID_EMPTY);
            DEBUG_PRINT("Empty video frame in onVideoFrame callback");
            return;
        }

#ifdef DEBUG_MODE
        struct timeval tv;
        gettimeofday(&tv,NULL);
        DEBUG_PRINT("\nvideo callback %d at time %f; size %u",
                    m_printCounter,
                    tv.tv_sec + tv.tv_usec / 1000000.0, frame->size);
        if (m_printCounter != 0) {
            struct timeval diff;
            timersub(&tv, &m_tvLast, &diff);
            DEBUG_PRINT(";\nelapsed time since last frame %f\n",
            diff.tv_sec + diff.tv_usec / 1000000.0);
        }
#endif //#ifdef DEBUG_MODE

        struct timespec monotonic_now;
        clock_gettime(CLOCK_MONOTONIC, &monotonic_now);

        struct timespec realtime_now;
        clock_gettime(CLOCK_REALTIME, &realtime_now);

        DEBUG_PRINT("\nVideo timestamp before correction %llu\n", frame->timeStamp);
        
        frame->timeStamp = frame->timeStamp
              + (uint64_t) realtime_now.tv_nsec - (uint64_t) monotonic_now.tv_nsec
              + ((uint64_t) realtime_now.tv_sec - (uint64_t) monotonic_now.tv_sec) * (1000ULL * 1000ULL * 1000ULL);

        DEBUG_PRINT("\nVideo timestamp after correction %llu\n", frame->timeStamp);

        if (m_framesAcquiredThisTime > m_framesStopRecording) {
            DEBUG_PRINT("\nonVideoFrame called after enough frames acquired; waiting for sched to detect\n");
            return;
        }

        // Do this here so Sched doesn't see false low-rate video
        m_framesAcquired++;

        if (m_currentMode == HIRES_WAIT_FIRST_VIDEO) {
            m_currentMode = HIRES_WAIT_LAST_VIDEO;
        }

        if (m_frameSkipCount < m_frameSkip) {
            DEBUG_PRINT("\nonVideoFrame skipping %u < %u\n", m_frameSkipCount, m_frameSkip);
            m_frameSkipCount++;
            return;
        }
        else {
            m_frameSkipCount = 0;
        }
        m_framesAcquiredThisTime++;

        if (isConnected_UnprocSend_OutputPort(0)) {
            // find open buffer
            Fw::Buffer buff;
            bool entryFound = false;

#ifdef DEBUG_MODE
            gettimeofday(&tv,NULL);
            DEBUG_PRINT("\nTaking m_bufferLock in video callback %d at time %f",
                        m_printCounter, tv.tv_sec + tv.tv_usec / 1000000.0);
#endif //DEBUG_MODE

            this->m_bufferLock.lock();
            for (NATIVE_INT_TYPE entry = 0; entry < HIRESCAM_MAX_NUM_BUFFERS; entry++) {
                if (this->m_buffSet[0][entry].available) {
                    this->m_buffSet[0][entry].available = false;
                    buff = this->m_buffSet[0][entry].readBuffer;
                    entryFound = true;
                    break;
                }
            }
            this->m_bufferLock.unLock();

#ifdef DEBUG_MODE
            gettimeofday(&tv,NULL);
            DEBUG_PRINT("\nReleasing m_bufferLock in video callback %d at time %f",
                        m_printCounter, tv.tv_sec + tv.tv_usec / 1000000.0);
#endif //DEBUG_MODE

            if (entryFound) {
                BYTE* ptr = (BYTE*)buff.getdata();

                // TODO(mereweth) - check that there is enough image

                size_t minSize = static_cast<size_t>(FW_MIN(HIRESCAM_BUFFER_SIZE
                                                            - Svc::CameraFrame::SERIALIZED_SIZE,
                                                            frame->size));

                DEBUG_PRINT("Buffer size %d, camframe size %d, frame->size %d, minSize %d\n",
                            HIRESCAM_BUFFER_SIZE,
                            Svc::CameraFrame::SERIALIZED_SIZE,
                            frame->size,
                            minSize);

                memcpy((void*)ptr, frame->data, minSize);
                buff.setsize(minSize);
                this->UnprocSend_out(0, buff);
                DEBUG_PRINT("\nHiresCam Sending video out on unproc port\n");
            }
            else {
                this->log_WARNING_LO_HIRESCAM_NoBuffers(0);
                DEBUG_PRINT("\nHIRESCAM no buffers for unproc port\n");
            }
        }
        else {
            DEBUG_PRINT("HIRESCAM unproc port not connected\n");
        }

#ifdef DEBUG_MODE
        m_tvLast = tv;
        m_printCounter++;

        gettimeofday(&tv,NULL);
        struct timeval diff;
        timersub(&tv, &m_tvLast, &diff);
        DEBUG_PRINT("\nvideo callback %d done with elapsed time %f\n", m_printCounter-1,
                    diff.tv_sec + diff.tv_usec / 1000000.0);
#endif // DEBUG_MODE
    }

    void HiresCamComponentImpl ::
      onPictureFrame(camera::ICameraFrame *frame)
    {
        if (m_currentMode == HIRES_DISCARD_IMAGE) {
            m_currentMode = HIRES_IDLE;
            DEBUG_PRINT("\nHiresCam discarded image to shut down preview\n");
            return;
        }

        if (!m_activated                        ||
            (m_currentMode != HIRES_WAIT_IMAGE)) {
            this->log_WARNING_LO_HIRESCAM_ExtraCallbackError(HIRESCAM_EXTRA_PICTURE);
            DEBUG_PRINT("\nonPictureFrame called while HiresCam (de)activating\n");
            return;
        }

        if ((frame == NULL) ||
            (frame->data == NULL)) {
            this->log_WARNING_HI_HIRESCAM_CameraError(HIRESCAM_IMG_EMPTY);
            DEBUG_PRINT("Empty image in onPictureFrame callback");
            return;
        }

#ifdef DEBUG_MODE
        struct timeval tv;
        gettimeofday(&tv,NULL);
        DEBUG_PRINT("\nPicture callback %d at time %f; size %u",
                    m_printCounter,
                    tv.tv_sec + tv.tv_usec / 1000000.0, frame->size);
        if (m_printCounter != 0) {
            struct timeval diff;
            timersub(&tv, &m_tvLast, &diff);
            DEBUG_PRINT(";\nelapsed time since last frame %f\n",
            diff.tv_sec + diff.tv_usec / 1000000.0);
        }
#endif // DEBUG_MODE

        // NOTE(mereweth) - timestamp from hardware is 0 for HiresCam
        struct timespec monotonic_now;
        clock_gettime(CLOCK_MONOTONIC, &monotonic_now);

        struct timespec realtime_now;
        clock_gettime(CLOCK_REALTIME, &realtime_now);

        DEBUG_PRINT("\nPicture timestamp before correction %llu\n", frame->timeStamp);
        
        // TODO(mereweth) - update this when image struct changes
        frame->timeStamp = frame->timeStamp
              + (uint64_t) realtime_now.tv_nsec - (uint64_t) monotonic_now.tv_nsec
              + ((uint64_t) realtime_now.tv_sec - (uint64_t) monotonic_now.tv_sec) * (1000ULL * 1000ULL * 1000ULL);
        
        DEBUG_PRINT("\nPicture timestamp after correction %llu\n", frame->timeStamp);

        m_captureParamsLock.lock();
        m_imagesAcquired++;
        m_currentMode = HIRES_GOT_IMAGE;

        this->log_ACTIVITY_HI_HIRESCAM_TookImg();

        for(PortOutType i = HIRESCAM_OUT_MIN; i < HIRESCAM_OUT_MAX; i = (PortOutType) ((int) i+1)) {
            if (((i == HIRESCAM_UNPROC_OUT) && (m_logMode != LOGGING_OFF) &&
		 isConnected_UnprocSend_OutputPort(0) &&
                 ((m_logMode == LOGGING_ALL)    ||
                  (m_logMode == LOGGING_UNPROC) ||
                  m_saveNextFrameUnproc))                   ||

                ((i == HIRESCAM_PROC_OUT) && (m_logMode != LOGGING_OFF) &&
		 isConnected_ProcSend_OutputPort(0) &&
                 ((m_logMode == LOGGING_ALL)  ||
                  (m_logMode == LOGGING_PROC) ||
                  m_saveNextFrameProc)))                    {

                // find open buffer
                Fw::Buffer buff;
                bool entryFound = false;
                NATIVE_UINT_TYPE entry = 0;

#ifdef DEBUG_MODE
                gettimeofday(&tv,NULL);
                DEBUG_PRINT("\nTaking m_bufferLock in picture callback %d at time %f",
                            m_printCounter, tv.tv_sec + tv.tv_usec / 1000000.0);
#endif // DEBUG_MODE

                this->m_bufferLock.lock();
                for (entry = 0; entry < HIRESCAM_MAX_NUM_BUFFERS; entry++) {
                    FW_ASSERT(static_cast<unsigned long>(entry) < FW_NUM_ARRAY_ELEMENTS(this->m_buffSet[0]), entry);
                    if (this->m_buffSet[0][entry].available) {
                        this->m_buffSet[0][entry].available = false;
                        buff = this->m_buffSet[0][entry].readBuffer;
                        entryFound = true;
                        break;
                    }
                }

                if (entryFound) {
                    BYTE* ptr = (BYTE*)buff.getdata();

                    // TODO(mereweth) - check that there is enough image

                    size_t minSize = static_cast<size_t>(FW_MIN(HIRESCAM_BUFFER_SIZE
                                                                - Svc::CameraFrame::SERIALIZED_SIZE,
                                                                frame->size));

                    DEBUG_PRINT("Buffer size %d, camframe size %d, frame->size %d, minSize %d\n",
                                HIRESCAM_BUFFER_SIZE,
                                Svc::CameraFrame::SERIALIZED_SIZE,
                                frame->size,
                                minSize);

                    memcpy((void*)ptr, frame->data, minSize);

                    switch (i) {
                        case HIRESCAM_PROC_OUT:
                        case HIRESCAM_UNPROC_OUT:
                        {
                            U32 offsets[3] = {0u, 0u, 0u};
                            U32 strides[3] = {0u, 0u, 0u};
                            Svc::CameraFrame camFrame;

                            camFrame.setformat(Svc::CAMFMT_IMG_HIRES_YUV420SP);
                            switch (m_stillType) {
                                case IMG_13MP_JPG:
                                case IMG_13MP_HDR_JPG:
                                    camFrame.setformat(Svc::CAMFMT_IMG_JPEG);
                                    //NOTE(mereweth) - fall through for rest of parms
                                case IMG_13MP:
                                case IMG_13MP_HDR:
                                    offsets[1] = HIRESCAM_13MP_IMAGE_CALLBACK_SKIP +
                                                 HIRESCAM_13MP_IMAGE_CALLBACK_WIDTH *
                                                 HIRESCAM_13MP_IMAGE_HEIGHT;
                                    offsets[2] = offsets[1]
                                                 + (HIRESCAM_13MP_IMAGE_WIDTH *
                                                    HIRESCAM_13MP_IMAGE_HEIGHT) / 4;
                                    strides[0] = HIRESCAM_13MP_IMAGE_CALLBACK_WIDTH;
                                    strides[1] = 0;
                                    strides[2] = 0;
                                    camFrame.setwidth(HIRESCAM_13MP_IMAGE_CALLBACK_WIDTH);
                                    camFrame.setheight(HIRESCAM_13MP_IMAGE_HEIGHT);
                                    camFrame.setsize(HIRESCAM_13MP_IMAGE_SIZE);
                                    break;
                                case IMG_2MP_JPG:
                                case IMG_2MP_HDR_JPG:
                                    camFrame.setformat(Svc::CAMFMT_IMG_JPEG);
                                    //NOTE(mereweth) - fall through for rest of parms
                                case IMG_2MP:
                                case IMG_2MP_HDR:
                                    offsets[1] = HIRESCAM_2MP_IMAGE_CALLBACK_SKIP +
                                                 HIRESCAM_2MP_IMAGE_WIDTH *
                                                 HIRESCAM_2MP_IMAGE_HEIGHT;
                                    offsets[2] = offsets[1]
                                                 + (HIRESCAM_2MP_IMAGE_WIDTH *
                                                    HIRESCAM_2MP_IMAGE_HEIGHT) / 4;
                                    strides[0] = HIRESCAM_2MP_IMAGE_WIDTH;
                                    strides[1] = 0;
                                    strides[2] = 0;
                                    camFrame.setwidth(HIRESCAM_2MP_IMAGE_WIDTH);
                                    camFrame.setheight(HIRESCAM_2MP_IMAGE_HEIGHT);
                                    camFrame.setsize(HIRESCAM_2MP_IMAGE_SIZE);
                                    break;
                                case IMG_VGA_JPG:
                                case IMG_VGA_HDR_JPG:
                                    camFrame.setformat(Svc::CAMFMT_IMG_JPEG);
                                    //NOTE(mereweth) - fall through for rest of parms
                                case IMG_VGA:
                                case IMG_VGA_HDR:
                                    offsets[1] = HIRESCAM_VGA_IMAGE_CALLBACK_SKIP +
                                                 HIRESCAM_VGA_IMAGE_WIDTH *
                                                 HIRESCAM_VGA_IMAGE_HEIGHT;
                                    offsets[2] = offsets[1]
                                                 + (HIRESCAM_VGA_IMAGE_WIDTH *
                                                    HIRESCAM_VGA_IMAGE_HEIGHT) / 4;
                                    strides[0] = HIRESCAM_VGA_IMAGE_WIDTH;
                                    strides[1] = 0;
                                    strides[2] = 0;
                                    camFrame.setwidth(HIRESCAM_VGA_IMAGE_WIDTH);
                                    camFrame.setheight(HIRESCAM_VGA_IMAGE_HEIGHT);
                                    camFrame.setsize(HIRESCAM_VGA_IMAGE_SIZE);
                                    break;
                                default:
                                    // TODO(mereweth) - warning
                                    camFrame.setformat(Svc::CAMFMT_IMG_UNKNOWN);
                                    break;
                            }

                            camFrame.settype(Svc::CAMFRAME_STILL);
                            camFrame.settimestamp(this->getTime());
                            // NOTE(mereweth) - timestamp from hardware is 0 for HiresCam
                            // frame->timeStamp has units of nanoseconds
                            /*U32 tempSec = frame->timeStamp / (1000UL * 1000UL * 1000UL);
                            U32 tempUsec = frame->timeStamp / 1000UL - tempSec * 1000UL * 1000UL;
                            camFrame.sethwTimestamp(Fw::Time(TB_WORKSTATION_TIME,
                                                             tempSec,
                                                             tempUsec));*/
                            camFrame.setoffsets(&offsets[0], 3);
                            camFrame.setstrides(&strides[0], 3);
                            camFrame.setseq(m_imagesAcquired);
                            camFrame.setquality(m_compQuality);
                            camFrame.setdestination(m_camFrameDest);
                            // TODO(mereweth) - how to get Rte hardware AAA result?
                            camFrame.setexposure_target(0u);
                            camFrame.setgain_target(0u);
                            camFrame.setexposure(0u);
                            camFrame.setgain(0u);
                            camFrame.setframe(buff);
                            DEBUG_PRINT("Serialize of camFrame into Buffer; offsets [%llu,%llu,%llu]; ptr %llu\n",
                                        (U64) offsets[0], (U64) offsets[1], (U64) offsets[2],
                                        (U64) ptr);
                            Fw::ExternalSerializeBuffer extBuf(ptr + minSize,
                                                               Svc::CameraFrame::SERIALIZED_SIZE);
                            Fw::SerializeStatus serStat = camFrame.serialize(extBuf);

                            if (HIRESCAM_PROC_OUT == i) {
                                DEBUG_PRINT("\nHiresCam Sending image out on image port as CameraFrame; i = %d\n", i);
                                if (serStat != Fw::FW_SERIALIZE_OK) {
                                    //TODO(mereweth) - warning
                                    this->m_buffSet[0][entry].available = true;
                                }
                                else {
                                    Fw::Buffer outBuff(0, 0, (U64) (ptr + minSize),
                                                       Svc::CameraFrame::SERIALIZED_SIZE);
                                    this->ProcSend_out(0, outBuff);
                                }
                            }
                            else {
                                // OK - buffer return handler resets size
                                buff.setsize(minSize);
                                DEBUG_PRINT("\nHiresCam sending image out on image port as Fw::Buffer; i = %d\n", i);
                                this->UnprocSend_out(0, buff);
                            }
                        } // this brace is just for scoping in the switch
                            break;
                        default:
                            DEBUG_PRINT("Unknown ncam port send in callback\n", i);
                            FW_ASSERT(0, i);
                    }
                }
                else {
                    this->log_WARNING_LO_HIRESCAM_NoBuffers(i);
                    DEBUG_PRINT("\nHIRESCAM no buffers for port %d\n", i);
                }

                this->m_bufferLock.unLock();

#ifdef DEBUG_MODE
                gettimeofday(&tv,NULL);
                DEBUG_PRINT("\nReleasing m_bufferLock in picture callback %d at time %f",
                            m_printCounter, tv.tv_sec + tv.tv_usec / 1000000.0);
#endif // DEBUG_MODE
            }
        }
        m_captureParamsLock.unLock();

#ifdef DEBUG_MODE
        m_tvLast = tv;
        m_printCounter++;

        gettimeofday(&tv,NULL);
        struct timeval diff;
        timersub(&tv, &m_tvLast, &diff);
        DEBUG_PRINT("\npicture callback %d done with elapsed time %f\n", m_printCounter-1,
                    diff.tv_sec + diff.tv_usec / 1000000.0);
#endif //DEBUG_MODE
    }

    void HiresCamComponentImpl ::
#ifdef SOC_8096 // TODO(mereweth) - figure out video for 8096
      onVideoFrame(camera::ICameraFrame *frame)
#else
      onPreviewFrame(camera::ICameraFrame *frame)
#endif
    {
#ifdef DEBUG_MODE
        struct timeval tv;
        gettimeofday(&tv,NULL);
        DEBUG_PRINT("\nHires Preview callback at time %f; size %u; timestamp %llu\n",
                    tv.tv_sec + tv.tv_usec / 1000000.0, frame->size, frame->timeStamp);
#endif // DEBUG_MODE
    }

} // end namespace SnapdragonFlight
