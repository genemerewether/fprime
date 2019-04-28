// ======================================================================
// \title  HiresCamComponentImplCommon.cpp
// \author mereweth
// \brief  cpp file for HiresCam component common
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

#include <stdio.h> // TODO(mereweth@jpl.nasa.gov) - remove the debug prints
#include <sys/time.h>

#include "Svc/ImgComp/ImgCompComponentImplCfg.hpp"

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
#if FW_OBJECT_NAMES == 1
    HiresCamComponentImpl(
        const char *const compName
    ) :
      HiresCamComponentBase(compName),
#else
    HiresCamComponentImpl(void) :
        HiresCamComponentBase(),
#endif
        m_buffSet(), // zero-initialize instead of default-initializing
        m_buffMem(NULL),
        m_allocatorId(-1),
#ifdef BUILD_SDFLIGHT
        m_cameraPtr(NULL),
#endif //BUILD_SDFLIGHT
        m_initialized(false),
        m_activated(false),
        m_imagesAcquired(0u),
        m_framesAcquired(0u),
        m_framesAcquiredLast(0u),
        m_framesAcquiredThisTime(0u),
        m_frameSkipCount(0u),
        m_frameSkip(0u),
        m_framesStopRecording(0u),
        m_framesStopRecordingWaypt(0u),
        m_numSchedStopRecordingWaypt(0u),
        m_saveNextFrameUnproc(false),
        m_saveNextFrameProc(false),
        m_compQuality(100u),
        m_camFrameDest(Svc::FRAMEDEST_STORAGE),
        m_logMode(LOGGING_WAYPT),
        m_captureParamsLock(),
        m_bufferLock(),
        m_currentMode(HIRES_IDLE),
        m_wasActivated(false),
        m_callbackTimeout(0u),
        m_cmdTriggered(false),
        m_cmdErrorOnDeactivate(false),
        m_opCode(),
        m_cmdSeq(0u)
#ifndef BUILD_DARWIN
        ,m_printCounter(0u),
        m_tvLast() // zero-initialize instead of default-initializing
#endif //BUILD_DARWIN
    {
        /* TODO(mereweth@jpl.nasa.gov) - load camera params from
         * PrmDb?
         */
    }


  // ----------------------------------------------------------------------
  // Public method implementations
  // ----------------------------------------------------------------------

    void HiresCamComponentImpl ::
      init(
          const NATIVE_INT_TYPE queueDepth,
          const NATIVE_INT_TYPE instance
      )
    {
        HiresCamComponentBase::init(queueDepth, instance);

        m_framesStopRecordingWaypt = static_cast<NATIVE_UINT_TYPE>(HIRESCAM_FRAMES_STOP_VID);
        m_numSchedStopRecordingWaypt = static_cast<NATIVE_UINT_TYPE>(HIRESCAM_SCHED_STOP_VID);

        for (U32 i = 0; i < FW_NUM_ARRAY_ELEMENTS(m_waypointSet); i++) {
            m_waypointSet[i].index = 0;
            m_waypointSet[i].dest = WAYPT_UNPROC;
            m_waypointSet[i].action = WPACT_STILL;
            m_waypointSet[i].stillType = IMG_13MP;
            m_waypointSet[i].vidType = VID_4K;
            m_waypointSet[i].quality = 100;
            m_waypointSet[i].used = false;
        }
    }

    int HiresCamComponentImpl ::
      spawnChild()
    {
        return HiresCamComponentBase::spawnChild();
    }

    //! Give the class its memory. Should be called after constructor
    //! and init, but before task is spawned.
    void HiresCamComponentImpl ::
      allocateBuffers(
        NATIVE_INT_TYPE identifier,
        Fw::MemAllocator& allocator,
        // TODO(mereweth) - reduce size of buffers
        NATIVE_UINT_TYPE numBuffers //!< The maximum number of buffers
    )
    {
        FW_ASSERT(numBuffers <= HIRESCAM_MAX_NUM_BUFFERS, numBuffers);

        this->m_allocatorId = identifier;

        const NATIVE_UINT_TYPE chunkSize = static_cast<NATIVE_UINT_TYPE>(Cfg::DIRECT_CHUNK_SIZE);

        // TODO(mereweth) - change to array of type Image serializable
        this->m_buffMem = static_cast<BYTE*>(
            allocator.allocate(identifier,
                               HIRESCAM_BUFFER_SIZE * numBuffers
                               + chunkSize - 1));

        BYTE* alignedBuff = (BYTE*) (((U64) this->m_buffMem
                                      | (chunkSize - 1)) + 1);

        for (NATIVE_UINT_TYPE i = 0; i < numBuffers; i++) {
            #pragma GCC diagnostic push
            #pragma GCC diagnostic ignored "-Wunused-local-typedefs"
            COMPILE_TIME_ASSERT(0 < FW_NUM_ARRAY_ELEMENTS(this->m_buffSet),
                                BUFF_SET_IDX_0);
            #pragma GCC diagnostic pop
            FW_ASSERT(i < FW_NUM_ARRAY_ELEMENTS(this->m_buffSet[0]), i);
            BYTE* ptr = alignedBuff + i * static_cast<NATIVE_UINT_TYPE>(HIRESCAM_BUFFER_SIZE);
            m_buffSet[0][i].readBuffer.setdata((U64) ptr);
            m_buffSet[0][i].readBuffer.setsize(static_cast<NATIVE_UINT_TYPE>(HIRESCAM_BUFFER_SIZE));
            m_buffSet[0][i].available = true;
        }
    }

    void HiresCamComponentImpl ::
      deallocateBuffers(Fw::MemAllocator& allocator)
    {
        allocator.deallocate(this->m_allocatorId, (void*)this->m_buffMem);
    }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

    void HiresCamComponentImpl ::
      pingIn_handler(NATIVE_INT_TYPE portNum, U32 key) {
        // respond to ping
        this->pingOut_out(0,key);
    }

    void HiresCamComponentImpl ::
      Sched_handler(
          const NATIVE_INT_TYPE portNum,
          NATIVE_UINT_TYPE context
      )
    {
        m_captureParamsLock.lock();

        /* We are still activated but are done capturing video */
        if ((m_currentMode == HIRES_WAIT_LAST_VIDEO) &&
            (m_framesAcquiredThisTime > m_framesStopRecording)) {
            m_currentMode = HIRES_GOT_LAST_VIDEO;
            DEBUG_PRINT("HiresCam auto-deactivating; %u of %u acquired\n",
                        m_framesAcquiredThisTime,
                        m_framesStopRecording+1);
        }

        int stat = 0;
        // handle various timeouts and successful completions
        HIRESCAM_CameraErrorType tempCamErrorType = HIRESCAM_IMG_TIMEOUT;
        switch (m_currentMode) {
            case HIRES_IDLE:
                break;
            case HIRES_PRE_WAIT_IMAGE:
                if (m_callbackTimeout == 0) {
                    stat = 0;
#ifdef BUILD_SDFLIGHT
#ifndef SOC_8096
                    stat = m_cameraPtr->takePicture();
#endif
#endif // BUILD_SDFLIGHT
                    if (stat == 0) {
                        m_callbackTimeout = static_cast<NATIVE_UINT_TYPE>(HIRESCAM_IMG_WAIT_SEC);
                        m_currentMode = HIRES_WAIT_IMAGE;
                    }
                    else {
                        m_callbackTimeout = static_cast<NATIVE_UINT_TYPE>(HIRESCAM_DEACTIVATE_WAIT_SEC);
                        m_currentMode = HIRES_DEACTIVATE_WAIT;
                        // attempt to deactivate if specific action above failed
                        if (!m_wasActivated) {
                            if (deactivate()) {
                                this->log_WARNING_HI_HIRESCAM_CameraError(HIRESCAM_DEACTIVATE_ERR);
                            }
                        }
                    }
                }
                else {
                    this->tlmWrite_HIRESCAM_TimeoutLeft(--m_callbackTimeout);
                }
                break;
            case HIRES_PRE_WAIT_FIRST_VIDEO:
                if (m_callbackTimeout == 0) {
                    stat = 0;
                    /* NOTE(mereweth) - when we get the first image callback,
                     * we set m_callbackTimeout to m_numSchedStopRecordingWaypt
                     * and transition to HIRES_WAIT_LAST_VIDEO
                     */
#ifdef BUILD_SDFLIGHT
                    stat = m_cameraPtr->startRecording();
#endif // BUILD_SDFLIGHT
                    if (stat == 0) {
                        m_callbackTimeout = static_cast<NATIVE_UINT_TYPE>(HIRESCAM_VID_WAIT_SEC);
                        m_currentMode = HIRES_WAIT_FIRST_VIDEO;
                    }
                    else {
                        m_callbackTimeout = static_cast<NATIVE_UINT_TYPE>(HIRESCAM_DEACTIVATE_WAIT_SEC);
                        m_currentMode = HIRES_DEACTIVATE_WAIT;
                        // attempt to deactivate if specific action above failed
                        if (!m_wasActivated) {
                            if (deactivate()) {
                                this->log_WARNING_HI_HIRESCAM_CameraError(HIRESCAM_DEACTIVATE_ERR);
                            }
                        }
                    }
                }
                else {
                    this->tlmWrite_HIRESCAM_TimeoutLeft(--m_callbackTimeout);
                }
                break;
            case HIRES_DEACTIVATE_WAIT:
                if (m_callbackTimeout == 0) {
                    m_currentMode = HIRES_IDLE;
                    if (m_cmdTriggered) {
                        m_cmdTriggered = false;
                        if (m_cmdErrorOnDeactivate) {
                            this->cmdResponse_out(m_opCode,m_cmdSeq,Fw::COMMAND_EXECUTION_ERROR);
                        }
                        else {
                            this->cmdResponse_out(m_opCode,m_cmdSeq,Fw::COMMAND_OK);
                        }
                        m_cmdErrorOnDeactivate = false;
                    }
                }
                else {
                    this->tlmWrite_HIRESCAM_TimeoutLeft(--m_callbackTimeout);
                }
                break;
            case HIRES_GOT_LAST_VIDEO:
#ifdef BUILD_SDFLIGHT
                m_cameraPtr->stopRecording();
#endif //BUILD_SDFLIGHT
                // NOTE(mereweth) - fall through to finish handling deactivate
            case HIRES_GOT_IMAGE:
                m_cmdErrorOnDeactivate = false;
                if (!m_wasActivated) {
                    if (deactivate()) {
                        m_cmdErrorOnDeactivate = true;
                        this->log_WARNING_HI_HIRESCAM_CameraError(HIRESCAM_DEACTIVATE_ERR);
                    }
                }
                m_callbackTimeout = static_cast<NATIVE_UINT_TYPE>(HIRESCAM_DEACTIVATE_WAIT_SEC);
                m_currentMode = HIRES_DEACTIVATE_WAIT;
                break;
            case HIRES_WAIT_LAST_VIDEO:
            case HIRES_WAIT_FIRST_VIDEO:
                if (m_callbackTimeout == 0) {
#ifdef BUILD_SDFLIGHT
                    m_cameraPtr->stopRecording();
#endif //BUILD_SDFLIGHT
                }
                tempCamErrorType = HIRESCAM_VID_TIMEOUT;
                // NOTE(mereweth) - fall through to finish handling timeout
            case HIRES_WAIT_IMAGE:
            case HIRES_DISCARD_IMAGE:
                m_cmdErrorOnDeactivate = false;
                // NOTE(mereweth) - tempCamErrorType set to HIRESCAM_IMG_TIMEOUT at top
                if (m_callbackTimeout == 0) {
                    this->log_WARNING_HI_HIRESCAM_CameraError(tempCamErrorType);
                    if (!m_wasActivated) {
                        if (deactivate()) {
                            m_cmdErrorOnDeactivate = true;
                            this->log_WARNING_HI_HIRESCAM_CameraError(HIRESCAM_DEACTIVATE_ERR);
                        }
                    }
                    m_callbackTimeout = static_cast<NATIVE_UINT_TYPE>(HIRESCAM_DEACTIVATE_WAIT_SEC);
                    m_currentMode = HIRES_DEACTIVATE_WAIT;
                }
                else {
                    this->tlmWrite_HIRESCAM_TimeoutLeft(--m_callbackTimeout);
                }
                break;
            default:
                DEBUG_PRINT("HiresCam unhandled mode %d in timeout check in sched\n",
                            m_currentMode);
                FW_ASSERT(0, m_currentMode);
                break;
        }
        m_captureParamsLock.unLock();

        NATIVE_UINT_TYPE frameCountDiff = m_framesAcquired - m_framesAcquiredLast;
        if (((m_currentMode == HIRES_WAIT_FIRST_VIDEO) ||
             (m_currentMode == HIRES_WAIT_LAST_VIDEO))                              &&
            (frameCountDiff < static_cast<NATIVE_UINT_TYPE>(HIRESCAM_MIN_RATE_WARN))) {

            this->log_WARNING_LO_HIRESCAM_LoRate(frameCountDiff);
        }
        this->tlmWrite_HIRESCAM_FrameRate(frameCountDiff);
        m_framesAcquiredLast = m_framesAcquired;

        this->tlmWrite_HIRESCAM_ImagesAcquired(m_imagesAcquired);
        this->tlmWrite_HIRESCAM_FramesAcquired(m_framesAcquired);
        this->tlmWrite_HIRESCAM_FramesAcquiredThisTime(m_framesAcquiredThisTime);
    }

    void HiresCamComponentImpl ::
      WaypointRecv_handler(
          const NATIVE_INT_TYPE portNum,
          U32 waypoint
      )
    {
        this->log_ACTIVITY_HI_HIRESCAM_WaypointRecv(waypoint);


        switch (m_currentMode) {
            case HIRES_WAIT_IMAGE:
            case HIRES_PRE_WAIT_IMAGE:
            case HIRES_PRE_WAIT_FIRST_VIDEO:
            case HIRES_GOT_IMAGE:
            case HIRES_DEACTIVATE_WAIT:
            case HIRES_DISCARD_IMAGE:
                this->log_WARNING_HI_HIRESCAM_BusyWaiting(waypoint);
                return;
            case HIRES_WAIT_FIRST_VIDEO:
            case HIRES_WAIT_LAST_VIDEO:
            case HIRES_GOT_LAST_VIDEO:
            case HIRES_IDLE:
                // these cases are checked so we get a compile warning if we are missing a case
                break;
        }

        bool doReturn = false;
        bool found = false;
        U32 idx = 0;
        m_captureParamsLock.lock();
        for (idx = 0; idx < FW_NUM_ARRAY_ELEMENTS(m_waypointSet); idx++) {
            if ((!m_waypointSet[idx].used) ||
                (m_waypointSet[idx].index != waypoint)) {
                continue;
            }

            found = true;
            break;
        }

        // already checked above for if we are waiting on an image; just need to check video
        if (found) {
            // video is started; can only stop it
            if ((m_currentMode == HIRES_WAIT_FIRST_VIDEO) ||
                (m_currentMode == HIRES_WAIT_LAST_VIDEO)  ||
                (m_currentMode == HIRES_GOT_LAST_VIDEO)) {

                // if video is running, no more configuration needed
                doReturn = true;

                if (m_waypointSet[idx].action == WPACT_VIDEO_END) {
                    this->m_currentMode = HIRES_GOT_LAST_VIDEO;
                }
                else {
                    this->log_WARNING_HI_HIRESCAM_BusyWaiting(waypoint);
                }
            }
            else if (m_waypointSet[idx].action == WPACT_VIDEO_END) {
                // TODO(mereweth) - warn that got video stop request while in HIRES_IDLE
                doReturn = true;
            }
        }
        else { // no waypoint spec found
            doReturn = true;
            this->log_ACTIVITY_HI_HIRESCAM_NoWaypointSpec(waypoint);
        }

        m_captureParamsLock.unLock();
        if (doReturn) {
            return;
        }

        /* Important that we only reach here if we are idle, and if the
         * requested action is NOT WPACT_VIDEO_END
         * All other cases are checked above, and lead to return
         */
        FW_ASSERT(m_currentMode == HIRES_IDLE, m_currentMode);

        m_cmdTriggered = false;

        if (found) {
            // TODO(mereweth) - refactor all below into a function to share with take_img and take_vid

            m_compQuality = m_waypointSet[idx].quality;
            m_stillType = m_waypointSet[idx].stillType;

            switch (m_waypointSet[idx].dest) {
                case WAYPT_STORAGE:
                    m_saveNextFrameProc = true;
                    m_camFrameDest = Svc::FRAMEDEST_STORAGE;
                    break;
                case WAYPT_XMIT:
                    m_saveNextFrameProc = true;
                    m_camFrameDest = Svc::FRAMEDEST_XMIT;
                    break;
                case WAYPT_STORAGE_XMIT:
                    m_saveNextFrameProc = true;
                    m_camFrameDest = Svc::FRAMEDEST_STORAGE_XMIT;
                    break;
                case WAYPT_UNPROC:
                    m_saveNextFrameUnproc = true;
                    break;
                default:
                    this->log_WARNING_HI_HIRESCAM_WaypointError(m_waypointSet[idx].index,
                                                            m_waypointSet[idx].dest,
                                                            m_waypointSet[idx].action,
                                                            m_waypointSet[idx].stillType,
                                                            m_waypointSet[idx].vidType);
                    break;
            }

            int stat = 0;
#ifdef BUILD_SDFLIGHT
            camera::ImageSize imageSize;
            switch (m_waypointSet[idx].action) {
                case WPACT_STILL:
                {
                    switch (m_waypointSet[idx].stillType) {
                        case IMG_13MP:
                        case IMG_13MP_HDR:
                        case IMG_13MP_JPG:
                        case IMG_13MP_HDR_JPG:
                            imageSize = camera::ImageSize(HIRESCAM_13MP_IMAGE_WIDTH,
                                                          HIRESCAM_13MP_IMAGE_HEIGHT);
                            break;
                        case IMG_2MP:
                        case IMG_2MP_HDR:
                        case IMG_2MP_JPG:
                        case IMG_2MP_HDR_JPG:
                            imageSize = camera::ImageSize(HIRESCAM_2MP_IMAGE_WIDTH,
                                                          HIRESCAM_2MP_IMAGE_HEIGHT);
                            break;
                        case IMG_VGA:
                        case IMG_VGA_HDR:
                        case IMG_VGA_JPG:
                        case IMG_VGA_HDR_JPG:
                            imageSize = camera::ImageSize(HIRESCAM_VGA_IMAGE_WIDTH,
                                                          HIRESCAM_VGA_IMAGE_HEIGHT);
                            break;
                        default:
                          DEBUG_PRINT("HiresCam unhandled still capture type %d\n");
                          FW_ASSERT(0, m_waypointSet[idx].stillType);
                          break;
                    }

                    if ((m_waypointSet[idx].stillType == IMG_13MP_HDR)     ||
                        (m_waypointSet[idx].stillType == IMG_13MP_HDR_JPG) ||
                        (m_waypointSet[idx].stillType == IMG_2MP_HDR)      ||
                        (m_waypointSet[idx].stillType == IMG_2MP_HDR_JPG)  ||
                        (m_waypointSet[idx].stillType == IMG_VGA_HDR)      ||
                        (m_waypointSet[idx].stillType == IMG_VGA_HDR_JPG))  {
                        m_params.set("scene-mode", "hdr");
                        m_params.set("hdr-need-1x", "true");
                        //m_params.set("zsl", "on");
                    }

                    if ((m_waypointSet[idx].stillType == IMG_13MP_JPG)     ||
                        (m_waypointSet[idx].stillType == IMG_13MP_HDR_JPG) ||
                        (m_waypointSet[idx].stillType == IMG_2MP_JPG)      ||
                        (m_waypointSet[idx].stillType == IMG_2MP_HDR_JPG)  ||
                        (m_waypointSet[idx].stillType == IMG_VGA_JPG)      ||
                        (m_waypointSet[idx].stillType == IMG_VGA_HDR_JPG))  {
                        m_params.set("picture-format", "jpeg");
                        m_params.set("preview-format", "yuv420sp");
                    }
                    else {
                        m_params.set("preview-format", "yuv420sp");
                        // TODO(mereweth) - check interleave
                        m_params.set("picture-format", "yuv420sp");
                    }

                    m_params.setPictureSize(imageSize);
                    imageSize = camera::ImageSize(320, 240);
                    m_params.setPreviewSize(imageSize);
                    stat = m_params.commit();

#ifdef DEBUG_MODE
                    const camera::ImageSize getSize = m_params.getPictureSize();
                    DEBUG_PRINT("\nHires has raw size %s; width %d; height %d\n",
                                m_params.get("raw-size").c_str(),
                                getSize.width,
                                getSize.height);
#endif // DEBUG_MODE

                    if (stat) {
                        this->log_WARNING_HI_HIRESCAM_CameraError(HIRESCAM_PARAM_ERR);
                    }
                }
                    break;
                case WPACT_VIDEO_START:
                {
                    switch (m_waypointSet[idx].vidType) {
                        case VID_4K:
                            imageSize = camera::ImageSize(4208, 3120);
                            break;
                        case VID_MAX_HDR:
                            imageSize = camera::ImageSize(3840, 2160);
                            break;
                        case VID_1080P:
                        case VID_1080P_HDR:
                            imageSize = camera::ImageSize(1920, 1080);
                            break;
                        case VID_720P:
                        case VID_720P_HDR:
                            imageSize = camera::ImageSize(1280, 720);
                            break;
                        case VID_480P:
                        case VID_480P_HDR:
                            imageSize = camera::ImageSize(640, 480);
                            break;
                        default:
                          DEBUG_PRINT("HiresCam unhandled video capture type %d\n");
                          FW_ASSERT(0, m_waypointSet[idx].vidType);
                          break;
                    }

                    if ((m_waypointSet[idx].vidType == VID_MAX_HDR)   ||
                        (m_waypointSet[idx].vidType == VID_1080P_HDR) ||
                        (m_waypointSet[idx].vidType == VID_720P_HDR)  ||
                        (m_waypointSet[idx].vidType == VID_480P_HDR)) {
                      m_params.set("scene-mode", "hdr");
                      m_params.set("hdr-need-1x", "true");
                      //m_params.set("zsl", "on");
                    }

                    m_params.set("preview-format", "yuv420sp");
                    //m_params.set("picture-format", "yuv420sp");

                    m_params.setVideoSize(imageSize);
                    // TODO(mereweth) - go smaller?
                    imageSize = camera::ImageSize(320, 240);
                    m_params.setPreviewSize(imageSize);

                    stat = m_params.commit();
                    if (stat) {
                        this->log_WARNING_HI_HIRESCAM_CameraError(HIRESCAM_PARAM_ERR);
                    }
                }
                    break;
                default:
                    DEBUG_PRINT("HiresCam unhandled action %d\n",
                                m_waypointSet[idx].action);
                    FW_ASSERT(0, m_waypointSet[idx].action);
                    break;
            }
#endif // BUILD_SDFLIGHT

            if (stat) {
                // we aren't activated yet, so OK to return - param commit failed above
                return;
            }

            if (m_activated) {
                m_wasActivated = true;
            }
            else {
                m_wasActivated = false;
                if (activate()) {
                    this->log_WARNING_HI_HIRESCAM_CameraError(HIRESCAM_ACTIVATE_ERR);
                    return;
                }
            }

            switch (m_waypointSet[idx].action) {
                case WPACT_STILL:
                    m_callbackTimeout = static_cast<NATIVE_UINT_TYPE>(HIRESCAM_IMG_PRE_WAIT_SEC);
                    m_currentMode = HIRES_PRE_WAIT_IMAGE;
                    break;
                case WPACT_VIDEO_START:
                    m_framesStopRecording = m_framesStopRecordingWaypt;
                    m_callbackTimeout = static_cast<NATIVE_UINT_TYPE>(HIRESCAM_VID_PRE_WAIT_SEC);
                    m_currentMode = HIRES_PRE_WAIT_FIRST_VIDEO;
                    break;
                default:
                    DEBUG_PRINT("HiresCam unhandled action %d in waypoint handler\n",
                                m_waypointSet[idx].action);
                    FW_ASSERT(0, m_waypointSet[idx].action);
                    break;
            }
        }
        else { // found = false; no waypoint spec found
            this->log_ACTIVITY_HI_HIRESCAM_NoWaypointSpec(waypoint);
        }
    }

    void HiresCamComponentImpl ::
      BufferReturn_handler(
          const NATIVE_INT_TYPE portNum,
          Fw::Buffer &fwBuffer
      )
    {
        struct timeval tv;
        gettimeofday(&tv,NULL);
        DEBUG_PRINT("\nTaking m_bufferLock in ImageBufferIn_handler at time %f",
                    tv.tv_sec + tv.tv_usec / 1000000.0);

        // reset buffer size to full size
        fwBuffer.setsize(static_cast<NATIVE_UINT_TYPE>(HIRESCAM_BUFFER_SIZE));

        this->m_bufferLock.lock();
        bool found = false;
        // search for open entry; HiresCam only has one buffer set for now
        for (NATIVE_UINT_TYPE entry = 0; entry < HIRESCAM_MAX_NUM_BUFFERS; entry++) {
            #pragma GCC diagnostic push
            #pragma GCC diagnostic ignored "-Wunused-local-typedefs"
            COMPILE_TIME_ASSERT(0 < FW_NUM_ARRAY_ELEMENTS(this->m_buffSet),
                                BUFF_SET_IDX_0_RETURN);
            #pragma GCC diagnostic pop
            FW_ASSERT(static_cast<unsigned long>(entry) < FW_NUM_ARRAY_ELEMENTS(this->m_buffSet[0]), entry);
            // Look for slots to fill. "Available" is from
            // the perspective of the driver thread looking for
            // buffers to fill, so add the buffer and make it available.
            if (not this->m_buffSet[0][entry].available) {
                this->m_buffSet[0][entry].readBuffer = fwBuffer;
                this->m_buffSet[0][entry].available = true;
                found = true;
                break;
            }
        }
        this->m_bufferLock.unLock();
        gettimeofday(&tv,NULL);
        DEBUG_PRINT("\nReleasing m_bufferLock in ImageBufferIn_handler at time %f",
                    tv.tv_sec + tv.tv_usec / 1000000.0);
        FW_ASSERT(found,fwBuffer.getbufferID(),fwBuffer.getmanagerID());
    }

  // ----------------------------------------------------------------------
  // Command handler implementations
  // ----------------------------------------------------------------------

    void HiresCamComponentImpl ::
      HIRESCAM_SET_LOGGING_cmdHandler(
          const FwOpcodeType opCode,
          const U32 cmdSeq,
          LogMode Mode
      )
    {
        //m_captureParamsLock.lock();

        m_logMode = Mode;

        switch(m_logMode) {
            case LOGGING_ALL:
                this->log_ACTIVITY_HI_HIRESCAM_LoggingModeChange(LOGGING_ALL_EVR);
                break;
            case LOGGING_PROC:
                this->log_ACTIVITY_HI_HIRESCAM_LoggingModeChange(LOGGING_PROC_EVR);
                break;
            case LOGGING_UNPROC:
                this->log_ACTIVITY_HI_HIRESCAM_LoggingModeChange(LOGGING_UNPROC_EVR);
                break;
            case LOGGING_WAYPT:
                this->log_ACTIVITY_HI_HIRESCAM_LoggingModeChange(LOGGING_WAYPT_EVR);
                break;
            case LOGGING_OFF:
                this->log_ACTIVITY_HI_HIRESCAM_LoggingModeChange(LOGGING_OFF_EVR);
                break;
            default:
                DEBUG_PRINT("Unknown logging mode in SET_LOGGING handler\n");
                FW_ASSERT(0, m_logMode);
                break;
        }

        //m_captureParamsLock.unLock();

        this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_OK);
    }

    void HiresCamComponentImpl ::
      HIRESCAM_ADD_WAYPOINT_cmdHandler(
          const FwOpcodeType opCode,
          const U32 cmdSeq,
          U32 waypoint,
          WayptDestination destination,
          WayptAction action,
          WayptCaptureType type,
          U8 quality
      )
    {
        if (!m_initialized) {
            this->log_WARNING_HI_HIRESCAM_UninitializedError();
            this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_EXECUTION_ERROR);
            return;
        }

        if (m_currentMode != HIRES_IDLE) {
            this->log_WARNING_HI_HIRESCAM_BusyWaiting(waypoint);
            this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_EXECUTION_ERROR);
            return;
        }

        if ((quality > static_cast<U32>(Svc::IMGCOMP_TJ_MAX_QUALITY)) ||
            (quality < static_cast<U32>(Svc::IMGCOMP_TJ_MIN_QUALITY))) {
            this->log_ACTIVITY_HI_HIRESCAM_OverrodeQuality(quality,
                                                       static_cast<U32>(Svc::IMGCOMP_TJ_MAX_QUALITY));
            quality = static_cast<U32>(Svc::IMGCOMP_TJ_MAX_QUALITY);
        }

        switch (type) {
            case WP_IMG_13MP:
            case WP_IMG_13MP_HDR:
            case WP_IMG_13MP_JPG:
            case WP_IMG_13MP_HDR_JPG:
            case WP_IMG_2MP:
            case WP_IMG_2MP_HDR:
            case WP_IMG_2MP_JPG:
            case WP_IMG_2MP_HDR_JPG:
            case WP_IMG_VGA:
            case WP_IMG_VGA_HDR:
            case WP_IMG_VGA_JPG:
            case WP_IMG_VGA_HDR_JPG:
                if (action != WPACT_STILL) {
                    this->log_WARNING_HI_HIRESCAM_WaypointError(waypoint,
                                                            destination,
                                                            action,
                                                            type, 0);
                    this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_EXECUTION_ERROR);
                    return;
                }
                break;
            case WP_VID_4K:
            case WP_VID_MAX_HDR:
            case WP_VID_1080P:
            case WP_VID_1080P_HDR:
            case WP_VID_720P:
            case WP_VID_720P_HDR:
            case WP_VID_480P:
            case WP_VID_480P_HDR:
                if ((action != WPACT_VIDEO_START) &&
                    (action != WPACT_VIDEO_END))  {
                    this->log_WARNING_HI_HIRESCAM_WaypointError(waypoint,
                                                            destination,
                                                            action,
                                                            0, type);
                    this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_EXECUTION_ERROR);
                    return;
                }
                break;
            default:
                DEBUG_PRINT("HiresCam unhandled waypoint type in ADD_WAYPOINT %d\n", type);
                FW_ASSERT(0, type);
                break;
        }

        bool found = false;
        for (U32 i = 0; i < FW_NUM_ARRAY_ELEMENTS(m_waypointSet); i++) {
            if (m_waypointSet[i].used) {
                continue;
            }

            found = true;
            m_waypointSet[i].index = waypoint;
            m_waypointSet[i].dest = destination;
            m_waypointSet[i].action = action;
            switch (type) {
                case WP_IMG_13MP:
                    m_waypointSet[i].stillType = IMG_13MP;
                    break;
                case WP_IMG_13MP_HDR:
                    m_waypointSet[i].stillType = IMG_13MP_HDR;
                    break;
                case WP_IMG_13MP_JPG:
                    m_waypointSet[i].stillType = IMG_13MP_JPG;
                    break;
                case WP_IMG_13MP_HDR_JPG:
                    m_waypointSet[i].stillType = IMG_13MP_HDR_JPG;
                    break;
                case WP_IMG_2MP:
                    m_waypointSet[i].stillType = IMG_2MP;
                    break;
                case WP_IMG_2MP_HDR:
                    m_waypointSet[i].stillType = IMG_2MP_HDR;
                    break;
                case WP_IMG_2MP_JPG:
                    m_waypointSet[i].stillType = IMG_2MP_JPG;
                    break;
                case WP_IMG_2MP_HDR_JPG:
                    m_waypointSet[i].stillType = IMG_2MP_HDR_JPG;
                    break;
                case WP_IMG_VGA:
                    m_waypointSet[i].stillType = IMG_VGA;
                    break;
                case WP_IMG_VGA_HDR:
                    m_waypointSet[i].stillType = IMG_VGA_HDR;
                    break;
                case WP_IMG_VGA_JPG:
                    m_waypointSet[i].stillType = IMG_VGA_JPG;
                    break;
                case WP_IMG_VGA_HDR_JPG:
                    m_waypointSet[i].stillType = IMG_VGA_HDR_JPG;
                    break;
                case WP_VID_4K:
                    m_waypointSet[i].vidType = VID_4K;
                    break;
                case WP_VID_MAX_HDR:
                    m_waypointSet[i].vidType = VID_MAX_HDR;
                    break;
                case WP_VID_1080P:
                    m_waypointSet[i].vidType = VID_1080P;
                    break;
                case WP_VID_1080P_HDR:
                    m_waypointSet[i].vidType = VID_1080P_HDR;
                    break;
                case WP_VID_720P:
                    m_waypointSet[i].vidType = VID_720P;
                    break;
                case WP_VID_720P_HDR:
                    m_waypointSet[i].vidType = VID_720P_HDR;
                    break;
                case WP_VID_480P:
                    m_waypointSet[i].vidType = VID_480P;
                    break;
                case WP_VID_480P_HDR:
                    m_waypointSet[i].vidType = VID_480P_HDR;
                    break;
                default:
                    DEBUG_PRINT("HiresCam unhandled waypoint type in ADD_WAYPOINT %d\n", type);
                    FW_ASSERT(0, type);
                    break;
            }
            m_waypointSet[i].quality = quality;
            m_waypointSet[i].used = true;
            break;
        }

        if (found) {
            this->log_ACTIVITY_LO_HIRESCAM_WaypointAdded(waypoint,
                                                     static_cast<U32>(destination),
                                                     action,
                                                     type,
                                                     quality);
            this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_OK);
        }
        else {
            this->log_WARNING_HI_HIRESCAM_NoWaypointsLeft();
            this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_EXECUTION_ERROR);
        }
    }

    void HiresCamComponentImpl ::
      HIRESCAM_WAYPOINT_TEST_cmdHandler(
          const FwOpcodeType opCode,
          const U32 cmdSeq,
          U32 index
      )
    {
        this->WaypointRecv_handler(0, index);
        this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_OK);
    }

    void HiresCamComponentImpl ::
      HIRESCAM_SET_VID_TIMEOUTS_cmdHandler(
          const FwOpcodeType opCode,
          const U32 cmdSeq,
          U32 frames,
          U32 sched
      )
    {
        if (m_currentMode != HIRES_IDLE) {
            this->log_WARNING_HI_HIRESCAM_BusyWaiting(0);
            this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_EXECUTION_ERROR);
            return;
        }

        m_framesStopRecordingWaypt = frames;
        m_numSchedStopRecordingWaypt = sched;
        this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_OK);
    }

    void HiresCamComponentImpl ::
      HIRESCAM_ACTIVATE_cmdHandler(
          const FwOpcodeType opCode,
          const U32 cmdSeq,
          HiresCamActive Mode
      )
    {
        // don't allow activate or deactivate while we are busy
        if (m_currentMode != HIRES_IDLE) {
            this->log_WARNING_HI_HIRESCAM_BusyWaiting(0);
            this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_EXECUTION_ERROR);
            return;
        }

        // we want to turn on, and we are not on - try to activate
        if ((IMAGING_ON == Mode) && !m_activated) {
            if (activate()) {
                this->log_WARNING_HI_HIRESCAM_CameraError(HIRESCAM_ACTIVATE_ERR);
                this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_EXECUTION_ERROR);
                return;
            }
        }
        // we want to turn off, and we are not off - try to deactivate
        else if ((IMAGING_OFF == Mode) && m_activated) {
            if (deactivate()) {
                this->log_WARNING_HI_HIRESCAM_CameraError(HIRESCAM_ACTIVATE_ERR);
                this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_EXECUTION_ERROR);
                return;
            }
        }

        this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_OK);
    }

    void HiresCamComponentImpl ::
      HIRESCAM_SET_VID_SKIP_cmdHandler(
          const FwOpcodeType opCode,
          const U32 cmdSeq,
          U32 skip
      )
    {
        if (m_currentMode != HIRES_IDLE) {
            this->log_WARNING_HI_HIRESCAM_BusyWaiting(0);
            this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_EXECUTION_ERROR);
            return;
        }

        m_frameSkipCount = skip;
        m_frameSkip = skip;
        this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_OK);
    }

    void HiresCamComponentImpl ::
      HIRESCAM_TAKE_VIDEO_cmdHandler(
          const FwOpcodeType opCode,
          const U32 cmdSeq,
          VidType setting,
          U32 frames
      )
    {
        if (frames == 0) {
            this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_EXECUTION_ERROR);
            return;
        }

        if (!m_initialized) {
            this->log_WARNING_HI_HIRESCAM_UninitializedError();
            this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_EXECUTION_ERROR);
            return;
        }

        if (m_currentMode != HIRES_IDLE) {
            this->log_WARNING_HI_HIRESCAM_BusyWaiting(0);
            this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_EXECUTION_ERROR);
            return;
        }

        m_vidType = setting;
        m_cmdTriggered = true;
        m_opCode = opCode;
        m_cmdSeq = cmdSeq;
        m_vidType = setting;

        // TODO(mereweth) - add destination for video?

        int stat = 0;
#ifdef BUILD_SDFLIGHT
        camera::ImageSize imageSize;
        switch (setting) {
            case VID_4K:
                imageSize = camera::ImageSize(4208, 3120);
                break;
            case VID_MAX_HDR:
                imageSize = camera::ImageSize(3840, 2160);
                break;
            case VID_1080P:
            case VID_1080P_HDR:
                imageSize = camera::ImageSize(1920, 1080);
                break;
            case VID_720P:
            case VID_720P_HDR:
                imageSize = camera::ImageSize(1280, 720);
                break;
            case VID_480P:
            case VID_480P_HDR:
                imageSize = camera::ImageSize(640, 480);
                break;
            default:
              DEBUG_PRINT("HiresCam unhandled video capture type %d\n");
              FW_ASSERT(0, setting);
              break;
        }

        if ((setting == VID_MAX_HDR)   ||
            (setting == VID_1080P_HDR) ||
            (setting == VID_720P_HDR)  ||
            (setting == VID_480P_HDR)) {
          m_params.set("scene-mode", "hdr");
          m_params.set("hdr-need-1x", "true");
          //m_params.set("zsl", "on");
        }

        m_params.set("preview-format", "yuv420sp");
        //m_params.set("picture-format", "yuv420sp");

        m_params.setVideoSize(imageSize);
        // TODO(mereweth) - go smaller?
        imageSize = camera::ImageSize(320, 240);
        m_params.setPreviewSize(imageSize);

        stat = m_params.commit();
        if (stat) {
            this->log_WARNING_HI_HIRESCAM_CameraError(HIRESCAM_PARAM_ERR);
        }
#endif // BUILD_SDFLIGHT

        if (stat) {
            // we aren't activated yet, so OK to return - param commit failed above
            this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_EXECUTION_ERROR);
            return;
        }

        m_framesStopRecording = frames - 1; // make zero-indexed
        m_framesAcquiredThisTime = 0;
        m_callbackTimeout = static_cast<NATIVE_UINT_TYPE>(HIRESCAM_VID_WAIT_SEC);
        /* NOTE(mereweth) - when we get the first image callback,
         * we set m_callbackTimeout to m_numSchedStopRecordingWaypt
         * and transition to HIRES_WAIT_LAST_VIDEO
         */

         if (m_activated) {
             m_wasActivated = true;
         }
         else {
             m_wasActivated = false;
             if (activate()) {
                 this->log_WARNING_HI_HIRESCAM_CameraError(HIRESCAM_ACTIVATE_ERR);
                 this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_EXECUTION_ERROR);
                 return;
             }
         }

#ifdef BUILD_SDFLIGHT
        stat = m_cameraPtr->startRecording();
#else
        stat = 0;
#endif // BUILD_SDFLIGHT
        if (stat == 0) {
            m_currentMode = HIRES_WAIT_FIRST_VIDEO;
        }

        // attempt to deactivate if specific action above failed
        if (stat && !m_wasActivated) {
            if (deactivate()) {
                this->log_WARNING_HI_HIRESCAM_CameraError(HIRESCAM_DEACTIVATE_ERR);
            }
        }
    }

    void HiresCamComponentImpl ::
      HIRESCAM_TAKE_IMG_cmdHandler(
          const FwOpcodeType opCode,
          const U32 cmdSeq,
          TakeImgDestination destination,
          ImgType type,
          U8 quality
      )
    {
        if (!m_initialized) {
            this->log_WARNING_HI_HIRESCAM_UninitializedError();
            this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_EXECUTION_ERROR);
            return;
        }

        if (m_currentMode != HIRES_IDLE) {
            this->log_WARNING_HI_HIRESCAM_BusyWaiting(0);
            this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_EXECUTION_ERROR);
            return;
        }

        m_cmdTriggered = true;
        m_opCode = opCode;
        m_cmdSeq = cmdSeq;

        // TODO(mereweth) - refactor all below into a function to share with take_img and take_vid
        m_saveNextFrameProc = false;
        m_saveNextFrameUnproc = false;

        m_compQuality = quality;
        m_stillType = type;

        switch (destination) {
            case TAKE_IMG_STORAGE:
                m_saveNextFrameProc = true;
                m_camFrameDest = Svc::FRAMEDEST_STORAGE;
                break;
            case TAKE_IMG_XMIT:
                m_saveNextFrameProc = true;
                m_camFrameDest = Svc::FRAMEDEST_XMIT;
                break;
            case TAKE_IMG_STORAGE_XMIT:
                m_saveNextFrameProc = true;
                m_camFrameDest = Svc::FRAMEDEST_STORAGE_XMIT;
                break;
            case TAKE_IMG_UNPROC:
                m_saveNextFrameUnproc = true;
                break;
            default:
                this->log_WARNING_HI_HIRESCAM_WaypointError(0, destination, 0, type, 0);
                break;
        }

        int stat = 0;
#ifdef BUILD_SDFLIGHT
        camera::ImageSize imageSize;
        switch (type) {
            case IMG_13MP:
            case IMG_13MP_HDR:
            case IMG_13MP_JPG:
            case IMG_13MP_HDR_JPG:
                imageSize = camera::ImageSize(HIRESCAM_13MP_IMAGE_WIDTH,
                                              HIRESCAM_13MP_IMAGE_HEIGHT);
                break;
            case IMG_2MP:
            case IMG_2MP_HDR:
            case IMG_2MP_JPG:
            case IMG_2MP_HDR_JPG:
                imageSize = camera::ImageSize(HIRESCAM_2MP_IMAGE_WIDTH,
                                              HIRESCAM_2MP_IMAGE_HEIGHT);
                break;
            case IMG_VGA:
            case IMG_VGA_HDR:
            case IMG_VGA_JPG:
            case IMG_VGA_HDR_JPG:
                imageSize = camera::ImageSize(HIRESCAM_VGA_IMAGE_WIDTH,
                                              HIRESCAM_VGA_IMAGE_HEIGHT);
                break;
            default:
              DEBUG_PRINT("HiresCam unhandled still capture type %d\n");
              FW_ASSERT(0, type);
              break;
        }

        if ((type == IMG_13MP_HDR)     ||
            (type == IMG_13MP_HDR_JPG) ||
            (type == IMG_2MP_HDR)      ||
            (type == IMG_2MP_HDR_JPG)  ||
            (type == IMG_VGA_HDR)      ||
            (type == IMG_VGA_HDR_JPG))  {
            m_params.set("scene-mode", "hdr");
            m_params.set("hdr-need-1x", "true");
            //m_params.set("zsl", "on");
        }

        if ((type == IMG_13MP_JPG)     ||
            (type == IMG_13MP_HDR_JPG) ||
            (type == IMG_2MP_JPG)      ||
            (type == IMG_2MP_HDR_JPG)  ||
            (type == IMG_VGA_JPG)      ||
            (type == IMG_VGA_HDR_JPG))  {
            m_params.set("picture-format", "jpeg");
            m_params.set("preview-format", "yuv420sp");
        }
        else {
            m_params.set("preview-format", "yuv420sp");
            m_params.set("picture-format", "yuv420sp");
        }

        m_params.setPictureSize(imageSize);
        imageSize = camera::ImageSize(320, 240);
        m_params.setPreviewSize(imageSize);
        stat = m_params.commit();
        if (stat) {
            this->log_WARNING_HI_HIRESCAM_CameraError(HIRESCAM_PARAM_ERR);
        }

#ifdef DEBUG_MODE
        const camera::ImageSize getSize = m_params.getPictureSize();
        DEBUG_PRINT("\nHires has raw size %s; width %d; height %d\n",
                    m_params.get("raw-size").c_str(),
                    getSize.width,
                    getSize.height);
#endif // DEBUG_MODE

#endif // BUILD_SDFLIGHT

        if (stat) {
            // we aren't activated yet, so OK to return - param commit failed above
            return;
        }

        if (m_activated) {
            m_wasActivated = true;
        }
        else {
            m_wasActivated = false;
            if (activate()) {
                this->log_WARNING_HI_HIRESCAM_CameraError(HIRESCAM_ACTIVATE_ERR);
                return;
            }
        }

        m_currentMode = HIRES_PRE_WAIT_IMAGE;
        m_callbackTimeout = static_cast<NATIVE_UINT_TYPE>(HIRESCAM_IMG_WAIT_SEC);
    }

    void HiresCamComponentImpl ::
      HIRESCAM_TEST_IMG_cmdHandler(
          const FwOpcodeType opCode,
          const U32 cmdSeq,
          TestImgDestination destination,
          TestImgType type,
          U8 quality
      )
    {
        if (m_currentMode != HIRES_IDLE) {
            // -1 indicates that the new image command was manual, not a waypoint
            this->log_WARNING_HI_HIRESCAM_BusyWaiting(-1);
            this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_BUSY);
            return;
        }

        // find open buffer
        Fw::Buffer buff;
        bool entryFound = false;
        NATIVE_UINT_TYPE entry;

        this->m_bufferLock.lock();
        U32 buffType = 0;
        for (entry = 0; entry < HIRESCAM_MAX_NUM_BUFFERS; entry++) {
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
            memset(ptr, 0, HIRESCAM_BUFFER_SIZE);

            DEBUG_PRINT("\nHiresCam Sending image out on image port in TEST_IMG handler\n");
	    Svc::CameraFrame camFrame;

            U32 offsets[3] = {0u, 0u, 0u};
            U32 strides[3] = {0u, 0u, 0u};

            U8* cbcrPtr = ptr;
            switch (type) {
                case TEST_IMG_13MP:
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
                    camFrame.setformat(Svc::CAMFMT_IMG_HIRES_YUV420SP);
                    cbcrPtr = ptr + HIRESCAM_13MP_IMAGE_WIDTH
                              * HIRESCAM_13MP_IMAGE_HEIGHT
                              + HIRESCAM_13MP_IMAGE_CALLBACK_SKIP;
                    break;
                case TEST_IMG_2MP:
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
                    camFrame.setformat(Svc::CAMFMT_IMG_HIRES_YUV420SP);
                    cbcrPtr = ptr + HIRESCAM_2MP_IMAGE_WIDTH
                              * HIRESCAM_2MP_IMAGE_HEIGHT
                              + HIRESCAM_2MP_IMAGE_CALLBACK_SKIP;
                    break;
                case TEST_IMG_VGA:
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
                    camFrame.setformat(Svc::CAMFMT_IMG_HIRES_YUV420SP);
                    cbcrPtr = ptr + HIRESCAM_VGA_IMAGE_WIDTH
                              * HIRESCAM_VGA_IMAGE_HEIGHT
                              + HIRESCAM_VGA_IMAGE_CALLBACK_SKIP;
                    break;
                case TEST_IMG_JPG:
                    camFrame.setformat(Svc::CAMFMT_IMG_JPEG);
                    camFrame.setwidth(HIRESCAM_VGA_IMAGE_WIDTH);
                    camFrame.setheight(HIRESCAM_VGA_IMAGE_HEIGHT);
                    camFrame.setsize(HIRESCAM_VGA_IMAGE_SIZE);
                    break;
                default:
                    DEBUG_PRINT("Unhandled type %d in HIRESCAM_TEST_IMG\n", type);
                    this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_EXECUTION_ERROR);
                    return;
            }
            camFrame.settype(Svc::CAMFRAME_STILL);
            camFrame.setdestination(static_cast<Svc::FrameDestType>(destination));
            camFrame.setquality(quality);
            camFrame.settimestamp(this->getTime());
            camFrame.setoffsets(&offsets[0], 3);
            camFrame.setstrides(&strides[0], 3);
            camFrame.setseq(0);
            camFrame.setexposure(0);
            camFrame.setexposure_target(0.0f);
            camFrame.setgain_target(0.0f);
            camFrame.setgain(0);
            camFrame.setframe(buff);

            DEBUG_PRINT("Before serialize of CameraFrame into Buffer\n");
            // shaded row test pattern
            U32 numDivs = camFrame.getheight() / 10;
            for (U32 i = 0; i < camFrame.getheight(); i += numDivs) {
                DEBUG_PRINT("HiresCam test img i=%d, val=%f, num=%d\n",
                            i, i * 255.0 / camFrame.getheight(),
                            camFrame.getwidth() * numDivs);
                memset(ptr + camFrame.getwidth() * i,
                       (U8) (i * 255.0 / camFrame.getheight()),
                       camFrame.getwidth() * numDivs);
            }

            // shaded row test pattern
            const U32 subsampHeight = camFrame.getheight() / 2;
            const U32 subsampWidth = camFrame.getwidth() / 2;
            numDivs = subsampHeight / 10;
            for (U32 i = 0; i < subsampHeight; i += numDivs) {
                DEBUG_PRINT("HiresCam test img i=%d, val=%f, num=%d\n",
                            i, ((i / numDivs) % 4) * 255.0 / 3,
                            subsampWidth * numDivs);
                memset(cbcrPtr + subsampWidth * i,
                       (U8) ((i / numDivs) % 4) * 255.0 / 3,
                       subsampWidth * numDivs);
            }

            size_t minSize = static_cast<size_t>(FW_MIN(HIRESCAM_BUFFER_SIZE
                                                        - Svc::CameraFrame::SERIALIZED_SIZE,
                                                        camFrame.getsize()));
            Fw::ExternalSerializeBuffer extBuf(ptr + minSize,
                                               Svc::CameraFrame::SERIALIZED_SIZE);
            Fw::SerializeStatus serStat = camFrame.serialize(extBuf);
            if (serStat != Fw::FW_SERIALIZE_OK) {
                this->m_buffSet[0][entry].available = true;
            }
            else {
                Fw::Buffer outBuff(0, 0, (U64) (ptr + minSize),
                                   Svc::CameraFrame::SERIALIZED_SIZE);
                this->ProcSend_out(0, outBuff);
            }
            this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_OK);
        }
        else {
            this->log_WARNING_LO_HIRESCAM_NoBuffers(1);
            DEBUG_PRINT("\nHIRESCAM no buffers in TEST_IMG handler\n");
            this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_EXECUTION_ERROR);
        }
        this->m_bufferLock.unLock();
    }

} // end namespace SnapdragonFlight
