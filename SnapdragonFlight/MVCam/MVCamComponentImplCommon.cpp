// ======================================================================
// \title  MVCamComponentImplCommon.cpp
// \author mereweth
// \brief  cpp file for MVCam component common
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

#include "Common/Cfg/QuestConstants.hpp"

#include "Svc/ImgComp/ImgCompComponentImplCfg.hpp"

#include <stdio.h> // TODO(mereweth@jpl.nasa.gov) - remove the debug prints
#include <sys/time.h>

//#define DEBUG_MODE

#ifdef DEBUG_MODE
#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
#else
#define DEBUG_PRINT(x,...)
#endif //#ifdef DEBUG_MODE

#ifdef BUILD_SDFLIGHT
#include "mv.h"
#include "mvCPA.h"
#endif // BUILD_SDFLIGHT

namespace SnapdragonFlight {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------
    MVCamComponentImpl ::
#if FW_OBJECT_NAMES == 1
    MVCamComponentImpl(
        const char *const compName
    ) :
      MVCamComponentBase(compName),
#else
    MVCamComponentImpl(void) :
        MVCamComponentBase(),
#endif
        m_waypointSet(), // zero-initialize instead of default-initializing
        m_buffSet(), // zero-initialize instead of default-initializing
        m_buffMemLP(NULL),
        m_buffMemHP(NULL),
        m_allocatorId(-1),
        m_numGncBufferReturnQueueErrs(0u),
        m_gncBufferReturnBuf(),
        m_gncBufferReturnSerialBuffObj(m_gncBufferReturnBuf, FW_NUM_ARRAY_ELEMENTS(m_gncBufferReturnBuf)),
        m_gncBufferReturnQueue(MVCAM_IMG_HP_BUFFER_POOL_SIZE, Fw::Buffer::SERIALIZED_SIZE),
#ifdef BUILD_SDFLIGHT
        m_cameraPtr(NULL),
        m_mvCPAPtr(NULL),
        m_mvCPAConfig(), // zero-initialize instead of default-initializing
#endif //BUILD_SDFLIGHT
        m_initialized(false),
        m_activated(false),
        m_imagesAcquired(0u),
        m_imagesAcquiredLast(0u),
        m_lpImagesAcquiredThisTime(0u),
        m_lpImageSkipCount(0u),
        m_lpImageSkip(0u),
        m_saveNextFrameUnproc(false),
        m_saveNextFrameProc(false),
        m_compQuality(100u),
        m_camFrameDest(Svc::FRAMEDEST_STORAGE),
        m_logMode(LOGGING_WAYPT),
        m_frameSkip(0u),
        m_frameCounter(0u),
        m_callbackType(CALLBACK_8BIT),
        m_exposureMode(EXPOSURE_AUTO),
        m_exposureTarget(-1.0f),
        m_gainTarget(-1.0f),
        m_exposureTargetTrunc(0u),
        m_gainTargetTrunc(0u),
        m_captureParamsLock(),
        m_bufferLock(),
        m_wasActivated(true),
        m_waitingImage(false),
        m_imageTimeout(0u),
        m_cmdTriggered(false),
        m_opCode(),
        m_cmdSeq(0u),
        m_imgTlmMode(MVCAM_IMG_TLM_OFF),
        m_flightMode(MVCAM_MODE_PREFLIGHT),
        m_maxExposureDelta(0u),
        m_maxExposureScale(0u),
        m_maxGainScale(0u),
        m_numNoHPBuffers(0u),
        m_numNoLPBuffers(0u),
        m_numLoRate(0u),
        m_numMaxExposureDelta(0u),
        m_firstSched(false)
#ifndef BUILD_DARWIN
        ,m_printCounter(0u),
        m_tvLast() // zero-initialize instead of default-initializing
#endif //BUILD_DARWIN
    {
        m_frameSkip = static_cast<NATIVE_UINT_TYPE>(MVCAM_FRAME_SKIP);
        m_frameCounter = m_frameSkip;

        m_maxExposureDelta = static_cast<NATIVE_UINT_TYPE>(MVCAM_CPA_EXPOSURE_MAX_DELTA);
        m_maxExposureScale = static_cast<NATIVE_UINT_TYPE>(MVCAM_MAX_EXPOSURE);
        m_maxGainScale = static_cast<NATIVE_UINT_TYPE>(MVCAM_MAX_GAIN);

        for (U32 i = 0; i < FW_NUM_ARRAY_ELEMENTS(m_waypointSet); i++) {
            m_waypointSet[i].index = 0;
            m_waypointSet[i].type = WAYPT_UNPROC;
            m_waypointSet[i].quality = 100;
            m_waypointSet[i].used = false;
        }
    }


  // ----------------------------------------------------------------------
  // Public method implementations
  // ----------------------------------------------------------------------

    void MVCamComponentImpl ::
      init(
          const NATIVE_INT_TYPE queueDepth,
          const NATIVE_INT_TYPE instance
      )
    {
        MVCamComponentBase::init(queueDepth, instance);
    }

    //! Give the class its memory. Should be called after constructor
    //! and init, but before task is spawned.
    void MVCamComponentImpl ::
      allocateBuffers(
        NATIVE_INT_TYPE identifier,
        Fw::MemAllocator& allocator,
        NATIVE_UINT_TYPE numHPImages, //!< The maximum number of high priority images
        NATIVE_UINT_TYPE numLPImages //!< The maximum number of low priority images
    )
    {
        FW_ASSERT(numHPImages <= MVCAM_IMG_HP_BUFFER_POOL_SIZE, numHPImages);
        FW_ASSERT(numLPImages <= MVCAM_IMG_LP_BUFFER_POOL_SIZE, numLPImages);

        this->m_allocatorId = identifier;

        const NATIVE_UINT_TYPE chunkSize = static_cast<NATIVE_UINT_TYPE>(Cfg::DIRECT_CHUNK_SIZE);

        // TODO(mereweth) - change to array of type Image serializable
        this->m_buffMemHP = static_cast<BYTE*>(
            allocator.allocate(identifier,
                               MVCAM_BUFFER_SIZE * numHPImages
                               + chunkSize - 1));
        this->m_buffMemLP = static_cast<BYTE*>(
            allocator.allocate(identifier,
                               MVCAM_BUFFER_SIZE * numLPImages
                               + chunkSize - 1));

        BYTE* alignedBuff = (BYTE*) (((U64) this->m_buffMemHP
                                      | (chunkSize - 1)) + 1);
        for (NATIVE_UINT_TYPE i = 0; i < numHPImages; i++) {
            #pragma GCC diagnostic push
            #pragma GCC diagnostic ignored "-Wunused-local-typedefs"
            COMPILE_TIME_ASSERT(0 < FW_NUM_ARRAY_ELEMENTS(this->m_buffSet),
                                BUFF_SET_IDX_0);
            #pragma GCC diagnostic pop
            FW_ASSERT(i < FW_NUM_ARRAY_ELEMENTS(this->m_buffSet[MVCAM_HP_BUFFER]), i);
            BYTE* ptr = alignedBuff + i * static_cast<NATIVE_UINT_TYPE>(MVCAM_BUFFER_SIZE);
            m_buffSet[MVCAM_HP_BUFFER][i].readBuffer.setdata((U64) ptr);
            m_buffSet[MVCAM_HP_BUFFER][i].readBuffer.setsize(static_cast<NATIVE_UINT_TYPE>(MVCAM_BUFFER_SIZE));
            m_buffSet[MVCAM_HP_BUFFER][i].available = true;
        }
        alignedBuff = (BYTE*) (((U64) this->m_buffMemLP
                                | (chunkSize - 1)) + 1);
        for (NATIVE_UINT_TYPE i = 0; i < numLPImages; i++) {
            #pragma GCC diagnostic push
            #pragma GCC diagnostic ignored "-Wunused-local-typedefs"
            COMPILE_TIME_ASSERT(1 < FW_NUM_ARRAY_ELEMENTS(this->m_buffSet),
                                BUFF_SET_IDX_1);
            #pragma GCC diagnostic pop
            FW_ASSERT(i < FW_NUM_ARRAY_ELEMENTS(this->m_buffSet[MVCAM_LP_BUFFER]), i);
            BYTE* ptr = alignedBuff + i * static_cast<NATIVE_UINT_TYPE>(MVCAM_BUFFER_SIZE);
            m_buffSet[MVCAM_LP_BUFFER][i].readBuffer.setdata((U64) ptr);
            m_buffSet[MVCAM_LP_BUFFER][i].readBuffer.setsize(static_cast<NATIVE_UINT_TYPE>(MVCAM_BUFFER_SIZE));
            m_buffSet[MVCAM_LP_BUFFER][i].available = true;
        }
    }

    void MVCamComponentImpl ::
      deallocateBuffers(Fw::MemAllocator& allocator)
    {
        allocator.deallocate(this->m_allocatorId, (void*)this->m_buffMemLP);
        allocator.deallocate(this->m_allocatorId, (void*)this->m_buffMemHP);
    }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

    void MVCamComponentImpl ::
      pingIn_handler(NATIVE_INT_TYPE portNum, U32 key) {
        // respond to ping
        this->pingOut_out(0,key);
    }

    void MVCamComponentImpl ::
      Sched_handler(
          const NATIVE_INT_TYPE portNum,
          NATIVE_UINT_TYPE context
      )
    {
        m_captureParamsLock.lock();
        // handle image timeout
        if (m_waitingImage) {
            if (m_imageTimeout == 0) {
                if (m_cmdTriggered) {
                    m_cmdTriggered = false;
                    this->cmdResponse_out(m_opCode,m_cmdSeq,Fw::COMMAND_EXECUTION_ERROR);
                }
                this->log_WARNING_HI_MVCAM_CameraError(MVCAM_IMG_TIMEOUT);
                m_waitingImage = false;
            }
            else {
                this->tlmWrite_MVCAM_ImageTimeoutLeft(--m_imageTimeout);
            }
        }

        /* Before the image request, we were deactivated; we are still
         * activated but have already captured the image
         */
        if (m_activated && !m_wasActivated && !m_waitingImage) {
            // reset this flag to prevent accidental deactivations
            m_wasActivated = true;
            // NOTE(mereweth) - don't send command reply - that happens in image callback
            if (deactivate()) {
                this->log_WARNING_HI_MVCAM_CameraError(MVCAM_DEACTIVATE_ERR);
            }
        }
        m_captureParamsLock.unLock();

        this->tlmWrite_MVCAM_ImagesAcquired(m_imagesAcquired);
        this->tlmWrite_MVCAM_ExposureCalc(m_exposureTarget);
        this->tlmWrite_MVCAM_GainCalc(m_gainTarget);
        this->tlmWrite_MVCAM_Exposure(m_exposureTargetTrunc);
        this->tlmWrite_MVCAM_Gain(m_gainTargetTrunc);

        NATIVE_UINT_TYPE imageCountDiff = m_imagesAcquired - m_imagesAcquiredLast;
        if (m_activated &&
            (imageCountDiff < static_cast<NATIVE_UINT_TYPE>(MVCAM_MIN_RATE_WARN))) {
            if (m_firstSched) {
                // NOTE(mereweth) - don't evr when starting up camera
                m_firstSched = false;
            }
            else {
                this->log_WARNING_HI_MVCAM_LoRate(imageCountDiff);
                this->tlmWrite_MVCAM_NumLoRate(++m_numLoRate);
            }
        }
        this->tlmWrite_MVCAM_ImageRate(imageCountDiff);
        this->tlmWrite_MVCAM_NumMaxExpDelta(m_numMaxExposureDelta);
        m_imagesAcquiredLast = m_imagesAcquired;
    }

    void MVCamComponentImpl ::
      WaypointRecv_handler(
          const NATIVE_INT_TYPE portNum,
          U32 waypoint
      )
    {
        this->log_ACTIVITY_HI_MVCAM_WaypointRecv(waypoint);
        if (!m_initialized) {
            this->log_WARNING_HI_MVCAM_UninitializedError();
            return;
        }

        if (!m_activated) {
            this->log_WARNING_HI_MVCAM_MissedWaypoint(waypoint);
            return;
        }

        if (m_waitingImage) {
            this->log_WARNING_HI_MVCAM_BusyWaitingImage(waypoint);
            return;
        }

        m_captureParamsLock.lock();
        bool found = false;
        U32 idx = 0;
        if (m_logMode != LOGGING_OFF) {
            for (idx = 0; idx < FW_NUM_ARRAY_ELEMENTS(m_waypointSet); idx++) {
                if ((!m_waypointSet[idx].used) ||
                    (m_waypointSet[idx].index != waypoint)) {
                    continue;
                }

                found = true;
                break;
            }
        }

        if (found) {
            m_compQuality = m_waypointSet[idx].quality;
            switch (m_waypointSet[idx].type) {
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
                    this->log_WARNING_HI_MVCAM_WaypointError(m_waypointSet[idx].index,
                                                            m_waypointSet[idx].type);
                    break;
            }
            m_waitingImage = true;
            m_imageTimeout = static_cast<NATIVE_UINT_TYPE>(MVCAM_IMG_WAIT_SEC);
        }
        else {
            this->log_ACTIVITY_HI_MVCAM_NoWaypointSpec(waypoint);
        }
        m_captureParamsLock.unLock();
    }

    void MVCamComponentImpl ::
      GncBufferReturn_handler(
          const NATIVE_INT_TYPE portNum,
          Fw::Buffer &fwBuffer
      )
    {
        Fw::SerializeStatus serStat;
        this->m_gncBufferReturnSerialBuffObj.resetSer();
        serStat = this->m_gncBufferReturnSerialBuffObj.serialize(fwBuffer);
        FW_ASSERT(Fw::FW_SERIALIZE_OK == serStat, serStat);

        Os::Queue::QueueStatus queueStat;
        queueStat = m_gncBufferReturnQueue.Send(this->m_gncBufferReturnSerialBuffObj.getBuffAddr(),
                                                this->m_gncBufferReturnSerialBuffObj.getBuffLength());
        if (Os::Queue::QUEUE_OK != queueStat) {
            this->log_WARNING_HI_MVCAM_GncBufferReturnQueueErr(static_cast<NATIVE_INT_TYPE>(queueStat));
            this->tlmWrite_MVCAM_NumGncBufferReturnQueueErrs(++m_numGncBufferReturnQueueErrs);
        }
    }

    void MVCamComponentImpl ::
      ImageBufferIn_handler(
          const NATIVE_INT_TYPE portNum,
          Fw::Buffer &fwBuffer
      )
    {
#ifdef DEBUG_MODE
        struct timeval tv;
        gettimeofday(&tv,NULL);
        DEBUG_PRINT("\nTaking m_bufferLock in ImageBufferIn_handler at time %f",
                    tv.tv_sec + tv.tv_usec / 1000000.0);
#endif // DEBUG_MODE

        // NOTE(mereweth) always low-priority type; high-priority type is sync handler
        const NATIVE_UINT_TYPE buffType = MVCAM_LP_BUFFER;

        this->m_bufferLock.lock();
        bool found = false;
        // search for open entry
        for (NATIVE_UINT_TYPE entry = 0; entry < MVCAM_IMG_MAX_NUM_BUFFERS; entry++) {
            // Look for slots to fill. "Available" is from
            // the perspective of the driver thread looking for
            // buffers to fill, so add the buffer and make it available.
            if (not this->m_buffSet[buffType][entry].available) {
                FW_ASSERT(buffType < FW_NUM_ARRAY_ELEMENTS(this->m_buffSet), buffType);
                FW_ASSERT(entry < FW_NUM_ARRAY_ELEMENTS(this->m_buffSet[buffType]), entry);
                this->m_buffSet[buffType][entry].readBuffer = fwBuffer;
                this->m_buffSet[buffType][entry].available = true;
                found = true;
                break;
            }
        }
        this->m_bufferLock.unLock();

        FW_ASSERT(found,fwBuffer.getbufferID(),fwBuffer.getmanagerID());

#ifdef DEBUG_MODE
        gettimeofday(&tv,NULL);
        DEBUG_PRINT("\nReleasing m_bufferLock in ImageBufferIn_handler at time %f",
                    tv.tv_sec + tv.tv_usec / 1000000.0);
#endif // DEBUG_MODE
    }

  // ----------------------------------------------------------------------
  // Command handler implementations
  // ----------------------------------------------------------------------

    void MVCamComponentImpl ::
      MVCAM_SET_LOGGING_cmdHandler(
          const FwOpcodeType opCode,
          const U32 cmdSeq,
          LogMode Mode
      )
    {
        //m_captureParamsLock.lock();

        m_logMode = Mode;

        switch(m_logMode) {
            case LOGGING_ALL:
                this->log_ACTIVITY_HI_MVCAM_LoggingModeChange(LOGGING_ALL_EVR);
                break;
            case LOGGING_PROC:
                this->log_ACTIVITY_HI_MVCAM_LoggingModeChange(LOGGING_PROC_EVR);
                break;
            case LOGGING_UNPROC:
                this->log_ACTIVITY_HI_MVCAM_LoggingModeChange(LOGGING_UNPROC_EVR);
                break;
            case LOGGING_WAYPT:
                this->log_ACTIVITY_HI_MVCAM_LoggingModeChange(LOGGING_WAYPT_EVR);
                break;
            case LOGGING_OFF:
                this->log_ACTIVITY_HI_MVCAM_LoggingModeChange(LOGGING_OFF_EVR);
                break;
            default:
                DEBUG_PRINT("Unknown logging mode in SET_LOGGING handler\n");
                FW_ASSERT(0, m_logMode);
                break;
        }

        //m_captureParamsLock.unLock();

        this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_OK);
    }

    void MVCamComponentImpl ::
      MVCAM_ADD_WAYPOINT_cmdHandler(
          const FwOpcodeType opCode,
          const U32 cmdSeq,
          U32 waypoint,
          WayptDestination destination,
          U8 quality
      )
    {
        if (!m_initialized) {
            this->log_WARNING_HI_MVCAM_UninitializedError();
            this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_EXECUTION_ERROR);
            return;
        }

        if (m_waitingImage) {
            this->log_WARNING_HI_MVCAM_BusyWaitingImage(waypoint);
            this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_EXECUTION_ERROR);
            return;
        }

        if ((quality > static_cast<U32>(Svc::IMGCOMP_TJ_MAX_QUALITY)) ||
            (quality < static_cast<U32>(Svc::IMGCOMP_TJ_MIN_QUALITY))) {
            this->log_ACTIVITY_HI_MVCAM_OverrodeQuality(quality,
							static_cast<U32>(Svc::IMGCOMP_TJ_MAX_QUALITY));
            quality = static_cast<U32>(Svc::IMGCOMP_TJ_MAX_QUALITY);
        }

        bool found = false;
        for (U32 i = 0; i < FW_NUM_ARRAY_ELEMENTS(m_waypointSet); i++) {
            if (m_waypointSet[i].used) {
                continue;
            }

            found = true;
            m_waypointSet[i].index = waypoint;
            m_waypointSet[i].quality = quality;
            m_waypointSet[i].type = destination;
            m_waypointSet[i].used = true;
            break;
        }

        if (found) {
            this->log_ACTIVITY_LO_MVCAM_WaypointAdded(waypoint,
                                                     static_cast<U32>(destination),
                                                     quality);
            this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_OK);
        }
        else {
            this->log_WARNING_HI_MVCAM_NoWaypointsLeft();
            this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_EXECUTION_ERROR);
        }
    }

    void MVCamComponentImpl ::
      MVCAM_TAKE_IMG_cmdHandler(
          const FwOpcodeType opCode,
          const U32 cmdSeq,
          TakeImgDestination destination,
          U8 quality
      )
    {
        if (!m_initialized) {
            this->log_WARNING_HI_MVCAM_UninitializedError();
            this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_EXECUTION_ERROR);
            return;
        }

        if (m_waitingImage) {
            // -1 indicates that the new image command was manual, not a waypoint
            this->log_WARNING_HI_MVCAM_BusyWaitingImage(-1);
            this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_BUSY);
            return;
        }

        m_captureParamsLock.lock();
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
                this->log_WARNING_HI_MVCAM_WaypointError(0, destination);
                break;
        }

        m_compQuality = quality;
        m_opCode = opCode;
        m_cmdSeq = cmdSeq;
        m_cmdTriggered = true;
        m_waitingImage = true;
        m_imageTimeout = static_cast<NATIVE_UINT_TYPE>(MVCAM_IMG_WAIT_SEC);
        m_captureParamsLock.unLock();

        if (m_activated) {
            m_wasActivated = true;
        }
        else {
            m_wasActivated = false;
            if (activate()) {
                this->log_WARNING_HI_MVCAM_CameraError(MVCAM_ACTIVATE_ERR);
                this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_EXECUTION_ERROR);
                switch (destination) {
                    case TAKE_IMG_STORAGE:
                    case TAKE_IMG_XMIT:
                    case TAKE_IMG_STORAGE_XMIT:
                        m_saveNextFrameProc = false;
                        break;
                    case TAKE_IMG_UNPROC:
                        m_saveNextFrameUnproc = false;
                        break;
                    default:
                        this->log_WARNING_HI_MVCAM_WaypointError(0, destination);
                        break;
                }
                return;
            }
        }
    }

    void MVCamComponentImpl ::
      MVCAM_ACTIVATE_cmdHandler(
          const FwOpcodeType opCode,
          const U32 cmdSeq,
          MVCamActive Mode
      )
    {
        if (!m_initialized) {
            this->log_WARNING_HI_MVCAM_UninitializedError();
            //DEBUG_PRINT("MVCam not initialized in MVCAM_ACTIVATE_cmdHandler");
            this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_EXECUTION_ERROR);
            return;
        }

        if ((m_activated && (Mode == IMAGING_ON)) ||
            (!m_activated && (Mode == IMAGING_OFF))) {

            // TODO(mereweth@jpl.nasa.gov) - log minor error?
            DEBUG_PRINT("MVCam got command for same activation state\n");
            this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_OK);
            return;
        }

        if (Mode == IMAGING_ON) {
            if (activate()) {
                this->log_WARNING_HI_MVCAM_CameraError(MVCAM_ACTIVATE_ERR);
                this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_EXECUTION_ERROR);
                return;
            }
        }
        else if(Mode == IMAGING_OFF) {
            if (deactivate()) {
                this->log_WARNING_HI_MVCAM_CameraError(MVCAM_DEACTIVATE_ERR);
                this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_EXECUTION_ERROR);
                return;
            }
        }

        this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_OK);
    }

    void MVCamComponentImpl ::
      MVCAM_TOGGLE_FIXED_EXPOSURE_cmdHandler(
          const FwOpcodeType opCode,
          const U32 cmdSeq,
          ExposureMode Mode
      )
    {
        m_exposureMode = Mode;

        this->log_WARNING_HI_MVCAM_ExposureModeChange(
                                  static_cast<ExposureModeEVR>(m_exposureMode));

        this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_OK);
    }

    void MVCamComponentImpl ::
      MVCAM_SET_EXPOSURE_GAIN_cmdHandler(
          const FwOpcodeType opCode,
          const U32 cmdSeq,
          U32 exposure,
          U32 gain
      )
    {
        if (!m_initialized) {
            this->log_WARNING_HI_MVCAM_UninitializedError();
            //DEBUG_PRINT("MVCam not initialized in MVCAM_SET_EXPOSURE_GAIN_cmdHandler");
            this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_EXECUTION_ERROR);
            return;
        }

        if (m_exposureMode == EXPOSURE_AUTO) {
            this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_EXECUTION_ERROR);
            return;
        }

        if (setExposureGain(exposure, gain)) {
            this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_EXECUTION_ERROR);
        }
        else {
            this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_OK);
        }
    }

    void MVCamComponentImpl ::
      MVCAM_OVERRIDE_POSTPROC_cmdHandler(
          const FwOpcodeType opCode,
          const U32 cmdSeq,
          U32 brightness,
          U32 contrast,
          PostProcISO iso,
          U32 sharpness
      )
    {
        if (!m_initialized) {
            this->log_WARNING_HI_MVCAM_UninitializedError();
            //DEBUG_PRINT("MVCam not initialized in MVCAM_OVERRIDE_POSTPROC_cmdHandler");
            this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_EXECUTION_ERROR);
            return;
        }

        if (setPostprocParams(brightness, contrast, iso, sharpness)) {
            this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_EXECUTION_ERROR);
        }
        else {
            this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_OK);
        }
    }

    void MVCamComponentImpl ::
      MVCAM_WAYPOINT_TEST_cmdHandler(
          const FwOpcodeType opCode,
          const U32 cmdSeq,
          U32 index
      )
    {
        this->WaypointRecv_handler(0, index);
        this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_OK);
    }

    void MVCamComponentImpl ::
      MVCAM_TEST_IMG_cmdHandler(
          const FwOpcodeType opCode,
          const U32 cmdSeq,
          TestImgDestination destination,
          U8 quality
      )
    {
        if (m_activated) {
            // -1 indicates that the new image command was manual, not a waypoint
            this->log_WARNING_HI_MVCAM_BusyWaitingImage(-1);
            this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_BUSY);
            return;
        }

        // find open buffer
        Fw::Buffer buff;
        bool entryFound = false;
        NATIVE_UINT_TYPE entry;

        this->m_bufferLock.lock();
        const U32 buffType = MVCAM_LP_BUFFER;
        for (entry = 0; entry < MVCAM_IMG_MAX_NUM_BUFFERS; entry++) {
            FW_ASSERT(buffType < FW_NUM_ARRAY_ELEMENTS(this->m_buffSet), buffType);
            FW_ASSERT(static_cast<unsigned long>(entry) < FW_NUM_ARRAY_ELEMENTS(this->m_buffSet[buffType]), entry);
            if (this->m_buffSet[buffType][entry].available) {
                this->m_buffSet[buffType][entry].available = false;
                buff = this->m_buffSet[buffType][entry].readBuffer;
                entryFound = true;
                break;
            }
        }

        if (entryFound) {
            BYTE* ptr = (BYTE*)buff.getdata();
            // shaded row test pattern
            const U32 numDivs = MVCAM_IMAGE_HEIGHT / 10;
            for (U32 i = 0; i < MVCAM_IMAGE_HEIGHT; i += numDivs) {
                memset(ptr + MVCAM_IMAGE_WIDTH * i,
                       static_cast<U8>(i * 255.0 / MVCAM_IMAGE_HEIGHT),
                       MVCAM_IMAGE_WIDTH * numDivs);
            }

            DEBUG_PRINT("\nMVCam Sending image out on image port in TEST_IMG handler\n");
            const U32 offsets[3] = {0u, 0u, 0u};
            const U32 strides[3] = {MVCAM_IMAGE_WIDTH, 0u, 0u};
	    Svc::CameraFrame camFrame;
            camFrame.settype(Svc::CAMFRAME_STILL);
            camFrame.setformat(Svc::CAMFMT_IMG_MVCAM_GRAY);
            camFrame.setdestination(static_cast<Svc::FrameDestType>(destination));
            camFrame.setquality(quality);
            camFrame.settimestamp(this->getTime());
            camFrame.setsize(MVCAM_IMAGE_SIZE);
            camFrame.setwidth(MVCAM_IMAGE_WIDTH);
            camFrame.setheight(MVCAM_IMAGE_HEIGHT);
            camFrame.setoffsets(&offsets[0], 3);
            camFrame.setstrides(&strides[0], 3);
            camFrame.setseq(0);
            camFrame.setexposure(0);
            camFrame.setgain(0);
            camFrame.setexposure_target(0.0f);
            camFrame.setgain_target(0.0f);
            camFrame.setframe(buff);
            DEBUG_PRINT("Before serialize of CameraFrame into Buffer\n");
            Fw::ExternalSerializeBuffer extBuf(ptr + MVCAM_IMAGE_SIZE,
                                               buff.getsize() - MVCAM_IMAGE_SIZE);
            Fw::SerializeStatus serStat = camFrame.serialize(extBuf);
            if (serStat != Fw::FW_SERIALIZE_OK) {
                this->m_buffSet[buffType][entry].available = true;
            }
            else {
                Fw::Buffer outBuff(0, 0, (U64) (ptr + MVCAM_IMAGE_SIZE),
                                   Svc::CameraFrame::SERIALIZED_SIZE);
                this->ProcSend_out(0, outBuff);
            }
            this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_OK);
        }
        else {
            this->log_ACTIVITY_HI_MVCAM_NoLPBuffers(1);
            DEBUG_PRINT("\nMVCAM no buffers in TEST_IMG handler\n");
            this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_EXECUTION_ERROR);
        }
        this->m_bufferLock.unLock();
    }

    void MVCamComponentImpl ::
      MVCAM_SET_EXP_PARMS_cmdHandler(
          const FwOpcodeType opCode,
          const U32 cmdSeq,
          U16 max_change,
          F32 exposure_cost,
          F32 gain_cost,
          U16 max_exposure,
          U16 max_gain
      )
    {
        if (!m_initialized) {
            this->log_WARNING_HI_MVCAM_UninitializedError();
            this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_EXECUTION_ERROR);
            return;
        }

        if (m_waitingImage || m_activated) {
            this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_EXECUTION_ERROR);
            return;
        }

        if ((exposure_cost <= 0) || (gain_cost <= 0)) {
            this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_EXECUTION_ERROR);
            return;
        }

        m_captureParamsLock.lock();
        m_maxExposureDelta = max_change;
        m_maxExposureScale = max_exposure;
        m_maxGainScale = max_gain;

#ifdef BUILD_SDFLIGHT
        // won't be NULL if we are initialized, and we check that above
        FW_ASSERT(m_mvCPAPtr != NULL, 0);
	// TODO(mereweth) - fix this!! probably have to call initialize again
        // mvCPA_SetCosts(m_mvCPAPtr, exposure_cost, gain_cost);
#endif // BUILD_SDFLIGHT
        m_captureParamsLock.unLock();

        this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_OK);
    }

    void MVCamComponentImpl ::
      MVCAM_FLIGHT_MODE_cmdHandler(
          const FwOpcodeType opCode,
          const U32 cmdSeq,
          MVCamFlightMode mode
      )
    {
        if (!m_initialized) {
            this->log_WARNING_HI_MVCAM_UninitializedError();
            this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_EXECUTION_ERROR);
            return;
        }

        m_captureParamsLock.lock();
        m_flightMode = mode;
        m_captureParamsLock.unLock();

        this->log_ACTIVITY_HI_MVCAM_FlightModeChange(
                                  static_cast<FlightModeEVR>(m_flightMode));

        this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_OK);
    }


    void MVCamComponentImpl ::
      MVCAM_IMG_TLM_MODE_cmdHandler(
          const FwOpcodeType opCode,
          const U32 cmdSeq,
          MVCamImgTlmMode mode
      )
    {
        if (!m_initialized) {
            this->log_WARNING_HI_MVCAM_UninitializedError();
            this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_EXECUTION_ERROR);
            return;
        }

        m_imgTlmMode = mode;

        this->log_ACTIVITY_LO_MVCAM_ImgTlmModeChange(
                                  static_cast<ImgTlmModeEVR>(m_imgTlmMode));

        this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_OK);
    }

    void MVCamComponentImpl ::
      MVCAM_SET_CALLBACK_cmdHandler(
          const FwOpcodeType opCode,
          const U32 cmdSeq,
          CallbackType Mode
      )
    {
        if (!m_initialized) {
            this->log_WARNING_HI_MVCAM_UninitializedError();
            this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_EXECUTION_ERROR);
            return;
        }

        if (m_waitingImage || m_activated) {
            this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_EXECUTION_ERROR);
            return;
        }

        if (setCallbackType(Mode)) {
            this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_EXECUTION_ERROR);
        }
        else {
            this->log_WARNING_HI_MVCAM_CallbackTypeChange(
                                      static_cast<CallbackTypeEVR>(m_callbackType));
            this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_OK);
        }
    }



    void MVCamComponentImpl ::
      MVCAM_SET_LOG_SKIP_cmdHandler(
          const FwOpcodeType opCode,
          const U32 cmdSeq,
          U32 skipCount
      )
    {
        if (!m_initialized) {
            this->log_WARNING_HI_MVCAM_UninitializedError();
            this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_EXECUTION_ERROR);
            return;
        }

        m_lpImageSkipCount = skipCount;
        m_lpImageSkip = skipCount;
        this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_OK);
    }

} // end namespace SnapdragonFlight
