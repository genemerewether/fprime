// ======================================================================
// \title  ImgCompImpl.cpp
// \author mereweth
// \brief  cpp file for ImgComp component implementation class
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


#include <Svc/ImgComp/ImgCompComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

#include <Svc/CameraFrame/CameraFrameSerializableAc.hpp>

#include <stdlib.h>
#include <sys/time.h>
#include <errno.h>

//#define DEBUG_MODE

#ifdef DEBUG_MODE
#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
#else
#define DEBUG_PRINT(x,...)
#endif //#ifdef DEBUG_MODE

namespace Svc {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

    ImgCompComponentImpl ::
      ImgCompComponentImpl(
      #if FW_OBJECT_NAMES == 1
          const char *const compName
      #endif
      ) :
        ImgCompComponentBase(
    #if FW_OBJECT_NAMES == 1
                           compName
    #endif
                           ),
        m_jpegHandle(NULL),
        m_buffersHandled(0u),
        m_deinterBuffer(NULL),
        m_deinterBufferSize(0u),
        m_tempCompBuffer(NULL),
        m_tempCompBufferSize(0u),
        m_thumbCompBuffer(NULL),
        m_thumbCompBufferSize(0u),
        m_setenvRet(0)
    {
        m_tempCompBufferSize = tjBufSize(IMGCOMP_TJ_MAX_WIDTH, IMGCOMP_TJ_MAX_HEIGHT, TJSAMP_420);
        m_tempCompBuffer = (BYTE*)malloc(m_tempCompBufferSize);
        FW_ASSERT(m_tempCompBuffer != NULL, 0);
        if (m_tempCompBuffer == NULL) {
            m_tempCompBufferSize = 0;
        }
        DEBUG_PRINT("TJ buffer max size %lu\n", m_tempCompBufferSize);

        m_thumbCompBufferSize = static_cast<U32>(IMGCOMP_TJ_THUMB_BUFFER_SIZE);
        m_thumbCompBuffer = (BYTE*)malloc(m_thumbCompBufferSize);
        FW_ASSERT(m_thumbCompBuffer != NULL, 0);
        if (m_thumbCompBuffer == NULL) {
            m_thumbCompBufferSize = 0;
        }

        m_deinterBufferSize = static_cast<U32>(IMGCOMP_TJ_MAX_WIDTH
                                               * IMGCOMP_TJ_MAX_HEIGHT / 4);
        m_deinterBuffer = (BYTE*)malloc(m_deinterBufferSize);
        FW_ASSERT(m_deinterBuffer != NULL, 0);
        if (m_deinterBuffer == NULL) {
            m_deinterBufferSize = 0;
        }
    }

    void ImgCompComponentImpl ::
      init(
          const NATIVE_INT_TYPE queueDepth,
          const NATIVE_INT_TYPE instance
      )
    {
        /* This environment variable is rechecked by TurboJpeg in the
         * compression function, but nothing else will change the env,
         * so it is ok to set it here. Also, the setenv function is not necessarily
         * thread-safe
         */
        m_setenvRet = setenv("TJ_RESTART", "1", 1);
#ifdef DEBUG_MODE
        system("echo ${TJ_RESTART}");
#endif
        if (m_setenvRet) {
            m_setenvRet = errno;
        }

        ImgCompComponentBase::init(queueDepth, instance);
    }

    ImgCompComponentImpl ::
      ~ImgCompComponentImpl(void)
    {
        tjDestroy(m_jpegHandle);
        free(m_tempCompBuffer);
        free(m_thumbCompBuffer);
    }

  // ----------------------------------------------------------------------
  // Implementations of methods inherited from base class
  // ----------------------------------------------------------------------

    void ImgCompComponentImpl ::
      preamble(void)
    {
        m_jpegHandle = tjInitCompress();
        if (m_jpegHandle == NULL) {
            Fw::LogStringArg softCompErr(tjGetErrorStr());
            this->log_WARNING_HI_IMGCOMP_SoftCompError(SOFTCOMP_INIT_ERR,
                                                       softCompErr);
        }

        if (m_setenvRet) {
            this->log_WARNING_HI_IMGCOMP_NoRestartMarkers(m_setenvRet);
        }
    }

    void ImgCompComponentImpl ::
      finalizer(void)
    {

    }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

    void ImgCompComponentImpl ::
      uncompressedIn_handler(
          const NATIVE_INT_TYPE portNum,
          Fw::Buffer &fwBuffer
      )
    {
        m_buffersHandled++;

#ifdef DEBUG_MODE
        struct timeval tv;
        gettimeofday(&tv,NULL);
        DEBUG_PRINT("\nImgComp uncompressedIn handler at time %f\n",
                    tv.tv_sec + tv.tv_usec / 1000000.0);
#endif // DEBUG_MODE
        CameraFrame camFrame;
        Fw::ExternalSerializeBuffer extBuf((BYTE*) fwBuffer.getdata(),
                                           fwBuffer.getsize());
        extBuf.setBuffLen(fwBuffer.getsize());
        Fw::SerializeStatus serStat = extBuf.deserialize(camFrame);
        if (serStat != Fw::FW_SERIALIZE_OK) {
            this->log_WARNING_HI_IMGCOMP_BadBuffer();
            DEBUG_PRINT("ImgComp uncompressedIn bad buffer\n");
            // Not returning buffer - we don't know what to return
            return;
        }
        Fw::Buffer imgBuffer = camFrame.getframe();

        if (camFrame.gettype() != CAMFRAME_STILL) {
            DEBUG_PRINT("ImgComp uncompressedIn not a still frame\n");
            this->log_WARNING_HI_IMGCOMP_BadSetting(portNum);
            this->uncompressedReturn_out(portNum, imgBuffer);
            return;
        }

        int tjFlags = TJFLAG_FASTDCT | TJFLAG_NOREALLOC;
        int tjSamp = 0;
        U64 imgBufData = imgBuffer.getdata();
        U32 imgBufSize = camFrame.getsize(); // book-keep separately - imgBuffer may have extra storage
        FW_ASSERT(imgBufData != 0, imgBufData);
#ifdef BUILD_SDFLIGHT
        uint8_t* planes[3];
        uint8_t* tempPlane;
#else
        const uint8_t* planes[3];
        const uint8_t* tempPlane;
#endif
        I32 strides[3];
        NATIVE_INT_TYPE offsetsSize = 0;
        NATIVE_INT_TYPE stridesSize = 0;
        const U32* offsets = camFrame.getoffsets(offsetsSize);
        const U32* stridesUnsigned = camFrame.getstrides(stridesSize);
        FW_ASSERT(offsets != NULL, 0);
        FW_ASSERT(stridesUnsigned != NULL, 0);
        FW_ASSERT(FW_NUM_ARRAY_ELEMENTS(planes) == offsetsSize,
		  FW_NUM_ARRAY_ELEMENTS(planes),
		  offsetsSize);
        FW_ASSERT(FW_NUM_ARRAY_ELEMENTS(strides) == stridesSize,
		  FW_NUM_ARRAY_ELEMENTS(strides),
		  stridesSize);
        for (int i = 0; i < offsetsSize; i++) {
            planes[i] = (uint8_t*) imgBufData + offsets[i];
            strides[i] = static_cast<I32>(stridesUnsigned[i]);
        }

        unsigned long ySize = 0u;
        unsigned long cbSize = 0u;
        unsigned long crSize = 0u;
        switch (camFrame.getformat()) {
            case CAMFMT_IMG_HIRES_YUV420SP:
            {
                tjSamp = TJSAMP_420;
                ySize = tjPlaneSizeYUV(0, camFrame.getwidth(), strides[0],
                                       camFrame.getheight(), tjSamp);
                cbSize = tjPlaneSizeYUV(1, camFrame.getwidth(), strides[1],
                                        camFrame.getheight(), tjSamp);
                crSize = tjPlaneSizeYUV(2, camFrame.getwidth(), strides[2],
                                        camFrame.getheight(), tjSamp);
                if (((U64) planes[0] + (U64) ySize  >= (U64) planes[1]) ||
                    // NOTE(mereweth) - interleaved anyway
                    //((U64) planes[1] + (U64) cbSize >= (U64) planes[2]) ||
                    ((U64) planes[1] <= (U64) planes[0])                ||
                    // NOTE(mereweth) - interleaved anyway
                    //((U64) planes[2] <= (U64) planes[1])                ||
                    (ySize + cbSize + crSize > imgBufSize))             {
                    DEBUG_PRINT("HIRES_YUV420SP too big; planes: [%llu,%llu,%llu], imgBufData %llu, imgBufSize %llu\n",
                                (U64) planes[0], (U64) planes[1], (U64) planes[2],
                                (U64) imgBufData, (U64) imgBufSize);
                    this->log_WARNING_HI_IMGCOMP_BadSetting(portNum);
                    this->uncompressedReturn_out(portNum, imgBuffer);
                    return;
                }
                // deinterleave
                U32 deinterSize = camFrame.getheight()
                                  * camFrame.getwidth() / 4;
                if ((m_deinterBufferSize < deinterSize) ||
                    (m_deinterBuffer == NULL))           {
                    Fw::LogStringArg msg("No deinterleave buffer");
                    this->log_WARNING_HI_IMGCOMP_SoftCompError(SOFTCOMP_OP_ERR,
                                                               msg);
                    this->uncompressedReturn_out(portNum, imgBuffer);
                    return;
                }

                /* tempPtr is just to avoid the const-ness of memory pointed to
                 * by planes[1], which is required for Linux TurboJpeg APIs
                 */
                uint8_t* tempPtr = (uint8_t*) imgBufData + offsets[1];
                for (U32 i = 0; i < deinterSize; i++) {
                    FW_ASSERT((U64) tempPtr + 2*i + 1 < imgBufData + imgBufSize,
                              (U64) tempPtr, i, deinterSize, imgBufData, imgBufSize);
                    // copy Cr data to new buffer
                    m_deinterBuffer[i] = *(tempPtr+ 2*i + 1);

                    // copy Cb data so that it is contiguous
                    *(tempPtr + i) = *(tempPtr+ 2*i);
                }
                planes[2] = m_deinterBuffer;

                // NOTE(mereweth) U and V channels are swapped
                tempPlane = planes[2];
                planes[2] = planes[1];
                planes[1] = tempPlane;
            }
                break;
            case CAMFMT_IMG_MVCAM_GRAY:
            case CAMFMT_IMG_HIRES_GRAY:
                tjFlags |= TJXOPT_GRAY;
                tjSamp = TJSAMP_GRAY;
                ySize = tjPlaneSizeYUV(0, camFrame.getwidth(), strides[0],
                                       camFrame.getheight(), tjSamp);


                if (ySize > imgBufSize) {
                    this->log_WARNING_HI_IMGCOMP_BadSetting(portNum);
                    DEBUG_PRINT("IMGCMP GRAY too big; %lu vs %lu\n",
                                ySize, imgBufSize);
                    this->uncompressedReturn_out(portNum, imgBuffer);
                    return;
                }
                break;
            /* NOTE(mereweth) precompressed by camera pipeline - dummy buffer
             * is so there are 3 buffers out per input buffer (thumbnail is the 3rd)
             */
            case CAMFMT_IMG_JPEG:
            {
                if ((camFrame.getdestination() == FRAMEDEST_XMIT)       ||
                    (camFrame.getdestination() == FRAMEDEST_STORAGE_XMIT)) {

                    // NOTE(mereweth) push a 0-size-buffer for thumb for logger count
                    getBuffAndPushXmit(portNum,
                                       fwBuffer.getsize(), (void*) fwBuffer.getdata(),
                                       0, NULL,
                                       imgBufSize, (void*) imgBufData);
                }

                if ((camFrame.getdestination() == FRAMEDEST_STORAGE)       ||
                    (camFrame.getdestination() == FRAMEDEST_STORAGE_XMIT)) {

                    // NOTE(mereweth) push a 0-size-buffer for thumb for logger count
                    getBuffAndPushStorage(portNum,
                                          fwBuffer.getsize(), (void*) fwBuffer.getdata(),
                                          0, NULL,
                                          imgBufSize, (void*) imgBufData);
                }

                DEBUG_PRINT("ImgComp got an already-compressed image\n");
                this->uncompressedReturn_out(portNum, imgBuffer);
                return;
            }
                break;
            case CAMFMT_IMG_MVCAM_RAW:
            case CAMFMT_IMG_HIRES_RAW: // for debugging
                DEBUG_PRINT("ImgComp got a raw format image\n");
                this->log_WARNING_HI_IMGCOMP_BadSetting(portNum);
                this->uncompressedReturn_out(portNum, imgBuffer);
                return;
                break;
            case CAMFMT_IMG_UNKNOWN:
            default:
                this->log_WARNING_HI_IMGCOMP_BadSetting(portNum);
                DEBUG_PRINT("ImgComp got an unknown image type %d\n",
                            camFrame.getformat());
                this->uncompressedReturn_out(portNum, imgBuffer);
                return;
        }

        DEBUG_PRINT("Y plane size %lu, U plane size %lu, V plane size %lu\n",
                    ySize, cbSize, crSize);

        DEBUG_PRINT("Offsets [%u,%u,%u]; Planes [%llu,%llu,%llu]; Buf %llu\n",
                    offsets[0], offsets[1], offsets[2],
                    (U64) planes[0], (U64) planes[1], (U64) planes[2],
                    imgBufData);

        if ((camFrame.getquality() > static_cast<U32>(IMGCOMP_TJ_MAX_QUALITY)) ||
            (camFrame.getquality() < static_cast<U32>(IMGCOMP_TJ_MIN_QUALITY))) {
            DEBUG_PRINT("ImgComp got invalid quality %d on port %d\n",
                        camFrame.getquality(), portNum);
            this->log_WARNING_HI_IMGCOMP_BadSetting(portNum);
            this->uncompressedReturn_out(portNum, imgBuffer);
            return;
        }

        if (NULL == m_jpegHandle) {
            Fw::LogStringArg softCompErr(tjGetErrorStr());
            this->log_WARNING_HI_IMGCOMP_SoftCompError(SOFTCOMP_INIT_ERR,
                                                       softCompErr);
            this->uncompressedReturn_out(portNum, imgBuffer);
            return;
        }

#ifdef DEBUG_MODE
        gettimeofday(&tv,NULL);
        DEBUG_PRINT("Before compressing buffer of type %d, size %d at time %f\n",
                    camFrame.getformat(), imgBufSize,
                    tv.tv_sec + tv.tv_usec / 1000000.0);
#endif // DEBUG_MODE

        double widthScaling = (double) camFrame.getwidth() /
                            (double) IMGCOMP_TJ_THUMB_WIDTH;

        double heightScaling = (double) camFrame.getheight() /
                            (double) IMGCOMP_TJ_THUMB_HEIGHT;

        double scaling = 0.0L;
        if (widthScaling > heightScaling) {
            scaling = widthScaling;
        }
        else {
            scaling = heightScaling;
        }

        memset(m_thumbCompBuffer, 0, m_thumbCompBufferSize);
        unsigned long thumbCompBufferSize = m_thumbCompBufferSize;
        for (int h = 0; h < IMGCOMP_TJ_THUMB_HEIGHT; h++) {
            NATIVE_UINT_TYPE destRowOffset = h * IMGCOMP_TJ_THUMB_WIDTH;
            if (destRowOffset >= thumbCompBufferSize) {  continue;  }
            NATIVE_UINT_TYPE srcRowOffset = (NATIVE_UINT_TYPE)
                                            ((double) h * scaling *
                                             (double) camFrame.getwidth());
            if (srcRowOffset >= ySize) {  continue;  }
            for (int w = 0; w < IMGCOMP_TJ_THUMB_WIDTH; w++) {
                NATIVE_UINT_TYPE destIdx = destRowOffset + w;
                if (destIdx >= thumbCompBufferSize) {  continue;  }
                NATIVE_UINT_TYPE srcIdx = srcRowOffset + (NATIVE_UINT_TYPE)
                                          ((double) w * scaling);
                if (srcIdx >= ySize) {  continue;  }

                m_thumbCompBuffer[destIdx] = planes[0][srcIdx];
            }
        }

        // NOTE(mereweth) - removed to make thumbnail constant size
        /*int thumbStat = tjCompressFromYUVPlanes(m_jpegHandle,
                                                &planes[0],
                                                camFrame.getwidth(),
                                                strides,
                                                camFrame.getheight(),
                                                tjSamp,
                                                &m_tempCompBuffer,
                                                &thumbCompBufferSize,
                                                1, // quality of 1
                                                tjFlags);
        thumbCompBufferSize = FW_MIN(thumbCompBufferSize, m_thumbCompBufferSize);
        memcpy((void*)m_thumbCompBuffer, m_tempCompBuffer, thumbCompBufferSize);

        if (thumbStat == -1) {
            Fw::LogStringArg softCompErr(tjGetErrorStr());
            this->log_WARNING_HI_IMGCOMP_SoftCompError(SOFTCOMP_OP_ERR,
                                                      softCompErr);
            DEBUG_PRINT("TurboJpeg thumbnail error %s\n", tjGetErrorStr());
        }*/

#ifdef DEBUG_MODE
        gettimeofday(&tv,NULL);
        DEBUG_PRINT("After compressing thumbnail to size %lu at time %f\n",
                    thumbCompBufferSize,
                    tv.tv_sec + tv.tv_usec / 1000000.0);
#endif // DEBUG_MODE

        unsigned long tempCompBufferSize = m_tempCompBufferSize;
        int stat = tjCompressFromYUVPlanes(m_jpegHandle,
                                           &planes[0],
                                           camFrame.getwidth(),
                                           strides,
                                           camFrame.getheight(),
                                           tjSamp,
                                           &m_tempCompBuffer,
                                           &tempCompBufferSize,
                                           camFrame.getquality(),
                                           tjFlags);
        tempCompBufferSize = FW_MIN(tempCompBufferSize, m_tempCompBufferSize);

#ifdef DEBUG_MODE
        gettimeofday(&tv,NULL);
        DEBUG_PRINT("After compressing buffer of type %d to size %lu at time %f\n",
                    camFrame.getformat(), tempCompBufferSize,
                    tv.tv_sec + tv.tv_usec / 1000000.0);

        {
            FILE* filp = fopen("./test.jpg", "wb");
            if (filp == NULL) {
                DEBUG_PRINT("Could not create jpeg test file\n");
            }
            else {
                fwrite((const void*)m_tempCompBuffer, 1, tempCompBufferSize, filp);
                fclose(filp);
            }
            filp = NULL;
        }
#endif // DEBUG_MODE

        // TODO(mereweth) - do we care about the thumbnail status?
        if (stat == -1) {
            Fw::LogStringArg softCompErr(tjGetErrorStr());
            this->log_WARNING_HI_IMGCOMP_SoftCompError(SOFTCOMP_OP_ERR,
                                                       softCompErr);
            DEBUG_PRINT("TurboJpeg error %s\n", tjGetErrorStr());
        }
        else {
            if ((camFrame.getdestination() == FRAMEDEST_XMIT)          ||
                (camFrame.getdestination() == FRAMEDEST_STORAGE_XMIT)) {

                getBuffAndPushXmit(portNum,
                                   fwBuffer.getsize(), (void*) fwBuffer.getdata(),
                                   thumbCompBufferSize, m_thumbCompBuffer,
                                   tempCompBufferSize, m_tempCompBuffer);
            }

            if ((camFrame.getdestination() == FRAMEDEST_STORAGE)       ||
                (camFrame.getdestination() == FRAMEDEST_STORAGE_XMIT)) {

                getBuffAndPushStorage(portNum,
                                      fwBuffer.getsize(), (void*) fwBuffer.getdata(),
                                      thumbCompBufferSize, m_thumbCompBuffer,
                                      tempCompBufferSize, m_tempCompBuffer);
            }

            if ((camFrame.getdestination() != FRAMEDEST_STORAGE)      &&
                (camFrame.getdestination() != FRAMEDEST_STORAGE_XMIT) &&
                (camFrame.getdestination() != FRAMEDEST_XMIT))        {
                DEBUG_PRINT("ImgComp got invalid destination %d\n",
                            camFrame.getdestination());
                this->log_WARNING_HI_IMGCOMP_BadSetting(portNum);
            }
        }

        this->uncompressedReturn_out(portNum, imgBuffer);
    }

    void ImgCompComponentImpl ::
      pushEmptyBuffStorage(U32 portNum,
                           BufferOffsetSkipType skip)
    {
        Fw::Buffer outBuff = this->compressedGetStorage_out(portNum, 0);
        if (0 == outBuff.getdata()) {
            this->log_WARNING_HI_IMGCOMP_BufferOffset(skip,
                                                      OUTPUT_STORAGE,
                                                      m_buffersHandled,
                                                      portNum);
        }
        else {
            this->compressedOutStorage_out(portNum, outBuff);
        }
    }

    void ImgCompComponentImpl ::
      pushEmptyBuffXmit(U32 portNum,
                        BufferOffsetSkipType skip)
    {
        Fw::Buffer outBuff = this->compressedGetXmit_out(portNum, 0);
        if (0 == outBuff.getdata()) {
            this->log_WARNING_HI_IMGCOMP_BufferOffset(skip,
                                                      OUTPUT_XMIT,
                                                      m_buffersHandled,
                                                      portNum);
        }
        else {
            this->compressedOutXmit_out(portNum, outBuff);
        }
    }


    void ImgCompComponentImpl ::
      getBuffAndPushStorage(U32 portNum,
                            U32 metaSize, void* metaPtr,
                            U32 thumbSize, void* thumbPtr,
                            U32 imgSize, void* imgPtr)
    {
        bool success = true;
        Fw::Buffer outMetaBuff;
        Fw::Buffer outThumbBuff;
        Fw::Buffer outImgBuff;

        outMetaBuff = this->compressedGetStorage_out(portNum, metaSize);

        // NOTE(mereweth) - allow requesting empty buffer for counting purposes
        if ((0 == outMetaBuff.getdata())                   ||
            (metaSize != outMetaBuff.getsize())) {
            success = false;
            this->log_WARNING_HI_IMGCOMP_NoBuffer(metaSize);
        }
        else { // buffer is good
            if (metaSize != 0) {
                memcpy((void*) outMetaBuff.getdata(),
                       metaPtr, metaSize);
            }
        }

        if (!success) { // getting outMetaBuff failed
            // zero size buffer for counting purposes
            pushEmptyBuffStorage(portNum, SKIP_METADATA);
            pushEmptyBuffStorage(portNum, SKIP_THUMB);
            pushEmptyBuffStorage(portNum, SKIP_IMAGE);
            return;
        }
        else {
            outThumbBuff = this->compressedGetStorage_out(portNum, thumbSize);

            // NOTE(mereweth) - allow requesting empty buffer for counting purposes
            if ((0 == outThumbBuff.getdata())                   ||
                (thumbSize != outThumbBuff.getsize())) {
                success = false;
                this->log_WARNING_HI_IMGCOMP_NoBuffer(thumbSize);
            }
            else { // buffer is good
                if (thumbSize != 0) {
                    memcpy((void*) outThumbBuff.getdata(),
                           thumbPtr, thumbSize);
                }
            }
        }

        if (!success) { // getting outThumbBuff failed
            this->compressedOutStorage_out(portNum, outMetaBuff);
            // zero size buffer for counting purposes
            pushEmptyBuffStorage(portNum, SKIP_THUMB);
            pushEmptyBuffStorage(portNum, SKIP_IMAGE);
            return;
        }
        else {
            outImgBuff = this->compressedGetStorage_out(portNum, imgSize);

            // NOTE(mereweth) - allow requesting empty buffer for counting purposes
            if ((0 == outImgBuff.getdata())                   ||
                (imgSize != outImgBuff.getsize())) {
                success = false;
                this->log_WARNING_HI_IMGCOMP_NoBuffer(imgSize);
            }
            else { // buffer is good
                if (imgSize != 0) {
                    memcpy((void*) outImgBuff.getdata(),
                           imgPtr, imgSize);
                }
            }
        }

        if (!success) { // getting outImgBuff failed
            this->compressedOutStorage_out(portNum, outMetaBuff);
            this->compressedOutStorage_out(portNum, outThumbBuff);
            // zero size buffer for counting purposes
            pushEmptyBuffStorage(portNum, SKIP_IMAGE);
        }
        else { // getting all buffers succeeded
           this->compressedOutStorage_out(portNum, outMetaBuff);
           this->compressedOutStorage_out(portNum, outThumbBuff);
           this->compressedOutStorage_out(portNum, outImgBuff);
       }
    }

    void ImgCompComponentImpl ::
      getBuffAndPushXmit(U32 portNum,
                         U32 metaSize, void* metaPtr,
                         U32 thumbSize, void* thumbPtr,
                         U32 imgSize, void* imgPtr)
    {
        bool success = true;
        Fw::Buffer outMetaBuff;
        Fw::Buffer outThumbBuff;
        Fw::Buffer outImgBuff;

        outMetaBuff = this->compressedGetXmit_out(portNum, metaSize);

        // NOTE(mereweth) - allow requesting empty buffer for counting purposes
        if ((0 == outMetaBuff.getdata())                   ||
            (metaSize != outMetaBuff.getsize())) {
            success = false;
            this->log_WARNING_HI_IMGCOMP_NoBuffer(metaSize);
        }
        else { // buffer is good
            if (metaSize != 0) {
                memcpy((void*) outMetaBuff.getdata(),
                       metaPtr, metaSize);
            }
        }

        if (!success) { // getting outMetaBuff failed
            // zero size buffer for counting purposes
            pushEmptyBuffXmit(portNum, SKIP_METADATA);
            pushEmptyBuffXmit(portNum, SKIP_THUMB);
            pushEmptyBuffXmit(portNum, SKIP_IMAGE);
            return;
        }
        else {
            outThumbBuff = this->compressedGetXmit_out(portNum, thumbSize);

            // NOTE(mereweth) - allow requesting empty buffer for counting purposes
            if ((0 == outThumbBuff.getdata())                   ||
                (thumbSize != outThumbBuff.getsize())) {
                success = false;
                this->log_WARNING_HI_IMGCOMP_NoBuffer(thumbSize);
            }
            else { // buffer is good
                if (thumbSize != 0) {
                    memcpy((void*) outThumbBuff.getdata(),
                           thumbPtr, thumbSize);
                }
            }
        }

        if (!success) { // getting outThumbBuff failed
            this->compressedOutXmit_out(portNum, outMetaBuff);
            // zero size buffer for counting purposes
            pushEmptyBuffXmit(portNum, SKIP_THUMB);
            pushEmptyBuffXmit(portNum, SKIP_IMAGE);
            return;
        }
        else {
            outImgBuff = this->compressedGetXmit_out(portNum, imgSize);

            // NOTE(mereweth) - allow requesting empty buffer for counting purposes
            if ((0 == outImgBuff.getdata())                   ||
                (imgSize != outImgBuff.getsize())) {
                success = false;
                this->log_WARNING_HI_IMGCOMP_NoBuffer(imgSize);
            }
            else { // buffer is good
                if (imgSize != 0) {
                    memcpy((void*) outImgBuff.getdata(),
                           imgPtr, imgSize);
                }
            }
        }

        if (!success) { // getting outImgBuff failed
            this->compressedOutXmit_out(portNum, outMetaBuff);
            this->compressedOutXmit_out(portNum, outThumbBuff);
            // zero size buffer for counting purposes
            pushEmptyBuffXmit(portNum, SKIP_IMAGE);
        }
        else { // getting all buffers succeeded
            this->compressedOutXmit_out(portNum, outMetaBuff);
            this->compressedOutXmit_out(portNum, outThumbBuff);
            this->compressedOutXmit_out(portNum, outImgBuff);
        }
    }

    void ImgCompComponentImpl ::
      pingIn_handler(
          const NATIVE_INT_TYPE portNum,
          U32 key
      )
    {
        // respond to ping
        this->pingOut_out(0,key);
    }

    void ImgCompComponentImpl ::
      schedIn_handler(
          const NATIVE_INT_TYPE portNum,
          NATIVE_UINT_TYPE context
      )
    {
        this->tlmWrite_IMGCOMP_BuffersHandled(m_buffersHandled);
    }

  // ----------------------------------------------------------------------
  // Command handler implementations
  // ----------------------------------------------------------------------

    void ImgCompComponentImpl ::
      IMGCOMP_NoOp_cmdHandler(
          const FwOpcodeType opCode,
          const U32 cmdSeq
      )
    {
        this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_OK);
    }

} // end namespace Svc
