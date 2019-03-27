// ======================================================================
// \title  ImgComp.hpp
// \author mereweth
// \brief  cpp file for ImgComp test harness implementation class
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

#include "Tester.hpp"

#define INSTANCE 0
#define MAX_HISTORY_SIZE 10
#define QUEUE_DEPTH 10

namespace Svc {

  // ----------------------------------------------------------------------
  // Construction and destruction
  // ----------------------------------------------------------------------

  Tester ::
    Tester(void) :
#if FW_OBJECT_NAMES == 1
      ImgCompGTestBase("Tester", MAX_HISTORY_SIZE),
      component("ImgComp"),
#else
      ImgCompGTestBase(MAX_HISTORY_SIZE),
      component(),
#endif
      m_bufGetMode(METADATA_BUF),
      m_metadataBuf(NULL),
      m_metadataBufSize(0u),
      m_thumbBuf(NULL),
      m_thumbBufSize(0u),
      m_imageBuf(NULL),
      m_imageBufSize(0u),
      m_bufGetFailMode(GET_BUF_SUCCEED),
      m_bufGetCountFailMode(GET_BUF_SUCCEED),
      m_bufMgrID(0),
      m_bufID(0)
  {
    this->initComponents();
    this->connectPorts();
  }

  Tester ::
    ~Tester(void)
  {

  }

  // ----------------------------------------------------------------------
  // Test nominal operations
  // ----------------------------------------------------------------------

  void Tester ::
    bufGetAllOk(void)
  {
      char metaBuf[10];
      char thumbBuf[20];
      char imageBuf[30];

      // give BufferGet handler some memory to return
      char toMetaBuf[10];
      char toThumbBuf[20];
      char toImageBuf[30];
      m_metadataBuf = toMetaBuf;
      m_metadataBufSize = sizeof(metaBuf);
      m_thumbBuf = toThumbBuf;
      m_thumbBufSize = sizeof(thumbBuf);
      m_imageBuf = toImageBuf;
      m_imageBufSize = sizeof(imageBuf);

      component.getBuffAndPushStorage(0,
                                      sizeof(metaBuf), metaBuf,
                                      sizeof(thumbBuf), thumbBuf,
                                      sizeof(imageBuf), imageBuf);

      ASSERT_from_compressedGetStorage_SIZE(3);

      ASSERT_from_compressedGetStorage(0, sizeof(metaBuf));
      ASSERT_from_compressedGetStorage(1, sizeof(thumbBuf));
      ASSERT_from_compressedGetStorage(2, sizeof(imageBuf));

      ASSERT_from_compressedOutStorage_SIZE(3);

      ASSERT_from_compressedOutStorage(0, Fw::Buffer(0, 0,
                                                     (U64) m_metadataBuf,
                                                     m_metadataBufSize));
      ASSERT_from_compressedOutStorage(1, Fw::Buffer(0, 1,
                                                     (U64) m_thumbBuf,
                                                     m_thumbBufSize));
      ASSERT_from_compressedOutStorage(2, Fw::Buffer(0, 2,
                                                     (U64) m_imageBuf,
                                                     m_imageBufSize));

      ASSERT_EVENTS_SIZE(0);

      m_bufGetMode = METADATA_BUF;
      m_bufGetFailMode = GET_BUF_SUCCEED;
      m_bufMgrID = 1;
      m_bufID = 0;

      component.getBuffAndPushXmit(0,
                                   sizeof(metaBuf), metaBuf,
                                   sizeof(thumbBuf), thumbBuf,
                                   sizeof(imageBuf), imageBuf);

      ASSERT_from_compressedGetXmit_SIZE(3);

      ASSERT_from_compressedGetXmit(0, sizeof(metaBuf));
      ASSERT_from_compressedGetXmit(1, sizeof(thumbBuf));
      ASSERT_from_compressedGetXmit(2, sizeof(imageBuf));

      ASSERT_from_compressedOutXmit_SIZE(3);

      ASSERT_from_compressedOutXmit(0, Fw::Buffer(1, 0,
                                                  (U64) m_metadataBuf,
                                                  m_metadataBufSize));
      ASSERT_from_compressedOutXmit(1, Fw::Buffer(1, 1,
                                                  (U64) m_thumbBuf,
                                                  m_thumbBufSize));
      ASSERT_from_compressedOutXmit(2, Fw::Buffer(1, 2,
                                                  (U64) m_imageBuf,
                                                  m_imageBufSize));

      ASSERT_EVENTS_SIZE(0);
  }

  // ----------------------------------------------------------------------
  // Test off-nominal buffer get calls
  // ----------------------------------------------------------------------

  void Tester ::
    bufGetNoMeta(void)
  {
      char metaBuf[10];
      char thumbBuf[20];
      char imageBuf[30];

      // give BufferGet handler some memory to return
      char toMetaBuf[10];
      char toThumbBuf[20];
      char toImageBuf[30];
      m_metadataBuf = toMetaBuf;
      m_metadataBufSize = sizeof(metaBuf);
      m_thumbBuf = toThumbBuf;
      m_thumbBufSize = sizeof(thumbBuf);
      m_imageBuf = toImageBuf;
      m_imageBufSize = sizeof(imageBuf);

      m_bufGetFailMode = FAIL_GET_METADATA_BUF;

      component.getBuffAndPushStorage(0,
                                      sizeof(metaBuf), metaBuf,
                                      sizeof(thumbBuf), thumbBuf,
                                      sizeof(imageBuf), imageBuf);

      ASSERT_EQ(m_bufGetMode, METADATA_BUF);

      ASSERT_from_compressedGetStorage_SIZE(4);
      ASSERT_from_compressedGetStorage(0, sizeof(metaBuf));
      // 3 buffers for counting purposes
      ASSERT_from_compressedGetStorage(1, 0);
      ASSERT_from_compressedGetStorage(2, 0);
      ASSERT_from_compressedGetStorage(3, 0);

      ASSERT_from_compressedOutStorage_SIZE(3);
      ASSERT_from_compressedOutStorage(0, Fw::Buffer(0, 0,
                                                     (U64) m_metadataBuf, 0));
      ASSERT_from_compressedOutStorage(1, Fw::Buffer(0, 1,
                                                     (U64) m_thumbBuf, 0));
      ASSERT_from_compressedOutStorage(2, Fw::Buffer(0, 2,
                                                     (U64) m_imageBuf, 0));

      ASSERT_EVENTS_IMGCOMP_NoBuffer_SIZE(1);
      ASSERT_EVENTS_IMGCOMP_NoBuffer(0, sizeof(metaBuf));

      ASSERT_EVENTS_IMGCOMP_BufferOffset_SIZE(0);

      clearHistory();

      m_bufMgrID = 1;
      m_bufID = 0;

      component.getBuffAndPushXmit(0,
                                   sizeof(metaBuf), metaBuf,
                                   sizeof(thumbBuf), thumbBuf,
                                   sizeof(imageBuf), imageBuf);

      ASSERT_EQ(m_bufGetMode, METADATA_BUF);

      ASSERT_from_compressedGetXmit_SIZE(4);
      ASSERT_from_compressedGetXmit(0, sizeof(metaBuf));
      // 3 buffers for counting purposes
      ASSERT_from_compressedGetXmit(1, 0);
      ASSERT_from_compressedGetXmit(2, 0);
      ASSERT_from_compressedGetXmit(3, 0);

      ASSERT_from_compressedOutXmit_SIZE(3);
      ASSERT_from_compressedOutXmit(0, Fw::Buffer(1, 0,
                                                  (U64) m_metadataBuf, 0));
      ASSERT_from_compressedOutXmit(1, Fw::Buffer(1, 1,
                                                  (U64) m_thumbBuf, 0));
      ASSERT_from_compressedOutXmit(2, Fw::Buffer(1, 2,
                                                  (U64) m_imageBuf, 0));

      ASSERT_EVENTS_IMGCOMP_NoBuffer_SIZE(1);
      ASSERT_EVENTS_IMGCOMP_NoBuffer(0, sizeof(metaBuf));

      ASSERT_EVENTS_IMGCOMP_BufferOffset_SIZE(0);
  }

  void Tester ::
    bufGetNoThumb(void)
  {
      char metaBuf[10];
      char thumbBuf[20];
      char imageBuf[30];

      // give BufferGet handler some memory to return
      char toMetaBuf[10];
      char toThumbBuf[20];
      char toImageBuf[30];
      m_metadataBuf = toMetaBuf;
      m_metadataBufSize = sizeof(metaBuf);
      m_thumbBuf = toThumbBuf;
      m_thumbBufSize = sizeof(thumbBuf);
      m_imageBuf = toImageBuf;
      m_imageBufSize = sizeof(imageBuf);

      m_bufGetFailMode = FAIL_GET_THUMB_BUF;

      component.getBuffAndPushStorage(0,
                                      sizeof(metaBuf), metaBuf,
                                      sizeof(thumbBuf), thumbBuf,
                                      sizeof(imageBuf), imageBuf);

      ASSERT_EQ(m_bufGetMode, METADATA_BUF);

      ASSERT_from_compressedGetStorage_SIZE(4);
      ASSERT_from_compressedGetStorage(0, sizeof(metaBuf));
      ASSERT_from_compressedGetStorage(1, sizeof(thumbBuf));
      // 2 buffers for counting purposes
      ASSERT_from_compressedGetStorage(2, 0);
      ASSERT_from_compressedGetStorage(3, 0);

      ASSERT_from_compressedOutStorage_SIZE(3);
      ASSERT_from_compressedOutStorage(0, Fw::Buffer(0, 0,
                                                     (U64) m_metadataBuf,
                                                     m_metadataBufSize));
      // 2 buffers for counting purposes
      ASSERT_from_compressedOutStorage(1, Fw::Buffer(0, 1,
                                                     (U64) m_thumbBuf, 0));
      ASSERT_from_compressedOutStorage(2, Fw::Buffer(0, 2,
                                                     (U64) m_imageBuf, 0));

      ASSERT_EVENTS_IMGCOMP_NoBuffer_SIZE(1);
      ASSERT_EVENTS_IMGCOMP_NoBuffer(0, sizeof(thumbBuf));

      ASSERT_EVENTS_IMGCOMP_BufferOffset_SIZE(0);

      clearHistory();

      m_bufMgrID = 1;
      m_bufID = 0;

      component.getBuffAndPushXmit(0,
                                   sizeof(metaBuf), metaBuf,
                                   sizeof(thumbBuf), thumbBuf,
                                   sizeof(imageBuf), imageBuf);

      ASSERT_EQ(m_bufGetMode, METADATA_BUF);

      ASSERT_from_compressedGetXmit_SIZE(4);
      ASSERT_from_compressedGetXmit(0, sizeof(metaBuf));
      ASSERT_from_compressedGetXmit(1, sizeof(thumbBuf));
      // 2 buffers for counting purposes
      ASSERT_from_compressedGetXmit(2, 0);
      ASSERT_from_compressedGetXmit(3, 0);

      ASSERT_from_compressedOutXmit_SIZE(3);
      ASSERT_from_compressedOutXmit(0, Fw::Buffer(1, 0,
                                                  (U64) m_metadataBuf,
                                                  m_metadataBufSize));
      // 2 buffers for counting purposes
      ASSERT_from_compressedOutXmit(1, Fw::Buffer(1, 1,
                                                  (U64) m_thumbBuf, 0));
      ASSERT_from_compressedOutXmit(2, Fw::Buffer(1, 2,
                                                  (U64) m_imageBuf, 0));

      ASSERT_EVENTS_IMGCOMP_NoBuffer_SIZE(1);
      ASSERT_EVENTS_IMGCOMP_NoBuffer(0, sizeof(thumbBuf));

      ASSERT_EVENTS_IMGCOMP_BufferOffset_SIZE(0);
  }

  void Tester ::
    bufGetNoImage(void)
  {
      char metaBuf[10];
      char thumbBuf[20];
      char imageBuf[30];

      // give BufferGet handler some memory to return
      char toMetaBuf[10];
      char toThumbBuf[20];
      char toImageBuf[30];
      m_metadataBuf = toMetaBuf;
      m_metadataBufSize = sizeof(metaBuf);
      m_thumbBuf = toThumbBuf;
      m_thumbBufSize = sizeof(thumbBuf);
      m_imageBuf = toImageBuf;
      m_imageBufSize = sizeof(imageBuf);

      m_bufGetFailMode = FAIL_GET_IMAGE_BUF;

      component.getBuffAndPushStorage(0,
                                      sizeof(metaBuf), metaBuf,
                                      sizeof(thumbBuf), thumbBuf,
                                      sizeof(imageBuf), imageBuf);

      ASSERT_EQ(m_bufGetMode, METADATA_BUF);

      ASSERT_from_compressedGetStorage_SIZE(4);
      ASSERT_from_compressedGetStorage(0, sizeof(metaBuf));
      ASSERT_from_compressedGetStorage(1, sizeof(thumbBuf));
      ASSERT_from_compressedGetStorage(2, sizeof(imageBuf));
      // 1 buffer for counting purposes
      ASSERT_from_compressedGetStorage(3, 0);

      ASSERT_from_compressedOutStorage_SIZE(3);
      ASSERT_from_compressedOutStorage(0, Fw::Buffer(0, 0,
                                                     (U64) m_metadataBuf,
                                                     m_metadataBufSize));
      ASSERT_from_compressedOutStorage(1, Fw::Buffer(0, 1,
                                                     (U64) m_thumbBuf,
                                                     m_thumbBufSize));
      // 1 buffer for counting purposes
      ASSERT_from_compressedOutStorage(2, Fw::Buffer(0, 2,
                                                     (U64) m_imageBuf, 0));

      ASSERT_EVENTS_IMGCOMP_NoBuffer_SIZE(1);
      ASSERT_EVENTS_IMGCOMP_NoBuffer(0, sizeof(imageBuf));

      ASSERT_EVENTS_IMGCOMP_BufferOffset_SIZE(0);

      clearHistory();

      m_bufMgrID = 1;
      m_bufID = 0;

      component.getBuffAndPushXmit(0,
                                   sizeof(metaBuf), metaBuf,
                                   sizeof(thumbBuf), thumbBuf,
                                   sizeof(imageBuf), imageBuf);

      ASSERT_EQ(m_bufGetMode, METADATA_BUF);

      ASSERT_from_compressedGetXmit_SIZE(4);
      ASSERT_from_compressedGetXmit(0, sizeof(metaBuf));
      ASSERT_from_compressedGetXmit(1, sizeof(thumbBuf));
      ASSERT_from_compressedGetXmit(2, sizeof(imageBuf));
      // 1 buffer for counting purposes
      ASSERT_from_compressedGetXmit(3, 0);

      ASSERT_from_compressedOutXmit_SIZE(3);
      ASSERT_from_compressedOutXmit(0, Fw::Buffer(1, 0,
                                                  (U64) m_metadataBuf,
                                                  m_metadataBufSize));
      ASSERT_from_compressedOutXmit(1, Fw::Buffer(1, 1,
                                                  (U64) m_thumbBuf,
                                                  m_thumbBufSize));
      // 1 buffer for counting purposes
      ASSERT_from_compressedOutXmit(2, Fw::Buffer(1, 2,
                                                  (U64) m_imageBuf, 0));

      ASSERT_EVENTS_IMGCOMP_NoBuffer_SIZE(1);
      ASSERT_EVENTS_IMGCOMP_NoBuffer(0, sizeof(imageBuf));

      ASSERT_EVENTS_IMGCOMP_BufferOffset_SIZE(0);
  }

  void Tester ::
    bufGetNoMetaNoCount(void)
  {
      char metaBuf[10];
      char thumbBuf[20];
      char imageBuf[30];

      // give BufferGet handler some memory to return
      char toMetaBuf[10];
      char toThumbBuf[20];
      char toImageBuf[30];
      m_metadataBuf = toMetaBuf;
      m_metadataBufSize = sizeof(metaBuf);
      m_thumbBuf = toThumbBuf;
      m_thumbBufSize = sizeof(thumbBuf);
      m_imageBuf = toImageBuf;
      m_imageBufSize = sizeof(imageBuf);

      m_bufGetFailMode = FAIL_GET_METADATA_BUF;
      m_bufGetCountFailMode = FAIL_GET_ALL;

      component.getBuffAndPushStorage(0,
                                      sizeof(metaBuf), metaBuf,
                                      sizeof(thumbBuf), thumbBuf,
                                      sizeof(imageBuf), imageBuf);

      ASSERT_EQ(m_bufGetMode, METADATA_BUF);

      ASSERT_from_compressedGetStorage_SIZE(4);
      ASSERT_from_compressedGetStorage(0, sizeof(metaBuf));
      // 3 buffers for counting purposes
      ASSERT_from_compressedGetStorage(1, 0);
      ASSERT_from_compressedGetStorage(2, 0);
      ASSERT_from_compressedGetStorage(3, 0);

      ASSERT_from_compressedOutStorage_SIZE(0);

      ASSERT_EVENTS_IMGCOMP_NoBuffer_SIZE(1);
      ASSERT_EVENTS_IMGCOMP_NoBuffer(0, sizeof(metaBuf));

      ASSERT_EVENTS_IMGCOMP_BufferOffset_SIZE(3);
      ASSERT_EVENTS_IMGCOMP_BufferOffset(0, ImgCompComponentImpl::SKIP_METADATA,
                                         ImgCompComponentImpl::OUTPUT_STORAGE,
                                         component.m_buffersHandled, 0);
      ASSERT_EVENTS_IMGCOMP_BufferOffset(1, ImgCompComponentImpl::SKIP_THUMB,
                                         ImgCompComponentImpl::OUTPUT_STORAGE,
                                         component.m_buffersHandled, 0);
      ASSERT_EVENTS_IMGCOMP_BufferOffset(2, ImgCompComponentImpl::SKIP_IMAGE,
                                         ImgCompComponentImpl::OUTPUT_STORAGE,
                                         component.m_buffersHandled, 0);

      clearHistory();

      m_bufMgrID = 1;
      m_bufID = 0;

      component.getBuffAndPushXmit(0,
                                   sizeof(metaBuf), metaBuf,
                                   sizeof(thumbBuf), thumbBuf,
                                   sizeof(imageBuf), imageBuf);

      ASSERT_EQ(m_bufGetMode, METADATA_BUF);

      ASSERT_from_compressedGetXmit_SIZE(4);
      ASSERT_from_compressedGetXmit(0, sizeof(metaBuf));
      // 3 buffers for counting purposes
      ASSERT_from_compressedGetXmit(1, 0);
      ASSERT_from_compressedGetXmit(2, 0);
      ASSERT_from_compressedGetXmit(3, 0);

      ASSERT_from_compressedOutXmit_SIZE(0);

      ASSERT_EVENTS_IMGCOMP_NoBuffer_SIZE(1);
      ASSERT_EVENTS_IMGCOMP_NoBuffer(0, sizeof(metaBuf));

      ASSERT_EVENTS_IMGCOMP_BufferOffset_SIZE(3);
      ASSERT_EVENTS_IMGCOMP_BufferOffset(0, ImgCompComponentImpl::SKIP_METADATA,
                                         ImgCompComponentImpl::OUTPUT_XMIT,
                                         component.m_buffersHandled, 0);
      ASSERT_EVENTS_IMGCOMP_BufferOffset(1, ImgCompComponentImpl::SKIP_THUMB,
                                         ImgCompComponentImpl::OUTPUT_XMIT,
                                         component.m_buffersHandled, 0);
      ASSERT_EVENTS_IMGCOMP_BufferOffset(2, ImgCompComponentImpl::SKIP_IMAGE,
                                         ImgCompComponentImpl::OUTPUT_XMIT,
                                         component.m_buffersHandled, 0);
  }

  void Tester ::
    bufGetNoThumbNoCount(void)
  {
      char metaBuf[10];
      char thumbBuf[20];
      char imageBuf[30];

      // give BufferGet handler some memory to return
      char toMetaBuf[10];
      char toThumbBuf[20];
      char toImageBuf[30];
      m_metadataBuf = toMetaBuf;
      m_metadataBufSize = sizeof(metaBuf);
      m_thumbBuf = toThumbBuf;
      m_thumbBufSize = sizeof(thumbBuf);
      m_imageBuf = toImageBuf;
      m_imageBufSize = sizeof(imageBuf);

      m_bufGetFailMode = FAIL_GET_THUMB_BUF;
      m_bufGetCountFailMode = FAIL_GET_ALL;

      component.getBuffAndPushStorage(0,
                                      sizeof(metaBuf), metaBuf,
                                      sizeof(thumbBuf), thumbBuf,
                                      sizeof(imageBuf), imageBuf);

      ASSERT_EQ(m_bufGetMode, METADATA_BUF);

      ASSERT_from_compressedGetStorage_SIZE(4);
      ASSERT_from_compressedGetStorage(0, sizeof(metaBuf));
      ASSERT_from_compressedGetStorage(1, sizeof(thumbBuf));
      // 2 buffers for counting purposes
      ASSERT_from_compressedGetStorage(2, 0);
      ASSERT_from_compressedGetStorage(3, 0);

      ASSERT_from_compressedOutStorage_SIZE(1);
      ASSERT_from_compressedOutStorage(0, Fw::Buffer(0, 0,
                                                     (U64) m_metadataBuf,
                                                     m_metadataBufSize));

      ASSERT_EVENTS_IMGCOMP_NoBuffer_SIZE(1);
      ASSERT_EVENTS_IMGCOMP_NoBuffer(0, sizeof(thumbBuf));

      ASSERT_EVENTS_IMGCOMP_BufferOffset_SIZE(2);
      ASSERT_EVENTS_IMGCOMP_BufferOffset(0, ImgCompComponentImpl::SKIP_THUMB,
                                         ImgCompComponentImpl::OUTPUT_STORAGE,
                                         component.m_buffersHandled, 0);
      ASSERT_EVENTS_IMGCOMP_BufferOffset(1, ImgCompComponentImpl::SKIP_IMAGE,
                                         ImgCompComponentImpl::OUTPUT_STORAGE,
                                         component.m_buffersHandled, 0);

      clearHistory();

      m_bufMgrID = 1;
      m_bufID = 0;

      component.getBuffAndPushXmit(0,
                                   sizeof(metaBuf), metaBuf,
                                   sizeof(thumbBuf), thumbBuf,
                                   sizeof(imageBuf), imageBuf);

      ASSERT_EQ(m_bufGetMode, METADATA_BUF);

      ASSERT_from_compressedGetXmit_SIZE(4);
      ASSERT_from_compressedGetXmit(0, sizeof(metaBuf));
      ASSERT_from_compressedGetXmit(1, sizeof(thumbBuf));
      // 2 buffers for counting purposes
      ASSERT_from_compressedGetXmit(2, 0);
      ASSERT_from_compressedGetXmit(3, 0);

      ASSERT_from_compressedOutXmit_SIZE(1);
      ASSERT_from_compressedOutXmit(0, Fw::Buffer(1, 0,
                                                  (U64) m_metadataBuf,
                                                  m_metadataBufSize));

      ASSERT_EVENTS_IMGCOMP_NoBuffer_SIZE(1);
      ASSERT_EVENTS_IMGCOMP_NoBuffer(0, sizeof(thumbBuf));

      ASSERT_EVENTS_IMGCOMP_BufferOffset_SIZE(2);
      ASSERT_EVENTS_IMGCOMP_BufferOffset(0, ImgCompComponentImpl::SKIP_THUMB,
                                         ImgCompComponentImpl::OUTPUT_XMIT,
                                         component.m_buffersHandled, 0);
      ASSERT_EVENTS_IMGCOMP_BufferOffset(1, ImgCompComponentImpl::SKIP_IMAGE,
                                         ImgCompComponentImpl::OUTPUT_XMIT,
                                         component.m_buffersHandled, 0);
  }

  void Tester ::
    bufGetNoImageNoCount(void)
  {
      char metaBuf[10];
      char thumbBuf[20];
      char imageBuf[30];

      // give BufferGet handler some memory to return
      char toMetaBuf[10];
      char toThumbBuf[20];
      char toImageBuf[30];
      m_metadataBuf = toMetaBuf;
      m_metadataBufSize = sizeof(metaBuf);
      m_thumbBuf = toThumbBuf;
      m_thumbBufSize = sizeof(thumbBuf);
      m_imageBuf = toImageBuf;
      m_imageBufSize = sizeof(imageBuf);

      m_bufGetFailMode = FAIL_GET_IMAGE_BUF;
      m_bufGetCountFailMode = FAIL_GET_ALL;

      component.getBuffAndPushStorage(0,
                                      sizeof(metaBuf), metaBuf,
                                      sizeof(thumbBuf), thumbBuf,
                                      sizeof(imageBuf), imageBuf);

      ASSERT_EQ(m_bufGetMode, METADATA_BUF);

      ASSERT_from_compressedGetStorage_SIZE(4);
      ASSERT_from_compressedGetStorage(0, sizeof(metaBuf));
      ASSERT_from_compressedGetStorage(1, sizeof(thumbBuf));
      ASSERT_from_compressedGetStorage(2, sizeof(imageBuf));
      // 1 buffer for counting purposes
      ASSERT_from_compressedGetStorage(3, 0);

      ASSERT_from_compressedOutStorage_SIZE(2);
      ASSERT_from_compressedOutStorage(0, Fw::Buffer(0, 0,
                                                     (U64) m_metadataBuf,
                                                     m_metadataBufSize));
      ASSERT_from_compressedOutStorage(1, Fw::Buffer(0, 1,
                                                     (U64) m_thumbBuf,
                                                     m_thumbBufSize));

      ASSERT_EVENTS_IMGCOMP_NoBuffer_SIZE(1);
      ASSERT_EVENTS_IMGCOMP_NoBuffer(0, sizeof(imageBuf));

      ASSERT_EVENTS_IMGCOMP_BufferOffset_SIZE(1);
      ASSERT_EVENTS_IMGCOMP_BufferOffset(0, ImgCompComponentImpl::SKIP_IMAGE,
                                         ImgCompComponentImpl::OUTPUT_STORAGE,
                                         component.m_buffersHandled, 0);

      clearHistory();

      m_bufMgrID = 1;
      m_bufID = 0;

      component.getBuffAndPushXmit(0,
                                   sizeof(metaBuf), metaBuf,
                                   sizeof(thumbBuf), thumbBuf,
                                   sizeof(imageBuf), imageBuf);

      ASSERT_EQ(m_bufGetMode, METADATA_BUF);

      ASSERT_from_compressedGetXmit_SIZE(4);
      ASSERT_from_compressedGetXmit(0, sizeof(metaBuf));
      ASSERT_from_compressedGetXmit(1, sizeof(thumbBuf));
      ASSERT_from_compressedGetXmit(2, sizeof(imageBuf));
      // 1 buffer for counting purposes
      ASSERT_from_compressedGetXmit(3, 0);

      ASSERT_from_compressedOutXmit_SIZE(2);
      ASSERT_from_compressedOutXmit(0, Fw::Buffer(1, 0,
                                                  (U64) m_metadataBuf,
                                                  m_metadataBufSize));
      ASSERT_from_compressedOutXmit(1, Fw::Buffer(1, 1,
                                                  (U64) m_thumbBuf,
                                                  m_thumbBufSize));

      ASSERT_EVENTS_IMGCOMP_NoBuffer_SIZE(1);
      ASSERT_EVENTS_IMGCOMP_NoBuffer(0, sizeof(imageBuf));

      ASSERT_EVENTS_IMGCOMP_BufferOffset_SIZE(1);
      ASSERT_EVENTS_IMGCOMP_BufferOffset(0, ImgCompComponentImpl::SKIP_IMAGE,
                                         ImgCompComponentImpl::OUTPUT_XMIT,
                                         component.m_buffersHandled, 0);
  }

  // ----------------------------------------------------------------------
  // Handlers for typed from ports
  // ----------------------------------------------------------------------

  void Tester ::
    from_compressedOutStorage_handler(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &fwBuffer
    )
  {
    this->pushFromPortEntry_compressedOutStorage(fwBuffer);
  }

  void Tester ::
    from_compressedOutXmit_handler(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &fwBuffer
    )
  {
    this->pushFromPortEntry_compressedOutXmit(fwBuffer);
  }

  Fw::Buffer Tester ::
    from_compressedGetStorage_handler(
        const NATIVE_INT_TYPE portNum,
        U32 size
    )
  {
    this->pushFromPortEntry_compressedGetStorage(size);
    return bufferGetHelper(size);
  }

  Fw::Buffer Tester ::
    from_compressedGetXmit_handler(
        const NATIVE_INT_TYPE portNum,
        U32 size
    )
  {
    this->pushFromPortEntry_compressedGetXmit(size);
    return bufferGetHelper(size);
  }

  void Tester ::
    from_uncompressedReturn_handler(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &fwBuffer
    )
  {
    this->pushFromPortEntry_uncompressedReturn(fwBuffer);
  }

  void Tester ::
    from_pingOut_handler(
        const NATIVE_INT_TYPE portNum,
        U32 key
    )
  {
    this->pushFromPortEntry_pingOut(key);
  }

  // ----------------------------------------------------------------------
  // Helper methods
  // ----------------------------------------------------------------------

  Fw::Buffer Tester ::
    bufferGetHelper(U32 size)
  {
      switch (m_bufGetMode) {
          case METADATA_BUF:
              if ((size != 0) &&
                  ((m_bufGetFailMode == FAIL_GET_METADATA_BUF) ||
                   (m_bufGetFailMode == FAIL_GET_ALL))) {
                  return Fw::Buffer(m_bufMgrID, 0, 0, size);
              }
              else if ((size == 0) &&
                       ((m_bufGetCountFailMode == FAIL_GET_METADATA_BUF) ||
                        (m_bufGetCountFailMode == FAIL_GET_ALL))) {
                  m_bufGetMode = THUMB_BUF;
                  return Fw::Buffer(m_bufMgrID, 0, 0, size);
              }
              else {
                  m_bufGetMode = THUMB_BUF;
                  return Fw::Buffer(m_bufMgrID, m_bufID++,
                                    (U64) m_metadataBuf, size);
              }
          case THUMB_BUF:
              if ((size != 0) &&
                  ((m_bufGetFailMode == FAIL_GET_THUMB_BUF) ||
                   (m_bufGetFailMode == FAIL_GET_ALL))) {
                  return Fw::Buffer(m_bufMgrID, 0, 0, size);
              }
              else if ((size == 0) &&
                       ((m_bufGetCountFailMode == FAIL_GET_THUMB_BUF) ||
                        (m_bufGetCountFailMode == FAIL_GET_ALL))) {
                  m_bufGetMode = IMAGE_BUF;
                  return Fw::Buffer(m_bufMgrID, 0, 0, size);
              }
              else {
                  m_bufGetMode = IMAGE_BUF;
                  return Fw::Buffer(m_bufMgrID, m_bufID++,
                                    (U64) m_thumbBuf, size);
              }
          case IMAGE_BUF:
              if ((size != 0) &&
                  ((m_bufGetFailMode == FAIL_GET_IMAGE_BUF) ||
                   (m_bufGetFailMode == FAIL_GET_ALL))) {
                  return Fw::Buffer(m_bufMgrID, 0, 0, size);
              }
              else if ((size == 0) &&
                       ((m_bufGetCountFailMode == FAIL_GET_IMAGE_BUF) ||
                        (m_bufGetCountFailMode == FAIL_GET_ALL))) {
                  m_bufGetMode = METADATA_BUF;
                  return Fw::Buffer(m_bufMgrID, 0, 0, size);
              }
              else {
                  m_bufGetMode = METADATA_BUF;
                  return Fw::Buffer(m_bufMgrID, m_bufID++,
                                    (U64) m_imageBuf, size);
              }
          default:
              ADD_FAILURE(); // can't use FAIL() in non-void-returning function
              return Fw::Buffer(m_bufMgrID, 0, 0, size);
      }
  }

  void Tester ::
    connectPorts(void)
  {

    // uncompressedIn
    for (NATIVE_INT_TYPE i = 0; i < 2; ++i) {
      this->connect_to_uncompressedIn(
          i,
          this->component.get_uncompressedIn_InputPort(i)
      );
    }

    // pingIn
    this->connect_to_pingIn(
        0,
        this->component.get_pingIn_InputPort(0)
    );

    // schedIn
    this->connect_to_schedIn(
        0,
        this->component.get_schedIn_InputPort(0)
    );

    // CmdDisp
    this->connect_to_CmdDisp(
        0,
        this->component.get_CmdDisp_InputPort(0)
    );

    // compressedOutStorage
    for (NATIVE_INT_TYPE i = 0; i < 2; ++i) {
      this->component.set_compressedOutStorage_OutputPort(
          i,
          this->get_from_compressedOutStorage(i)
      );
    }

    // compressedOutXmit
    for (NATIVE_INT_TYPE i = 0; i < 2; ++i) {
      this->component.set_compressedOutXmit_OutputPort(
          i,
          this->get_from_compressedOutXmit(i)
      );
    }

    // compressedGetStorage
    for (NATIVE_INT_TYPE i = 0; i < 2; ++i) {
      this->component.set_compressedGetStorage_OutputPort(
          i,
          this->get_from_compressedGetStorage(i)
      );
    }

    // compressedGetXmit
    for (NATIVE_INT_TYPE i = 0; i < 2; ++i) {
      this->component.set_compressedGetXmit_OutputPort(
          i,
          this->get_from_compressedGetXmit(i)
      );
    }

    // uncompressedReturn
    for (NATIVE_INT_TYPE i = 0; i < 2; ++i) {
      this->component.set_uncompressedReturn_OutputPort(
          i,
          this->get_from_uncompressedReturn(i)
      );
    }
    
    // timeCaller
    this->component.set_timeCaller_OutputPort(
        0, 
        this->get_from_timeCaller(0)
    );

    // pingOut
    this->component.set_pingOut_OutputPort(
        0, 
        this->get_from_pingOut(0)
    );

    // CmdStatus
    this->component.set_CmdStatus_OutputPort(
        0, 
        this->get_from_CmdStatus(0)
    );

    // CmdReg
    this->component.set_CmdReg_OutputPort(
        0, 
        this->get_from_CmdReg(0)
    );

    // Tlm
    this->component.set_Tlm_OutputPort(
        0, 
        this->get_from_Tlm(0)
    );

    // Log
    this->component.set_Log_OutputPort(
        0, 
        this->get_from_Log(0)
    );

    // LogText
    this->component.set_LogText_OutputPort(
        0, 
        this->get_from_LogText(0)
    );


  }

  void Tester ::
    initComponents(void)
  {
    this->init();
    this->component.init(
        QUEUE_DEPTH, INSTANCE
    );
  }

} // end namespace Svc
