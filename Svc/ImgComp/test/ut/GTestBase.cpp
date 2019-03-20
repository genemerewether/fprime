// ======================================================================
// \title  ImgComp/test/ut/GTestBase.cpp
// \author Auto-generated
// \brief  cpp file for ImgComp component Google Test harness base class
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

#include "GTestBase.hpp"

namespace Svc {

  // ----------------------------------------------------------------------
  // Construction and destruction
  // ----------------------------------------------------------------------

  ImgCompGTestBase ::
    ImgCompGTestBase(
#if FW_OBJECT_NAMES == 1
        const char *const compName,
        const U32 maxHistorySize
#else
        const U32 maxHistorySize
#endif
    ) : 
        ImgCompTesterBase (
#if FW_OBJECT_NAMES == 1
            compName,
#endif
            maxHistorySize
        )
  {

  }

  ImgCompGTestBase ::
    ~ImgCompGTestBase(void)
  {

  }

  // ----------------------------------------------------------------------
  // Commands
  // ----------------------------------------------------------------------

  void ImgCompGTestBase ::
    assertCmdResponse_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ((unsigned long) size, this->cmdResponseHistory->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of command response history\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->cmdResponseHistory->size() << "\n";
  }

  void ImgCompGTestBase ::
    assertCmdResponse(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const FwOpcodeType opCode,
        const U32 cmdSeq,
        const Fw::CommandResponse response
    )
    const
  {
    ASSERT_LT(__index, this->cmdResponseHistory->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into command response history\n"
      << "  Expected: Less than size of command response history (" 
      << this->cmdResponseHistory->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const CmdResponse& e = this->cmdResponseHistory->at(__index);
    ASSERT_EQ(opCode, e.opCode)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Opcode at index "
      << __index
      << " in command response history\n"
      << "  Expected: " << opCode << "\n"
      << "  Actual:   " << e.opCode << "\n";
    ASSERT_EQ(cmdSeq, e.cmdSeq)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Command sequence number at index "
      << __index
      << " in command response history\n"
      << "  Expected: " << cmdSeq << "\n"
      << "  Actual:   " << e.cmdSeq << "\n";
    ASSERT_EQ(response, e.response)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Command response at index "
      << __index
      << " in command resopnse history\n"
      << "  Expected: " << response << "\n"
      << "  Actual:   " << e.response << "\n";
  }

  // ----------------------------------------------------------------------
  // Telemetry
  // ----------------------------------------------------------------------

  void ImgCompGTestBase ::
    assertTlm_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->tlmSize)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Total size of all telemetry histories\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmSize << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: IMGCOMP_BuffersHandled
  // ----------------------------------------------------------------------

  void ImgCompGTestBase ::
    assertTlm_IMGCOMP_BuffersHandled_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_IMGCOMP_BuffersHandled->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel IMGCOMP_BuffersHandled\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_IMGCOMP_BuffersHandled->size() << "\n";
  }

  void ImgCompGTestBase ::
    assertTlm_IMGCOMP_BuffersHandled(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const U32& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_IMGCOMP_BuffersHandled->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel IMGCOMP_BuffersHandled\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_IMGCOMP_BuffersHandled->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_IMGCOMP_BuffersHandled& e =
      this->tlmHistory_IMGCOMP_BuffersHandled->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel IMGCOMP_BuffersHandled\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Events
  // ----------------------------------------------------------------------

  void ImgCompGTestBase ::
    assertEvents_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventsSize)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Total size of all event histories\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventsSize << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: IMGCOMP_SoftCompError
  // ----------------------------------------------------------------------

  void ImgCompGTestBase ::
    assertEvents_IMGCOMP_SoftCompError_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_IMGCOMP_SoftCompError->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event IMGCOMP_SoftCompError\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_IMGCOMP_SoftCompError->size() << "\n";
  }

  void ImgCompGTestBase ::
    assertEvents_IMGCOMP_SoftCompError(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        ImgCompComponentBase::SoftCompErrorType error,
        const char *const msg
    ) const
  {
    ASSERT_GT(this->eventHistory_IMGCOMP_SoftCompError->size(), __index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event IMGCOMP_SoftCompError\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_IMGCOMP_SoftCompError->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const EventEntry_IMGCOMP_SoftCompError& e =
      this->eventHistory_IMGCOMP_SoftCompError->at(__index);
    ASSERT_EQ(error, e.error)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument error at index "
      << __index
      << " in history of event IMGCOMP_SoftCompError\n"
      << "  Expected: " << error << "\n"
      << "  Actual:   " << e.error << "\n";
    ASSERT_STREQ(msg, e.msg.toChar())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument msg at index "
      << __index
      << " in history of event IMGCOMP_SoftCompError\n"
      << "  Expected: " << msg << "\n"
      << "  Actual:   " << e.msg.toChar() << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: IMGCOMP_BadBuffer
  // ----------------------------------------------------------------------

  void ImgCompGTestBase ::
    assertEvents_IMGCOMP_BadBuffer_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventsSize_IMGCOMP_BadBuffer)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event IMGCOMP_BadBuffer\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventsSize_IMGCOMP_BadBuffer << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: IMGCOMP_BadSetting
  // ----------------------------------------------------------------------

  void ImgCompGTestBase ::
    assertEvents_IMGCOMP_BadSetting_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_IMGCOMP_BadSetting->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event IMGCOMP_BadSetting\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_IMGCOMP_BadSetting->size() << "\n";
  }

  void ImgCompGTestBase ::
    assertEvents_IMGCOMP_BadSetting(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const U32 portNum
    ) const
  {
    ASSERT_GT(this->eventHistory_IMGCOMP_BadSetting->size(), __index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event IMGCOMP_BadSetting\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_IMGCOMP_BadSetting->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const EventEntry_IMGCOMP_BadSetting& e =
      this->eventHistory_IMGCOMP_BadSetting->at(__index);
    ASSERT_EQ(portNum, e.portNum)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument portNum at index "
      << __index
      << " in history of event IMGCOMP_BadSetting\n"
      << "  Expected: " << portNum << "\n"
      << "  Actual:   " << e.portNum << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: IMGCOMP_NoBuffer
  // ----------------------------------------------------------------------

  void ImgCompGTestBase ::
    assertEvents_IMGCOMP_NoBuffer_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_IMGCOMP_NoBuffer->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event IMGCOMP_NoBuffer\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_IMGCOMP_NoBuffer->size() << "\n";
  }

  void ImgCompGTestBase ::
    assertEvents_IMGCOMP_NoBuffer(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const U32 size
    ) const
  {
    ASSERT_GT(this->eventHistory_IMGCOMP_NoBuffer->size(), __index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event IMGCOMP_NoBuffer\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_IMGCOMP_NoBuffer->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const EventEntry_IMGCOMP_NoBuffer& e =
      this->eventHistory_IMGCOMP_NoBuffer->at(__index);
    ASSERT_EQ(size, e.size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument size at index "
      << __index
      << " in history of event IMGCOMP_NoBuffer\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << e.size << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: IMGCOMP_BufferOffset
  // ----------------------------------------------------------------------

  void ImgCompGTestBase ::
    assertEvents_IMGCOMP_BufferOffset_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_IMGCOMP_BufferOffset->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event IMGCOMP_BufferOffset\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_IMGCOMP_BufferOffset->size() << "\n";
  }

  void ImgCompGTestBase ::
    assertEvents_IMGCOMP_BufferOffset(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        ImgCompComponentBase::BufferOffsetSkipType type,
        ImgCompComponentBase::BufferOffsetSkipOutput output,
        const U32 inBuffer,
        const U32 portNum
    ) const
  {
    ASSERT_GT(this->eventHistory_IMGCOMP_BufferOffset->size(), __index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event IMGCOMP_BufferOffset\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_IMGCOMP_BufferOffset->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const EventEntry_IMGCOMP_BufferOffset& e =
      this->eventHistory_IMGCOMP_BufferOffset->at(__index);
    ASSERT_EQ(type, e.type)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument type at index "
      << __index
      << " in history of event IMGCOMP_BufferOffset\n"
      << "  Expected: " << type << "\n"
      << "  Actual:   " << e.type << "\n";
    ASSERT_EQ(output, e.output)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument output at index "
      << __index
      << " in history of event IMGCOMP_BufferOffset\n"
      << "  Expected: " << output << "\n"
      << "  Actual:   " << e.output << "\n";
    ASSERT_EQ(inBuffer, e.inBuffer)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument inBuffer at index "
      << __index
      << " in history of event IMGCOMP_BufferOffset\n"
      << "  Expected: " << inBuffer << "\n"
      << "  Actual:   " << e.inBuffer << "\n";
    ASSERT_EQ(portNum, e.portNum)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument portNum at index "
      << __index
      << " in history of event IMGCOMP_BufferOffset\n"
      << "  Expected: " << portNum << "\n"
      << "  Actual:   " << e.portNum << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: IMGCOMP_NoRestartMarkers
  // ----------------------------------------------------------------------

  void ImgCompGTestBase ::
    assertEvents_IMGCOMP_NoRestartMarkers_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_IMGCOMP_NoRestartMarkers->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event IMGCOMP_NoRestartMarkers\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_IMGCOMP_NoRestartMarkers->size() << "\n";
  }

  void ImgCompGTestBase ::
    assertEvents_IMGCOMP_NoRestartMarkers(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const I32 error
    ) const
  {
    ASSERT_GT(this->eventHistory_IMGCOMP_NoRestartMarkers->size(), __index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event IMGCOMP_NoRestartMarkers\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_IMGCOMP_NoRestartMarkers->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const EventEntry_IMGCOMP_NoRestartMarkers& e =
      this->eventHistory_IMGCOMP_NoRestartMarkers->at(__index);
    ASSERT_EQ(error, e.error)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument error at index "
      << __index
      << " in history of event IMGCOMP_NoRestartMarkers\n"
      << "  Expected: " << error << "\n"
      << "  Actual:   " << e.error << "\n";
  }

  // ----------------------------------------------------------------------
  // From ports
  // ----------------------------------------------------------------------

  void ImgCompGTestBase ::
    assertFromPortHistory_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->fromPortHistorySize)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Total size of all from port histories\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->fromPortHistorySize << "\n";
  }

  // ----------------------------------------------------------------------
  // From port: compressedOutStorage
  // ----------------------------------------------------------------------

  void ImgCompGTestBase ::
    assert_from_compressedOutStorage_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->fromPortHistory_compressedOutStorage->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for from_compressedOutStorage\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->fromPortHistory_compressedOutStorage->size() << "\n";
  }

  // ----------------------------------------------------------------------
  // From port: compressedOutXmit
  // ----------------------------------------------------------------------

  void ImgCompGTestBase ::
    assert_from_compressedOutXmit_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->fromPortHistory_compressedOutXmit->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for from_compressedOutXmit\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->fromPortHistory_compressedOutXmit->size() << "\n";
  }

  // ----------------------------------------------------------------------
  // From port: compressedGetStorage
  // ----------------------------------------------------------------------

  void ImgCompGTestBase ::
    assert_from_compressedGetStorage_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->fromPortHistory_compressedGetStorage->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for from_compressedGetStorage\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->fromPortHistory_compressedGetStorage->size() << "\n";
  }

  // ----------------------------------------------------------------------
  // From port: compressedGetXmit
  // ----------------------------------------------------------------------

  void ImgCompGTestBase ::
    assert_from_compressedGetXmit_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->fromPortHistory_compressedGetXmit->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for from_compressedGetXmit\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->fromPortHistory_compressedGetXmit->size() << "\n";
  }

  // ----------------------------------------------------------------------
  // From port: uncompressedReturn
  // ----------------------------------------------------------------------

  void ImgCompGTestBase ::
    assert_from_uncompressedReturn_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->fromPortHistory_uncompressedReturn->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for from_uncompressedReturn\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->fromPortHistory_uncompressedReturn->size() << "\n";
  }

  // ----------------------------------------------------------------------
  // From port: pingOut
  // ----------------------------------------------------------------------

  void ImgCompGTestBase ::
    assert_from_pingOut_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->fromPortHistory_pingOut->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for from_pingOut\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->fromPortHistory_pingOut->size() << "\n";
  }

} // end namespace Svc
