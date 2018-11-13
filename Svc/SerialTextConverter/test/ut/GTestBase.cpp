// ======================================================================
// \title  SerialTextConverter/test/ut/GTestBase.cpp
// \author Auto-generated
// \brief  cpp file for SerialTextConverter component Google Test harness base class
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

  SerialTextConverterGTestBase ::
    SerialTextConverterGTestBase(
#if FW_OBJECT_NAMES == 1
        const char *const compName,
        const U32 maxHistorySize
#else
        const U32 maxHistorySize
#endif
    ) : 
        SerialTextConverterTesterBase (
#if FW_OBJECT_NAMES == 1
            compName,
#endif
            maxHistorySize
        )
  {

  }

  SerialTextConverterGTestBase ::
    ~SerialTextConverterGTestBase(void)
  {

  }

  // ----------------------------------------------------------------------
  // Commands
  // ----------------------------------------------------------------------

  void SerialTextConverterGTestBase ::
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

  void SerialTextConverterGTestBase ::
    assertCmdResponse(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        const FwOpcodeType opCode,
        const U32 cmdSeq,
        const Fw::CommandResponse response
    )
    const
  {
    ASSERT_LT(index, this->cmdResponseHistory->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into command response history\n"
      << "  Expected: Less than size of command response history (" 
      << this->cmdResponseHistory->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const CmdResponse& e = this->cmdResponseHistory->at(index);
    ASSERT_EQ(opCode, e.opCode)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Opcode at index "
      << index
      << " in command response history\n"
      << "  Expected: " << opCode << "\n"
      << "  Actual:   " << e.opCode << "\n";
    ASSERT_EQ(cmdSeq, e.cmdSeq)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Command sequence number at index "
      << index
      << " in command response history\n"
      << "  Expected: " << cmdSeq << "\n"
      << "  Actual:   " << e.cmdSeq << "\n";
    ASSERT_EQ(response, e.response)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Command response at index "
      << index
      << " in command resopnse history\n"
      << "  Expected: " << response << "\n"
      << "  Actual:   " << e.response << "\n";
  }

  // ----------------------------------------------------------------------
  // Events
  // ----------------------------------------------------------------------

  void SerialTextConverterGTestBase ::
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
  // Event: STC_SerialText
  // ----------------------------------------------------------------------

  void SerialTextConverterGTestBase ::
    assertEvents_STC_SerialText_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_STC_SerialText->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event STC_SerialText\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_STC_SerialText->size() << "\n";
  }

  void SerialTextConverterGTestBase ::
    assertEvents_STC_SerialText(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        const char *const data
    ) const
  {
    ASSERT_GT(this->eventHistory_STC_SerialText->size(), index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event STC_SerialText\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_STC_SerialText->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const EventEntry_STC_SerialText& e =
      this->eventHistory_STC_SerialText->at(index);
    ASSERT_STREQ(data, e.data.toChar())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument data at index "
      << index
      << " in history of event STC_SerialText\n"
      << "  Expected: " << data << "\n"
      << "  Actual:   " << e.data.toChar() << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: STC_SerialReadBadStatus
  // ----------------------------------------------------------------------

  void SerialTextConverterGTestBase ::
    assertEvents_STC_SerialReadBadStatus_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_STC_SerialReadBadStatus->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event STC_SerialReadBadStatus\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_STC_SerialReadBadStatus->size() << "\n";
  }

  void SerialTextConverterGTestBase ::
    assertEvents_STC_SerialReadBadStatus(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        const I32 status
    ) const
  {
    ASSERT_GT(this->eventHistory_STC_SerialReadBadStatus->size(), index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event STC_SerialReadBadStatus\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_STC_SerialReadBadStatus->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const EventEntry_STC_SerialReadBadStatus& e =
      this->eventHistory_STC_SerialReadBadStatus->at(index);
    ASSERT_EQ(status, e.status)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument status at index "
      << index
      << " in history of event STC_SerialReadBadStatus\n"
      << "  Expected: " << status << "\n"
      << "  Actual:   " << e.status << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: STC_SetMode_Cmd_Sent
  // ----------------------------------------------------------------------

  void SerialTextConverterGTestBase ::
    assertEvents_STC_SetMode_Cmd_Sent_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_STC_SetMode_Cmd_Sent->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event STC_SetMode_Cmd_Sent\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_STC_SetMode_Cmd_Sent->size() << "\n";
  }

  void SerialTextConverterGTestBase ::
    assertEvents_STC_SetMode_Cmd_Sent(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        SerialTextConverterComponentBase::StcModeEv mode
    ) const
  {
    ASSERT_GT(this->eventHistory_STC_SetMode_Cmd_Sent->size(), index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event STC_SetMode_Cmd_Sent\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_STC_SetMode_Cmd_Sent->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const EventEntry_STC_SetMode_Cmd_Sent& e =
      this->eventHistory_STC_SetMode_Cmd_Sent->at(index);
    ASSERT_EQ(mode, e.mode)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument mode at index "
      << index
      << " in history of event STC_SetMode_Cmd_Sent\n"
      << "  Expected: " << mode << "\n"
      << "  Actual:   " << e.mode << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: STC_SetMode_Cmd_Invalid
  // ----------------------------------------------------------------------

  void SerialTextConverterGTestBase ::
    assertEvents_STC_SetMode_Cmd_Invalid_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventsSize_STC_SetMode_Cmd_Invalid)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event STC_SetMode_Cmd_Invalid\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventsSize_STC_SetMode_Cmd_Invalid << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: STC_File
  // ----------------------------------------------------------------------

  void SerialTextConverterGTestBase ::
    assertEvents_STC_File_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_STC_File->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event STC_File\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_STC_File->size() << "\n";
  }

  void SerialTextConverterGTestBase ::
    assertEvents_STC_File(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        const char *const log_file
    ) const
  {
    ASSERT_GT(this->eventHistory_STC_File->size(), index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event STC_File\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_STC_File->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const EventEntry_STC_File& e =
      this->eventHistory_STC_File->at(index);
    ASSERT_STREQ(log_file, e.log_file.toChar())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument log_file at index "
      << index
      << " in history of event STC_File\n"
      << "  Expected: " << log_file << "\n"
      << "  Actual:   " << e.log_file.toChar() << "\n";
  }

  // ----------------------------------------------------------------------
  // From ports
  // ----------------------------------------------------------------------

  void SerialTextConverterGTestBase ::
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
  // From port: SerialBufferSend
  // ----------------------------------------------------------------------

  void SerialTextConverterGTestBase ::
    assert_from_SerialBufferSend_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->fromPortHistory_SerialBufferSend->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for from_SerialBufferSend\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->fromPortHistory_SerialBufferSend->size() << "\n";
  }

} // end namespace Svc
