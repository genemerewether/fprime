// ======================================================================
// \title  ActiveL2PrmDb/test/ut/GTestBase.cpp
// \author Auto-generated
// \brief  cpp file for ActiveL2PrmDb component Google Test harness base class
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

  ActiveL2PrmDbGTestBase ::
    ActiveL2PrmDbGTestBase(
#if FW_OBJECT_NAMES == 1
        const char *const compName,
        const U32 maxHistorySize
#else
        const U32 maxHistorySize
#endif
    ) : 
        ActiveL2PrmDbTesterBase (
#if FW_OBJECT_NAMES == 1
            compName,
#endif
            maxHistorySize
        )
  {

  }

  ActiveL2PrmDbGTestBase ::
    ~ActiveL2PrmDbGTestBase(void)
  {

  }

  // ----------------------------------------------------------------------
  // Commands
  // ----------------------------------------------------------------------

  void ActiveL2PrmDbGTestBase ::
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

  void ActiveL2PrmDbGTestBase ::
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
  // Events
  // ----------------------------------------------------------------------

  void ActiveL2PrmDbGTestBase ::
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
  // Event: PrmIdNotFound
  // ----------------------------------------------------------------------

  void ActiveL2PrmDbGTestBase ::
    assertEvents_PrmIdNotFound_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_PrmIdNotFound->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event PrmIdNotFound\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_PrmIdNotFound->size() << "\n";
  }

  void ActiveL2PrmDbGTestBase ::
    assertEvents_PrmIdNotFound(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const U32 Id
    ) const
  {
    ASSERT_GT(this->eventHistory_PrmIdNotFound->size(), __index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event PrmIdNotFound\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_PrmIdNotFound->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const EventEntry_PrmIdNotFound& e =
      this->eventHistory_PrmIdNotFound->at(__index);
    ASSERT_EQ(Id, e.Id)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument Id at index "
      << __index
      << " in history of event PrmIdNotFound\n"
      << "  Expected: " << Id << "\n"
      << "  Actual:   " << e.Id << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: PrmIdUpdated
  // ----------------------------------------------------------------------

  void ActiveL2PrmDbGTestBase ::
    assertEvents_PrmIdUpdated_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_PrmIdUpdated->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event PrmIdUpdated\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_PrmIdUpdated->size() << "\n";
  }

  void ActiveL2PrmDbGTestBase ::
    assertEvents_PrmIdUpdated(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const U32 Id
    ) const
  {
    ASSERT_GT(this->eventHistory_PrmIdUpdated->size(), __index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event PrmIdUpdated\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_PrmIdUpdated->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const EventEntry_PrmIdUpdated& e =
      this->eventHistory_PrmIdUpdated->at(__index);
    ASSERT_EQ(Id, e.Id)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument Id at index "
      << __index
      << " in history of event PrmIdUpdated\n"
      << "  Expected: " << Id << "\n"
      << "  Actual:   " << e.Id << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: PrmDbFull
  // ----------------------------------------------------------------------

  void ActiveL2PrmDbGTestBase ::
    assertEvents_PrmDbFull_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_PrmDbFull->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event PrmDbFull\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_PrmDbFull->size() << "\n";
  }

  void ActiveL2PrmDbGTestBase ::
    assertEvents_PrmDbFull(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const U32 Id
    ) const
  {
    ASSERT_GT(this->eventHistory_PrmDbFull->size(), __index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event PrmDbFull\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_PrmDbFull->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const EventEntry_PrmDbFull& e =
      this->eventHistory_PrmDbFull->at(__index);
    ASSERT_EQ(Id, e.Id)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument Id at index "
      << __index
      << " in history of event PrmDbFull\n"
      << "  Expected: " << Id << "\n"
      << "  Actual:   " << e.Id << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: PrmIdAdded
  // ----------------------------------------------------------------------

  void ActiveL2PrmDbGTestBase ::
    assertEvents_PrmIdAdded_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_PrmIdAdded->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event PrmIdAdded\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_PrmIdAdded->size() << "\n";
  }

  void ActiveL2PrmDbGTestBase ::
    assertEvents_PrmIdAdded(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const U32 Id
    ) const
  {
    ASSERT_GT(this->eventHistory_PrmIdAdded->size(), __index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event PrmIdAdded\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_PrmIdAdded->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const EventEntry_PrmIdAdded& e =
      this->eventHistory_PrmIdAdded->at(__index);
    ASSERT_EQ(Id, e.Id)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument Id at index "
      << __index
      << " in history of event PrmIdAdded\n"
      << "  Expected: " << Id << "\n"
      << "  Actual:   " << e.Id << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: PrmSendTooLarge
  // ----------------------------------------------------------------------

  void ActiveL2PrmDbGTestBase ::
    assertEvents_PrmSendTooLarge_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_PrmSendTooLarge->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event PrmSendTooLarge\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_PrmSendTooLarge->size() << "\n";
  }

  void ActiveL2PrmDbGTestBase ::
    assertEvents_PrmSendTooLarge(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const U32 prmId,
        const U32 prmSize
    ) const
  {
    ASSERT_GT(this->eventHistory_PrmSendTooLarge->size(), __index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event PrmSendTooLarge\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_PrmSendTooLarge->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const EventEntry_PrmSendTooLarge& e =
      this->eventHistory_PrmSendTooLarge->at(__index);
    ASSERT_EQ(prmId, e.prmId)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument prmId at index "
      << __index
      << " in history of event PrmSendTooLarge\n"
      << "  Expected: " << prmId << "\n"
      << "  Actual:   " << e.prmId << "\n";
    ASSERT_EQ(prmSize, e.prmSize)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument prmSize at index "
      << __index
      << " in history of event PrmSendTooLarge\n"
      << "  Expected: " << prmSize << "\n"
      << "  Actual:   " << e.prmSize << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: PrmSendBufferFull
  // ----------------------------------------------------------------------

  void ActiveL2PrmDbGTestBase ::
    assertEvents_PrmSendBufferFull_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventsSize_PrmSendBufferFull)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event PrmSendBufferFull\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventsSize_PrmSendBufferFull << "\n";
  }

  // ----------------------------------------------------------------------
  // From ports
  // ----------------------------------------------------------------------

  void ActiveL2PrmDbGTestBase ::
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
  // From port: sendPrm
  // ----------------------------------------------------------------------

  void ActiveL2PrmDbGTestBase ::
    assert_from_sendPrm_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->fromPortHistory_sendPrm->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for from_sendPrm\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->fromPortHistory_sendPrm->size() << "\n";
  }

  // ----------------------------------------------------------------------
  // From port: recvPrmReady
  // ----------------------------------------------------------------------

  void ActiveL2PrmDbGTestBase ::
    assert_from_recvPrmReady_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->fromPortHistory_recvPrmReady->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for from_recvPrmReady\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->fromPortHistory_recvPrmReady->size() << "\n";
  }

  // ----------------------------------------------------------------------
  // From port: pingOut
  // ----------------------------------------------------------------------

  void ActiveL2PrmDbGTestBase ::
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
