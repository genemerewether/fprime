// ======================================================================
// \title  BufferAccumulator/test/ut/GTestBase.cpp
// \author Auto-generated
// \brief  cpp file for BufferAccumulator component Google Test harness base class
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

  BufferAccumulatorGTestBase ::
    BufferAccumulatorGTestBase(
#if FW_OBJECT_NAMES == 1
        const char *const compName,
        const U32 maxHistorySize
#else
        const U32 maxHistorySize
#endif
    ) : 
        BufferAccumulatorTesterBase (
#if FW_OBJECT_NAMES == 1
            compName,
#endif
            maxHistorySize
        )
  {

  }

  BufferAccumulatorGTestBase ::
    ~BufferAccumulatorGTestBase(void)
  {

  }

  // ----------------------------------------------------------------------
  // Commands
  // ----------------------------------------------------------------------

  void BufferAccumulatorGTestBase ::
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

  void BufferAccumulatorGTestBase ::
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

  void BufferAccumulatorGTestBase ::
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
  // Channel: BA_NumQueuedBuffers
  // ----------------------------------------------------------------------

  void BufferAccumulatorGTestBase ::
    assertTlm_BA_NumQueuedBuffers_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_BA_NumQueuedBuffers->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel BA_NumQueuedBuffers\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_BA_NumQueuedBuffers->size() << "\n";
  }

  void BufferAccumulatorGTestBase ::
    assertTlm_BA_NumQueuedBuffers(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const U32& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_BA_NumQueuedBuffers->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel BA_NumQueuedBuffers\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_BA_NumQueuedBuffers->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_BA_NumQueuedBuffers& e =
      this->tlmHistory_BA_NumQueuedBuffers->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel BA_NumQueuedBuffers\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Events
  // ----------------------------------------------------------------------

  void BufferAccumulatorGTestBase ::
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
  // Event: BA_BufferAccepted
  // ----------------------------------------------------------------------

  void BufferAccumulatorGTestBase ::
    assertEvents_BA_BufferAccepted_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventsSize_BA_BufferAccepted)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event BA_BufferAccepted\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventsSize_BA_BufferAccepted << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: BA_QueueFull
  // ----------------------------------------------------------------------

  void BufferAccumulatorGTestBase ::
    assertEvents_BA_QueueFull_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventsSize_BA_QueueFull)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event BA_QueueFull\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventsSize_BA_QueueFull << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: BA_StillDraining
  // ----------------------------------------------------------------------

  void BufferAccumulatorGTestBase ::
    assertEvents_BA_StillDraining_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_BA_StillDraining->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event BA_StillDraining\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_BA_StillDraining->size() << "\n";
  }

  void BufferAccumulatorGTestBase ::
    assertEvents_BA_StillDraining(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const U32 numDrained,
        const U32 numToDrain
    ) const
  {
    ASSERT_GT(this->eventHistory_BA_StillDraining->size(), __index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event BA_StillDraining\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_BA_StillDraining->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const EventEntry_BA_StillDraining& e =
      this->eventHistory_BA_StillDraining->at(__index);
    ASSERT_EQ(numDrained, e.numDrained)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument numDrained at index "
      << __index
      << " in history of event BA_StillDraining\n"
      << "  Expected: " << numDrained << "\n"
      << "  Actual:   " << e.numDrained << "\n";
    ASSERT_EQ(numToDrain, e.numToDrain)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument numToDrain at index "
      << __index
      << " in history of event BA_StillDraining\n"
      << "  Expected: " << numToDrain << "\n"
      << "  Actual:   " << e.numToDrain << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: BA_AlreadyDraining
  // ----------------------------------------------------------------------

  void BufferAccumulatorGTestBase ::
    assertEvents_BA_AlreadyDraining_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventsSize_BA_AlreadyDraining)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event BA_AlreadyDraining\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventsSize_BA_AlreadyDraining << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: BA_DrainStalled
  // ----------------------------------------------------------------------

  void BufferAccumulatorGTestBase ::
    assertEvents_BA_DrainStalled_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_BA_DrainStalled->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event BA_DrainStalled\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_BA_DrainStalled->size() << "\n";
  }

  void BufferAccumulatorGTestBase ::
    assertEvents_BA_DrainStalled(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const U32 numDrained,
        const U32 numToDrain
    ) const
  {
    ASSERT_GT(this->eventHistory_BA_DrainStalled->size(), __index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event BA_DrainStalled\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_BA_DrainStalled->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const EventEntry_BA_DrainStalled& e =
      this->eventHistory_BA_DrainStalled->at(__index);
    ASSERT_EQ(numDrained, e.numDrained)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument numDrained at index "
      << __index
      << " in history of event BA_DrainStalled\n"
      << "  Expected: " << numDrained << "\n"
      << "  Actual:   " << e.numDrained << "\n";
    ASSERT_EQ(numToDrain, e.numToDrain)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument numToDrain at index "
      << __index
      << " in history of event BA_DrainStalled\n"
      << "  Expected: " << numToDrain << "\n"
      << "  Actual:   " << e.numToDrain << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: BA_PartialDrainDone
  // ----------------------------------------------------------------------

  void BufferAccumulatorGTestBase ::
    assertEvents_BA_PartialDrainDone_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_BA_PartialDrainDone->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event BA_PartialDrainDone\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_BA_PartialDrainDone->size() << "\n";
  }

  void BufferAccumulatorGTestBase ::
    assertEvents_BA_PartialDrainDone(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const U32 numDrained
    ) const
  {
    ASSERT_GT(this->eventHistory_BA_PartialDrainDone->size(), __index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event BA_PartialDrainDone\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_BA_PartialDrainDone->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const EventEntry_BA_PartialDrainDone& e =
      this->eventHistory_BA_PartialDrainDone->at(__index);
    ASSERT_EQ(numDrained, e.numDrained)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument numDrained at index "
      << __index
      << " in history of event BA_PartialDrainDone\n"
      << "  Expected: " << numDrained << "\n"
      << "  Actual:   " << e.numDrained << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: BA_NonBlockDrain
  // ----------------------------------------------------------------------

  void BufferAccumulatorGTestBase ::
    assertEvents_BA_NonBlockDrain_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_BA_NonBlockDrain->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event BA_NonBlockDrain\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_BA_NonBlockDrain->size() << "\n";
  }

  void BufferAccumulatorGTestBase ::
    assertEvents_BA_NonBlockDrain(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const U32 numWillDrain,
        const U32 numReqDrain
    ) const
  {
    ASSERT_GT(this->eventHistory_BA_NonBlockDrain->size(), __index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event BA_NonBlockDrain\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_BA_NonBlockDrain->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const EventEntry_BA_NonBlockDrain& e =
      this->eventHistory_BA_NonBlockDrain->at(__index);
    ASSERT_EQ(numWillDrain, e.numWillDrain)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument numWillDrain at index "
      << __index
      << " in history of event BA_NonBlockDrain\n"
      << "  Expected: " << numWillDrain << "\n"
      << "  Actual:   " << e.numWillDrain << "\n";
    ASSERT_EQ(numReqDrain, e.numReqDrain)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument numReqDrain at index "
      << __index
      << " in history of event BA_NonBlockDrain\n"
      << "  Expected: " << numReqDrain << "\n"
      << "  Actual:   " << e.numReqDrain << "\n";
  }

  // ----------------------------------------------------------------------
  // From ports
  // ----------------------------------------------------------------------

  void BufferAccumulatorGTestBase ::
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
  // From port: bufferSendOutDrain
  // ----------------------------------------------------------------------

  void BufferAccumulatorGTestBase ::
    assert_from_bufferSendOutDrain_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->fromPortHistory_bufferSendOutDrain->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for from_bufferSendOutDrain\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->fromPortHistory_bufferSendOutDrain->size() << "\n";
  }

  // ----------------------------------------------------------------------
  // From port: bufferSendOutReturn
  // ----------------------------------------------------------------------

  void BufferAccumulatorGTestBase ::
    assert_from_bufferSendOutReturn_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->fromPortHistory_bufferSendOutReturn->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for from_bufferSendOutReturn\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->fromPortHistory_bufferSendOutReturn->size() << "\n";
  }

  // ----------------------------------------------------------------------
  // From port: pingOut
  // ----------------------------------------------------------------------

  void BufferAccumulatorGTestBase ::
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
