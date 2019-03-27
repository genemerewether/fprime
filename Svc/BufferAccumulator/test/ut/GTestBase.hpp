// ======================================================================
// \title  BufferAccumulator/test/ut/GTestBase.hpp
// \author Auto-generated
// \brief  hpp file for BufferAccumulator component Google Test harness base class
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

#ifndef BufferAccumulator_GTEST_BASE_HPP
#define BufferAccumulator_GTEST_BASE_HPP

#include "TesterBase.hpp"
#include "gtest/gtest.h"

// ----------------------------------------------------------------------
// Macros for command history assertions
// ----------------------------------------------------------------------

#define ASSERT_CMD_RESPONSE_SIZE(size) \
  this->assertCmdResponse_size(__FILE__, __LINE__, size)

#define ASSERT_CMD_RESPONSE(index, opCode, cmdSeq, response) \
  this->assertCmdResponse(__FILE__, __LINE__, index, opCode, cmdSeq, response)

// ----------------------------------------------------------------------
// Macros for telemetry history assertions
// ----------------------------------------------------------------------

#define ASSERT_TLM_SIZE(size) \
  this->assertTlm_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_BA_NumQueuedBuffers_SIZE(size) \
  this->assertTlm_BA_NumQueuedBuffers_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_BA_NumQueuedBuffers(index, value) \
  this->assertTlm_BA_NumQueuedBuffers(__FILE__, __LINE__, index, value)

// ----------------------------------------------------------------------
// Macros for event history assertions 
// ----------------------------------------------------------------------

#define ASSERT_EVENTS_SIZE(size) \
  this->assertEvents_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_BA_BufferAccepted_SIZE(size) \
  this->assertEvents_BA_BufferAccepted_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_BA_QueueFull_SIZE(size) \
  this->assertEvents_BA_QueueFull_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_BA_StillDraining_SIZE(size) \
  this->assertEvents_BA_StillDraining_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_BA_StillDraining(index, _numDrained, _numToDrain) \
  this->assertEvents_BA_StillDraining(__FILE__, __LINE__, index, _numDrained, _numToDrain)

#define ASSERT_EVENTS_BA_AlreadyDraining_SIZE(size) \
  this->assertEvents_BA_AlreadyDraining_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_BA_DrainStalled_SIZE(size) \
  this->assertEvents_BA_DrainStalled_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_BA_DrainStalled(index, _numDrained, _numToDrain) \
  this->assertEvents_BA_DrainStalled(__FILE__, __LINE__, index, _numDrained, _numToDrain)

#define ASSERT_EVENTS_BA_PartialDrainDone_SIZE(size) \
  this->assertEvents_BA_PartialDrainDone_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_BA_PartialDrainDone(index, _numDrained) \
  this->assertEvents_BA_PartialDrainDone(__FILE__, __LINE__, index, _numDrained)

#define ASSERT_EVENTS_BA_NonBlockDrain_SIZE(size) \
  this->assertEvents_BA_NonBlockDrain_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_BA_NonBlockDrain(index, _numWillDrain, _numReqDrain) \
  this->assertEvents_BA_NonBlockDrain(__FILE__, __LINE__, index, _numWillDrain, _numReqDrain)

// ----------------------------------------------------------------------
// Macros for typed user from port history assertions
// ----------------------------------------------------------------------

#define ASSERT_FROM_PORT_HISTORY_SIZE(size) \
  this->assertFromPortHistory_size(__FILE__, __LINE__, size)

#define ASSERT_from_bufferSendOutDrain_SIZE(size) \
  this->assert_from_bufferSendOutDrain_size(__FILE__, __LINE__, size)

#define ASSERT_from_bufferSendOutDrain(index, _fwBuffer) \
  { \
    ASSERT_GT(this->fromPortHistory_bufferSendOutDrain->size(), static_cast<U32>(index)) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Index into history of from_bufferSendOutDrain\n" \
    << "  Expected: Less than size of history (" \
    << this->fromPortHistory_bufferSendOutDrain->size() << ")\n" \
    << "  Actual:   " << index << "\n"; \
    const FromPortEntry_bufferSendOutDrain& _e = \
      this->fromPortHistory_bufferSendOutDrain->at(index); \
    ASSERT_EQ(_fwBuffer, _e.fwBuffer) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument fwBuffer at index " \
    << index \
    << " in history of from_bufferSendOutDrain\n" \
    << "  Expected: " << _fwBuffer << "\n" \
    << "  Actual:   " << _e.fwBuffer << "\n"; \
  }

#define ASSERT_from_bufferSendOutReturn_SIZE(size) \
  this->assert_from_bufferSendOutReturn_size(__FILE__, __LINE__, size)

#define ASSERT_from_bufferSendOutReturn(index, _fwBuffer) \
  { \
    ASSERT_GT(this->fromPortHistory_bufferSendOutReturn->size(), static_cast<U32>(index)) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Index into history of from_bufferSendOutReturn\n" \
    << "  Expected: Less than size of history (" \
    << this->fromPortHistory_bufferSendOutReturn->size() << ")\n" \
    << "  Actual:   " << index << "\n"; \
    const FromPortEntry_bufferSendOutReturn& _e = \
      this->fromPortHistory_bufferSendOutReturn->at(index); \
    ASSERT_EQ(_fwBuffer, _e.fwBuffer) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument fwBuffer at index " \
    << index \
    << " in history of from_bufferSendOutReturn\n" \
    << "  Expected: " << _fwBuffer << "\n" \
    << "  Actual:   " << _e.fwBuffer << "\n"; \
  }

#define ASSERT_from_pingOut_SIZE(size) \
  this->assert_from_pingOut_size(__FILE__, __LINE__, size)

#define ASSERT_from_pingOut(index, _key) \
  { \
    ASSERT_GT(this->fromPortHistory_pingOut->size(), static_cast<U32>(index)) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Index into history of from_pingOut\n" \
    << "  Expected: Less than size of history (" \
    << this->fromPortHistory_pingOut->size() << ")\n" \
    << "  Actual:   " << index << "\n"; \
    const FromPortEntry_pingOut& _e = \
      this->fromPortHistory_pingOut->at(index); \
    ASSERT_EQ(_key, _e.key) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument key at index " \
    << index \
    << " in history of from_pingOut\n" \
    << "  Expected: " << _key << "\n" \
    << "  Actual:   " << _e.key << "\n"; \
  }

namespace Svc {

  //! \class BufferAccumulatorGTestBase
  //! \brief Auto-generated base class for BufferAccumulator component Google Test harness
  //!
  class BufferAccumulatorGTestBase :
    public BufferAccumulatorTesterBase
  {

    protected:

      // ----------------------------------------------------------------------
      // Construction and destruction
      // ----------------------------------------------------------------------

      //! Construct object BufferAccumulatorGTestBase
      //!
      BufferAccumulatorGTestBase(
#if FW_OBJECT_NAMES == 1
          const char *const compName, /*!< The component name*/
          const U32 maxHistorySize /*!< The maximum size of each history*/
#else
          const U32 maxHistorySize /*!< The maximum size of each history*/
#endif
      );

      //! Destroy object BufferAccumulatorGTestBase
      //!
      virtual ~BufferAccumulatorGTestBase(void);

    protected:

      // ----------------------------------------------------------------------
      // Commands
      // ----------------------------------------------------------------------

      //! Assert size of command response history
      //!
      void assertCmdResponse_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      //! Assert command response in history at index
      //!
      void assertCmdResponse(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq, /*!< The command sequence number*/
          const Fw::CommandResponse response /*!< The command response*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Telemetry
      // ----------------------------------------------------------------------

      //! Assert size of telemetry history
      //!
      void assertTlm_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: BA_NumQueuedBuffers
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_BA_NumQueuedBuffers_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_BA_NumQueuedBuffers(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const U32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Events
      // ----------------------------------------------------------------------

      void assertEvents_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: BA_BufferAccepted
      // ----------------------------------------------------------------------

      void assertEvents_BA_BufferAccepted_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: BA_QueueFull
      // ----------------------------------------------------------------------

      void assertEvents_BA_QueueFull_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: BA_StillDraining
      // ----------------------------------------------------------------------

      void assertEvents_BA_StillDraining_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_BA_StillDraining(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const U32 numDrained, 
          const U32 numToDrain 
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: BA_AlreadyDraining
      // ----------------------------------------------------------------------

      void assertEvents_BA_AlreadyDraining_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: BA_DrainStalled
      // ----------------------------------------------------------------------

      void assertEvents_BA_DrainStalled_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_BA_DrainStalled(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const U32 numDrained, 
          const U32 numToDrain 
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: BA_PartialDrainDone
      // ----------------------------------------------------------------------

      void assertEvents_BA_PartialDrainDone_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_BA_PartialDrainDone(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const U32 numDrained 
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: BA_NonBlockDrain
      // ----------------------------------------------------------------------

      void assertEvents_BA_NonBlockDrain_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_BA_NonBlockDrain(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const U32 numWillDrain, 
          const U32 numReqDrain 
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // From ports 
      // ----------------------------------------------------------------------

      void assertFromPortHistory_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // From port: bufferSendOutDrain 
      // ----------------------------------------------------------------------

      void assert_from_bufferSendOutDrain_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // From port: bufferSendOutReturn 
      // ----------------------------------------------------------------------

      void assert_from_bufferSendOutReturn_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // From port: pingOut 
      // ----------------------------------------------------------------------

      void assert_from_pingOut_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

  };

} // end namespace Svc

#endif
