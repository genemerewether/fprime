// ======================================================================
// \title  LLRouter/test/ut/GTestBase.hpp
// \author Auto-generated
// \brief  hpp file for LLRouter component Google Test harness base class
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

#ifndef LLRouter_GTEST_BASE_HPP
#define LLRouter_GTEST_BASE_HPP

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

#define ASSERT_TLM_LLR_CrcErrors_SIZE(size) \
  this->assertTlm_LLR_CrcErrors_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_LLR_CrcErrors(index, value) \
  this->assertTlm_LLR_CrcErrors(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_LLR_ResyncErrors_SIZE(size) \
  this->assertTlm_LLR_ResyncErrors_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_LLR_ResyncErrors(index, value) \
  this->assertTlm_LLR_ResyncErrors(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_LLR_NumPackets_SIZE(size) \
  this->assertTlm_LLR_NumPackets_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_LLR_NumPackets(index, value) \
  this->assertTlm_LLR_NumPackets(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_LLR_NumSerialErrors_SIZE(size) \
  this->assertTlm_LLR_NumSerialErrors_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_LLR_NumSerialErrors(index, value) \
  this->assertTlm_LLR_NumSerialErrors(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_LLR_NumGoodPackets_SIZE(size) \
  this->assertTlm_LLR_NumGoodPackets_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_LLR_NumGoodPackets(index, value) \
  this->assertTlm_LLR_NumGoodPackets(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_LLR_NumDecodeErrors_SIZE(size) \
  this->assertTlm_LLR_NumDecodeErrors_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_LLR_NumDecodeErrors(index, value) \
  this->assertTlm_LLR_NumDecodeErrors(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_LLR_NumInvalidPorts_SIZE(size) \
  this->assertTlm_LLR_NumInvalidPorts_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_LLR_NumInvalidPorts(index, value) \
  this->assertTlm_LLR_NumInvalidPorts(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_LLR_NumBadSerialPortCalls_SIZE(size) \
  this->assertTlm_LLR_NumBadSerialPortCalls_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_LLR_NumBadSerialPortCalls(index, value) \
  this->assertTlm_LLR_NumBadSerialPortCalls(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_LLR_NumOuputBufferOverflows_SIZE(size) \
  this->assertTlm_LLR_NumOuputBufferOverflows_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_LLR_NumOuputBufferOverflows(index, value) \
  this->assertTlm_LLR_NumOuputBufferOverflows(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_LLR_NumZeroPktSize_SIZE(size) \
  this->assertTlm_LLR_NumZeroPktSize_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_LLR_NumZeroPktSize(index, value) \
  this->assertTlm_LLR_NumZeroPktSize(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_LLR_MaxCycleTime_SIZE(size) \
  this->assertTlm_LLR_MaxCycleTime_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_LLR_MaxCycleTime(index, value) \
  this->assertTlm_LLR_MaxCycleTime(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_LLR_CycleTime_SIZE(size) \
  this->assertTlm_LLR_CycleTime_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_LLR_CycleTime(index, value) \
  this->assertTlm_LLR_CycleTime(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_LLR_PatchProgress_SIZE(size) \
  this->assertTlm_LLR_PatchProgress_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_LLR_PatchProgress(index, value) \
  this->assertTlm_LLR_PatchProgress(__FILE__, __LINE__, index, value)

// ----------------------------------------------------------------------
// Macros for event history assertions 
// ----------------------------------------------------------------------

#define ASSERT_EVENTS_SIZE(size) \
  this->assertEvents_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_LLR_DataReceiveError_SIZE(size) \
  this->assertEvents_LLR_DataReceiveError_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_LLR_DataReceiveError(index, _error) \
  this->assertEvents_LLR_DataReceiveError(__FILE__, __LINE__, index, _error)

#define ASSERT_EVENTS_LLR_DataCrcError_SIZE(size) \
  this->assertEvents_LLR_DataCrcError_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_LLR_DataCrcError(index, _expected, _actual) \
  this->assertEvents_LLR_DataCrcError(__FILE__, __LINE__, index, _expected, _actual)

#define ASSERT_EVENTS_LLR_InvalidPortNum_SIZE(size) \
  this->assertEvents_LLR_InvalidPortNum_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_LLR_InvalidPortNum(index, _portNum) \
  this->assertEvents_LLR_InvalidPortNum(__FILE__, __LINE__, index, _portNum)

#define ASSERT_EVENTS_LLR_BadSerialPortCall_SIZE(size) \
  this->assertEvents_LLR_BadSerialPortCall_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_LLR_BadSerialPortCall(index, _status, _portNum) \
  this->assertEvents_LLR_BadSerialPortCall(__FILE__, __LINE__, index, _status, _portNum)

#define ASSERT_EVENTS_LLR_PatchFileOpen_SIZE(size) \
  this->assertEvents_LLR_PatchFileOpen_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_LLR_PatchFileOpen(index, _filename, _filesize) \
  this->assertEvents_LLR_PatchFileOpen(__FILE__, __LINE__, index, _filename, _filesize)

#define ASSERT_EVENTS_LLR_PatchModeChange_SIZE(size) \
  this->assertEvents_LLR_PatchModeChange_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_LLR_PatchModeChange(index, _newmode) \
  this->assertEvents_LLR_PatchModeChange(__FILE__, __LINE__, index, _newmode)

#define ASSERT_EVENTS_LLR_FirstACK_SIZE(size) \
  this->assertEvents_LLR_FirstACK_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_LLR_PatchEvent_SIZE(size) \
  this->assertEvents_LLR_PatchEvent_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_LLR_PatchEvent(index, _progress) \
  this->assertEvents_LLR_PatchEvent(__FILE__, __LINE__, index, _progress)

#define ASSERT_EVENTS_LLR_PatchComplete_SIZE(size) \
  this->assertEvents_LLR_PatchComplete_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_LLR_PatchError_SIZE(size) \
  this->assertEvents_LLR_PatchError_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_LLR_PatchError(index, _error, _err_num, _err_string) \
  this->assertEvents_LLR_PatchError(__FILE__, __LINE__, index, _error, _err_num, _err_string)

#define ASSERT_EVENTS_LLR_PolyError_SIZE(size) \
  this->assertEvents_LLR_PolyError_size(__FILE__, __LINE__, size)

// ----------------------------------------------------------------------
// Macros for typed user from port history assertions
// ----------------------------------------------------------------------

#define ASSERT_FROM_PORT_HISTORY_SIZE(size) \
  this->assertFromPortHistory_size(__FILE__, __LINE__, size)

#define ASSERT_from_SerWritePort_SIZE(size) \
  this->assert_from_SerWritePort_size(__FILE__, __LINE__, size)

#define ASSERT_from_SerWritePort(index, _serBuffer) \
  { \
    ASSERT_GT(this->fromPortHistory_SerWritePort->size(), static_cast<U32>(index)) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Index into history of from_SerWritePort\n" \
    << "  Expected: Less than size of history (" \
    << this->fromPortHistory_SerWritePort->size() << ")\n" \
    << "  Actual:   " << index << "\n"; \
    const FromPortEntry_SerWritePort& _e = \
      this->fromPortHistory_SerWritePort->at(index); \
    ASSERT_EQ(_serBuffer, _e.serBuffer) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument serBuffer at index " \
    << index \
    << " in history of from_SerWritePort\n" \
    << "  Expected: " << _serBuffer << "\n" \
    << "  Actual:   " << _e.serBuffer << "\n"; \
  }

#define ASSERT_from_SerialBufferSend_SIZE(size) \
  this->assert_from_SerialBufferSend_size(__FILE__, __LINE__, size)

#define ASSERT_from_SerialBufferSend(index, _fwBuffer) \
  { \
    ASSERT_GT(this->fromPortHistory_SerialBufferSend->size(), static_cast<U32>(index)) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Index into history of from_SerialBufferSend\n" \
    << "  Expected: Less than size of history (" \
    << this->fromPortHistory_SerialBufferSend->size() << ")\n" \
    << "  Actual:   " << index << "\n"; \
    const FromPortEntry_SerialBufferSend& _e = \
      this->fromPortHistory_SerialBufferSend->at(index); \
    ASSERT_EQ(_fwBuffer, _e.fwBuffer) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument fwBuffer at index " \
    << index \
    << " in history of from_SerialBufferSend\n" \
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

#define ASSERT_from_PolyGet_SIZE(size) \
  this->assert_from_PolyGet_size(__FILE__, __LINE__, size)

#define ASSERT_from_PolyGet(index, _entry, _status, _time, _val) \
  { \
    ASSERT_GT(this->fromPortHistory_PolyGet->size(), static_cast<U32>(index)) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Index into history of from_PolyGet\n" \
    << "  Expected: Less than size of history (" \
    << this->fromPortHistory_PolyGet->size() << ")\n" \
    << "  Actual:   " << index << "\n"; \
    const FromPortEntry_PolyGet& _e = \
      this->fromPortHistory_PolyGet->at(index); \
    ASSERT_EQ(_entry, _e.entry) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument entry at index " \
    << index \
    << " in history of from_PolyGet\n" \
    << "  Expected: " << _entry << "\n" \
    << "  Actual:   " << _e.entry << "\n"; \
    ASSERT_EQ(_status, _e.status) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument status at index " \
    << index \
    << " in history of from_PolyGet\n" \
    << "  Expected: " << _status << "\n" \
    << "  Actual:   " << _e.status << "\n"; \
    ASSERT_EQ(_time, _e.time) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument time at index " \
    << index \
    << " in history of from_PolyGet\n" \
    << "  Expected: " << _time << "\n" \
    << "  Actual:   " << _e.time << "\n"; \
    ASSERT_EQ(_val, _e.val) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument val at index " \
    << index \
    << " in history of from_PolyGet\n" \
    << "  Expected: " << _val << "\n" \
    << "  Actual:   " << _e.val << "\n"; \
  }

namespace HLProc {

  //! \class LLRouterGTestBase
  //! \brief Auto-generated base class for LLRouter component Google Test harness
  //!
  class LLRouterGTestBase :
    public LLRouterTesterBase
  {

    protected:

      // ----------------------------------------------------------------------
      // Construction and destruction
      // ----------------------------------------------------------------------

      //! Construct object LLRouterGTestBase
      //!
      LLRouterGTestBase(
#if FW_OBJECT_NAMES == 1
          const char *const compName, /*!< The component name*/
          const U32 maxHistorySize /*!< The maximum size of each history*/
#else
          const U32 maxHistorySize /*!< The maximum size of each history*/
#endif
      );

      //! Destroy object LLRouterGTestBase
      //!
      virtual ~LLRouterGTestBase(void);

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
          const U32 index, /*!< The index*/
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
      // Channel: LLR_CrcErrors
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_LLR_CrcErrors_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_LLR_CrcErrors(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 index, /*!< The index*/
          const U32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: LLR_ResyncErrors
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_LLR_ResyncErrors_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_LLR_ResyncErrors(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 index, /*!< The index*/
          const U32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: LLR_NumPackets
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_LLR_NumPackets_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_LLR_NumPackets(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 index, /*!< The index*/
          const U32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: LLR_NumSerialErrors
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_LLR_NumSerialErrors_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_LLR_NumSerialErrors(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 index, /*!< The index*/
          const U32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: LLR_NumGoodPackets
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_LLR_NumGoodPackets_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_LLR_NumGoodPackets(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 index, /*!< The index*/
          const U32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: LLR_NumDecodeErrors
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_LLR_NumDecodeErrors_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_LLR_NumDecodeErrors(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 index, /*!< The index*/
          const U32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: LLR_NumInvalidPorts
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_LLR_NumInvalidPorts_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_LLR_NumInvalidPorts(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 index, /*!< The index*/
          const U32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: LLR_NumBadSerialPortCalls
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_LLR_NumBadSerialPortCalls_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_LLR_NumBadSerialPortCalls(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 index, /*!< The index*/
          const U32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: LLR_NumOuputBufferOverflows
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_LLR_NumOuputBufferOverflows_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_LLR_NumOuputBufferOverflows(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 index, /*!< The index*/
          const U32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: LLR_NumZeroPktSize
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_LLR_NumZeroPktSize_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_LLR_NumZeroPktSize(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 index, /*!< The index*/
          const U32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: LLR_MaxCycleTime
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_LLR_MaxCycleTime_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_LLR_MaxCycleTime(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 index, /*!< The index*/
          const U32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: LLR_CycleTime
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_LLR_CycleTime_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_LLR_CycleTime(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 index, /*!< The index*/
          const U32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: LLR_PatchProgress
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_LLR_PatchProgress_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_LLR_PatchProgress(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 index, /*!< The index*/
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
      // Event: LLR_DataReceiveError
      // ----------------------------------------------------------------------

      void assertEvents_LLR_DataReceiveError_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_LLR_DataReceiveError(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 index, /*!< The index*/
          LLRouterComponentBase::LLR_ReceiveError error /*!< The receive eror code*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: LLR_DataCrcError
      // ----------------------------------------------------------------------

      void assertEvents_LLR_DataCrcError_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_LLR_DataCrcError(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 index, /*!< The index*/
          const U16 expected, 
          const U16 actual 
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: LLR_InvalidPortNum
      // ----------------------------------------------------------------------

      void assertEvents_LLR_InvalidPortNum_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_LLR_InvalidPortNum(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 index, /*!< The index*/
          const U8 portNum 
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: LLR_BadSerialPortCall
      // ----------------------------------------------------------------------

      void assertEvents_LLR_BadSerialPortCall_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_LLR_BadSerialPortCall(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 index, /*!< The index*/
          const I32 status, 
          const U8 portNum 
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: LLR_PatchFileOpen
      // ----------------------------------------------------------------------

      void assertEvents_LLR_PatchFileOpen_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_LLR_PatchFileOpen(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 index, /*!< The index*/
          const char *const filename, 
          const U32 filesize 
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: LLR_PatchModeChange
      // ----------------------------------------------------------------------

      void assertEvents_LLR_PatchModeChange_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_LLR_PatchModeChange(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 index, /*!< The index*/
          const char *const newmode 
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: LLR_FirstACK
      // ----------------------------------------------------------------------

      void assertEvents_LLR_FirstACK_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: LLR_PatchEvent
      // ----------------------------------------------------------------------

      void assertEvents_LLR_PatchEvent_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_LLR_PatchEvent(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 index, /*!< The index*/
          const U32 progress 
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: LLR_PatchComplete
      // ----------------------------------------------------------------------

      void assertEvents_LLR_PatchComplete_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: LLR_PatchError
      // ----------------------------------------------------------------------

      void assertEvents_LLR_PatchError_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_LLR_PatchError(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 index, /*!< The index*/
          LLRouterComponentBase::LLR_PatchError error, 
          const I32 err_num, 
          const char *const err_string 
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: LLR_PolyError
      // ----------------------------------------------------------------------

      void assertEvents_LLR_PolyError_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
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
      // From port: SerWritePort 
      // ----------------------------------------------------------------------

      void assert_from_SerWritePort_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // From port: SerialBufferSend 
      // ----------------------------------------------------------------------

      void assert_from_SerialBufferSend_size(
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

    protected:

      // ----------------------------------------------------------------------
      // From port: PolyGet 
      // ----------------------------------------------------------------------

      void assert_from_PolyGet_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

  };

} // end namespace HLProc

#endif
