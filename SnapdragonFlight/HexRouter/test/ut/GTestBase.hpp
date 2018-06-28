// ======================================================================
// \title  HexRouter/test/ut/GTestBase.hpp
// \author Auto-generated
// \brief  hpp file for HexRouter component Google Test harness base class
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

#ifndef HexRouter_GTEST_BASE_HPP
#define HexRouter_GTEST_BASE_HPP

#include "TesterBase.hpp"
#include "gtest/gtest.h"

// ----------------------------------------------------------------------
// Macros for telemetry history assertions
// ----------------------------------------------------------------------

#define ASSERT_TLM_SIZE(size) \
  this->assertTlm_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_HR_ResyncErrors_SIZE(size) \
  this->assertTlm_HR_ResyncErrors_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_HR_ResyncErrors(index, value) \
  this->assertTlm_HR_ResyncErrors(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_HR_NumPackets_SIZE(size) \
  this->assertTlm_HR_NumPackets_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_HR_NumPackets(index, value) \
  this->assertTlm_HR_NumPackets(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_HR_NumDecodeErrors_SIZE(size) \
  this->assertTlm_HR_NumDecodeErrors_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_HR_NumDecodeErrors(index, value) \
  this->assertTlm_HR_NumDecodeErrors(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_HR_NumInvalidPorts_SIZE(size) \
  this->assertTlm_HR_NumInvalidPorts_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_HR_NumInvalidPorts(index, value) \
  this->assertTlm_HR_NumInvalidPorts(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_HR_NumBadSerialPortCalls_SIZE(size) \
  this->assertTlm_HR_NumBadSerialPortCalls_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_HR_NumBadSerialPortCalls(index, value) \
  this->assertTlm_HR_NumBadSerialPortCalls(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_HR_NumOutputBufferOverflows_SIZE(size) \
  this->assertTlm_HR_NumOutputBufferOverflows_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_HR_NumOutputBufferOverflows(index, value) \
  this->assertTlm_HR_NumOutputBufferOverflows(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_HR_NumZeroPktSize_SIZE(size) \
  this->assertTlm_HR_NumZeroPktSize_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_HR_NumZeroPktSize(index, value) \
  this->assertTlm_HR_NumZeroPktSize(__FILE__, __LINE__, index, value)

// ----------------------------------------------------------------------
// Macros for event history assertions 
// ----------------------------------------------------------------------

#define ASSERT_EVENTS_SIZE(size) \
  this->assertEvents_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_HR_DataReceiveError_SIZE(size) \
  this->assertEvents_HR_DataReceiveError_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_HR_DataReceiveError(index, _error, _stat) \
  this->assertEvents_HR_DataReceiveError(__FILE__, __LINE__, index, _error, _stat)

#define ASSERT_EVENTS_HR_InvalidPortNum_SIZE(size) \
  this->assertEvents_HR_InvalidPortNum_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_HR_InvalidPortNum(index, _PortType, _portNum) \
  this->assertEvents_HR_InvalidPortNum(__FILE__, __LINE__, index, _PortType, _portNum)

#define ASSERT_EVENTS_HR_BadSerialPortCall_SIZE(size) \
  this->assertEvents_HR_BadSerialPortCall_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_HR_BadSerialPortCall(index, _status, _portNum) \
  this->assertEvents_HR_BadSerialPortCall(__FILE__, __LINE__, index, _status, _portNum)

#define ASSERT_EVENTS_HR_MemoryError_SIZE(size) \
  this->assertEvents_HR_MemoryError_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_HR_MemoryError(index, _error) \
  this->assertEvents_HR_MemoryError(__FILE__, __LINE__, index, _error)

#define ASSERT_EVENTS_HR_WriteError_SIZE(size) \
  this->assertEvents_HR_WriteError_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_HR_WriteError(index, _error) \
  this->assertEvents_HR_WriteError(__FILE__, __LINE__, index, _error)

// ----------------------------------------------------------------------
// Macros for typed user from port history assertions
// ----------------------------------------------------------------------

#define ASSERT_FROM_PORT_HISTORY_SIZE(size) \
  this->assertFromPortHistory_size(__FILE__, __LINE__, size)

#define ASSERT_from_readBufferSend_SIZE(size) \
  this->assert_from_readBufferSend_size(__FILE__, __LINE__, size)

#define ASSERT_from_readBufferSend(index, _fwBuffer) \
  { \
    ASSERT_GT(this->fromPortHistory_readBufferSend->size(), static_cast<U32>(index)) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Index into history of from_readBufferSend\n" \
    << "  Expected: Less than size of history (" \
    << this->fromPortHistory_readBufferSend->size() << ")\n" \
    << "  Actual:   " << index << "\n"; \
    const FromPortEntry_readBufferSend& _e = \
      this->fromPortHistory_readBufferSend->at(index); \
    ASSERT_EQ(_fwBuffer, _e.fwBuffer) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument fwBuffer at index " \
    << index \
    << " in history of from_readBufferSend\n" \
    << "  Expected: " << _fwBuffer << "\n" \
    << "  Actual:   " << _e.fwBuffer << "\n"; \
  }

namespace SnapdragonFlight {

  //! \class HexRouterGTestBase
  //! \brief Auto-generated base class for HexRouter component Google Test harness
  //!
  class HexRouterGTestBase :
    public HexRouterTesterBase
  {

    protected:

      // ----------------------------------------------------------------------
      // Construction and destruction
      // ----------------------------------------------------------------------

      //! Construct object HexRouterGTestBase
      //!
      HexRouterGTestBase(
#if FW_OBJECT_NAMES == 1
          const char *const compName, /*!< The component name*/
          const U32 maxHistorySize /*!< The maximum size of each history*/
#else
          const U32 maxHistorySize /*!< The maximum size of each history*/
#endif
      );

      //! Destroy object HexRouterGTestBase
      //!
      virtual ~HexRouterGTestBase(void);

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
      // Channel: HR_ResyncErrors
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_HR_ResyncErrors_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_HR_ResyncErrors(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 index, /*!< The index*/
          const U32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: HR_NumPackets
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_HR_NumPackets_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_HR_NumPackets(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 index, /*!< The index*/
          const U32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: HR_NumDecodeErrors
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_HR_NumDecodeErrors_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_HR_NumDecodeErrors(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 index, /*!< The index*/
          const U32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: HR_NumInvalidPorts
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_HR_NumInvalidPorts_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_HR_NumInvalidPorts(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 index, /*!< The index*/
          const U32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: HR_NumBadSerialPortCalls
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_HR_NumBadSerialPortCalls_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_HR_NumBadSerialPortCalls(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 index, /*!< The index*/
          const U32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: HR_NumOutputBufferOverflows
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_HR_NumOutputBufferOverflows_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_HR_NumOutputBufferOverflows(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 index, /*!< The index*/
          const U32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: HR_NumZeroPktSize
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_HR_NumZeroPktSize_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_HR_NumZeroPktSize(
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
      // Event: HR_DataReceiveError
      // ----------------------------------------------------------------------

      void assertEvents_HR_DataReceiveError_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_HR_DataReceiveError(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 index, /*!< The index*/
          HexRouterComponentBase::HR_ReceiveError error, /*!< The receive error code*/
          const I32 stat /*!< Status code - depends on error type*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: HR_InvalidPortNum
      // ----------------------------------------------------------------------

      void assertEvents_HR_InvalidPortNum_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_HR_InvalidPortNum(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 index, /*!< The index*/
          HexRouterComponentBase::HR_PortType PortType, /*!< Type of port that was not connected*/
          const U8 portNum 
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: HR_BadSerialPortCall
      // ----------------------------------------------------------------------

      void assertEvents_HR_BadSerialPortCall_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_HR_BadSerialPortCall(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 index, /*!< The index*/
          const I32 status, 
          const U8 portNum 
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: HR_MemoryError
      // ----------------------------------------------------------------------

      void assertEvents_HR_MemoryError_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_HR_MemoryError(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 index, /*!< The index*/
          HexRouterComponentBase::HR_MemoryError error /*!< Type of memory operation that failed*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: HR_WriteError
      // ----------------------------------------------------------------------

      void assertEvents_HR_WriteError_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_HR_WriteError(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 index, /*!< The index*/
          const I32 error /*!< The error code*/
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
      // From port: readBufferSend 
      // ----------------------------------------------------------------------

      void assert_from_readBufferSend_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

  };

} // end namespace SnapdragonFlight

#endif
