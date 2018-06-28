// ======================================================================
// \title  KraitRouter/test/ut/GTestBase.hpp
// \author Auto-generated
// \brief  hpp file for KraitRouter component Google Test harness base class
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

#ifndef KraitRouter_GTEST_BASE_HPP
#define KraitRouter_GTEST_BASE_HPP

#include "TesterBase.hpp"
#include "gtest/gtest.h"

// ----------------------------------------------------------------------
// Macros for telemetry history assertions
// ----------------------------------------------------------------------

#define ASSERT_TLM_SIZE(size) \
  this->assertTlm_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_KR_ResyncErrors_SIZE(size) \
  this->assertTlm_KR_ResyncErrors_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_KR_ResyncErrors(index, value) \
  this->assertTlm_KR_ResyncErrors(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_KR_NumPackets_SIZE(size) \
  this->assertTlm_KR_NumPackets_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_KR_NumPackets(index, value) \
  this->assertTlm_KR_NumPackets(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_KR_NumDecodeErrors_SIZE(size) \
  this->assertTlm_KR_NumDecodeErrors_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_KR_NumDecodeErrors(index, value) \
  this->assertTlm_KR_NumDecodeErrors(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_KR_NumInvalidPorts_SIZE(size) \
  this->assertTlm_KR_NumInvalidPorts_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_KR_NumInvalidPorts(index, value) \
  this->assertTlm_KR_NumInvalidPorts(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_KR_NumBadSerialPortCalls_SIZE(size) \
  this->assertTlm_KR_NumBadSerialPortCalls_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_KR_NumBadSerialPortCalls(index, value) \
  this->assertTlm_KR_NumBadSerialPortCalls(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_KR_NumOutputBufferOverflows_SIZE(size) \
  this->assertTlm_KR_NumOutputBufferOverflows_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_KR_NumOutputBufferOverflows(index, value) \
  this->assertTlm_KR_NumOutputBufferOverflows(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_KR_NumZeroPktSize_SIZE(size) \
  this->assertTlm_KR_NumZeroPktSize_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_KR_NumZeroPktSize(index, value) \
  this->assertTlm_KR_NumZeroPktSize(__FILE__, __LINE__, index, value)

// ----------------------------------------------------------------------
// Macros for event history assertions 
// ----------------------------------------------------------------------

#define ASSERT_EVENTS_SIZE(size) \
  this->assertEvents_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_KR_DataReceiveError_SIZE(size) \
  this->assertEvents_KR_DataReceiveError_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_KR_DataReceiveError(index, _error, _stat) \
  this->assertEvents_KR_DataReceiveError(__FILE__, __LINE__, index, _error, _stat)

#define ASSERT_EVENTS_KR_InvalidPortNum_SIZE(size) \
  this->assertEvents_KR_InvalidPortNum_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_KR_InvalidPortNum(index, _PortType, _portNum) \
  this->assertEvents_KR_InvalidPortNum(__FILE__, __LINE__, index, _PortType, _portNum)

#define ASSERT_EVENTS_KR_BadSerialPortCall_SIZE(size) \
  this->assertEvents_KR_BadSerialPortCall_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_KR_BadSerialPortCall(index, _status, _portNum) \
  this->assertEvents_KR_BadSerialPortCall(__FILE__, __LINE__, index, _status, _portNum)

#define ASSERT_EVENTS_KR_MemoryError_SIZE(size) \
  this->assertEvents_KR_MemoryError_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_KR_MemoryError(index, _error) \
  this->assertEvents_KR_MemoryError(__FILE__, __LINE__, index, _error)

#define ASSERT_EVENTS_KR_WriteError_SIZE(size) \
  this->assertEvents_KR_WriteError_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_KR_WriteError(index, _error) \
  this->assertEvents_KR_WriteError(__FILE__, __LINE__, index, _error)

namespace SnapdragonFlight {

  //! \class KraitRouterGTestBase
  //! \brief Auto-generated base class for KraitRouter component Google Test harness
  //!
  class KraitRouterGTestBase :
    public KraitRouterTesterBase
  {

    protected:

      // ----------------------------------------------------------------------
      // Construction and destruction
      // ----------------------------------------------------------------------

      //! Construct object KraitRouterGTestBase
      //!
      KraitRouterGTestBase(
#if FW_OBJECT_NAMES == 1
          const char *const compName, /*!< The component name*/
          const U32 maxHistorySize /*!< The maximum size of each history*/
#else
          const U32 maxHistorySize /*!< The maximum size of each history*/
#endif
      );

      //! Destroy object KraitRouterGTestBase
      //!
      virtual ~KraitRouterGTestBase(void);

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
      // Channel: KR_ResyncErrors
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_KR_ResyncErrors_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_KR_ResyncErrors(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 index, /*!< The index*/
          const U32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: KR_NumPackets
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_KR_NumPackets_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_KR_NumPackets(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 index, /*!< The index*/
          const U32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: KR_NumDecodeErrors
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_KR_NumDecodeErrors_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_KR_NumDecodeErrors(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 index, /*!< The index*/
          const U32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: KR_NumInvalidPorts
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_KR_NumInvalidPorts_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_KR_NumInvalidPorts(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 index, /*!< The index*/
          const U32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: KR_NumBadSerialPortCalls
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_KR_NumBadSerialPortCalls_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_KR_NumBadSerialPortCalls(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 index, /*!< The index*/
          const U32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: KR_NumOutputBufferOverflows
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_KR_NumOutputBufferOverflows_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_KR_NumOutputBufferOverflows(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 index, /*!< The index*/
          const U32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: KR_NumZeroPktSize
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_KR_NumZeroPktSize_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_KR_NumZeroPktSize(
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
      // Event: KR_DataReceiveError
      // ----------------------------------------------------------------------

      void assertEvents_KR_DataReceiveError_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_KR_DataReceiveError(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 index, /*!< The index*/
          KraitRouterComponentBase::KR_ReceiveError error, /*!< The receive error code*/
          const I32 stat /*!< Status code - depends on error type*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: KR_InvalidPortNum
      // ----------------------------------------------------------------------

      void assertEvents_KR_InvalidPortNum_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_KR_InvalidPortNum(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 index, /*!< The index*/
          KraitRouterComponentBase::KR_PortType PortType, /*!< Type of port that was not connected*/
          const U8 portNum 
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: KR_BadSerialPortCall
      // ----------------------------------------------------------------------

      void assertEvents_KR_BadSerialPortCall_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_KR_BadSerialPortCall(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 index, /*!< The index*/
          const I32 status, 
          const U8 portNum 
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: KR_MemoryError
      // ----------------------------------------------------------------------

      void assertEvents_KR_MemoryError_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_KR_MemoryError(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 index, /*!< The index*/
          KraitRouterComponentBase::KR_MemoryError error /*!< Type of memory operation that failed*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: KR_WriteError
      // ----------------------------------------------------------------------

      void assertEvents_KR_WriteError_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_KR_WriteError(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 index, /*!< The index*/
          const I32 error /*!< The error code*/
      ) const;

  };

} // end namespace SnapdragonFlight

#endif
