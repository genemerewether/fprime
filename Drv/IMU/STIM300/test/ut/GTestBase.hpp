// ======================================================================
// \title  STIM300/test/ut/GTestBase.hpp
// \author Auto-generated
// \brief  hpp file for STIM300 component Google Test harness base class
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged.
//
// ======================================================================

#ifndef STIM300_GTEST_BASE_HPP
#define STIM300_GTEST_BASE_HPP

#include "TesterBase.hpp"
#include "gtest/gtest.h"

// ----------------------------------------------------------------------
// Macros for telemetry history assertions
// ----------------------------------------------------------------------

#define ASSERT_TLM_SIZE(size) \
  this->assertTlm_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_NumPackets_SIZE(size) \
  this->assertTlm_NumPackets_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_NumPackets(index, value) \
  this->assertTlm_NumPackets(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_TimeSyncStatus_SIZE(size) \
  this->assertTlm_TimeSyncStatus_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_TimeSyncStatus(index, value) \
  this->assertTlm_TimeSyncStatus(__FILE__, __LINE__, index, value)

// ----------------------------------------------------------------------
// Macros for event history assertions
// ----------------------------------------------------------------------

#define ASSERT_EVENTS_SIZE(size) \
  this->assertEvents_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_BufferFull_SIZE(size) \
  this->assertEvents_BufferFull_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_UartError_SIZE(size) \
  this->assertEvents_UartError_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_NoEvents_SIZE(size) \
  this->assertEvents_NoEvents_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_InvalidCounter_SIZE(size) \
  this->assertEvents_InvalidCounter_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_InvalidCounter(index, _actualCount, _expectedCount) \
  this->assertEvents_InvalidCounter(__FILE__, __LINE__, index, _actualCount, _expectedCount)

#define ASSERT_EVENTS_TooManyEvents_SIZE(size) \
  this->assertEvents_TooManyEvents_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_TooManyEvents(index, _maxEvents) \
  this->assertEvents_TooManyEvents(__FILE__, __LINE__, index, _maxEvents)

#define ASSERT_EVENTS_BadTimeSync_SIZE(size) \
  this->assertEvents_BadTimeSync_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_SyncComplete_SIZE(size) \
  this->assertEvents_SyncComplete_size(__FILE__, __LINE__, size)

// ----------------------------------------------------------------------
// Macros for typed user from port history assertions
// ----------------------------------------------------------------------

#define ASSERT_FROM_PORT_HISTORY_SIZE(size) \
  this->assertFromPortHistory_size(__FILE__, __LINE__, size)

#define ASSERT_from_IMU_SIZE(size) \
  this->assert_from_IMU_size(__FILE__, __LINE__, size)

#define ASSERT_from_IMU(index, _ImuNoCov) \
  { \
    ASSERT_GT(this->fromPortHistory_IMU->size(), static_cast<U32>(index)) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Index into history of from_IMU\n" \
    << "  Expected: Less than size of history (" \
    << this->fromPortHistory_IMU->size() << ")\n" \
    << "  Actual:   " << index << "\n"; \
    const FromPortEntry_IMU& _e = \
      this->fromPortHistory_IMU->at(index); \
    ASSERT_EQ(_ImuNoCov, _e.ImuNoCov) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument ImuNoCov at index " \
    << index \
    << " in history of from_IMU\n" \
    << "  Expected: " << _ImuNoCov << "\n" \
    << "  Actual:   " << _e.ImuNoCov << "\n"; \
  }

#define ASSERT_from_packetTime_SIZE(size) \
  this->assert_from_packetTime_size(__FILE__, __LINE__, size)

#define ASSERT_from_packetTime(index, _time) \
  { \
    ASSERT_GT(this->fromPortHistory_packetTime->size(), static_cast<U32>(index)) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Index into history of from_packetTime\n" \
    << "  Expected: Less than size of history (" \
    << this->fromPortHistory_packetTime->size() << ")\n" \
    << "  Actual:   " << index << "\n"; \
    const FromPortEntry_packetTime& _e = \
      this->fromPortHistory_packetTime->at(index); \
    ASSERT_EQ(_time, _e.time) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument time at index " \
    << index \
    << " in history of from_packetTime\n" \
    << "  Expected: " << _time << "\n" \
    << "  Actual:   " << _e.time << "\n"; \
  }

#define ASSERT_from_serialRead_SIZE(size) \
  this->assert_from_serialRead_size(__FILE__, __LINE__, size)

#define ASSERT_from_serialRead(index, _serBuffer, _status) \
  { \
    ASSERT_GT(this->fromPortHistory_serialRead->size(), static_cast<U32>(index)) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Index into history of from_serialRead\n" \
    << "  Expected: Less than size of history (" \
    << this->fromPortHistory_serialRead->size() << ")\n" \
    << "  Actual:   " << index << "\n"; \
    const FromPortEntry_serialRead& _e = \
      this->fromPortHistory_serialRead->at(index); \
    ASSERT_EQ(_serBuffer, _e.serBuffer) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument serBuffer at index " \
    << index \
    << " in history of from_serialRead\n" \
    << "  Expected: " << _serBuffer << "\n" \
    << "  Actual:   " << _e.serBuffer << "\n"; \
    ASSERT_EQ(_status, _e.status) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument status at index " \
    << index \
    << " in history of from_serialRead\n" \
    << "  Expected: " << _status << "\n" \
    << "  Actual:   " << _e.status << "\n"; \
  }

namespace Drv {

  //! \class STIM300GTestBase
  //! \brief Auto-generated base class for STIM300 component Google Test harness
  //!
  class STIM300GTestBase :
    public STIM300TesterBase
  {

    protected:

      // ----------------------------------------------------------------------
      // Construction and destruction
      // ----------------------------------------------------------------------

      //! Construct object STIM300GTestBase
      //!
      STIM300GTestBase(
#if FW_OBJECT_NAMES == 1
          const char *const compName, /*!< The component name*/
          const U32 maxHistorySize /*!< The maximum size of each history*/
#else
          const U32 maxHistorySize /*!< The maximum size of each history*/
#endif
      );

      //! Destroy object STIM300GTestBase
      //!
      virtual ~STIM300GTestBase(void);

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
      // Channel: NumPackets
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_NumPackets_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_NumPackets(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const U32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: TimeSyncStatus
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_TimeSyncStatus_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_TimeSyncStatus(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const STIM300ComponentBase::STIM300TimeSync& val /*!< The channel value*/
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
      // Event: BufferFull
      // ----------------------------------------------------------------------

      void assertEvents_BufferFull_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: UartError
      // ----------------------------------------------------------------------

      void assertEvents_UartError_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: NoEvents
      // ----------------------------------------------------------------------

      void assertEvents_NoEvents_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: InvalidCounter
      // ----------------------------------------------------------------------

      void assertEvents_InvalidCounter_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_InvalidCounter(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const U32 actualCount, 
          const U32 expectedCount 
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: TooManyEvents
      // ----------------------------------------------------------------------

      void assertEvents_TooManyEvents_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_TooManyEvents(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const U32 maxEvents 
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: BadTimeSync
      // ----------------------------------------------------------------------

      void assertEvents_BadTimeSync_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: SyncComplete
      // ----------------------------------------------------------------------

      void assertEvents_SyncComplete_size(
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
      // From port: IMU
      // ----------------------------------------------------------------------

      void assert_from_IMU_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // From port: packetTime
      // ----------------------------------------------------------------------

      void assert_from_packetTime_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // From port: serialRead
      // ----------------------------------------------------------------------

      void assert_from_serialRead_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

  };

} // end namespace Drv

#endif
