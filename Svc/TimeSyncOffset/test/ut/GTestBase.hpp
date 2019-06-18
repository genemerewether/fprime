// ======================================================================
// \title  TimeSyncOffset/test/ut/GTestBase.hpp
// \author Auto-generated
// \brief  hpp file for TimeSyncOffset component Google Test harness base class
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged.
//
// ======================================================================

#ifndef TimeSyncOffset_GTEST_BASE_HPP
#define TimeSyncOffset_GTEST_BASE_HPP

#include "TesterBase.hpp"
#include "gtest/gtest.h"

// ----------------------------------------------------------------------
// Macros for telemetry history assertions
// ----------------------------------------------------------------------

#define ASSERT_TLM_SIZE(size) \
  this->assertTlm_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_LLTime_SIZE(size) \
  this->assertTlm_LLTime_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_LLTime(index, value) \
  this->assertTlm_LLTime(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_HLTime_SIZE(size) \
  this->assertTlm_HLTime_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_HLTime(index, value) \
  this->assertTlm_HLTime(__FILE__, __LINE__, index, value)

// ----------------------------------------------------------------------
// Macros for typed user from port history assertions
// ----------------------------------------------------------------------

#define ASSERT_FROM_PORT_HISTORY_SIZE(size) \
  this->assertFromPortHistory_size(__FILE__, __LINE__, size)

#define ASSERT_from_GPIOPulse_SIZE(size) \
  this->assert_from_GPIOPulse_size(__FILE__, __LINE__, size)

#define ASSERT_from_GPIOPulse(index, _state) \
  { \
    ASSERT_GT(this->fromPortHistory_GPIOPulse->size(), static_cast<U32>(index)) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Index into history of from_GPIOPulse\n" \
    << "  Expected: Less than size of history (" \
    << this->fromPortHistory_GPIOPulse->size() << ")\n" \
    << "  Actual:   " << index << "\n"; \
    const FromPortEntry_GPIOPulse& _e = \
      this->fromPortHistory_GPIOPulse->at(index); \
    ASSERT_EQ(_state, _e.state) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument state at index " \
    << index \
    << " in history of from_GPIOPulse\n" \
    << "  Expected: " << _state << "\n" \
    << "  Actual:   " << _e.state << "\n"; \
  }

#define ASSERT_from_ClockTimes_SIZE(size) \
  this->assert_from_ClockTimes_size(__FILE__, __LINE__, size)

#define ASSERT_from_ClockTimes(index, _time1, _time2) \
  { \
    ASSERT_GT(this->fromPortHistory_ClockTimes->size(), static_cast<U32>(index)) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Index into history of from_ClockTimes\n" \
    << "  Expected: Less than size of history (" \
    << this->fromPortHistory_ClockTimes->size() << ")\n" \
    << "  Actual:   " << index << "\n"; \
    const FromPortEntry_ClockTimes& _e = \
      this->fromPortHistory_ClockTimes->at(index); \
    ASSERT_EQ(_time1, _e.time1) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument time1 at index " \
    << index \
    << " in history of from_ClockTimes\n" \
    << "  Expected: " << _time1 << "\n" \
    << "  Actual:   " << _e.time1 << "\n"; \
    ASSERT_EQ(_time2, _e.time2) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument time2 at index " \
    << index \
    << " in history of from_ClockTimes\n" \
    << "  Expected: " << _time2 << "\n" \
    << "  Actual:   " << _e.time2 << "\n"; \
  }

namespace Svc {

  //! \class TimeSyncOffsetGTestBase
  //! \brief Auto-generated base class for TimeSyncOffset component Google Test harness
  //!
  class TimeSyncOffsetGTestBase :
    public TimeSyncOffsetTesterBase
  {

    protected:

      // ----------------------------------------------------------------------
      // Construction and destruction
      // ----------------------------------------------------------------------

      //! Construct object TimeSyncOffsetGTestBase
      //!
      TimeSyncOffsetGTestBase(
#if FW_OBJECT_NAMES == 1
          const char *const compName, /*!< The component name*/
          const U32 maxHistorySize /*!< The maximum size of each history*/
#else
          const U32 maxHistorySize /*!< The maximum size of each history*/
#endif
      );

      //! Destroy object TimeSyncOffsetGTestBase
      //!
      virtual ~TimeSyncOffsetGTestBase(void);

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
      // Channel: LLTime
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_LLTime_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_LLTime(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const F64& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: HLTime
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_HLTime_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_HLTime(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const F64& val /*!< The channel value*/
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
      // From port: GPIOPulse
      // ----------------------------------------------------------------------

      void assert_from_GPIOPulse_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // From port: ClockTimes
      // ----------------------------------------------------------------------

      void assert_from_ClockTimes_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

  };

} // end namespace Svc

#endif
