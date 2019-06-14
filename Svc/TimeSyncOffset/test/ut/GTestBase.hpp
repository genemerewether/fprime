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

#define ASSERT_TLM_LLOffset_SIZE(size) \
  this->assertTlm_LLOffset_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_LLOffset(index, value) \
  this->assertTlm_LLOffset(__FILE__, __LINE__, index, value)

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

#define ASSERT_from_Offset_SIZE(size) \
  this->assert_from_Offset_size(__FILE__, __LINE__, size)

#define ASSERT_from_Offset(index, _time) \
  { \
    ASSERT_GT(this->fromPortHistory_Offset->size(), static_cast<U32>(index)) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Index into history of from_Offset\n" \
    << "  Expected: Less than size of history (" \
    << this->fromPortHistory_Offset->size() << ")\n" \
    << "  Actual:   " << index << "\n"; \
    const FromPortEntry_Offset& _e = \
      this->fromPortHistory_Offset->at(index); \
    ASSERT_EQ(_time, _e.time) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument time at index " \
    << index \
    << " in history of from_Offset\n" \
    << "  Expected: " << _time << "\n" \
    << "  Actual:   " << _e.time << "\n"; \
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
      // Channel: LLOffset
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_LLOffset_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_LLOffset(
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
      // From port: Offset
      // ----------------------------------------------------------------------

      void assert_from_Offset_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

  };

} // end namespace Svc

#endif
