// ======================================================================
// \title  BlspPwmDriver/test/ut/GTestBase.hpp
// \author Auto-generated
// \brief  hpp file for BlspPwmDriver component Google Test harness base class
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged.
//
// ======================================================================

#ifndef BlspPwmDriver_GTEST_BASE_HPP
#define BlspPwmDriver_GTEST_BASE_HPP

#include "TesterBase.hpp"
#include "gtest/gtest.h"

// ----------------------------------------------------------------------
// Macros for event history assertions
// ----------------------------------------------------------------------

#define ASSERT_EVENTS_SIZE(size) \
  this->assertEvents_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_PWM_OpenError_SIZE(size) \
  this->assertEvents_PWM_OpenError_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_PWM_OpenError(index, _chip, _line, _error) \
  this->assertEvents_PWM_OpenError(__FILE__, __LINE__, index, _chip, _line, _error)

#define ASSERT_EVENTS_PWM_ConfigError_SIZE(size) \
  this->assertEvents_PWM_ConfigError_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_PWM_ConfigError(index, _chip, _line, _error) \
  this->assertEvents_PWM_ConfigError(__FILE__, __LINE__, index, _chip, _line, _error)

#define ASSERT_EVENTS_PWM_WriteError_SIZE(size) \
  this->assertEvents_PWM_WriteError_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_PWM_WriteError(index, _chip, _line, _error) \
  this->assertEvents_PWM_WriteError(__FILE__, __LINE__, index, _chip, _line, _error)

#define ASSERT_EVENTS_PWM_ReadError_SIZE(size) \
  this->assertEvents_PWM_ReadError_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_PWM_ReadError(index, _chip, _line, _error) \
  this->assertEvents_PWM_ReadError(__FILE__, __LINE__, index, _chip, _line, _error)

#define ASSERT_EVENTS_PWM_PortOpened_SIZE(size) \
  this->assertEvents_PWM_PortOpened_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_PWM_PortOpened(index, _chip, _line) \
  this->assertEvents_PWM_PortOpened(__FILE__, __LINE__, index, _chip, _line)

namespace SnapdragonFlight {

  //! \class BlspPwmDriverGTestBase
  //! \brief Auto-generated base class for BlspPwmDriver component Google Test harness
  //!
  class BlspPwmDriverGTestBase :
    public BlspPwmDriverTesterBase
  {

    protected:

      // ----------------------------------------------------------------------
      // Construction and destruction
      // ----------------------------------------------------------------------

      //! Construct object BlspPwmDriverGTestBase
      //!
      BlspPwmDriverGTestBase(
#if FW_OBJECT_NAMES == 1
          const char *const compName, /*!< The component name*/
          const U32 maxHistorySize /*!< The maximum size of each history*/
#else
          const U32 maxHistorySize /*!< The maximum size of each history*/
#endif
      );

      //! Destroy object BlspPwmDriverGTestBase
      //!
      virtual ~BlspPwmDriverGTestBase(void);

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
      // Event: PWM_OpenError
      // ----------------------------------------------------------------------

      void assertEvents_PWM_OpenError_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_PWM_OpenError(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const I32 chip, /*!< The PWM chip*/
          const I32 line, /*!< The PWM line*/
          const I32 error /*!< The error code*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: PWM_ConfigError
      // ----------------------------------------------------------------------

      void assertEvents_PWM_ConfigError_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_PWM_ConfigError(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const I32 chip, /*!< The PWM chip*/
          const I32 line, /*!< The PWM line*/
          const I32 error /*!< The error code*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: PWM_WriteError
      // ----------------------------------------------------------------------

      void assertEvents_PWM_WriteError_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_PWM_WriteError(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const I32 chip, /*!< The PWM chip*/
          const I32 line, /*!< The PWM line*/
          const I32 error /*!< The error code*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: PWM_ReadError
      // ----------------------------------------------------------------------

      void assertEvents_PWM_ReadError_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_PWM_ReadError(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const I32 chip, /*!< The PWM chip*/
          const I32 line, /*!< The PWM line*/
          const I32 error /*!< The error code*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: PWM_PortOpened
      // ----------------------------------------------------------------------

      void assertEvents_PWM_PortOpened_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_PWM_PortOpened(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const I32 chip, /*!< The PWM chip*/
          const I32 line /*!< The PWM line*/
      ) const;

  };

} // end namespace SnapdragonFlight

#endif
