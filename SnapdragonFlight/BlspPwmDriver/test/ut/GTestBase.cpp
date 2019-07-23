// ======================================================================
// \title  BlspPwmDriver/test/ut/GTestBase.cpp
// \author Auto-generated
// \brief  cpp file for BlspPwmDriver component Google Test harness base class
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged.
//
// ======================================================================

#include "GTestBase.hpp"

namespace SnapdragonFlight {

  // ----------------------------------------------------------------------
  // Construction and destruction
  // ----------------------------------------------------------------------

  BlspPwmDriverGTestBase ::
    BlspPwmDriverGTestBase(
#if FW_OBJECT_NAMES == 1
        const char *const compName,
        const U32 maxHistorySize
#else
        const U32 maxHistorySize
#endif
    ) :
        BlspPwmDriverTesterBase (
#if FW_OBJECT_NAMES == 1
            compName,
#endif
            maxHistorySize
        )
  {

  }

  BlspPwmDriverGTestBase ::
    ~BlspPwmDriverGTestBase(void)
  {

  }

  // ----------------------------------------------------------------------
  // Events
  // ----------------------------------------------------------------------

  void BlspPwmDriverGTestBase ::
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
  // Event: PWM_OpenError
  // ----------------------------------------------------------------------

  void BlspPwmDriverGTestBase ::
    assertEvents_PWM_OpenError_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_PWM_OpenError->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event PWM_OpenError\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_PWM_OpenError->size() << "\n";
  }

  void BlspPwmDriverGTestBase ::
    assertEvents_PWM_OpenError(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const I32 chip,
        const I32 line,
        const I32 error
    ) const
  {
    ASSERT_GT(this->eventHistory_PWM_OpenError->size(), __index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event PWM_OpenError\n"
      << "  Expected: Less than size of history ("
      << this->eventHistory_PWM_OpenError->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const EventEntry_PWM_OpenError& e =
      this->eventHistory_PWM_OpenError->at(__index);
    ASSERT_EQ(chip, e.chip)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument chip at index "
      << __index
      << " in history of event PWM_OpenError\n"
      << "  Expected: " << chip << "\n"
      << "  Actual:   " << e.chip << "\n";
    ASSERT_EQ(line, e.line)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument line at index "
      << __index
      << " in history of event PWM_OpenError\n"
      << "  Expected: " << line << "\n"
      << "  Actual:   " << e.line << "\n";
    ASSERT_EQ(error, e.error)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument error at index "
      << __index
      << " in history of event PWM_OpenError\n"
      << "  Expected: " << error << "\n"
      << "  Actual:   " << e.error << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: PWM_ConfigError
  // ----------------------------------------------------------------------

  void BlspPwmDriverGTestBase ::
    assertEvents_PWM_ConfigError_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_PWM_ConfigError->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event PWM_ConfigError\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_PWM_ConfigError->size() << "\n";
  }

  void BlspPwmDriverGTestBase ::
    assertEvents_PWM_ConfigError(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const I32 chip,
        const I32 line,
        const I32 error
    ) const
  {
    ASSERT_GT(this->eventHistory_PWM_ConfigError->size(), __index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event PWM_ConfigError\n"
      << "  Expected: Less than size of history ("
      << this->eventHistory_PWM_ConfigError->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const EventEntry_PWM_ConfigError& e =
      this->eventHistory_PWM_ConfigError->at(__index);
    ASSERT_EQ(chip, e.chip)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument chip at index "
      << __index
      << " in history of event PWM_ConfigError\n"
      << "  Expected: " << chip << "\n"
      << "  Actual:   " << e.chip << "\n";
    ASSERT_EQ(line, e.line)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument line at index "
      << __index
      << " in history of event PWM_ConfigError\n"
      << "  Expected: " << line << "\n"
      << "  Actual:   " << e.line << "\n";
    ASSERT_EQ(error, e.error)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument error at index "
      << __index
      << " in history of event PWM_ConfigError\n"
      << "  Expected: " << error << "\n"
      << "  Actual:   " << e.error << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: PWM_WriteError
  // ----------------------------------------------------------------------

  void BlspPwmDriverGTestBase ::
    assertEvents_PWM_WriteError_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_PWM_WriteError->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event PWM_WriteError\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_PWM_WriteError->size() << "\n";
  }

  void BlspPwmDriverGTestBase ::
    assertEvents_PWM_WriteError(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const I32 chip,
        const I32 line,
        const I32 error
    ) const
  {
    ASSERT_GT(this->eventHistory_PWM_WriteError->size(), __index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event PWM_WriteError\n"
      << "  Expected: Less than size of history ("
      << this->eventHistory_PWM_WriteError->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const EventEntry_PWM_WriteError& e =
      this->eventHistory_PWM_WriteError->at(__index);
    ASSERT_EQ(chip, e.chip)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument chip at index "
      << __index
      << " in history of event PWM_WriteError\n"
      << "  Expected: " << chip << "\n"
      << "  Actual:   " << e.chip << "\n";
    ASSERT_EQ(line, e.line)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument line at index "
      << __index
      << " in history of event PWM_WriteError\n"
      << "  Expected: " << line << "\n"
      << "  Actual:   " << e.line << "\n";
    ASSERT_EQ(error, e.error)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument error at index "
      << __index
      << " in history of event PWM_WriteError\n"
      << "  Expected: " << error << "\n"
      << "  Actual:   " << e.error << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: PWM_ReadError
  // ----------------------------------------------------------------------

  void BlspPwmDriverGTestBase ::
    assertEvents_PWM_ReadError_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_PWM_ReadError->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event PWM_ReadError\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_PWM_ReadError->size() << "\n";
  }

  void BlspPwmDriverGTestBase ::
    assertEvents_PWM_ReadError(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const I32 chip,
        const I32 line,
        const I32 error
    ) const
  {
    ASSERT_GT(this->eventHistory_PWM_ReadError->size(), __index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event PWM_ReadError\n"
      << "  Expected: Less than size of history ("
      << this->eventHistory_PWM_ReadError->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const EventEntry_PWM_ReadError& e =
      this->eventHistory_PWM_ReadError->at(__index);
    ASSERT_EQ(chip, e.chip)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument chip at index "
      << __index
      << " in history of event PWM_ReadError\n"
      << "  Expected: " << chip << "\n"
      << "  Actual:   " << e.chip << "\n";
    ASSERT_EQ(line, e.line)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument line at index "
      << __index
      << " in history of event PWM_ReadError\n"
      << "  Expected: " << line << "\n"
      << "  Actual:   " << e.line << "\n";
    ASSERT_EQ(error, e.error)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument error at index "
      << __index
      << " in history of event PWM_ReadError\n"
      << "  Expected: " << error << "\n"
      << "  Actual:   " << e.error << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: PWM_PortOpened
  // ----------------------------------------------------------------------

  void BlspPwmDriverGTestBase ::
    assertEvents_PWM_PortOpened_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_PWM_PortOpened->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event PWM_PortOpened\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_PWM_PortOpened->size() << "\n";
  }

  void BlspPwmDriverGTestBase ::
    assertEvents_PWM_PortOpened(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const I32 chip,
        const I32 line
    ) const
  {
    ASSERT_GT(this->eventHistory_PWM_PortOpened->size(), __index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event PWM_PortOpened\n"
      << "  Expected: Less than size of history ("
      << this->eventHistory_PWM_PortOpened->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const EventEntry_PWM_PortOpened& e =
      this->eventHistory_PWM_PortOpened->at(__index);
    ASSERT_EQ(chip, e.chip)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument chip at index "
      << __index
      << " in history of event PWM_PortOpened\n"
      << "  Expected: " << chip << "\n"
      << "  Actual:   " << e.chip << "\n";
    ASSERT_EQ(line, e.line)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument line at index "
      << __index
      << " in history of event PWM_PortOpened\n"
      << "  Expected: " << line << "\n"
      << "  Actual:   " << e.line << "\n";
  }

} // end namespace SnapdragonFlight
