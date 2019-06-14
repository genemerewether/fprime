// ======================================================================
// \title  TimeSyncOffset/test/ut/GTestBase.cpp
// \author Auto-generated
// \brief  cpp file for TimeSyncOffset component Google Test harness base class
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged.
//
// ======================================================================

#include "GTestBase.hpp"

namespace Svc {

  // ----------------------------------------------------------------------
  // Construction and destruction
  // ----------------------------------------------------------------------

  TimeSyncOffsetGTestBase ::
    TimeSyncOffsetGTestBase(
#if FW_OBJECT_NAMES == 1
        const char *const compName,
        const U32 maxHistorySize
#else
        const U32 maxHistorySize
#endif
    ) :
        TimeSyncOffsetTesterBase (
#if FW_OBJECT_NAMES == 1
            compName,
#endif
            maxHistorySize
        )
  {

  }

  TimeSyncOffsetGTestBase ::
    ~TimeSyncOffsetGTestBase(void)
  {

  }

  // ----------------------------------------------------------------------
  // Telemetry
  // ----------------------------------------------------------------------

  void TimeSyncOffsetGTestBase ::
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
  // Channel: LLOffset
  // ----------------------------------------------------------------------

  void TimeSyncOffsetGTestBase ::
    assertTlm_LLOffset_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_LLOffset->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel LLOffset\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_LLOffset->size() << "\n";
  }

  void TimeSyncOffsetGTestBase ::
    assertTlm_LLOffset(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const F64& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_LLOffset->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel LLOffset\n"
      << "  Expected: Less than size of history ("
      << this->tlmHistory_LLOffset->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_LLOffset& e =
      this->tlmHistory_LLOffset->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel LLOffset\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // From ports
  // ----------------------------------------------------------------------

  void TimeSyncOffsetGTestBase ::
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
  // From port: GPIOPulse
  // ----------------------------------------------------------------------

  void TimeSyncOffsetGTestBase ::
    assert_from_GPIOPulse_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->fromPortHistory_GPIOPulse->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for from_GPIOPulse\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->fromPortHistory_GPIOPulse->size() << "\n";
  }

  // ----------------------------------------------------------------------
  // From port: Offset
  // ----------------------------------------------------------------------

  void TimeSyncOffsetGTestBase ::
    assert_from_Offset_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->fromPortHistory_Offset->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for from_Offset\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->fromPortHistory_Offset->size() << "\n";
  }

} // end namespace Svc
