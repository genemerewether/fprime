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
  // Channel: LLTime
  // ----------------------------------------------------------------------

  void TimeSyncOffsetGTestBase ::
    assertTlm_LLTime_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_LLTime->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel LLTime\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_LLTime->size() << "\n";
  }

  void TimeSyncOffsetGTestBase ::
    assertTlm_LLTime(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const F64& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_LLTime->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel LLTime\n"
      << "  Expected: Less than size of history ("
      << this->tlmHistory_LLTime->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_LLTime& e =
      this->tlmHistory_LLTime->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel LLTime\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: HLTime
  // ----------------------------------------------------------------------

  void TimeSyncOffsetGTestBase ::
    assertTlm_HLTime_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_HLTime->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel HLTime\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_HLTime->size() << "\n";
  }

  void TimeSyncOffsetGTestBase ::
    assertTlm_HLTime(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const F64& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_HLTime->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel HLTime\n"
      << "  Expected: Less than size of history ("
      << this->tlmHistory_HLTime->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_HLTime& e =
      this->tlmHistory_HLTime->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel HLTime\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Events
  // ----------------------------------------------------------------------

  void TimeSyncOffsetGTestBase ::
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
  // Event: SchedIn_Timeout
  // ----------------------------------------------------------------------

  void TimeSyncOffsetGTestBase ::
    assertEvents_SchedIn_Timeout_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_SchedIn_Timeout->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event SchedIn_Timeout\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_SchedIn_Timeout->size() << "\n";
  }

  void TimeSyncOffsetGTestBase ::
    assertEvents_SchedIn_Timeout(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const U8 sched_timeout
    ) const
  {
    ASSERT_GT(this->eventHistory_SchedIn_Timeout->size(), __index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event SchedIn_Timeout\n"
      << "  Expected: Less than size of history ("
      << this->eventHistory_SchedIn_Timeout->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const EventEntry_SchedIn_Timeout& e =
      this->eventHistory_SchedIn_Timeout->at(__index);
    ASSERT_EQ(sched_timeout, e.sched_timeout)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument sched_timeout at index "
      << __index
      << " in history of event SchedIn_Timeout\n"
      << "  Expected: " << sched_timeout << "\n"
      << "  Actual:   " << e.sched_timeout << "\n";
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
  // From port: ClockTimes
  // ----------------------------------------------------------------------

  void TimeSyncOffsetGTestBase ::
    assert_from_ClockTimes_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->fromPortHistory_ClockTimes->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for from_ClockTimes\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->fromPortHistory_ClockTimes->size() << "\n";
  }

} // end namespace Svc
