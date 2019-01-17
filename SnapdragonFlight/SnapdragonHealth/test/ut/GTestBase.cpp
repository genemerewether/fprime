// ======================================================================
// \title  SnapdragonHealth/test/ut/GTestBase.cpp
// \author Auto-generated
// \brief  cpp file for SnapdragonHealth component Google Test harness base class
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

#include "GTestBase.hpp"

namespace SnapdragonFlight {

  // ----------------------------------------------------------------------
  // Construction and destruction
  // ----------------------------------------------------------------------

  SnapdragonHealthGTestBase ::
    SnapdragonHealthGTestBase(
#if FW_OBJECT_NAMES == 1
        const char *const compName,
        const U32 maxHistorySize
#else
        const U32 maxHistorySize
#endif
    ) : 
        SnapdragonHealthTesterBase (
#if FW_OBJECT_NAMES == 1
            compName,
#endif
            maxHistorySize
        )
  {

  }

  SnapdragonHealthGTestBase ::
    ~SnapdragonHealthGTestBase(void)
  {

  }

  // ----------------------------------------------------------------------
  // Commands
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertCmdResponse_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ((unsigned long) size, this->cmdResponseHistory->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of command response history\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->cmdResponseHistory->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertCmdResponse(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const FwOpcodeType opCode,
        const U32 cmdSeq,
        const Fw::CommandResponse response
    )
    const
  {
    ASSERT_LT(__index, this->cmdResponseHistory->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into command response history\n"
      << "  Expected: Less than size of command response history (" 
      << this->cmdResponseHistory->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const CmdResponse& e = this->cmdResponseHistory->at(__index);
    ASSERT_EQ(opCode, e.opCode)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Opcode at index "
      << __index
      << " in command response history\n"
      << "  Expected: " << opCode << "\n"
      << "  Actual:   " << e.opCode << "\n";
    ASSERT_EQ(cmdSeq, e.cmdSeq)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Command sequence number at index "
      << __index
      << " in command response history\n"
      << "  Expected: " << cmdSeq << "\n"
      << "  Actual:   " << e.cmdSeq << "\n";
    ASSERT_EQ(response, e.response)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Command response at index "
      << __index
      << " in command resopnse history\n"
      << "  Expected: " << response << "\n"
      << "  Actual:   " << e.response << "\n";
  }

  // ----------------------------------------------------------------------
  // Telemetry
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
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
  // Channel: SnapTempZone0DspCpuStat
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone0DspCpuStat_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_SnapTempZone0DspCpuStat->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel SnapTempZone0DspCpuStat\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_SnapTempZone0DspCpuStat->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone0DspCpuStat(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const bool& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_SnapTempZone0DspCpuStat->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel SnapTempZone0DspCpuStat\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_SnapTempZone0DspCpuStat->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_SnapTempZone0DspCpuStat& e =
      this->tlmHistory_SnapTempZone0DspCpuStat->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel SnapTempZone0DspCpuStat\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: SnapTempZone0DspCpuTemp
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone0DspCpuTemp_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_SnapTempZone0DspCpuTemp->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel SnapTempZone0DspCpuTemp\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_SnapTempZone0DspCpuTemp->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone0DspCpuTemp(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const I32& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_SnapTempZone0DspCpuTemp->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel SnapTempZone0DspCpuTemp\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_SnapTempZone0DspCpuTemp->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_SnapTempZone0DspCpuTemp& e =
      this->tlmHistory_SnapTempZone0DspCpuTemp->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel SnapTempZone0DspCpuTemp\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: SnapTempZone1DspModemStat
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone1DspModemStat_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_SnapTempZone1DspModemStat->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel SnapTempZone1DspModemStat\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_SnapTempZone1DspModemStat->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone1DspModemStat(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const bool& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_SnapTempZone1DspModemStat->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel SnapTempZone1DspModemStat\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_SnapTempZone1DspModemStat->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_SnapTempZone1DspModemStat& e =
      this->tlmHistory_SnapTempZone1DspModemStat->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel SnapTempZone1DspModemStat\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: SnapTempZone1DspModemTemp
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone1DspModemTemp_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_SnapTempZone1DspModemTemp->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel SnapTempZone1DspModemTemp\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_SnapTempZone1DspModemTemp->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone1DspModemTemp(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const I32& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_SnapTempZone1DspModemTemp->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel SnapTempZone1DspModemTemp\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_SnapTempZone1DspModemTemp->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_SnapTempZone1DspModemTemp& e =
      this->tlmHistory_SnapTempZone1DspModemTemp->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel SnapTempZone1DspModemTemp\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: SnapTempZone2ModemStat
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone2ModemStat_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_SnapTempZone2ModemStat->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel SnapTempZone2ModemStat\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_SnapTempZone2ModemStat->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone2ModemStat(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const bool& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_SnapTempZone2ModemStat->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel SnapTempZone2ModemStat\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_SnapTempZone2ModemStat->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_SnapTempZone2ModemStat& e =
      this->tlmHistory_SnapTempZone2ModemStat->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel SnapTempZone2ModemStat\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: SnapTempZone2ModemTemp
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone2ModemTemp_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_SnapTempZone2ModemTemp->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel SnapTempZone2ModemTemp\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_SnapTempZone2ModemTemp->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone2ModemTemp(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const I32& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_SnapTempZone2ModemTemp->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel SnapTempZone2ModemTemp\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_SnapTempZone2ModemTemp->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_SnapTempZone2ModemTemp& e =
      this->tlmHistory_SnapTempZone2ModemTemp->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel SnapTempZone2ModemTemp\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: SnapTempZone3VideoStat
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone3VideoStat_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_SnapTempZone3VideoStat->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel SnapTempZone3VideoStat\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_SnapTempZone3VideoStat->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone3VideoStat(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const bool& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_SnapTempZone3VideoStat->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel SnapTempZone3VideoStat\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_SnapTempZone3VideoStat->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_SnapTempZone3VideoStat& e =
      this->tlmHistory_SnapTempZone3VideoStat->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel SnapTempZone3VideoStat\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: SnapTempZone3VideoTemp
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone3VideoTemp_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_SnapTempZone3VideoTemp->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel SnapTempZone3VideoTemp\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_SnapTempZone3VideoTemp->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone3VideoTemp(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const I32& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_SnapTempZone3VideoTemp->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel SnapTempZone3VideoTemp\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_SnapTempZone3VideoTemp->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_SnapTempZone3VideoTemp& e =
      this->tlmHistory_SnapTempZone3VideoTemp->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel SnapTempZone3VideoTemp\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: SnapTempZone4WlanCpuStat
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone4WlanCpuStat_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_SnapTempZone4WlanCpuStat->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel SnapTempZone4WlanCpuStat\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_SnapTempZone4WlanCpuStat->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone4WlanCpuStat(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const bool& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_SnapTempZone4WlanCpuStat->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel SnapTempZone4WlanCpuStat\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_SnapTempZone4WlanCpuStat->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_SnapTempZone4WlanCpuStat& e =
      this->tlmHistory_SnapTempZone4WlanCpuStat->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel SnapTempZone4WlanCpuStat\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: SnapTempZone4WlanCpuTemp
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone4WlanCpuTemp_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_SnapTempZone4WlanCpuTemp->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel SnapTempZone4WlanCpuTemp\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_SnapTempZone4WlanCpuTemp->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone4WlanCpuTemp(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const I32& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_SnapTempZone4WlanCpuTemp->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel SnapTempZone4WlanCpuTemp\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_SnapTempZone4WlanCpuTemp->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_SnapTempZone4WlanCpuTemp& e =
      this->tlmHistory_SnapTempZone4WlanCpuTemp->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel SnapTempZone4WlanCpuTemp\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: SnapTempZone5Cpu0Stat
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone5Cpu0Stat_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_SnapTempZone5Cpu0Stat->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel SnapTempZone5Cpu0Stat\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_SnapTempZone5Cpu0Stat->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone5Cpu0Stat(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const bool& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_SnapTempZone5Cpu0Stat->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel SnapTempZone5Cpu0Stat\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_SnapTempZone5Cpu0Stat->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_SnapTempZone5Cpu0Stat& e =
      this->tlmHistory_SnapTempZone5Cpu0Stat->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel SnapTempZone5Cpu0Stat\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: SnapTempZone5Cpu0Temp
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone5Cpu0Temp_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_SnapTempZone5Cpu0Temp->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel SnapTempZone5Cpu0Temp\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_SnapTempZone5Cpu0Temp->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone5Cpu0Temp(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const I32& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_SnapTempZone5Cpu0Temp->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel SnapTempZone5Cpu0Temp\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_SnapTempZone5Cpu0Temp->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_SnapTempZone5Cpu0Temp& e =
      this->tlmHistory_SnapTempZone5Cpu0Temp->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel SnapTempZone5Cpu0Temp\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: SnapTempZone6Cpu1Stat
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone6Cpu1Stat_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_SnapTempZone6Cpu1Stat->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel SnapTempZone6Cpu1Stat\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_SnapTempZone6Cpu1Stat->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone6Cpu1Stat(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const bool& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_SnapTempZone6Cpu1Stat->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel SnapTempZone6Cpu1Stat\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_SnapTempZone6Cpu1Stat->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_SnapTempZone6Cpu1Stat& e =
      this->tlmHistory_SnapTempZone6Cpu1Stat->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel SnapTempZone6Cpu1Stat\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: SnapTempZone6Cpu1Temp
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone6Cpu1Temp_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_SnapTempZone6Cpu1Temp->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel SnapTempZone6Cpu1Temp\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_SnapTempZone6Cpu1Temp->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone6Cpu1Temp(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const I32& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_SnapTempZone6Cpu1Temp->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel SnapTempZone6Cpu1Temp\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_SnapTempZone6Cpu1Temp->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_SnapTempZone6Cpu1Temp& e =
      this->tlmHistory_SnapTempZone6Cpu1Temp->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel SnapTempZone6Cpu1Temp\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: SnapTempZone7Cpu2Stat
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone7Cpu2Stat_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_SnapTempZone7Cpu2Stat->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel SnapTempZone7Cpu2Stat\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_SnapTempZone7Cpu2Stat->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone7Cpu2Stat(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const bool& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_SnapTempZone7Cpu2Stat->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel SnapTempZone7Cpu2Stat\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_SnapTempZone7Cpu2Stat->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_SnapTempZone7Cpu2Stat& e =
      this->tlmHistory_SnapTempZone7Cpu2Stat->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel SnapTempZone7Cpu2Stat\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: SnapTempZone7Cpu2Temp
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone7Cpu2Temp_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_SnapTempZone7Cpu2Temp->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel SnapTempZone7Cpu2Temp\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_SnapTempZone7Cpu2Temp->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone7Cpu2Temp(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const I32& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_SnapTempZone7Cpu2Temp->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel SnapTempZone7Cpu2Temp\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_SnapTempZone7Cpu2Temp->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_SnapTempZone7Cpu2Temp& e =
      this->tlmHistory_SnapTempZone7Cpu2Temp->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel SnapTempZone7Cpu2Temp\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: SnapTempZone8Cpu3Stat
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone8Cpu3Stat_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_SnapTempZone8Cpu3Stat->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel SnapTempZone8Cpu3Stat\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_SnapTempZone8Cpu3Stat->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone8Cpu3Stat(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const bool& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_SnapTempZone8Cpu3Stat->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel SnapTempZone8Cpu3Stat\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_SnapTempZone8Cpu3Stat->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_SnapTempZone8Cpu3Stat& e =
      this->tlmHistory_SnapTempZone8Cpu3Stat->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel SnapTempZone8Cpu3Stat\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: SnapTempZone8Cpu3Temp
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone8Cpu3Temp_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_SnapTempZone8Cpu3Temp->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel SnapTempZone8Cpu3Temp\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_SnapTempZone8Cpu3Temp->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone8Cpu3Temp(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const I32& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_SnapTempZone8Cpu3Temp->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel SnapTempZone8Cpu3Temp\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_SnapTempZone8Cpu3Temp->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_SnapTempZone8Cpu3Temp& e =
      this->tlmHistory_SnapTempZone8Cpu3Temp->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel SnapTempZone8Cpu3Temp\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: SnapTempZone9GpuAStat
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone9GpuAStat_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_SnapTempZone9GpuAStat->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel SnapTempZone9GpuAStat\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_SnapTempZone9GpuAStat->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone9GpuAStat(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const bool& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_SnapTempZone9GpuAStat->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel SnapTempZone9GpuAStat\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_SnapTempZone9GpuAStat->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_SnapTempZone9GpuAStat& e =
      this->tlmHistory_SnapTempZone9GpuAStat->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel SnapTempZone9GpuAStat\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: SnapTempZone9GpuATemp
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone9GpuATemp_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_SnapTempZone9GpuATemp->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel SnapTempZone9GpuATemp\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_SnapTempZone9GpuATemp->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone9GpuATemp(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const I32& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_SnapTempZone9GpuATemp->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel SnapTempZone9GpuATemp\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_SnapTempZone9GpuATemp->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_SnapTempZone9GpuATemp& e =
      this->tlmHistory_SnapTempZone9GpuATemp->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel SnapTempZone9GpuATemp\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: SnapTempZone10GpuBStat
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone10GpuBStat_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_SnapTempZone10GpuBStat->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel SnapTempZone10GpuBStat\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_SnapTempZone10GpuBStat->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone10GpuBStat(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const bool& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_SnapTempZone10GpuBStat->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel SnapTempZone10GpuBStat\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_SnapTempZone10GpuBStat->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_SnapTempZone10GpuBStat& e =
      this->tlmHistory_SnapTempZone10GpuBStat->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel SnapTempZone10GpuBStat\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: SnapTempZone10GpuBTemp
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone10GpuBTemp_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_SnapTempZone10GpuBTemp->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel SnapTempZone10GpuBTemp\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_SnapTempZone10GpuBTemp->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone10GpuBTemp(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const I32& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_SnapTempZone10GpuBTemp->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel SnapTempZone10GpuBTemp\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_SnapTempZone10GpuBTemp->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_SnapTempZone10GpuBTemp& e =
      this->tlmHistory_SnapTempZone10GpuBTemp->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel SnapTempZone10GpuBTemp\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: SnapdragonTotalCpuUtil
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapdragonTotalCpuUtil_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_SnapdragonTotalCpuUtil->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel SnapdragonTotalCpuUtil\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_SnapdragonTotalCpuUtil->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapdragonTotalCpuUtil(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const F32& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_SnapdragonTotalCpuUtil->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel SnapdragonTotalCpuUtil\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_SnapdragonTotalCpuUtil->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_SnapdragonTotalCpuUtil& e =
      this->tlmHistory_SnapdragonTotalCpuUtil->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel SnapdragonTotalCpuUtil\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: SnapdragonCpu0Util
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapdragonCpu0Util_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_SnapdragonCpu0Util->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel SnapdragonCpu0Util\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_SnapdragonCpu0Util->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapdragonCpu0Util(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const F32& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_SnapdragonCpu0Util->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel SnapdragonCpu0Util\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_SnapdragonCpu0Util->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_SnapdragonCpu0Util& e =
      this->tlmHistory_SnapdragonCpu0Util->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel SnapdragonCpu0Util\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: SnapdragonCpu1Util
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapdragonCpu1Util_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_SnapdragonCpu1Util->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel SnapdragonCpu1Util\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_SnapdragonCpu1Util->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapdragonCpu1Util(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const F32& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_SnapdragonCpu1Util->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel SnapdragonCpu1Util\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_SnapdragonCpu1Util->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_SnapdragonCpu1Util& e =
      this->tlmHistory_SnapdragonCpu1Util->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel SnapdragonCpu1Util\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: SnapdragonCpu2Util
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapdragonCpu2Util_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_SnapdragonCpu2Util->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel SnapdragonCpu2Util\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_SnapdragonCpu2Util->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapdragonCpu2Util(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const F32& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_SnapdragonCpu2Util->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel SnapdragonCpu2Util\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_SnapdragonCpu2Util->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_SnapdragonCpu2Util& e =
      this->tlmHistory_SnapdragonCpu2Util->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel SnapdragonCpu2Util\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: SnapdragonCpu3Util
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapdragonCpu3Util_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_SnapdragonCpu3Util->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel SnapdragonCpu3Util\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_SnapdragonCpu3Util->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapdragonCpu3Util(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const F32& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_SnapdragonCpu3Util->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel SnapdragonCpu3Util\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_SnapdragonCpu3Util->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_SnapdragonCpu3Util& e =
      this->tlmHistory_SnapdragonCpu3Util->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel SnapdragonCpu3Util\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: SnapdragonCpu0Freq
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapdragonCpu0Freq_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_SnapdragonCpu0Freq->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel SnapdragonCpu0Freq\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_SnapdragonCpu0Freq->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapdragonCpu0Freq(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const U32& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_SnapdragonCpu0Freq->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel SnapdragonCpu0Freq\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_SnapdragonCpu0Freq->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_SnapdragonCpu0Freq& e =
      this->tlmHistory_SnapdragonCpu0Freq->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel SnapdragonCpu0Freq\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: SnapdragonCpu1Freq
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapdragonCpu1Freq_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_SnapdragonCpu1Freq->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel SnapdragonCpu1Freq\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_SnapdragonCpu1Freq->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapdragonCpu1Freq(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const U32& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_SnapdragonCpu1Freq->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel SnapdragonCpu1Freq\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_SnapdragonCpu1Freq->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_SnapdragonCpu1Freq& e =
      this->tlmHistory_SnapdragonCpu1Freq->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel SnapdragonCpu1Freq\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: SnapdragonCpu2Freq
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapdragonCpu2Freq_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_SnapdragonCpu2Freq->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel SnapdragonCpu2Freq\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_SnapdragonCpu2Freq->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapdragonCpu2Freq(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const U32& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_SnapdragonCpu2Freq->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel SnapdragonCpu2Freq\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_SnapdragonCpu2Freq->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_SnapdragonCpu2Freq& e =
      this->tlmHistory_SnapdragonCpu2Freq->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel SnapdragonCpu2Freq\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: SnapdragonCpu3Freq
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapdragonCpu3Freq_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_SnapdragonCpu3Freq->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel SnapdragonCpu3Freq\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_SnapdragonCpu3Freq->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapdragonCpu3Freq(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const U32& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_SnapdragonCpu3Freq->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel SnapdragonCpu3Freq\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_SnapdragonCpu3Freq->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_SnapdragonCpu3Freq& e =
      this->tlmHistory_SnapdragonCpu3Freq->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel SnapdragonCpu3Freq\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: SnapdragonPowerSaver
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapdragonPowerSaver_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_SnapdragonPowerSaver->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel SnapdragonPowerSaver\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_SnapdragonPowerSaver->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapdragonPowerSaver(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const SnapdragonHealthComponentBase::PowerSaverModeTlm& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_SnapdragonPowerSaver->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel SnapdragonPowerSaver\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_SnapdragonPowerSaver->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_SnapdragonPowerSaver& e =
      this->tlmHistory_SnapdragonPowerSaver->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel SnapdragonPowerSaver\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: SnapdragonBootCount
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapdragonBootCount_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_SnapdragonBootCount->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel SnapdragonBootCount\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_SnapdragonBootCount->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapdragonBootCount(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const U32& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_SnapdragonBootCount->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel SnapdragonBootCount\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_SnapdragonBootCount->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_SnapdragonBootCount& e =
      this->tlmHistory_SnapdragonBootCount->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel SnapdragonBootCount\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: SnapTempZone11PM8841Stat
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone11PM8841Stat_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_SnapTempZone11PM8841Stat->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel SnapTempZone11PM8841Stat\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_SnapTempZone11PM8841Stat->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone11PM8841Stat(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const bool& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_SnapTempZone11PM8841Stat->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel SnapTempZone11PM8841Stat\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_SnapTempZone11PM8841Stat->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_SnapTempZone11PM8841Stat& e =
      this->tlmHistory_SnapTempZone11PM8841Stat->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel SnapTempZone11PM8841Stat\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: SnapTempZone11PM8841Temp
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone11PM8841Temp_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_SnapTempZone11PM8841Temp->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel SnapTempZone11PM8841Temp\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_SnapTempZone11PM8841Temp->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone11PM8841Temp(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const F32& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_SnapTempZone11PM8841Temp->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel SnapTempZone11PM8841Temp\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_SnapTempZone11PM8841Temp->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_SnapTempZone11PM8841Temp& e =
      this->tlmHistory_SnapTempZone11PM8841Temp->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel SnapTempZone11PM8841Temp\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: SnapTempZone12PM8941Stat
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone12PM8941Stat_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_SnapTempZone12PM8941Stat->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel SnapTempZone12PM8941Stat\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_SnapTempZone12PM8941Stat->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone12PM8941Stat(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const bool& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_SnapTempZone12PM8941Stat->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel SnapTempZone12PM8941Stat\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_SnapTempZone12PM8941Stat->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_SnapTempZone12PM8941Stat& e =
      this->tlmHistory_SnapTempZone12PM8941Stat->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel SnapTempZone12PM8941Stat\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: SnapTempZone12PM8941Temp
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone12PM8941Temp_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_SnapTempZone12PM8941Temp->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel SnapTempZone12PM8941Temp\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_SnapTempZone12PM8941Temp->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone12PM8941Temp(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const F32& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_SnapTempZone12PM8941Temp->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel SnapTempZone12PM8941Temp\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_SnapTempZone12PM8941Temp->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_SnapTempZone12PM8941Temp& e =
      this->tlmHistory_SnapTempZone12PM8941Temp->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel SnapTempZone12PM8941Temp\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: SnapTempZone13PA0Stat
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone13PA0Stat_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_SnapTempZone13PA0Stat->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel SnapTempZone13PA0Stat\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_SnapTempZone13PA0Stat->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone13PA0Stat(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const bool& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_SnapTempZone13PA0Stat->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel SnapTempZone13PA0Stat\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_SnapTempZone13PA0Stat->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_SnapTempZone13PA0Stat& e =
      this->tlmHistory_SnapTempZone13PA0Stat->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel SnapTempZone13PA0Stat\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: SnapTempZone13PA0Temp
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone13PA0Temp_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_SnapTempZone13PA0Temp->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel SnapTempZone13PA0Temp\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_SnapTempZone13PA0Temp->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone13PA0Temp(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const I32& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_SnapTempZone13PA0Temp->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel SnapTempZone13PA0Temp\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_SnapTempZone13PA0Temp->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_SnapTempZone13PA0Temp& e =
      this->tlmHistory_SnapTempZone13PA0Temp->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel SnapTempZone13PA0Temp\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: SnapTempZone14PA1Stat
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone14PA1Stat_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_SnapTempZone14PA1Stat->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel SnapTempZone14PA1Stat\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_SnapTempZone14PA1Stat->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone14PA1Stat(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const bool& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_SnapTempZone14PA1Stat->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel SnapTempZone14PA1Stat\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_SnapTempZone14PA1Stat->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_SnapTempZone14PA1Stat& e =
      this->tlmHistory_SnapTempZone14PA1Stat->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel SnapTempZone14PA1Stat\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: SnapTempZone14PA1Temp
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone14PA1Temp_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_SnapTempZone14PA1Temp->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel SnapTempZone14PA1Temp\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_SnapTempZone14PA1Temp->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone14PA1Temp(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const I32& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_SnapTempZone14PA1Temp->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel SnapTempZone14PA1Temp\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_SnapTempZone14PA1Temp->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_SnapTempZone14PA1Temp& e =
      this->tlmHistory_SnapTempZone14PA1Temp->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel SnapTempZone14PA1Temp\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: SnapTempZone15EmmcStat
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone15EmmcStat_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_SnapTempZone15EmmcStat->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel SnapTempZone15EmmcStat\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_SnapTempZone15EmmcStat->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone15EmmcStat(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const bool& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_SnapTempZone15EmmcStat->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel SnapTempZone15EmmcStat\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_SnapTempZone15EmmcStat->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_SnapTempZone15EmmcStat& e =
      this->tlmHistory_SnapTempZone15EmmcStat->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel SnapTempZone15EmmcStat\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: SnapTempZone15EmmcTemp
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone15EmmcTemp_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_SnapTempZone15EmmcTemp->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel SnapTempZone15EmmcTemp\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_SnapTempZone15EmmcTemp->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone15EmmcTemp(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const I32& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_SnapTempZone15EmmcTemp->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel SnapTempZone15EmmcTemp\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_SnapTempZone15EmmcTemp->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_SnapTempZone15EmmcTemp& e =
      this->tlmHistory_SnapTempZone15EmmcTemp->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel SnapTempZone15EmmcTemp\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: SnapTempZone16MsmStat
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone16MsmStat_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_SnapTempZone16MsmStat->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel SnapTempZone16MsmStat\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_SnapTempZone16MsmStat->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone16MsmStat(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const bool& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_SnapTempZone16MsmStat->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel SnapTempZone16MsmStat\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_SnapTempZone16MsmStat->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_SnapTempZone16MsmStat& e =
      this->tlmHistory_SnapTempZone16MsmStat->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel SnapTempZone16MsmStat\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: SnapTempZone16MsmTemp
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone16MsmTemp_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_SnapTempZone16MsmTemp->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel SnapTempZone16MsmTemp\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_SnapTempZone16MsmTemp->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertTlm_SnapTempZone16MsmTemp(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const I32& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_SnapTempZone16MsmTemp->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel SnapTempZone16MsmTemp\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_SnapTempZone16MsmTemp->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_SnapTempZone16MsmTemp& e =
      this->tlmHistory_SnapTempZone16MsmTemp->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel SnapTempZone16MsmTemp\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: SH_FswVersion
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertTlm_SH_FswVersion_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_SH_FswVersion->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel SH_FswVersion\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_SH_FswVersion->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertTlm_SH_FswVersion(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const U32& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_SH_FswVersion->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel SH_FswVersion\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_SH_FswVersion->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_SH_FswVersion& e =
      this->tlmHistory_SH_FswVersion->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel SH_FswVersion\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Events
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
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
  // Event: SH_FileOpenError
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertEvents_SH_FileOpenError_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_SH_FileOpenError->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event SH_FileOpenError\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_SH_FileOpenError->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertEvents_SH_FileOpenError(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const I32 type,
        const I32 stat
    ) const
  {
    ASSERT_GT(this->eventHistory_SH_FileOpenError->size(), __index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event SH_FileOpenError\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_SH_FileOpenError->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const EventEntry_SH_FileOpenError& e =
      this->eventHistory_SH_FileOpenError->at(__index);
    ASSERT_EQ(type, e.type)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument type at index "
      << __index
      << " in history of event SH_FileOpenError\n"
      << "  Expected: " << type << "\n"
      << "  Actual:   " << e.type << "\n";
    ASSERT_EQ(stat, e.stat)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument stat at index "
      << __index
      << " in history of event SH_FileOpenError\n"
      << "  Expected: " << stat << "\n"
      << "  Actual:   " << e.stat << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: SH_FileReadError
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertEvents_SH_FileReadError_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_SH_FileReadError->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event SH_FileReadError\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_SH_FileReadError->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertEvents_SH_FileReadError(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const I32 type,
        const I32 stat
    ) const
  {
    ASSERT_GT(this->eventHistory_SH_FileReadError->size(), __index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event SH_FileReadError\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_SH_FileReadError->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const EventEntry_SH_FileReadError& e =
      this->eventHistory_SH_FileReadError->at(__index);
    ASSERT_EQ(type, e.type)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument type at index "
      << __index
      << " in history of event SH_FileReadError\n"
      << "  Expected: " << type << "\n"
      << "  Actual:   " << e.type << "\n";
    ASSERT_EQ(stat, e.stat)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument stat at index "
      << __index
      << " in history of event SH_FileReadError\n"
      << "  Expected: " << stat << "\n"
      << "  Actual:   " << e.stat << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: SH_FileWriteError
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertEvents_SH_FileWriteError_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_SH_FileWriteError->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event SH_FileWriteError\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_SH_FileWriteError->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertEvents_SH_FileWriteError(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const I32 type,
        const I32 stat
    ) const
  {
    ASSERT_GT(this->eventHistory_SH_FileWriteError->size(), __index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event SH_FileWriteError\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_SH_FileWriteError->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const EventEntry_SH_FileWriteError& e =
      this->eventHistory_SH_FileWriteError->at(__index);
    ASSERT_EQ(type, e.type)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument type at index "
      << __index
      << " in history of event SH_FileWriteError\n"
      << "  Expected: " << type << "\n"
      << "  Actual:   " << e.type << "\n";
    ASSERT_EQ(stat, e.stat)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument stat at index "
      << __index
      << " in history of event SH_FileWriteError\n"
      << "  Expected: " << stat << "\n"
      << "  Actual:   " << e.stat << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: SH_FswVersion
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertEvents_SH_FswVersion_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_SH_FswVersion->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event SH_FswVersion\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_SH_FswVersion->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertEvents_SH_FswVersion(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const U32 hash,
        const char *const branch
    ) const
  {
    ASSERT_GT(this->eventHistory_SH_FswVersion->size(), __index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event SH_FswVersion\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_SH_FswVersion->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const EventEntry_SH_FswVersion& e =
      this->eventHistory_SH_FswVersion->at(__index);
    ASSERT_EQ(hash, e.hash)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument hash at index "
      << __index
      << " in history of event SH_FswVersion\n"
      << "  Expected: " << hash << "\n"
      << "  Actual:   " << e.hash << "\n";
    ASSERT_STREQ(branch, e.branch.toChar())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument branch at index "
      << __index
      << " in history of event SH_FswVersion\n"
      << "  Expected: " << branch << "\n"
      << "  Actual:   " << e.branch.toChar() << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: SH_PowerSaver
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assertEvents_SH_PowerSaver_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_SH_PowerSaver->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event SH_PowerSaver\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_SH_PowerSaver->size() << "\n";
  }

  void SnapdragonHealthGTestBase ::
    assertEvents_SH_PowerSaver(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        SnapdragonHealthComponentBase::PowerSaverModeEvr Mode
    ) const
  {
    ASSERT_GT(this->eventHistory_SH_PowerSaver->size(), __index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event SH_PowerSaver\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_SH_PowerSaver->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const EventEntry_SH_PowerSaver& e =
      this->eventHistory_SH_PowerSaver->at(__index);
    ASSERT_EQ(Mode, e.Mode)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument Mode at index "
      << __index
      << " in history of event SH_PowerSaver\n"
      << "  Expected: " << Mode << "\n"
      << "  Actual:   " << e.Mode << "\n";
  }

  // ----------------------------------------------------------------------
  // From ports
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
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
  // From port: PowerReport
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assert_from_PowerReport_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->fromPortHistory_PowerReport->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for from_PowerReport\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->fromPortHistory_PowerReport->size() << "\n";
  }

  // ----------------------------------------------------------------------
  // From port: pingOut
  // ----------------------------------------------------------------------

  void SnapdragonHealthGTestBase ::
    assert_from_pingOut_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->fromPortHistory_pingOut->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for from_pingOut\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->fromPortHistory_pingOut->size() << "\n";
  }

} // end namespace SnapdragonFlight
