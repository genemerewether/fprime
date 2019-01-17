// ======================================================================
// \title  SnapdragonHealth/test/ut/GTestBase.hpp
// \author Auto-generated
// \brief  hpp file for SnapdragonHealth component Google Test harness base class
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

#ifndef SnapdragonHealth_GTEST_BASE_HPP
#define SnapdragonHealth_GTEST_BASE_HPP

#include "TesterBase.hpp"
#include "gtest/gtest.h"

// ----------------------------------------------------------------------
// Macros for command history assertions
// ----------------------------------------------------------------------

#define ASSERT_CMD_RESPONSE_SIZE(size) \
  this->assertCmdResponse_size(__FILE__, __LINE__, size)

#define ASSERT_CMD_RESPONSE(index, opCode, cmdSeq, response) \
  this->assertCmdResponse(__FILE__, __LINE__, index, opCode, cmdSeq, response)

// ----------------------------------------------------------------------
// Macros for telemetry history assertions
// ----------------------------------------------------------------------

#define ASSERT_TLM_SIZE(size) \
  this->assertTlm_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_SnapTempZone0DspCpuStat_SIZE(size) \
  this->assertTlm_SnapTempZone0DspCpuStat_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_SnapTempZone0DspCpuStat(index, value) \
  this->assertTlm_SnapTempZone0DspCpuStat(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_SnapTempZone0DspCpuTemp_SIZE(size) \
  this->assertTlm_SnapTempZone0DspCpuTemp_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_SnapTempZone0DspCpuTemp(index, value) \
  this->assertTlm_SnapTempZone0DspCpuTemp(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_SnapTempZone1DspModemStat_SIZE(size) \
  this->assertTlm_SnapTempZone1DspModemStat_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_SnapTempZone1DspModemStat(index, value) \
  this->assertTlm_SnapTempZone1DspModemStat(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_SnapTempZone1DspModemTemp_SIZE(size) \
  this->assertTlm_SnapTempZone1DspModemTemp_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_SnapTempZone1DspModemTemp(index, value) \
  this->assertTlm_SnapTempZone1DspModemTemp(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_SnapTempZone2ModemStat_SIZE(size) \
  this->assertTlm_SnapTempZone2ModemStat_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_SnapTempZone2ModemStat(index, value) \
  this->assertTlm_SnapTempZone2ModemStat(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_SnapTempZone2ModemTemp_SIZE(size) \
  this->assertTlm_SnapTempZone2ModemTemp_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_SnapTempZone2ModemTemp(index, value) \
  this->assertTlm_SnapTempZone2ModemTemp(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_SnapTempZone3VideoStat_SIZE(size) \
  this->assertTlm_SnapTempZone3VideoStat_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_SnapTempZone3VideoStat(index, value) \
  this->assertTlm_SnapTempZone3VideoStat(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_SnapTempZone3VideoTemp_SIZE(size) \
  this->assertTlm_SnapTempZone3VideoTemp_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_SnapTempZone3VideoTemp(index, value) \
  this->assertTlm_SnapTempZone3VideoTemp(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_SnapTempZone4WlanCpuStat_SIZE(size) \
  this->assertTlm_SnapTempZone4WlanCpuStat_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_SnapTempZone4WlanCpuStat(index, value) \
  this->assertTlm_SnapTempZone4WlanCpuStat(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_SnapTempZone4WlanCpuTemp_SIZE(size) \
  this->assertTlm_SnapTempZone4WlanCpuTemp_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_SnapTempZone4WlanCpuTemp(index, value) \
  this->assertTlm_SnapTempZone4WlanCpuTemp(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_SnapTempZone5Cpu0Stat_SIZE(size) \
  this->assertTlm_SnapTempZone5Cpu0Stat_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_SnapTempZone5Cpu0Stat(index, value) \
  this->assertTlm_SnapTempZone5Cpu0Stat(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_SnapTempZone5Cpu0Temp_SIZE(size) \
  this->assertTlm_SnapTempZone5Cpu0Temp_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_SnapTempZone5Cpu0Temp(index, value) \
  this->assertTlm_SnapTempZone5Cpu0Temp(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_SnapTempZone6Cpu1Stat_SIZE(size) \
  this->assertTlm_SnapTempZone6Cpu1Stat_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_SnapTempZone6Cpu1Stat(index, value) \
  this->assertTlm_SnapTempZone6Cpu1Stat(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_SnapTempZone6Cpu1Temp_SIZE(size) \
  this->assertTlm_SnapTempZone6Cpu1Temp_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_SnapTempZone6Cpu1Temp(index, value) \
  this->assertTlm_SnapTempZone6Cpu1Temp(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_SnapTempZone7Cpu2Stat_SIZE(size) \
  this->assertTlm_SnapTempZone7Cpu2Stat_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_SnapTempZone7Cpu2Stat(index, value) \
  this->assertTlm_SnapTempZone7Cpu2Stat(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_SnapTempZone7Cpu2Temp_SIZE(size) \
  this->assertTlm_SnapTempZone7Cpu2Temp_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_SnapTempZone7Cpu2Temp(index, value) \
  this->assertTlm_SnapTempZone7Cpu2Temp(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_SnapTempZone8Cpu3Stat_SIZE(size) \
  this->assertTlm_SnapTempZone8Cpu3Stat_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_SnapTempZone8Cpu3Stat(index, value) \
  this->assertTlm_SnapTempZone8Cpu3Stat(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_SnapTempZone8Cpu3Temp_SIZE(size) \
  this->assertTlm_SnapTempZone8Cpu3Temp_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_SnapTempZone8Cpu3Temp(index, value) \
  this->assertTlm_SnapTempZone8Cpu3Temp(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_SnapTempZone9GpuAStat_SIZE(size) \
  this->assertTlm_SnapTempZone9GpuAStat_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_SnapTempZone9GpuAStat(index, value) \
  this->assertTlm_SnapTempZone9GpuAStat(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_SnapTempZone9GpuATemp_SIZE(size) \
  this->assertTlm_SnapTempZone9GpuATemp_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_SnapTempZone9GpuATemp(index, value) \
  this->assertTlm_SnapTempZone9GpuATemp(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_SnapTempZone10GpuBStat_SIZE(size) \
  this->assertTlm_SnapTempZone10GpuBStat_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_SnapTempZone10GpuBStat(index, value) \
  this->assertTlm_SnapTempZone10GpuBStat(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_SnapTempZone10GpuBTemp_SIZE(size) \
  this->assertTlm_SnapTempZone10GpuBTemp_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_SnapTempZone10GpuBTemp(index, value) \
  this->assertTlm_SnapTempZone10GpuBTemp(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_SnapdragonTotalCpuUtil_SIZE(size) \
  this->assertTlm_SnapdragonTotalCpuUtil_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_SnapdragonTotalCpuUtil(index, value) \
  this->assertTlm_SnapdragonTotalCpuUtil(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_SnapdragonCpu0Util_SIZE(size) \
  this->assertTlm_SnapdragonCpu0Util_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_SnapdragonCpu0Util(index, value) \
  this->assertTlm_SnapdragonCpu0Util(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_SnapdragonCpu1Util_SIZE(size) \
  this->assertTlm_SnapdragonCpu1Util_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_SnapdragonCpu1Util(index, value) \
  this->assertTlm_SnapdragonCpu1Util(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_SnapdragonCpu2Util_SIZE(size) \
  this->assertTlm_SnapdragonCpu2Util_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_SnapdragonCpu2Util(index, value) \
  this->assertTlm_SnapdragonCpu2Util(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_SnapdragonCpu3Util_SIZE(size) \
  this->assertTlm_SnapdragonCpu3Util_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_SnapdragonCpu3Util(index, value) \
  this->assertTlm_SnapdragonCpu3Util(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_SnapdragonCpu0Freq_SIZE(size) \
  this->assertTlm_SnapdragonCpu0Freq_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_SnapdragonCpu0Freq(index, value) \
  this->assertTlm_SnapdragonCpu0Freq(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_SnapdragonCpu1Freq_SIZE(size) \
  this->assertTlm_SnapdragonCpu1Freq_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_SnapdragonCpu1Freq(index, value) \
  this->assertTlm_SnapdragonCpu1Freq(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_SnapdragonCpu2Freq_SIZE(size) \
  this->assertTlm_SnapdragonCpu2Freq_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_SnapdragonCpu2Freq(index, value) \
  this->assertTlm_SnapdragonCpu2Freq(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_SnapdragonCpu3Freq_SIZE(size) \
  this->assertTlm_SnapdragonCpu3Freq_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_SnapdragonCpu3Freq(index, value) \
  this->assertTlm_SnapdragonCpu3Freq(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_SnapdragonPowerSaver_SIZE(size) \
  this->assertTlm_SnapdragonPowerSaver_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_SnapdragonPowerSaver(index, value) \
  this->assertTlm_SnapdragonPowerSaver(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_SnapdragonBootCount_SIZE(size) \
  this->assertTlm_SnapdragonBootCount_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_SnapdragonBootCount(index, value) \
  this->assertTlm_SnapdragonBootCount(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_SnapTempZone11PM8841Stat_SIZE(size) \
  this->assertTlm_SnapTempZone11PM8841Stat_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_SnapTempZone11PM8841Stat(index, value) \
  this->assertTlm_SnapTempZone11PM8841Stat(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_SnapTempZone11PM8841Temp_SIZE(size) \
  this->assertTlm_SnapTempZone11PM8841Temp_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_SnapTempZone11PM8841Temp(index, value) \
  this->assertTlm_SnapTempZone11PM8841Temp(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_SnapTempZone12PM8941Stat_SIZE(size) \
  this->assertTlm_SnapTempZone12PM8941Stat_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_SnapTempZone12PM8941Stat(index, value) \
  this->assertTlm_SnapTempZone12PM8941Stat(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_SnapTempZone12PM8941Temp_SIZE(size) \
  this->assertTlm_SnapTempZone12PM8941Temp_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_SnapTempZone12PM8941Temp(index, value) \
  this->assertTlm_SnapTempZone12PM8941Temp(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_SnapTempZone13PA0Stat_SIZE(size) \
  this->assertTlm_SnapTempZone13PA0Stat_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_SnapTempZone13PA0Stat(index, value) \
  this->assertTlm_SnapTempZone13PA0Stat(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_SnapTempZone13PA0Temp_SIZE(size) \
  this->assertTlm_SnapTempZone13PA0Temp_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_SnapTempZone13PA0Temp(index, value) \
  this->assertTlm_SnapTempZone13PA0Temp(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_SnapTempZone14PA1Stat_SIZE(size) \
  this->assertTlm_SnapTempZone14PA1Stat_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_SnapTempZone14PA1Stat(index, value) \
  this->assertTlm_SnapTempZone14PA1Stat(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_SnapTempZone14PA1Temp_SIZE(size) \
  this->assertTlm_SnapTempZone14PA1Temp_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_SnapTempZone14PA1Temp(index, value) \
  this->assertTlm_SnapTempZone14PA1Temp(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_SnapTempZone15EmmcStat_SIZE(size) \
  this->assertTlm_SnapTempZone15EmmcStat_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_SnapTempZone15EmmcStat(index, value) \
  this->assertTlm_SnapTempZone15EmmcStat(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_SnapTempZone15EmmcTemp_SIZE(size) \
  this->assertTlm_SnapTempZone15EmmcTemp_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_SnapTempZone15EmmcTemp(index, value) \
  this->assertTlm_SnapTempZone15EmmcTemp(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_SnapTempZone16MsmStat_SIZE(size) \
  this->assertTlm_SnapTempZone16MsmStat_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_SnapTempZone16MsmStat(index, value) \
  this->assertTlm_SnapTempZone16MsmStat(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_SnapTempZone16MsmTemp_SIZE(size) \
  this->assertTlm_SnapTempZone16MsmTemp_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_SnapTempZone16MsmTemp(index, value) \
  this->assertTlm_SnapTempZone16MsmTemp(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_SH_FswVersion_SIZE(size) \
  this->assertTlm_SH_FswVersion_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_SH_FswVersion(index, value) \
  this->assertTlm_SH_FswVersion(__FILE__, __LINE__, index, value)

// ----------------------------------------------------------------------
// Macros for event history assertions 
// ----------------------------------------------------------------------

#define ASSERT_EVENTS_SIZE(size) \
  this->assertEvents_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_SH_FileOpenError_SIZE(size) \
  this->assertEvents_SH_FileOpenError_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_SH_FileOpenError(index, _type, _stat) \
  this->assertEvents_SH_FileOpenError(__FILE__, __LINE__, index, _type, _stat)

#define ASSERT_EVENTS_SH_FileReadError_SIZE(size) \
  this->assertEvents_SH_FileReadError_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_SH_FileReadError(index, _type, _stat) \
  this->assertEvents_SH_FileReadError(__FILE__, __LINE__, index, _type, _stat)

#define ASSERT_EVENTS_SH_FileWriteError_SIZE(size) \
  this->assertEvents_SH_FileWriteError_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_SH_FileWriteError(index, _type, _stat) \
  this->assertEvents_SH_FileWriteError(__FILE__, __LINE__, index, _type, _stat)

#define ASSERT_EVENTS_SH_FswVersion_SIZE(size) \
  this->assertEvents_SH_FswVersion_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_SH_FswVersion(index, _hash, _branch) \
  this->assertEvents_SH_FswVersion(__FILE__, __LINE__, index, _hash, _branch)

#define ASSERT_EVENTS_SH_PowerSaver_SIZE(size) \
  this->assertEvents_SH_PowerSaver_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_SH_PowerSaver(index, _Mode) \
  this->assertEvents_SH_PowerSaver(__FILE__, __LINE__, index, _Mode)

// ----------------------------------------------------------------------
// Macros for typed user from port history assertions
// ----------------------------------------------------------------------

#define ASSERT_FROM_PORT_HISTORY_SIZE(size) \
  this->assertFromPortHistory_size(__FILE__, __LINE__, size)

#define ASSERT_from_PowerReport_SIZE(size) \
  this->assert_from_PowerReport_size(__FILE__, __LINE__, size)

#define ASSERT_from_PowerReport(index, _entry, _status, _time, _val) \
  { \
    ASSERT_GT(this->fromPortHistory_PowerReport->size(), static_cast<U32>(index)) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Index into history of from_PowerReport\n" \
    << "  Expected: Less than size of history (" \
    << this->fromPortHistory_PowerReport->size() << ")\n" \
    << "  Actual:   " << index << "\n"; \
    const FromPortEntry_PowerReport& _e = \
      this->fromPortHistory_PowerReport->at(index); \
    ASSERT_EQ(_entry, _e.entry) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument entry at index " \
    << index \
    << " in history of from_PowerReport\n" \
    << "  Expected: " << _entry << "\n" \
    << "  Actual:   " << _e.entry << "\n"; \
    ASSERT_EQ(_status, _e.status) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument status at index " \
    << index \
    << " in history of from_PowerReport\n" \
    << "  Expected: " << _status << "\n" \
    << "  Actual:   " << _e.status << "\n"; \
    ASSERT_EQ(_time, _e.time) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument time at index " \
    << index \
    << " in history of from_PowerReport\n" \
    << "  Expected: " << _time << "\n" \
    << "  Actual:   " << _e.time << "\n"; \
    ASSERT_EQ(_val, _e.val) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument val at index " \
    << index \
    << " in history of from_PowerReport\n" \
    << "  Expected: " << _val << "\n" \
    << "  Actual:   " << _e.val << "\n"; \
  }

#define ASSERT_from_pingOut_SIZE(size) \
  this->assert_from_pingOut_size(__FILE__, __LINE__, size)

#define ASSERT_from_pingOut(index, _key) \
  { \
    ASSERT_GT(this->fromPortHistory_pingOut->size(), static_cast<U32>(index)) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Index into history of from_pingOut\n" \
    << "  Expected: Less than size of history (" \
    << this->fromPortHistory_pingOut->size() << ")\n" \
    << "  Actual:   " << index << "\n"; \
    const FromPortEntry_pingOut& _e = \
      this->fromPortHistory_pingOut->at(index); \
    ASSERT_EQ(_key, _e.key) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument key at index " \
    << index \
    << " in history of from_pingOut\n" \
    << "  Expected: " << _key << "\n" \
    << "  Actual:   " << _e.key << "\n"; \
  }

namespace SnapdragonFlight {

  //! \class SnapdragonHealthGTestBase
  //! \brief Auto-generated base class for SnapdragonHealth component Google Test harness
  //!
  class SnapdragonHealthGTestBase :
    public SnapdragonHealthTesterBase
  {

    protected:

      // ----------------------------------------------------------------------
      // Construction and destruction
      // ----------------------------------------------------------------------

      //! Construct object SnapdragonHealthGTestBase
      //!
      SnapdragonHealthGTestBase(
#if FW_OBJECT_NAMES == 1
          const char *const compName, /*!< The component name*/
          const U32 maxHistorySize /*!< The maximum size of each history*/
#else
          const U32 maxHistorySize /*!< The maximum size of each history*/
#endif
      );

      //! Destroy object SnapdragonHealthGTestBase
      //!
      virtual ~SnapdragonHealthGTestBase(void);

    protected:

      // ----------------------------------------------------------------------
      // Commands
      // ----------------------------------------------------------------------

      //! Assert size of command response history
      //!
      void assertCmdResponse_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      //! Assert command response in history at index
      //!
      void assertCmdResponse(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq, /*!< The command sequence number*/
          const Fw::CommandResponse response /*!< The command response*/
      ) const;

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
      // Channel: SnapTempZone0DspCpuStat
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_SnapTempZone0DspCpuStat_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_SnapTempZone0DspCpuStat(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const bool& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone0DspCpuTemp
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_SnapTempZone0DspCpuTemp_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_SnapTempZone0DspCpuTemp(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const I32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone1DspModemStat
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_SnapTempZone1DspModemStat_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_SnapTempZone1DspModemStat(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const bool& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone1DspModemTemp
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_SnapTempZone1DspModemTemp_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_SnapTempZone1DspModemTemp(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const I32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone2ModemStat
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_SnapTempZone2ModemStat_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_SnapTempZone2ModemStat(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const bool& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone2ModemTemp
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_SnapTempZone2ModemTemp_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_SnapTempZone2ModemTemp(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const I32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone3VideoStat
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_SnapTempZone3VideoStat_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_SnapTempZone3VideoStat(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const bool& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone3VideoTemp
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_SnapTempZone3VideoTemp_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_SnapTempZone3VideoTemp(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const I32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone4WlanCpuStat
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_SnapTempZone4WlanCpuStat_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_SnapTempZone4WlanCpuStat(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const bool& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone4WlanCpuTemp
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_SnapTempZone4WlanCpuTemp_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_SnapTempZone4WlanCpuTemp(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const I32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone5Cpu0Stat
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_SnapTempZone5Cpu0Stat_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_SnapTempZone5Cpu0Stat(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const bool& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone5Cpu0Temp
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_SnapTempZone5Cpu0Temp_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_SnapTempZone5Cpu0Temp(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const I32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone6Cpu1Stat
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_SnapTempZone6Cpu1Stat_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_SnapTempZone6Cpu1Stat(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const bool& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone6Cpu1Temp
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_SnapTempZone6Cpu1Temp_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_SnapTempZone6Cpu1Temp(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const I32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone7Cpu2Stat
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_SnapTempZone7Cpu2Stat_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_SnapTempZone7Cpu2Stat(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const bool& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone7Cpu2Temp
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_SnapTempZone7Cpu2Temp_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_SnapTempZone7Cpu2Temp(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const I32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone8Cpu3Stat
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_SnapTempZone8Cpu3Stat_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_SnapTempZone8Cpu3Stat(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const bool& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone8Cpu3Temp
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_SnapTempZone8Cpu3Temp_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_SnapTempZone8Cpu3Temp(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const I32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone9GpuAStat
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_SnapTempZone9GpuAStat_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_SnapTempZone9GpuAStat(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const bool& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone9GpuATemp
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_SnapTempZone9GpuATemp_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_SnapTempZone9GpuATemp(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const I32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone10GpuBStat
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_SnapTempZone10GpuBStat_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_SnapTempZone10GpuBStat(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const bool& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone10GpuBTemp
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_SnapTempZone10GpuBTemp_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_SnapTempZone10GpuBTemp(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const I32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapdragonTotalCpuUtil
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_SnapdragonTotalCpuUtil_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_SnapdragonTotalCpuUtil(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const F32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapdragonCpu0Util
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_SnapdragonCpu0Util_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_SnapdragonCpu0Util(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const F32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapdragonCpu1Util
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_SnapdragonCpu1Util_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_SnapdragonCpu1Util(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const F32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapdragonCpu2Util
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_SnapdragonCpu2Util_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_SnapdragonCpu2Util(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const F32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapdragonCpu3Util
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_SnapdragonCpu3Util_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_SnapdragonCpu3Util(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const F32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapdragonCpu0Freq
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_SnapdragonCpu0Freq_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_SnapdragonCpu0Freq(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const U32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapdragonCpu1Freq
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_SnapdragonCpu1Freq_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_SnapdragonCpu1Freq(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const U32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapdragonCpu2Freq
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_SnapdragonCpu2Freq_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_SnapdragonCpu2Freq(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const U32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapdragonCpu3Freq
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_SnapdragonCpu3Freq_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_SnapdragonCpu3Freq(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const U32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapdragonPowerSaver
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_SnapdragonPowerSaver_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_SnapdragonPowerSaver(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const SnapdragonHealthComponentBase::PowerSaverModeTlm& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapdragonBootCount
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_SnapdragonBootCount_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_SnapdragonBootCount(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const U32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone11PM8841Stat
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_SnapTempZone11PM8841Stat_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_SnapTempZone11PM8841Stat(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const bool& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone11PM8841Temp
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_SnapTempZone11PM8841Temp_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_SnapTempZone11PM8841Temp(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const F32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone12PM8941Stat
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_SnapTempZone12PM8941Stat_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_SnapTempZone12PM8941Stat(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const bool& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone12PM8941Temp
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_SnapTempZone12PM8941Temp_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_SnapTempZone12PM8941Temp(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const F32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone13PA0Stat
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_SnapTempZone13PA0Stat_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_SnapTempZone13PA0Stat(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const bool& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone13PA0Temp
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_SnapTempZone13PA0Temp_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_SnapTempZone13PA0Temp(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const I32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone14PA1Stat
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_SnapTempZone14PA1Stat_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_SnapTempZone14PA1Stat(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const bool& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone14PA1Temp
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_SnapTempZone14PA1Temp_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_SnapTempZone14PA1Temp(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const I32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone15EmmcStat
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_SnapTempZone15EmmcStat_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_SnapTempZone15EmmcStat(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const bool& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone15EmmcTemp
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_SnapTempZone15EmmcTemp_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_SnapTempZone15EmmcTemp(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const I32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone16MsmStat
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_SnapTempZone16MsmStat_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_SnapTempZone16MsmStat(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const bool& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone16MsmTemp
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_SnapTempZone16MsmTemp_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_SnapTempZone16MsmTemp(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const I32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SH_FswVersion
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_SH_FswVersion_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_SH_FswVersion(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
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
      // Event: SH_FileOpenError
      // ----------------------------------------------------------------------

      void assertEvents_SH_FileOpenError_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_SH_FileOpenError(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const I32 type, /*!< File type*/
          const I32 stat /*!< File error status*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: SH_FileReadError
      // ----------------------------------------------------------------------

      void assertEvents_SH_FileReadError_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_SH_FileReadError(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const I32 type, /*!< File type*/
          const I32 stat /*!< File error status*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: SH_FileWriteError
      // ----------------------------------------------------------------------

      void assertEvents_SH_FileWriteError_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_SH_FileWriteError(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const I32 type, /*!< File type*/
          const I32 stat /*!< File error status*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: SH_FswVersion
      // ----------------------------------------------------------------------

      void assertEvents_SH_FswVersion_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_SH_FswVersion(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const U32 hash, /*!< FSW GIT hash*/
          const char *const branch /*!< FSW GIT branch*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: SH_PowerSaver
      // ----------------------------------------------------------------------

      void assertEvents_SH_PowerSaver_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_SH_PowerSaver(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          SnapdragonHealthComponentBase::PowerSaverModeEvr Mode 
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
      // From port: PowerReport 
      // ----------------------------------------------------------------------

      void assert_from_PowerReport_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // From port: pingOut 
      // ----------------------------------------------------------------------

      void assert_from_pingOut_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

  };

} // end namespace SnapdragonFlight

#endif
