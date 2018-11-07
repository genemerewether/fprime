// ======================================================================
// \title  LLRouter/test/ut/GTestBase.cpp
// \author Auto-generated
// \brief  cpp file for LLRouter component Google Test harness base class
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

namespace HLProc {

  // ----------------------------------------------------------------------
  // Construction and destruction
  // ----------------------------------------------------------------------

  LLRouterGTestBase ::
    LLRouterGTestBase(
#if FW_OBJECT_NAMES == 1
        const char *const compName,
        const U32 maxHistorySize
#else
        const U32 maxHistorySize
#endif
    ) : 
        LLRouterTesterBase (
#if FW_OBJECT_NAMES == 1
            compName,
#endif
            maxHistorySize
        )
  {

  }

  LLRouterGTestBase ::
    ~LLRouterGTestBase(void)
  {

  }

  // ----------------------------------------------------------------------
  // Commands
  // ----------------------------------------------------------------------

  void LLRouterGTestBase ::
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

  void LLRouterGTestBase ::
    assertCmdResponse(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        const FwOpcodeType opCode,
        const U32 cmdSeq,
        const Fw::CommandResponse response
    )
    const
  {
    ASSERT_LT(index, this->cmdResponseHistory->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into command response history\n"
      << "  Expected: Less than size of command response history (" 
      << this->cmdResponseHistory->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const CmdResponse& e = this->cmdResponseHistory->at(index);
    ASSERT_EQ(opCode, e.opCode)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Opcode at index "
      << index
      << " in command response history\n"
      << "  Expected: " << opCode << "\n"
      << "  Actual:   " << e.opCode << "\n";
    ASSERT_EQ(cmdSeq, e.cmdSeq)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Command sequence number at index "
      << index
      << " in command response history\n"
      << "  Expected: " << cmdSeq << "\n"
      << "  Actual:   " << e.cmdSeq << "\n";
    ASSERT_EQ(response, e.response)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Command response at index "
      << index
      << " in command resopnse history\n"
      << "  Expected: " << response << "\n"
      << "  Actual:   " << e.response << "\n";
  }

  // ----------------------------------------------------------------------
  // Telemetry
  // ----------------------------------------------------------------------

  void LLRouterGTestBase ::
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
  // Channel: LLR_CrcErrors
  // ----------------------------------------------------------------------

  void LLRouterGTestBase ::
    assertTlm_LLR_CrcErrors_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_LLR_CrcErrors->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel LLR_CrcErrors\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_LLR_CrcErrors->size() << "\n";
  }

  void LLRouterGTestBase ::
    assertTlm_LLR_CrcErrors(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        const U32& val
    )
    const
  {
    ASSERT_LT(index, this->tlmHistory_LLR_CrcErrors->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel LLR_CrcErrors\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_LLR_CrcErrors->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const TlmEntry_LLR_CrcErrors& e =
      this->tlmHistory_LLR_CrcErrors->at(index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << index
      << " on telmetry channel LLR_CrcErrors\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: LLR_ResyncErrors
  // ----------------------------------------------------------------------

  void LLRouterGTestBase ::
    assertTlm_LLR_ResyncErrors_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_LLR_ResyncErrors->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel LLR_ResyncErrors\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_LLR_ResyncErrors->size() << "\n";
  }

  void LLRouterGTestBase ::
    assertTlm_LLR_ResyncErrors(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        const U32& val
    )
    const
  {
    ASSERT_LT(index, this->tlmHistory_LLR_ResyncErrors->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel LLR_ResyncErrors\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_LLR_ResyncErrors->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const TlmEntry_LLR_ResyncErrors& e =
      this->tlmHistory_LLR_ResyncErrors->at(index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << index
      << " on telmetry channel LLR_ResyncErrors\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: LLR_NumPackets
  // ----------------------------------------------------------------------

  void LLRouterGTestBase ::
    assertTlm_LLR_NumPackets_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_LLR_NumPackets->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel LLR_NumPackets\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_LLR_NumPackets->size() << "\n";
  }

  void LLRouterGTestBase ::
    assertTlm_LLR_NumPackets(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        const U32& val
    )
    const
  {
    ASSERT_LT(index, this->tlmHistory_LLR_NumPackets->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel LLR_NumPackets\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_LLR_NumPackets->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const TlmEntry_LLR_NumPackets& e =
      this->tlmHistory_LLR_NumPackets->at(index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << index
      << " on telmetry channel LLR_NumPackets\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: LLR_NumSerialErrors
  // ----------------------------------------------------------------------

  void LLRouterGTestBase ::
    assertTlm_LLR_NumSerialErrors_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_LLR_NumSerialErrors->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel LLR_NumSerialErrors\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_LLR_NumSerialErrors->size() << "\n";
  }

  void LLRouterGTestBase ::
    assertTlm_LLR_NumSerialErrors(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        const U32& val
    )
    const
  {
    ASSERT_LT(index, this->tlmHistory_LLR_NumSerialErrors->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel LLR_NumSerialErrors\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_LLR_NumSerialErrors->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const TlmEntry_LLR_NumSerialErrors& e =
      this->tlmHistory_LLR_NumSerialErrors->at(index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << index
      << " on telmetry channel LLR_NumSerialErrors\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: LLR_NumGoodPackets
  // ----------------------------------------------------------------------

  void LLRouterGTestBase ::
    assertTlm_LLR_NumGoodPackets_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_LLR_NumGoodPackets->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel LLR_NumGoodPackets\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_LLR_NumGoodPackets->size() << "\n";
  }

  void LLRouterGTestBase ::
    assertTlm_LLR_NumGoodPackets(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        const U32& val
    )
    const
  {
    ASSERT_LT(index, this->tlmHistory_LLR_NumGoodPackets->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel LLR_NumGoodPackets\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_LLR_NumGoodPackets->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const TlmEntry_LLR_NumGoodPackets& e =
      this->tlmHistory_LLR_NumGoodPackets->at(index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << index
      << " on telmetry channel LLR_NumGoodPackets\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: LLR_NumDecodeErrors
  // ----------------------------------------------------------------------

  void LLRouterGTestBase ::
    assertTlm_LLR_NumDecodeErrors_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_LLR_NumDecodeErrors->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel LLR_NumDecodeErrors\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_LLR_NumDecodeErrors->size() << "\n";
  }

  void LLRouterGTestBase ::
    assertTlm_LLR_NumDecodeErrors(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        const U32& val
    )
    const
  {
    ASSERT_LT(index, this->tlmHistory_LLR_NumDecodeErrors->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel LLR_NumDecodeErrors\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_LLR_NumDecodeErrors->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const TlmEntry_LLR_NumDecodeErrors& e =
      this->tlmHistory_LLR_NumDecodeErrors->at(index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << index
      << " on telmetry channel LLR_NumDecodeErrors\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: LLR_NumInvalidPorts
  // ----------------------------------------------------------------------

  void LLRouterGTestBase ::
    assertTlm_LLR_NumInvalidPorts_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_LLR_NumInvalidPorts->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel LLR_NumInvalidPorts\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_LLR_NumInvalidPorts->size() << "\n";
  }

  void LLRouterGTestBase ::
    assertTlm_LLR_NumInvalidPorts(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        const U32& val
    )
    const
  {
    ASSERT_LT(index, this->tlmHistory_LLR_NumInvalidPorts->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel LLR_NumInvalidPorts\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_LLR_NumInvalidPorts->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const TlmEntry_LLR_NumInvalidPorts& e =
      this->tlmHistory_LLR_NumInvalidPorts->at(index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << index
      << " on telmetry channel LLR_NumInvalidPorts\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: LLR_NumBadSerialPortCalls
  // ----------------------------------------------------------------------

  void LLRouterGTestBase ::
    assertTlm_LLR_NumBadSerialPortCalls_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_LLR_NumBadSerialPortCalls->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel LLR_NumBadSerialPortCalls\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_LLR_NumBadSerialPortCalls->size() << "\n";
  }

  void LLRouterGTestBase ::
    assertTlm_LLR_NumBadSerialPortCalls(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        const U32& val
    )
    const
  {
    ASSERT_LT(index, this->tlmHistory_LLR_NumBadSerialPortCalls->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel LLR_NumBadSerialPortCalls\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_LLR_NumBadSerialPortCalls->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const TlmEntry_LLR_NumBadSerialPortCalls& e =
      this->tlmHistory_LLR_NumBadSerialPortCalls->at(index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << index
      << " on telmetry channel LLR_NumBadSerialPortCalls\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: LLR_NumOuputBufferOverflows
  // ----------------------------------------------------------------------

  void LLRouterGTestBase ::
    assertTlm_LLR_NumOuputBufferOverflows_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_LLR_NumOuputBufferOverflows->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel LLR_NumOuputBufferOverflows\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_LLR_NumOuputBufferOverflows->size() << "\n";
  }

  void LLRouterGTestBase ::
    assertTlm_LLR_NumOuputBufferOverflows(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        const U32& val
    )
    const
  {
    ASSERT_LT(index, this->tlmHistory_LLR_NumOuputBufferOverflows->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel LLR_NumOuputBufferOverflows\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_LLR_NumOuputBufferOverflows->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const TlmEntry_LLR_NumOuputBufferOverflows& e =
      this->tlmHistory_LLR_NumOuputBufferOverflows->at(index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << index
      << " on telmetry channel LLR_NumOuputBufferOverflows\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: LLR_NumZeroPktSize
  // ----------------------------------------------------------------------

  void LLRouterGTestBase ::
    assertTlm_LLR_NumZeroPktSize_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_LLR_NumZeroPktSize->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel LLR_NumZeroPktSize\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_LLR_NumZeroPktSize->size() << "\n";
  }

  void LLRouterGTestBase ::
    assertTlm_LLR_NumZeroPktSize(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        const U32& val
    )
    const
  {
    ASSERT_LT(index, this->tlmHistory_LLR_NumZeroPktSize->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel LLR_NumZeroPktSize\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_LLR_NumZeroPktSize->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const TlmEntry_LLR_NumZeroPktSize& e =
      this->tlmHistory_LLR_NumZeroPktSize->at(index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << index
      << " on telmetry channel LLR_NumZeroPktSize\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: LLR_MaxCycleTime
  // ----------------------------------------------------------------------

  void LLRouterGTestBase ::
    assertTlm_LLR_MaxCycleTime_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_LLR_MaxCycleTime->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel LLR_MaxCycleTime\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_LLR_MaxCycleTime->size() << "\n";
  }

  void LLRouterGTestBase ::
    assertTlm_LLR_MaxCycleTime(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        const U32& val
    )
    const
  {
    ASSERT_LT(index, this->tlmHistory_LLR_MaxCycleTime->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel LLR_MaxCycleTime\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_LLR_MaxCycleTime->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const TlmEntry_LLR_MaxCycleTime& e =
      this->tlmHistory_LLR_MaxCycleTime->at(index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << index
      << " on telmetry channel LLR_MaxCycleTime\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: LLR_CycleTime
  // ----------------------------------------------------------------------

  void LLRouterGTestBase ::
    assertTlm_LLR_CycleTime_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_LLR_CycleTime->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel LLR_CycleTime\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_LLR_CycleTime->size() << "\n";
  }

  void LLRouterGTestBase ::
    assertTlm_LLR_CycleTime(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        const U32& val
    )
    const
  {
    ASSERT_LT(index, this->tlmHistory_LLR_CycleTime->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel LLR_CycleTime\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_LLR_CycleTime->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const TlmEntry_LLR_CycleTime& e =
      this->tlmHistory_LLR_CycleTime->at(index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << index
      << " on telmetry channel LLR_CycleTime\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: LLR_PatchProgress
  // ----------------------------------------------------------------------

  void LLRouterGTestBase ::
    assertTlm_LLR_PatchProgress_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_LLR_PatchProgress->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel LLR_PatchProgress\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_LLR_PatchProgress->size() << "\n";
  }

  void LLRouterGTestBase ::
    assertTlm_LLR_PatchProgress(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        const U32& val
    )
    const
  {
    ASSERT_LT(index, this->tlmHistory_LLR_PatchProgress->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel LLR_PatchProgress\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_LLR_PatchProgress->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const TlmEntry_LLR_PatchProgress& e =
      this->tlmHistory_LLR_PatchProgress->at(index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << index
      << " on telmetry channel LLR_PatchProgress\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Events
  // ----------------------------------------------------------------------

  void LLRouterGTestBase ::
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
  // Event: LLR_DataReceiveError
  // ----------------------------------------------------------------------

  void LLRouterGTestBase ::
    assertEvents_LLR_DataReceiveError_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_LLR_DataReceiveError->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event LLR_DataReceiveError\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_LLR_DataReceiveError->size() << "\n";
  }

  void LLRouterGTestBase ::
    assertEvents_LLR_DataReceiveError(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        LLRouterComponentBase::LLR_ReceiveError error
    ) const
  {
    ASSERT_GT(this->eventHistory_LLR_DataReceiveError->size(), index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event LLR_DataReceiveError\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_LLR_DataReceiveError->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const EventEntry_LLR_DataReceiveError& e =
      this->eventHistory_LLR_DataReceiveError->at(index);
    ASSERT_EQ(error, e.error)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument error at index "
      << index
      << " in history of event LLR_DataReceiveError\n"
      << "  Expected: " << error << "\n"
      << "  Actual:   " << e.error << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: LLR_DataCrcError
  // ----------------------------------------------------------------------

  void LLRouterGTestBase ::
    assertEvents_LLR_DataCrcError_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_LLR_DataCrcError->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event LLR_DataCrcError\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_LLR_DataCrcError->size() << "\n";
  }

  void LLRouterGTestBase ::
    assertEvents_LLR_DataCrcError(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        const U16 expected,
        const U16 actual
    ) const
  {
    ASSERT_GT(this->eventHistory_LLR_DataCrcError->size(), index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event LLR_DataCrcError\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_LLR_DataCrcError->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const EventEntry_LLR_DataCrcError& e =
      this->eventHistory_LLR_DataCrcError->at(index);
    ASSERT_EQ(expected, e.expected)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument expected at index "
      << index
      << " in history of event LLR_DataCrcError\n"
      << "  Expected: " << expected << "\n"
      << "  Actual:   " << e.expected << "\n";
    ASSERT_EQ(actual, e.actual)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument actual at index "
      << index
      << " in history of event LLR_DataCrcError\n"
      << "  Expected: " << actual << "\n"
      << "  Actual:   " << e.actual << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: LLR_InvalidPortNum
  // ----------------------------------------------------------------------

  void LLRouterGTestBase ::
    assertEvents_LLR_InvalidPortNum_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_LLR_InvalidPortNum->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event LLR_InvalidPortNum\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_LLR_InvalidPortNum->size() << "\n";
  }

  void LLRouterGTestBase ::
    assertEvents_LLR_InvalidPortNum(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        const U8 portNum
    ) const
  {
    ASSERT_GT(this->eventHistory_LLR_InvalidPortNum->size(), index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event LLR_InvalidPortNum\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_LLR_InvalidPortNum->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const EventEntry_LLR_InvalidPortNum& e =
      this->eventHistory_LLR_InvalidPortNum->at(index);
    ASSERT_EQ(portNum, e.portNum)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument portNum at index "
      << index
      << " in history of event LLR_InvalidPortNum\n"
      << "  Expected: " << portNum << "\n"
      << "  Actual:   " << e.portNum << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: LLR_BadSerialPortCall
  // ----------------------------------------------------------------------

  void LLRouterGTestBase ::
    assertEvents_LLR_BadSerialPortCall_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_LLR_BadSerialPortCall->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event LLR_BadSerialPortCall\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_LLR_BadSerialPortCall->size() << "\n";
  }

  void LLRouterGTestBase ::
    assertEvents_LLR_BadSerialPortCall(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        const I32 status,
        const U8 portNum
    ) const
  {
    ASSERT_GT(this->eventHistory_LLR_BadSerialPortCall->size(), index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event LLR_BadSerialPortCall\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_LLR_BadSerialPortCall->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const EventEntry_LLR_BadSerialPortCall& e =
      this->eventHistory_LLR_BadSerialPortCall->at(index);
    ASSERT_EQ(status, e.status)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument status at index "
      << index
      << " in history of event LLR_BadSerialPortCall\n"
      << "  Expected: " << status << "\n"
      << "  Actual:   " << e.status << "\n";
    ASSERT_EQ(portNum, e.portNum)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument portNum at index "
      << index
      << " in history of event LLR_BadSerialPortCall\n"
      << "  Expected: " << portNum << "\n"
      << "  Actual:   " << e.portNum << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: LLR_PatchFileOpen
  // ----------------------------------------------------------------------

  void LLRouterGTestBase ::
    assertEvents_LLR_PatchFileOpen_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_LLR_PatchFileOpen->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event LLR_PatchFileOpen\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_LLR_PatchFileOpen->size() << "\n";
  }

  void LLRouterGTestBase ::
    assertEvents_LLR_PatchFileOpen(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        const char *const filename,
        const U32 filesize
    ) const
  {
    ASSERT_GT(this->eventHistory_LLR_PatchFileOpen->size(), index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event LLR_PatchFileOpen\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_LLR_PatchFileOpen->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const EventEntry_LLR_PatchFileOpen& e =
      this->eventHistory_LLR_PatchFileOpen->at(index);
    ASSERT_STREQ(filename, e.filename.toChar())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument filename at index "
      << index
      << " in history of event LLR_PatchFileOpen\n"
      << "  Expected: " << filename << "\n"
      << "  Actual:   " << e.filename.toChar() << "\n";
    ASSERT_EQ(filesize, e.filesize)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument filesize at index "
      << index
      << " in history of event LLR_PatchFileOpen\n"
      << "  Expected: " << filesize << "\n"
      << "  Actual:   " << e.filesize << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: LLR_PatchModeChange
  // ----------------------------------------------------------------------

  void LLRouterGTestBase ::
    assertEvents_LLR_PatchModeChange_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_LLR_PatchModeChange->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event LLR_PatchModeChange\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_LLR_PatchModeChange->size() << "\n";
  }

  void LLRouterGTestBase ::
    assertEvents_LLR_PatchModeChange(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        const char *const newmode
    ) const
  {
    ASSERT_GT(this->eventHistory_LLR_PatchModeChange->size(), index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event LLR_PatchModeChange\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_LLR_PatchModeChange->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const EventEntry_LLR_PatchModeChange& e =
      this->eventHistory_LLR_PatchModeChange->at(index);
    ASSERT_STREQ(newmode, e.newmode.toChar())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument newmode at index "
      << index
      << " in history of event LLR_PatchModeChange\n"
      << "  Expected: " << newmode << "\n"
      << "  Actual:   " << e.newmode.toChar() << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: LLR_FirstACK
  // ----------------------------------------------------------------------

  void LLRouterGTestBase ::
    assertEvents_LLR_FirstACK_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventsSize_LLR_FirstACK)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event LLR_FirstACK\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventsSize_LLR_FirstACK << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: LLR_PatchEvent
  // ----------------------------------------------------------------------

  void LLRouterGTestBase ::
    assertEvents_LLR_PatchEvent_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_LLR_PatchEvent->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event LLR_PatchEvent\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_LLR_PatchEvent->size() << "\n";
  }

  void LLRouterGTestBase ::
    assertEvents_LLR_PatchEvent(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        const U32 progress
    ) const
  {
    ASSERT_GT(this->eventHistory_LLR_PatchEvent->size(), index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event LLR_PatchEvent\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_LLR_PatchEvent->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const EventEntry_LLR_PatchEvent& e =
      this->eventHistory_LLR_PatchEvent->at(index);
    ASSERT_EQ(progress, e.progress)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument progress at index "
      << index
      << " in history of event LLR_PatchEvent\n"
      << "  Expected: " << progress << "\n"
      << "  Actual:   " << e.progress << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: LLR_PatchComplete
  // ----------------------------------------------------------------------

  void LLRouterGTestBase ::
    assertEvents_LLR_PatchComplete_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventsSize_LLR_PatchComplete)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event LLR_PatchComplete\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventsSize_LLR_PatchComplete << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: LLR_PatchError
  // ----------------------------------------------------------------------

  void LLRouterGTestBase ::
    assertEvents_LLR_PatchError_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_LLR_PatchError->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event LLR_PatchError\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_LLR_PatchError->size() << "\n";
  }

  void LLRouterGTestBase ::
    assertEvents_LLR_PatchError(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        LLRouterComponentBase::LLR_PatchError error,
        const I32 err_num,
        const char *const err_string
    ) const
  {
    ASSERT_GT(this->eventHistory_LLR_PatchError->size(), index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event LLR_PatchError\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_LLR_PatchError->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const EventEntry_LLR_PatchError& e =
      this->eventHistory_LLR_PatchError->at(index);
    ASSERT_EQ(error, e.error)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument error at index "
      << index
      << " in history of event LLR_PatchError\n"
      << "  Expected: " << error << "\n"
      << "  Actual:   " << e.error << "\n";
    ASSERT_EQ(err_num, e.err_num)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument err_num at index "
      << index
      << " in history of event LLR_PatchError\n"
      << "  Expected: " << err_num << "\n"
      << "  Actual:   " << e.err_num << "\n";
    ASSERT_STREQ(err_string, e.err_string.toChar())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument err_string at index "
      << index
      << " in history of event LLR_PatchError\n"
      << "  Expected: " << err_string << "\n"
      << "  Actual:   " << e.err_string.toChar() << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: LLR_PolyError
  // ----------------------------------------------------------------------

  void LLRouterGTestBase ::
    assertEvents_LLR_PolyError_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventsSize_LLR_PolyError)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event LLR_PolyError\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventsSize_LLR_PolyError << "\n";
  }

  // ----------------------------------------------------------------------
  // From ports
  // ----------------------------------------------------------------------

  void LLRouterGTestBase ::
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
  // From port: SerWritePort
  // ----------------------------------------------------------------------

  void LLRouterGTestBase ::
    assert_from_SerWritePort_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->fromPortHistory_SerWritePort->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for from_SerWritePort\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->fromPortHistory_SerWritePort->size() << "\n";
  }

  // ----------------------------------------------------------------------
  // From port: SerialBufferSend
  // ----------------------------------------------------------------------

  void LLRouterGTestBase ::
    assert_from_SerialBufferSend_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->fromPortHistory_SerialBufferSend->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for from_SerialBufferSend\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->fromPortHistory_SerialBufferSend->size() << "\n";
  }

  // ----------------------------------------------------------------------
  // From port: pingOut
  // ----------------------------------------------------------------------

  void LLRouterGTestBase ::
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

  // ----------------------------------------------------------------------
  // From port: PolyGet
  // ----------------------------------------------------------------------

  void LLRouterGTestBase ::
    assert_from_PolyGet_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->fromPortHistory_PolyGet->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for from_PolyGet\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->fromPortHistory_PolyGet->size() << "\n";
  }

} // end namespace HLProc
