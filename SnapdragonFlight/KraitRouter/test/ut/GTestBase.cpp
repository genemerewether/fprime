// ======================================================================
// \title  KraitRouter/test/ut/GTestBase.cpp
// \author Auto-generated
// \brief  cpp file for KraitRouter component Google Test harness base class
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

  KraitRouterGTestBase ::
    KraitRouterGTestBase(
#if FW_OBJECT_NAMES == 1
        const char *const compName,
        const U32 maxHistorySize
#else
        const U32 maxHistorySize
#endif
    ) : 
        KraitRouterTesterBase (
#if FW_OBJECT_NAMES == 1
            compName,
#endif
            maxHistorySize
        )
  {

  }

  KraitRouterGTestBase ::
    ~KraitRouterGTestBase(void)
  {

  }

  // ----------------------------------------------------------------------
  // Telemetry
  // ----------------------------------------------------------------------

  void KraitRouterGTestBase ::
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
  // Channel: KR_ResyncErrors
  // ----------------------------------------------------------------------

  void KraitRouterGTestBase ::
    assertTlm_KR_ResyncErrors_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_KR_ResyncErrors->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel KR_ResyncErrors\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_KR_ResyncErrors->size() << "\n";
  }

  void KraitRouterGTestBase ::
    assertTlm_KR_ResyncErrors(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        const U32& val
    )
    const
  {
    ASSERT_LT(index, this->tlmHistory_KR_ResyncErrors->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel KR_ResyncErrors\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_KR_ResyncErrors->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const TlmEntry_KR_ResyncErrors& e =
      this->tlmHistory_KR_ResyncErrors->at(index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << index
      << " on telmetry channel KR_ResyncErrors\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: KR_NumPackets
  // ----------------------------------------------------------------------

  void KraitRouterGTestBase ::
    assertTlm_KR_NumPackets_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_KR_NumPackets->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel KR_NumPackets\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_KR_NumPackets->size() << "\n";
  }

  void KraitRouterGTestBase ::
    assertTlm_KR_NumPackets(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        const U32& val
    )
    const
  {
    ASSERT_LT(index, this->tlmHistory_KR_NumPackets->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel KR_NumPackets\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_KR_NumPackets->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const TlmEntry_KR_NumPackets& e =
      this->tlmHistory_KR_NumPackets->at(index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << index
      << " on telmetry channel KR_NumPackets\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: KR_NumDecodeErrors
  // ----------------------------------------------------------------------

  void KraitRouterGTestBase ::
    assertTlm_KR_NumDecodeErrors_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_KR_NumDecodeErrors->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel KR_NumDecodeErrors\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_KR_NumDecodeErrors->size() << "\n";
  }

  void KraitRouterGTestBase ::
    assertTlm_KR_NumDecodeErrors(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        const U32& val
    )
    const
  {
    ASSERT_LT(index, this->tlmHistory_KR_NumDecodeErrors->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel KR_NumDecodeErrors\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_KR_NumDecodeErrors->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const TlmEntry_KR_NumDecodeErrors& e =
      this->tlmHistory_KR_NumDecodeErrors->at(index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << index
      << " on telmetry channel KR_NumDecodeErrors\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: KR_NumInvalidPorts
  // ----------------------------------------------------------------------

  void KraitRouterGTestBase ::
    assertTlm_KR_NumInvalidPorts_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_KR_NumInvalidPorts->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel KR_NumInvalidPorts\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_KR_NumInvalidPorts->size() << "\n";
  }

  void KraitRouterGTestBase ::
    assertTlm_KR_NumInvalidPorts(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        const U32& val
    )
    const
  {
    ASSERT_LT(index, this->tlmHistory_KR_NumInvalidPorts->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel KR_NumInvalidPorts\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_KR_NumInvalidPorts->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const TlmEntry_KR_NumInvalidPorts& e =
      this->tlmHistory_KR_NumInvalidPorts->at(index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << index
      << " on telmetry channel KR_NumInvalidPorts\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: KR_NumBadSerialPortCalls
  // ----------------------------------------------------------------------

  void KraitRouterGTestBase ::
    assertTlm_KR_NumBadSerialPortCalls_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_KR_NumBadSerialPortCalls->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel KR_NumBadSerialPortCalls\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_KR_NumBadSerialPortCalls->size() << "\n";
  }

  void KraitRouterGTestBase ::
    assertTlm_KR_NumBadSerialPortCalls(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        const U32& val
    )
    const
  {
    ASSERT_LT(index, this->tlmHistory_KR_NumBadSerialPortCalls->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel KR_NumBadSerialPortCalls\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_KR_NumBadSerialPortCalls->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const TlmEntry_KR_NumBadSerialPortCalls& e =
      this->tlmHistory_KR_NumBadSerialPortCalls->at(index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << index
      << " on telmetry channel KR_NumBadSerialPortCalls\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: KR_NumOutputBufferOverflows
  // ----------------------------------------------------------------------

  void KraitRouterGTestBase ::
    assertTlm_KR_NumOutputBufferOverflows_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_KR_NumOutputBufferOverflows->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel KR_NumOutputBufferOverflows\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_KR_NumOutputBufferOverflows->size() << "\n";
  }

  void KraitRouterGTestBase ::
    assertTlm_KR_NumOutputBufferOverflows(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        const U32& val
    )
    const
  {
    ASSERT_LT(index, this->tlmHistory_KR_NumOutputBufferOverflows->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel KR_NumOutputBufferOverflows\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_KR_NumOutputBufferOverflows->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const TlmEntry_KR_NumOutputBufferOverflows& e =
      this->tlmHistory_KR_NumOutputBufferOverflows->at(index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << index
      << " on telmetry channel KR_NumOutputBufferOverflows\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: KR_NumZeroPktSize
  // ----------------------------------------------------------------------

  void KraitRouterGTestBase ::
    assertTlm_KR_NumZeroPktSize_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_KR_NumZeroPktSize->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel KR_NumZeroPktSize\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_KR_NumZeroPktSize->size() << "\n";
  }

  void KraitRouterGTestBase ::
    assertTlm_KR_NumZeroPktSize(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        const U32& val
    )
    const
  {
    ASSERT_LT(index, this->tlmHistory_KR_NumZeroPktSize->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel KR_NumZeroPktSize\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_KR_NumZeroPktSize->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const TlmEntry_KR_NumZeroPktSize& e =
      this->tlmHistory_KR_NumZeroPktSize->at(index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << index
      << " on telmetry channel KR_NumZeroPktSize\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Events
  // ----------------------------------------------------------------------

  void KraitRouterGTestBase ::
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
  // Event: KR_DataReceiveError
  // ----------------------------------------------------------------------

  void KraitRouterGTestBase ::
    assertEvents_KR_DataReceiveError_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_KR_DataReceiveError->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event KR_DataReceiveError\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_KR_DataReceiveError->size() << "\n";
  }

  void KraitRouterGTestBase ::
    assertEvents_KR_DataReceiveError(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        KraitRouterComponentBase::KR_ReceiveError error,
        const I32 stat
    ) const
  {
    ASSERT_GT(this->eventHistory_KR_DataReceiveError->size(), index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event KR_DataReceiveError\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_KR_DataReceiveError->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const EventEntry_KR_DataReceiveError& e =
      this->eventHistory_KR_DataReceiveError->at(index);
    ASSERT_EQ(error, e.error)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument error at index "
      << index
      << " in history of event KR_DataReceiveError\n"
      << "  Expected: " << error << "\n"
      << "  Actual:   " << e.error << "\n";
    ASSERT_EQ(stat, e.stat)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument stat at index "
      << index
      << " in history of event KR_DataReceiveError\n"
      << "  Expected: " << stat << "\n"
      << "  Actual:   " << e.stat << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: KR_InvalidPortNum
  // ----------------------------------------------------------------------

  void KraitRouterGTestBase ::
    assertEvents_KR_InvalidPortNum_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_KR_InvalidPortNum->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event KR_InvalidPortNum\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_KR_InvalidPortNum->size() << "\n";
  }

  void KraitRouterGTestBase ::
    assertEvents_KR_InvalidPortNum(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        KraitRouterComponentBase::KR_PortType PortType,
        const U8 portNum
    ) const
  {
    ASSERT_GT(this->eventHistory_KR_InvalidPortNum->size(), index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event KR_InvalidPortNum\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_KR_InvalidPortNum->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const EventEntry_KR_InvalidPortNum& e =
      this->eventHistory_KR_InvalidPortNum->at(index);
    ASSERT_EQ(PortType, e.PortType)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument PortType at index "
      << index
      << " in history of event KR_InvalidPortNum\n"
      << "  Expected: " << PortType << "\n"
      << "  Actual:   " << e.PortType << "\n";
    ASSERT_EQ(portNum, e.portNum)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument portNum at index "
      << index
      << " in history of event KR_InvalidPortNum\n"
      << "  Expected: " << portNum << "\n"
      << "  Actual:   " << e.portNum << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: KR_BadSerialPortCall
  // ----------------------------------------------------------------------

  void KraitRouterGTestBase ::
    assertEvents_KR_BadSerialPortCall_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_KR_BadSerialPortCall->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event KR_BadSerialPortCall\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_KR_BadSerialPortCall->size() << "\n";
  }

  void KraitRouterGTestBase ::
    assertEvents_KR_BadSerialPortCall(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        const I32 status,
        const U8 portNum
    ) const
  {
    ASSERT_GT(this->eventHistory_KR_BadSerialPortCall->size(), index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event KR_BadSerialPortCall\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_KR_BadSerialPortCall->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const EventEntry_KR_BadSerialPortCall& e =
      this->eventHistory_KR_BadSerialPortCall->at(index);
    ASSERT_EQ(status, e.status)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument status at index "
      << index
      << " in history of event KR_BadSerialPortCall\n"
      << "  Expected: " << status << "\n"
      << "  Actual:   " << e.status << "\n";
    ASSERT_EQ(portNum, e.portNum)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument portNum at index "
      << index
      << " in history of event KR_BadSerialPortCall\n"
      << "  Expected: " << portNum << "\n"
      << "  Actual:   " << e.portNum << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: KR_MemoryError
  // ----------------------------------------------------------------------

  void KraitRouterGTestBase ::
    assertEvents_KR_MemoryError_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_KR_MemoryError->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event KR_MemoryError\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_KR_MemoryError->size() << "\n";
  }

  void KraitRouterGTestBase ::
    assertEvents_KR_MemoryError(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        KraitRouterComponentBase::KR_MemoryError error
    ) const
  {
    ASSERT_GT(this->eventHistory_KR_MemoryError->size(), index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event KR_MemoryError\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_KR_MemoryError->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const EventEntry_KR_MemoryError& e =
      this->eventHistory_KR_MemoryError->at(index);
    ASSERT_EQ(error, e.error)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument error at index "
      << index
      << " in history of event KR_MemoryError\n"
      << "  Expected: " << error << "\n"
      << "  Actual:   " << e.error << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: KR_WriteError
  // ----------------------------------------------------------------------

  void KraitRouterGTestBase ::
    assertEvents_KR_WriteError_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_KR_WriteError->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event KR_WriteError\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_KR_WriteError->size() << "\n";
  }

  void KraitRouterGTestBase ::
    assertEvents_KR_WriteError(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        const I32 error
    ) const
  {
    ASSERT_GT(this->eventHistory_KR_WriteError->size(), index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event KR_WriteError\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_KR_WriteError->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const EventEntry_KR_WriteError& e =
      this->eventHistory_KR_WriteError->at(index);
    ASSERT_EQ(error, e.error)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument error at index "
      << index
      << " in history of event KR_WriteError\n"
      << "  Expected: " << error << "\n"
      << "  Actual:   " << e.error << "\n";
  }

} // end namespace SnapdragonFlight
