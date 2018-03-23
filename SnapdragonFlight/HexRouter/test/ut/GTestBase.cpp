// ======================================================================
// \title  HexRouter/test/ut/GTestBase.cpp
// \author Auto-generated
// \brief  cpp file for HexRouter component Google Test harness base class
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

  HexRouterGTestBase ::
    HexRouterGTestBase(
#if FW_OBJECT_NAMES == 1
        const char *const compName,
        const U32 maxHistorySize
#else
        const U32 maxHistorySize
#endif
    ) : 
        HexRouterTesterBase (
#if FW_OBJECT_NAMES == 1
            compName,
#endif
            maxHistorySize
        )
  {

  }

  HexRouterGTestBase ::
    ~HexRouterGTestBase(void)
  {

  }

  // ----------------------------------------------------------------------
  // Telemetry
  // ----------------------------------------------------------------------

  void HexRouterGTestBase ::
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
  // Channel: HR_ResyncErrors
  // ----------------------------------------------------------------------

  void HexRouterGTestBase ::
    assertTlm_HR_ResyncErrors_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_HR_ResyncErrors->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel HR_ResyncErrors\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_HR_ResyncErrors->size() << "\n";
  }

  void HexRouterGTestBase ::
    assertTlm_HR_ResyncErrors(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        const U32& val
    )
    const
  {
    ASSERT_LT(index, this->tlmHistory_HR_ResyncErrors->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel HR_ResyncErrors\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_HR_ResyncErrors->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const TlmEntry_HR_ResyncErrors& e =
      this->tlmHistory_HR_ResyncErrors->at(index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << index
      << " on telmetry channel HR_ResyncErrors\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: HR_NumPackets
  // ----------------------------------------------------------------------

  void HexRouterGTestBase ::
    assertTlm_HR_NumPackets_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_HR_NumPackets->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel HR_NumPackets\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_HR_NumPackets->size() << "\n";
  }

  void HexRouterGTestBase ::
    assertTlm_HR_NumPackets(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        const U32& val
    )
    const
  {
    ASSERT_LT(index, this->tlmHistory_HR_NumPackets->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel HR_NumPackets\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_HR_NumPackets->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const TlmEntry_HR_NumPackets& e =
      this->tlmHistory_HR_NumPackets->at(index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << index
      << " on telmetry channel HR_NumPackets\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: HR_NumDecodeErrors
  // ----------------------------------------------------------------------

  void HexRouterGTestBase ::
    assertTlm_HR_NumDecodeErrors_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_HR_NumDecodeErrors->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel HR_NumDecodeErrors\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_HR_NumDecodeErrors->size() << "\n";
  }

  void HexRouterGTestBase ::
    assertTlm_HR_NumDecodeErrors(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        const U32& val
    )
    const
  {
    ASSERT_LT(index, this->tlmHistory_HR_NumDecodeErrors->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel HR_NumDecodeErrors\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_HR_NumDecodeErrors->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const TlmEntry_HR_NumDecodeErrors& e =
      this->tlmHistory_HR_NumDecodeErrors->at(index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << index
      << " on telmetry channel HR_NumDecodeErrors\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: HR_NumInvalidPorts
  // ----------------------------------------------------------------------

  void HexRouterGTestBase ::
    assertTlm_HR_NumInvalidPorts_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_HR_NumInvalidPorts->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel HR_NumInvalidPorts\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_HR_NumInvalidPorts->size() << "\n";
  }

  void HexRouterGTestBase ::
    assertTlm_HR_NumInvalidPorts(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        const U32& val
    )
    const
  {
    ASSERT_LT(index, this->tlmHistory_HR_NumInvalidPorts->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel HR_NumInvalidPorts\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_HR_NumInvalidPorts->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const TlmEntry_HR_NumInvalidPorts& e =
      this->tlmHistory_HR_NumInvalidPorts->at(index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << index
      << " on telmetry channel HR_NumInvalidPorts\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: HR_NumBadSerialPortCalls
  // ----------------------------------------------------------------------

  void HexRouterGTestBase ::
    assertTlm_HR_NumBadSerialPortCalls_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_HR_NumBadSerialPortCalls->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel HR_NumBadSerialPortCalls\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_HR_NumBadSerialPortCalls->size() << "\n";
  }

  void HexRouterGTestBase ::
    assertTlm_HR_NumBadSerialPortCalls(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        const U32& val
    )
    const
  {
    ASSERT_LT(index, this->tlmHistory_HR_NumBadSerialPortCalls->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel HR_NumBadSerialPortCalls\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_HR_NumBadSerialPortCalls->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const TlmEntry_HR_NumBadSerialPortCalls& e =
      this->tlmHistory_HR_NumBadSerialPortCalls->at(index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << index
      << " on telmetry channel HR_NumBadSerialPortCalls\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: HR_NumOutputBufferOverflows
  // ----------------------------------------------------------------------

  void HexRouterGTestBase ::
    assertTlm_HR_NumOutputBufferOverflows_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_HR_NumOutputBufferOverflows->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel HR_NumOutputBufferOverflows\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_HR_NumOutputBufferOverflows->size() << "\n";
  }

  void HexRouterGTestBase ::
    assertTlm_HR_NumOutputBufferOverflows(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        const U32& val
    )
    const
  {
    ASSERT_LT(index, this->tlmHistory_HR_NumOutputBufferOverflows->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel HR_NumOutputBufferOverflows\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_HR_NumOutputBufferOverflows->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const TlmEntry_HR_NumOutputBufferOverflows& e =
      this->tlmHistory_HR_NumOutputBufferOverflows->at(index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << index
      << " on telmetry channel HR_NumOutputBufferOverflows\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: HR_NumZeroPktSize
  // ----------------------------------------------------------------------

  void HexRouterGTestBase ::
    assertTlm_HR_NumZeroPktSize_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_HR_NumZeroPktSize->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel HR_NumZeroPktSize\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_HR_NumZeroPktSize->size() << "\n";
  }

  void HexRouterGTestBase ::
    assertTlm_HR_NumZeroPktSize(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        const U32& val
    )
    const
  {
    ASSERT_LT(index, this->tlmHistory_HR_NumZeroPktSize->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel HR_NumZeroPktSize\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_HR_NumZeroPktSize->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const TlmEntry_HR_NumZeroPktSize& e =
      this->tlmHistory_HR_NumZeroPktSize->at(index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << index
      << " on telmetry channel HR_NumZeroPktSize\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Events
  // ----------------------------------------------------------------------

  void HexRouterGTestBase ::
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
  // Event: HR_DataReceiveError
  // ----------------------------------------------------------------------

  void HexRouterGTestBase ::
    assertEvents_HR_DataReceiveError_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_HR_DataReceiveError->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event HR_DataReceiveError\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_HR_DataReceiveError->size() << "\n";
  }

  void HexRouterGTestBase ::
    assertEvents_HR_DataReceiveError(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        HexRouterComponentBase::HR_ReceiveError error,
        const I32 stat
    ) const
  {
    ASSERT_GT(this->eventHistory_HR_DataReceiveError->size(), index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event HR_DataReceiveError\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_HR_DataReceiveError->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const EventEntry_HR_DataReceiveError& e =
      this->eventHistory_HR_DataReceiveError->at(index);
    ASSERT_EQ(error, e.error)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument error at index "
      << index
      << " in history of event HR_DataReceiveError\n"
      << "  Expected: " << error << "\n"
      << "  Actual:   " << e.error << "\n";
    ASSERT_EQ(stat, e.stat)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument stat at index "
      << index
      << " in history of event HR_DataReceiveError\n"
      << "  Expected: " << stat << "\n"
      << "  Actual:   " << e.stat << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: HR_InvalidPortNum
  // ----------------------------------------------------------------------

  void HexRouterGTestBase ::
    assertEvents_HR_InvalidPortNum_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_HR_InvalidPortNum->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event HR_InvalidPortNum\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_HR_InvalidPortNum->size() << "\n";
  }

  void HexRouterGTestBase ::
    assertEvents_HR_InvalidPortNum(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        HexRouterComponentBase::HR_PortType PortType,
        const U8 portNum
    ) const
  {
    ASSERT_GT(this->eventHistory_HR_InvalidPortNum->size(), index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event HR_InvalidPortNum\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_HR_InvalidPortNum->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const EventEntry_HR_InvalidPortNum& e =
      this->eventHistory_HR_InvalidPortNum->at(index);
    ASSERT_EQ(PortType, e.PortType)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument PortType at index "
      << index
      << " in history of event HR_InvalidPortNum\n"
      << "  Expected: " << PortType << "\n"
      << "  Actual:   " << e.PortType << "\n";
    ASSERT_EQ(portNum, e.portNum)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument portNum at index "
      << index
      << " in history of event HR_InvalidPortNum\n"
      << "  Expected: " << portNum << "\n"
      << "  Actual:   " << e.portNum << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: HR_BadSerialPortCall
  // ----------------------------------------------------------------------

  void HexRouterGTestBase ::
    assertEvents_HR_BadSerialPortCall_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_HR_BadSerialPortCall->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event HR_BadSerialPortCall\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_HR_BadSerialPortCall->size() << "\n";
  }

  void HexRouterGTestBase ::
    assertEvents_HR_BadSerialPortCall(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        const I32 status,
        const U8 portNum
    ) const
  {
    ASSERT_GT(this->eventHistory_HR_BadSerialPortCall->size(), index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event HR_BadSerialPortCall\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_HR_BadSerialPortCall->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const EventEntry_HR_BadSerialPortCall& e =
      this->eventHistory_HR_BadSerialPortCall->at(index);
    ASSERT_EQ(status, e.status)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument status at index "
      << index
      << " in history of event HR_BadSerialPortCall\n"
      << "  Expected: " << status << "\n"
      << "  Actual:   " << e.status << "\n";
    ASSERT_EQ(portNum, e.portNum)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument portNum at index "
      << index
      << " in history of event HR_BadSerialPortCall\n"
      << "  Expected: " << portNum << "\n"
      << "  Actual:   " << e.portNum << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: HR_MemoryError
  // ----------------------------------------------------------------------

  void HexRouterGTestBase ::
    assertEvents_HR_MemoryError_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_HR_MemoryError->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event HR_MemoryError\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_HR_MemoryError->size() << "\n";
  }

  void HexRouterGTestBase ::
    assertEvents_HR_MemoryError(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        HexRouterComponentBase::HR_MemoryError error
    ) const
  {
    ASSERT_GT(this->eventHistory_HR_MemoryError->size(), index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event HR_MemoryError\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_HR_MemoryError->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const EventEntry_HR_MemoryError& e =
      this->eventHistory_HR_MemoryError->at(index);
    ASSERT_EQ(error, e.error)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument error at index "
      << index
      << " in history of event HR_MemoryError\n"
      << "  Expected: " << error << "\n"
      << "  Actual:   " << e.error << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: HR_WriteError
  // ----------------------------------------------------------------------

  void HexRouterGTestBase ::
    assertEvents_HR_WriteError_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_HR_WriteError->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event HR_WriteError\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_HR_WriteError->size() << "\n";
  }

  void HexRouterGTestBase ::
    assertEvents_HR_WriteError(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        const I32 error
    ) const
  {
    ASSERT_GT(this->eventHistory_HR_WriteError->size(), index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event HR_WriteError\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_HR_WriteError->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const EventEntry_HR_WriteError& e =
      this->eventHistory_HR_WriteError->at(index);
    ASSERT_EQ(error, e.error)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument error at index "
      << index
      << " in history of event HR_WriteError\n"
      << "  Expected: " << error << "\n"
      << "  Actual:   " << e.error << "\n";
  }

  // ----------------------------------------------------------------------
  // From ports
  // ----------------------------------------------------------------------

  void HexRouterGTestBase ::
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
  // From port: readBufferSend
  // ----------------------------------------------------------------------

  void HexRouterGTestBase ::
    assert_from_readBufferSend_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->fromPortHistory_readBufferSend->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for from_readBufferSend\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->fromPortHistory_readBufferSend->size() << "\n";
  }

} // end namespace SnapdragonFlight
