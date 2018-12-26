// ======================================================================
// \title  UdpReceiver/test/ut/GTestBase.cpp
// \author Auto-generated
// \brief  cpp file for UdpReceiver component Google Test harness base class
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

namespace Svc {

  // ----------------------------------------------------------------------
  // Construction and destruction
  // ----------------------------------------------------------------------

  UdpReceiverGTestBase ::
    UdpReceiverGTestBase(
#if FW_OBJECT_NAMES == 1
        const char *const compName,
        const U32 maxHistorySize
#else
        const U32 maxHistorySize
#endif
    ) : 
        UdpReceiverTesterBase (
#if FW_OBJECT_NAMES == 1
            compName,
#endif
            maxHistorySize
        )
  {

  }

  UdpReceiverGTestBase ::
    ~UdpReceiverGTestBase(void)
  {

  }

  // ----------------------------------------------------------------------
  // Telemetry
  // ----------------------------------------------------------------------

  void UdpReceiverGTestBase ::
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
  // Channel: UR_PacketsReceived
  // ----------------------------------------------------------------------

  void UdpReceiverGTestBase ::
    assertTlm_UR_PacketsReceived_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_UR_PacketsReceived->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel UR_PacketsReceived\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_UR_PacketsReceived->size() << "\n";
  }

  void UdpReceiverGTestBase ::
    assertTlm_UR_PacketsReceived(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        const U32& val
    )
    const
  {
    ASSERT_LT(index, this->tlmHistory_UR_PacketsReceived->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel UR_PacketsReceived\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_UR_PacketsReceived->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const TlmEntry_UR_PacketsReceived& e =
      this->tlmHistory_UR_PacketsReceived->at(index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << index
      << " on telmetry channel UR_PacketsReceived\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: UR_BytesReceived
  // ----------------------------------------------------------------------

  void UdpReceiverGTestBase ::
    assertTlm_UR_BytesReceived_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_UR_BytesReceived->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel UR_BytesReceived\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_UR_BytesReceived->size() << "\n";
  }

  void UdpReceiverGTestBase ::
    assertTlm_UR_BytesReceived(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        const U32& val
    )
    const
  {
    ASSERT_LT(index, this->tlmHistory_UR_BytesReceived->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel UR_BytesReceived\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_UR_BytesReceived->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const TlmEntry_UR_BytesReceived& e =
      this->tlmHistory_UR_BytesReceived->at(index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << index
      << " on telmetry channel UR_BytesReceived\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: UR_PacketsDropped
  // ----------------------------------------------------------------------

  void UdpReceiverGTestBase ::
    assertTlm_UR_PacketsDropped_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_UR_PacketsDropped->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel UR_PacketsDropped\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_UR_PacketsDropped->size() << "\n";
  }

  void UdpReceiverGTestBase ::
    assertTlm_UR_PacketsDropped(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        const U32& val
    )
    const
  {
    ASSERT_LT(index, this->tlmHistory_UR_PacketsDropped->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel UR_PacketsDropped\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_UR_PacketsDropped->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const TlmEntry_UR_PacketsDropped& e =
      this->tlmHistory_UR_PacketsDropped->at(index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << index
      << " on telmetry channel UR_PacketsDropped\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: UR_DecodeErrors
  // ----------------------------------------------------------------------

  void UdpReceiverGTestBase ::
    assertTlm_UR_DecodeErrors_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_UR_DecodeErrors->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel UR_DecodeErrors\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_UR_DecodeErrors->size() << "\n";
  }

  void UdpReceiverGTestBase ::
    assertTlm_UR_DecodeErrors(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        const U32& val
    )
    const
  {
    ASSERT_LT(index, this->tlmHistory_UR_DecodeErrors->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel UR_DecodeErrors\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_UR_DecodeErrors->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const TlmEntry_UR_DecodeErrors& e =
      this->tlmHistory_UR_DecodeErrors->at(index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << index
      << " on telmetry channel UR_DecodeErrors\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Events
  // ----------------------------------------------------------------------

  void UdpReceiverGTestBase ::
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
  // Event: UR_PortOpened
  // ----------------------------------------------------------------------

  void UdpReceiverGTestBase ::
    assertEvents_UR_PortOpened_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventsSize_UR_PortOpened)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event UR_PortOpened\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventsSize_UR_PortOpened << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: UR_SocketError
  // ----------------------------------------------------------------------

  void UdpReceiverGTestBase ::
    assertEvents_UR_SocketError_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_UR_SocketError->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event UR_SocketError\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_UR_SocketError->size() << "\n";
  }

  void UdpReceiverGTestBase ::
    assertEvents_UR_SocketError(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        const char *const error
    ) const
  {
    ASSERT_GT(this->eventHistory_UR_SocketError->size(), index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event UR_SocketError\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_UR_SocketError->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const EventEntry_UR_SocketError& e =
      this->eventHistory_UR_SocketError->at(index);
    ASSERT_STREQ(error, e.error.toChar())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument error at index "
      << index
      << " in history of event UR_SocketError\n"
      << "  Expected: " << error << "\n"
      << "  Actual:   " << e.error.toChar() << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: UR_BindError
  // ----------------------------------------------------------------------

  void UdpReceiverGTestBase ::
    assertEvents_UR_BindError_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_UR_BindError->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event UR_BindError\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_UR_BindError->size() << "\n";
  }

  void UdpReceiverGTestBase ::
    assertEvents_UR_BindError(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        const char *const error
    ) const
  {
    ASSERT_GT(this->eventHistory_UR_BindError->size(), index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event UR_BindError\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_UR_BindError->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const EventEntry_UR_BindError& e =
      this->eventHistory_UR_BindError->at(index);
    ASSERT_STREQ(error, e.error.toChar())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument error at index "
      << index
      << " in history of event UR_BindError\n"
      << "  Expected: " << error << "\n"
      << "  Actual:   " << e.error.toChar() << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: UR_RecvError
  // ----------------------------------------------------------------------

  void UdpReceiverGTestBase ::
    assertEvents_UR_RecvError_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_UR_RecvError->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event UR_RecvError\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_UR_RecvError->size() << "\n";
  }

  void UdpReceiverGTestBase ::
    assertEvents_UR_RecvError(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        const char *const error
    ) const
  {
    ASSERT_GT(this->eventHistory_UR_RecvError->size(), index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event UR_RecvError\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_UR_RecvError->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const EventEntry_UR_RecvError& e =
      this->eventHistory_UR_RecvError->at(index);
    ASSERT_STREQ(error, e.error.toChar())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument error at index "
      << index
      << " in history of event UR_RecvError\n"
      << "  Expected: " << error << "\n"
      << "  Actual:   " << e.error.toChar() << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: UR_DecodeError
  // ----------------------------------------------------------------------

  void UdpReceiverGTestBase ::
    assertEvents_UR_DecodeError_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_UR_DecodeError->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event UR_DecodeError\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_UR_DecodeError->size() << "\n";
  }

  void UdpReceiverGTestBase ::
    assertEvents_UR_DecodeError(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 index,
        UdpReceiverComponentBase::DecodeStage stage,
        const I32 error
    ) const
  {
    ASSERT_GT(this->eventHistory_UR_DecodeError->size(), index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event UR_DecodeError\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_UR_DecodeError->size() << ")\n"
      << "  Actual:   " << index << "\n";
    const EventEntry_UR_DecodeError& e =
      this->eventHistory_UR_DecodeError->at(index);
    ASSERT_EQ(stage, e.stage)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument stage at index "
      << index
      << " in history of event UR_DecodeError\n"
      << "  Expected: " << stage << "\n"
      << "  Actual:   " << e.stage << "\n";
    ASSERT_EQ(error, e.error)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument error at index "
      << index
      << " in history of event UR_DecodeError\n"
      << "  Expected: " << error << "\n"
      << "  Actual:   " << e.error << "\n";
  }

} // end namespace Svc
