// ======================================================================
// \title  BlspSpiDriver/test/ut/GTestBase.cpp
// \author Auto-generated
// \brief  cpp file for BlspSpiDriver component Google Test harness base class
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

  BlspSpiDriverGTestBase ::
    BlspSpiDriverGTestBase(
#if FW_OBJECT_NAMES == 1
        const char *const compName,
        const U32 maxHistorySize
#else
        const U32 maxHistorySize
#endif
    ) : 
        BlspSpiDriverTesterBase (
#if FW_OBJECT_NAMES == 1
            compName,
#endif
            maxHistorySize
        )
  {

  }

  BlspSpiDriverGTestBase ::
    ~BlspSpiDriverGTestBase(void)
  {

  }

  // ----------------------------------------------------------------------
  // Telemetry
  // ----------------------------------------------------------------------

  void BlspSpiDriverGTestBase ::
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
  // Channel: SPI_Bytes
  // ----------------------------------------------------------------------

  void BlspSpiDriverGTestBase ::
    assertTlm_SPI_Bytes_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_SPI_Bytes->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel SPI_Bytes\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_SPI_Bytes->size() << "\n";
  }

  void BlspSpiDriverGTestBase ::
    assertTlm_SPI_Bytes(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const U32& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_SPI_Bytes->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel SPI_Bytes\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_SPI_Bytes->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_SPI_Bytes& e =
      this->tlmHistory_SPI_Bytes->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel SPI_Bytes\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Events
  // ----------------------------------------------------------------------

  void BlspSpiDriverGTestBase ::
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
  // Event: SPI_OpenError
  // ----------------------------------------------------------------------

  void BlspSpiDriverGTestBase ::
    assertEvents_SPI_OpenError_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_SPI_OpenError->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event SPI_OpenError\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_SPI_OpenError->size() << "\n";
  }

  void BlspSpiDriverGTestBase ::
    assertEvents_SPI_OpenError(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const I32 device,
        const I32 error
    ) const
  {
    ASSERT_GT(this->eventHistory_SPI_OpenError->size(), __index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event SPI_OpenError\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_SPI_OpenError->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const EventEntry_SPI_OpenError& e =
      this->eventHistory_SPI_OpenError->at(__index);
    ASSERT_EQ(device, e.device)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument device at index "
      << __index
      << " in history of event SPI_OpenError\n"
      << "  Expected: " << device << "\n"
      << "  Actual:   " << e.device << "\n";
    ASSERT_EQ(error, e.error)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument error at index "
      << __index
      << " in history of event SPI_OpenError\n"
      << "  Expected: " << error << "\n"
      << "  Actual:   " << e.error << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: SPI_ConfigError
  // ----------------------------------------------------------------------

  void BlspSpiDriverGTestBase ::
    assertEvents_SPI_ConfigError_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_SPI_ConfigError->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event SPI_ConfigError\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_SPI_ConfigError->size() << "\n";
  }

  void BlspSpiDriverGTestBase ::
    assertEvents_SPI_ConfigError(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const I32 device,
        const I32 error
    ) const
  {
    ASSERT_GT(this->eventHistory_SPI_ConfigError->size(), __index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event SPI_ConfigError\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_SPI_ConfigError->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const EventEntry_SPI_ConfigError& e =
      this->eventHistory_SPI_ConfigError->at(__index);
    ASSERT_EQ(device, e.device)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument device at index "
      << __index
      << " in history of event SPI_ConfigError\n"
      << "  Expected: " << device << "\n"
      << "  Actual:   " << e.device << "\n";
    ASSERT_EQ(error, e.error)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument error at index "
      << __index
      << " in history of event SPI_ConfigError\n"
      << "  Expected: " << error << "\n"
      << "  Actual:   " << e.error << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: SPI_WriteError
  // ----------------------------------------------------------------------

  void BlspSpiDriverGTestBase ::
    assertEvents_SPI_WriteError_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_SPI_WriteError->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event SPI_WriteError\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_SPI_WriteError->size() << "\n";
  }

  void BlspSpiDriverGTestBase ::
    assertEvents_SPI_WriteError(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const I32 device,
        const I32 error
    ) const
  {
    ASSERT_GT(this->eventHistory_SPI_WriteError->size(), __index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event SPI_WriteError\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_SPI_WriteError->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const EventEntry_SPI_WriteError& e =
      this->eventHistory_SPI_WriteError->at(__index);
    ASSERT_EQ(device, e.device)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument device at index "
      << __index
      << " in history of event SPI_WriteError\n"
      << "  Expected: " << device << "\n"
      << "  Actual:   " << e.device << "\n";
    ASSERT_EQ(error, e.error)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument error at index "
      << __index
      << " in history of event SPI_WriteError\n"
      << "  Expected: " << error << "\n"
      << "  Actual:   " << e.error << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: SPI_PortOpened
  // ----------------------------------------------------------------------

  void BlspSpiDriverGTestBase ::
    assertEvents_SPI_PortOpened_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_SPI_PortOpened->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event SPI_PortOpened\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_SPI_PortOpened->size() << "\n";
  }

  void BlspSpiDriverGTestBase ::
    assertEvents_SPI_PortOpened(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const I32 device
    ) const
  {
    ASSERT_GT(this->eventHistory_SPI_PortOpened->size(), __index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event SPI_PortOpened\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_SPI_PortOpened->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const EventEntry_SPI_PortOpened& e =
      this->eventHistory_SPI_PortOpened->at(__index);
    ASSERT_EQ(device, e.device)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument device at index "
      << __index
      << " in history of event SPI_PortOpened\n"
      << "  Expected: " << device << "\n"
      << "  Actual:   " << e.device << "\n";
  }

} // end namespace SnapdragonFlight
