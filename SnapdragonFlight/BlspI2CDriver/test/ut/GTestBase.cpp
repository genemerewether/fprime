// ======================================================================
// \title  BlspI2CDriver/test/ut/GTestBase.cpp
// \author Auto-generated
// \brief  cpp file for BlspI2CDriver component Google Test harness base class
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

  BlspI2CDriverGTestBase ::
    BlspI2CDriverGTestBase(
#if FW_OBJECT_NAMES == 1
        const char *const compName,
        const U32 maxHistorySize
#else
        const U32 maxHistorySize
#endif
    ) : 
        BlspI2CDriverTesterBase (
#if FW_OBJECT_NAMES == 1
            compName,
#endif
            maxHistorySize
        )
  {

  }

  BlspI2CDriverGTestBase ::
    ~BlspI2CDriverGTestBase(void)
  {

  }

  // ----------------------------------------------------------------------
  // Telemetry
  // ----------------------------------------------------------------------

  void BlspI2CDriverGTestBase ::
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
  // Channel: I2C_ReadBytes
  // ----------------------------------------------------------------------

  void BlspI2CDriverGTestBase ::
    assertTlm_I2C_ReadBytes_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_I2C_ReadBytes->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel I2C_ReadBytes\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_I2C_ReadBytes->size() << "\n";
  }

  void BlspI2CDriverGTestBase ::
    assertTlm_I2C_ReadBytes(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const U32& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_I2C_ReadBytes->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel I2C_ReadBytes\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_I2C_ReadBytes->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_I2C_ReadBytes& e =
      this->tlmHistory_I2C_ReadBytes->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel I2C_ReadBytes\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: I2C_WriteBytes
  // ----------------------------------------------------------------------

  void BlspI2CDriverGTestBase ::
    assertTlm_I2C_WriteBytes_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_I2C_WriteBytes->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel I2C_WriteBytes\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_I2C_WriteBytes->size() << "\n";
  }

  void BlspI2CDriverGTestBase ::
    assertTlm_I2C_WriteBytes(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const U32& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_I2C_WriteBytes->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel I2C_WriteBytes\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_I2C_WriteBytes->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_I2C_WriteBytes& e =
      this->tlmHistory_I2C_WriteBytes->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel I2C_WriteBytes\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Events
  // ----------------------------------------------------------------------

  void BlspI2CDriverGTestBase ::
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
  // Event: I2C_OpenError
  // ----------------------------------------------------------------------

  void BlspI2CDriverGTestBase ::
    assertEvents_I2C_OpenError_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_I2C_OpenError->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event I2C_OpenError\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_I2C_OpenError->size() << "\n";
  }

  void BlspI2CDriverGTestBase ::
    assertEvents_I2C_OpenError(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const I32 device,
        const I32 error
    ) const
  {
    ASSERT_GT(this->eventHistory_I2C_OpenError->size(), __index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event I2C_OpenError\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_I2C_OpenError->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const EventEntry_I2C_OpenError& e =
      this->eventHistory_I2C_OpenError->at(__index);
    ASSERT_EQ(device, e.device)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument device at index "
      << __index
      << " in history of event I2C_OpenError\n"
      << "  Expected: " << device << "\n"
      << "  Actual:   " << e.device << "\n";
    ASSERT_EQ(error, e.error)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument error at index "
      << __index
      << " in history of event I2C_OpenError\n"
      << "  Expected: " << error << "\n"
      << "  Actual:   " << e.error << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: I2C_ConfigError
  // ----------------------------------------------------------------------

  void BlspI2CDriverGTestBase ::
    assertEvents_I2C_ConfigError_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_I2C_ConfigError->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event I2C_ConfigError\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_I2C_ConfigError->size() << "\n";
  }

  void BlspI2CDriverGTestBase ::
    assertEvents_I2C_ConfigError(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const I32 device,
        const I32 error
    ) const
  {
    ASSERT_GT(this->eventHistory_I2C_ConfigError->size(), __index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event I2C_ConfigError\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_I2C_ConfigError->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const EventEntry_I2C_ConfigError& e =
      this->eventHistory_I2C_ConfigError->at(__index);
    ASSERT_EQ(device, e.device)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument device at index "
      << __index
      << " in history of event I2C_ConfigError\n"
      << "  Expected: " << device << "\n"
      << "  Actual:   " << e.device << "\n";
    ASSERT_EQ(error, e.error)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument error at index "
      << __index
      << " in history of event I2C_ConfigError\n"
      << "  Expected: " << error << "\n"
      << "  Actual:   " << e.error << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: I2C_ReadWriteError
  // ----------------------------------------------------------------------

  void BlspI2CDriverGTestBase ::
    assertEvents_I2C_ReadWriteError_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_I2C_ReadWriteError->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event I2C_ReadWriteError\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_I2C_ReadWriteError->size() << "\n";
  }

  void BlspI2CDriverGTestBase ::
    assertEvents_I2C_ReadWriteError(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const I32 device,
        const U32 addr,
        const I32 error,
        const I32 bytesAct,
        const I32 bytesDes
    ) const
  {
    ASSERT_GT(this->eventHistory_I2C_ReadWriteError->size(), __index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event I2C_ReadWriteError\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_I2C_ReadWriteError->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const EventEntry_I2C_ReadWriteError& e =
      this->eventHistory_I2C_ReadWriteError->at(__index);
    ASSERT_EQ(device, e.device)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument device at index "
      << __index
      << " in history of event I2C_ReadWriteError\n"
      << "  Expected: " << device << "\n"
      << "  Actual:   " << e.device << "\n";
    ASSERT_EQ(addr, e.addr)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument addr at index "
      << __index
      << " in history of event I2C_ReadWriteError\n"
      << "  Expected: " << addr << "\n"
      << "  Actual:   " << e.addr << "\n";
    ASSERT_EQ(error, e.error)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument error at index "
      << __index
      << " in history of event I2C_ReadWriteError\n"
      << "  Expected: " << error << "\n"
      << "  Actual:   " << e.error << "\n";
    ASSERT_EQ(bytesAct, e.bytesAct)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument bytesAct at index "
      << __index
      << " in history of event I2C_ReadWriteError\n"
      << "  Expected: " << bytesAct << "\n"
      << "  Actual:   " << e.bytesAct << "\n";
    ASSERT_EQ(bytesDes, e.bytesDes)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument bytesDes at index "
      << __index
      << " in history of event I2C_ReadWriteError\n"
      << "  Expected: " << bytesDes << "\n"
      << "  Actual:   " << e.bytesDes << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: I2C_WriteError
  // ----------------------------------------------------------------------

  void BlspI2CDriverGTestBase ::
    assertEvents_I2C_WriteError_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_I2C_WriteError->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event I2C_WriteError\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_I2C_WriteError->size() << "\n";
  }

  void BlspI2CDriverGTestBase ::
    assertEvents_I2C_WriteError(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const I32 device,
        const U32 addr,
        const I32 error,
        const I32 bytesAct,
        const I32 bytesDes
    ) const
  {
    ASSERT_GT(this->eventHistory_I2C_WriteError->size(), __index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event I2C_WriteError\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_I2C_WriteError->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const EventEntry_I2C_WriteError& e =
      this->eventHistory_I2C_WriteError->at(__index);
    ASSERT_EQ(device, e.device)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument device at index "
      << __index
      << " in history of event I2C_WriteError\n"
      << "  Expected: " << device << "\n"
      << "  Actual:   " << e.device << "\n";
    ASSERT_EQ(addr, e.addr)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument addr at index "
      << __index
      << " in history of event I2C_WriteError\n"
      << "  Expected: " << addr << "\n"
      << "  Actual:   " << e.addr << "\n";
    ASSERT_EQ(error, e.error)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument error at index "
      << __index
      << " in history of event I2C_WriteError\n"
      << "  Expected: " << error << "\n"
      << "  Actual:   " << e.error << "\n";
    ASSERT_EQ(bytesAct, e.bytesAct)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument bytesAct at index "
      << __index
      << " in history of event I2C_WriteError\n"
      << "  Expected: " << bytesAct << "\n"
      << "  Actual:   " << e.bytesAct << "\n";
    ASSERT_EQ(bytesDes, e.bytesDes)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument bytesDes at index "
      << __index
      << " in history of event I2C_WriteError\n"
      << "  Expected: " << bytesDes << "\n"
      << "  Actual:   " << e.bytesDes << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: I2C_PortOpened
  // ----------------------------------------------------------------------

  void BlspI2CDriverGTestBase ::
    assertEvents_I2C_PortOpened_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_I2C_PortOpened->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event I2C_PortOpened\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_I2C_PortOpened->size() << "\n";
  }

  void BlspI2CDriverGTestBase ::
    assertEvents_I2C_PortOpened(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const I32 device
    ) const
  {
    ASSERT_GT(this->eventHistory_I2C_PortOpened->size(), __index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event I2C_PortOpened\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_I2C_PortOpened->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const EventEntry_I2C_PortOpened& e =
      this->eventHistory_I2C_PortOpened->at(__index);
    ASSERT_EQ(device, e.device)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument device at index "
      << __index
      << " in history of event I2C_PortOpened\n"
      << "  Expected: " << device << "\n"
      << "  Actual:   " << e.device << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: I2C_ReadError
  // ----------------------------------------------------------------------

  void BlspI2CDriverGTestBase ::
    assertEvents_I2C_ReadError_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_I2C_ReadError->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event I2C_ReadError\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_I2C_ReadError->size() << "\n";
  }

  void BlspI2CDriverGTestBase ::
    assertEvents_I2C_ReadError(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const I32 device,
        const U32 addr,
        const I32 error,
        const I32 bytesAct,
        const I32 bytesDes
    ) const
  {
    ASSERT_GT(this->eventHistory_I2C_ReadError->size(), __index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event I2C_ReadError\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_I2C_ReadError->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const EventEntry_I2C_ReadError& e =
      this->eventHistory_I2C_ReadError->at(__index);
    ASSERT_EQ(device, e.device)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument device at index "
      << __index
      << " in history of event I2C_ReadError\n"
      << "  Expected: " << device << "\n"
      << "  Actual:   " << e.device << "\n";
    ASSERT_EQ(addr, e.addr)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument addr at index "
      << __index
      << " in history of event I2C_ReadError\n"
      << "  Expected: " << addr << "\n"
      << "  Actual:   " << e.addr << "\n";
    ASSERT_EQ(error, e.error)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument error at index "
      << __index
      << " in history of event I2C_ReadError\n"
      << "  Expected: " << error << "\n"
      << "  Actual:   " << e.error << "\n";
    ASSERT_EQ(bytesAct, e.bytesAct)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument bytesAct at index "
      << __index
      << " in history of event I2C_ReadError\n"
      << "  Expected: " << bytesAct << "\n"
      << "  Actual:   " << e.bytesAct << "\n";
    ASSERT_EQ(bytesDes, e.bytesDes)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument bytesDes at index "
      << __index
      << " in history of event I2C_ReadError\n"
      << "  Expected: " << bytesDes << "\n"
      << "  Actual:   " << e.bytesDes << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: I2C_BadReadWriteCall
  // ----------------------------------------------------------------------

  void BlspI2CDriverGTestBase ::
    assertEvents_I2C_BadReadWriteCall_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_I2C_BadReadWriteCall->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event I2C_BadReadWriteCall\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_I2C_BadReadWriteCall->size() << "\n";
  }

  void BlspI2CDriverGTestBase ::
    assertEvents_I2C_BadReadWriteCall(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const I32 device
    ) const
  {
    ASSERT_GT(this->eventHistory_I2C_BadReadWriteCall->size(), __index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event I2C_BadReadWriteCall\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_I2C_BadReadWriteCall->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const EventEntry_I2C_BadReadWriteCall& e =
      this->eventHistory_I2C_BadReadWriteCall->at(__index);
    ASSERT_EQ(device, e.device)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument device at index "
      << __index
      << " in history of event I2C_BadReadWriteCall\n"
      << "  Expected: " << device << "\n"
      << "  Actual:   " << e.device << "\n";
  }

} // end namespace SnapdragonFlight
