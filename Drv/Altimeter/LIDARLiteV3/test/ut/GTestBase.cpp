// ======================================================================
// \title  LIDARLiteV3/test/ut/GTestBase.cpp
// \author Auto-generated
// \brief  cpp file for LIDARLiteV3 component Google Test harness base class
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged.
//
// ======================================================================

#include "GTestBase.hpp"

namespace Drv {

  // ----------------------------------------------------------------------
  // Construction and destruction
  // ----------------------------------------------------------------------

  LIDARLiteV3GTestBase ::
    LIDARLiteV3GTestBase(
#if FW_OBJECT_NAMES == 1
        const char *const compName,
        const U32 maxHistorySize
#else
        const U32 maxHistorySize
#endif
    ) :
        LIDARLiteV3TesterBase (
#if FW_OBJECT_NAMES == 1
            compName,
#endif
            maxHistorySize
        )
  {

  }

  LIDARLiteV3GTestBase ::
    ~LIDARLiteV3GTestBase(void)
  {

  }

  // ----------------------------------------------------------------------
  // Telemetry
  // ----------------------------------------------------------------------

  void LIDARLiteV3GTestBase ::
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
  // Channel: LLV3_Distance
  // ----------------------------------------------------------------------

  void LIDARLiteV3GTestBase ::
    assertTlm_LLV3_Distance_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_LLV3_Distance->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel LLV3_Distance\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_LLV3_Distance->size() << "\n";
  }

  void LIDARLiteV3GTestBase ::
    assertTlm_LLV3_Distance(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const F32& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_LLV3_Distance->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel LLV3_Distance\n"
      << "  Expected: Less than size of history ("
      << this->tlmHistory_LLV3_Distance->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_LLV3_Distance& e =
      this->tlmHistory_LLV3_Distance->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel LLV3_Distance\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Events
  // ----------------------------------------------------------------------

  void LIDARLiteV3GTestBase ::
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
  // Event: LLV3_InitComplete
  // ----------------------------------------------------------------------

  void LIDARLiteV3GTestBase ::
    assertEvents_LLV3_InitComplete_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventsSize_LLV3_InitComplete)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event LLV3_InitComplete\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventsSize_LLV3_InitComplete << "\n";
  }

  // ----------------------------------------------------------------------
  // From ports
  // ----------------------------------------------------------------------

  void LIDARLiteV3GTestBase ::
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
  // From port: I2CWriteRead
  // ----------------------------------------------------------------------

  void LIDARLiteV3GTestBase ::
    assert_from_I2CWriteRead_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->fromPortHistory_I2CWriteRead->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for from_I2CWriteRead\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->fromPortHistory_I2CWriteRead->size() << "\n";
  }

  // ----------------------------------------------------------------------
  // From port: I2CWriteReadStatus
  // ----------------------------------------------------------------------

  void LIDARLiteV3GTestBase ::
    assert_from_I2CWriteReadStatus_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->fromPortHistory_I2CWriteReadStatus->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for from_I2CWriteReadStatus\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->fromPortHistory_I2CWriteReadStatus->size() << "\n";
  }

  // ----------------------------------------------------------------------
  // From port: I2CConfig
  // ----------------------------------------------------------------------

  void LIDARLiteV3GTestBase ::
    assert_from_I2CConfig_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->fromPortHistory_I2CConfig->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for from_I2CConfig\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->fromPortHistory_I2CConfig->size() << "\n";
  }

  // ----------------------------------------------------------------------
  // From port: AltimeterSend
  // ----------------------------------------------------------------------

  void LIDARLiteV3GTestBase ::
    assert_from_AltimeterSend_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->fromPortHistory_AltimeterSend->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for from_AltimeterSend\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->fromPortHistory_AltimeterSend->size() << "\n";
  }

} // end namespace Drv
