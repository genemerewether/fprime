// ======================================================================
// \title  LIDARLiteV3/test/ut/GTestBase.cpp
// \author Auto-generated
// \brief  cpp file for LIDARLiteV3 component Google Test harness base class
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

} // end namespace Drv
