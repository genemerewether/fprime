// ======================================================================
// \title  HLRosIface/test/ut/GTestBase.cpp
// \author Auto-generated
// \brief  cpp file for HLRosIface component Google Test harness base class
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

  HLRosIfaceGTestBase ::
    HLRosIfaceGTestBase(
#if FW_OBJECT_NAMES == 1
        const char *const compName,
        const U32 maxHistorySize
#else
        const U32 maxHistorySize
#endif
    ) : 
        HLRosIfaceTesterBase (
#if FW_OBJECT_NAMES == 1
            compName,
#endif
            maxHistorySize
        )
  {

  }

  HLRosIfaceGTestBase ::
    ~HLRosIfaceGTestBase(void)
  {

  }

  // ----------------------------------------------------------------------
  // From ports
  // ----------------------------------------------------------------------

  void HLRosIfaceGTestBase ::
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
  // From port: Tlm
  // ----------------------------------------------------------------------

  void HLRosIfaceGTestBase ::
    assert_from_Tlm_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->fromPortHistory_Tlm->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for from_Tlm\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->fromPortHistory_Tlm->size() << "\n";
  }

  // ----------------------------------------------------------------------
  // From port: Log
  // ----------------------------------------------------------------------

  void HLRosIfaceGTestBase ::
    assert_from_Log_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->fromPortHistory_Log->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for from_Log\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->fromPortHistory_Log->size() << "\n";
  }

  // ----------------------------------------------------------------------
  // From port: LogText
  // ----------------------------------------------------------------------

  void HLRosIfaceGTestBase ::
    assert_from_LogText_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->fromPortHistory_LogText->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for from_LogText\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->fromPortHistory_LogText->size() << "\n";
  }

  // ----------------------------------------------------------------------
  // From port: pingOut
  // ----------------------------------------------------------------------

  void HLRosIfaceGTestBase ::
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
  // From port: Time
  // ----------------------------------------------------------------------

  void HLRosIfaceGTestBase ::
    assert_from_Time_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->fromPortHistory_Time->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for from_Time\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->fromPortHistory_Time->size() << "\n";
  }

  // ----------------------------------------------------------------------
  // From port: ImuStateUpdate
  // ----------------------------------------------------------------------

  void HLRosIfaceGTestBase ::
    assert_from_ImuStateUpdate_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->fromPortHistory_ImuStateUpdate->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for from_ImuStateUpdate\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->fromPortHistory_ImuStateUpdate->size() << "\n";
  }

  // ----------------------------------------------------------------------
  // From port: ActuatorsData
  // ----------------------------------------------------------------------

  void HLRosIfaceGTestBase ::
    assert_from_ActuatorsData_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->fromPortHistory_ActuatorsData->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for from_ActuatorsData\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->fromPortHistory_ActuatorsData->size() << "\n";
  }

} // end namespace HLProc
