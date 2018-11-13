// ======================================================================
// \title  HLRouter/test/ut/GTestBase.cpp
// \author Auto-generated
// \brief  cpp file for HLRouter component Google Test harness base class
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

namespace LLProc {

  // ----------------------------------------------------------------------
  // Construction and destruction
  // ----------------------------------------------------------------------

  HLRouterGTestBase ::
    HLRouterGTestBase(
#if FW_OBJECT_NAMES == 1
        const char *const compName,
        const U32 maxHistorySize
#else
        const U32 maxHistorySize
#endif
    ) : 
        HLRouterTesterBase (
#if FW_OBJECT_NAMES == 1
            compName,
#endif
            maxHistorySize
        )
  {

  }

  HLRouterGTestBase ::
    ~HLRouterGTestBase(void)
  {

  }

  // ----------------------------------------------------------------------
  // From ports
  // ----------------------------------------------------------------------

  void HLRouterGTestBase ::
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
  // From port: SerReadPort
  // ----------------------------------------------------------------------

  void HLRouterGTestBase ::
    assert_from_SerReadPort_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->fromPortHistory_SerReadPort->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for from_SerReadPort\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->fromPortHistory_SerReadPort->size() << "\n";
  }

  // ----------------------------------------------------------------------
  // From port: SerWritePort
  // ----------------------------------------------------------------------

  void HLRouterGTestBase ::
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
  // From port: DebugOut
  // ----------------------------------------------------------------------

  void HLRouterGTestBase ::
    assert_from_DebugOut_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->fromPortHistory_DebugOut->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for from_DebugOut\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->fromPortHistory_DebugOut->size() << "\n";
  }

} // end namespace LLProc
