// ======================================================================
// \title  R5SpiMasterDriver/test/ut/GTestBase.cpp
// \author Auto-generated
// \brief  cpp file for R5SpiMasterDriver component Google Test harness base class
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

namespace R5 {

  // ----------------------------------------------------------------------
  // Construction and destruction
  // ----------------------------------------------------------------------

  R5SpiMasterDriverGTestBase ::
    R5SpiMasterDriverGTestBase(
#if FW_OBJECT_NAMES == 1
        const char *const compName,
        const U32 maxHistorySize
#else
        const U32 maxHistorySize
#endif
    ) : 
        R5SpiMasterDriverTesterBase (
#if FW_OBJECT_NAMES == 1
            compName,
#endif
            maxHistorySize
        )
  {

  }

  R5SpiMasterDriverGTestBase ::
    ~R5SpiMasterDriverGTestBase(void)
  {

  }

} // end namespace R5
