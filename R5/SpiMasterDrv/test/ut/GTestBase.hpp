// ======================================================================
// \title  R5SpiMasterDriver/test/ut/GTestBase.hpp
// \author Auto-generated
// \brief  hpp file for R5SpiMasterDriver component Google Test harness base class
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

#ifndef R5SpiMasterDriver_GTEST_BASE_HPP
#define R5SpiMasterDriver_GTEST_BASE_HPP

#include "TesterBase.hpp"
#include "gtest/gtest.h"

namespace R5 {

  //! \class R5SpiMasterDriverGTestBase
  //! \brief Auto-generated base class for R5SpiMasterDriver component Google Test harness
  //!
  class R5SpiMasterDriverGTestBase :
    public R5SpiMasterDriverTesterBase
  {

    protected:

      // ----------------------------------------------------------------------
      // Construction and destruction
      // ----------------------------------------------------------------------

      //! Construct object R5SpiMasterDriverGTestBase
      //!
      R5SpiMasterDriverGTestBase(
#if FW_OBJECT_NAMES == 1
          const char *const compName, /*!< The component name*/
          const U32 maxHistorySize /*!< The maximum size of each history*/
#else
          const U32 maxHistorySize /*!< The maximum size of each history*/
#endif
      );

      //! Destroy object R5SpiMasterDriverGTestBase
      //!
      virtual ~R5SpiMasterDriverGTestBase(void);

  };

} // end namespace R5

#endif
