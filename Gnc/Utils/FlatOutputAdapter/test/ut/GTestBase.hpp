// ======================================================================
// \title  FlatOutputAdapter/test/ut/GTestBase.hpp
// \author Auto-generated
// \brief  hpp file for FlatOutputAdapter component Google Test harness base class
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged.
//
// ======================================================================

#ifndef FlatOutputAdapter_GTEST_BASE_HPP
#define FlatOutputAdapter_GTEST_BASE_HPP

#include "TesterBase.hpp"
#include "gtest/gtest.h"

// ----------------------------------------------------------------------
// Macros for typed user from port history assertions
// ----------------------------------------------------------------------

#define ASSERT_FROM_PORT_HISTORY_SIZE(size) \
  this->assertFromPortHistory_size(__FILE__, __LINE__, size)

#define ASSERT_from_flatOutput_SIZE(size) \
  this->assert_from_flatOutput_size(__FILE__, __LINE__, size)

#define ASSERT_from_flatOutput(index, _FlatOutput) \
  { \
    ASSERT_GT(this->fromPortHistory_flatOutput->size(), static_cast<U32>(index)) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Index into history of from_flatOutput\n" \
    << "  Expected: Less than size of history (" \
    << this->fromPortHistory_flatOutput->size() << ")\n" \
    << "  Actual:   " << index << "\n"; \
    const FromPortEntry_flatOutput& _e = \
      this->fromPortHistory_flatOutput->at(index); \
    ASSERT_EQ(_FlatOutput, _e.FlatOutput) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument FlatOutput at index " \
    << index \
    << " in history of from_flatOutput\n" \
    << "  Expected: " << _FlatOutput << "\n" \
    << "  Actual:   " << _e.FlatOutput << "\n"; \
  }

namespace Gnc {

  //! \class FlatOutputAdapterGTestBase
  //! \brief Auto-generated base class for FlatOutputAdapter component Google Test harness
  //!
  class FlatOutputAdapterGTestBase :
    public FlatOutputAdapterTesterBase
  {

    protected:

      // ----------------------------------------------------------------------
      // Construction and destruction
      // ----------------------------------------------------------------------

      //! Construct object FlatOutputAdapterGTestBase
      //!
      FlatOutputAdapterGTestBase(
#if FW_OBJECT_NAMES == 1
          const char *const compName, /*!< The component name*/
          const U32 maxHistorySize /*!< The maximum size of each history*/
#else
          const U32 maxHistorySize /*!< The maximum size of each history*/
#endif
      );

      //! Destroy object FlatOutputAdapterGTestBase
      //!
      virtual ~FlatOutputAdapterGTestBase(void);

    protected:

      // ----------------------------------------------------------------------
      // From ports
      // ----------------------------------------------------------------------

      void assertFromPortHistory_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // From port: flatOutput
      // ----------------------------------------------------------------------

      void assert_from_flatOutput_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

  };

} // end namespace Gnc

#endif
