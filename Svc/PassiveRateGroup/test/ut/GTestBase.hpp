// ======================================================================
// \title  PassiveRateGroup/test/ut/GTestBase.hpp
// \author Auto-generated
// \brief  hpp file for PassiveRateGroup component Google Test harness base class
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

#ifndef PassiveRateGroup_GTEST_BASE_HPP
#define PassiveRateGroup_GTEST_BASE_HPP

#include "TesterBase.hpp"
#include "gtest/gtest.h"

// ----------------------------------------------------------------------
// Macros for typed user from port history assertions
// ----------------------------------------------------------------------

#define ASSERT_FROM_PORT_HISTORY_SIZE(size) \
  this->assertFromPortHistory_size(__FILE__, __LINE__, size)

#define ASSERT_from_RateGroupMemberOut_SIZE(size) \
  this->assert_from_RateGroupMemberOut_size(__FILE__, __LINE__, size)

#define ASSERT_from_RateGroupMemberOut(index, _context) \
  { \
    ASSERT_GT(this->fromPortHistory_RateGroupMemberOut->size(), static_cast<U32>(index)) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Index into history of from_RateGroupMemberOut\n" \
    << "  Expected: Less than size of history (" \
    << this->fromPortHistory_RateGroupMemberOut->size() << ")\n" \
    << "  Actual:   " << index << "\n"; \
    const FromPortEntry_RateGroupMemberOut& _e = \
      this->fromPortHistory_RateGroupMemberOut->at(index); \
    ASSERT_EQ(_context, _e.context) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument context at index " \
    << index \
    << " in history of from_RateGroupMemberOut\n" \
    << "  Expected: " << _context << "\n" \
    << "  Actual:   " << _e.context << "\n"; \
  }

namespace Svc {

  //! \class PassiveRateGroupGTestBase
  //! \brief Auto-generated base class for PassiveRateGroup component Google Test harness
  //!
  class PassiveRateGroupGTestBase :
    public PassiveRateGroupTesterBase
  {

    protected:

      // ----------------------------------------------------------------------
      // Construction and destruction
      // ----------------------------------------------------------------------

      //! Construct object PassiveRateGroupGTestBase
      //!
      PassiveRateGroupGTestBase(
#if FW_OBJECT_NAMES == 1
          const char *const compName, /*!< The component name*/
          const U32 maxHistorySize /*!< The maximum size of each history*/
#else
          const U32 maxHistorySize /*!< The maximum size of each history*/
#endif
      );

      //! Destroy object PassiveRateGroupGTestBase
      //!
      virtual ~PassiveRateGroupGTestBase(void);

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
      // From port: RateGroupMemberOut 
      // ----------------------------------------------------------------------

      void assert_from_RateGroupMemberOut_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

  };

} // end namespace Svc

#endif
