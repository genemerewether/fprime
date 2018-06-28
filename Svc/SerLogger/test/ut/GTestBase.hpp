// ======================================================================
// \title  SerLogger/test/ut/GTestBase.hpp
// \author Auto-generated
// \brief  hpp file for SerLogger component Google Test harness base class
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

#ifndef SerLogger_GTEST_BASE_HPP
#define SerLogger_GTEST_BASE_HPP

#include "TesterBase.hpp"
#include "gtest/gtest.h"

// ----------------------------------------------------------------------
// Macros for typed user from port history assertions
// ----------------------------------------------------------------------

#define ASSERT_FROM_PORT_HISTORY_SIZE(size) \
  this->assertFromPortHistory_size(__FILE__, __LINE__, size)

#define ASSERT_from_LogOut_SIZE(size) \
  this->assert_from_LogOut_size(__FILE__, __LINE__, size)

#define ASSERT_from_LogOut(index, _data) \
  { \
    ASSERT_GT(this->fromPortHistory_LogOut->size(), static_cast<U32>(index)) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Index into history of from_LogOut\n" \
    << "  Expected: Less than size of history (" \
    << this->fromPortHistory_LogOut->size() << ")\n" \
    << "  Actual:   " << index << "\n"; \
    const FromPortEntry_LogOut& _e = \
      this->fromPortHistory_LogOut->at(index); \
    ASSERT_EQ(_data, _e.data) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument data at index " \
    << index \
    << " in history of from_LogOut\n" \
    << "  Expected: " << _data << "\n" \
    << "  Actual:   " << _e.data << "\n"; \
  }

namespace Svc {

  //! \class SerLoggerGTestBase
  //! \brief Auto-generated base class for SerLogger component Google Test harness
  //!
  class SerLoggerGTestBase :
    public SerLoggerTesterBase
  {

    protected:

      // ----------------------------------------------------------------------
      // Construction and destruction
      // ----------------------------------------------------------------------

      //! Construct object SerLoggerGTestBase
      //!
      SerLoggerGTestBase(
#if FW_OBJECT_NAMES == 1
          const char *const compName, /*!< The component name*/
          const U32 maxHistorySize /*!< The maximum size of each history*/
#else
          const U32 maxHistorySize /*!< The maximum size of each history*/
#endif
      );

      //! Destroy object SerLoggerGTestBase
      //!
      virtual ~SerLoggerGTestBase(void);

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
      // From port: LogOut 
      // ----------------------------------------------------------------------

      void assert_from_LogOut_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

  };

} // end namespace Svc

#endif
