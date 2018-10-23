// ======================================================================
// \title  HLRouter/test/ut/GTestBase.hpp
// \author Auto-generated
// \brief  hpp file for HLRouter component Google Test harness base class
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

#ifndef HLRouter_GTEST_BASE_HPP
#define HLRouter_GTEST_BASE_HPP

#include "TesterBase.hpp"
#include "gtest/gtest.h"

// ----------------------------------------------------------------------
// Macros for typed user from port history assertions
// ----------------------------------------------------------------------

#define ASSERT_FROM_PORT_HISTORY_SIZE(size) \
  this->assertFromPortHistory_size(__FILE__, __LINE__, size)

#define ASSERT_from_SerReadPort_SIZE(size) \
  this->assert_from_SerReadPort_size(__FILE__, __LINE__, size)

#define ASSERT_from_SerReadPort(index, _serBuffer, _status) \
  { \
    ASSERT_GT(this->fromPortHistory_SerReadPort->size(), static_cast<U32>(index)) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Index into history of from_SerReadPort\n" \
    << "  Expected: Less than size of history (" \
    << this->fromPortHistory_SerReadPort->size() << ")\n" \
    << "  Actual:   " << index << "\n"; \
    const FromPortEntry_SerReadPort& _e = \
      this->fromPortHistory_SerReadPort->at(index); \
    ASSERT_EQ(_serBuffer, _e.serBuffer) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument serBuffer at index " \
    << index \
    << " in history of from_SerReadPort\n" \
    << "  Expected: " << _serBuffer << "\n" \
    << "  Actual:   " << _e.serBuffer << "\n"; \
    ASSERT_EQ(_status, _e.status) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument status at index " \
    << index \
    << " in history of from_SerReadPort\n" \
    << "  Expected: " << _status << "\n" \
    << "  Actual:   " << _e.status << "\n"; \
  }

#define ASSERT_from_SerWritePort_SIZE(size) \
  this->assert_from_SerWritePort_size(__FILE__, __LINE__, size)

#define ASSERT_from_SerWritePort(index, _serBuffer) \
  { \
    ASSERT_GT(this->fromPortHistory_SerWritePort->size(), static_cast<U32>(index)) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Index into history of from_SerWritePort\n" \
    << "  Expected: Less than size of history (" \
    << this->fromPortHistory_SerWritePort->size() << ")\n" \
    << "  Actual:   " << index << "\n"; \
    const FromPortEntry_SerWritePort& _e = \
      this->fromPortHistory_SerWritePort->at(index); \
    ASSERT_EQ(_serBuffer, _e.serBuffer) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument serBuffer at index " \
    << index \
    << " in history of from_SerWritePort\n" \
    << "  Expected: " << _serBuffer << "\n" \
    << "  Actual:   " << _e.serBuffer << "\n"; \
  }

#define ASSERT_from_DebugOut_SIZE(size) \
  this->assert_from_DebugOut_size(__FILE__, __LINE__, size)

#define ASSERT_from_DebugOut(index, _string, _size) \
  { \
    ASSERT_GT(this->fromPortHistory_DebugOut->size(), static_cast<U32>(index)) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Index into history of from_DebugOut\n" \
    << "  Expected: Less than size of history (" \
    << this->fromPortHistory_DebugOut->size() << ")\n" \
    << "  Actual:   " << index << "\n"; \
    const FromPortEntry_DebugOut& _e = \
      this->fromPortHistory_DebugOut->at(index); \
    ASSERT_EQ(_string, _e.string) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument string at index " \
    << index \
    << " in history of from_DebugOut\n" \
    << "  Expected: " << _string << "\n" \
    << "  Actual:   " << _e.string << "\n"; \
    ASSERT_EQ(_size, _e.size) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument size at index " \
    << index \
    << " in history of from_DebugOut\n" \
    << "  Expected: " << _size << "\n" \
    << "  Actual:   " << _e.size << "\n"; \
  }

namespace LLProc {

  //! \class HLRouterGTestBase
  //! \brief Auto-generated base class for HLRouter component Google Test harness
  //!
  class HLRouterGTestBase :
    public HLRouterTesterBase
  {

    protected:

      // ----------------------------------------------------------------------
      // Construction and destruction
      // ----------------------------------------------------------------------

      //! Construct object HLRouterGTestBase
      //!
      HLRouterGTestBase(
#if FW_OBJECT_NAMES == 1
          const char *const compName, /*!< The component name*/
          const U32 maxHistorySize /*!< The maximum size of each history*/
#else
          const U32 maxHistorySize /*!< The maximum size of each history*/
#endif
      );

      //! Destroy object HLRouterGTestBase
      //!
      virtual ~HLRouterGTestBase(void);

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
      // From port: SerReadPort 
      // ----------------------------------------------------------------------

      void assert_from_SerReadPort_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // From port: SerWritePort 
      // ----------------------------------------------------------------------

      void assert_from_SerWritePort_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // From port: DebugOut 
      // ----------------------------------------------------------------------

      void assert_from_DebugOut_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

  };

} // end namespace LLProc

#endif
