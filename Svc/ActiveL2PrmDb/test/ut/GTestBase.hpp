// ======================================================================
// \title  ActiveL2PrmDb/test/ut/GTestBase.hpp
// \author Auto-generated
// \brief  hpp file for ActiveL2PrmDb component Google Test harness base class
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

#ifndef ActiveL2PrmDb_GTEST_BASE_HPP
#define ActiveL2PrmDb_GTEST_BASE_HPP

#include "TesterBase.hpp"
#include "gtest/gtest.h"

// ----------------------------------------------------------------------
// Macros for command history assertions
// ----------------------------------------------------------------------

#define ASSERT_CMD_RESPONSE_SIZE(size) \
  this->assertCmdResponse_size(__FILE__, __LINE__, size)

#define ASSERT_CMD_RESPONSE(index, opCode, cmdSeq, response) \
  this->assertCmdResponse(__FILE__, __LINE__, index, opCode, cmdSeq, response)

// ----------------------------------------------------------------------
// Macros for event history assertions 
// ----------------------------------------------------------------------

#define ASSERT_EVENTS_SIZE(size) \
  this->assertEvents_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_PrmIdNotFound_SIZE(size) \
  this->assertEvents_PrmIdNotFound_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_PrmIdNotFound(index, _Id) \
  this->assertEvents_PrmIdNotFound(__FILE__, __LINE__, index, _Id)

#define ASSERT_EVENTS_PrmIdUpdated_SIZE(size) \
  this->assertEvents_PrmIdUpdated_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_PrmIdUpdated(index, _Id) \
  this->assertEvents_PrmIdUpdated(__FILE__, __LINE__, index, _Id)

#define ASSERT_EVENTS_PrmDbFull_SIZE(size) \
  this->assertEvents_PrmDbFull_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_PrmDbFull(index, _Id) \
  this->assertEvents_PrmDbFull(__FILE__, __LINE__, index, _Id)

#define ASSERT_EVENTS_PrmIdAdded_SIZE(size) \
  this->assertEvents_PrmIdAdded_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_PrmIdAdded(index, _Id) \
  this->assertEvents_PrmIdAdded(__FILE__, __LINE__, index, _Id)

#define ASSERT_EVENTS_PrmSendTooLarge_SIZE(size) \
  this->assertEvents_PrmSendTooLarge_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_PrmSendTooLarge(index, _prmId, _prmSize) \
  this->assertEvents_PrmSendTooLarge(__FILE__, __LINE__, index, _prmId, _prmSize)

#define ASSERT_EVENTS_PrmSendBufferFull_SIZE(size) \
  this->assertEvents_PrmSendBufferFull_size(__FILE__, __LINE__, size)

// ----------------------------------------------------------------------
// Macros for typed user from port history assertions
// ----------------------------------------------------------------------

#define ASSERT_FROM_PORT_HISTORY_SIZE(size) \
  this->assertFromPortHistory_size(__FILE__, __LINE__, size)

#define ASSERT_from_sendPrm_SIZE(size) \
  this->assert_from_sendPrm_size(__FILE__, __LINE__, size)

#define ASSERT_from_sendPrm(index, _morePrms, _val) \
  { \
    ASSERT_GT(this->fromPortHistory_sendPrm->size(), static_cast<U32>(index)) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Index into history of from_sendPrm\n" \
    << "  Expected: Less than size of history (" \
    << this->fromPortHistory_sendPrm->size() << ")\n" \
    << "  Actual:   " << index << "\n"; \
    const FromPortEntry_sendPrm& _e = \
      this->fromPortHistory_sendPrm->at(index); \
    ASSERT_EQ(_morePrms, _e.morePrms) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument morePrms at index " \
    << index \
    << " in history of from_sendPrm\n" \
    << "  Expected: " << _morePrms << "\n" \
    << "  Actual:   " << _e.morePrms << "\n"; \
    ASSERT_EQ(_val, _e.val) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument val at index " \
    << index \
    << " in history of from_sendPrm\n" \
    << "  Expected: " << _val << "\n" \
    << "  Actual:   " << _e.val << "\n"; \
  }

#define ASSERT_from_recvPrmReady_SIZE(size) \
  this->assert_from_recvPrmReady_size(__FILE__, __LINE__, size)

#define ASSERT_from_recvPrmReady(index, _maxSize, _reload) \
  { \
    ASSERT_GT(this->fromPortHistory_recvPrmReady->size(), static_cast<U32>(index)) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Index into history of from_recvPrmReady\n" \
    << "  Expected: Less than size of history (" \
    << this->fromPortHistory_recvPrmReady->size() << ")\n" \
    << "  Actual:   " << index << "\n"; \
    const FromPortEntry_recvPrmReady& _e = \
      this->fromPortHistory_recvPrmReady->at(index); \
    ASSERT_EQ(_maxSize, _e.maxSize) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument maxSize at index " \
    << index \
    << " in history of from_recvPrmReady\n" \
    << "  Expected: " << _maxSize << "\n" \
    << "  Actual:   " << _e.maxSize << "\n"; \
    ASSERT_EQ(_reload, _e.reload) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument reload at index " \
    << index \
    << " in history of from_recvPrmReady\n" \
    << "  Expected: " << _reload << "\n" \
    << "  Actual:   " << _e.reload << "\n"; \
  }

#define ASSERT_from_pingOut_SIZE(size) \
  this->assert_from_pingOut_size(__FILE__, __LINE__, size)

#define ASSERT_from_pingOut(index, _key) \
  { \
    ASSERT_GT(this->fromPortHistory_pingOut->size(), static_cast<U32>(index)) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Index into history of from_pingOut\n" \
    << "  Expected: Less than size of history (" \
    << this->fromPortHistory_pingOut->size() << ")\n" \
    << "  Actual:   " << index << "\n"; \
    const FromPortEntry_pingOut& _e = \
      this->fromPortHistory_pingOut->at(index); \
    ASSERT_EQ(_key, _e.key) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument key at index " \
    << index \
    << " in history of from_pingOut\n" \
    << "  Expected: " << _key << "\n" \
    << "  Actual:   " << _e.key << "\n"; \
  }

namespace Svc {

  //! \class ActiveL2PrmDbGTestBase
  //! \brief Auto-generated base class for ActiveL2PrmDb component Google Test harness
  //!
  class ActiveL2PrmDbGTestBase :
    public ActiveL2PrmDbTesterBase
  {

    protected:

      // ----------------------------------------------------------------------
      // Construction and destruction
      // ----------------------------------------------------------------------

      //! Construct object ActiveL2PrmDbGTestBase
      //!
      ActiveL2PrmDbGTestBase(
#if FW_OBJECT_NAMES == 1
          const char *const compName, /*!< The component name*/
          const U32 maxHistorySize /*!< The maximum size of each history*/
#else
          const U32 maxHistorySize /*!< The maximum size of each history*/
#endif
      );

      //! Destroy object ActiveL2PrmDbGTestBase
      //!
      virtual ~ActiveL2PrmDbGTestBase(void);

    protected:

      // ----------------------------------------------------------------------
      // Commands
      // ----------------------------------------------------------------------

      //! Assert size of command response history
      //!
      void assertCmdResponse_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      //! Assert command response in history at index
      //!
      void assertCmdResponse(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq, /*!< The command sequence number*/
          const Fw::CommandResponse response /*!< The command response*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Events
      // ----------------------------------------------------------------------

      void assertEvents_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: PrmIdNotFound
      // ----------------------------------------------------------------------

      void assertEvents_PrmIdNotFound_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_PrmIdNotFound(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const U32 Id /*!< The parameter ID*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: PrmIdUpdated
      // ----------------------------------------------------------------------

      void assertEvents_PrmIdUpdated_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_PrmIdUpdated(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const U32 Id /*!< The parameter ID*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: PrmDbFull
      // ----------------------------------------------------------------------

      void assertEvents_PrmDbFull_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_PrmDbFull(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const U32 Id /*!< The parameter ID*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: PrmIdAdded
      // ----------------------------------------------------------------------

      void assertEvents_PrmIdAdded_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_PrmIdAdded(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const U32 Id /*!< The parameter ID*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: PrmSendTooLarge
      // ----------------------------------------------------------------------

      void assertEvents_PrmSendTooLarge_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_PrmSendTooLarge(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const U32 prmId, /*!< Parameter that could not be sent*/
          const U32 prmSize /*!< Size of serialized parameter*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: PrmSendBufferFull
      // ----------------------------------------------------------------------

      void assertEvents_PrmSendBufferFull_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

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
      // From port: sendPrm 
      // ----------------------------------------------------------------------

      void assert_from_sendPrm_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // From port: recvPrmReady 
      // ----------------------------------------------------------------------

      void assert_from_recvPrmReady_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // From port: pingOut 
      // ----------------------------------------------------------------------

      void assert_from_pingOut_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

  };

} // end namespace Svc

#endif
