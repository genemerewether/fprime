// ======================================================================
// \title  SerialTextConverter/test/ut/GTestBase.hpp
// \author Auto-generated
// \brief  hpp file for SerialTextConverter component Google Test harness base class
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

#ifndef SerialTextConverter_GTEST_BASE_HPP
#define SerialTextConverter_GTEST_BASE_HPP

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

#define ASSERT_EVENTS_STC_SerialText_SIZE(size) \
  this->assertEvents_STC_SerialText_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_STC_SerialText(index, _data) \
  this->assertEvents_STC_SerialText(__FILE__, __LINE__, index, _data)

#define ASSERT_EVENTS_STC_SerialReadBadStatus_SIZE(size) \
  this->assertEvents_STC_SerialReadBadStatus_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_STC_SerialReadBadStatus(index, _status) \
  this->assertEvents_STC_SerialReadBadStatus(__FILE__, __LINE__, index, _status)

#define ASSERT_EVENTS_STC_SetMode_Cmd_Sent_SIZE(size) \
  this->assertEvents_STC_SetMode_Cmd_Sent_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_STC_SetMode_Cmd_Sent(index, _mode) \
  this->assertEvents_STC_SetMode_Cmd_Sent(__FILE__, __LINE__, index, _mode)

#define ASSERT_EVENTS_STC_SetMode_Cmd_Invalid_SIZE(size) \
  this->assertEvents_STC_SetMode_Cmd_Invalid_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_STC_File_SIZE(size) \
  this->assertEvents_STC_File_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_STC_File(index, _log_file) \
  this->assertEvents_STC_File(__FILE__, __LINE__, index, _log_file)

// ----------------------------------------------------------------------
// Macros for typed user from port history assertions
// ----------------------------------------------------------------------

#define ASSERT_FROM_PORT_HISTORY_SIZE(size) \
  this->assertFromPortHistory_size(__FILE__, __LINE__, size)

#define ASSERT_from_SerialBufferSend_SIZE(size) \
  this->assert_from_SerialBufferSend_size(__FILE__, __LINE__, size)

#define ASSERT_from_SerialBufferSend(index, _fwBuffer) \
  { \
    ASSERT_GT(this->fromPortHistory_SerialBufferSend->size(), static_cast<U32>(index)) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Index into history of from_SerialBufferSend\n" \
    << "  Expected: Less than size of history (" \
    << this->fromPortHistory_SerialBufferSend->size() << ")\n" \
    << "  Actual:   " << index << "\n"; \
    const FromPortEntry_SerialBufferSend& _e = \
      this->fromPortHistory_SerialBufferSend->at(index); \
    ASSERT_EQ(_fwBuffer, _e.fwBuffer) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument fwBuffer at index " \
    << index \
    << " in history of from_SerialBufferSend\n" \
    << "  Expected: " << _fwBuffer << "\n" \
    << "  Actual:   " << _e.fwBuffer << "\n"; \
  }

namespace Svc {

  //! \class SerialTextConverterGTestBase
  //! \brief Auto-generated base class for SerialTextConverter component Google Test harness
  //!
  class SerialTextConverterGTestBase :
    public SerialTextConverterTesterBase
  {

    protected:

      // ----------------------------------------------------------------------
      // Construction and destruction
      // ----------------------------------------------------------------------

      //! Construct object SerialTextConverterGTestBase
      //!
      SerialTextConverterGTestBase(
#if FW_OBJECT_NAMES == 1
          const char *const compName, /*!< The component name*/
          const U32 maxHistorySize /*!< The maximum size of each history*/
#else
          const U32 maxHistorySize /*!< The maximum size of each history*/
#endif
      );

      //! Destroy object SerialTextConverterGTestBase
      //!
      virtual ~SerialTextConverterGTestBase(void);

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
          const U32 index, /*!< The index*/
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
      // Event: STC_SerialText
      // ----------------------------------------------------------------------

      void assertEvents_STC_SerialText_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_STC_SerialText(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 index, /*!< The index*/
          const char *const data /*!< Serial data turned into a string*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: STC_SerialReadBadStatus
      // ----------------------------------------------------------------------

      void assertEvents_STC_SerialReadBadStatus_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_STC_SerialReadBadStatus(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 index, /*!< The index*/
          const I32 status /*!< Serial read status*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: STC_SetMode_Cmd_Sent
      // ----------------------------------------------------------------------

      void assertEvents_STC_SetMode_Cmd_Sent_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_STC_SetMode_Cmd_Sent(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 index, /*!< The index*/
          SerialTextConverterComponentBase::StcModeEv mode 
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: STC_SetMode_Cmd_Invalid
      // ----------------------------------------------------------------------

      void assertEvents_STC_SetMode_Cmd_Invalid_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: STC_File
      // ----------------------------------------------------------------------

      void assertEvents_STC_File_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_STC_File(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 index, /*!< The index*/
          const char *const log_file /*!< Log file being used*/
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
      // From port: SerialBufferSend 
      // ----------------------------------------------------------------------

      void assert_from_SerialBufferSend_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

  };

} // end namespace Svc

#endif
