// ======================================================================
// \title  ImgComp/test/ut/GTestBase.hpp
// \author Auto-generated
// \brief  hpp file for ImgComp component Google Test harness base class
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

#ifndef ImgComp_GTEST_BASE_HPP
#define ImgComp_GTEST_BASE_HPP

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
// Macros for telemetry history assertions
// ----------------------------------------------------------------------

#define ASSERT_TLM_SIZE(size) \
  this->assertTlm_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_IMGCOMP_BuffersHandled_SIZE(size) \
  this->assertTlm_IMGCOMP_BuffersHandled_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_IMGCOMP_BuffersHandled(index, value) \
  this->assertTlm_IMGCOMP_BuffersHandled(__FILE__, __LINE__, index, value)

// ----------------------------------------------------------------------
// Macros for event history assertions 
// ----------------------------------------------------------------------

#define ASSERT_EVENTS_SIZE(size) \
  this->assertEvents_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_IMGCOMP_SoftCompError_SIZE(size) \
  this->assertEvents_IMGCOMP_SoftCompError_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_IMGCOMP_SoftCompError(index, _error, _msg) \
  this->assertEvents_IMGCOMP_SoftCompError(__FILE__, __LINE__, index, _error, _msg)

#define ASSERT_EVENTS_IMGCOMP_BadBuffer_SIZE(size) \
  this->assertEvents_IMGCOMP_BadBuffer_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_IMGCOMP_BadSetting_SIZE(size) \
  this->assertEvents_IMGCOMP_BadSetting_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_IMGCOMP_BadSetting(index, _portNum) \
  this->assertEvents_IMGCOMP_BadSetting(__FILE__, __LINE__, index, _portNum)

#define ASSERT_EVENTS_IMGCOMP_NoBuffer_SIZE(size) \
  this->assertEvents_IMGCOMP_NoBuffer_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_IMGCOMP_NoBuffer(index, _size) \
  this->assertEvents_IMGCOMP_NoBuffer(__FILE__, __LINE__, index, _size)

#define ASSERT_EVENTS_IMGCOMP_BufferOffset_SIZE(size) \
  this->assertEvents_IMGCOMP_BufferOffset_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_IMGCOMP_BufferOffset(index, _type, _output, _inBuffer, _portNum) \
  this->assertEvents_IMGCOMP_BufferOffset(__FILE__, __LINE__, index, _type, _output, _inBuffer, _portNum)

#define ASSERT_EVENTS_IMGCOMP_NoRestartMarkers_SIZE(size) \
  this->assertEvents_IMGCOMP_NoRestartMarkers_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_IMGCOMP_NoRestartMarkers(index, _error) \
  this->assertEvents_IMGCOMP_NoRestartMarkers(__FILE__, __LINE__, index, _error)

// ----------------------------------------------------------------------
// Macros for typed user from port history assertions
// ----------------------------------------------------------------------

#define ASSERT_FROM_PORT_HISTORY_SIZE(size) \
  this->assertFromPortHistory_size(__FILE__, __LINE__, size)

#define ASSERT_from_compressedOutStorage_SIZE(size) \
  this->assert_from_compressedOutStorage_size(__FILE__, __LINE__, size)

#define ASSERT_from_compressedOutStorage(index, _fwBuffer) \
  { \
    ASSERT_GT(this->fromPortHistory_compressedOutStorage->size(), static_cast<U32>(index)) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Index into history of from_compressedOutStorage\n" \
    << "  Expected: Less than size of history (" \
    << this->fromPortHistory_compressedOutStorage->size() << ")\n" \
    << "  Actual:   " << index << "\n"; \
    const FromPortEntry_compressedOutStorage& _e = \
      this->fromPortHistory_compressedOutStorage->at(index); \
    ASSERT_EQ(_fwBuffer, _e.fwBuffer) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument fwBuffer at index " \
    << index \
    << " in history of from_compressedOutStorage\n" \
    << "  Expected: " << _fwBuffer << "\n" \
    << "  Actual:   " << _e.fwBuffer << "\n"; \
  }

#define ASSERT_from_compressedOutXmit_SIZE(size) \
  this->assert_from_compressedOutXmit_size(__FILE__, __LINE__, size)

#define ASSERT_from_compressedOutXmit(index, _fwBuffer) \
  { \
    ASSERT_GT(this->fromPortHistory_compressedOutXmit->size(), static_cast<U32>(index)) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Index into history of from_compressedOutXmit\n" \
    << "  Expected: Less than size of history (" \
    << this->fromPortHistory_compressedOutXmit->size() << ")\n" \
    << "  Actual:   " << index << "\n"; \
    const FromPortEntry_compressedOutXmit& _e = \
      this->fromPortHistory_compressedOutXmit->at(index); \
    ASSERT_EQ(_fwBuffer, _e.fwBuffer) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument fwBuffer at index " \
    << index \
    << " in history of from_compressedOutXmit\n" \
    << "  Expected: " << _fwBuffer << "\n" \
    << "  Actual:   " << _e.fwBuffer << "\n"; \
  }

#define ASSERT_from_compressedGetStorage_SIZE(size) \
  this->assert_from_compressedGetStorage_size(__FILE__, __LINE__, size)

#define ASSERT_from_compressedGetStorage(index, _size) \
  { \
    ASSERT_GT(this->fromPortHistory_compressedGetStorage->size(), static_cast<U32>(index)) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Index into history of from_compressedGetStorage\n" \
    << "  Expected: Less than size of history (" \
    << this->fromPortHistory_compressedGetStorage->size() << ")\n" \
    << "  Actual:   " << index << "\n"; \
    const FromPortEntry_compressedGetStorage& _e = \
      this->fromPortHistory_compressedGetStorage->at(index); \
    ASSERT_EQ(_size, _e.size) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument size at index " \
    << index \
    << " in history of from_compressedGetStorage\n" \
    << "  Expected: " << _size << "\n" \
    << "  Actual:   " << _e.size << "\n"; \
  }

#define ASSERT_from_compressedGetXmit_SIZE(size) \
  this->assert_from_compressedGetXmit_size(__FILE__, __LINE__, size)

#define ASSERT_from_compressedGetXmit(index, _size) \
  { \
    ASSERT_GT(this->fromPortHistory_compressedGetXmit->size(), static_cast<U32>(index)) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Index into history of from_compressedGetXmit\n" \
    << "  Expected: Less than size of history (" \
    << this->fromPortHistory_compressedGetXmit->size() << ")\n" \
    << "  Actual:   " << index << "\n"; \
    const FromPortEntry_compressedGetXmit& _e = \
      this->fromPortHistory_compressedGetXmit->at(index); \
    ASSERT_EQ(_size, _e.size) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument size at index " \
    << index \
    << " in history of from_compressedGetXmit\n" \
    << "  Expected: " << _size << "\n" \
    << "  Actual:   " << _e.size << "\n"; \
  }

#define ASSERT_from_uncompressedReturn_SIZE(size) \
  this->assert_from_uncompressedReturn_size(__FILE__, __LINE__, size)

#define ASSERT_from_uncompressedReturn(index, _fwBuffer) \
  { \
    ASSERT_GT(this->fromPortHistory_uncompressedReturn->size(), static_cast<U32>(index)) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Index into history of from_uncompressedReturn\n" \
    << "  Expected: Less than size of history (" \
    << this->fromPortHistory_uncompressedReturn->size() << ")\n" \
    << "  Actual:   " << index << "\n"; \
    const FromPortEntry_uncompressedReturn& _e = \
      this->fromPortHistory_uncompressedReturn->at(index); \
    ASSERT_EQ(_fwBuffer, _e.fwBuffer) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument fwBuffer at index " \
    << index \
    << " in history of from_uncompressedReturn\n" \
    << "  Expected: " << _fwBuffer << "\n" \
    << "  Actual:   " << _e.fwBuffer << "\n"; \
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

  //! \class ImgCompGTestBase
  //! \brief Auto-generated base class for ImgComp component Google Test harness
  //!
  class ImgCompGTestBase :
    public ImgCompTesterBase
  {

    protected:

      // ----------------------------------------------------------------------
      // Construction and destruction
      // ----------------------------------------------------------------------

      //! Construct object ImgCompGTestBase
      //!
      ImgCompGTestBase(
#if FW_OBJECT_NAMES == 1
          const char *const compName, /*!< The component name*/
          const U32 maxHistorySize /*!< The maximum size of each history*/
#else
          const U32 maxHistorySize /*!< The maximum size of each history*/
#endif
      );

      //! Destroy object ImgCompGTestBase
      //!
      virtual ~ImgCompGTestBase(void);

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
      // Telemetry
      // ----------------------------------------------------------------------

      //! Assert size of telemetry history
      //!
      void assertTlm_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: IMGCOMP_BuffersHandled
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_IMGCOMP_BuffersHandled_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_IMGCOMP_BuffersHandled(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const U32& val /*!< The channel value*/
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
      // Event: IMGCOMP_SoftCompError
      // ----------------------------------------------------------------------

      void assertEvents_IMGCOMP_SoftCompError_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_IMGCOMP_SoftCompError(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          ImgCompComponentBase::SoftCompErrorType error, /*!< The error type*/
          const char *const msg 
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: IMGCOMP_BadBuffer
      // ----------------------------------------------------------------------

      void assertEvents_IMGCOMP_BadBuffer_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: IMGCOMP_BadSetting
      // ----------------------------------------------------------------------

      void assertEvents_IMGCOMP_BadSetting_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_IMGCOMP_BadSetting(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const U32 portNum 
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: IMGCOMP_NoBuffer
      // ----------------------------------------------------------------------

      void assertEvents_IMGCOMP_NoBuffer_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_IMGCOMP_NoBuffer(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const U32 size 
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: IMGCOMP_BufferOffset
      // ----------------------------------------------------------------------

      void assertEvents_IMGCOMP_BufferOffset_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_IMGCOMP_BufferOffset(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          ImgCompComponentBase::BufferOffsetSkipType type, /*!< The error type*/
          ImgCompComponentBase::BufferOffsetSkipOutput output, /*!< The error type*/
          const U32 inBuffer, 
          const U32 portNum 
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: IMGCOMP_NoRestartMarkers
      // ----------------------------------------------------------------------

      void assertEvents_IMGCOMP_NoRestartMarkers_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_IMGCOMP_NoRestartMarkers(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const I32 error 
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
      // From port: compressedOutStorage 
      // ----------------------------------------------------------------------

      void assert_from_compressedOutStorage_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // From port: compressedOutXmit 
      // ----------------------------------------------------------------------

      void assert_from_compressedOutXmit_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // From port: compressedGetStorage 
      // ----------------------------------------------------------------------

      void assert_from_compressedGetStorage_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // From port: compressedGetXmit 
      // ----------------------------------------------------------------------

      void assert_from_compressedGetXmit_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // From port: uncompressedReturn 
      // ----------------------------------------------------------------------

      void assert_from_uncompressedReturn_size(
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
