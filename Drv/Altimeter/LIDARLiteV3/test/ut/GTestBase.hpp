// ======================================================================
// \title  LIDARLiteV3/test/ut/GTestBase.hpp
// \author Auto-generated
// \brief  hpp file for LIDARLiteV3 component Google Test harness base class
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

#ifndef LIDARLiteV3_GTEST_BASE_HPP
#define LIDARLiteV3_GTEST_BASE_HPP

#include "TesterBase.hpp"
#include "gtest/gtest.h"

// ----------------------------------------------------------------------
// Macros for typed user from port history assertions
// ----------------------------------------------------------------------

#define ASSERT_FROM_PORT_HISTORY_SIZE(size) \
  this->assertFromPortHistory_size(__FILE__, __LINE__, size)

#define ASSERT_from_AltimeterSend_SIZE(size) \
  this->assert_from_AltimeterSend_size(__FILE__, __LINE__, size)

#define ASSERT_from_AltimeterSend(index, _altimeterEuData) \
  { \
    ASSERT_GT(this->fromPortHistory_AltimeterSend->size(), static_cast<U32>(index)) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Index into history of from_AltimeterSend\n" \
    << "  Expected: Less than size of history (" \
    << this->fromPortHistory_AltimeterSend->size() << ")\n" \
    << "  Actual:   " << index << "\n"; \
    const FromPortEntry_AltimeterSend& _e = \
      this->fromPortHistory_AltimeterSend->at(index); \
    ASSERT_EQ(_altimeterEuData, _e.altimeterEuData) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument altimeterEuData at index " \
    << index \
    << " in history of from_AltimeterSend\n" \
    << "  Expected: " << _altimeterEuData << "\n" \
    << "  Actual:   " << _e.altimeterEuData << "\n"; \
  }

#define ASSERT_from_I2CConfig_SIZE(size) \
  this->assert_from_I2CConfig_size(__FILE__, __LINE__, size)

#define ASSERT_from_I2CConfig(index, _busSpeed, _slaveAddr, _timeout) \
  { \
    ASSERT_GT(this->fromPortHistory_I2CConfig->size(), static_cast<U32>(index)) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Index into history of from_I2CConfig\n" \
    << "  Expected: Less than size of history (" \
    << this->fromPortHistory_I2CConfig->size() << ")\n" \
    << "  Actual:   " << index << "\n"; \
    const FromPortEntry_I2CConfig& _e = \
      this->fromPortHistory_I2CConfig->at(index); \
    ASSERT_EQ(_busSpeed, _e.busSpeed) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument busSpeed at index " \
    << index \
    << " in history of from_I2CConfig\n" \
    << "  Expected: " << _busSpeed << "\n" \
    << "  Actual:   " << _e.busSpeed << "\n"; \
    ASSERT_EQ(_slaveAddr, _e.slaveAddr) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument slaveAddr at index " \
    << index \
    << " in history of from_I2CConfig\n" \
    << "  Expected: " << _slaveAddr << "\n" \
    << "  Actual:   " << _e.slaveAddr << "\n"; \
    ASSERT_EQ(_timeout, _e.timeout) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument timeout at index " \
    << index \
    << " in history of from_I2CConfig\n" \
    << "  Expected: " << _timeout << "\n" \
    << "  Actual:   " << _e.timeout << "\n"; \
  }

#define ASSERT_from_I2CWriteRead_SIZE(size) \
  this->assert_from_I2CWriteRead_size(__FILE__, __LINE__, size)

#define ASSERT_from_I2CWriteRead(index, _writeBuffer, _readBuffer) \
  { \
    ASSERT_GT(this->fromPortHistory_I2CWriteRead->size(), static_cast<U32>(index)) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Index into history of from_I2CWriteRead\n" \
    << "  Expected: Less than size of history (" \
    << this->fromPortHistory_I2CWriteRead->size() << ")\n" \
    << "  Actual:   " << index << "\n"; \
    const FromPortEntry_I2CWriteRead& _e = \
      this->fromPortHistory_I2CWriteRead->at(index); \
    ASSERT_EQ(_writeBuffer, _e.writeBuffer) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument writeBuffer at index " \
    << index \
    << " in history of from_I2CWriteRead\n" \
    << "  Expected: " << _writeBuffer << "\n" \
    << "  Actual:   " << _e.writeBuffer << "\n"; \
    ASSERT_EQ(_readBuffer, _e.readBuffer) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument readBuffer at index " \
    << index \
    << " in history of from_I2CWriteRead\n" \
    << "  Expected: " << _readBuffer << "\n" \
    << "  Actual:   " << _e.readBuffer << "\n"; \
  }

#define ASSERT_from_I2CWriteReadStatus_SIZE(size) \
  this->assert_from_I2CWriteReadStatus_size(__FILE__, __LINE__, size)

#define ASSERT_from_I2CWriteReadStatus(index, _shouldBlock) \
  { \
    ASSERT_GT(this->fromPortHistory_I2CWriteReadStatus->size(), static_cast<U32>(index)) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Index into history of from_I2CWriteReadStatus\n" \
    << "  Expected: Less than size of history (" \
    << this->fromPortHistory_I2CWriteReadStatus->size() << ")\n" \
    << "  Actual:   " << index << "\n"; \
    const FromPortEntry_I2CWriteReadStatus& _e = \
      this->fromPortHistory_I2CWriteReadStatus->at(index); \
    ASSERT_EQ(_shouldBlock, _e.shouldBlock) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument shouldBlock at index " \
    << index \
    << " in history of from_I2CWriteReadStatus\n" \
    << "  Expected: " << _shouldBlock << "\n" \
    << "  Actual:   " << _e.shouldBlock << "\n"; \
  }

namespace Drv {

  //! \class LIDARLiteV3GTestBase
  //! \brief Auto-generated base class for LIDARLiteV3 component Google Test harness
  //!
  class LIDARLiteV3GTestBase :
    public LIDARLiteV3TesterBase
  {

    protected:

      // ----------------------------------------------------------------------
      // Construction and destruction
      // ----------------------------------------------------------------------

      //! Construct object LIDARLiteV3GTestBase
      //!
      LIDARLiteV3GTestBase(
#if FW_OBJECT_NAMES == 1
          const char *const compName, /*!< The component name*/
          const U32 maxHistorySize /*!< The maximum size of each history*/
#else
          const U32 maxHistorySize /*!< The maximum size of each history*/
#endif
      );

      //! Destroy object LIDARLiteV3GTestBase
      //!
      virtual ~LIDARLiteV3GTestBase(void);

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
      // From port: AltimeterSend 
      // ----------------------------------------------------------------------

      void assert_from_AltimeterSend_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // From port: I2CConfig 
      // ----------------------------------------------------------------------

      void assert_from_I2CConfig_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // From port: I2CWriteRead 
      // ----------------------------------------------------------------------

      void assert_from_I2CWriteRead_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // From port: I2CWriteReadStatus 
      // ----------------------------------------------------------------------

      void assert_from_I2CWriteReadStatus_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

  };

} // end namespace Drv

#endif
