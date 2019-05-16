// ======================================================================
// \title  BlspI2CDriver/test/ut/GTestBase.hpp
// \author Auto-generated
// \brief  hpp file for BlspI2CDriver component Google Test harness base class
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

#ifndef BlspI2CDriver_GTEST_BASE_HPP
#define BlspI2CDriver_GTEST_BASE_HPP

#include "TesterBase.hpp"
#include "gtest/gtest.h"

// ----------------------------------------------------------------------
// Macros for telemetry history assertions
// ----------------------------------------------------------------------

#define ASSERT_TLM_SIZE(size) \
  this->assertTlm_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_I2C_ReadBytes_SIZE(size) \
  this->assertTlm_I2C_ReadBytes_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_I2C_ReadBytes(index, value) \
  this->assertTlm_I2C_ReadBytes(__FILE__, __LINE__, index, value)

#define ASSERT_TLM_I2C_WriteBytes_SIZE(size) \
  this->assertTlm_I2C_WriteBytes_size(__FILE__, __LINE__, size)

#define ASSERT_TLM_I2C_WriteBytes(index, value) \
  this->assertTlm_I2C_WriteBytes(__FILE__, __LINE__, index, value)

// ----------------------------------------------------------------------
// Macros for event history assertions 
// ----------------------------------------------------------------------

#define ASSERT_EVENTS_SIZE(size) \
  this->assertEvents_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_I2C_OpenError_SIZE(size) \
  this->assertEvents_I2C_OpenError_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_I2C_OpenError(index, _device, _error) \
  this->assertEvents_I2C_OpenError(__FILE__, __LINE__, index, _device, _error)

#define ASSERT_EVENTS_I2C_ConfigError_SIZE(size) \
  this->assertEvents_I2C_ConfigError_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_I2C_ConfigError(index, _device, _error) \
  this->assertEvents_I2C_ConfigError(__FILE__, __LINE__, index, _device, _error)

#define ASSERT_EVENTS_I2C_ReadWriteError_SIZE(size) \
  this->assertEvents_I2C_ReadWriteError_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_I2C_ReadWriteError(index, _device, _addr, _error, _bytesAct, _bytesDes) \
  this->assertEvents_I2C_ReadWriteError(__FILE__, __LINE__, index, _device, _addr, _error, _bytesAct, _bytesDes)

#define ASSERT_EVENTS_I2C_WriteError_SIZE(size) \
  this->assertEvents_I2C_WriteError_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_I2C_WriteError(index, _device, _addr, _error, _bytesAct, _bytesDes) \
  this->assertEvents_I2C_WriteError(__FILE__, __LINE__, index, _device, _addr, _error, _bytesAct, _bytesDes)

#define ASSERT_EVENTS_I2C_PortOpened_SIZE(size) \
  this->assertEvents_I2C_PortOpened_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_I2C_PortOpened(index, _device) \
  this->assertEvents_I2C_PortOpened(__FILE__, __LINE__, index, _device)

#define ASSERT_EVENTS_I2C_ReadError_SIZE(size) \
  this->assertEvents_I2C_ReadError_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_I2C_ReadError(index, _device, _addr, _error, _bytesAct, _bytesDes) \
  this->assertEvents_I2C_ReadError(__FILE__, __LINE__, index, _device, _addr, _error, _bytesAct, _bytesDes)

#define ASSERT_EVENTS_I2C_BadReadWriteCall_SIZE(size) \
  this->assertEvents_I2C_BadReadWriteCall_size(__FILE__, __LINE__, size)

#define ASSERT_EVENTS_I2C_BadReadWriteCall(index, _device) \
  this->assertEvents_I2C_BadReadWriteCall(__FILE__, __LINE__, index, _device)

namespace SnapdragonFlight {

  //! \class BlspI2CDriverGTestBase
  //! \brief Auto-generated base class for BlspI2CDriver component Google Test harness
  //!
  class BlspI2CDriverGTestBase :
    public BlspI2CDriverTesterBase
  {

    protected:

      // ----------------------------------------------------------------------
      // Construction and destruction
      // ----------------------------------------------------------------------

      //! Construct object BlspI2CDriverGTestBase
      //!
      BlspI2CDriverGTestBase(
#if FW_OBJECT_NAMES == 1
          const char *const compName, /*!< The component name*/
          const U32 maxHistorySize /*!< The maximum size of each history*/
#else
          const U32 maxHistorySize /*!< The maximum size of each history*/
#endif
      );

      //! Destroy object BlspI2CDriverGTestBase
      //!
      virtual ~BlspI2CDriverGTestBase(void);

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
      // Channel: I2C_ReadBytes
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_I2C_ReadBytes_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_I2C_ReadBytes(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const U32& val /*!< The channel value*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Channel: I2C_WriteBytes
      // ----------------------------------------------------------------------

      //! Assert telemetry value in history at index
      //!
      void assertTlm_I2C_WriteBytes_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertTlm_I2C_WriteBytes(
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
      // Event: I2C_OpenError
      // ----------------------------------------------------------------------

      void assertEvents_I2C_OpenError_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_I2C_OpenError(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const I32 device, /*!< The device*/
          const I32 error /*!< The error code*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: I2C_ConfigError
      // ----------------------------------------------------------------------

      void assertEvents_I2C_ConfigError_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_I2C_ConfigError(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const I32 device, /*!< The device*/
          const I32 error /*!< The error code*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: I2C_ReadWriteError
      // ----------------------------------------------------------------------

      void assertEvents_I2C_ReadWriteError_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_I2C_ReadWriteError(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const I32 device, /*!< The device*/
          const U32 addr, 
          const I32 error, /*!< The error code*/
          const I32 bytesAct, 
          const I32 bytesDes 
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: I2C_WriteError
      // ----------------------------------------------------------------------

      void assertEvents_I2C_WriteError_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_I2C_WriteError(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const I32 device, /*!< The device*/
          const U32 addr, 
          const I32 error, /*!< The error code*/
          const I32 bytesAct, 
          const I32 bytesDes 
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: I2C_PortOpened
      // ----------------------------------------------------------------------

      void assertEvents_I2C_PortOpened_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_I2C_PortOpened(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const I32 device /*!< The device*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: I2C_ReadError
      // ----------------------------------------------------------------------

      void assertEvents_I2C_ReadError_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_I2C_ReadError(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const I32 device, /*!< The device*/
          const U32 addr, 
          const I32 error, /*!< The error code*/
          const I32 bytesAct, 
          const I32 bytesDes 
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // Event: I2C_BadReadWriteCall
      // ----------------------------------------------------------------------

      void assertEvents_I2C_BadReadWriteCall_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

      void assertEvents_I2C_BadReadWriteCall(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 __index, /*!< The index*/
          const I32 device /*!< The device*/
      ) const;

  };

} // end namespace SnapdragonFlight

#endif
