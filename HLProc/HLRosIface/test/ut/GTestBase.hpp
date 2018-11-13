// ======================================================================
// \title  HLRosIface/test/ut/GTestBase.hpp
// \author Auto-generated
// \brief  hpp file for HLRosIface component Google Test harness base class
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

#ifndef HLRosIface_GTEST_BASE_HPP
#define HLRosIface_GTEST_BASE_HPP

#include "TesterBase.hpp"
#include "gtest/gtest.h"

// ----------------------------------------------------------------------
// Macros for typed user from port history assertions
// ----------------------------------------------------------------------

#define ASSERT_FROM_PORT_HISTORY_SIZE(size) \
  this->assertFromPortHistory_size(__FILE__, __LINE__, size)

#define ASSERT_from_Tlm_SIZE(size) \
  this->assert_from_Tlm_size(__FILE__, __LINE__, size)

#define ASSERT_from_Tlm(index, _id, _timeTag, _val) \
  { \
    ASSERT_GT(this->fromPortHistory_Tlm->size(), static_cast<U32>(index)) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Index into history of from_Tlm\n" \
    << "  Expected: Less than size of history (" \
    << this->fromPortHistory_Tlm->size() << ")\n" \
    << "  Actual:   " << index << "\n"; \
    const FromPortEntry_Tlm& _e = \
      this->fromPortHistory_Tlm->at(index); \
    ASSERT_EQ(_id, _e.id) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument id at index " \
    << index \
    << " in history of from_Tlm\n" \
    << "  Expected: " << _id << "\n" \
    << "  Actual:   " << _e.id << "\n"; \
    ASSERT_EQ(_timeTag, _e.timeTag) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument timeTag at index " \
    << index \
    << " in history of from_Tlm\n" \
    << "  Expected: " << _timeTag << "\n" \
    << "  Actual:   " << _e.timeTag << "\n"; \
    ASSERT_EQ(_val, _e.val) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument val at index " \
    << index \
    << " in history of from_Tlm\n" \
    << "  Expected: " << _val << "\n" \
    << "  Actual:   " << _e.val << "\n"; \
  }

#define ASSERT_from_Log_SIZE(size) \
  this->assert_from_Log_size(__FILE__, __LINE__, size)

#define ASSERT_from_Log(index, _id, _timeTag, _severity, _args) \
  { \
    ASSERT_GT(this->fromPortHistory_Log->size(), static_cast<U32>(index)) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Index into history of from_Log\n" \
    << "  Expected: Less than size of history (" \
    << this->fromPortHistory_Log->size() << ")\n" \
    << "  Actual:   " << index << "\n"; \
    const FromPortEntry_Log& _e = \
      this->fromPortHistory_Log->at(index); \
    ASSERT_EQ(_id, _e.id) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument id at index " \
    << index \
    << " in history of from_Log\n" \
    << "  Expected: " << _id << "\n" \
    << "  Actual:   " << _e.id << "\n"; \
    ASSERT_EQ(_timeTag, _e.timeTag) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument timeTag at index " \
    << index \
    << " in history of from_Log\n" \
    << "  Expected: " << _timeTag << "\n" \
    << "  Actual:   " << _e.timeTag << "\n"; \
    ASSERT_EQ(_severity, _e.severity) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument severity at index " \
    << index \
    << " in history of from_Log\n" \
    << "  Expected: " << _severity << "\n" \
    << "  Actual:   " << _e.severity << "\n"; \
    ASSERT_EQ(_args, _e.args) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument args at index " \
    << index \
    << " in history of from_Log\n" \
    << "  Expected: " << _args << "\n" \
    << "  Actual:   " << _e.args << "\n"; \
  }

#define ASSERT_from_LogText_SIZE(size) \
  this->assert_from_LogText_size(__FILE__, __LINE__, size)

#define ASSERT_from_LogText(index, _id, _timeTag, _severity, _text) \
  { \
    ASSERT_GT(this->fromPortHistory_LogText->size(), static_cast<U32>(index)) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Index into history of from_LogText\n" \
    << "  Expected: Less than size of history (" \
    << this->fromPortHistory_LogText->size() << ")\n" \
    << "  Actual:   " << index << "\n"; \
    const FromPortEntry_LogText& _e = \
      this->fromPortHistory_LogText->at(index); \
    ASSERT_EQ(_id, _e.id) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument id at index " \
    << index \
    << " in history of from_LogText\n" \
    << "  Expected: " << _id << "\n" \
    << "  Actual:   " << _e.id << "\n"; \
    ASSERT_EQ(_timeTag, _e.timeTag) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument timeTag at index " \
    << index \
    << " in history of from_LogText\n" \
    << "  Expected: " << _timeTag << "\n" \
    << "  Actual:   " << _e.timeTag << "\n"; \
    ASSERT_EQ(_severity, _e.severity) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument severity at index " \
    << index \
    << " in history of from_LogText\n" \
    << "  Expected: " << _severity << "\n" \
    << "  Actual:   " << _e.severity << "\n"; \
    ASSERT_EQ(_text, _e.text) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument text at index " \
    << index \
    << " in history of from_LogText\n" \
    << "  Expected: " << _text << "\n" \
    << "  Actual:   " << _e.text << "\n"; \
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

#define ASSERT_from_Time_SIZE(size) \
  this->assert_from_Time_size(__FILE__, __LINE__, size)

#define ASSERT_from_Time(index, _time) \
  { \
    ASSERT_GT(this->fromPortHistory_Time->size(), static_cast<U32>(index)) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Index into history of from_Time\n" \
    << "  Expected: Less than size of history (" \
    << this->fromPortHistory_Time->size() << ")\n" \
    << "  Actual:   " << index << "\n"; \
    const FromPortEntry_Time& _e = \
      this->fromPortHistory_Time->at(index); \
    ASSERT_EQ(_time, _e.time) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument time at index " \
    << index \
    << " in history of from_Time\n" \
    << "  Expected: " << _time << "\n" \
    << "  Actual:   " << _e.time << "\n"; \
  }

#define ASSERT_from_ImuStateUpdate_SIZE(size) \
  this->assert_from_ImuStateUpdate_size(__FILE__, __LINE__, size)

#define ASSERT_from_ImuStateUpdate(index, _ImuStateUpdate) \
  { \
    ASSERT_GT(this->fromPortHistory_ImuStateUpdate->size(), static_cast<U32>(index)) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Index into history of from_ImuStateUpdate\n" \
    << "  Expected: Less than size of history (" \
    << this->fromPortHistory_ImuStateUpdate->size() << ")\n" \
    << "  Actual:   " << index << "\n"; \
    const FromPortEntry_ImuStateUpdate& _e = \
      this->fromPortHistory_ImuStateUpdate->at(index); \
    ASSERT_EQ(_ImuStateUpdate, _e.ImuStateUpdate) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument ImuStateUpdate at index " \
    << index \
    << " in history of from_ImuStateUpdate\n" \
    << "  Expected: " << _ImuStateUpdate << "\n" \
    << "  Actual:   " << _e.ImuStateUpdate << "\n"; \
  }

#define ASSERT_from_ActuatorsData_SIZE(size) \
  this->assert_from_ActuatorsData_size(__FILE__, __LINE__, size)

#define ASSERT_from_ActuatorsData(index, _Actuators) \
  { \
    ASSERT_GT(this->fromPortHistory_ActuatorsData->size(), static_cast<U32>(index)) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Index into history of from_ActuatorsData\n" \
    << "  Expected: Less than size of history (" \
    << this->fromPortHistory_ActuatorsData->size() << ")\n" \
    << "  Actual:   " << index << "\n"; \
    const FromPortEntry_ActuatorsData& _e = \
      this->fromPortHistory_ActuatorsData->at(index); \
    ASSERT_EQ(_Actuators, _e.Actuators) \
    << "\n" \
    << "  File:     " << __FILE__ << "\n" \
    << "  Line:     " << __LINE__ << "\n" \
    << "  Value:    Value of argument Actuators at index " \
    << index \
    << " in history of from_ActuatorsData\n" \
    << "  Expected: " << _Actuators << "\n" \
    << "  Actual:   " << _e.Actuators << "\n"; \
  }

namespace HLProc {

  //! \class HLRosIfaceGTestBase
  //! \brief Auto-generated base class for HLRosIface component Google Test harness
  //!
  class HLRosIfaceGTestBase :
    public HLRosIfaceTesterBase
  {

    protected:

      // ----------------------------------------------------------------------
      // Construction and destruction
      // ----------------------------------------------------------------------

      //! Construct object HLRosIfaceGTestBase
      //!
      HLRosIfaceGTestBase(
#if FW_OBJECT_NAMES == 1
          const char *const compName, /*!< The component name*/
          const U32 maxHistorySize /*!< The maximum size of each history*/
#else
          const U32 maxHistorySize /*!< The maximum size of each history*/
#endif
      );

      //! Destroy object HLRosIfaceGTestBase
      //!
      virtual ~HLRosIfaceGTestBase(void);

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
      // From port: Tlm 
      // ----------------------------------------------------------------------

      void assert_from_Tlm_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // From port: Log 
      // ----------------------------------------------------------------------

      void assert_from_Log_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // From port: LogText 
      // ----------------------------------------------------------------------

      void assert_from_LogText_size(
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

    protected:

      // ----------------------------------------------------------------------
      // From port: Time 
      // ----------------------------------------------------------------------

      void assert_from_Time_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // From port: ImuStateUpdate 
      // ----------------------------------------------------------------------

      void assert_from_ImuStateUpdate_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

    protected:

      // ----------------------------------------------------------------------
      // From port: ActuatorsData 
      // ----------------------------------------------------------------------

      void assert_from_ActuatorsData_size(
          const char *const __callSiteFileName, /*!< The name of the file containing the call site*/
          const U32 __callSiteLineNumber, /*!< The line number of the call site*/
          const U32 size /*!< The asserted size*/
      ) const;

  };

} // end namespace HLProc

#endif
