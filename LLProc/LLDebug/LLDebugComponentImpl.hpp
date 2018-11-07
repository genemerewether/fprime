// ====================================================================== 
// \title  LLDebugImpl.hpp
// \author tcanham
// \brief  hpp file for LLDebug component implementation class
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

#ifndef LLDebug_HPP
#define LLDebug_HPP

#include "LLProc/LLDebug/LLDebugComponentAc.hpp"
#include "LLProc/LLDebug/LLDebugComponentImplCfg.hpp"
#include <Fw/Types/Assert.hpp>

namespace LLProc {

  class LLDebugComponentImpl :
    public LLDebugComponentBase, private Fw::AssertHook
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object LLDebug
      //!
      LLDebugComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object LLDebug
      //!
      void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object LLDebug
      //!
      ~LLDebugComponentImpl(void);

    PRIVATE:
      // Assert handling
      void reportAssert(
              FILE_NAME_ARG file,
              NATIVE_UINT_TYPE lineNo,
              NATIVE_UINT_TYPE numArgs,
              AssertArg arg1,
              AssertArg arg2,
              AssertArg arg3,
              AssertArg arg4,
              AssertArg arg5,
              AssertArg arg6
              );
      // copy assert text to serial port
      void printAssert(const I8* msg);
      // set software fault line, then assert
      void doAssert(void);

      //! Handler implementation for DebugStringIn
      //!
      void DebugStringIn_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U8 *string, /*!< string*/
          U32 size /*!< size of string*/
      );

      //! Handler implementation for AssertEnable
      //!
      void AssertEnable_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Common::EnableState enabled
      );

      // debug string storage
      U8 m_debugText[LL_DEBUG_TEXT_SIZE];
      // container for serial data buffer
      Fw::Buffer m_textBuffer;
      // flag to enable/disable asserts
      bool m_disableAssert;

    };

} // end namespace LLProc

#endif
