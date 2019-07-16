// ====================================================================== 
// \title  LLDebugImpl.cpp
// \author tcanham
// \brief  cpp file for LLDebug component implementation class
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

#include <LLProc/LLDebug/LLDebugComponentImpl.hpp>
#include <Fw/Types/BasicTypes.hpp>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>

namespace LLProc {

    // ----------------------------------------------------------------------
    // Construction, initialization, and destruction
    // ----------------------------------------------------------------------

    LLDebugComponentImpl::
#if FW_OBJECT_NAMES == 1
    LLDebugComponentImpl(const char * const compName) :
            LLDebugComponentBase(compName)
#else
    LLDebugImpl(void)
#endif
    ,m_disableAssert(false)
    {
        this->m_textBuffer.setdata((U64)this->m_debugText);
        this->m_textBuffer.setmanagerID(0);
        this->m_textBuffer.setbufferID(0);
    }

    void LLDebugComponentImpl::init(const NATIVE_INT_TYPE instance) {
        LLDebugComponentBase::init(instance);
        // register assertions
        this->registerHook();
    }

    LLDebugComponentImpl::~LLDebugComponentImpl(void) {
    }

    void LLDebugComponentImpl ::
      DebugStringIn_handler(
          const NATIVE_INT_TYPE portNum,
          U8 *string,
          U32 size
      )
    {
        NATIVE_INT_TYPE sizeToSend = size < LL_DEBUG_TEXT_SIZE?size:LL_DEBUG_TEXT_SIZE;
        strncpy((char*)this->m_debugText,(const char*)string,LL_DEBUG_TEXT_SIZE);
        // null terminate
        this->m_debugText[LL_DEBUG_TEXT_SIZE-1] = 0;
        this->m_textBuffer.setsize(sizeToSend);
        this->SerWritePort_out(0,this->m_textBuffer);
    }

#if FW_ASSERT_LEVEL == FW_FILEID_ASSERT
#define fileIdFs "FID 0x%08X:%d "
#else
#define fileIdFs "File \"%s\":%d "
#endif

    // Assert handling
    void LLDebugComponentImpl::reportAssert(
            FILE_NAME_ARG file,
            NATIVE_UINT_TYPE lineNo,
            NATIVE_UINT_TYPE numArgs,
            AssertArg arg1,
            AssertArg arg2,
            AssertArg arg3,
            AssertArg arg4,
            AssertArg arg5,
            AssertArg arg6
            ) {
        // create assert text
        NATIVE_UINT_TYPE sizeToSend = snprintf((char*)this->m_debugText,LL_DEBUG_TEXT_SIZE,fileIdFs "%d %d %d %d %d %d",file,lineNo,
                                arg1,arg2,arg3,arg4,arg5,arg6);

        // truncate if too big
        if (sizeToSend > LL_DEBUG_TEXT_SIZE) {
            sizeToSend = LL_DEBUG_TEXT_SIZE;
        }
        // null terminate
        this->m_debugText[sizeToSend-1] = 0;
        this->m_textBuffer.setsize(sizeToSend);
        printf("%s", this->m_debugText);
        this->SerWritePort_out(0,this->m_textBuffer);
    }
    // copy assert text to serial port
    void LLDebugComponentImpl::printAssert(const I8* msg) {
        // Nothing. Taken care of in reportAssert()
    }
    void LLDebugComponentImpl::doAssert(void) {
        // write to fault GPIO
        if (this->isConnected_SwFault_OutputPort(0)) {
            this->SwFault_out(0, true);
        }
        // assert locally if GPIO output doesn't reset us
        if (not this->m_disableAssert) {
#ifndef BUILD_UT // disable for unit testing
            assert(0);
#endif
        }
    }

    void LLDebugComponentImpl ::
      AssertEnable_handler(
          const NATIVE_INT_TYPE portNum,
          Common::EnableState enabled
      )
    {
      this->m_disableAssert = (Common::DISABLED == enabled)?true:false;
    }

} // end namespace LLProc
