// ====================================================================== 
// \title  FatalHandlerImpl.cpp
// \author tcanham
// \brief  cpp file for FatalHandler component implementation class
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged.
// 
// ====================================================================== 


#include <Svc/FatalHandler/FatalHandlerComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

namespace Svc {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction 
  // ----------------------------------------------------------------------

  FatalHandlerComponentImpl ::
#if FW_OBJECT_NAMES == 1
    FatalHandlerComponentImpl(
        const char *const compName
    ) :
      FatalHandlerComponentBase(compName)
#else
    FatalHandlerImpl(void)
#endif
    ,m_disableAssert(false)
  {

  }

  void FatalHandlerComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    ) 
  {
    FatalHandlerComponentBase::init(instance);
  }

  FatalHandlerComponentImpl ::
    ~FatalHandlerComponentImpl(void)
  {

  }

  void FatalHandlerComponentImpl ::
    FH_ENABLE_ASSERT_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq,
        AssertEnable enable
    )
  {
      switch (enable) {
          case ASSERT_ENABLE:
              this->m_disableAssert = false;
              this->log_ACTIVITY_HI_FH_AssertEnabled();
              break;
          case ASSERT_DISABLE:
              this->m_disableAssert = true;
              this->log_ACTIVITY_HI_FH_AssertDisabled();
              break;
          default:
              this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_VALIDATION_ERROR);
              return;
      }
      this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_OK);

  }

} // end namespace Svc
