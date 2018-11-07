// ====================================================================== 
// \title  LLDebug.hpp
// \author tcanham
// \brief  cpp file for LLDebug test harness implementation class
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

#include "Tester.hpp"

#define INSTANCE 0
#define MAX_HISTORY_SIZE 10
#include <string.h>
#include <LLProc/LLDebug/LLDebugComponentImplCfg.hpp>
#include <Fw/Test/UnitTest.hpp>

namespace LLProc {

  // ----------------------------------------------------------------------
  // Construction and destruction 
  // ----------------------------------------------------------------------

  Tester ::
    Tester(void) : 
#if FW_OBJECT_NAMES == 1
      LLDebugGTestBase("Tester", MAX_HISTORY_SIZE),
      component("LLDebug")
#else
      LLDebugGTestBase(MAX_HISTORY_SIZE),
      component()
#endif
  {
    this->initComponents();
    this->connectPorts();
  }

  Tester ::
    ~Tester(void) 
  {
    
  }

  // ----------------------------------------------------------------------
  // Tests 
  // ----------------------------------------------------------------------

  void Tester ::
    debugTest(void)
  {
      REQUIREMENT("LL-DBG-001");
      const char * str = "Hello, World!";
      this->invoke_to_DebugStringIn(0,(U8*)str,strlen(str));
      ASSERT_from_SerWritePort_SIZE(1);
      Fw::Buffer buffer(0,0,(U64)this->component.m_debugText,strlen((char*)str));
      ASSERT_from_SerWritePort(0,buffer);
      ASSERT_STREQ((char*)str,(char*)this->component.m_debugText);
  }

  void Tester ::
    assertTest(void)
  {

      REQUIREMENT("LL-DBG-002");
      FW_ASSERT(0,1,2,3,4,5,6);
      ASSERT_from_SerWritePort_SIZE(1);
      REQUIREMENT("LL-DBG-004");
      ASSERT_from_SwFault_SIZE(1);
      ASSERT_from_SwFault(0,SW_ASSERT_GPIO_BANK,SW_ASSERT_GPIO_BIT,R5::GPIO_SET_SET);
      // build assert string
      char assertStr[255];
      NATIVE_UINT_TYPE bytes = sprintf(assertStr,"File \"%s\":%d %d %d %d %d %d %d",__FILE__,__LINE__-6,1,2,3,4,5,6);
      // truncate
      assertStr[LL_DEBUG_TEXT_SIZE-1]=0;
      //printf("AS: %s\n",assertStr);
      Fw::Buffer buffer(0,0,(U64)this->component.m_debugText,
              bytes>LL_DEBUG_TEXT_SIZE?LL_DEBUG_TEXT_SIZE:strlen((char*)assertStr));
      REQUIREMENT("LL-DBG-003");
      ASSERT_from_SerWritePort(0,buffer);
      ASSERT_EQ(0,strcmp((char*)assertStr,(char*)this->component.m_debugText));
  }

  // ----------------------------------------------------------------------
  // Handlers for typed from ports
  // ----------------------------------------------------------------------

  void Tester ::
    from_SerWritePort_handler(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &serBuffer
    )
  {
    this->pushFromPortEntry_SerWritePort(serBuffer);
  }

  void Tester ::
    from_SwFault_handler(
        const NATIVE_INT_TYPE portNum,
        R5::GpioSetBank bank,
        U32 bit,
        R5::GpioSetVal val
    )
  {
    this->pushFromPortEntry_SwFault(bank, bit, val);
  }

  // ----------------------------------------------------------------------
  // Helper methods 
  // ----------------------------------------------------------------------

  void Tester ::
    connectPorts(void) 
  {

    // DebugStringIn
    this->connect_to_DebugStringIn(
        0,
        this->component.get_DebugStringIn_InputPort(0)
    );

    // SerWritePort
    this->component.set_SerWritePort_OutputPort(
        0, 
        this->get_from_SerWritePort(0)
    );

    // SwFault
    this->component.set_SwFault_OutputPort(
        0, 
        this->get_from_SwFault(0)
    );




  }

  void Tester ::
    initComponents(void) 
  {
    this->init();
    this->component.init(
        INSTANCE
    );
  }

} // end namespace LLProc
