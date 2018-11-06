// ====================================================================== 
// \title  EventExpander.hpp
// \author tcanham
// \brief  cpp file for EventExpander test harness implementation class
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

namespace HLProc {

  // ----------------------------------------------------------------------
  // Construction and destruction 
  // ----------------------------------------------------------------------

  Tester ::
    Tester(void) : 
#if FW_OBJECT_NAMES == 1
      EventExpanderGTestBase("Tester", MAX_HISTORY_SIZE),
      component("EventExpander")
#else
      EventExpanderGTestBase(MAX_HISTORY_SIZE),
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
      nominal(void)
  {

      // Build a short event
      Fw::LogBuffer args;
      U8* argsPtr = args.getBuffAddr();
      for (NATIVE_UINT_TYPE byte = 0; byte < 10; byte++) {
          argsPtr[byte] = byte;
      }
      ASSERT_TRUE(args.setBuffLen(10) == Fw::FW_SERIALIZE_OK);

      this->m_testTime.set(TB_WORKSTATION_TIME,10,11);

      this->invoke_to_LogRecv(0,1000,static_cast<U8>(Fw::LOG_ACTIVITY_HI),args);
      ASSERT_FROM_PORT_HISTORY_SIZE(1);
      ASSERT_from_LogSend_SIZE(1);
      ASSERT_from_LogSend(0,(NATIVE_UINT_TYPE)1000,this->m_testTime,Fw::LOG_ACTIVITY_HI,args);

  }

  // ----------------------------------------------------------------------
  // Handlers for typed from ports
  // ----------------------------------------------------------------------

  void Tester ::
    from_LogSend_handler(
        const NATIVE_INT_TYPE portNum,
        FwEventIdType id,
        Fw::Time &timeTag,
        Fw::LogSeverity severity,
        Fw::LogBuffer &args
    )
  {
    this->pushFromPortEntry_LogSend(id, timeTag, severity, args);
  }

  void Tester ::
    from_Time_handler(
        const NATIVE_INT_TYPE portNum,
        Fw::Time &time
    )
  {
    time = this->m_testTime;
  }

  // ----------------------------------------------------------------------
  // Helper methods 
  // ----------------------------------------------------------------------

  void Tester ::
    connectPorts(void) 
  {

    // LogRecv
    this->connect_to_LogRecv(
        0,
        this->component.get_LogRecv_InputPort(0)
    );

    // LogSend
    this->component.set_LogSend_OutputPort(
        0, 
        this->get_from_LogSend(0)
    );

    // Time
    this->component.set_Time_OutputPort(
        0, 
        this->get_from_Time(0)
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

} // end namespace HLProc
