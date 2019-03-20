// ====================================================================== 
// \title  ZmqRadio.hpp
// \author dkooi
// \brief  cpp file for ZmqRadio test harness implementation class
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
#define QUEUE_DEPTH 10

namespace Zmq {

  // ----------------------------------------------------------------------
  // Construction and destruction 
  // ----------------------------------------------------------------------

  Tester ::
    Tester(void) : 
#if FW_OBJECT_NAMES == 1
      ZmqRadioGTestBase("Tester", MAX_HISTORY_SIZE),
      component("ZmqRadio")
#else
      ZmqRadioGTestBase(MAX_HISTORY_SIZE),
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
    testConnection(void)

  {

    // Initialize component and open a connection
    this->component.init(100, 1);
    this->component.open("localhost", 5555, "flight_1");
    this->component.start(0, 90, 20*1024);
    
    // Delay: Wait for component to startup and register
    for(int i = 0; i<10000000; i++){
      // Pass
    }

    // Call reconnect handler to activate telemetry
    this->component.reconnect_handler(0,0);

    //printf("Before Disp\n");
    this->dispatchAll();
    ASSERT_TLM_ZR_NumConnects(0,1); // Expecting 1 connection
    ASSERT_EVENTS_ZR_Connection_SIZE(1); // Expecting 1 connection event

    // Transition to disconnected state
    this->component.m_state.transitionDisconnected();
    // Call reconnect handler to activate telemetry
    this->component.reconnect_handler(0,0);
    this->dispatchAll();
    ASSERT_EVENTS_ZR_Disconnection_SIZE(1); // Expecting one disconnection event
    ASSERT_TLM_ZR_NumDisconnects(1,1); // Expecting 1 disconnect
    
    // reconnect handler should have made a reconenction attempt
    ASSERT_EVENTS_ZR_Connection_SIZE(2); // Expect 2 connection events 
    ASSERT_TLM_ZR_NumConnects(1,2); // Expect 2 connections

  }
  // ----------------------------------------------------------------------
  // Handlers for typed from ports
  // ----------------------------------------------------------------------

  void Tester ::
    from_fileUplinkBufferSendOut_handler(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer fwBuffer
    )
  {
    this->pushFromPortEntry_fileUplinkBufferSendOut(fwBuffer);
  }

  void Tester ::
    from_uplinkPort_handler(
        const NATIVE_INT_TYPE portNum,
        Fw::ComBuffer &data,
        U32 context
    )
  {
    this->pushFromPortEntry_uplinkPort(data, context);
  }

  void Tester ::
    from_fileDownlinkBufferSendOut_handler(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer fwBuffer
    )
  {
    this->pushFromPortEntry_fileDownlinkBufferSendOut(fwBuffer);
  }

  Fw::Buffer Tester ::
    from_fileUplinkBufferGet_handler(
        const NATIVE_INT_TYPE portNum,
        U32 size
    )
  {
    this->pushFromPortEntry_fileUplinkBufferGet(size);
    // TODO: Return a value
  }

  // ----------------------------------------------------------------------
  // Helper methods 
  // ----------------------------------------------------------------------
  //! Dispatch internal messages
  //!
  void Tester::dispatchAll(void)
  {
      while(this->component.m_queue.getNumMsgs() > 0) {
          this->component.doDispatch();
      }
  }

  void Tester ::
    connectPorts(void) 
  {

    // reconnect
    this->connect_to_reconnect(
        0,
        this->component.get_reconnect_InputPort(0)
    );

    // downlinkPort
    this->connect_to_downlinkPort(
        0,
        this->component.get_downlinkPort_InputPort(0)
    );

    // fileDownlinkBufferSendIn
    this->connect_to_fileDownlinkBufferSendIn(
        0,
        this->component.get_fileDownlinkBufferSendIn_InputPort(0)
    );

    // fileUplinkBufferSendOut
    this->component.set_fileUplinkBufferSendOut_OutputPort(
        0, 
        this->get_from_fileUplinkBufferSendOut(0)
    );

    // Log
    this->component.set_Log_OutputPort(
        0, 
        this->get_from_Log(0)
    );

    // LogText
    this->component.set_LogText_OutputPort(
        0, 
        this->get_from_LogText(0)
    );

    // Time
    this->component.set_Time_OutputPort(
        0, 
        this->get_from_Time(0)
    );

    // uplinkPort
    this->component.set_uplinkPort_OutputPort(
        0, 
        this->get_from_uplinkPort(0)
    );

    // fileDownlinkBufferSendOut
    this->component.set_fileDownlinkBufferSendOut_OutputPort(
        0, 
        this->get_from_fileDownlinkBufferSendOut(0)
    );

    // fileUplinkBufferGet
    this->component.set_fileUplinkBufferGet_OutputPort(
        0, 
        this->get_from_fileUplinkBufferGet(0)
    );

    // tlmOut
    this->component.set_tlmOut_OutputPort(
        0, 
        this->get_from_tlmOut(0)
    );

  }

  void Tester ::
    initComponents(void) 
  {
    this->init();
    this->component.init(
        QUEUE_DEPTH, INSTANCE
    );
  }

} // end namespace Zmq
