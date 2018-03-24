// ======================================================================
// \title  HexRouter.hpp
// \author mereweth
// \brief  cpp file for HexRouter test harness implementation class
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
#include "Fw/Types/SerialBuffer.hpp"

#ifdef BUILD_SDFLIGHT
    #include <ut_hexrtr.h>
#endif

#define INSTANCE 0
#define MAX_HISTORY_SIZE 10
#define QUEUE_DEPTH 10
#define MSG_SIZE 2*READ_PORT_SIZE // TODO(mereweth) - increase this for off-nominal testing?

#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
//#define DEBUG_PRINT(x,...)

namespace SnapdragonFlight {

char gBuff[KR_PORT_BUFF_SIZE];
unsigned int gLen;
unsigned int gPortNum;

  // ----------------------------------------------------------------------
  // Construction and destruction
  // ----------------------------------------------------------------------

  Tester ::
    Tester(void) :
#if FW_OBJECT_NAMES == 1
      HexRouterGTestBase("Tester", MAX_HISTORY_SIZE),
      component("HexRouter")
#else
      HexRouterGTestBase(MAX_HISTORY_SIZE),
      component()
#endif
  {
    this->initComponents();
    this->connectPorts();

    this->component.startBuffReadThread(90,20*1024,-1);
    this->component.startPortReadThread(90,20*1024,-1);
  }

  Tester ::
    ~Tester(void)
  {
    this->component.quitReadThreads();
  }

  // ----------------------------------------------------------------------
  // Tests
  // ----------------------------------------------------------------------

  void Tester ::
    run_port_read_write_test(void)
  {
#ifdef BUILD_SDFLIGHT
    ut_hexrtr_init();
#endif // BUILD_SDFLIGHT

    Fw::ExternalSerializeBuffer bufObj;
    char buf[200] = {"hi"};
    DEBUG_PRINT("Contents of buf: %s\n", buf);
    bufObj.setExtBuffer((U8*) buf, 200);

    bufObj.setBuffLen(3);
    this->invoke_to_KraitPortsIn(0, bufObj);
    this->dispatchAll();

#ifndef BUILD_SDFLIGHT
    DEBUG_PRINT("Port %d, Len %d, Contents of gBuff: %s\n",
                gPortNum, gLen, gBuff);
#endif

    this->invoke_to_KraitPortsIn(0, bufObj);
    this->dispatchAll();

#ifdef BUILD_SDFLIGHT
    Os::Task::delay(1000);
    ut_hexrtr_sched();
#endif // BUILD_SDFLIGHT

#ifdef BUILD_SDFLIGHT
    Os::Task::delay(1000);
    ut_hexrtr_fini();
#endif // BUILD_SDFLIGHT
  }

  // ----------------------------------------------------------------------
  // Handlers for typed from ports
  // ----------------------------------------------------------------------

  void Tester ::
    from_readBufferSend_handler(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer fwBuffer
    )
  {
    this->pushFromPortEntry_readBufferSend(fwBuffer);
  }

  // ----------------------------------------------------------------------
  // Handlers for serial from ports
  // ----------------------------------------------------------------------

  void Tester ::
    from_HexPortsOut_handler(
        NATIVE_INT_TYPE portNum, /*!< The port number*/
        Fw::SerializeBufferBase &Buffer /*!< The serialization buffer*/
    )
  {
    // TODO
  }

  // ----------------------------------------------------------------------
  // Helper methods
  // ----------------------------------------------------------------------

  void Tester ::
    connectPorts(void)
  {

    // readBufferRecv
    for (NATIVE_INT_TYPE i = 0; i < 5; ++i) {
      this->connect_to_readBufferRecv(
          i,
          this->component.get_readBufferRecv_InputPort(i)
      );
    }

    // Sched
    this->connect_to_Sched(
        0,
        this->component.get_Sched_InputPort(0)
    );

    // readBufferSend
    for (NATIVE_INT_TYPE i = 0; i < 5; ++i) {
      this->component.set_readBufferSend_OutputPort(
          i,
          this->get_from_readBufferSend(i)
      );
    }

    // Tlm
    this->component.set_Tlm_OutputPort(
        0,
        this->get_from_Tlm(0)
    );

    // Time
    this->component.set_Time_OutputPort(
        0,
        this->get_from_Time(0)
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


  // ----------------------------------------------------------------------
  // Connect serial output ports
  // ----------------------------------------------------------------------
    for (NATIVE_INT_TYPE i = 0; i < 25; ++i) {
      this->component.set_HexPortsOut_OutputPort(
          i,
          this->get_from_HexPortsOut(i)
      );
    }


  // ----------------------------------------------------------------------
  // Connect serial input ports
  // ----------------------------------------------------------------------
    // KraitPortsIn
    for (NATIVE_INT_TYPE i = 0; i < 25; ++i) {
      this->connect_to_KraitPortsIn(
          i,
          this->component.get_KraitPortsIn_InputPort(i)
      );
    }


  }

  void Tester ::
    dispatchOne(void)
  {
      this->component.doDispatch();
  }

  void Tester ::
    dispatchAll(void)
  {
      while(this->component.m_queue.getNumMsgs() > 0)
      this->dispatchOne();
  }

  void Tester ::
    initComponents(void)
  {
    this->init();
    this->component.init(
        QUEUE_DEPTH, MSG_SIZE, INSTANCE
    );
  }

} // end namespace SnapdragonFlight
