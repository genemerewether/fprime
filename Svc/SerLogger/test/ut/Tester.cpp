// ====================================================================== 
// \title  SerLogger.hpp
// \author tcanham
// \brief  cpp file for SerLogger test harness implementation class
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
#include <Svc/ActiveFileLogger/ActiveFileLoggerPacket.hpp>

#define INSTANCE 0
#define MAX_HISTORY_SIZE 10

namespace Svc {

  // ----------------------------------------------------------------------
  // Construction and destruction 
  // ----------------------------------------------------------------------

  Tester ::
    Tester(void) : 
#if FW_OBJECT_NAMES == 1
      SerLoggerGTestBase("Tester", MAX_HISTORY_SIZE),
      component("SerLogger")
#else
      SerLoggerGTestBase(MAX_HISTORY_SIZE),
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
      nominalCopy(void)
  {
      // set the stream ID
      this->component.setStreamId(static_cast<active_file_logger_stream_t>(100));
      // Create a serialized buffer (borrow the active logger buffer type)
      Svc::ActiveFileLoggerPacket testBuff;
      // File Buffer
      Svc::ActiveFileLoggerPacket packet;
      // serialize stream ID
      ASSERT_EQ(Fw::FW_SERIALIZE_OK,packet.serialize((U8)100));
      NATIVE_UINT_TYPE len = testBuff.getBuffCapacity() - 1;
      for (NATIVE_UINT_TYPE byte = 0; byte < len; byte++) {
          ASSERT_EQ(Fw::FW_SERIALIZE_OK,testBuff.serialize((U8)byte));
          ASSERT_EQ(Fw::FW_SERIALIZE_OK,packet.serialize((U8)byte));
      }
      // send packet
      this->invoke_to_SerPortIn(0,testBuff);
      // should be a converted port call
      ASSERT_FROM_PORT_HISTORY_SIZE(1);
      ASSERT_from_LogOut_SIZE(1);
      ASSERT_from_LogOut(0,packet);
  }

  // ----------------------------------------------------------------------
  // Handlers for typed from ports
  // ----------------------------------------------------------------------

  void Tester ::
    from_LogOut_handler(
        const NATIVE_INT_TYPE portNum,
        Svc::ActiveFileLoggerPacket &data
    )
  {
    this->pushFromPortEntry_LogOut(data);
  }

  // ----------------------------------------------------------------------
  // Helper methods 
  // ----------------------------------------------------------------------

  void Tester ::
    connectPorts(void) 
  {

    // LogOut
    this->component.set_LogOut_OutputPort(
        0, 
        this->get_from_LogOut(0)
    );



  // ----------------------------------------------------------------------
  // Connect serial input ports
  // ----------------------------------------------------------------------
    // SerPortIn
    this->connect_to_SerPortIn(
        0,
        this->component.get_SerPortIn_InputPort(0)
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

} // end namespace Svc
