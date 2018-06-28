// ======================================================================
// \title  KraitRouter.hpp
// \author mereweth
// \brief  cpp file for KraitRouter test harness implementation class
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

#ifdef BUILD_DSPAL
#include <HAP_farf.h>
#define DEBUG_PRINT(x,...) FARF(ALWAYS,x,##__VA_ARGS__);
#else
#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
#endif

//#undef DEBUG_PRINT
//#define DEBUG_PRINT(x,...)

namespace SnapdragonFlight {

  // ----------------------------------------------------------------------
  // Construction and destruction
  // ----------------------------------------------------------------------

  Tester ::
    Tester(void) :
#if FW_OBJECT_NAMES == 1
      KraitRouterGTestBase("Tester", MAX_HISTORY_SIZE),
      component("KraitRouter")
#else
      KraitRouterGTestBase(MAX_HISTORY_SIZE),
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
    run_port_read_write_test(void)
  {
      this->component.write(0, (const unsigned char *)"hi", 3);
      this->invoke_to_Sched(0, 0);
      unsigned int port = 0;
      unsigned char buff[512];
      int bytes = -1;
      this->component.portRead(&port,
                               (unsigned char *)buff,
                               FW_NUM_ARRAY_ELEMENTS(buff),
                               &bytes);
      DEBUG_PRINT("Round trip for port %d with %d bytes.\n",
                  port, bytes);
  }

  // ----------------------------------------------------------------------
  // Handlers for serial from ports
  // ----------------------------------------------------------------------

  void Tester ::
    from_KraitPortsOut_handler(
        NATIVE_INT_TYPE portNum, /*!< The port number*/
        Fw::SerializeBufferBase &Buffer /*!< The serialization buffer*/
    )
  {
      DEBUG_PRINT("KRAITRTR called port %d with %d bytes.\n",
                  portNum, Buffer.getBuffLength());
      this->invoke_to_HexPortsIn(portNum, Buffer);
  }

  // ----------------------------------------------------------------------
  // Helper methods
  // ----------------------------------------------------------------------

  void Tester ::
    connectPorts(void)
  {

    // Sched
    this->connect_to_Sched(
        0,
        this->component.get_Sched_InputPort(0)
    );

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
      this->component.set_KraitPortsOut_OutputPort(
          i,
          this->get_from_KraitPortsOut(i)
      );
    }


  // ----------------------------------------------------------------------
  // Connect serial input ports
  // ----------------------------------------------------------------------
    // HexPortsIn
    for (NATIVE_INT_TYPE i = 0; i < 25; ++i) {
      this->connect_to_HexPortsIn(
          i,
          this->component.get_HexPortsIn_InputPort(i)
      );
    }


  }

  void Tester ::
    initComponents(void)
  {
    this->init();
    this->component.init(
        INSTANCE
    );
  }

} // end namespace SnapdragonFlight
