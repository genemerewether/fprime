// ====================================================================== 
// \title  GPSPosAdapter.hpp
// \author mereweth
// \brief  cpp file for GPSPosAdapter test harness implementation class
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

namespace Drv {

  // ----------------------------------------------------------------------
  // Construction and destruction 
  // ----------------------------------------------------------------------

  Tester ::
    Tester(void) : 
#if FW_OBJECT_NAMES == 1
      GPSPosAdapterGTestBase("Tester", MAX_HISTORY_SIZE),
      component("GPSPosAdapter")
#else
      GPSPosAdapterGTestBase(MAX_HISTORY_SIZE),
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
    toDo(void) 
  {
    // TODO
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

  // ----------------------------------------------------------------------
  // Helper methods 
  // ----------------------------------------------------------------------

  void Tester ::
    connectPorts(void) 
  {

    // Guid
    this->connect_to_Guid(
        0,
        this->component.get_Guid_InputPort(0)
    );

    // Nav
    this->connect_to_Nav(
        0,
        this->component.get_Nav_InputPort(0)
    );

    // sched
    this->connect_to_sched(
        0,
        this->component.get_sched_InputPort(0)
    );

    // SerReadPort
    this->connect_to_SerReadPort(
        0,
        this->component.get_SerReadPort_InputPort(0)
    );

    // SerWritePort
    this->component.set_SerWritePort_OutputPort(
        0, 
        this->get_from_SerWritePort(0)
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

} // end namespace Drv
