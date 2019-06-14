// ====================================================================== 
// \title  TimeSyncOffset.hpp
// \author kedelson
// \brief  cpp file for TimeSyncOffset test harness implementation class
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged.
// 
// ====================================================================== 

#include "Tester.hpp"

#define INSTANCE 0
#define MAX_HISTORY_SIZE 10
#define QUEUE_DEPTH 10

namespace Svc {

  // ----------------------------------------------------------------------
  // Construction and destruction 
  // ----------------------------------------------------------------------

  Tester ::
    Tester(void) : 
#if FW_OBJECT_NAMES == 1
      TimeSyncOffsetGTestBase("Tester", MAX_HISTORY_SIZE),
      component("TimeSyncOffset")
#else
      TimeSyncOffsetGTestBase(MAX_HISTORY_SIZE),
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
    timeSyncTest(void) 
  {
    // TODO

    // ASSERT_TLM_SIZE
    // ASSERT_TLM_LLOffset_SIZE
    // ASSERT_TLM_LLOffset

    // ASSERT_FROM_PORT_HISTORY_SIZE
    // ASSERT_from_GPIOPulse_SIZE
    // ASSERT_from_GPIOPulse
    // ASSERT_from_Offset_SIZE
    // ASSERT_from_Offset

    
  }

  // ----------------------------------------------------------------------
  // Handlers for typed from ports
  // ----------------------------------------------------------------------

  void Tester ::
    from_GPIOPulse_handler(
        const NATIVE_INT_TYPE portNum,
        bool state
    )
  {
    this->pushFromPortEntry_GPIOPulse(state);
  }

  void Tester ::
    from_Offset_handler(
        const NATIVE_INT_TYPE portNum,
        Fw::Time &time
    )
  {
    this->pushFromPortEntry_Offset(time);
  }

  // ----------------------------------------------------------------------
  // Helper methods 
  // ----------------------------------------------------------------------

  void Tester ::
    connectPorts(void) 
  {

    // SchedIn
    this->connect_to_SchedIn(
        0,
        this->component.get_SchedIn_InputPort(0)
    );

    // LLTime
    this->connect_to_LLTime(
        0,
        this->component.get_LLTime_InputPort(0)
    );

    // GPIOPulse
    this->component.set_GPIOPulse_OutputPort(
        0, 
        this->get_from_GPIOPulse(0)
    );

    // Offset
    this->component.set_Offset_OutputPort(
        0, 
        this->get_from_Offset(0)
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




  }

  void Tester ::
    initComponents(void) 
  {
    this->init();
    this->component.init(
        QUEUE_DEPTH, INSTANCE
    );
  }

} // end namespace Svc
