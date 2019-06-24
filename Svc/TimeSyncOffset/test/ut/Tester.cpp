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
    SendPulseRecieveTimeTest(void) 
  {
    // TODO

    // check initial conditions
    ASSERT_FROM_PORT_HISTORY_SIZE(0);
    ASSERT_TLM_SIZE(0);
    ASSERT_TLM_LLTime_SIZE(0);
    ASSERT_TLM_HLTime_SIZE(0);
    ASSERT_EVENTS_SIZE(0);
    ASSERT_EVENTS_SchedIn_Timeout_SIZE(0);

    // ASSERT_TLM_LLTime(index, value);
    // ASSERT_TLM_HLTime(index, value);
    // ASSERT_EVENTS_SchedIn_Timeout(index, _sched_timeout)

    // ASSERT_from_GPIOPulse_SIZE(size);
    // ASSERT_from_GPIOPulse(index, _state);




    this->invoke_to_SchedIn(0,0);
    // this->component.doDispatch();
    // // this->dispatchAll();

    
    // // Fw::Time LLTime = this->getTime();
    // // this->setTestTime(const Fw::Time& timeTag);
    // Fw::Time LLTime(69,666666);
    // this->invoke_to_LLTime(0, LLTime);

    // // check initial conditions
    // ASSERT_FROM_PORT_HISTORY_SIZE(0);
    // ASSERT_TLM_SIZE(0);
    // ASSERT_TLM_LLTime_SIZE(0);
    // ASSERT_TLM_HLTime_SIZE(0);
    // ASSERT_EVENTS_SIZE(0);
    // ASSERT_EVENTS_SchedIn_Timeout_SIZE(0);

    // this->component.doDispatch();
    // // this->dispatchAll();

    // // check initial conditions
    // ASSERT_FROM_PORT_HISTORY_SIZE(0);
    // ASSERT_TLM_SIZE(0);
    // ASSERT_TLM_LLTime_SIZE(0);
    // ASSERT_TLM_HLTime_SIZE(0);
    // ASSERT_EVENTS_SIZE(0);
    // ASSERT_EVENTS_SchedIn_Timeout_SIZE(0);
    

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
    from_ClockTimes_handler(
        const NATIVE_INT_TYPE portNum,
        Fw::Time time1,
        Fw::Time time2
    )
  {
    this->pushFromPortEntry_ClockTimes(time1, time2);
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

    // ClockTimes
    this->component.set_ClockTimes_OutputPort(
        0, 
        this->get_from_ClockTimes(0)
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
