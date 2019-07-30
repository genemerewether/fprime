// ====================================================================== 
// \title  BlspPwmDriver.hpp
// \author mereweth
// \brief  cpp file for BlspPwmDriver test harness implementation class
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

#include "Os/Task.hpp"

namespace SnapdragonFlight {

  // ----------------------------------------------------------------------
  // Construction and destruction 
  // ----------------------------------------------------------------------

  Tester ::
    Tester(void) : 
#if FW_OBJECT_NAMES == 1
      BlspPwmDriverGTestBase("Tester", MAX_HISTORY_SIZE),
      component("BlspPwmDriver")
#else
      BlspPwmDriverGTestBase(MAX_HISTORY_SIZE),
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
    testPwm(NATIVE_UINT_TYPE pwm,
            NATIVE_UINT_TYPE gpio, NATIVE_INT_TYPE cycles, F32 duty) {
      this->component.open(pwm, &gpio, 1, &duty, 10 * 1000);

      for (NATIVE_INT_TYPE cycle = 0; cycle < cycles; cycle++) {
          printf("Cycle: %d\n",cycle);
          //this->invoke_to_gpioWrite(0,state);
          Os::Task::delay(500);
      }
  }
  

  // ----------------------------------------------------------------------
  // Helper methods 
  // ----------------------------------------------------------------------

  void Tester ::
    connectPorts(void) 
  {

    // pwmConfig
    this->connect_to_pwmConfig(
        0,
        this->component.get_pwmConfig_InputPort(0)
    );

    // pwmSetDuty
    for (NATIVE_INT_TYPE i = 0; i < 2; ++i) {
      this->connect_to_pwmSetDuty(
          i,
          this->component.get_pwmSetDuty_InputPort(i)
      );
    }

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
        INSTANCE
    );
  }

} // end namespace SnapdragonFlight
