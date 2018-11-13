// ====================================================================== 
// \title  HLRosIface.hpp
// \author mereweth
// \brief  cpp file for HLRosIface test harness implementation class
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

#include <stdio.h>
#include <iostream>

//#define DEBUG_MODE

#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
//#define DEBUG_PRINT(x,...)

namespace HLProc {

  // ----------------------------------------------------------------------
  // Construction and destruction 
  // ----------------------------------------------------------------------

  Tester ::
    Tester(void) : 
#if FW_OBJECT_NAMES == 1
      HLRosIfaceGTestBase("Tester", MAX_HISTORY_SIZE),
      component("HLRosIface")
#else
      HLRosIfaceGTestBase(MAX_HISTORY_SIZE),
      component()
#endif
  {
    this->initComponents();
    this->connectPorts();

    component.startPub();
    component.startIntTask(30, 20*1024);
  }

  Tester ::
    ~Tester(void) 
  {
    
  }

  // ----------------------------------------------------------------------
  // Tests 
  // ----------------------------------------------------------------------

  void Tester ::
    callSched(void) 
  {
      this->invoke_to_sched(0, 0);
  }

  // ----------------------------------------------------------------------
  // Handlers for typed from ports
  // ----------------------------------------------------------------------

  void Tester ::
    from_Tlm_handler(
        const NATIVE_INT_TYPE portNum,
        FwChanIdType id,
        Fw::Time &timeTag,
        Fw::TlmBuffer &val
    )
  {
    this->pushFromPortEntry_Tlm(id, timeTag, val);
  }

  void Tester ::
    from_Log_handler(
        const NATIVE_INT_TYPE portNum,
        FwEventIdType id,
        Fw::Time &timeTag,
        Fw::LogSeverity severity,
        Fw::LogBuffer &args
    )
  {
    this->pushFromPortEntry_Log(id, timeTag, severity, args);
  }

  void Tester ::
    from_LogText_handler(
        const NATIVE_INT_TYPE portNum,
        FwEventIdType id,
        Fw::Time &timeTag,
        Fw::TextLogSeverity severity,
        Fw::TextLogString &text
    )
  {
    this->pushFromPortEntry_LogText(id, timeTag, severity, text);
  }

  void Tester ::
    from_pingOut_handler(
        const NATIVE_INT_TYPE portNum,
        U32 key
    )
  {
    this->pushFromPortEntry_pingOut(key);
  }

  void Tester ::
    from_Time_handler(
        const NATIVE_INT_TYPE portNum,
        Fw::Time &time
    )
  {
    this->pushFromPortEntry_Time(time);
  }

  void Tester ::
    from_ImuStateUpdate_handler(
        const NATIVE_INT_TYPE portNum,
        ROS::mav_msgs::ImuStateUpdate &ImuStateUpdate
    )
  {
    this->pushFromPortEntry_ImuStateUpdate(ImuStateUpdate);
  }

  void Tester ::
    from_ActuatorsData_handler(
        const NATIVE_INT_TYPE portNum,
        ROS::mav_msgs::Actuators &Actuators
    )
  {
    this->pushFromPortEntry_ActuatorsData(Actuators);
    NATIVE_INT_TYPE angSize;
    NATIVE_INT_TYPE angVelSize;
    NATIVE_INT_TYPE normSize;
    (void) Actuators.getangles(angSize);
    (void) Actuators.getangular_velocities(angVelSize);
    (void) Actuators.getnormalized(normSize);
    
    DEBUG_PRINT("Got angles sz %d cnt %u, angvel sz %d cnt %u, norm sz %d cnt %u\n",
                angSize, Actuators.getangles_count(),
                angVelSize, Actuators.getangular_velocities_count(),
                normSize, Actuators.getnormalized_count());
#ifdef DEBUG_MODE
    std::cout << Actuators << "\n";
#endif
  }

  // ----------------------------------------------------------------------
  // Helper methods 
  // ----------------------------------------------------------------------

  void Tester ::
    connectPorts(void) 
  {

    // Imu
    this->connect_to_Imu(
        0,
        this->component.get_Imu_InputPort(0)
    );

    // sched
    this->connect_to_sched(
        0,
        this->component.get_sched_InputPort(0)
    );

    // Odometry
    this->connect_to_Odometry(
        0,
        this->component.get_Odometry_InputPort(0)
    );

    // pingIn
    this->connect_to_pingIn(
        0,
        this->component.get_pingIn_InputPort(0)
    );

    // Tlm
    this->component.set_Tlm_OutputPort(
        0, 
        this->get_from_Tlm(0)
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

    // pingOut
    this->component.set_pingOut_OutputPort(
        0, 
        this->get_from_pingOut(0)
    );

    // Time
    this->component.set_Time_OutputPort(
        0, 
        this->get_from_Time(0)
    );

    // ImuStateUpdate
    this->component.set_ImuStateUpdate_OutputPort(
        0, 
        this->get_from_ImuStateUpdate(0)
    );

    // ActuatorsData
    for (NATIVE_INT_TYPE i = 0; i < 2; ++i) {
      this->component.set_ActuatorsData_OutputPort(
          i, 
          this->get_from_ActuatorsData(i)
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

} // end namespace HLProc
