// ====================================================================== 
// \title  BlspI2CDriver.hpp
// \author mereweth
// \brief  cpp file for BlspI2CDriver test harness implementation class
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

namespace SnapdragonFlight {

  // ----------------------------------------------------------------------
  // Construction and destruction 
  // ----------------------------------------------------------------------

  Tester ::
    Tester(void) : 
#if FW_OBJECT_NAMES == 1
      BlspI2CDriverGTestBase("Tester", MAX_HISTORY_SIZE),
      component("BlspI2CDriver")
#else
      BlspI2CDriverGTestBase(MAX_HISTORY_SIZE),
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

  void Tester::testWriteRead(BYTE* buffer, NATIVE_INT_TYPE size,
                             NATIVE_INT_TYPE readSize) {
      Fw::Buffer w;

      w.setdata((U64)buffer);
      w.setsize(size);

      printf("WRITE: ");
      for (NATIVE_INT_TYPE byte = 0; byte < size; byte++) {
          printf("0x%02X ",buffer[byte]);
      }
      printf("\n");

      BYTE* rb = 0;
      rb = (BYTE*) malloc(readSize);


      Fw::Buffer r(0,0,(U64)rb,readSize);

      this->invoke_to_I2CReadWrite(0,w,r);

      BYTE* d = (BYTE*)r.getdata();
      printf("READ: ");
      for (NATIVE_INT_TYPE byte = 0; byte < readSize; byte++) {
          printf("0x%02X ",d[byte]);
      }
      printf("\n");

      free(rb);
  }

  // ----------------------------------------------------------------------
  // Helper methods 
  // ----------------------------------------------------------------------

  void Tester ::
    connectPorts(void) 
  {

    // I2CReadWrite
    this->connect_to_I2CReadWrite(
        0,
        this->component.get_I2CReadWrite_InputPort(0)
    );

    // I2CConfig
    this->connect_to_I2CConfig(
        0,
        this->component.get_I2CConfig_InputPort(0)
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

    // Time
    this->component.set_Time_OutputPort(
        0, 
        this->get_from_Time(0)
    );




  }

  void Tester::textLogIn(const FwEventIdType id, //!< The event ID
          Fw::Time& timeTag, //!< The time
          const Fw::TextLogSeverity severity, //!< The severity
          const Fw::TextLogString& text //!< The event string
          ) {
      TextLogEntry e = { id, timeTag, severity, text };

      printTextLogHistoryEntry(e, stdout);
  }
  
  void Tester ::
    initComponents(void) 
  {
    this->init();
    this->component.init(
        INSTANCE
    );
  }

  void Tester ::
    openAndConfig(int i2c, U32 slave, U32 busSpeed) {

      this->component.open(i2c);

      this->invoke_to_I2CConfig(0, busSpeed, slave, 500);
  }  

} // end namespace SnapdragonFlight
