// ====================================================================== 
// \title  Tee.hpp
// \author parallels
// \brief  cpp file for Tee test harness implementation class
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
#include "Fw/Types/EightyCharString.hpp"

#define INSTANCE 0
#define MAX_HISTORY_SIZE 10

namespace Svc {

  // ----------------------------------------------------------------------
  // Construction and destruction 
  // ----------------------------------------------------------------------

  Tester ::
    Tester(void) : 
#if FW_OBJECT_NAMES == 1
      TeeGTestBase("Tester", MAX_HISTORY_SIZE),
      component("Tee"),
#else
      TeeGTestBase(MAX_HISTORY_SIZE),
      component(),
#endif
      m_buff(),
      m_expected_buffer(0),
      m_saw_buff()
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
    twoOutputTest(void) 
  {
    this->m_buff[0].serialize("Hello World!");
    this->m_buff[1].serialize("Test string 2");

    this->m_expected_buffer = 0;

    this->invoke_to_DataIn(0, this->m_buff[0]);

    ASSERT_EQ(2, this->m_saw_buff[0]);

    this->m_expected_buffer = 1;

    this->invoke_to_DataIn(0, this->m_buff[1]);

    ASSERT_EQ(2, this->m_saw_buff[1]);

  }

  // ----------------------------------------------------------------------
  // Handlers for serial from ports
  // ----------------------------------------------------------------------

  void Tester ::
    from_DataOut_handler(
        NATIVE_INT_TYPE portNum, /*!< The port number*/
        Fw::SerializeBufferBase &Buffer /*!< The serialization buffer*/
    )
  {
    ASSERT_TRUE(this->m_buff[this->m_expected_buffer] == Buffer);

    this->m_saw_buff[this->m_expected_buffer]++;
  }

  // ----------------------------------------------------------------------
  // Helper methods 
  // ----------------------------------------------------------------------

  void Tester ::
    connectPorts(void) 
  {


  // ----------------------------------------------------------------------
  // Connect serial output ports
  // ----------------------------------------------------------------------
    // Only connect 2 outputs
    for (NATIVE_INT_TYPE i = 0; i < 2; ++i) {
      this->component.set_DataOut_OutputPort(
          i, 
          this->get_from_DataOut(i)
      );
    }


  // ----------------------------------------------------------------------
  // Connect serial input ports
  // ----------------------------------------------------------------------
    // DataIn
    this->connect_to_DataIn(
        0,
        this->component.get_DataIn_InputPort(0)
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
