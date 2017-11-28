// ======================================================================
// \title  RosImg.hpp
// \author vagrant
// \brief  cpp file for RosImg test harness implementation class
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
#define QUEUE_DEPTH 10

namespace Navoutdoor {

  // ----------------------------------------------------------------------
  // Construction and destruction
  // ----------------------------------------------------------------------

  Tester ::
    Tester(void) :
#if FW_OBJECT_NAMES == 1
      RosImgGTestBase("Tester", MAX_HISTORY_SIZE),
      component("RosImg")
#else
      RosImgGTestBase(MAX_HISTORY_SIZE),
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
    send_once(void) 
  {
    // TODO
  }

  // ----------------------------------------------------------------------
  // Helper methods
  // ----------------------------------------------------------------------

  void Tester ::
    connectPorts(void)
  {

    // ImageRecv
    this->connect_to_ImageRecv(
        0,
        this->component.get_ImageRecv_InputPort(0)
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

} // end namespace Navoutdoor
