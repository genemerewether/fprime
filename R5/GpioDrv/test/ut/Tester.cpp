// ======================================================================
// \title  R5GpioDriver.hpp
// \author tcanham
// \brief  cpp file for R5GpioDriver test harness implementation class
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

#include <R5/TiHal/include/HL_gio.h>

#define INSTANCE 0
#define MAX_HISTORY_SIZE 10

#define GIO_FLAG_2_MASK  0x4

namespace R5 {

  // ----------------------------------------------------------------------
  // Construction and destruction
  // ----------------------------------------------------------------------

  Tester ::
    Tester(void) :
#if FW_OBJECT_NAMES == 1
      R5GpioDriverTesterBase("Tester", MAX_HISTORY_SIZE),
      component("R5GpioDriver")
#else
      R5GpioDriverTesterBase(MAX_HISTORY_SIZE),
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
      // GIOA[0] connected to GIOA[1]
      GpioGetVal ret;

      // Check default GIOA[0] "0" value
      invoke_to_get(0, GPIO_GET_BANK_A, 0, ret);
      if(GPIO_GET_CLEAR != ret) {
          printf("!!!! Failed GIOA[0]=0x%x expected=0x%x\n", ret, GPIO_GET_CLEAR);
      }

      // Check default GIOA[1] "0" value
      invoke_to_get(0, GPIO_GET_BANK_A, 1, ret);
      if(GPIO_GET_CLEAR != ret) {
          printf("!!!! Failed GIOA[1]=0x%x expected=0x%x\n", ret, GPIO_GET_CLEAR);
      }

      // Set GIOA[0] to "1"
      invoke_to_set(0, GPIO_SET_BANK_A, 0, GPIO_SET_SET);

      // Check GIOA[0] "1" value
      invoke_to_get(0, GPIO_GET_BANK_A, 0, ret);
      if(GPIO_GET_SET != ret) {
          printf("!!!! Failed GIOA[0]=0x%x expected=0x%x\n", ret, GPIO_GET_SET);
      }

      // Check GIOA[1] "1" value
      invoke_to_get(0, GPIO_GET_BANK_A, 1, ret);
      if(GPIO_GET_SET != ret) {
          printf("!!!! Failed GIOA[1]=0x%x expected=0x%x\n", ret, GPIO_GET_SET);
      }

      // Set GIOA[0] to "0"
      invoke_to_set(0, GPIO_SET_BANK_A, 0, GPIO_SET_CLEAR);

      // Check GIOA[0] "0" value
      invoke_to_get(0, GPIO_GET_BANK_A, 0, ret);
      if(GPIO_GET_CLEAR != ret) {
          printf("!!!! Failed GIOA[0]=0x%x expected=0x%x\n", ret, GPIO_GET_CLEAR);
      }

      // Check GIOA[1] "0" value
      invoke_to_get(0, GPIO_GET_BANK_A, 1, ret);
      if(GPIO_GET_CLEAR != ret) {
          printf("!!!! Failed GIOA[1]=0x%x expected=0x%x\n", ret, GPIO_GET_CLEAR);
      }

      // Manual connect and disconnect for wait_handler() test
      printf("Connect GIOA[0] to GIOA[2](RTI clock) and press \"Enter\"\n");
      // Wait for manual operation completion
      getchar();

      // Check GIOA[2] "0" value
      invoke_to_get(0, GPIO_GET_BANK_A, 2, ret);
      if(GPIO_GET_CLEAR != ret) {
          printf("!!!! Failed GIOA[2]=0x%x expected=0x%x\n", ret, GPIO_GET_CLEAR);
      }

      // Check GIO_FLAG "0" value
      U32 gio_flg = gioREG->FLG;
      if(0 != (GIO_FLAG_2_MASK & gio_flg)) {
          printf("!!!! Failed GIO_FLAG bit 2 is set (0x%x)\n", gio_flg);
      }

      // Set GIOA[0] to "1"
      invoke_to_set(0, GPIO_SET_BANK_A, 0, GPIO_SET_SET);

      // Check GIOA[0] "1" value
      invoke_to_get(0, GPIO_GET_BANK_A, 0, ret);
      if(GPIO_GET_SET != ret) {
          printf("!!!! Failed GIOA[0]=0x%x expected=0x%x\n", ret, GPIO_GET_SET);
      }

      // Check GIOA[2] "1" value
      invoke_to_get(0, GPIO_GET_BANK_A, 2, ret);
      if(GPIO_GET_SET != ret) {
          printf("!!!! Failed GIOA[2]=0x%x expected=0x%x\n", ret, GPIO_GET_SET);
      }

      // Check GIO_FLAG[2] "1" value
      gio_flg = gioREG->FLG;
      if(0x4 != (GIO_FLAG_2_MASK & gio_flg)) {
          printf("!!!! Failed GIO_FLAG bit 2 is cleared (0x%x)\n", gio_flg);
      }

      bool alreadySet;

      invoke_to_wait(0, GPIO_WAIT_BANK_A, 2, alreadySet);
      if(true != alreadySet) {
          printf("!!!! Failed GIOA_FLAG[2] was not set as expected\n");
      }

      // Check GIO_FLAG[2] "0" value
      gio_flg = gioREG->FLG;
      if(0 != (GIO_FLAG_2_MASK & gio_flg)) {
          printf("!!!! Failed clearing GIO_FLAG bit 2 is set (0x%x)\n", gio_flg);
      }

      printf("Disconnect GIOA[0] from GIOA[2](RTI clock) and press \"Enter\"\n");
      // Wait for manual operation completion
      getchar();

      // Check GIOA[2] "0" value
      invoke_to_get(0, GPIO_GET_BANK_A, 2, ret);
      if(GPIO_GET_CLEAR != ret) {
          printf("!!!! Failed GIOA[2]=0x%x expected=0x%x\n", ret, GPIO_GET_CLEAR);
      }

      // Clear GIO_FLAG[2]
      gioREG->FLG = 0x4;

      // Check GIO_FLAG[2] "0" value
      gio_flg = gioREG->FLG;
      if(0 != (GIO_FLAG_2_MASK & gio_flg)) {
          printf("!!!! Failed clearing GIO_FLAG bit 2 is set (0x%x)\n", gio_flg);
      }

      printf("Connect GIOA[0] to GIOA[2](RTI clock) and press \"Enter\"\n");

      invoke_to_wait(0, GPIO_WAIT_BANK_A, 2, alreadySet);

      // Wait for manual operation completion
      getchar();

      if(false != alreadySet) {
          printf("!!!! Failed GIOA_FLAG[2] was unexpectedly set\n");
      }

      printf("Test completed\n");

      // The end
      for(;;);
  }

  // ----------------------------------------------------------------------
  // Helper methods
  // ----------------------------------------------------------------------

  void Tester ::
    connectPorts(void)
  {

    // set
    this->connect_to_set(
        0,
        this->component.get_set_InputPort(0)
    );

    // get
    this->connect_to_get(
        0,
        this->component.get_get_InputPort(0)
    );

    // wait
    this->connect_to_wait(
        0,
        this->component.get_wait_InputPort(0)
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

} // end namespace R5
