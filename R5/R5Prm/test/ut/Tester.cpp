// ====================================================================== 
// \title  R5Prm.hpp
// \author parallels
// \brief  cpp file for R5Prm test harness implementation class
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

namespace R5 {

  // ----------------------------------------------------------------------
  // Construction and destruction 
  // ----------------------------------------------------------------------

  Tester ::
    Tester(void) : 
#if FW_OBJECT_NAMES == 1
      R5PrmGTestBase("Tester", MAX_HISTORY_SIZE),
      component("R5Prm")
#else
      R5PrmGTestBase(MAX_HISTORY_SIZE),
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
      component.clear_parameter_memory();

      Fw::PolyType poly_var;
      R5::PrmEntry prm;
      bool valid = true;
      bool failed = false;

      printf("Checking cleared parameters\r\n");

      int i;
      for (i = 0; i < R5::PrmEntry::NUM_PRMS; i++) {

          prm.set((R5::PrmEntry::PrmId)i, poly_var);
          //this->invoke_to_get(0, prm, valid);
          component.get_handler(0, prm, valid);

          if (valid != false) {
              printf("!!!! Failed parameter %u should be invalid\r\n", i);
              failed = true;
          }
      }

      printf("Checking initialized parameters\r\n");

      for (i = 0; i < R5::PrmEntry::NUM_PRMS; i++) {

          poly_var = (U32)(i + 10);
          prm.set((R5::PrmEntry::PrmId)i, poly_var);
          this->invoke_to_set(0, prm, valid);

          if (valid == false) {
              printf("!!!! Failed to set parameter %u\r\n", i);
              failed = true;
          }
      }

      for (i = 0; i < R5::PrmEntry::NUM_PRMS; i++) {

          prm.set((R5::PrmEntry::PrmId)i, poly_var);
          this->invoke_to_get(0, prm, valid);

          if (valid == false) {
              printf("!!!! Failed to get parameter %u\r\n", i);
              failed = true;
          } else {
              U32 prm_val;
              prm.val().get(prm_val);

              if (prm_val != i + 10) {
                  printf("!!!! Incorrect value for parameter %u. Expected %u, but received %u\r\n", i, i + 10, prm_val);
                  failed = true;
              }
          }
      }

      printf("Checking overwriting parameters\r\n");

      poly_var = (U32)(0x789A1234);
      prm.set((R5::PrmEntry::PrmId)10, poly_var);

      this->invoke_to_set(0, prm, valid);

      if (valid == false) {
          printf("!!!! Failed to overwrite parameter\r\n");
          failed = true;
      } else {

          poly_var = (U32)(0);
          this->invoke_to_get(0, prm, valid);

          if (valid == false) {
              printf("!!!! Failed to get parameter\r\n");
              failed = true;
          } else {
              U32 prm_val;
              prm.val().get(prm_val);

              if (prm_val != 0x789A1234) {
                  printf("!!!! Incorrect value for parameter. Expected %x, but received %x\r\n", 0x789A1234, prm_val);
                  failed = true;
              }
          }
      }

      printf("Checking out-of-bounds parameters\r\n");

      prm.set((R5::PrmEntry::PrmId)R5::PrmEntry::NUM_PRMS, poly_var);

      this->invoke_to_set(0, prm, valid);

      if (valid == true) {
          printf("!!!! Out-of-bounds parameter successfully set\r\n");
          failed = true;
      }

      printf("Checking doubles parameters\r\n");

      double temp_double = 3.1415;

      poly_var = temp_double;

      prm.set((R5::PrmEntry::PrmId)0, poly_var);

      this->invoke_to_set(0, prm, valid);

      if (valid == false) {
          printf("!!!! Failed to set parameter\r\n");
          failed = true;
      } else {

          poly_var = (U32)(0);

          prm.set((R5::PrmEntry::PrmId)0, poly_var);

          this->invoke_to_get(0, prm, valid);

          if (valid == false) {
              printf("!!!! Failed to get parameter\r\n");
              failed = true;
          } else {
              double double_val;

              prm.val().get(double_val);

              if (double_val != temp_double) {
                  printf("!!!! Incorrect value for parameter. Expected %lf, but received %lf\r\n", temp_double, double_val);
                  failed = true;
              }
          }
      }
  }

  bool Tester ::
    writeFlash(void)
  {
      PrmWriteStatus status;

      this->invoke_to_write(0, true);

      int count = 8000;

      while (count) {
          this->invoke_to_check(0, status);
          if (status == R5::PRM_WRITE_IDLE) {
              return true;
          }

          count--;
      }

      return false;
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

    // write
    this->connect_to_write(
        0,
        this->component.get_write_InputPort(0)
    );

    // check
    this->connect_to_check(
        0,
        this->component.get_check_InputPort(0)
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
