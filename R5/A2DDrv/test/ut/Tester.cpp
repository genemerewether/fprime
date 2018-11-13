// ======================================================================
// \title  R5A2DDriver.hpp
// \author tcanham
// \brief  cpp file for R5A2DDriver test harness implementation class
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

#define adc1Ram ((U32 volatile*)0xFF3E0000U)

namespace R5 {

  // ----------------------------------------------------------------------
  // Construction and destruction
  // ----------------------------------------------------------------------

  Tester ::
    Tester(void) :
#if FW_OBJECT_NAMES == 1
      R5A2DDriverTesterBase("Tester", MAX_HISTORY_SIZE),
      component("R5A2DDriver")
#else
      R5A2DDriverTesterBase(MAX_HISTORY_SIZE),
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

  unsigned channels[] = {0, 21, 7, 10, 6};
  typedef struct {
      F32 low;
      F32 high;
  } Range;
  Range ranges[] = {{/* 0V */ 0.0f, 0.2f}, {/* 3.3V */ 2.9f, 3.1f}, {/* 5.0V */ 2.9f, 3.1f}, {/* 0V */ 0.0f, 0.2f}, {1.5f, 1.7f}};

  void Tester ::
    toDo(void)
  {
    // TODO
      F32 val;

      for(unsigned j = 0; j < 4; ++j) {
          for(unsigned i = 0;  i < 32; ++i) {
              U32 tmp = adc1Ram[i];
              if((0x1F & (tmp >> 12)) != i) {
                  printf("!!!! Failed channel memory location chid=%u expected=%u\n", (0x1F & (tmp >> 12)), i);
              }
          }

          for(unsigned i = 0;  i < (sizeof(channels) / sizeof(channels[0])); ++i) {
              U32 channel = channels[i];

              invoke_to_get(0, D2A_GET_BANK_A, channel, val);
              if((val < ranges[i].low) || (ranges[i].high < val)) {
                  printf("!!!! Failed channel=%u val=%f expected (%f, %f)\n", channel, val, ranges[i].low, ranges[i].high);
              }
// printf("val=%f channel=%u\n", val, channel);
//
// U32 tmp = adc1Ram[channel];
// printf("tmp=0x%x channel=%u chid=%u\n", tmp, channel, (0x1F & (tmp >> 12)));
          }
      }

      printf("Disconnect AD1[10] from the GND and press \"Enter\"\n");
      // Wait for manual operation completion
      getchar();
      invoke_to_get(0, D2A_GET_BANK_A, channels[3], val);
      if((val < 2.9f) || (3.1f < val)) {
          printf("!!!! Failed channel=%u val=%f expected (%f, %f)\n", channels[3], val, 2.9f, 3.1f);
      }

      printf("Connect AD1[10] to the GND and press \"Enter\"\n");
      // Wait for manual operation completion
      getchar();
      invoke_to_get(0, D2A_GET_BANK_A, channels[3], val);
      if((val < ranges[3].low) || (ranges[3].high < val)) {
          printf("!!!! Failed channel=%u val=%f expected (%f, %f)\n", channels[3], val, ranges[3].low, ranges[3].high);
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

    // get
    this->connect_to_get(
        0,
        this->component.get_get_InputPort(0)
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
