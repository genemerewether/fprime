// ======================================================================
// \title  R5SpiSlaveDriver.hpp
// \author iuchenik
// \brief  cpp file for R5SpiSlaveDriver test harness implementation class
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

#include <R5/SpiSlaveDrv/SpiSlaveDrv.hpp>
#include <R5/R5Mem/R5DmaAllocator.hpp>
#include <R5/DmaDrv/DmaDrv.hpp>

#include <R5/TiHal/include/HL_sys_dma.h>
#include <R5/TiHal/include/HL_spi.h>

#define INSTANCE 0
#define MAX_HISTORY_SIZE 10

#define BUF_SIZE  256
#define SIZE  256

#define DMA_CH2_MASK  0x4

namespace R5 {

  // ----------------------------------------------------------------------
  // Construction and destruction
  // ----------------------------------------------------------------------

  Tester ::
    Tester(void) :
#if FW_OBJECT_NAMES == 1
      R5SpiSlaveDriverTesterBase("Tester", MAX_HISTORY_SIZE),
      component("R5SpiSlaveDriver"), spi_master_component("R5SpiMasterDriver")
#else
      R5SpiSlaveDriverTesterBase(MAX_HISTORY_SIZE),
      component(), spi_master_component()
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
  
  const U8 imu_packet[] = {0x47, 0x47, 0, 1, 2, 0xFF, 9, 9, 8, 8, 7, 7, 6, 5};

  unsigned tx_sizes[] = {2, 12, 2, 14, 12, 42, 108, 108, 36, 120, 120, 108, 132, 132, 108, 48, 0};
  const unsigned rx_sizes_exp[] = {0, 0, 14, 14, 0, 42, 112, 112, 28, 126, 126, 98, 140, 126, 112, 42};

  /*
   * 0: 0
   * 1: 0
   * 2: 1
   * 3: 1
   * 4: 0
   * 5: 3
   * 6: 8
   * 7: 8
   * 8: 2
   * 9: 9
   * 10: 9
   * 11: 7
   * 12: 10
   * 13: 9
   * 14: 8
   * 15: 3
   */

  U8 rx_buf[BUF_SIZE];
  U8 tx_buf[SIZE];

  void Tester ::
    toDo(void)
  {
      R5DmaAllocator r5DmaAllocator;

      // Reset DMA
      dmaREG->GCTRL = 1;
      dmaDisable();
      spiInit();

      // Init SpiSlaveDrv
      component.initDriver(0, 0, (BUF_SIZE * sizeof(U8)), r5DmaAllocator, 0x4747, sizeof(imu_packet));
      spi_master_component.initDriver(0, 0, (SIZE * sizeof(U32)), 1, (SIZE * sizeof(U8)), r5DmaAllocator);

      // Init DmaDrv
      DmaDrvInit();

      Fw::Buffer rx_buffer(0, 0, (U64)rx_buf, (SIZE * sizeof(U8)));
      Fw::Buffer tx_buffer(0, 1, (U64)tx_buf, (SIZE * sizeof(U8)));

      // Attempt to receive data (no data expected)
      invoke_to_spiRecv(0, rx_buffer);
      U32 received = rx_buffer.getsize() / sizeof(U8);
      if(0 != received) {
          printf("!!!! Failed received=%u when no data expected\n", received);
      }

      unsigned imu_idx = 0;
      unsigned loop_cnt;
      for(loop_cnt = 1; ; ++loop_cnt) {
          unsigned tx_size = tx_sizes[loop_cnt - 1];
          if(0 == tx_size) {
              printf("Test completed on loop %u\n", loop_cnt);
              break;
          }

          for(unsigned i = 0; i < SIZE; i++) {
              rx_buf[i] = 0xd3;
          }

          // Clear RX data and load TX data
          for(unsigned i = 0; i < tx_size; i++) {
              //tx_buf[i] = (0x1010 * loop_cnt) + i;

              if (imu_packet[imu_idx] == 0xFF) {
                  // Make the packet less regular
                  tx_buf[i] = (U8)loop_cnt;
              } else {
                tx_buf[i] = imu_packet[imu_idx];
              }

              imu_idx++;
              if (imu_idx >= (sizeof(imu_packet)/sizeof(imu_packet[0]))) {
                  imu_idx = 0;
              }
          }

          // Send data
          rx_buffer.setsize(SIZE * sizeof(U8));
          tx_buffer.setsize(tx_size * sizeof(U8));
//           invoke_to_spiSend(0, tx_buffer);
          m_to_spiSend[0].invoke(&tx_buffer, 1);

          // Wait for SPI Master DMA to complete receiving data
          U32 reg_btc;
          do {
              reg_btc = dmaREG->BTCFLAG;
          } while(0 == (DMA_CH2_MASK & reg_btc));

          // Receive data
          invoke_to_spiRecv(0, rx_buffer);
          received = rx_buffer.getsize() / sizeof(U8);

          // Validate received data

          if(received == rx_sizes_exp[loop_cnt - 1]) {

              for (unsigned i = 0; i < received; i++) {

                  unsigned tempImuIdx = i % (sizeof(imu_packet)/sizeof(imu_packet[0]));

                  //if (rx_buf[i] != imu_packet[tempImuIdx]) {
                      //printf("!!!! Failed loop=%u rx_buf[%u]=0x%x expected=0x%x\n",
                              //loop_cnt, i, rx_buf[i], imu_packet[tempImuIdx]);
                  //}
              }
          } 
          else {
              printf("!!!! Failed loop=%u received=%u expected=%u\n", loop_cnt, received, rx_sizes_exp[loop_cnt - 1]);
          }

          //if(received == tx_size) {
              //for(unsigned i = 0; i < received; i++) {
                  //if(rx_buf[i] != tx_buf[i])
                  //{
                      //printf("!!!! Failed loop=%u rx_buf[%u]=0x%x expected=0x%x\n", loop_cnt, i, rx_buf[i], tx_buf[i]);
                  //}
              //}
          //}

          //// Receive data
          //rx_buffer.setsize(BUF_SIZE * sizeof(U8));
          //invoke_to_spiRecv(0, rx_buffer);
          //received = rx_buffer.getsize() / sizeof(U8);

          //// Validate received data
          //if(0 != received) {
            //printf("!!!! Failed second receive loop=%u received=%u expected=0\n", loop_cnt, received);
          //}
          ////printf("Done loop %u %u reg_btc=0x%x\n", loop_cnt, tx_size, reg_btc);
      }

      // The end
      for(;;);
  }

  // ----------------------------------------------------------------------
  // Helper methods
  // ----------------------------------------------------------------------

  void Tester ::
    connectPorts(void)
  {

    // spiRecv
    this->connect_to_spiRecv(0, this->component.get_spiRecv_InputPort(0));

    // spiSend
//     this->connect_to_spiSend(0, this->component.get_spiSend_InputPort(0));
    this->m_to_spiSend[0].addCallPort(spi_master_component.get_spiSend_InputPort(0));

  }

  void Tester ::
    initComponents(void)
  {
    this->init();
    this->component.init(INSTANCE);
    this->spi_master_component.init(INSTANCE);
  }

} // end namespace R5
