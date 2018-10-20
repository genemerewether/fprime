// ======================================================================
// \title  R5SpiMasterDriver.hpp
// \author iuchenik
// \brief  cpp file for R5SpiMasterDriver test harness implementation class
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

#include <R5/SpiMasterDrv/SpiMasterDrv.hpp>
#include <R5/R5Mem/R5DmaAllocator.hpp>
#include <R5/DmaDrv/DmaDrv.hpp>

#include <R5/TiHal/include/HL_spi.h>
#include <R5/TiHal/include/HL_sys_dma.h>

#define INSTANCE 0
#define MAX_HISTORY_SIZE 10

#define BUF_SIZE  64

#define DMA_CH2_MASK  0x4
#define DMA_CH3_MASK  0x8

namespace R5 {

  // ----------------------------------------------------------------------
  // Construction and destruction
  // ----------------------------------------------------------------------

  Tester ::
    Tester(void) :
#if FW_OBJECT_NAMES == 1
      R5SpiMasterDriverTesterBase("Tester", MAX_HISTORY_SIZE),
      component("R5SpiMasterDriver")
#else
      R5SpiMasterDriverTesterBase(MAX_HISTORY_SIZE),
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

  spiDAT1_t const dataconfig_t = {TRUE, FALSE, SPI_FMT_0, SPI_CS_0};

  unsigned tx_sizes[] = {1, 3, 5, 7, 11, 2, 10, 22, 54, 42, 41, 0};
  U16 rx_buf[BUF_SIZE];
  U16 tx_buf[BUF_SIZE];

  void Tester ::
    toDo(void)
  {
      R5DmaAllocator r5DmaAllocator;

      // Init SpiMasterDrv
      component.initDriver(0, 0, (BUF_SIZE * sizeof(U32)), 1, (BUF_SIZE * sizeof(U16)), r5DmaAllocator);

      // Init DmaDrv
      DmaDrvInit();

      // Enable SPI loopback
//       spiEnableLoopback(spiREG1, Analog_Lbk);

      Fw::Buffer rx_buffer(0, 0, (U64)rx_buf, (BUF_SIZE * sizeof(U16)));
      Fw::Buffer tx_buffer[4];
      for(unsigned i = 0; i < FW_NUM_ARRAY_ELEMENTS(tx_buffer); ++i) {
          tx_buffer[i].set(0, 1, (U64)tx_buf, (BUF_SIZE * sizeof(U16)));
      }

      U32 tx_config = ((dataconfig_t.CS_HOLD) ? 0x10000000U : 0U) |
                      ((dataconfig_t.WDEL) ? 0x04000000U : 0U) |
                      ((uint32_t)(dataconfig_t.DFSEL) << 24U) |
                      ((uint32_t)(dataconfig_t.CSNR) << 16U);

      if(tx_config != SPI_TX_CONFIG) {
          printf("!!!! Bad SPI_TX_CONFIG=0x%x tx_config=0x%x\n", SPI_TX_CONFIG, tx_config);
      }

      unsigned loop_cnt;
      for(loop_cnt = 1; ; ++loop_cnt) {
          unsigned tx_size = tx_sizes[loop_cnt - 1];
          if(0 == tx_size) {
              printf("Test completed on loop %u\n", loop_cnt);
              break;
          }

          // Clear RX data and load TX data
          for(unsigned i = 0; i < BUF_SIZE; i++) {
              rx_buf[i] = 0xd3d3;
              tx_buf[i] = (0x1010 * loop_cnt) + i;
          }

          // Send data
          rx_buffer.setsize(BUF_SIZE * sizeof(U16));
          unsigned num_bufs = (tx_size + (BUF_SIZE / FW_NUM_ARRAY_ELEMENTS(tx_buffer) - 1)) / (BUF_SIZE / FW_NUM_ARRAY_ELEMENTS(tx_buffer));
          for(unsigned i = 0, num_placed = 0; i < num_bufs; ++i) {
              unsigned buf_size = (((BUF_SIZE / FW_NUM_ARRAY_ELEMENTS(tx_buffer)) * (i + 1)) <= tx_size) ?
                                  (BUF_SIZE / FW_NUM_ARRAY_ELEMENTS(tx_buffer)) :
                                  (tx_size - ((BUF_SIZE / FW_NUM_ARRAY_ELEMENTS(tx_buffer)) * i));

              tx_buffer[i].setdata((U64)(tx_buf + num_placed));
              tx_buffer[i].setsize(buf_size * sizeof(U16));
              num_placed += buf_size;
          }
          invoke_to_spiSend(0, tx_buffer, num_bufs);

          // Wait for DMA to complete transmitting data
          U32 reg_btc;
          do {
              reg_btc = dmaREG->BTCFLAG;
          } while(0 == (DMA_CH3_MASK & reg_btc));

          // Receive data
          invoke_to_spiRecv(0, rx_buffer);
          U32 received = rx_buffer.getsize() / sizeof(U16);

          // Validate received data
          if(received == tx_size) {
              for(unsigned i = 0; i < received; i++) {
                  if(rx_buf[i] != tx_buf[i])
                  {
                      printf("!!!! Failed loop=%u rx_buf[%u]=0x%x expected=0x%x\n", loop_cnt, i, rx_buf[i], tx_buf[i]);
                  }
              }
          }
          else {
              printf("!!!! Failed loop=%u received=%u expected=%u\n", loop_cnt, received, tx_size);
          }

//           printf("Done loop %u %u bytes reg_btc=0x%x\n", loop_cnt, tx_size, reg_btc);
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

    // spiSend
    this->connect_to_spiSend(
        0,
        this->component.get_spiSend_InputPort(0)
    );

    // spiRecv
    this->connect_to_spiRecv(
        0,
        this->component.get_spiRecv_InputPort(0)
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
