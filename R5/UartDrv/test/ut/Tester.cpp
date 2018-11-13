// ======================================================================
// \title  R5UartDriver.hpp
// \author tcanham
// \brief  cpp file for R5UartDriver test harness implementation class
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

#include <R5/R5Mem/R5DmaAllocator.hpp>
#include <R5/DmaDrv/DmaDrv.hpp>
#include <R5/UartDrv/UartDrv.hpp>

#include <R5/TiHal/include/HL_sci.h>
#include <R5/TiHal/include/HL_sys_dma.h>
#include <R5/TiHal/include/HL_rti.h>

#include <string.h>

#define MAX_HISTORY_SIZE 10

#define BUF_SIZE  64

#define SCI_TX_EMPTY  0x800

namespace R5 {

  // ----------------------------------------------------------------------
  // Construction and destruction
  // ----------------------------------------------------------------------

  Tester::Tester(NATIVE_INT_TYPE instance) :
#if FW_OBJECT_NAMES == 1
    R5UartDriverTesterBase("Tester", MAX_HISTORY_SIZE),
      component("R5UartDriver")
#else
      R5UartDriverTesterBase(MAX_HISTORY_SIZE),
      component()
#endif
  {
      this->instance = instance;
      this->initComponents(instance);
      this->connectPorts();
  }

  Tester ::
    ~Tester(void)
  {

  }

  // ----------------------------------------------------------------------
  // Tests
  // ----------------------------------------------------------------------

    unsigned tx_sizes[] = {1, 63, 1, 2, 13, 1, 1, 1, 2, 3, 3, 31, 27, 0};
    U8 rx_buf[BUF_SIZE];
    U8 tx_buf[BUF_SIZE];

    void Tester::toDo(void)
    {
        R5DmaAllocator r5DmaAllocator;

        // Reset DMA
        dmaREG->GCTRL = 1;
        dmaDisable();
        sciInit();

        // Init UardDrv
        component.initDriver(0, BUF_SIZE, 1, BUF_SIZE, r5DmaAllocator);

        // Init DmaDrv
        DmaDrvInit();

        printf("UART unit test instance=%u\n", this->instance);

        sciBASE_t* sciREG = sciREG1;
        U32 mask = 0x20;
        U32 delay = 0;

        // Enable UART loopback
        switch(this->instance) {
            case 0:
                sciREG = sciREG1;
                mask = 0x20;
                break;

            case 1:
                sciREG = sciREG2;
                sciEnableLoopback(sciREG, Analog_Lbk);
                mask = 0x80;
                break;

            case 2:
                sciREG = sciREG3;
//                 sciEnableLoopback(sciREG, Digital_Lbk);
                mask = 0x200;
                delay = 10;
                break;

            default:
                for(;;);
        }

        Fw::Buffer rx_buffer(0, 0, (U64)rx_buf, BUF_SIZE);
        Fw::Buffer tx_buffer(0, 1, (U64)tx_buf, BUF_SIZE);

        // Attempt to receive data (no data expected)
        Drv::SerialReadStatus status;
        invoke_to_serialRecv(0, rx_buffer, status);
        U32 received = rx_buffer.getsize();
        if(0 != received) {
            printf("!!!! Failed received=%u when no data expected\n", received);
        }

        for(unsigned loop_cnt = 1; ; loop_cnt++) {
            unsigned tx_size = tx_sizes[loop_cnt - 1];
            if(0 == tx_size) {
                printf("Test completed on loop %u\n", loop_cnt);
                break;
            }

            // Clear RX data and load TX data
            for(unsigned i = 0; i < BUF_SIZE; i++) {
                rx_buf[i] = 0xd3;
                tx_buf[i] = (0x10 * loop_cnt) + i;
            }

            // Send data
            rx_buffer.setsize(BUF_SIZE);
            tx_buffer.setsize(tx_size);
            invoke_to_serialSend(0, tx_buffer);

            // Wait for DMA to complete receiving data
            U32 reg_btc;
            do {
                reg_btc = dmaREG->BTCFLAG;
            } while(0 == (mask & reg_btc));

            // Wait for UART to complete sending data
            U32 uart_fl;
            do {
                uart_fl = sciREG->FLR;
            } while(0 == (SCI_TX_EMPTY & uart_fl));

            // Delay
            U32 time = rtiGetMedResTimestamp() + delay;
            while(rtiGetMedResTimestamp() < time) {}

            // Receive data
            invoke_to_serialRecv(0, rx_buffer, status);
            received = rx_buffer.getsize();

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

            // Receive data
            rx_buffer.setsize(BUF_SIZE);
            invoke_to_serialRecv(0, rx_buffer, status);
            received = rx_buffer.getsize();

            // Validate received data
            if(0 != received) {
               printf("!!!! Failed second receive loop=%u received=%u expected=0\n", loop_cnt, received);
            }
            //           printf("Done loop %u %u bytes\n", loop_cnt, tx_size);
        }

        // The end
//         for(;;);
    }

    void Tester::r5FlowTester(void)
    {
        /*
         * This test will be used to test R5 communication
         * It will send 256 byte buffers with ramping patterns,
         * and verify the same.  921K baud, and 1 ms delay between
         * transmits
         */

        R5DmaAllocator r5DmaAllocator;

        const U32 BUF_SIZE_R = 256;
        U8 rx_buf_r[BUF_SIZE_R];
        U8 tx_buf_r[BUF_SIZE_R];

        double numPkts = 0.0;
        int numErrs = 0;
        BYTE expVal = 0;

        // Reset DMA
        dmaREG->GCTRL = 1;
        dmaDisable();
        sciInit();

        // Init UardDrv
        component.initDriver(0, BUF_SIZE_R, 1, BUF_SIZE_R, r5DmaAllocator);

        // Init DmaDrv
        DmaDrvInit();

        U32 delay = 1000; // delay of 1 ms
        U32 mask = 0x200;
        sciBASE_t* sciREG = sciREG3;

        // Enable UART loopback
        switch(this->instance) {
            case 0:
                sciREG = sciREG1;
                mask = 0x20;
                break;

            case 1:
                sciREG = sciREG2;
                mask = 0x80;
                break;

            case 2:
                sciREG = sciREG3;
                mask = 0x200;
                break;

            default:
                for(;;);
        }

        Fw::Buffer rx_buffer(0, 0, (U64)rx_buf_r, BUF_SIZE_R);
        Fw::Buffer tx_buffer(0, 1, (U64)tx_buf_r, BUF_SIZE_R);

        // Set up buffer data:
        memset(tx_buf_r,0,sizeof(tx_buf_r));

        for (U32 i = 0; i < BUF_SIZE_R; ++i) {
            tx_buf_r[i] = i;
        }

        // Send/read data every 1 ms:
        Drv::SerialReadStatus status;
        //for (U32 i = 0; i < 20*60*1000; ++i) {
        for (;;) {

            numPkts++;

            // Send data:
            rx_buffer.setsize(BUF_SIZE_R);
            tx_buffer.setsize(BUF_SIZE_R);
            invoke_to_serialSend(0, tx_buffer);

            // Wait for DMA to complete receiving data
            U32 reg_btc;
            do {
                reg_btc = dmaREG->BTCFLAG;
            } while(0 == (mask & reg_btc));

            // Wait for UART to complete sending data
            U32 uart_fl;
            do {
                uart_fl = sciREG->FLR;
            } while(0 == (SCI_TX_EMPTY & uart_fl));


            // Receive data:
            invoke_to_serialRecv(0, rx_buffer, status);

            // Verify ramping pattern:
            U32 error = 0;
            for (U32 byte = 0; byte < rx_buffer.getsize(); ++byte) {

                if (rx_buf_r[byte] != expVal) {

                    error = 1;
//                    printf("ERROR: exp: %d idx: %d got: %d size: %d\n",
//                            expVal,byte,rx_buf_r[byte],rx_buffer.getsize());
                    //break;
                }
                ++expVal;

            }

            if (error) {

                numErrs++;
//                for (NATIVE_UINT_TYPE byte = 0; byte < rx_buffer.getsize(); byte++) {
//                    printf("%d: 0x%02X ",byte,rx_buf_r[byte]);
//                }
//                printf("\nTOTAL NUMBER OF ERRORS: %d\n",numErrs);
//                printf("TOTAL ERROR PERCENTAGE: %f\n", (numErrs/numPkts)*100.0);
//
//                printf("\n");
//                printf("\n");

            }

            // Delay
            U32 time = rtiGetMedResTimestamp() + delay;
            while(rtiGetMedResTimestamp() < time) {}

        }

        printf("TOTAL NUMBER OF PACKETS: %d\n",numPkts);
        printf("TOTAL NUMBER OF ERRORS: %d\n",numErrs);
        printf("TOTAL ERROR PERCENTAGE: %f\n",(numErrs/numPkts)*100.0);

    }

  // ----------------------------------------------------------------------
  // Helper methods
  // ----------------------------------------------------------------------

  void Tester ::
    connectPorts(void)
  {

    // serialRecv
    this->connect_to_serialRecv(
        0,
        this->component.get_serialRecv_InputPort(0)
    );

    // serialSend
    this->connect_to_serialSend(
        0,
        this->component.get_serialSend_InputPort(0)
    );

  }

    void Tester::initComponents(NATIVE_INT_TYPE instance)
    {
        this->init();
        this->component.init(instance);
    }

} // end namespace R5
