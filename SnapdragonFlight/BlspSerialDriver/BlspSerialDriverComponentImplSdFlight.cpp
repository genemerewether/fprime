// ====================================================================== 
// \title  BlspSerialDriverImpl.cpp
// \author tcanham
// \brief  cpp file for BlspSerialDriver component implementation class
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

#include <SnapdragonFlight/BlspSerialDriver/BlspSerialDriverComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"
#include "SnapdragonFlight/DspRelay/dsp_relay.h"
#include "SnapdragonFlight/DspRelay/uart_defs.h"
#include <Fw/Types/EightyCharString.hpp>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>

namespace SnapdragonFlight {

    // ----------------------------------------------------------------------
    // Construction, initialization, and destruction
    // ----------------------------------------------------------------------

    void BlspSerialDriverComponentImpl::open(NATIVE_UINT_TYPE device,
            UartBaudRate baud, UartFlowControl fc, bool block) {

        DspUartRates relayRate = UART_BITRATE_ILLEGAL_1;
        switch (baud) {
            // TODO add more as needed
            case BAUD_115K:
                relayRate = UART_BITRATE_115200;
                break;
            case BAUD_230K:
                relayRate = UART_BITRATE_230400;
                break;
            case BAUD_921K:
                relayRate = UART_BITRATE_921600;
                break;
            default:
                FW_ASSERT(0, baud);
                break;
        }

        this->m_device = device;

        // open device
        this->m_fd = dsp_relay_uart_relay_open(device);
        if (-1 == this->m_fd) {
            this->log_WARNING_HI_DR_OpenError(device, this->m_fd);
            return;
        }

        // configure device
        NATIVE_INT_TYPE stat = dsp_relay_uart_relay_configure(this->m_fd,
                this->m_device, (int) relayRate, 0, 0, 0, fc == NO_FLOW ? 0 : 1,
                block);
        if (-1 == stat) {
            this->log_WARNING_HI_DR_OpenError(device, stat);
            return;
        }

        this->log_ACTIVITY_HI_DR_PortOpened(device);

    }

    BlspSerialDriverComponentImpl::~BlspSerialDriverComponentImpl(void) {
        //dsp_relay_uart_relay_close(this->m_fd,this->m_device);
    }

    // ----------------------------------------------------------------------
    // Handler implementations for user-defined typed input ports
    // ----------------------------------------------------------------------

    void BlspSerialDriverComponentImpl::serialSend_handler(
            const NATIVE_INT_TYPE portNum, Fw::Buffer &serBuffer) {
        if (this->m_fd == -1) {
            return;
        }

        unsigned char* data =
                reinterpret_cast<unsigned char*>(serBuffer.getdata());
        NATIVE_INT_TYPE xferSize = serBuffer.getsize();

        for (NATIVE_INT_TYPE chunk = 0; chunk < xferSize; chunk +=
                WRITE_BUFF_SIZE) {

            NATIVE_INT_TYPE thisSize = FW_MIN(WRITE_BUFF_SIZE,
                    xferSize - chunk);
            NATIVE_INT_TYPE stat = dsp_relay_uart_relay_write(this->m_fd,
                    this->m_device, data + chunk, thisSize);
            if (-1 == stat) {
                this->log_WARNING_HI_DR_WriteError(this->m_device, stat);
                return;
            }
        }
    }

    void BlspSerialDriverComponentImpl::serialReadTaskEntry(void * ptr) {

        Drv::SerialReadStatus serReadStat; // added by m.chase 03.06.2017

        BlspSerialDriverComponentImpl* comp =
                static_cast<BlspSerialDriverComponentImpl*>(ptr);

        int stat = dsp_relay_uart_receive_allocate(comp->m_device,
                READ_BUFF_SIZE);
        if (-1 == stat) {
            comp->log_WARNING_HI_DR_ReadError(comp->m_device, stat);
            return;
        }

        Fw::Buffer buff;

        while (1) {
            // wait for data
            int sizeRead = 0;

            // find open buffer

            comp->m_readBuffMutex.lock();
            // search for open entry
            NATIVE_INT_TYPE entryFound = false;
            for (NATIVE_INT_TYPE entry = 0; entry < MAX_NUM_BUFFERS; entry++) {
                if (comp->m_buffSet[entry].available) {
                    comp->m_buffSet[entry].available = false;
                    buff = comp->m_buffSet[entry].readBuffer;
                    entryFound = true;
                    break;
                }
            }
            comp->m_readBuffMutex.unLock();

            if (not entryFound) {
                comp->log_WARNING_HI_DR_NoBuffers(comp->m_device);
                Fw::Buffer dummy;
                serReadStat = Drv::SER_NO_BUFFERS; // added by m.chase 03.06.2017
                //comp->serialRecv_out(0,buff,Drv::SER_NO_BUFFERS);
                comp->serialRecv_out(0, buff, serReadStat); // added by m.chase 03.06.2017
                // to avoid spinning, wait 50 ms
                Os::Task::delay(50);
                continue;
            }

//          timespec stime;
//          (void)clock_gettime(CLOCK_REALTIME,&stime);
//          printf("<<< Calling dsp_relay_uart_relay_read() at %d %d\n", stime.tv_sec, stime.tv_nsec);

            bool waiting = true;
            int stat = 0;

            while (waiting) {

                stat = dsp_relay_uart_relay_read(comp->m_device,
                        reinterpret_cast<unsigned char*>(buff.getdata()),
                        buff.getsize(), &sizeRead);

                // check for timeout
                if (1 == stat) {
                    if (comp->m_quitReadThread) {
                        return;
                    }
                } else { // quit if other error or data
                    waiting = false;
                }
            }

            if (comp->m_quitReadThread) {
                return;
            }

            if (stat != 0) {
                comp->log_WARNING_HI_DR_ReadError(comp->m_device, stat);
            }

            // check stat, maybe output event
            if (stat == -1) {
                serReadStat = Drv::SER_OTHER_ERR; // added by m.chase 03.06.2017
                //comp->serialRecv_out(0,buff,Drv::SER_OTHER_ERR);
            } else {
//              (void)clock_gettime(CLOCK_REALTIME,&stime);
//              printf("<!<! Sending data to RceAdapter %u at %d %d\n", buff.getsize(), stime.tv_sec, stime.tv_nsec);
                buff.setsize(sizeRead);
                serReadStat = Drv::SER_OK; // added by m.chase 03.06.2017
                //comp->serialRecv_out(0,buff,Drv::SER_OK);
            }
            comp->serialRecv_out(0, buff, serReadStat); // added by m.chase 03.06.2017
        }
    }

    void BlspSerialDriverComponentImpl::startReadThread(
            NATIVE_INT_TYPE priority, NATIVE_INT_TYPE stackSize,
            NATIVE_INT_TYPE cpuAffinity) {

        // Allocate read buffer
        Fw::EightyCharString task("SerReader");
        Os::Task::TaskStatus stat = this->m_readTask.start(task, 0, priority, stackSize,
                                                           serialReadTaskEntry, this, cpuAffinity);
        FW_ASSERT(stat == Os::Task::TASK_OK, stat);
    }

    void BlspSerialDriverComponentImpl::quitReadThread(void) {
        this->m_quitReadThread = true;
        dsp_relay_uart_relay_quit(this->m_device);
    }

} // end namespace Snapdragon
