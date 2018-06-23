// ======================================================================
// \title  LinuxSpiDriverImplDspal.cpp
// \author mereweth
// \brief  cpp file for LinuxSpiDriver DSPAL component implementation class
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

#include <Drv/LinuxSpiDriver/LinuxSpiDriverComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"
#include <Fw/Types/Assert.hpp>

#include <dev_fs_lib_spi.h>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>

#include <HAP_farf.h>
//#define DEBUG_PRINT(x,...) FARF(ALWAYS,x,##__VA_ARGS__);
#define DEBUG_PRINT(x,...)

namespace Drv {

    // ----------------------------------------------------------------------
    // Handler implementations for user-defined typed input ports
    // ----------------------------------------------------------------------

    void LinuxSpiDriverComponentImpl ::
      SpiConfig_handler(
          const NATIVE_INT_TYPE portNum,
          U32 busSpeed
      )
    {
        struct dspal_spi_ioctl_set_bus_frequency rate = {
            .bus_frequency_in_hz = busSpeed
        };

        // configure SPI clock rate
        if (ioctl(this->m_fd, SPI_IOCTL_SET_BUS_FREQUENCY_IN_HZ, (void *)&rate) != 0) {
            DEBUG_PRINT("ioctl SPI SET FREQ fd %d failed. %d: %s",this->m_fd,errno,strerror(errno));
            this->log_WARNING_HI_SPI_ConfigError(this->m_device,this->m_select,errno);
            return;
        } else {
            DEBUG_PRINT("SPI fd %d freq successfully configured to %d",this->m_fd,busSpeed);
        }
    }

    void LinuxSpiDriverComponentImpl::SpiReadWrite_handler(
            const NATIVE_INT_TYPE portNum, Fw::Buffer &writeBuffer,
            Fw::Buffer &readBuffer) {

        if (this->m_fd == -1) {
            DEBUG_PRINT("Forgot to open SPI fd\n");
            return;
        }

        int result = 0;
        struct dspal_spi_ioctl_read_write read_write;

        read_write.read_buffer = (void*) readBuffer.getdata();
        read_write.read_buffer_length = readBuffer.getsize();
        read_write.write_buffer = (void*) writeBuffer.getdata();
        read_write.write_buffer_length = writeBuffer.getsize();

        unsigned int byte;
        unsigned char* read_data = (unsigned char*) readBuffer.getdata();
        for (byte = 0; byte < read_write.read_buffer_length; byte++) {
            read_data[byte] = 0xA5;
        }
/*
        // We must update the slave address before/after writing to get the chip
        // select behavior desired:
        struct dspal_spi_ioctl_set_options options = {
            .slave_address = 0,
            .is_tx_data_synchronous = 0,
            .tx_data_callback = 0,
            .rx_data_callback = 0,
        };

        result = ioctl(this->m_fd, SPI_IOCTL_SET_OPTIONS, &options);

        if (result < 0) {
            DEBUG_PRINT("SPI %d slave set 1 error! %d: %s",this->m_fd,errno,strerror(errno));
            this->log_WARNING_HI_SPI_WriteError(this->m_device,this->m_select,errno);
            return;
        }

        // For some reason we need to re-assert this every write.  Otherwise, we
        // see the clk line go idle high after the first write.
        struct dspal_spi_ioctl_set_spi_mode mode = {
            .eClockPolarity = SPI_CLOCK_IDLE_LOW,
            .eShiftMode = SPI_INPUT_FIRST,
        };

        // configure SPI mode:
        if (ioctl(this->m_fd, SPI_IOCTL_SET_SPI_MODE, (void *)&mode) != 0) {
            DEBUG_PRINT("ioctl SPI SET MODE 1 fd %d failed. %d: %s",this->m_fd,errno,strerror(errno));
            this->log_WARNING_HI_SPI_WriteError(this->m_device,this->m_select,errno);
            return;
        }
*/
        // Finally, we can write:
        DEBUG_PRINT("Writing %d bytes to SPI",read_write.write_buffer_length);

        result = ioctl(this->m_fd, SPI_IOCTL_RDWR, &read_write);
        if (result != read_write.read_buffer_length) {
            DEBUG_PRINT("SPI %d read/write error %d vs %d actual! %d: %s",
                        this->m_fd, read_write.read_buffer_length, result,
                        errno,strerror(errno));
            this->log_WARNING_HI_SPI_WriteError(this->m_device,this->m_select,errno);
            return;
        }
/*
        // Once again to get the desired chip select behavior after writing:
        options.slave_address = 1;

        result = ioctl(this->m_fd, SPI_IOCTL_SET_OPTIONS, &options);

        if (result < 0) {
            DEBUG_PRINT("SPI %d slave set 2 error! %d: %s",this->m_fd,errno,strerror(errno));
            return;
        }

        // Once again re-asserting the SPI mode, so that the clk is idle low after writing:
        if (ioctl(this->m_fd, SPI_IOCTL_SET_SPI_MODE, (void *)&mode) != 0) {
            DEBUG_PRINT("ioctl SPI SET MODE 2 fd %d failed. %d: %s",this->m_fd,errno,strerror(errno));
            this->log_WARNING_HI_SPI_WriteError(this->m_device,this->m_select,errno);
            return;
        }
*/
        this->m_bytes += readBuffer.getsize();
        this->tlmWrite_SPI_Bytes(this->m_bytes);
    }

    void LinuxSpiDriverComponentImpl::open(NATIVE_INT_TYPE device,
                                           NATIVE_INT_TYPE select,
                                           SpiFrequency clock) {
        this->m_device = device;
        this->m_select = select;
        NATIVE_INT_TYPE fd;

        // TODO(mereweth) - chip select?

        // Open:
        char devName[256];
        snprintf(devName,sizeof(devName),"/dev/spi-%d",device);
        // null terminate
        devName[sizeof(devName)-1] = 0;
        DEBUG_PRINT("Opening SPI device %s\n",devName);

        fd = ::open(devName, O_RDWR);
        if (fd == -1) {
            DEBUG_PRINT("open SPI device %d.%d failed. %d\n",device,select,errno);
            this->log_WARNING_HI_SPI_OpenError(device,select,fd);
            return;
        } else {
            DEBUG_PRINT("Successfully opened SPI device %s fd %d\n",devName,fd);
        }

        this->m_fd = fd;
/*
        struct dspal_spi_ioctl_set_bus_frequency rate = {
            .bus_frequency_in_hz = clock
        };

        struct dspal_spi_ioctl_loopback loopback;

        // configure SPI clock rate
        if (ioctl(this->m_fd, SPI_IOCTL_SET_BUS_FREQUENCY_IN_HZ, (void *)&rate) != 0) {
            DEBUG_PRINT("ioctl SPI SET FREQ fd %d failed. %d: %s",this->m_fd,errno,strerror(errno));
            this->log_WARNING_HI_SPI_ConfigError(device,select,errno);
            return;
        } else {
            DEBUG_PRINT("SPI fd %d freq successfully configured to %d",this->m_fd,clock);
        }

        struct dspal_spi_ioctl_set_spi_mode mode = {
            .eClockPolarity = SPI_CLOCK_IDLE_LOW,
            .eShiftMode = SPI_INPUT_FIRST,
        };

        // configure SPI clock rate
        if (ioctl(this->m_fd, SPI_IOCTL_SET_SPI_MODE, (void *)&mode) != 0) {
            DEBUG_PRINT("ioctl SPI SET MODE fd %d failed. %d: %s",this->m_fd,errno,strerror(errno));
            this->log_WARNING_HI_SPI_ConfigError(device,select,errno);
            return;
        } else {
            DEBUG_PRINT("SPI fd %d mode successfully configured",this->m_fd);
        }

        loopback.state = SPI_LOOPBACK_STATE_DISABLED;

        int result = ioctl(this->m_fd, SPI_IOCTL_LOOPBACK_TEST, &loopback);

        if (result < 0) {
            DEBUG_PRINT("error: unable to activate spi %d loopback mode. %d: %s",this->m_fd,errno,strerror(errno));
            this->log_WARNING_HI_SPI_ConfigError(device,select,errno);
            return;
        } else {
            DEBUG_PRINT("%s SPI loopback mode",loopback.state == SPI_LOOPBACK_STATE_DISABLED?"Disabled":"Enabled");
        }

        // Updating the slave address to get the desired chip select behavior:
        struct dspal_spi_ioctl_set_options options = {
            .slave_address = 1,
            .is_tx_data_synchronous = 0,
            .tx_data_callback = 0,
            .rx_data_callback = 0,
        };

        result = ioctl(this->m_fd, SPI_IOCTL_SET_OPTIONS, &options);

        if (result < 0) {
            DEBUG_PRINT("SPI %d slave set error! %d: %s",this->m_fd,errno,strerror(errno));
            this->log_WARNING_HI_SPI_ConfigError(device,select,errno);
            return;
        }
*/
    }

    LinuxSpiDriverComponentImpl::~LinuxSpiDriverComponentImpl(void) {
        DEBUG_PRINT("Closing SPI device %d\n",this->m_fd);
        (void) close(this->m_fd);
    }

} // end namespace Drv
