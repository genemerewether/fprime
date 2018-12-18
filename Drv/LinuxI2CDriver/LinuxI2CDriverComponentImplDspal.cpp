// ======================================================================
// \title  LinuxI2CDriverImplDspal.cpp
// \author mereweth
// \brief  cpp file for LinuxI2CDriver DSPAL component implementation class
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

#include <Drv/LinuxI2CDriver/LinuxI2CDriverComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"
#include <Fw/Types/Assert.hpp>

#include <dev_fs_lib_i2c.h>
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

    void LinuxI2CDriverComponentImpl ::
      I2CConfig_handler(
          const NATIVE_INT_TYPE portNum,
          U32 busSpeed,
          U32 slaveAddr,
          U32 timeout
      )
    {
        struct dspal_i2c_ioctl_slave_config slaveConfig = {
            .bus_frequency_in_khz = busSpeed,
            .slave_address = slaveAddr,
            .byte_transer_timeout_in_usecs = timeout
        };

        // configure I2C clock rate
        if (ioctl(this->m_fd, I2C_IOCTL_SLAVE, (void *)&slaveConfig) != 0) {
            DEBUG_PRINT("ioctl I2C SLAVE fd %d failed. %d: %s",this->m_fd,errno,strerror(errno));
            this->log_WARNING_HI_I2C_ConfigError(this->m_device,errno);
            return;
        } else {
          DEBUG_PRINT("I2C fd %d freq %u, slave 0x%0x",this->m_fd, busSpeed, slaveAddr);
        }
    }

    void LinuxI2CDriverComponentImpl::I2CReadWrite_handler(
            const NATIVE_INT_TYPE portNum, Fw::Buffer &writeBuffer,
            Fw::Buffer &readBuffer) {

        if (this->m_fd == -1) {
            DEBUG_PRINT("Forgot to open I2C fd\n");
            return;
        }

        int result = 0;
        struct dspal_i2c_ioctl_combined_write_read read_write;

        read_write.read_buf = (U8*) readBuffer.getdata();
        read_write.read_buf_len = readBuffer.getsize();
        read_write.write_buf = (U8*) writeBuffer.getdata();
        read_write.write_buf_len = writeBuffer.getsize();

        U8 bakWriteBuf[1] = {0};
        U8 bakReadBuf[1] = {0};
        if (NULL == read_write.read_buf) {
            read_write.read_buf = bakReadBuf;
            read_write.read_buf_len = 1;
        }
        if (NULL == read_write.write_buf) {
            read_write.write_buf = bakWriteBuf;
            read_write.write_buf_len = 1;
        }
        
        unsigned int byte;
        unsigned char* read_data = (unsigned char*) readBuffer.getdata();
        for (byte = 0; byte < read_write.read_buf_len; byte++) {
            read_data[byte] = 0xA5;
        }

        // Finally, we can write:
        DEBUG_PRINT("Writing %d bytes to I2C",read_write.write_buf_len);

        result = ioctl(this->m_fd, I2C_IOCTL_RDWR, &read_write);
        if (result != read_write.read_buf_len) {
            DEBUG_PRINT("I2C %d read/write error %d vs %d actual! %d: %s",
                        this->m_fd, read_write.read_buf_len, result,
                        errno,strerror(errno));
            this->log_WARNING_HI_I2C_WriteError(this->m_device,errno);
            return;
        }

        this->m_bytes += writeBuffer.getsize();
        this->tlmWrite_I2C_Bytes(this->m_bytes);
    }

    void LinuxI2CDriverComponentImpl::open(NATIVE_INT_TYPE device,
                                           I2CFrequency clock) {
        this->m_device = device;
        NATIVE_INT_TYPE fd;

        // Open:
        char devName[256];
        snprintf(devName,sizeof(devName),"/dev/iic-%d",device);
        // null terminate
        devName[sizeof(devName)-1] = 0;
        DEBUG_PRINT("Opening I2C device %s\n",devName);

        fd = ::open(devName, O_RDWR);
        if (fd == -1) {
            DEBUG_PRINT("open I2C device %d failed. %d\n",device,errno);
            this->log_WARNING_HI_I2C_OpenError(device,fd);
            return;
        } else {
            DEBUG_PRINT("Successfully opened I2C device %s fd %d\n",devName,fd);
        }

        this->m_fd = fd;
    }

    LinuxI2CDriverComponentImpl::~LinuxI2CDriverComponentImpl(void) {
        DEBUG_PRINT("Closing I2C device %d\n",this->m_fd);
        (void) close(this->m_fd);
    }

} // end namespace Drv
