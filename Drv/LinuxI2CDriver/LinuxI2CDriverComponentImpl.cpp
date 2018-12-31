// ======================================================================
// \title  LinuxI2CDriverImpl.cpp
// \author tcanham
// \brief  cpp file for LinuxI2CDriver component implementation class
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

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <errno.h>

//#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
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
        if (this->m_fd == -1) {
            return;
        }
        /*
         * Max speed in Hz
         */
        // TODO(mereweth)
        int ret = -1; //ioctl(this->m_fd, , &busSpeed);
        if (ret == -1) {
            DEBUG_PRINT("ioctl fd %d failed. %d\n",this->m_fd,errno);
            this->log_WARNING_HI_I2C_ConfigError(this->m_device,ret);
        } else {
            DEBUG_PRINT("I2C fd %d WR freq successfully configured to %d\n",this->m_fd,busSpeed);
        }
    }

    void LinuxI2CDriverComponentImpl::I2CReadWrite_handler(
            const NATIVE_INT_TYPE portNum, Fw::Buffer &writeBuffer,
            Fw::Buffer &readBuffer) {

        if (this->m_fd == -1) {
            return;
        }

        DEBUG_PRINT("Writing %d bytes to I2C\n",writeBuffer.getsize());

        // TODO(mereweth)
        NATIVE_INT_TYPE stat = -1;

        if (stat < 1) {
            this->log_WARNING_HI_I2C_WriteError(this->m_device,stat);
        }
        this->m_readBytes += readBuffer.getsize();
        this->m_writeBytes += writeBuffer.getsize();
        this->tlmWrite_I2C_ReadBytes(this->m_readBytes);
        this->tlmWrite_I2C_WriteBytes(this->m_writeBytes);

    }

    void LinuxI2CDriverComponentImpl::open(NATIVE_INT_TYPE device,
                                           I2CFrequency clock) {

        this->m_device = device;
        NATIVE_INT_TYPE fd;
        NATIVE_INT_TYPE ret;

        // Open:
        char devName[256];
        snprintf(devName,sizeof(devName),"/dev/i2c-%d",device);
        // null terminate
        devName[sizeof(devName)-1] = 0;
        DEBUG_PRINT("Opening I2C device %s\n",devName);

        fd = ::open(devName, O_RDWR);
        if (fd == -1) {
            DEBUG_PRINT("open I2C device %d.%d failed. %d\n",device,errno);
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
